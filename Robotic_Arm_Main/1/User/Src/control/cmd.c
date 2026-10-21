/**
 * @file    cmd.c
 * @layer   Control（命令/控制层，不依赖 HAL）
 * @brief   命令队列 + 执行：位姿/单关节/复位/急停/查询。
 * @deps    control/cmd.h, control/motion.h, config/robot_params.h, <math.h>
 * @callers proto.c（Cmd_Submit/ReportError）、app.c（Task/Home/StopNow/GetStatus）
 *
 * 调用关系：
 *   proto.c 解析帧 -> Cmd_Submit() 入队
 *   app.c 每轮 -> Cmd_Task() 取一条执行 -> 根据 CmdResult 回 0x81/0x82
 *   Cmd_Task() 内部 -> Kinematics_InWorkspace() / IK_Solve() / Motion_MoveTo()
 *   UI 通过 Cmd_GetStatus() 读取最近位姿、当前角、busy 和错误码。
 *
 * 注意：cmd 不直接调用 proto 的发送函数，避免 cmd <-> proto 循环依赖。
 */
#include "control/cmd.h"
#include "control/motion.h"
#include "config/robot_params.h"
#include <math.h>

static Command s_queue[CMD_QUEUE_SIZE];  /* 环形命令队列 */
static uint8_t s_head;                   /* 队头（最旧命令） */
static uint8_t s_tail;                   /* 队尾（下一个空位） */
static uint8_t s_count;                  /* 当前命令数 */
static Pose    s_pose;                   /* UI 显示的最近目标/查询位姿 */
static uint8_t s_err;                    /* 最近一次命令/协议错误码 */

/** @brief 关节角数组拷贝。 */
static void copy_joints(float dst[J_NUM], const float src[J_NUM])
{
    uint8_t i;

    for (i = 0; i < J_NUM; i++)
    {
        dst[i] = src[i];
    }
}

/**
 * @brief 按最大关节差自动计算运动时长。
 * @return 0（几乎无位移）或 MOTION_MIN_MS..MOTION_MAX_MS
 * @note  速度取 MOTION_MAX_SPEED_DPS(60°/s)，最短 300ms，最长 2s。
 */
static uint16_t calc_duration(const float cur[J_NUM], const float target[J_NUM])
{
    float max_delta;
    float ms;
    uint8_t i;

    max_delta = 0.0f;
    for (i = 0; i < J_NUM; i++)
    {
        float delta = fabsf(target[i] - cur[i]);

        if (delta > max_delta)
        {
            max_delta = delta;
        }
    }

    if (max_delta < 0.05f)
    {
        return 0;
    }

    ms = max_delta * 1000.0f / MOTION_MAX_SPEED_DPS;

    if (ms < (float)MOTION_MIN_MS)
    {
        ms = (float)MOTION_MIN_MS;
    }
    if (ms > (float)MOTION_MAX_MS)
    {
        ms = (float)MOTION_MAX_MS;
    }

    return (uint16_t)(ms + 0.5f);
}

/** @brief 从环形队列头部取一条命令。调用前需保证 s_count>0。 */
static void queue_pop(Command *c)
{
    *c = s_queue[s_head];
    s_head = (uint8_t)((s_head + 1) % CMD_QUEUE_SIZE);
    s_count--;
}

/**
 * @brief 初始化队列和 UI 位姿。
 * @note  必须在 Motion_Init() 之后调用，才能读到 home 当前角。
 */
void Cmd_Init(void)
{
    float cur[J_NUM];

    s_head  = 0;
    s_tail  = 0;
    s_count = 0;
    s_err   = CMD_ERR_NONE;

    Motion_GetCurrent(cur);
    FK_Solve(cur, &s_pose);
}

/** @brief 清空队列（不改变当前运动状态）。 */
void Cmd_Clear(void)
{
    s_head  = 0;
    s_tail  = 0;
    s_count = 0;
}

/**
 * @brief 手动复位（KEY3）。
 * @note  与串口 HOME 的差别：这里会清空队列并立即更新 UI 位姿。
 */
void Cmd_Home(void)
{
    float home[J_NUM];

    Joint_GetHome(home);
    Cmd_Clear();
    Motion_MoveTo(home, MOTION_HOME_MS);
    FK_Solve(home, &s_pose);
    s_err = CMD_ERR_NONE;
}

/**
 * @brief 手动急停（KEY4/退出 HOST/通信超时）。
 * @note  不改变 current，只停止插值；UI 位姿更新为当前位姿。
 */
void Cmd_StopNow(void)
{
    float cur[J_NUM];

    Cmd_Clear();
    Motion_Stop();
    Motion_GetCurrent(cur);
    FK_Solve(cur, &s_pose);
    s_err = CMD_ERR_NONE;
}

/** @brief 协议解析错误上报给 UI；下一条成功命令会自动清零。 */
void Cmd_ReportError(uint8_t err)
{
    s_err = err;
}

/**
 * @brief 命令入队。
 * @note  STOP 拥有最高优先级：先清空旧队列，再把自己放入队头。
 *        队列满时丢弃最旧命令（s_head 前移），符合文档约定。
 */
uint8_t Cmd_Submit(const Command *c)
{
    if (c == 0)
    {
        return 0;
    }

    if (c->type == CMD_STOP)
    {
        Cmd_Clear();
    }

    if (s_count >= CMD_QUEUE_SIZE)
    {
        s_head = (uint8_t)((s_head + 1) % CMD_QUEUE_SIZE);
        s_count--;
    }

    s_queue[s_tail] = *c;
    s_tail = (uint8_t)((s_tail + 1) % CMD_QUEUE_SIZE);
    s_count++;

    return 1;
}

/**
 * @brief 取一条命令并执行。
 * @return 1=执行了一条（r->valid=1），0=队列为空
 *
 * 执行链：
 *   MOVE      -> 包围盒粗判 -> IK -> Motion_MoveTo(自动时长)
 *   SET_JOINT -> 单关节赋值 -> Joint_Clamp -> Motion_MoveTo(自动时长)
 *   HOME      -> Motion_MoveTo(home, 800ms)
 *   STOP      -> Motion_Stop
 *   QUERY     -> 返回当前角度
 */
uint8_t Cmd_Task(CmdResult *r)
{
    Command c;
    float cur[J_NUM];
    float out[J_NUM];

    if (r == 0)
    {
        return 0;
    }

    if (s_count == 0)
    {
        return 0;
    }

    queue_pop(&c);

    Motion_GetCurrent(cur);
    copy_joints(out, cur);

    r->valid = 1;
    r->err   = CMD_ERR_NONE;

    switch (c.type)
    {
        case CMD_SET_POSE:
        {
            Pose target;
            float solved[J_NUM];

            target.x   = c.arg[0];
            target.y   = c.arg[1];
            target.z   = c.arg[2];
            target.phi = c.arg[3];

            /* 先用包围盒拒绝明显越界，再由 IK 做精确限位判断。 */
            if (!Kinematics_InWorkspace(&target))
            {
                r->err = CMD_ERR_WS;
                break;
            }

            if (!IK_Solve(&target, cur, solved))
            {
                r->err = CMD_ERR_WS;
                break;
            }

            Motion_MoveTo(solved, calc_duration(cur, solved));
            copy_joints(out, solved);
            s_pose = target;
            break;
        }

        case CMD_SET_JOINT:
        {
            uint8_t joint = (uint8_t)c.arg[0];

            if (joint >= J_NUM)
            {
                r->err = CMD_ERR_PARAM;
                break;
            }

            copy_joints(out, cur);
            out[joint] = c.arg[1];
            Joint_Clamp(out);   /* F10：越界值夹紧，不报错 */

            Motion_MoveTo(out, calc_duration(cur, out));
            FK_Solve(out, &s_pose);
            break;
        }

        case CMD_HOME:
        {
            float home[J_NUM];

            Joint_GetHome(home);
            Motion_MoveTo(home, MOTION_HOME_MS);
            copy_joints(out, home);
            FK_Solve(home, &s_pose);
            break;
        }

        case CMD_STOP:
        {
            Motion_Stop();
            FK_Solve(cur, &s_pose);
            break;
        }

        case CMD_QUERY:
        {
            FK_Solve(cur, &s_pose);
            break;
        }

        default:
        {
            r->err = CMD_ERR_UNKNOWN;
            break;
        }
    }

    /* 成功时 ACK 目标/当前角；失败时 ACK 不用，但仍填当前角便于调试。 */
    if (r->err == CMD_ERR_NONE)
    {
        copy_joints(r->theta, out);
        s_err = CMD_ERR_NONE;
    }
    else
    {
        copy_joints(r->theta, cur);
        s_err = r->err;
    }

    return 1;
}

/** @brief 供 UI 读取的快照：当前角来自 Motion，位姿/错误来自命令层。 */
void Cmd_GetStatus(CmdStatus *s)
{
    if (s == 0)
    {
        return;
    }

    Motion_GetCurrent(s->theta);
    s->pose   = s_pose;
    s->moving = Motion_IsBusy();
    s->err    = s_err;
}

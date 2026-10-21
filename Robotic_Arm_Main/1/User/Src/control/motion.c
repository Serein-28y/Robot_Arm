/**
 * @file    motion.c
 * @layer   Control（纯逻辑层，不依赖 HAL）
 * @brief   关节空间线性插值 + ease-in-out 缓动 + 急停。
 * @deps    control/motion.h, config/robot_params.h
 * @callers cmd.c（Motion_MoveTo/Stop）、app.c（20ms 周期 Motion_Task）
 *
 * 时间模型：
 *   不使用 HAL_GetTick，而是把总时长换算成 MOTION_STEP_MS(20ms) 的步数。
 *   每调用一次 Motion_Task() 前进一步，因此本模块可在主机侧单独验证。
 *
 * 数据流：
 *   cmd.c 给出 target -> Motion_MoveTo() 记录 start/current/target
 *   app.c 每 20ms 调 Motion_Task() 更新 current
 *   app.c 读取 Motion_GetCurrent() -> Joint_ThetaToUs() -> Servo_SetPulse()
 */
#include "control/motion.h"
#include "config/robot_params.h"

static float    s_current[J_NUM];   /* 当前插值角，实际输出值 */
static float    s_start[J_NUM];     /* 本次运动起点 */
static float    s_target[J_NUM];    /* 本次运动目标（已夹紧） */
static uint16_t s_steps_total;      /* 总步数 = ceil(dur/20ms) */
static uint16_t s_step;             /* 已走步数 */
static uint8_t  s_busy;             /* 1=正在运动 */

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
 * @brief 用 home 角初始化 current/start/target，busy=0。
 * @note  App_Init 在 Servo_Init 之前调用，保证 Motion_GetCurrent 与上电 PWM 一致。
 */
void Motion_Init(void)
{
    Joint_GetHome(s_current);
    copy_joints(s_start, s_current);
    copy_joints(s_target, s_current);
    s_steps_total = 0;
    s_step        = 0;
    s_busy        = 0;
}

/**
 * @brief 平滑移动到目标。
 * @note  dur_ms < 20ms 视为立即到位；否则总步数向上取整。
 *        若正在运动，start 取当前插值位置，实现无缝换目标。
 */
void Motion_MoveTo(const float target[J_NUM], uint16_t dur_ms)
{
    if (target == 0)
    {
        return;
    }

    copy_joints(s_start, s_current);
    copy_joints(s_target, target);
    Joint_Clamp(s_target);   /* 目标先过软限位 */

    s_step = 0;
    s_steps_total = 0;

    if (dur_ms < MOTION_STEP_MS)
    {
        copy_joints(s_current, s_target);
        s_busy = 0;
        return;
    }

    s_steps_total = (uint16_t)((dur_ms + MOTION_STEP_MS - 1) / MOTION_STEP_MS);
    s_busy = 1;
}

/** @brief 急停：目标设为当前，busy 清零，下一步不再改变角度。 */
void Motion_Stop(void)
{
    copy_joints(s_target, s_current);
    s_step        = 0;
    s_steps_total = 0;
    s_busy        = 0;
}

/**
 * @brief 20ms 插值任务。
 * @note  缓动曲线 s = 3p² - 2p³，p=step/total；
 *        最后一步直接对齐 target，避免浮点残留。
 */
void Motion_Task(void)
{
    float p;
    float s;
    uint8_t i;

    if (!s_busy)
    {
        return;
    }

    if (s_step < s_steps_total)
    {
        s_step++;
    }

    if (s_step >= s_steps_total)
    {
        copy_joints(s_current, s_target);
        s_busy = 0;
        return;
    }

    p = (float)s_step / (float)s_steps_total;
    s = 3.0f * p * p - 2.0f * p * p * p;

    for (i = 0; i < J_NUM; i++)
    {
        s_current[i] = s_start[i] + (s_target[i] - s_start[i]) * s;
    }
}

/** @brief 返回 busy 标志。 */
uint8_t Motion_IsBusy(void)
{
    return s_busy;
}

/** @brief 读取当前插值角；app.c 用它输出 PWM。 */
void Motion_GetCurrent(float out[J_NUM])
{
    if (out == 0)
    {
        return;
    }

    copy_joints(out, s_current);
}

/** @brief 读取当前目标角（调试/状态用）。 */
void Motion_GetTarget(float out[J_NUM])
{
    if (out == 0)
    {
        return;
    }

    copy_joints(out, s_target);
}

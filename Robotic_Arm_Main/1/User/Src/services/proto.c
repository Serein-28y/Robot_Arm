/**
 * @file    proto.c
 * @layer   Services（协议层）
 * @brief   二进制帧解析/组包：找 0xAA、校验、命令分发、ACK/ERR。
 * @deps    services/proto.h, services/ringbuf.h, drivers/uart_drv.h,
 *          control/cmd.h, config/robot_params.h
 * @callers app.c（HOST 模式下 Proto_Poll；应答函数由 app.c 调用）
 *
 * 帧格式：
 *   0xAA | LEN | CMD | PAYLOAD... | SUM
 *   LEN 为整帧字节数；SUM = 前 LEN-1 字节累加和低 8 位；多字节小端。
 *
 * 重同步策略：
 *   非 0xAA / LEN 非法 -> 丢 1 字节继续找；
 *   校验失败 -> 回 0x82/0x02，丢 1 字节继续找；
 *   已知命令但 LEN 不匹配 -> 回 0x82/0x03，丢掉整帧；
 *   未知命令 -> 回 0x82/0x04，丢掉整帧。
 *
 * 分层说明：
 *   proto 只负责“解析 + 入队 + 应答”，不直接控制运动；
 *   cmd 的结果由 app.c 通过 Proto_SendAck/Err() 回传，避免 cmd <-> proto 循环依赖。
 */
#include "services/proto.h"
#include "services/ringbuf.h"
#include "drivers/uart_drv.h"
#include "control/cmd.h"
#include "config/robot_params.h"

static uint32_t s_last_valid;   /* 最近有效帧 tick，供 app.c 看门狗使用 */

/** @brief 小端 int16 还原。 */
static int16_t read_le16(uint8_t lo, uint8_t hi)
{
    return (int16_t)((uint16_t)lo | ((uint16_t)hi << 8));
}

/** @brief 计算前 len-1 字节累加和。 */
static uint8_t frame_sum(const uint8_t *frame, uint8_t len)
{
    uint8_t i;
    uint8_t sum = 0;

    for (i = 0; i < (uint8_t)(len - 1); i++)
    {
        sum = (uint8_t)(sum + frame[i]);
    }

    return sum;
}

/** @brief 协议错误统一出口：同时更新 UI 错误并发送 0x82。 */
static void send_error(uint8_t code)
{
    Cmd_ReportError(code);
    Proto_SendErr(code);
}

/** @brief 清空协议内部时间戳。 */
void Proto_Init(void)
{
    s_last_valid = 0;
}

/** @brief 进入 HOST 模式时重置看门狗基准。 */
void Proto_ResetWatchdog(uint32_t now_ms)
{
    s_last_valid = now_ms;
}

/** @brief 最近有效帧 tick。 */
uint32_t Proto_GetLastValidTick(void)
{
    return s_last_valid;
}

/**
 * @brief 主循环解析入口。
 * @note  只要 ringbuf 里还有完整帧就继续处理；半帧则 break 等下次。
 */
void Proto_Poll(uint32_t now_ms)
{
    while (RingBuf_GetLength() >= 4)
    {
        uint8_t len;
        uint8_t frame[PROTO_MAX_LEN];
        uint8_t cmd;
        uint8_t i;
        Command command = {0};

        /* 找包头。 */
        if (RingBuf_Peek(0) != 0xAA)
        {
            RingBuf_Skip(1);
            continue;
        }
        //找到包头就看下一个，指向下一个
        len = RingBuf_Peek(1);
        //是否合法
        if (len < 4 || len > PROTO_MAX_LEN)
        {
            RingBuf_Skip(1);
            continue;
        }
        //只传了一部分帧，直接退，等有一帧再接收
        if (RingBuf_GetLength() < len)
        {
            break;   /* 半帧，等更多数据 */
        }

        /* 拷贝整帧，后续读字段更方便。 */
        for (i = 0; i < len; i++)
        {
            frame[i] = RingBuf_Peek(i);
        }
        //计算校验和
        if (frame_sum(frame, len) != frame[len - 1])
        {
            send_error(CMD_ERR_CHECKSUM);
            RingBuf_Skip(1);   /* 文档约定：校验失败丢 1 字节重找 */
            continue;
        }
        //校验正确后，获取命令码
        cmd = frame[2];
        //确定传来的是那种命令
        if (cmd == 0x01)   /* MOVE: x,y,z,phi(int16 LE, phi 0.1°) */
        {
            if (len != 12)
            {
                send_error(CMD_ERR_PARAM);
                RingBuf_Skip(len);
                continue;
            }
            //命令码正确后，开始存解析并存入姿态数据
            command.type = CMD_SET_POSE;
            command.n    = 4;
            command.arg[0] = (float)read_le16(frame[3], frame[4]);
            command.arg[1] = (float)read_le16(frame[5], frame[6]);
            command.arg[2] = (float)read_le16(frame[7], frame[8]);
            command.arg[3] = (float)read_le16(frame[9], frame[10]) / 10.0f;
        }
        else if (cmd == 0x02)   /* HOME */
        {
            if (len != 4)
            {
                send_error(CMD_ERR_PARAM);
                RingBuf_Skip(len);
                continue;
            }

            command.type = CMD_HOME;
        }
        else if (cmd == 0x03)   /* STOP */
        {
            if (len != 4)
            {
                send_error(CMD_ERR_PARAM);
                RingBuf_Skip(len);
                continue;
            }

            command.type = CMD_STOP;
        }
        else if (cmd == 0x04)   /* QUERY */
        {
            if (len != 4)
            {
                send_error(CMD_ERR_PARAM);
                RingBuf_Skip(len);
                continue;
            }

            command.type = CMD_QUERY;
        }
        else if (cmd == 0x05)   /* SET_JOINT: joint(u8) + deg(int16 0.1°) */
        {
            uint8_t joint;

            if (len != 7)
            {
                send_error(CMD_ERR_PARAM);
                RingBuf_Skip(len);
                continue;
            }

            joint = frame[3];
            if (joint >= J_NUM)
            {
                send_error(CMD_ERR_PARAM);
                RingBuf_Skip(len);
                continue;
            }

            command.type   = CMD_SET_JOINT;
            command.n      = 2;
            command.arg[0] = (float)joint;
            command.arg[1] = (float)read_le16(frame[4], frame[5]) / 10.0f;
        }
        else
        {
            send_error(CMD_ERR_UNKNOWN);
            RingBuf_Skip(len);
            continue;
        }

        /* 只有结构完整、校验通过、命令已知的帧才刷新看门狗。 */
        s_last_valid = now_ms;
        Cmd_Submit(&command);
        RingBuf_Skip(len);
    }
}

/**
 * @brief 组 0x81 ACK：四角度按 0.1° 转 int16 小端。
 * @note  帧长固定 12；J_NUM 必须为 4（当前硬件）。
 */
void Proto_SendAck(const float theta[J_NUM])
{
    uint8_t frame[12];
    uint8_t i;
    uint8_t sum;

    frame[0] = 0xAA;
    frame[1] = 12;
    frame[2] = 0x81;

    for (i = 0; i < J_NUM; i++)
    {
        float value = theta[i] * 10.0f;
        int16_t scaled = (int16_t)(value >= 0.0f ? value + 0.5f : value - 0.5f);
        uint16_t raw = (uint16_t)scaled;

        frame[3 + 2 * i] = (uint8_t)(raw & 0xFF);
        frame[4 + 2 * i] = (uint8_t)((raw >> 8) & 0xFF);
    }

    sum = 0;
    for (i = 0; i < 11; i++)
    {
        sum = (uint8_t)(sum + frame[i]);
    }
    frame[11] = sum;

    Uart_Send(frame, 12);
}

/** @brief 组 0x82 ERR：LEN=5，PAYLOAD 为错误码。 */
void Proto_SendErr(uint8_t code)
{
    uint8_t frame[5];
    uint8_t i;
    uint8_t sum = 0;

    frame[0] = 0xAA;
    frame[1] = 5;
    frame[2] = 0x82;
    frame[3] = code;

    for (i = 0; i < 4; i++)
    {
        sum = (uint8_t)(sum + frame[i]);
    }
    frame[4] = sum;

    Uart_Send(frame, 5);
}

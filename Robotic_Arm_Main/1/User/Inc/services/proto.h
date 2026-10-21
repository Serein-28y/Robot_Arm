/**
 * @file    proto.h
 * @layer   Services（协议层）
 * @brief   二进制帧：0xAA | LEN | CMD | PAYLOAD | SUM。
 * @deps    services/ringbuf.h, drivers/uart_drv.h, control/joint.h, control/cmd.h
 * @callers app.c（Proto_Poll / 应答）、proto.c 内部
 *
 * 帧方向：
 *   下行 MOVE(0x01) HOME(0x02) STOP(0x03) QUERY(0x04) SET_JOINT(0x05)
 *   上行 OK(0x81) + 四角度(0.1°)，ERR(0x82) + 错误码
 * 约定：
 *   LEN 为整帧字节数（含包头和校验）；多字节字段小端；
 *   SUM = 前 LEN-1 字节累加和的低 8 位。
 */
#ifndef __PROTO_H
#define __PROTO_H

#include <stdint.h>
#include "control/joint.h"

/** @brief 清空协议内部状态（有效帧时间戳）。 */
void     Proto_Init(void);

/**
 * @brief 从 ringbuf 中找帧、校验并提交命令。
 * @param now_ms 当前毫秒 tick，用于刷新通信看门狗时间戳
 * @note  只在 HOST 模式主循环调用；中断里不做解析。
 *        校验失败回 0x82/0x02 并丢 1 字节重同步；未知命令回 0x82/0x04。
 */
void     Proto_Poll(uint32_t now_ms);

/**
 * @brief 发送成功应答 0x81。
 * @param theta 四个关节角（°），内部换算成 0.1° 的 int16 小端
 */
void     Proto_SendAck(const float theta[J_NUM]);

/** @brief 发送错误应答 0x82 + 错误码。 */
void     Proto_SendErr(uint8_t code);

/** @brief 最近一次有效帧的 tick；app.c 用它判断通信超时。 */
uint32_t Proto_GetLastValidTick(void);

/** @brief 进入 HOST 模式时重置看门狗基准时间。 */
void     Proto_ResetWatchdog(uint32_t now_ms);

#endif

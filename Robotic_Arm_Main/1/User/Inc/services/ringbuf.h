/**
 * @file    ringbuf.h
 * @layer   Services（纯逻辑层）
 * @brief   单生产者/单消费者字节环形缓冲：中断写、主循环读。
 * @deps    <stdint.h>
 * @callers drivers/uart_drv.c（写）、services/proto.c / app.c（读）
 *
 * 并发模型：
 *   - 生产者：USART2 RxToIdle 中断回调，只调用 RingBuf_Write()。
 *   - 消费者：主循环 Proto_Poll()/菜单丢弃逻辑，只调用 Peek/Skip/GetLength。
 *   - 8 位读写下标在 Cortex-M3 上是原子访问，因此不需要关中断。
 *   - 本模块只搬字节，不解析协议；协议在 proto.c。
 */
#ifndef __RINGBUF_H
#define __RINGBUF_H

#include <stdint.h>

/** @brief 读写指针清零；上电时由 App_Init() 调用。 */
void    RingBuf_Init(void);

/**
 * @brief 写入一段字节。
 * @param data   源数据
 * @param length 字节数（<=127）
 * @return 实际写入字节数；空间不足返回 0，不覆盖旧数据
 * @note  仅允许中断上下文调用（生产者）。
 */
uint8_t RingBuf_Write(const uint8_t *data, uint8_t length);

/**
 * @brief 查看第 i 个待读字节，不移动读指针。
 * @param i 相对读指针的偏移，必须 < RingBuf_GetLength()
 */
uint8_t RingBuf_Peek(uint8_t i);

/** @brief 跳过 n 个字节（移动读指针）。 */
void    RingBuf_Skip(uint8_t length);

/** @brief 当前已用字节数。 */
uint8_t RingBuf_GetLength(void);

/** @brief 剩余可写空间。 */
uint8_t RingBuf_GetRemain(void);

#endif

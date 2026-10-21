/**
 * @file    uart_drv.h
 * @layer   Drivers（硬件驱动层，依赖 HAL）
 * @brief   USART2 115200 8N1：RxToIdle 中断接收 + 阻塞发送。
 * @deps    main.h（huart2）、services/ringbuf.h、config/robot_params.h
 * @callers app.c（Uart_Init）、services/proto.c（Uart_Send）
 *
 * 接收链路：
 *   USART2 IRQ -> HAL_UARTEx_RxEventCallback()
 *              -> RingBuf_Write()
 *              -> 重新挂 HAL_UARTEx_ReceiveToIdle_IT()
 * 解析不在中断内做，统一由主循环 Proto_Poll() 处理。
 */
#ifndef __UART_DRV_H
#define __UART_DRV_H

#include <stdint.h>

/** @brief 启动 RxToIdle 中断接收（必须在 MX_USART2_UART_Init 之后调用）。 */
void Uart_Init(void);

/**
 * @brief 阻塞发送一段数据。
 * @param data 数据指针
 * @param len  字节数
 * @note  只在主循环调用；超时 100ms。发送期间接收中断仍可写入 ringbuf。
 */
void Uart_Send(const uint8_t *data, uint16_t len);

#endif

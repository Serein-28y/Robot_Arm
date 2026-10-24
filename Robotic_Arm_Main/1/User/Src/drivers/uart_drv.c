/**
 * @file    uart_drv.c
 * @layer   Drivers（硬件驱动层，依赖 HAL）
 * @brief   USART2 RxToIdle 中断接收 -> ringbuf；阻塞发送。
 * @deps    drivers/uart_drv.h, services/ringbuf.h, config/robot_params.h, main.h
 * @callers app.c（Uart_Init）、services/proto.c（Uart_Send）
 *
 * 中断约束：
 *   RxEventCallback 只做 RingBuf_Write + 重新挂接收，不做协议解析、不碰浮点。
 *   主循环 Proto_Poll() 负责找帧/校验/执行。
 */
#include "drivers/uart_drv.h"
#include "services/ringbuf.h"
#include "config/robot_params.h"
#include "main.h"

extern UART_HandleTypeDef huart2;

static uint8_t s_rx_buf[UART_RX_CHUNK];   /* 中断接收临时缓冲 */

/** @brief 启动空闲中断接收。 */
void Uart_Init(void)
{
    HAL_UARTEx_ReceiveToIdle_IT(&huart2, s_rx_buf, sizeof(s_rx_buf));
}

/**
 * @brief 阻塞发送。
 * @note  只在主循环调用；100ms 超时防止异常时永久卡死。
 */
void Uart_Send(const uint8_t *data, uint16_t len)
{
    if (data == 0 || len == 0)
    {
        return;
    }

    HAL_UART_Transmit(&huart2, (uint8_t *)data, len, 100);
}

/**
 * @brief RxToIdle 事件回调（USART2 中断上下文）。
 * @note  收到一批数据/总线空闲时触发；写入 ringbuf 后立即重新挂接收。
 *        若 ringbuf 满，RingBuf_Write 返回 0，本批数据丢弃（文档 R7）。
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance != USART2)
    {
        return;
    }
    //将数据写入缓冲区
    if (Size > 0)
    {
        RingBuf_Write(s_rx_buf, (uint8_t)Size);
    }

    HAL_UARTEx_ReceiveToIdle_IT(huart, s_rx_buf, sizeof(s_rx_buf));
}

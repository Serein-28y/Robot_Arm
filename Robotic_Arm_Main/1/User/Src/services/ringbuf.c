/**
 * @file    ringbuf.c
 * @layer   Services（纯逻辑层）
 * @brief   128 字节 SPSC 环形缓冲实现。
 * @deps    services/ringbuf.h, <string.h>
 * @callers drivers/uart_drv.c（中断写）、services/proto.c / app.c（主循环读）
 *
 * 设计说明：
 *   - RINGBUF_SIZE=128，保留 1 字节用于区分“空”和“满”，有效容量 127。
 *   - readIndex 只由消费者写，writeIndex 只由生产者写；
 *     GetLength/GetRemain 可能读到“瞬时不一致”，但最坏只会提前/延后一次
 *     写入成功判断，不会破坏数据区。
 *   - 满时整批拒绝，不覆盖旧数据，避免协议帧被截断。
 */
#include "services/ringbuf.h"
#include <string.h>

#define RINGBUF_SIZE 128

static uint8_t buffer[RINGBUF_SIZE];

/* 读索引由主循环写，写索引由串口中断写；volatile 防止编译器优化掉访问。 */
static volatile uint8_t readIndex;
static volatile uint8_t writeIndex;

/** @brief 读写指针清零。 */
void RingBuf_Init(void)
{
    readIndex  = 0;
    writeIndex = 0;
}

/** @brief 已用字节数 = (write - read) mod SIZE。 */
uint8_t RingBuf_GetLength(void)
{
    return (uint8_t)((writeIndex + RINGBUF_SIZE - readIndex) % RINGBUF_SIZE);
}

/** @brief 剩余空间；故意少 1 字节，避免写满后读写索引相等被误判为空。 */
uint8_t RingBuf_GetRemain(void)
{
    return (uint8_t)((RINGBUF_SIZE - 1) - RingBuf_GetLength());
}

/**
 * @brief 写入一段字节。
 * @note  先检查剩余空间，再处理跨尾部环绕；返回 0 表示整批未写入。
 */
uint8_t RingBuf_Write(const uint8_t *data, uint8_t length)
{
    uint8_t firstLength;

    if (length == 0 || length > RingBuf_GetRemain())
    {
        return 0;
    }

    if (writeIndex + length < RINGBUF_SIZE)
    {
        memcpy(buffer + writeIndex, data, length);
        writeIndex = (uint8_t)((writeIndex + length) % RINGBUF_SIZE);
    }
    else
    {
        /* 跨过缓冲区尾部：先写尾段，再写头部剩余。 */
        firstLength = (uint8_t)(RINGBUF_SIZE - writeIndex);
        memcpy(buffer + writeIndex, data, firstLength);
        memcpy(buffer, data + firstLength, length - firstLength);
        writeIndex = (uint8_t)((length - firstLength) % RINGBUF_SIZE);
    }

    return length;
}

/** @brief 不移动读指针，查看第 i 个待读字节。 */
uint8_t RingBuf_Peek(uint8_t i)
{
    return buffer[(uint8_t)((readIndex + i) % RINGBUF_SIZE)];
}

/** @brief 移动读指针，跳过已处理字节。 */
void RingBuf_Skip(uint8_t length)
{
    readIndex = (uint8_t)((readIndex + length) % RINGBUF_SIZE);
}

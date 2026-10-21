/**
 * @file    sched.c
 * @layer   Services（软定时服务）
 * @brief   基于 HAL_GetTick 的周期任务判定。
 * @deps    services/sched.h, main.h
 * @callers app.c
 *
 * 说明：本项目只在 sched.c 中直接使用 HAL_GetTick；
 * app.c 通过 Sched_Now()/Sched_IsDue() 获得时间，避免应用层直接依赖 HAL。
 */
#include "services/sched.h"
#include "main.h"

static uint32_t s_last[SCHED_NUM];  /* 每个任务上次执行 tick */

/** @brief 记录各任务初始 tick。 */
void Sched_Init(void)
{
    uint8_t i;

    for (i = 0; i < SCHED_NUM; i++)
    {
        s_last[i] = HAL_GetTick();
    }
}

/** @brief 返回当前毫秒 tick。 */
uint32_t Sched_Now(void)
{
    return HAL_GetTick();
}

/**
 * @brief 判断任务是否到期。
 * @note  到期后把基准时间设为当前 now，避免主循环偶发变慢后连续补跑。
 *        now - last 使用无符号减法，tick 回绕安全。
 */
uint8_t Sched_IsDue(SchedId id, uint32_t period_ms)
{
    uint32_t now;

    if (id >= SCHED_NUM)
    {
        return 0;
    }

    now = HAL_GetTick();
    if ((uint32_t)(now - s_last[id]) >= period_ms)
    {
        s_last[id] = now;
        return 1;
    }

    return 0;
}

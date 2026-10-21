/**
 * @file    sched.h
 * @layer   Services（软定时服务）
 * @brief   基于 HAL_GetTick 的周期任务判断。
 * @deps    <stdint.h>；sched.c 依赖 main.h/HAL_GetTick
 * @callers app.c
 *
 * 这是项目中唯一保留 HAL_GetTick 调用的服务模块；
 * button/proto/cmd/motion 等逻辑模块都由 app.c 传入 now_ms，保持可移植。
 */
#ifndef __SCHED_H
#define __SCHED_H

#include <stdint.h>

typedef enum
{
    SCHED_BUTTON = 0,   /* 10ms 按键 */
    SCHED_MOTION,       /* 20ms 插值 */
    SCHED_UI,           /* 50ms OLED */
    SCHED_WDG,          /* 50ms 看门狗 */
    SCHED_NUM
} SchedId;

/** @brief 记录各任务初始 tick。 */
void     Sched_Init(void);

/** @brief 读取当前毫秒 tick（HAL_GetTick 的薄封装）。 */
uint32_t Sched_Now(void);

/**
 * @brief 判断任务是否到期。
 * @param id        任务编号
 * @param period_ms 周期
 * @return 1=到期（并更新该任务基准时间为 now），0=未到期
 * @note  使用无符号减法，tick 回绕安全。
 */
uint8_t  Sched_IsDue(SchedId id, uint32_t period_ms);

#endif

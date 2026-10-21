/**
 * @file    button.h
 * @layer   Services（纯逻辑层，不直接依赖 HAL）
 * @brief   按键去抖/短按/长按/连发事件。
 * @deps    drivers/key_drv.h, config/robot_params.h
 * @callers app.c（每 10ms 扫描）
 *
 * 时间来源：
 *   不调用 HAL_GetTick()，由 app.c 传入 now_ms，便于逻辑独立测试。
 *   状态机：原始电平变化 -> 去抖计时 -> stable 电平 -> PRESS/RELEASE；
 *   repeat_en=1 时，长按超过 HOLD_MS 后按 REPEAT_MS 连发。
 */
#ifndef __BUTTON_H
#define __BUTTON_H

#include <stdint.h>

typedef enum
{
    BTN_NONE = 0,   /* 无事件 */
    BTN_PRESS,      /* 稳定按下 */
    BTN_REPEAT,     /* 长按连发 */
    BTN_RELEASE     /* 稳定松开 */
} BtnEvent;

/** @brief 读取四路初始电平并建立去抖状态。 */
void     Button_Init(uint32_t now_ms);

/**
 * @brief 扫描一个按键。
 * @param id     0..3 = KEY1..KEY4
 * @param now_ms 当前毫秒 tick
 * @return 本周期事件
 * @note  必须每 BUTTON_SCAN_MS(10ms) 调用一次。
 */
BtnEvent Button_Scan(uint8_t id, uint32_t now_ms);

/**
 * @brief 判断按键是否已稳定按住至少 ms 毫秒。
 * @note  用于 KEY2 长按退出 HOST；与 BTN_REPEAT 独立。
 */
uint8_t  Button_Held(uint8_t id, uint32_t now_ms, uint32_t ms);

#endif

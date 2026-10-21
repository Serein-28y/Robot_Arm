/**
 * @file    button.c
 * @layer   Services（纯逻辑层，不直接依赖 HAL）
 * @brief   四键去抖/短按/长按/连发状态机。
 * @deps    services/button.h, drivers/key_drv.h, config/robot_params.h
 * @callers app.c（每 10ms 调用 Button_Scan）
 *
 * 时间由 app.c 传入 now_ms，本文件不调用 HAL_GetTick。
 * 扫描流程：
 *   原始电平变化 -> 记录变化时刻并等待去抖
 *   去抖时间到 -> stable 更新 -> 产生 PRESS/RELEASE
 *   持续按住且 repeat_en=1 -> 超过 HOLD_MS 后按 REPEAT_MS 连发
 */
#include "services/button.h"
#include "drivers/key_drv.h"
#include "config/robot_params.h"

/* 单个按键的去抖/长按状态。 */
typedef struct
{
    uint8_t  last_raw;    /* 上次原始电平 */
    uint8_t  stable;      /* 去抖后的稳定电平 */
    uint8_t  repeat_en;   /* 是否允许长按连发 */
    uint32_t t_change;    /* 原始电平变化时刻 */
    uint32_t t_down;      /* 稳定按下时刻 */
    uint32_t t_repeat;    /* 上次连发时刻 */
} ButtonState;

static ButtonState s_button[KEY_NUM];

/** @brief 读取四路初始电平并建立状态；同时调用 KeyDrv_Init()。 */
void Button_Init(uint32_t now_ms)
{
    uint8_t i;

    KeyDrv_Init();

    for (i = 0; i < KEY_NUM; i++)
    {
        uint8_t raw = KeyDrv_Read(i);

        s_button[i].last_raw  = raw;
        s_button[i].stable    = raw;
        s_button[i].repeat_en = 0;   /* 本应用不使用连发 */
        s_button[i].t_change  = now_ms;
        s_button[i].t_down    = now_ms;
        s_button[i].t_repeat  = now_ms;
    }
}

/**
 * @brief 扫描一个按键并返回事件。
 * @note  所有时间都用无符号减法，tick 回绕安全。
 */
BtnEvent Button_Scan(uint8_t id, uint32_t now_ms)
{
    ButtonState *b;
    uint8_t raw;

    if (id >= KEY_NUM)
    {
        return BTN_NONE;
    }

    b   = &s_button[id];
    raw = KeyDrv_Read(id);

    /* 原始电平变化：先记录，等去抖时间。 */
    if (raw != b->last_raw)
    {
        b->last_raw = raw;
        b->t_change = now_ms;
        return BTN_NONE;
    }

    if ((uint32_t)(now_ms - b->t_change) < BUTTON_DEBOUNCE_MS)
    {
        return BTN_NONE;
    }

    /* 去抖后电平变化：产生按下/松开事件。 */
    if (raw != b->stable)
    {
        b->stable = raw;

        if (raw)
        {
            b->t_down   = now_ms;
            b->t_repeat = now_ms;
            return BTN_PRESS;
        }

        return BTN_RELEASE;
    }

    /* 长按连发：本应用 repeat_en=0，保留给未来菜单/参数调节。 */
    if (raw && b->repeat_en &&
        (uint32_t)(now_ms - b->t_down) >= BUTTON_HOLD_MS &&
        (uint32_t)(now_ms - b->t_repeat) >= BUTTON_REPEAT_MS)
    {
        b->t_repeat = now_ms;
        return BTN_REPEAT;
    }

    return BTN_NONE;
}

/** @brief 判断是否已稳定按住至少 ms 毫秒（KEY2 长按退出使用）。 */
uint8_t Button_Held(uint8_t id, uint32_t now_ms, uint32_t ms)
{
    if (id >= KEY_NUM)
    {
        return 0;
    }

    if (!s_button[id].stable)
    {
        return 0;
    }

    return ((uint32_t)(now_ms - s_button[id].t_down) >= ms) ? 1 : 0;
}

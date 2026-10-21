/**
 * @file    key_drv.c
 * @layer   Drivers（硬件驱动层，依赖 HAL）
 * @brief   四路按键 GPIO 读取。
 * @deps    drivers/key_drv.h, main.h
 * @callers services/button.c
 *
 * 按键接法：PB12..PB15，内部上拉，低电平有效。
 * 本文件只返回原始 0/1；去抖和事件在 button.c。
 */
#include "drivers/key_drv.h"
#include "main.h"

/** @brief 预留初始化；GPIO 已由 CubeMX 的 MX_GPIO_Init() 配好。 */
void KeyDrv_Init(void)
{
}

/**
 * @brief 读取原始按键状态。
 * @return 1=按下（低电平），0=松开或非法 id
 */
uint8_t KeyDrv_Read(uint8_t id)
{
    GPIO_TypeDef *port;
    uint16_t pin;

    switch (id)
    {
        case KEY_ID_1:
            port = KEY1_GPIO_Port;
            pin  = KEY1_Pin;
            break;
        case KEY_ID_2:
            port = KEY2_GPIO_Port;
            pin  = KEY2_Pin;
            break;
        case KEY_ID_3:
            port = KEY3_GPIO_Port;
            pin  = KEY3_Pin;
            break;
        case KEY_ID_4:
            port = KEY4_GPIO_Port;
            pin  = KEY4_Pin;
            break;
        default:
            return 0;
    }

    return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET) ? 1 : 0;
}

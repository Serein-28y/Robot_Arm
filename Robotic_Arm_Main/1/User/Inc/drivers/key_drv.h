/**
 * @file    key_drv.h
 * @layer   Drivers（硬件驱动层，依赖 HAL）
 * @brief   四路按键 GPIO 读取（PB12..PB15，内部上拉，低有效）。
 * @deps    main.h（KEYx_Pin / GPIOB）
 * @callers services/button.c
 *
 * 本驱动只做“原始电平 -> 0/1”，去抖和事件状态机在 button.c。
 */
#ifndef __KEY_DRV_H
#define __KEY_DRV_H

#include <stdint.h>

#define KEY_NUM   (4)
#define KEY_ID_1  (0)   /* PB15 */
#define KEY_ID_2  (1)   /* PB14 */
#define KEY_ID_3  (2)   /* PB13 */
#define KEY_ID_4  (3)   /* PB12 */

/** @brief 预留初始化入口；GPIO 已由 CubeMX MX_GPIO_Init 配置。 */
void    KeyDrv_Init(void);

/**
 * @brief 读取原始按键状态。
 * @param id 0..3 = KEY1..KEY4
 * @return 1=按下（低电平），0=松开
 */
uint8_t KeyDrv_Read(uint8_t id);

#endif

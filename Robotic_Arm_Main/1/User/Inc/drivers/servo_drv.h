/**
 * @file    servo_drv.h
 * @layer   Drivers（硬件驱动层，依赖 HAL）
 * @brief   TIM1 CH1..CH4 四路 50Hz PWM 输出。
 * @deps    main.h（htim1）、config/robot_params.h
 * @callers app.c（Servo_Init / 20ms 输出）
 *
 * 通道语义：本驱动只认物理通道 0..3 = CH1..CH4；
 * 逻辑关节到物理通道的映射在 joint.c 的 joint_cfg[].servo_ch。
 * 硬件接线：CH4=底座，CH3=大臂，CH2=小臂，CH1=末端。
 */
#ifndef __SERVO_DRV_H
#define __SERVO_DRV_H

#include <stdint.h>

#define SERVO_NUM (4)

/**
 * @brief 先写入四路 home 脉宽，再启动 PWM。
 * @note  顺序很重要：先写 CCR 再 HAL_TIM_PWM_Start，上电首个脉冲即 home，
 *        避免从 0 脉宽起跳。
 */
void     Servo_Init(void);

/**
 * @brief 设置物理通道脉宽。
 * @param ch 0..3 = CH1..CH4
 * @param us 脉宽（us），内部按该通道 SERVO_CHx_PWM_MIN/MAX 再夹紧一次
 */
void     Servo_SetPulse(uint8_t ch, uint16_t us);

/** @brief 读取驱动内部记录的当前脉宽（us），用于避免重复写 CCR。 */
uint16_t Servo_GetPulse(uint8_t ch);

#endif

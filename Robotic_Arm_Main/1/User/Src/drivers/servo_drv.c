/**
 * @file    servo_drv.c
 * @layer   Drivers（硬件驱动层，依赖 HAL）
 * @brief   TIM1 CH1..CH4 四路 50Hz PWM 输出与脉宽限制。
 * @deps    drivers/servo_drv.h, config/robot_params.h, main.h
 * @callers app.c（Servo_Init / Servo_SetPulse）
 *
 * 硬件映射：
 *   ch0=CH1=PA8 -> 末端，ch1=CH2=PA9 -> 小臂
 *   ch2=CH3=PA10 -> 大臂，ch3=CH4=PA11 -> 底座
 * 本驱动只认物理通道；逻辑关节映射在 joint.c。
 */
#include "drivers/servo_drv.h"
#include "config/robot_params.h"
#include "main.h"

static const uint32_t s_tim_channel[SERVO_NUM] =
{
    TIM_CHANNEL_1,
    TIM_CHANNEL_2,
    TIM_CHANNEL_3,
    TIM_CHANNEL_4
};

/* 每通道的物理脉宽限位，驱动层最后一道保护。 */
static const uint16_t s_pwm_min[SERVO_NUM] =
{
    SERVO_CH1_PWM_MIN,
    SERVO_CH2_PWM_MIN,
    SERVO_CH3_PWM_MIN,
    SERVO_CH4_PWM_MIN
};

static const uint16_t s_pwm_max[SERVO_NUM] =
{
    SERVO_CH1_PWM_MAX,
    SERVO_CH2_PWM_MAX,
    SERVO_CH3_PWM_MAX,
    SERVO_CH4_PWM_MAX
};

/* 上电 home 脉宽，必须与 joint_cfg[].home_us 一致。 */
static const uint16_t s_home_us[SERVO_NUM] =
{
    SERVO_CH1_HOME_US,
    SERVO_CH2_HOME_US,
    SERVO_CH3_HOME_US,
    SERVO_CH4_HOME_US
};

static uint16_t s_pulse[SERVO_NUM];   /* 当前已写入的脉宽 */

/**
 * @brief 先写 CCR 再启动 PWM。------->原因是ccr的初始值是0，保证初始的脉冲以及home同步
 * @note  这样第一个输出脉冲就是 home，不会从 0us 起跳造成上电猛动。
 */
void Servo_Init(void)
{
    uint8_t i;

    for (i = 0; i < SERVO_NUM; i++)
    {
        s_pulse[i] = s_home_us[i];
        __HAL_TIM_SET_COMPARE(&htim1, s_tim_channel[i], s_pulse[i]);
    }

    for (i = 0; i < SERVO_NUM; i++)
    {
        HAL_TIM_PWM_Start(&htim1, s_tim_channel[i]);
    }
}

/**
 * @brief 设置物理通道脉宽。
 * @note  joint.c 已经按关节限位夹过一次；这里再按通道硬件限位夹一次。
 */
void Servo_SetPulse(uint8_t ch, uint16_t us)
{
    if (ch >= SERVO_NUM)
    {
        return;
    }

    if (us < s_pwm_min[ch])
    {
        us = s_pwm_min[ch];
    }
    if (us > s_pwm_max[ch])
    {
        us = s_pwm_max[ch];
    }

    s_pulse[ch] = us;
    __HAL_TIM_SET_COMPARE(&htim1, s_tim_channel[ch], us);
}

/** @brief 返回驱动内部记录的当前脉宽。 */
uint16_t Servo_GetPulse(uint8_t ch)
{
    if (ch >= SERVO_NUM)
    {
        return 0;
    }

    return s_pulse[ch];
}

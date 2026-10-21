/**
 * @file    joint.h
 * @layer   Control（纯逻辑层，不依赖 HAL）
 * @brief   关节模型：角度<->脉宽、软限位、home、逻辑关节到物理通道映射。
 * @deps    config/robot_params.h
 * @callers motion.c / kinematics.c / cmd.c / app.c
 *
 * 角度约定：
 *   θ1 底座：标准极角，+x=0°，逆时针为正；home 112.5°。
 *   θ2 大臂：相对水平面仰角，向上为正。
 *   θ3 小臂：相对大臂夹角，0°共线，向下弯为负。
 *   θ4 末端：相对小臂夹角，向小臂收为正。
 */
#ifndef __JOINT_H
#define __JOINT_H

#include <stdint.h>
#include "config/robot_params.h"

typedef enum
{
    J_BASE = 0,     /* 底座，物理 CH4 */
    J_UPPER,        /* 大臂，物理 CH3 */
    J_FOREARM,      /* 小臂，物理 CH2 */
    J_END,          /* 末端，物理 CH1 */
    J_NUM           /* 关节数量 = 4 */
} JointId;

/* 单个关节的标定参数；joint.c 中用 robot_params.h 初始化。 */
typedef struct
{
    const char *name;             /* 调试显示名 */
    uint16_t    pwm_min;          /* 逻辑脉宽下限（us） */
    uint16_t    pwm_max;          /* 逻辑脉宽上限（us） */
    float       us_per_deg;       /* 斜率（us/°） */
    float       theta_at_pwm_min; /* pwm_min 对应的角度（°） */
    int8_t      dir;              /* 脉宽增大时角度变化方向：+1/-1 */
    float       theta_min;        /* 软限位下限（°） */
    float       theta_max;        /* 软限位上限（°） */
    uint16_t    home_us;          /* 上电 home 脉宽（us） */
    uint8_t     servo_ch;         /* 物理通道：0=CH1 ... 3=CH4 */
} JointCfg;

extern const JointCfg joint_cfg[J_NUM];

/** @brief 预计算 home 角度，供 Motion_Init()/按键复位使用。 */
void     Joint_Init(void);

/**
 * @brief 脉宽 -> 角度。
 * @param j  关节编号
 * @param us 脉宽（us），内部先夹到该关节 pwm_min/pwm_max
 * @return 角度（°）；非法关节返回 0
 */
float    Joint_UsToTheta(JointId j, uint16_t us);

/**
 * @brief 角度 -> 脉宽。
 * @param j   关节编号
 * @param deg 角度（°），内部先夹到 theta_min/theta_max
 * @return 脉宽（us），再夹到 pwm_min/pwm_max；非法关节返回 0
 * @note  这是“软限位”的最后一道保障，cmd/motion 最终都经过这里。
 */
uint16_t Joint_ThetaToUs(JointId j, float deg);

/** @brief 原地把四个关节角夹到各自软限位内。 */
void     Joint_Clamp(float theta[J_NUM]);

/** @brief 读取预计算的 home 关节角（°）。 */
void     Joint_GetHome(float out[J_NUM]);

/** @brief 逻辑关节 -> 物理舵机通道 0..3（0=CH1）。 */
uint8_t  Joint_ServoChannel(JointId j);

#endif

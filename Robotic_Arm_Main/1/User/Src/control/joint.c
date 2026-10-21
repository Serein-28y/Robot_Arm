/**
 * @file    joint.c
 * @layer   Control（纯逻辑层，不依赖 HAL）
 * @brief   关节参数表与角度/脉宽换算、软限位、home、通道映射。
 * @deps    control/joint.h, config/robot_params.h
 * @callers motion.c（Motion_Init/Clamp）、cmd.c（Home/Clamp）、app.c（输出 PWM）
 *
 * 底座标定（本次修正）：
 *   θ1 = 202.5 - (us-500)*180/2000
 *   500us -> 202.5°，750us -> 180°(-x轴)，1500us -> 112.5°(home)，2500us -> 22.5°。
 * 其余关节仍按文档的线性标定，小臂/末端斜率为标称近似值。
 */
#include "control/joint.h"

/* 逻辑关节顺序固定为 BASE/UPPER/FOREARM/END；
 * servo_ch 是物理通道号，Servo_SetPulse() 只认这个号。 */
const JointCfg joint_cfg[J_NUM] =
{
    {
        "BASE",
        J_BASE_PWM_MIN, J_BASE_PWM_MAX,
        J_BASE_US_PER_DEG, J_BASE_THETA_AT_PWM_MIN, J_BASE_DIR,
        J_BASE_THETA_MIN, J_BASE_THETA_MAX,
        SERVO_CH4_HOME_US, SERVO_CH_BASE
    },
    {
        "UPPER",
        J_UPPER_PWM_MIN, J_UPPER_PWM_MAX,
        J_UPPER_US_PER_DEG, J_UPPER_THETA_AT_PWM_MIN, J_UPPER_DIR,
        J_UPPER_THETA_MIN, J_UPPER_THETA_MAX,
        SERVO_CH3_HOME_US, SERVO_CH_UPPER
    },
    {
        "FOREARM",
        J_FOREARM_PWM_MIN, J_FOREARM_PWM_MAX,
        J_FOREARM_US_PER_DEG, J_FOREARM_THETA_AT_PWM_MIN, J_FOREARM_DIR,
        J_FOREARM_THETA_MIN, J_FOREARM_THETA_MAX,
        SERVO_CH2_HOME_US, SERVO_CH_FOREARM
    },
    {
        "END",
        J_END_PWM_MIN, J_END_PWM_MAX,
        J_END_US_PER_DEG, J_END_THETA_AT_PWM_MIN, J_END_DIR,
        J_END_THETA_MIN, J_END_THETA_MAX,
        SERVO_CH1_HOME_US, SERVO_CH_END
    }
};

static float   s_home[J_NUM];   /* 预计算的 home 关节角，避免每次重复换算 */
static uint8_t s_ready;         /* 1=Joint_Init 已执行 */

/** @brief 通用浮点夹紧，避免引入 math.h。 */
static float clampf(float v, float lo, float hi)
{
    if (v < lo)
    {
        return lo;
    }
    if (v > hi)
    {
        return hi;
    }
    return v;
}

/**
 * @brief 预计算 home 角度。
 * @note  Motion_Init() 和按键复位都依赖 Joint_GetHome()，所以 App_Init 必须先调用本函数。
 */
void Joint_Init(void)
{
    uint8_t i;

    for (i = 0; i < J_NUM; i++)
    {
        s_home[i] = Joint_UsToTheta((JointId)i, joint_cfg[i].home_us);
    }
    s_ready = 1;
}

/**
 * @brief 脉宽 -> 角度。
 * @note  先把输入脉宽夹到该关节 pwm_min/pwm_max，再按线性公式换算。
 *        750us 在底座上会得到 180°，这是本次标定修正的关键点。
 */
float Joint_UsToTheta(JointId j, uint16_t us)
{
    const JointCfg *c;
    float u;

    if (j >= J_NUM)
    {
        return 0.0f;
    }

    c = &joint_cfg[j];
    u = (float)us;
    if (u < (float)c->pwm_min)
    {
        u = (float)c->pwm_min;
    }
    if (u > (float)c->pwm_max)
    {
        u = (float)c->pwm_max;
    }

    return c->theta_at_pwm_min +
           (float)c->dir * (u - (float)c->pwm_min) / c->us_per_deg;
}

/**
 * @brief 角度 -> 脉宽。
 * @note  先夹角度（软限位），再夹脉宽（硬件限位），是安全输出的最后一道。
 */
uint16_t Joint_ThetaToUs(JointId j, float deg)
{
    const JointCfg *c;
    float d;
    float u;

    if (j >= J_NUM)
    {
        return 0;
    }

    c = &joint_cfg[j];
    d = clampf(deg, c->theta_min, c->theta_max);
    u = (float)c->pwm_min +
        (float)c->dir * (d - c->theta_at_pwm_min) * c->us_per_deg;

    if (u < (float)c->pwm_min)
    {
        u = (float)c->pwm_min;
    }
    if (u > (float)c->pwm_max)
    {
        u = (float)c->pwm_max;
    }

    return (uint16_t)(u + 0.5f);
}

/** @brief 原地夹紧四个关节角；null 指针直接返回。 */
void Joint_Clamp(float theta[J_NUM])
{
    uint8_t i;

    if (theta == 0)
    {
        return;
    }

    for (i = 0; i < J_NUM; i++)
    {
        theta[i] = clampf(theta[i], joint_cfg[i].theta_min, joint_cfg[i].theta_max);
    }
}

/** @brief 读取预计算的 home 角；若未初始化则惰性初始化。 */
void Joint_GetHome(float out[J_NUM])
{
    uint8_t i;

    if (out == 0)
    {
        return;
    }

    if (!s_ready)
    {
        Joint_Init();
    }

    for (i = 0; i < J_NUM; i++)
    {
        out[i] = s_home[i];
    }
}

/** @brief 逻辑关节 -> 物理通道（0=CH1 ... 3=CH4）。 */
uint8_t Joint_ServoChannel(JointId j)
{
    if (j >= J_NUM)
    {
        return 0;
    }

    return joint_cfg[j].servo_ch;
}

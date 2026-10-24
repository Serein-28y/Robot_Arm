/**
 * @file    kinematics.c
 * @layer   Control（纯逻辑层，不依赖 HAL）
 * @brief   4 自由度 FK/IK 与工作空间包围盒粗判。
 * @deps    control/kinematics.h, config/robot_params.h, <math.h>
 * @callers cmd.c（IK_Solve/Kinematics_InWorkspace）、cmd.c/app.c（FK_Solve）
 *
 * 坐标与公式：
 *   原点在肩轴，z 向上；r = 水平投影半径，phi = 末端板绝对角。
 *   r = L2*cos(θ2) + L3*cos(θ2+θ3) + L4*cos(θ2+θ3+θ4)
 *   z = L2*sin(θ2) + L3*sin(θ2+θ3) + L4*sin(θ2+θ3+θ4)
 *   x = r*cos(θ1), y = r*sin(θ1)
 *
 * IK 亮点：
 *   - 底座 θ1=atan2(y,x) 归一化到 [0,2π)，与 joint_cfg 的 22.5°~202.5° 限位比较。
 *   - 先减掉末端 L4 得到“腕心”，再用余弦定理求两肘解 ±acos(D)。
 *   - 两支解都过滤关节限位，最后选与 current 角度差最小的一支，避免中途翻肘。
 */
#include "control/kinematics.h"
#include "config/robot_params.h"
#include <math.h>

#define ARM_PI (3.14159265358979323846f)

/** @brief 角度制转弧度制。 */
static float deg2rad(float d)
{
    return d * ARM_PI / 180.0f;
}

/** @brief 弧度制转角度制。 */
static float rad2deg(float r)
{
    return r * 180.0f / ARM_PI;
}

/**
 * @brief 判断四个关节角是否都在软限位内。
 * @note  允许 0.001° 浮点误差；IK 候选解过滤和最终安全检查共用。
 */
static uint8_t in_limits(const float theta[J_NUM])
{
    uint8_t i;

    for (i = 0; i < J_NUM; i++)
    {
        if (theta[i] < (joint_cfg[i].theta_min - 1e-3f))
        {
            return 0;
        }
        if (theta[i] > (joint_cfg[i].theta_max + 1e-3f))
        {
            return 0;
        }
    }

    return 1;
}

/**
 * @brief 工作空间包围盒粗判。
 * @note  包围盒由 22.5°~202.5° 底座范围和关节限位离线采样得到；
 *        盒内不代表一定可达，最终仍由 IK + in_limits 细判。
 */
uint8_t Kinematics_InWorkspace(const Pose *t)
{
    if (t == 0)
    {
        return 0;
    }

    if (t->x < WS_X_MIN || t->x > WS_X_MAX)
    {
        return 0;
    }
    if (t->y < WS_Y_MIN || t->y > WS_Y_MAX)
    {
        return 0;
    }
    if (t->z < WS_Z_MIN || t->z > WS_Z_MAX)
    {
        return 0;
    }
    if (t->phi < WS_PHI_MIN || t->phi > WS_PHI_MAX)
    {
        return 0;
    }

    return 1;
}

/**
 * @brief 正运动学：四个关节角 -> 末端位姿。
 * @note  输入 θ1 是标准极角（+x=0°，逆时针）；输出 phi 单位度。
 */
uint8_t FK_Solve(const float theta[J_NUM], Pose *out)
{
    float t1;
    float t2;
    float t3;
    float t4;
    float r;
    float z;

    if (theta == 0 || out == 0)
    {
        return 0;
    }

    t1 = deg2rad(theta[J_BASE]);
    t2 = deg2rad(theta[J_UPPER]);
    t3 = deg2rad(theta[J_FOREARM]);
    t4 = deg2rad(theta[J_END]);

    r = ARM_L2_MM * cosf(t2) +
        ARM_L3_MM * cosf(t2 + t3) +
        ARM_L4_MM * cosf(t2 + t3 + t4);

    z = ARM_L2_MM * sinf(t2) +
        ARM_L3_MM * sinf(t2 + t3) +
        ARM_L4_MM * sinf(t2 + t3 + t4);

    out->x   = r * cosf(t1);
    out->y   = r * sinf(t1);
    out->z   = z;
    out->phi = rad2deg(t2 + t3 + t4);

    return 1;
}

/**
 * @brief 逆运动学：目标位姿 -> 四个关节角。
 * @note  cur 用于“选最近解”；两肘解都超出限位时返回 0。
 */
uint8_t IK_Solve(const Pose *t, const float cur[J_NUM], float out[J_NUM])
{
    float phi;
    float t1;
    float r;
    float wr;
    float wz;
    float d;
    float a;
    float cand[2];
    uint8_t i;
    uint8_t j;
    uint8_t found;
    float best_dist;
    float best[J_NUM];

    if (t == 0 || cur == 0 || out == 0)
    {
        return 0;
    }

    /* ① 底座方位：atan2 归一到 [0,2π)，再由 joint_cfg 的 22.5°~202.5° 过滤。 */
    phi = deg2rad(t->phi);
    t1 = atan2f(t->y, t->x);
    if (t1 < 0.0f)
    {
        t1 += 2.0f * ARM_PI;
    }

    /* ② 把末端 L4 从目标点减掉，得到腕心 (wr,wz)。 */
    r  = sqrtf(t->x * t->x + t->y * t->y);
    wr = r - ARM_L4_MM * cosf(phi);
    wz = t->z - ARM_L4_MM * sinf(phi);

    /* ③ 余弦定理求 θ3；D 超出 [-1,1] 说明目标不可达。 */
    d = (wr * wr + wz * wz - ARM_L2_MM * ARM_L2_MM - ARM_L3_MM * ARM_L3_MM) /
        (2.0f * ARM_L2_MM * ARM_L3_MM);

    if (d > (1.0f + 1e-5f) || d < (-1.0f - 1e-5f))
    {
        return 0;
    }

    if (d > 1.0f)
    {
        d = 1.0f;
    }
    if (d < -1.0f)
    {
        d = -1.0f;
    }

    a = acosf(d);
    cand[0] = a;    /* 肘下一支 */
    cand[1] = -a;   /* 肘上一支 */

    found = 0;
    best_dist = 0.0f;

    /* ④ 对两支解分别求 θ2/θ4，过滤关节限位，选离 current 最近者。 */
    for (i = 0; i < 2; i++)
    {
        float t3;
        float t2;
        float t4;
        float theta[J_NUM];
        float dist;

        t3 = cand[i];
        t2 = atan2f(wz, wr) -
             atan2f(ARM_L3_MM * sinf(t3), ARM_L2_MM + ARM_L3_MM * cosf(t3));
        t4 = phi - t2 - t3;

        theta[J_BASE]    = rad2deg(t1);
        theta[J_UPPER]   = rad2deg(t2);
        theta[J_FOREARM] = rad2deg(t3);
        theta[J_END]     = rad2deg(t4);

        if (!in_limits(theta))
        {
            continue;
        }

        dist = 0.0f;
        for (j = 0; j < J_NUM; j++)
        {
            dist += fabsf(theta[j] - cur[j]);
        }

        if (!found || dist < best_dist)
        {
            found = 1;
            best_dist = dist;
            for (j = 0; j < J_NUM; j++)
            {
                best[j] = theta[j];
            }
        }
    }

    if (!found)
    {
        return 0;
    }
    //将最佳解复制到输出数组
    for (j = 0; j < J_NUM; j++)
    {
        out[j] = best[j];
    }

    return 1;
}

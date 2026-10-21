/**
 * @file    kinematics.h
 * @layer   Control（纯逻辑层，不依赖 HAL）
 * @brief   4 自由度正/逆运动学与工作空间粗判。
 * @deps    control/joint.h, config/robot_params.h
 * @callers cmd.c（IK/粗判）、app.c（状态初始化间接使用）
 *
 * 坐标系：原点在肩轴中心，z 轴竖直向上，x/y 为水平面。
 * FK：theta(deg) -> Pose(x,y,z mm, phi deg)。
 * IK：Pose -> theta(deg)，两肘解中过滤限位后选离当前姿态最近的一支。
 */
#ifndef __KINEMATICS_H
#define __KINEMATICS_H

#include <stdint.h>
#include "control/joint.h"

/* 末端位姿；位置单位 mm，phi 为末端板绝对角（度）。 */
typedef struct
{
    float x;
    float y;
    float z;
    float phi;
} Pose;

/**
 * @brief 正运动学：关节角 -> 末端位姿。
 * @param theta 四个关节角（°）
 * @param out   输出位姿；phi 单位度
 * @return 1=成功，0=参数为空
 */
uint8_t FK_Solve(const float theta[J_NUM], Pose *out);

/**
 * @brief 逆运动学：末端位姿 -> 关节角。
 * @param t   目标位姿（x/y/z mm，phi 度）
 * @param cur 当前关节角（°），用于在两肘解中选最近解
 * @param out 输出四个关节角（°）
 * @return 1=成功，0=不可达/超出关节限位
 * @note  调用前通常先用 Kinematics_InWorkspace() 做包围盒粗判。
 */
uint8_t IK_Solve(const Pose *t, const float cur[J_NUM], float out[J_NUM]);

/**
 * @brief 工作空间包围盒粗判。
 * @return 1=在盒内（仍可能被 IK/关节限位拒绝），0=明显越界
 */
uint8_t Kinematics_InWorkspace(const Pose *t);

#endif

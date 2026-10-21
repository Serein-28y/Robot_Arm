/**
 * @file    motion.h
 * @layer   Control（纯逻辑层，不依赖 HAL）
 * @brief   关节空间轨迹插值：线性 + ease-in-out、急停、busy 状态。
 * @deps    control/joint.h, config/robot_params.h
 * @callers cmd.c（下发目标）、app.c（20ms 周期调用 Motion_Task）
 *
 * 设计要点：
 *   - 只维护 current/start/target 三个关节角数组，不直接碰舵机。
 *   - 采用“步数”而不是绝对时间：每调用一次 Motion_Task 前进 20ms 一步，
 *     因此纯逻辑模块不依赖 HAL_GetTick，可在主机侧单独验证。
 *   - 任何新目标都从当前插值位置重新起步，所以天然支持中途换目标/急停。
 */
#ifndef __MOTION_H
#define __MOTION_H

#include <stdint.h>
#include "control/joint.h"

/** @brief 用 Joint_GetHome() 初始化 current/start/target，busy=0。 */
void    Motion_Init(void);

/**
 * @brief 平滑移动到目标关节角。
 * @param target 目标关节角（°），内部会 Joint_Clamp()
 * @param dur_ms 期望时长；< MOTION_STEP_MS 时立即到位
 * @note  若正在运动，会从当前插值位置重新起步，避免跳变。
 */
void    Motion_MoveTo(const float target[J_NUM], uint16_t dur_ms);

/** @brief 急停：target=current，busy=0，下一步不再改变角度。 */
void    Motion_Stop(void);

/** @brief 插值任务；每 MOTION_STEP_MS(20ms) 调用一次，内部使用 3t²-2t³ 缓动。 */
void    Motion_Task(void);

/** @brief 是否仍在运动：1=运动中。 */
uint8_t Motion_IsBusy(void);

/** @brief 读取当前插值关节角（°），用于 app.c -> Joint_ThetaToUs -> PWM。 */
void    Motion_GetCurrent(float out[J_NUM]);

/** @brief 读取当前目标关节角（°）。 */
void    Motion_GetTarget(float out[J_NUM]);

#endif

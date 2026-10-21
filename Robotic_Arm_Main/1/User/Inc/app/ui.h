/**
 * @file    ui.h
 * @layer   App（应用层，显示）
 * @brief   OLED 页面：菜单页 + HOST 状态页。
 * @deps    control/joint.h, control/kinematics.h（Pose/J_NUM）
 * @callers app.c
 *
 * 数据流：
 *   app.c 采集 CmdStatus/模式/急停/超时 -> UI_SetState()
 *   UI_Task() 只在数据变化时刷新对应 OLED 区域，避免刷屏阻塞主循环。
 */
#ifndef __UI_H
#define __UI_H

#include <stdint.h>
#include "control/joint.h"
#include "control/kinematics.h"

/* 一次 UI 刷新的完整快照，由 app.c 填充。 */
typedef struct
{
    uint8_t mode;          /* 0=菜单页，1=HOST 页 */
    uint8_t moving;        /* Motion_IsBusy() */
    uint8_t err;           /* CmdStatus.err，0=无错误 */
    uint8_t timeout;       /* 上位机通信超时标志 */
    uint8_t estop;         /* KEY4 急停标志 */
    Pose    pose;          /* 最近一次目标/查询位姿，phi 单位：度 */
    float   theta[J_NUM];  /* 当前四个关节角，单位：度 */
} UiState;

/** @brief 初始化 OLED 并绘制菜单页。 */
void UI_Init(void);

/**
 * @brief 提交一帧 UI 快照。
 * @param s 快照指针；内部只拷贝，不保存调用者指针。
 * @note  置脏标志，真正刷屏发生在下一次 UI_Task()。
 */
void UI_SetState(const UiState *s);

/**
 * @brief 刷新 OLED（50ms 周期调用）。
 * @note  页面切换时整屏刷新；同页只更新变化的数值区域。
 */
void UI_Task(void);

#endif

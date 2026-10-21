/**
 * @file    cmd.h
 * @layer   Control（命令/控制层，不依赖 HAL）
 * @brief   命令队列与执行：MOVE/HOME/STOP/QUERY/SET_JOINT。
 * @deps    control/joint.h, control/kinematics.h, control/motion.h
 * @callers proto.c（Cmd_Submit）、app.c（Cmd_Task/Cmd_GetStatus）
 *
 * 分层关系：
 *   proto.c 只负责“找帧/校验/入队”，不直接控制运动；
 *   cmd.c 负责把命令翻译成关节目标并调用 Motion；
 *   app.c 负责把 Cmd_Task 的结果通过 Proto_SendAck/Err() 回给上位机。
 *   这样 cmd 不反向依赖 proto，避免循环依赖。
 */
#ifndef __CMD_H
#define __CMD_H

#include <stdint.h>
#include "control/joint.h"
#include "control/kinematics.h"

/* 协议错误码，cmd 与 proto 共用，避免两边定义不一致。 */
#define CMD_ERR_NONE      (0x00)
#define CMD_ERR_WS        (0x01)   /* 目标超出工作空间 / IK 不可达 */
#define CMD_ERR_CHECKSUM  (0x02)   /* 帧校验失败 */
#define CMD_ERR_PARAM     (0x03)   /* 参数非法 */
#define CMD_ERR_UNKNOWN   (0x04)   /* 未知命令 */

typedef enum
{
    CMD_SET_POSE = 0,   /* 末端位姿 x,y,z,phi */
    CMD_SET_JOINT,      /* 单关节角度 */
    CMD_HOME,           /* 回 home */
    CMD_STOP,           /* 急停 */
    CMD_QUERY           /* 查询当前角度 */
} CmdType;

/* 一条待执行命令；arg 单位：位置 mm、角度 度。 */
typedef struct
{
    CmdType type;
    uint8_t n;
    float   arg[6];
} Command;

/* Cmd_Task 的执行结果，由 app.c 转成 0x81/0x82 应答。 */
typedef struct
{
    uint8_t valid;        /* 1=本次确实执行了一条命令 */
    uint8_t err;          /* CMD_ERR_*，0 表示成功 */
    float   theta[J_NUM]; /* 成功时为目标/当前角；失败时为当前角 */
} CmdResult;

/* UI 使用的只读状态快照。 */
typedef struct
{
    Pose    pose;         /* 最近一次目标/查询位姿 */
    float   theta[J_NUM]; /* 当前插值关节角 */
    uint8_t moving;       /* Motion_IsBusy() */
    uint8_t err;          /* 最近一次命令/协议错误 */
} CmdStatus;

/** @brief 清空队列、错误，并用当前 Motion 姿态初始化 UI 位姿。 */
void    Cmd_Init(void);

/** @brief 仅清空命令队列，不改运动状态。 */
void    Cmd_Clear(void);

/** @brief 手动复位：清队列、以 MOTION_HOME_MS 回 home，并更新 UI 位姿。 */
void    Cmd_Home(void);

/** @brief 手动急停：清队列、Motion_Stop，并更新 UI 位姿。 */
void    Cmd_StopNow(void);

/** @brief 记录协议/命令错误，供 UI 显示；成功命令会由 Cmd_Task 清零。 */
void    Cmd_ReportError(uint8_t err);

/**
 * @brief 入队一条命令。
 * @return 1=已入队
 * @note  STOP 会先清空队列；队列满时丢弃最旧命令（文档约定）。
 *        本函数只在主循环调用，不在中断中调用。
 */
uint8_t Cmd_Submit(const Command *c);

/**
 * @brief 取出并执行一条命令。
 * @param r 输出结果；仅当返回 1 时有效
 * @return 1=执行了一条命令，0=队列为空
 */
uint8_t Cmd_Task(CmdResult *r);

/** @brief 读取 UI 状态快照。 */
void    Cmd_GetStatus(CmdStatus *s);

#endif

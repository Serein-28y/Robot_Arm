/**
 * @file    app.h
 * @layer   App（应用层）
 * @brief   固件总入口：各模块初始化 + 前后台主循环调度。
 * @deps    无（实现文件 app.c 依赖全部下层模块）
 * @callers Core/Src/main.c
 */
#ifndef __APP_H
#define __APP_H

/**
 * @brief 初始化所有业务模块。
 * @note  调用顺序固定：Joint -> Motion -> Servo -> UART -> Sched ->
 *        Button -> Cmd -> Proto -> UI。main.c 在 MX_*_Init() 之后调用。
 */
void App_Init(void);

/**
 * @brief 主循环单次调度。
 * @note  非阻塞；内部通过 Sched_IsDue() 驱动 10/20/50ms 任务。
 *        串口解析、命令执行、插值、舵机输出、看门狗和 UI 都在这里汇合。
 */
void App_Run(void);

#endif

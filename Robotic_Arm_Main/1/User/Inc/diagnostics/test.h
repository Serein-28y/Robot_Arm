/**
 * @file    test.h
 * @layer   Diagnostics（手动标定/诊断页）
 * @brief   原有舵机 PWM 标定页入口；主固件不调用。
 * @deps    main.h, drivers/display/oled.h
 * @callers 仅手动调试时由调用者使用；main.c 不引用
 *
 * 说明：
 *   该页面用于逐通道加减脉宽、观察角度/PWM，不参与正式 App_Run 调度。
 *   本次底座标定修正不修改此页的角度显示逻辑。
 */
#ifndef __TEST_H
#define __TEST_H

/** @brief 进入标定页并死循环（仅手动调试）。 */
void Test(void);

#endif

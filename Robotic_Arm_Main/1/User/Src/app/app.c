/**
 * @file    app.c
 * @layer   App（应用层）
 * @brief   固件初始化、模式状态机、主循环任务调度。
 * @deps    app/app.h, config/robot_params.h,
 *          services/ringbuf.h, services/button.h, services/sched.h,
 *          services/proto.h, control/joint.h, control/motion.h,
 *          control/cmd.h, drivers/servo_drv.h, drivers/uart_drv.h,
 *          drivers/key_drv.h, app/ui.h
 * @callers Core/Src/main.c
 *
 * ============================ 调用链 ============================
 * main()
 *  ├─ App_Init()
 *  │   ├─ RingBuf_Init -> Proto_Init
 *  │   ├─ Joint_Init -> Motion_Init -> Servo_Init
 *  │   ├─ Uart_Init -> Sched_Init -> Button_Init
 *  │   └─ Cmd_Init -> UI_Init
 *  └─ App_Run()  (while(1)，非阻塞)
 *      ├─ HOST: Proto_Poll -> Cmd_Task -> Proto_SendAck/Err
 *      ├─ MENU: 丢弃串口数据 / 清空命令队列
 *      ├─ 10ms: Button_Scan -> 模式/复位/急停
 *      ├─ 20ms: Motion_Task -> Motion_GetCurrent -> Joint_ThetaToUs
 *      │        -> Servo_SetPulse
 *      ├─ 50ms: watchdog_task（运动中暂停，运动结束后 500ms 无帧急停）
 *      └─ 50ms: Cmd_GetStatus -> UI_SetState -> UI_Task
 *
 * 模式：
 *   MENU（上电默认）：只响应 KEY1/KEY2/KEY3/KEY4，串口数据丢弃。
 *   HOST：KEY2 进入，执行串口命令；KEY2 长按 1s 退出（退出时急停）。
 * =================================================================
 */
#include "app/app.h"
#include "config/robot_params.h"
#include "services/ringbuf.h"
#include "control/joint.h"
#include "control/motion.h"
#include "drivers/servo_drv.h"
#include "drivers/uart_drv.h"
#include "drivers/key_drv.h"
#include "services/button.h"
#include "services/sched.h"
#include "control/cmd.h"
#include "services/proto.h"
#include "app/ui.h"

#define APP_MODE_MENU (0)//菜单模式
#define APP_MODE_HOST (1)//主机模式

static uint8_t  s_mode;              /* 当前顶层模式 */
static uint8_t  s_key2_ignore_hold;  /* 刚进入 HOST 时屏蔽 KEY2 长按，等松开 */
static uint8_t  s_timeout_latched;   /* 通信超时已触发，避免重复急停 */
static uint8_t  s_estop;             /* KEY4 手动急停标志，供 UI 显示 */
static uint32_t s_last_activity;     /* 最近活动时刻：有效帧或运动中 */
static uint32_t s_prev_rx_tick;     /* 上一次有效帧 tick，用于检测新帧 */

/**
 * @brief 进入 HOST：清队列、清错误、重置看门狗和活动时间。
 * @note  KEY2 短按触发；进入后要求先松开 KEY2，避免同一次长按立即退出。
 */
static void enter_host(uint32_t now_ms)
{
    s_mode = APP_MODE_HOST;
    Cmd_Clear();
    Cmd_ReportError(CMD_ERR_NONE);
    Proto_ResetWatchdog(now_ms);
    s_prev_rx_tick  = Proto_GetLastValidTick();
    s_last_activity = now_ms;
    s_timeout_latched = 0;
    s_estop = 0;
}

/** @brief 退出 HOST：急停、清队列，回到菜单页。 */
static void exit_host(void)
{
    Cmd_StopNow();
    s_mode = APP_MODE_MENU;
    s_timeout_latched = 0;
    s_estop = 0;
}

/** @brief KEY3 手动复位：清队列、800ms 回 home，并更新 UI 位姿/错误。 */
static void do_home(void)
{
    Cmd_Home();
    s_timeout_latched = 0;
    s_estop = 0;
}

/** @brief KEY4 手动急停：清队列、停止插值、置 UI 急停标志。 */
static void do_estop(void)
{
    Cmd_StopNow();
    s_estop = 1;
}

/**
 * @brief 把当前插值角输出到四路舵机。
 * @note  只有脉宽变化才写 CCR，减少无意义的总线/定时器操作。
 *        逻辑关节 -> 物理通道的映射由 Joint_ServoChannel() 完成。
 */
static void apply_servo(void)
{
    float theta[J_NUM];
    uint8_t j;

    Motion_GetCurrent(theta);

    for (j = 0; j < J_NUM; j++)
    {
        uint8_t  ch = Joint_ServoChannel((JointId)j);
        uint16_t us = Joint_ThetaToUs((JointId)j, theta[j]);

        if (Servo_GetPulse(ch) != us)
        {
            Servo_SetPulse(ch, us);
        }
    }
}

/** @brief 组装 UI 快照：模式/运动/错误/超时/急停/位姿/当前角。 */
static void update_ui(void)
{
    CmdStatus status;
    UiState   state;
    uint8_t   j;

    Cmd_GetStatus(&status);

    state.mode    = s_mode;
    state.moving  = status.moving;
    state.err     = status.err;
    state.timeout = s_timeout_latched;
    state.estop   = s_estop;
    state.pose    = status.pose;

    for (j = 0; j < J_NUM; j++)
    {
        state.theta[j] = status.theta[j];
    }

    UI_SetState(&state);
}

/**
 * @brief 10ms 按键任务。
 *
 * 按键映射：
 *   KEY1：模式选择（当前只有 MODE1，保留）
 *   KEY2：短按进入 HOST；HOST 内长按 1s 退出
 *   KEY3：全局复位 home
 *   KEY4：全局急停
 */
static void button_task(uint32_t now_ms)
{
    BtnEvent event;

    /* KEY1 当前只有一个模式，只扫描状态，不做动作。 */
    (void)Button_Scan(KEY_ID_1, now_ms);
    //按键扫描，判断key2是否按下以及是点按还是长按
    event = Button_Scan(KEY_ID_2, now_ms);

    if (event == BTN_PRESS && s_mode == APP_MODE_MENU)
    {
        enter_host(now_ms);
        s_key2_ignore_hold = 1;
    }

    if (event == BTN_RELEASE)
    {
        s_key2_ignore_hold = 0;
    }
   
    if (s_mode == APP_MODE_HOST &&
        !s_key2_ignore_hold &&
        Button_Held(KEY_ID_2, now_ms, BUTTON_HOLD_MS))
    {
        exit_host();
    }
    
    //检测KEY3按下，执行复位
    event = Button_Scan(KEY_ID_3, now_ms);
    if (event == BTN_PRESS)
    {
        do_home();
    }
    //检测KEY4按下，执行急停
    event = Button_Scan(KEY_ID_4, now_ms);
    if (event == BTN_PRESS)
    {
        do_estop();
    }
}

/**
 * @brief 50ms 通信看门狗。
 *
 * 语义（已确认）：
 *   - 运动过程中不超时，让当前命令走完；
 *   - 运动结束后，若 500ms 内没有新的有效帧，则急停并锁定超时提示；
 *   - 一旦收到新有效帧，解除锁定并重新计时。
 */
static void watchdog_task(uint32_t now_ms)
{
    uint32_t last_valid;

    if (s_mode != APP_MODE_HOST)
    {
        return;
    }

    last_valid = Proto_GetLastValidTick();

    if (last_valid != s_prev_rx_tick)
    {
        s_prev_rx_tick  = last_valid;
        s_last_activity = now_ms;
        s_timeout_latched = 0;
    }

    if (Motion_IsBusy())
    {
        s_last_activity = now_ms;   /* 运动中持续刷新活动时间 */
    }

    if (!s_timeout_latched &&
        (uint32_t)(now_ms - s_last_activity) > UART_TIMEOUT_MS)
    {
        Cmd_StopNow();
        s_timeout_latched = 1;
    }
}

/**
 * @brief 全系统初始化。
 * @note  顺序不能随意调换：
 *        1) Joint_Init 先算 home；
 *        2) Motion_Init 用 home 初始化当前角；
 *        3) Servo_Init 写 home 并启动 PWM；
 *        4) Uart/Proto 建立接收链路；
 *        5) Cmd_Init 用当前角初始化 UI 位姿；
 *        6) UI_Init 最后绘制菜单。
 */
void App_Init(void)
{
    uint32_t now_ms;

    RingBuf_Init();//读写指针清零readIndex  = 0; writeIndex = 0;
    Proto_Init();//清空协议内部时间戳
    Joint_Init();//根据设置的初始脉冲计算出相对应的初始角度，同时给Motion_GetCurrent()提供初始角度
    Motion_Init();//读取Joint_GetHome()的角度初始化当前角度、起始角度、目标角度，并将busy标志清零
    Servo_Init();//输出初始的脉冲到舵机，启动PWM
    Uart_Init();//开启串口中断接收以及中断后进去回调函数
    Sched_Init();//规定好初始的tick时间
    Button_Init(Sched_Now());//按键初始化，读取四路按键的初始电平并建立状态
    Cmd_Init();//读取关节角算初始末位姿态
    UI_Init();//OLED初始化，清空状态并绘制菜单页

    s_mode             = APP_MODE_MENU;
    s_key2_ignore_hold = 0;
    s_timeout_latched  = 0;
    s_estop            = 0;

    now_ms = Sched_Now();//获取当前tick时间
    s_last_activity = now_ms;//记录最近活动时刻：有效帧或运动中
    s_prev_rx_tick  = Proto_GetLastValidTick();//获取上一次有效帧 tick，用于检测新帧
}

/**
 * @brief 主循环单次调度。
 * @note  所有任务都不阻塞；OLED 整屏刷新只在页面切换时发生。
 */
void App_Run(void)
{
    uint32_t now_ms;
    CmdResult result;

    now_ms = Sched_Now();

    if (s_mode == APP_MODE_HOST)
    {
        /* HOST：解析串口帧 -> 执行一条命令 -> 立即回 ACK/ERR。 */
        Proto_Poll(now_ms);
        //接收指令，进入某个模式，解算运动，设置舵机线性插值运动
        if (Cmd_Task(&result))
        {
            if (result.err == CMD_ERR_NONE)
            {
                Proto_SendAck(result.theta);
                s_estop = 0;
            }
            else
            {
                Proto_SendErr(result.err);
            }
        }
    }
    else
    {
        /* MENU：丢弃串口数据，避免进入 HOST 时执行陈旧命令。 */
        RingBuf_Skip(RingBuf_GetLength());
        Cmd_Clear();
    }
    //轮询是都到了10/20/50ms，到了就执行对应的任务

    //按键轮询
    if (Sched_IsDue(SCHED_BUTTON, BUTTON_SCAN_MS))
    {
        button_task(Sched_Now());
    }

    //运动轮询(底层驱动层)
    if (Sched_IsDue(SCHED_MOTION, MOTION_STEP_MS))
    {
        Motion_Task();
        apply_servo();
    }

    //看门狗轮询
    if (Sched_IsDue(SCHED_WDG, WDG_PERIOD_MS))
    {
        watchdog_task(Sched_Now());
    }

    //UI轮询（OLED等）
    if (Sched_IsDue(SCHED_UI, UI_PERIOD_MS))
    {
        update_ui();
        UI_Task();
    }
}

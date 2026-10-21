/**
 * @file    robot_params.h
 * @layer   Config（配置层，纯宏定义）
 * @brief   全项目唯一参数入口：连杆、关节限位、脉宽映射、home、工作空间、
 *          通信/运动/按键/UI 节拍。
 * @deps    无
 * @callers joint.c / kinematics.c / motion.c / cmd.c / servo_drv.c /
 *          uart_drv.c / button.c / proto.c / app.c
 *
 * 修改原则：
 *   1. 业务代码不允许出现魔法数字，所有标定量集中在这里。
 *   2. 关节角度统一用“度”，位置统一用“mm”，脉宽统一用“us”。
 *   3. 底座标定见 J_BASE_*：θ1 = 202.5 - (us-500)*180/2000。
 *      500us -> 202.5°，750us -> 180°（-x 轴），
 *      1500us -> 112.5°（home），2500us -> 22.5°。
 */
#ifndef __ROBOT_PARAMS_H
#define __ROBOT_PARAMS_H

/* ===== 连杆尺寸（实测，原点在肩轴中心）===== */
#define ARM_A1_MM                 (0.0f)      /* 原点在肩轴，忽略 1.56mm 偏航偏移 */
#define ARM_D1_MM                 (0.0f)      /* 原点在肩轴，不再有底座高度项 */
#define ARM_L2_MM                 (100.47f)   /* 肩 -> 肘 */
#define ARM_L3_MM                 (86.68f)    /* 肘 -> 腕 */
#define ARM_L4_MM                 (31.51f)    /* 腕 -> 末端板端 */

/* ===== 关节角度软限位（度）===== */
#define J_BASE_THETA_MIN          (22.5f)     /* 底座右极限，距 +x 轴 22.5° */
#define J_BASE_THETA_MAX          (202.5f)    /* 底座左极限，越过 -x 轴 22.5° */
#define J_UPPER_THETA_MIN         (31.5f)     /* 大臂最低仰角 */
#define J_UPPER_THETA_MAX         (45.0f)     /* 大臂最高仰角 */
#define J_FOREARM_THETA_MIN       (-90.0f)    /* 小臂最内收（近似） */
#define J_FOREARM_THETA_MAX       (-45.0f)    /* 小臂最外伸（近似） */
#define J_END_THETA_MIN           (0.0f)      /* 末端沿小臂方向 */
#define J_END_THETA_MAX           (135.0f)    /* 末端向小臂收（近似） */

/* ===== 各关节逻辑脉宽限位（us）===== */
#define J_BASE_PWM_MIN            (500)
#define J_BASE_PWM_MAX            (2500)
#define J_UPPER_PWM_MIN           (500)
#define J_UPPER_PWM_MAX           (800)
#define J_FOREARM_PWM_MIN         (500)
#define J_FOREARM_PWM_MAX         (1000)
#define J_END_PWM_MIN             (500)
#define J_END_PWM_MAX             (2000)

/* ===== 角度 <-> 脉宽线性映射参数 =====
 * theta = theta_at_pwm_min + dir * (us - pwm_min) / us_per_deg
 * us    = pwm_min + dir * (theta - theta_at_pwm_min) * us_per_deg
 */
#define J_BASE_US_PER_DEG         (2000.0f / 180.0f)
#define J_BASE_THETA_AT_PWM_MIN   (202.5f)
#define J_BASE_DIR                (-1)        /* 脉宽增大，角度减小 */

#define J_UPPER_US_PER_DEG        (2000.0f / 90.0f)
#define J_UPPER_THETA_AT_PWM_MIN  (45.0f)
#define J_UPPER_DIR               (-1)

#define J_FOREARM_US_PER_DEG      (1.0f / 0.09f)
#define J_FOREARM_THETA_AT_PWM_MIN (-90.0f)
#define J_FOREARM_DIR             (1)

#define J_END_US_PER_DEG          (1.0f / 0.09f)
#define J_END_THETA_AT_PWM_MIN    (0.0f)
#define J_END_DIR                 (1)

/* ===== 逻辑关节 -> 物理舵机通道映射（0=CH1 ... 3=CH4）=====
 * 硬件接线：CH4=底座，CH3=大臂，CH2=小臂，CH1=末端。
 * 该映射由 joint.c 的 joint_cfg[] 使用，Servo_SetPulse() 只认物理通道。
 */
#define SERVO_CH_END              (0)
#define SERVO_CH_FOREARM          (1)
#define SERVO_CH_UPPER            (2)
#define SERVO_CH_BASE             (3)

/* ===== 物理通道脉宽限位（us），用于驱动层最后一道夹紧 ===== */
#define SERVO_CH1_PWM_MIN         (500)
#define SERVO_CH1_PWM_MAX         (2000)
#define SERVO_CH2_PWM_MIN         (500)
#define SERVO_CH2_PWM_MAX         (1000)
#define SERVO_CH3_PWM_MIN         (500)
#define SERVO_CH3_PWM_MAX         (800)
#define SERVO_CH4_PWM_MIN         (500)
#define SERVO_CH4_PWM_MAX         (2500)

/* ===== 上电 home 脉宽（us，按物理通道 CH1..CH4）=====
 * 与 joint_cfg[].home_us 使用同一组常量，保证 Motion_Init 与 Servo_Init 一致。
 * home 角度：底座 112.5°，大臂 +45°，小臂 -90°，末端 +90°。
 */
#define SERVO_CH1_HOME_US         (1500)
#define SERVO_CH2_HOME_US         (500)
#define SERVO_CH3_HOME_US         (500)
#define SERVO_CH4_HOME_US         (1500)

/* ===== 通信缓冲 ===== */
#define RINGBUF_SIZE              (128)       /* ringbuf.c 内部实际容量，有效 127 */
#define UART_RX_CHUNK             (64)        /* RxToIdle 中断单次接收缓冲 */
#define PROTO_MAX_LEN             (16)        /* 协议最大帧长，MOVE 为 12 */

/* ===== 工作空间包围盒（按底座 22.5°~202.5° 离线采样）=====
 * 仅用于命令层粗判；最终仍由 IK + 关节限位细判。
 * z 和 phi 由大臂/小臂/末端限位决定，与底座角度无关。
 */
#define WS_X_MIN                  (-201.46f)
#define WS_X_MAX                  (186.12f)
#define WS_Y_MIN                  (-77.10f)
#define WS_Y_MAX                  (201.46f)
#define WS_Z_MIN                  (-48.28f)
#define WS_Z_MAX                  (102.55f)
#define WS_PHI_MIN                (-58.5f)
#define WS_PHI_MAX                (135.0f)

/* ===== 运动插值参数 ===== */
#define MOTION_STEP_MS            (20)        /* Motion_Task 调用周期 */
#define MOTION_HOME_MS            (800)       /* 复位平滑时长 */
#define MOTION_MIN_MS             (300)       /* MOVE 最短时长 */
#define MOTION_MAX_MS             (2000)      /* MOVE 最长时长 */
#define MOTION_MAX_SPEED_DPS      (60.0f)     /* 自动时长使用的最大关节速度 */

/* ===== 上位机通信看门狗 ===== */
#define UART_TIMEOUT_MS           (500)       /* 运动结束后无有效帧的停机时间 */

/* ===== 按键时间参数 ===== */
#define BUTTON_SCAN_MS            (10)        /* 按键扫描周期 */
#define BUTTON_DEBOUNCE_MS        (20)        /* 去抖时间 */
#define BUTTON_HOLD_MS            (1000)      /* 长按判定 */
#define BUTTON_REPEAT_MS          (50)        /* 长按连发间隔 */

/* ===== 任务周期 ===== */
#define UI_PERIOD_MS              (50)        /* OLED 刷新周期 */
#define WDG_PERIOD_MS             (50)        /* 看门狗检查周期 */

/* ===== 命令队列 ===== */
#define CMD_QUEUE_SIZE            (4)         /* 满时丢弃最旧命令 */

#endif

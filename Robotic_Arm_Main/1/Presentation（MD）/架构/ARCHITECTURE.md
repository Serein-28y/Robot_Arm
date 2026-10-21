# 机械臂固件系统架构与代码地图

> 工程：`Robotic_Arm_Main\1`（CMake 工程名 `Test3`）
> 适用固件：STM32F103C8T6 + 4 路总线舵机/PWM + 软件 I2C OLED + 4 键
> 运行框架：裸机前后台（`while(1)` + 软节拍）
>
> 本文档是**代码级架构地图**，重点说明分层、函数调用关系和数据流。
> 需求、协议细节、标定参数分别见同目录 `design_idea.md` 和 `../资料/ARM_HANDOFF.md`。
> 参数数值统一以 `User/Inc/config/robot_params.h` 为准，本文不复制具体标定量。

---

## 1. 一句话总览

> **输入（串口/按键）→ 命令层 → 算法/运动层 → 驱动层 → 舵机执行；ACK、OLED 是反馈旁路。**

核心原则：

1. **单一真值**：`motion.c` 维护 `current/start/target` 三个关节角数组；所有输入只改目标或触发急停。
2. **统一输出**：只有 `app.c` 的 `apply_servo()` 把关节角转成 PWM，避免多个模块直接抢舵机。
3. **纯逻辑下沉**：`joint/kinematics/motion/cmd` 不依赖 HAL，便于移植和单测。
4. **中断只搬运**：UART 中断只写 ringbuf，协议解析、IK、OLED 全在主循环。

---

## 2. 分层与目录

手写代码全部在 `User/`，`Core/` 只保留 CubeMX 生成文件。

```
Core/                         CubeMX 生成
├─ Inc/  main.h / stm32f1xx_it.h / stm32f1xx_hal_conf.h
└─ Src/  main.c / stm32f1xx_it.c / stm32f1xx_hal_msp.c
          syscalls.c / sysmem.c / system_stm32f1xx.c

User/                         手写代码
├─ Inc/
│  ├─ app/          app.h, ui.h
│  ├─ control/      cmd.h, motion.h, kinematics.h, joint.h
│  ├─ services/     proto.h, ringbuf.h, button.h, sched.h
│  ├─ drivers/      servo_drv.h, uart_drv.h, key_drv.h
│  ├─ drivers/display/  oled.h, oled_data.h
│  ├─ config/       robot_params.h
│  └─ diagnostics/  test.h
└─ Src/
   ├─ app/          app.c, ui.c
   ├─ control/      cmd.c, motion.c, kinematics.c, joint.c
   ├─ services/     proto.c, ringbuf.c, button.c, sched.c
   ├─ drivers/      servo_drv.c, uart_drv.c, key_drv.c
   ├─ drivers/display/  oled.c, oled_data.c
   └─ diagnostics/  test.c

Drivers/                      STM32 HAL，原样保留
cmake/                        CubeMX 生成
CMakeLists.txt                根构建脚本，只列 User/Src
Test3.ioc
Presentation（MD）/
├─ 架构/     ARCHITECTURE.md（本文）、design_idea.md
├─ 学习/     DAY1_学习清单与自检.md、每日记录/
└─ 资料/     ARM_HANDOFF.md
```

### 分层职责

| 层 | 文件 | 职责 | 是否碰 HAL |
|---|---|---|---|
| **应用层 App** | `app.c/h`、`ui.c/h`、`diagnostics/test.c/h` | 初始化、MENU/HOST 状态机、任务调度、OLED、手动标定页 | 否 |
| **控制层 Control** | `cmd.c/h`、`motion.c/h` | 命令队列/执行、轨迹插值、急停、busy 状态 | 否 |
| **算法层 Algorithm** | `kinematics.c/h`、`joint.c/h` | FK/IK、工作空间、关节模型、θ↔us、软限位、home | 否 |
| **服务层 Services** | `proto.c/h`、`ringbuf.c/h`、`button.c/h`、`sched.c/h` | 协议、字节缓冲、按键事件、软节拍 | `sched` 用 `HAL_GetTick` |
| **驱动层 Drivers/BSP** | `servo_drv.c/h`、`uart_drv.c/h`、`key_drv.c/h`、`oled.c/h` | TIM1 PWM、USART2、GPIO 按键、软件 I2C OLED | 是 |
| **配置层 Config** | `robot_params.h` | 连杆、限位、标定、home、工作空间、节拍 | 否 |
| **HAL/CubeMX** | `Core/`、`Drivers/` | 时钟、外设初始化、中断入口 | 是 |

> 当前 `User/*/control/` 目录里同时放了控制层和算法层；逻辑上可以再把 `joint/kinematics` 拆成 `algorithm/`，目前保持现结构。

---

## 3. 分层架构图

```
┌──────────────────────────────────────────────────────┐
│ App 应用层                                           │
│  app.c    初始化 / MENU·HOST 状态机 / 任务调度        │
│  ui.c     OLED 菜单页 + HOST 页                       │
│  test.c   手动标定页（main 不调用）                   │
├──────────────────────────────────────────────────────┤
│ Control 控制层                                       │
│  cmd.c    命令队列、MOVE/HOME/STOP/QUERY/SET_JOINT    │
│  motion.c 20ms 关节插值、ease-in-out、急停            │
├──────────────────────────────────────────────────────┤
│ Algorithm 算法层                                     │
│  kinematics.c  FK / IK / 工作空间包围盒               │
│  joint.c       关节模型、θ↔us、软限位、home           │
├──────────────────────────────────────────────────────┤
│ Services 服务层                                      │
│  proto.c    二进制帧解析、ACK/ERR                     │
│  ringbuf.c  中断写 / 主循环读                         │
│  button.c   去抖、短按、长按                          │
│  sched.c    10/20/50ms 软节拍                         │
├──────────────────────────────────────────────────────┤
│ Drivers 驱动层                                       │
│  servo_drv  TIM1 CH1..CH4 PWM                        │
│  uart_drv   USART2 RxToIdle + 发送                    │
│  key_drv    PB12..PB15 按键                           │
│  oled       软件 I2C 显示                            │
├──────────────────────────────────────────────────────┤
│ Config / HAL                                         │
│  robot_params.h → HAL/CubeMX → STM32F103             │
└──────────────────────────────────────────────────────┘
```

---

## 4. 依赖方向

```
main
 └─ app
     ├─ cmd ──► motion ──► joint
     │    └───► kinematics ──► joint
     ├─ ui ──► oled
     ├─ proto ──► ringbuf
     │    ├────► uart_drv
     │    └────► cmd
     ├─ button ──► key_drv
     ├─ sched ──► HAL_GetTick
     └─ servo_drv / uart_drv / key_drv
```

规则：

- 上层依赖下层，算法层不碰 HAL。
- `cmd` **不依赖** `proto`；协议解析后只调用 `Cmd_Submit()`。
- ACK/ERR 由 `app.c` 根据 `CmdResult` 调用 `Proto_SendAck/Err()`，避免 `cmd ↔ proto` 循环依赖。
- `motion` **不直接写舵机**；它只更新当前关节角，`app.c` 负责 `θ→us→PWM`。
- `ui` 不直接读 `cmd/motion`，只接收 `app.c` 组装好的 `UiState`。

---

## 5. 启动流程

`Core/Src/main.c`：

```
HAL_Init
SystemClock_Config
MX_GPIO_Init
MX_TIM1_Init
MX_USART2_UART_Init
App_Init()
while (1) App_Run()
```

`App_Init()`（`User/Src/app/app.c`）顺序不能乱：

```
RingBuf_Init
Proto_Init
Joint_Init        → 计算 home 关节角
Motion_Init       → 用 home 初始化 current/target
Servo_Init        → 先写 home 脉宽，再启动 PWM
Uart_Init         → 启动 RxToIdle 中断
Sched_Init
Button_Init       → 调用 KeyDrv_Init
Cmd_Init          → 用当前角 FK 初始化 UI 位姿
UI_Init           → OLED + 菜单页
```

原因：

- `Joint_Init` 最先，`Motion_Init` 和 `Cmd_Init` 都依赖 home/当前角。
- `Servo_Init` 先写 CCR 再启动 PWM，避免上电从 0us 起跳。
- `Uart_Init` 在 `RingBuf_Init` 之后，中断才能安全写缓冲。
- `UI_Init` 最后，屏幕显示的是已经准备好的系统状态。

---

## 6. 主循环与任务节拍

`App_Run()`（`User/Src/app/app.c`）每轮执行：

```
now = Sched_Now()

if HOST:
    Proto_Poll(now)          // 串口 → 命令队列
    if Cmd_Task(&result):    // 执行一条命令
        result.err==0 ? Proto_SendAck(result.theta)
                      : Proto_SendErr(result.err)
else MENU:
    RingBuf_Skip(所有数据)   // 菜单页丢弃串口
    Cmd_Clear()

if 10ms:  button_task()      // 模式/复位/急停
if 20ms:  Motion_Task()      // 插值
          apply_servo()      // θ→us→PWM
if 50ms:  watchdog_task()    // 通信看门狗
if 50ms:  update_ui()
          UI_Task()
```

| 周期 | 任务 | 函数 |
|---|---|---|
| 每循环 | 串口解析/命令执行（仅 HOST） | `Proto_Poll` / `Cmd_Task` |
| 10ms | 按键 | `button_task` → `Button_Scan` |
| 20ms | 轨迹插值 + 舵机输出 | `Motion_Task` + `apply_servo` |
| 50ms | 通信看门狗 | `watchdog_task` |
| 50ms | OLED | `update_ui` + `UI_Task` |

---

## 7. 数据流

### 7.1 串口下行 → 舵机

```
USART2 中断
  → HAL_UARTEx_RxEventCallback()        drivers/uart_drv.c
  → RingBuf_Write()                     services/ringbuf.c
  → 重新挂 HAL_UARTEx_ReceiveToIdle_IT

App_Run(HOST)
  → Proto_Poll()                        services/proto.c
      找 0xAA / LEN / 校验 / 解析 CMD
      → Command{type,n,arg[]}
      → Cmd_Submit()                    control/cmd.c

  → Cmd_Task()                          control/cmd.c
      MOVE:      InWorkspace → IK_Solve → Motion_MoveTo
      SET_JOINT: 单关节赋值 → Joint_Clamp → Motion_MoveTo
      HOME:      Motion_MoveTo(home,800ms)
      STOP:      Motion_Stop
      QUERY:     返回当前角

  → 每 20ms: Motion_Task()              control/motion.c
      current = start + (target-start) * ease(p)

  → apply_servo()                       app/app.c
      Motion_GetCurrent(theta)
      → Joint_ServoChannel(j)
      → Joint_ThetaToUs(j,theta)        control/joint.c
      → Servo_SetPulse(ch,us)           drivers/servo_drv.c
      → TIM1 CCR
```

### 7.2 上行 ACK / ERR

```
Cmd_Task(&result)
  → app.c
      result.err == 0
        → Proto_SendAck(result.theta)   services/proto.c  → 0x81
      else
        → Proto_SendErr(result.err)     services/proto.c  → 0x82
  → Uart_Send()                         drivers/uart_drv.c
  → USART2 TX
```

错误码：

| code | 来源 |
|---|---|
| `0x01` | 工作空间越界 / IK 不可达 |
| `0x02` | 帧校验失败 |
| `0x03` | 参数非法/长度错误 |
| `0x04` | 未知命令 |

`Proto` 解析错误会同时调用 `Cmd_ReportError()`，让 OLED 也能显示错误。

### 7.3 按键 → 模式/安全

```
GPIO PB12..PB15
  → KeyDrv_Read()                       drivers/key_drv.c
  → Button_Scan()                       services/button.c
      去抖/短按/长按
  → button_task()                       app/app.c

KEY1: 仅扫描（当前只有 MODE1）
KEY2: 短按 MENU→HOST；HOST 长按 1s→MENU
KEY3: Cmd_Home()     → 清队列 → 800ms 回 home
KEY4: Cmd_StopNow()  → 清队列 → Motion_Stop → UI 急停
```

进入 HOST 时 `s_key2_ignore_hold=1`，等 KEY2 松开后才允许长按退出，避免同一次按住立即退出。

### 7.4 状态 → OLED

```
Cmd_GetStatus(&status)                  control/cmd.c
  → Motion_GetCurrent(theta)
  → s_pose / s_err / Motion_IsBusy()

app.c 再叠加：
  模式 s_mode
  超时 s_timeout_latched
  急停 s_estop

  → UiState
  → UI_SetState()                       app/ui.c
  → UI_Task()                           app/ui.c
      页面切换整屏刷新
      同页只更新变化的数值区域
```

### 7.5 通信看门狗

```
watchdog_task()                          app/app.c
  if 模式 != HOST: return
  if 收到新有效帧: 刷新 s_last_activity，清超时锁
  if Motion_IsBusy(): 持续刷新 s_last_activity
  if 空闲且 now - s_last_activity > 500ms:
      Cmd_StopNow()
      s_timeout_latched = 1
```

语义：**运动中不打断当前命令；运动结束后 500ms 无有效帧才急停。**

---

## 8. 函数调用树（带注释）

> 这张图不是让你背函数名，而是回答三个问题：
>
> 1. **谁在什么时候调用谁？**（初始化 / 每循环 / 10ms / 20ms / 50ms）
> 2. **调用时传什么、返回什么、改了什么状态？**
> 3. **哪些是公共接口，哪些是内部实现，哪些可以当黑盒？**

### 8.1 三种视角

| 视角 | 看什么 | 用途 |
|---|---|---|
| **时间视角** | `App_Init` 一次；`App_Run` 每循环；10/20/50ms 任务 | 解释非阻塞和实时性 |
| **数据视角** | `Command → theta → us → PWM`，`状态 → UI/ACK` | 解释端到端流程 |
| **层视角** | 公共 API / 内部 helper / 黑盒边界 | 解释模块化和可移植性 |

先记住 30 秒总览：

```
输入（串口/按键）
  → Command
  → Cmd_Task
  → 目标关节角
  → Motion 插值
  → 当前关节角
  → Joint（θ→us）
  → Servo（PWM）

反馈：CmdStatus + App 状态 → UiState → OLED
      CmdResult → 0x81/0x82 → UART
```

### 8.2 初始化调用树

`App_Init` 只在 `main()` 里调用一次。顺序不能乱：

```
main
└─ App_Init()
   ├─ RingBuf_Init()        // 先有缓冲，UART 中断才能安全写
   ├─ Proto_Init()          // 清协议内部有效帧时间戳
   ├─ Joint_Init()          // 先算 home 关节角；Motion/Cmd 都依赖它
   ├─ Motion_Init()         // current=start=target=home
   │  └─ Joint_GetHome()
   ├─ Servo_Init()          // 先写四路 home CCR，再启动 PWM
   ├─ Uart_Init()           // 启动 RxToIdle；之后中断开始写 ringbuf
   ├─ Sched_Init()          // 记录 10/20/50ms 任务基准 tick
   ├─ Button_Init()         // 建立四键去抖状态
   │  └─ KeyDrv_Init()
   ├─ Cmd_Init()            // 清队列/错误；FK(current) 初始化 UI 位姿
   │  ├─ Motion_GetCurrent()
   │  └─ FK_Solve()
   └─ UI_Init()             // OLED 初始化并绘制菜单页
      ├─ OLED_Init()
      └─ draw_menu()
```

为什么是这个顺序：

- `Joint_Init` 必须先算 home，`Motion_Init` 和 `Cmd_Init` 都读当前角。
- `Servo_Init` 先写 CCR 再 `HAL_TIM_PWM_Start`，上电第一个脉冲就是 home。
- `Uart_Init` 必须在 `RingBuf_Init` 之后，否则中断写缓冲不安全。
- `UI_Init` 最后，屏幕显示的是已经准备好的系统状态。

#### 8.2.1 初始化逐函数说明

| 函数 | 谁调用 | 用途 | 输入 | 输出 / 状态变化 | 它调用谁 |
|---|---|---|---|---|---|
| `App_Init()` | `main()` | 全系统初始化 | 无 | 各模块就绪；进入 MENU；清标志；建立看门狗基准 | `RingBuf_Init`、`Proto_Init`、`Joint_Init`、`Motion_Init`、`Servo_Init`、`Uart_Init`、`Sched_Init`、`Button_Init`、`Cmd_Init`、`UI_Init`、`Sched_Now`、`Proto_GetLastValidTick` |
| `RingBuf_Init()` | `App_Init` | 清环形缓冲读写索引 | 无 | 缓冲为空 | 无 |
| `Proto_Init()` | `App_Init` | 清协议内部有效帧时间戳 | 无 | `s_last_valid=0` | 无 |
| `Joint_Init()` | `App_Init` | 预计算 home 关节角 | 无 | 填 `s_home[]`；`s_ready=1` | `Joint_UsToTheta` |
| `Motion_Init()` | `App_Init` | 用 home 初始化运动状态 | 无 | `current=start=target=home`；`busy=0` | `Joint_GetHome`、`copy_joints` |
| `Servo_Init()` | `App_Init` | 先写 home CCR，再启动四路 PWM | 无 | TIM1 CH1..CH4 输出 home 脉宽 | `__HAL_TIM_SET_COMPARE`、`HAL_TIM_PWM_Start` |
| `Uart_Init()` | `App_Init` | 启动 USART2 RxToIdle 接收 | 无 | 中断接收就绪；回调开始写 ringbuf | `HAL_UARTEx_ReceiveToIdle_IT` |
| `Sched_Init()` | `App_Init` | 记录各软任务基准 tick | 无 | 填 `s_last[]` | `HAL_GetTick` |
| `Button_Init(now)` | `App_Init` | 建立四键去抖状态 | `now` | 四键 `last_raw/stable/t_down` 初始化 | `KeyDrv_Init`、`KeyDrv_Read` |
| `Cmd_Init()` | `App_Init` | 清队列/错误；初始化 UI 位姿 | 无 | 队列空；`s_pose=FK(current)` | `Motion_GetCurrent`、`FK_Solve` |
| `UI_Init()` | `App_Init` | 初始化 OLED 并绘制菜单页 | 无 | 菜单页显示 | `OLED_Init`、`draw_menu` |
| `Sched_Now()` | `App_Init`、`App_Run` | 读取当前毫秒 tick | 无 | 返回 tick | `HAL_GetTick` |
| `Proto_GetLastValidTick()` | `App_Init`、`watchdog_task` | 读取最近有效帧 tick | 无 | 返回 tick | 无 |

### 8.3 运行时调用树（按时间片）

`App_Run` 每轮执行；不要把它看成一条顺序大链，而是 4 个时间片。

> 下面的树是**快速索引**；8.3.1~8.3.6 的表格才是逐函数说明（谁调用、用途、输入、输出/状态、它调用谁）。

```
App_Run()                                // 主循环单次调度：按 4 个时间片各跑一次，全程不阻塞
│
├─【每轮】【HOST】串口控制链              // 仅 HOST 模式执行
│  ├─ Proto_Poll(now)                     // 解析串口：找帧→验和→组 Command→入队，不执行运动
│  │  ├─ RingBuf_GetLength/Peek/Skip      // 从 ISR 缓冲取字节；GetLength 判可读，Peek 看，Skip 消费
│  │  ├─ frame_sum()                      // 内部 helper：前 len-1 字节累加和，与末字节比较做校验
│  │  ├─ send_error()                     // 内部 helper：Cmd_ReportError（UI 显示）+ Proto_SendErr（回 0x82）
│  │  └─ Cmd_Submit(&command)             // 只入队，不执行运动；STOP 先清队列，队列满丢最旧
│  │
│  ├─ Cmd_Task(&result)                   // 每轮最多执行一条：出队→按类型执行→填 CmdResult
│  │  ├─ queue_pop()                      // 内部 helper：环形队列出队（head 前移，count--）
│  │  ├─ Motion_GetCurrent(cur)           // 读当前插值角，供 IK 选“离当前最近”的肘解
│  │  ├─ [MOVE] Kinematics_InWorkspace → IK_Solve   // 先包围盒粗判，再逆解+关节限位+选最近解
│  │  ├─ [SET_JOINT] Joint_Clamp          // 单关节目标先夹到软限位，再规划运动
│  │  ├─ [MOVE/SET_JOINT] Motion_MoveTo(target, dur) // 按最大角差算时长，只设 start/target/步数
│  │  ├─ [HOME] Joint_GetHome → Motion_MoveTo(home, 800ms) // 读预计算 home，固定 800ms 回零
│  │  ├─ [STOP] Motion_Stop               // target=current、busy=0，立即停住不再插值
│  │  └─ [QUERY] FK_Solve                 // 正解算末端位姿，更新 UI 显示并回当前角
│  │
│  └─ Proto_SendAck / Proto_SendErr       // 把 CmdResult 变成 0x81 成功（带四角）/ 0x82 错误（带错误码）
│     └─ Uart_Send                        // 阻塞发送 12/5 字节（HAL_UART_Transmit）
│
├─【每轮】【MENU】丢弃串口                // 菜单态不响应上位机，防止切 HOST 后执行陈年旧帧
│  ├─ RingBuf_Skip(RingBuf_GetLength())   // 一次丢弃全部未读字节，读指针前移
│  └─ Cmd_Clear()                         // 清空命令队列（head/tail/count=0），不改运动状态
│
├─【10ms】button_task                     // 四键扫描：去抖→短按/长按→模式/回零/急停
│  ├─ Button_Scan(KEY1..KEY4) → KeyDrv_Read // 去抖后产生 PRESS/RELEASE/REPEAT；KeyDrv_Read 读原始 GPIO
│  ├─ KEY2 短按 → enter_host；长按 → exit_host // MENU→HOST（清队列/重置看门狗）；HOST 长按 1s 退出并急停
│  ├─ KEY3 → Cmd_Home                     // 手动回零：清队列，target=home，UI 位姿=home
│  └─ KEY4 → Cmd_StopNow                  // 手动急停：清队列，target=current，置 s_estop 供 UI 显示
│
├─【20ms】Motion_Task                     // 按 20ms 步数做 ease-in-out 插值（s=3p²-2p³）
│  └─ 只更新 current；不直接碰舵机        // 运动与输出解耦：真正写 PWM 在 apply_servo
│
├─【20ms】apply_servo                     // 全系统唯一写舵机的出口：current→角度→us→CCR
│  ├─ Motion_GetCurrent(theta)            // 取当前插值角（度）
│  ├─ Joint_ServoChannel(j)               // 逻辑关节 → 物理 CH（BASE=CH4 ... END=CH1）
│  ├─ Joint_ThetaToUs(j, theta)           // 角度 → us（含关节软限位 + 脉宽硬限位）
│  ├─ Servo_GetPulse(ch)                  // 读驱动缓存脉宽，与目标相同则跳过，减少重复写
│  └─ Servo_SetPulse(ch, us) → TIM1 CCR   // 限幅后写 CCR 并更新缓存，改变舵机脉宽
│
├─【50ms】watchdog_task                   // 仅 HOST 生效：500ms 无有效帧且未在运动则急停（只触发一次）
│  ├─ Proto_GetLastValidTick()            // 读最近一次有效帧 tick，与上轮比较判断有无新帧
│  ├─ Motion_IsBusy()                     // 运动中持续刷新活动时间，避免正常长运动被误判超时
│  └─ Cmd_StopNow()                       // 超时急停：清队列、停止插值，置 s_timeout_latched
│
└─【50ms】UI 链                           // 提交快照 → 增量刷屏
   ├─ update_ui()                         // 组装 UiState：模式/运动/错误/超时/急停/位姿/四关节角
   │  ├─ Cmd_GetStatus() → Motion_GetCurrent() // 读命令层位姿/错误 + 运动层当前角/busy
   │  └─ UI_SetState()                    // 只拷贝快照并置 s_dirty=1，不刷屏
   └─ UI_Task()                           // 真正刷屏：切页整屏刷新，同页只刷变化区域
      └─ OLED_Show... / OLED_UpdateArea   // 写显示缓冲并局部/整屏刷新（软件 I2C）
```

> 注意：`update_ui()` 和 `UI_Task()` 是 `App_Run` 中的两个**顺序调用**，不是 `update_ui()` 调用 `UI_Task()`。
> `update_ui()` 只负责组装并提交 `UiState`；`UI_Task()` 才真正刷 OLED。

#### 8.3.1 每轮 HOST：逐函数说明

| 函数 | 谁调用 | 用途 | 输入 | 输出 / 状态变化 | 它调用谁 |
|---|---|---|---|---|---|
| `App_Run()` | `main` 的 `while(1)` | 主循环单次调度 | 无 | 无返回；把各时间片任务跑一遍 | `Sched_Now`、`Sched_IsDue`、`Proto_Poll`、`Cmd_Task`、`Proto_SendAck/Err`、`RingBuf_Skip`、`Cmd_Clear`、`button_task`、`Motion_Task`、`apply_servo`、`watchdog_task`、`update_ui`、`UI_Task` |
| `Proto_Poll(now)` | `App_Run`（HOST） | 从 ringbuf 找完整帧、校验、解析成 `Command` 并入队 | `now`：当前 tick | 刷新 `s_last_valid`；通过 `Cmd_Submit` 改命令队列；解析错误回 ERR | `RingBuf_GetLength/Peek/Skip`、`read_le16`、`frame_sum`、`send_error`、`Cmd_Submit` |
| `RingBuf_GetLength/Peek/Skip` | `Proto_Poll`、`App_Run`（MENU） | 查看/消费中断写入的字节 | `Peek/Skip` 的索引/长度 | 返回长度/字节；`Skip` 移动读指针 | 无 |
| `frame_sum(frame,len)` | `Proto_Poll` | 计算前 `len-1` 字节累加和 | `frame`、`len` | 返回校验和 | 无 |
| `send_error(code)` | `Proto_Poll` | 解析错误统一出口 | 错误码 | 更新 UI 错误；发送 `0x82` | `Cmd_ReportError`、`Proto_SendErr` |
| `Cmd_Submit(&command)` | `Proto_Poll` | 把解析结果放入命令队列 | 一条 `Command` | 返回 1；队列满丢最旧；STOP 先清队列 | `Cmd_Clear`（STOP 时） |
| `Cmd_Task(&result)` | `App_Run`（HOST） | 取一条命令并执行 | `result` 输出指针 | 返回 1=执行了一条，0=队列空；填 `CmdResult`；改 Motion target/busy | `queue_pop`、`copy_joints`、`Motion_GetCurrent`、`Kinematics_InWorkspace`、`IK_Solve`、`Joint_Clamp`、`Motion_MoveTo`、`Joint_GetHome`、`Motion_Stop`、`FK_Solve` |
| `queue_pop(&c)` | `Cmd_Task` | 从环形队列取一条命令 | 队列状态 | 输出 `Command`；head/count 变化 | 无 |
| `Motion_GetCurrent(cur)` | `Cmd_Task`、`apply_servo`、`Cmd_GetStatus` | 读当前插值关节角 | 输出数组 | 填 `cur`（度） | 无 |
| `Kinematics_InWorkspace(&target)` | `Cmd_Task`（MOVE） | 工作空间包围盒粗判 | `Pose` | 返回 1=盒内，0=明显越界 | 无 |
| `IK_Solve(&target,cur,solved)` | `Cmd_Task`（MOVE） | 4 自由度逆解；两肘解过滤限位并选最近解 | 目标 `Pose`、当前角 | 返回 1=成功，0=不可达；填 `solved`（度） | `in_limits`、`deg2rad`、`rad2deg` |
| `Joint_Clamp(out)` | `Cmd_Task`（SET_JOINT）、`Motion_MoveTo` | 把四个角夹到软限位 | 四角数组 | 原地修改 | 无 |
| `Motion_MoveTo(target,dur)` | `Cmd_Task`（MOVE/SET_JOINT/HOME） | 设置本次运动的目标、时长和步数 | 目标角、时长 ms | 改 `start/target/steps/busy`；**不改 current** | `Joint_Clamp`、`copy_joints` |
| `Joint_GetHome(home)` | `Cmd_Task`（HOME）、`Motion_Init` | 读取预计算的 home 角 | 输出数组 | 填 `home` | 无 |
| `Motion_Stop()` | `Cmd_Task`（STOP）、`Cmd_StopNow` | 急停：目标=当前 | 无 | `target=current`，`busy=0` | `copy_joints` |
| `FK_Solve(out,&s_pose)` | `Cmd_Task`（SET_JOINT/HOME/STOP/QUERY） | 正解，更新 UI 显示的位姿 | 四角 | 返回 1；填 `Pose` | `deg2rad`、`rad2deg` |
| `Proto_SendAck(theta)` | `App_Run`（HOST） | 组 `0x81` 成功应答 | 四角（度） | 发送 12 字节 | `Uart_Send` |
| `Proto_SendErr(code)` | `App_Run`（HOST）、`send_error` | 组 `0x82` 错误应答 | 错误码 | 发送 5 字节 | `Uart_Send` |
| `Uart_Send(buf,len)` | `Proto_SendAck/Err` | 阻塞串口发送 | 缓冲区、长度 | USART2 TX；无返回 | `HAL_UART_Transmit` |

#### 8.3.2 每轮 MENU：逐函数说明

| 函数 | 谁调用 | 用途 | 输入 | 输出 / 状态变化 | 它调用谁 |
|---|---|---|---|---|---|
| `RingBuf_Skip(RingBuf_GetLength())` | `App_Run`（MENU） | 菜单页丢弃全部串口数据 | 当前长度 | 读指针前移 | 无 |
| `Cmd_Clear()` | `App_Run`（MENU）、`Cmd_Submit`（STOP） | 清空命令队列 | 无 | `head/tail/count=0` | 无 |

#### 8.3.3 10ms 按键：逐函数说明

| 函数 | 谁调用 | 用途 | 输入 | 输出 / 状态变化 | 它调用谁 |
|---|---|---|---|---|---|
| `button_task(now)` | `App_Run`（10ms） | 扫描四键并分发模式/复位/急停 | `now` | 改模式、队列、运动、急停标志 | `Button_Scan`、`Button_Held`、`enter_host`、`exit_host`、`Cmd_Home`、`Cmd_StopNow` |
| `Button_Scan(id,now)` | `button_task` | 去抖、短按、长按事件 | 按键号、`now` | 返回 `BtnEvent`；更新按键状态 | `KeyDrv_Read` |
| `KeyDrv_Read(id)` | `Button_Scan` | 读原始 GPIO 电平 | 按键号 | 返回 1=按下，0=松开 | `HAL_GPIO_ReadPin` |
| `Button_Held(id,now,ms)` | `button_task`（KEY2） | 判断是否持续按住 | 按键号、`now`、阈值 | 返回 1/0 | 无 |
| `enter_host(now)` | `button_task`（KEY2 短按） | 进入 HOST | `now` | `s_mode=HOST`；清队列/错误；重置看门狗；清超时/急停 | `Cmd_Clear`、`Cmd_ReportError`、`Proto_ResetWatchdog`、`Proto_GetLastValidTick` |
| `exit_host()` | `button_task`（KEY2 长按） | 退出 HOST | 无 | `s_mode=MENU`；急停；清超时/急停 | `Cmd_StopNow` |
| `Cmd_Home()` | `button_task`（KEY3） | 手动复位 | 无 | 清队列；`Motion` 目标=home；UI 位姿=home；清错误 | `Cmd_Clear`、`Joint_GetHome`、`Motion_MoveTo`、`FK_Solve` |
| `Cmd_StopNow()` | `button_task`（KEY4）、`watchdog_task`、`exit_host` | 手动/超时急停 | 无 | 清队列；`Motion_Stop`；UI 位姿=current；清错误 | `Cmd_Clear`、`Motion_Stop`、`Motion_GetCurrent`、`FK_Solve` |

#### 8.3.4 20ms 运动与输出：逐函数说明

| 函数 | 谁调用 | 用途 | 输入 | 输出 / 状态变化 | 它调用谁 |
|---|---|---|---|---|---|
| `Motion_Task()` | `App_Run`（20ms） | 按 20ms 步进插值 | 无 | 更新 `current`；结束时 `busy=0` | `copy_joints` |
| `apply_servo()` | `App_Run`（20ms） | 把当前关节角输出到四路舵机 | 无 | 写 TIM1 CCR；更新驱动脉宽缓存 | `Motion_GetCurrent`、`Joint_ServoChannel`、`Joint_ThetaToUs`、`Servo_GetPulse`、`Servo_SetPulse` |
| `Motion_GetCurrent(theta)` | `apply_servo`、`Cmd_Task`、`Cmd_GetStatus` | 读当前插值角 | 输出数组 | 填 `theta` | 无 |
| `Joint_ServoChannel(j)` | `apply_servo` | 逻辑关节 → 物理通道 | 关节号 | 0..3（0=CH1） | 无 |
| `Joint_ThetaToUs(j,deg)` | `apply_servo` | 角度 → 脉宽，含软限位/脉宽限位 | 关节号、角度 | 返回 us | `clampf` |
| `Servo_GetPulse(ch)` | `apply_servo` | 读驱动缓存的当前脉宽 | 物理通道 | 返回 us | 无 |
| `Servo_SetPulse(ch,us)` | `apply_servo` | 写 CCR 并缓存脉宽 | 物理通道、us | PWM 改变 | `__HAL_TIM_SET_COMPARE` |

#### 8.3.5 50ms 看门狗：逐函数说明

| 函数 | 谁调用 | 用途 | 输入 | 输出 / 状态变化 | 它调用谁 |
|---|---|---|---|---|---|
| `watchdog_task(now)` | `App_Run`（50ms） | 判断上位机是否超时 | `now` | 可能调用急停；置/清超时标志 | `Proto_GetLastValidTick`、`Motion_IsBusy`、`Cmd_StopNow` |
| `Proto_GetLastValidTick()` | `watchdog_task` | 读最近有效帧时间 | 无 | 返回 tick | 无 |
| `Motion_IsBusy()` | `watchdog_task` | 判断是否正在运动 | 无 | 返回 1/0 | 无 |
| `Cmd_StopNow()` | `watchdog_task` | 超时急停 | 无 | 清队列、停止插值 | 同 8.3.3 |

#### 8.3.6 50ms UI：逐函数说明

| 函数 | 谁调用 | 用途 | 输入 | 输出 / 状态变化 | 它调用谁 |
|---|---|---|---|---|---|
| `update_ui()` | `App_Run`（50ms） | 组装一帧 UI 快照 | 无 | 提交 `UiState`，置 UI 脏标志 | `Cmd_GetStatus`、`UI_SetState` |
| `Cmd_GetStatus(&status)` | `update_ui` | 读命令层/运动层状态 | 输出指针 | 填 `CmdStatus`（位姿、当前角、busy、错误） | `Motion_GetCurrent`、`Motion_IsBusy` |
| `UI_SetState(&state)` | `update_ui` | 拷贝快照 | `UiState` | 更新 UI 内部状态，置 `s_dirty=1` | 无 |
| `UI_Task()` | `App_Run`（50ms） | 真正刷新 OLED | 无 | 页面切换整屏刷新；同页局部刷新 | `draw_menu`、`draw_host_static`、`draw_values`、`OLED_Update/UpdateArea` |
| `OLED_Show... / OLED_UpdateArea` | `UI_Task` | 显示文字/数字/区域 | 坐标、文本、值 | 屏幕变化 | 软件 I2C（黑盒） |

### 8.4 调用时到底改了什么

这张表比“谁调用谁”更重要：

| 被调用函数 | 输入 | 输出 / 状态变化 |
|---|---|---|
| `Proto_Poll` | ringbuf 字节 | 通过 `Cmd_Submit` 改变命令队列；刷新有效帧 tick |
| `Cmd_Submit` | 一条 `Command` | 入队；STOP 会先清空队列 |
| `Cmd_Task` | 一条 `Command` | 改变 `Motion` 的 target/busy；产出 `CmdResult` |
| `Motion_MoveTo` | 目标角、时长 | 设置 `start/target/steps/busy`，**不改 current** |
| `Motion_Task` | 无 | 按 20ms 步进改变 `current` |
| `Motion_Stop` | 无 | `target=current`，`busy=0` |
| `Joint_Clamp` | 四个角度 | 原地夹到软限位 |
| `Joint_ThetaToUs` | 关节号、角度 | 返回 us；角度/脉宽双重夹紧 |
| `Servo_SetPulse` | 物理通道、us | 写 CCR，更新驱动内部脉宽缓存 |
| `Cmd_GetStatus` | 无 | 读取 `Motion` 当前角、命令层位姿/错误、busy |
| `UI_SetState` | `UiState` | 拷贝快照，置脏标志；真正刷屏在 `UI_Task` |

### 8.5 公共接口 vs 内部实现

你只需要熟记“公共接口”，内部 helper 可以跳读。

| 模块 | 需要会用的公共接口 | 内部 helper（可跳读） |
|---|---|---|
| `app` | `App_Init`、`App_Run` | `enter_host`、`exit_host`、`button_task`、`watchdog_task` |
| `cmd` | `Cmd_Init`、`Cmd_Clear`、`Cmd_Home`、`Cmd_StopNow`、`Cmd_ReportError`、`Cmd_Submit`、`Cmd_Task`、`Cmd_GetStatus` | `queue_pop`、`calc_duration`、`copy_joints` |
| `motion` | `Motion_Init`、`Motion_MoveTo`、`Motion_Task`、`Motion_Stop`、`Motion_IsBusy`、`Motion_GetCurrent`、`Motion_GetTarget` | `copy_joints` |
| `kinematics` | `IK_Solve`、`FK_Solve`、`Kinematics_InWorkspace` | `in_limits`、`deg2rad`、`rad2deg` |
| `joint` | `Joint_Init`、`Joint_UsToTheta`、`Joint_ThetaToUs`、`Joint_Clamp`、`Joint_GetHome`、`Joint_ServoChannel` | `clampf` |
| `proto` | `Proto_Init`、`Proto_Poll`、`Proto_SendAck`、`Proto_SendErr`、`Proto_GetLastValidTick`、`Proto_ResetWatchdog` | `read_le16`、`frame_sum`、`send_error` |
| `button` | `Button_Init`、`Button_Scan`、`Button_Held` | 去抖状态机的内部字段 |
| `sched` | `Sched_Init`、`Sched_Now`、`Sched_IsDue` | — |
| `servo_drv` | `Servo_Init`、`Servo_SetPulse`、`Servo_GetPulse` | 通道/限位表 |
| `uart_drv` | `Uart_Init`、`Uart_Send` | `HAL_UARTEx_RxEventCallback` 的搬运逻辑 |
| `key_drv` | `KeyDrv_Init`、`KeyDrv_Read` | — |
| `ui` | `UI_Init`、`UI_SetState`、`UI_Task` | `draw_menu`、`draw_host_static`、`draw_values` |

### 8.6 哪些可以当黑盒

| 黑盒 | 你只需要知道 | 不需要知道 |
|---|---|---|
| `IK_Solve` | 输入 `Pose + current`，输出四角或失败；会过滤限位、选最近解 | 余弦定理推导、两肘解符号 |
| `Motion_Task` | 20ms 一步，`3t²-2t³` 缓动，只改 current | 每个浮点运算 |
| `Proto_Poll` | 帧格式、校验、错误码、入队 | 校验和每一位怎么算 |
| `RingBuf_*` | 单生产者/单消费者，满时丢弃新数据 | 环形下标回绕细节 |
| `Button_Scan` | 返回 PRESS/REPEAT/RELEASE | 去抖状态机每个分支 |
| `OLED_*` | 初始化、显示字符串/数字、区域刷新 | 字模、软件 I2C 时序 |
| HAL/CubeMX | 外设已初始化，句柄可用 | 寄存器/时钟树细节 |

### 8.7 建议的读代码顺序

不要从调用树根部硬啃，按“一条控制链”读：

```
1. Core/Src/main.c
2. User/Src/app/app.c        → 只看 App_Init / App_Run
3. User/Src/services/proto.c → 只看“字节 → Command”
4. User/Src/control/cmd.c    → 只看“Command → 目标角”
5. User/Src/control/motion.c → 只看“目标角 → current”
6. User/Src/control/joint.c  → 只看“角度 → us”
7. User/Src/drivers/servo_drv.c → 只看“us → CCR”
```

算法内部的 `IK_Solve` 和 HAL/OLED 内部先跳过。

### 8.8 一句话记忆

```
输入 → Command → 目标角 → current → us → PWM
反馈：状态 → OLED；结果 → ACK/ERR
```

只要你能把这句话展开成上面的调用链，并且说清“谁拥有状态、谁可以替换”，就已经掌握了这张图。

### 8.9 四个常见误解

1. **`Proto_Poll` 不执行运动。**  
   它只把字节翻译成 `Command` 并入队；真正执行在 `Cmd_Task`。

2. **`Cmd_Task` 不发 ACK。**  
   它只产出 `CmdResult`；`app.c` 根据结果调用 `Proto_SendAck/Err`，避免 `cmd ↔ proto` 循环依赖。

3. **`Motion_Task` 不写 PWM。**  
   它只更新 `current`；`app.c` 的 `apply_servo()` 才把 `current` 转成 us 并写舵机。

4. **`UI` 不直接读 `Motion`。**  
   `app.c` 通过 `Cmd_GetStatus` 组装 `UiState`，`UI_SetState/UI_Task` 只负责显示。

---

## 9. 一次 MOVE 的完整链路

以 `MOVE(x,y,z,φ)` 为例：

```
1. PC 发送 AA 0C 01 x y z φ SUM
2. USART2 中断 → RingBuf_Write
3. App_Run(HOST) → Proto_Poll
4. 校验通过 → Command{CMD_SET_POSE}
5. Cmd_Submit 入队
6. Cmd_Task：
     包围盒粗判
     IK_Solve 两肘解 → 过滤限位 → 选最近解
     Motion_MoveTo(目标角, 300..2000ms)
7. 每 20ms Motion_Task：
     current 按 3t²-2t³ 缓动逼近 target
8. apply_servo：
     current → Joint_ThetaToUs → Servo_SetPulse → TIM1
9. App 立即回 0x81 + 四角度（0.1°）
```

注意：ACK 是在“命令被接受/目标已下发”时返回，不等运动结束。

---

## 10. 核心数据结构与单位

| 结构 | 位置 | 含义 |
|---|---|---|
| `JointCfg joint_cfg[4]` | `control/joint.c` | 每个关节的脉宽/角度/限位/home/物理通道 |
| `float theta[4]` | 全局约定 | 四个关节角，单位 **度** |
| `Pose` | `control/kinematics.h` | `x,y,z` 单位 **mm**，`phi` 单位 **度** |
| `Command` | `control/cmd.h` | 队列中的一条命令，`arg[]` 单位 mm/度 |
| `CmdResult` | `control/cmd.h` | 命令执行结果：`valid/err/theta` |
| `CmdStatus` | `control/cmd.h` | UI 快照：最近位姿、当前角、busy、错误 |
| `UiState` | `app/ui.h` | App 传给 UI 的完整显示快照 |
| `BtnEvent` | `services/button.h` | NONE/PRESS/REPEAT/RELEASE |
| `SchedId` | `services/sched.h` | 四个软任务编号 |

单一真值：

- **Motion** 拥有 `current/start/target` 三个关节角数组。
- 串口、KEY3、STOP 都只通过 `Motion_MoveTo/Stop` 改目标。
- **App** 是唯一把 `theta` 转成 PWM 的地方。
- Servo 驱动只缓存最近写入的脉宽，不做业务判断。

---

## 11. 模式与安全链

### 模式

```
MENU（上电默认）
  ├─ 串口数据丢弃
  ├─ 命令队列清空
  ├─ KEY2 短按 → HOST
  ├─ KEY3 复位
  └─ KEY4 急停

HOST
  ├─ 串口命令执行
  ├─ KEY2 长按 1s → MENU（退出时急停）
  ├─ KEY3 复位
  ├─ KEY4 急停
  └─ 通信看门狗有效
```

### 安全链

1. **IK 限位**：两支解都过滤 `joint_cfg[].theta_min/max`。
2. **Joint_Clamp**：SET_JOINT 和 Motion 目标再次夹紧。
3. **Joint_ThetaToUs**：角度和脉宽双重夹紧。
4. **Servo_SetPulse**：按物理通道再夹一次。
5. **STOP**：清空队列 + `Motion_Stop`。
6. **KEY4**：立即 `Cmd_StopNow`。
7. **看门狗**：运动中不打断；运动结束后 500ms 无有效帧才急停。
8. **上电**：先写 home 再启动 PWM。

---

## 12. 层间接口的例外

1. **`proto`（服务层）调用 `cmd`（控制层）**
   - 这是有意的接口：协议只负责把帧翻译成 `Command` 并入队。
   - `cmd` 不反向调用 `proto`，ACK/ERR 由 `app` 负责，避免循环依赖。

2. **`app`（应用层）直接调用驱动**
   - `App_Init` 调 `Servo_Init/Uart_Init`，`App_Run` 调 `Servo_SetPulse`。
   - 因为 `app` 是总编排者，负责“谁在什么时候输出”；算法层保持与硬件无关。

---

## 13. 快速定位表

| 想改什么 | 看哪里 |
|---|---|
| 串口帧/协议 | `services/proto.c/h` |
| 命令队列/执行 | `control/cmd.c/h` |
| 逆解/FK | `control/kinematics.c/h` |
| 关节限位/θ↔us | `control/joint.c/h`、`config/robot_params.h` |
| 插值/急停 | `control/motion.c/h` |
| 模式/按键/看门狗 | `app/app.c`、`services/button.c` |
| OLED 页面 | `app/ui.c` |
| PWM/串口/按键/OLED 硬件 | `drivers/...` |
| 启动/中断 | `Core/Src/main.c`、`stm32f1xx_it.c` |
| 底座/连杆/工作空间参数 | `config/robot_params.h` |

---

## 14. 新增模块 checklist

1. 纯逻辑模块放 `User/Inc/control` 或 `User/Inc/services`，不要包含 HAL 头文件。
2. 硬件模块放 `User/Inc/drivers`，只通过 `main.h` 使用 HAL 句柄。
3. 头文件用带层路径引用，例如 `#include "control/joint.h"`。
4. 新增 `.c` 必须加入根 `CMakeLists.txt` 的 `USER_SOURCES`。
5. 新增 `.h` 放在 `User/Inc` 下即可，不需要改 include 路径。
6. 初始化函数在 `App_Init()` 中按依赖顺序调用。
7. 周期任务在 `App_Run()` 中通过 `Sched_IsDue()` 驱动。
8. 对外接口写在 `.h`，内部变量/辅助函数用 `static`。

---

## 15. CMake 说明

### `.md` 文档

**不需要写进 CMakeLists.txt。**

- CMake 只编译 `target_sources` 里的 `.c/.s`；
- `.md` 不是编译输入，不会生成目标文件；
- `target_include_directories` 只给头文件搜索路径；
- 新建 Markdown 后 VS Code 直接可见，不需要 CMake 参与。

### 手写代码

| 变化 | 是否改 CMake |
|---|---|
| 新增 `.md` 文档 | 不改 |
| 新增/移动 `.c` | 改根 `CMakeLists.txt` 的 `USER_SOURCES` |
| 新增/移动 `.h` | 通常不改，只要在 `User/Inc` 下用带层路径引用 |
| 新增第三方库 | 视情况加 include 路径/链接库 |
| 安装/拷贝文档到构建输出 | 需要额外 `install(FILES ...)`，当前不需要 |

当前根 `CMakeLists.txt` 的关键部分：

```cmake
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Inc
)

set(USER_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Src/app/app.c
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Src/app/ui.c
    ...
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${USER_SOURCES})
```

`Core/Src/main.c`、中断、HAL 启动文件仍由 `cmake/stm32cubemx/CMakeLists.txt` 管理，不要把手写模块或文档加进那个生成文件。

---

## 16. 相关文档

| 文档 | 内容 |
|---|---|
| `design_idea.md` | 需求、详细设计、协议、开发计划、参数表 |
| `../资料/ARM_HANDOFF.md` | 硬件/关节标定移交 |
| `ARCHITECTURE.md` | 本文档：分层、调用树、数据流、代码地图 |

# 机械臂项目设计文档

> 工程主线：`Robotic_Arm_Main\1`
> 硬件平台：STM32F103C8T6 + 4 路总线舵机（兼容 PWM）+ 软件 I2C OLED + 4 键
> 运行框架：裸机前后台（`while(1)` + 软节拍）

---

## 0. 修订记录 / 概述

| 版本 | 日期 | 说明 |
|---|---|---|
| v1.0 | 2026-10-16 | 初版：系统架构、详细设计、接口协议、开发计划 |
| v1.1 | 2026-10-17 | 协议改为二进制帧 + RxToIdle 中断；新增环形缓冲 `ringbuf` 模块；尺寸改 `TODO(3D)` |
| v1.2 | 2026-10-17 | 填入实测尺寸；原点定在肩轴；无抓夹、第4关节=末端；改为 4 自由度 IK；按键映射更新 |
| v1.3 | 2026-10-17 | 增加模块化/可移植性设计原则、移植边界与指南、函数术语中文对照、交接摘要 |
| v1.4 | 2026-10-17 | 底座标定修正：500us→202.5°、750us→180°(-x)、1500us→112.5°、2500us→22.5°；工作空间重算；代码迁到 `User/` 分层目录 |

本文档面向机械臂正式版固件开发，包含需求、架构、详细设计、接口协议、工程结构、开发计划与风险。

### 已确认决策速览

| 项 | 决定 |
|---|---|
| 主线工程 | `Robotic_Arm_Main\1` |
| 运行框架 | 裸机前后台（`while(1)` + 软节拍） |
| 串口 | USART2，PA2(TX)/PA3(RX)，115200 8N1 |
| 串口接收 | `HAL_UARTEx_ReceiveToIdle_IT`（空闲断帧）+ 环形缓冲 |
| 串口协议 | 二进制帧 `0xAA / LEN / CMD / PAYLOAD / SUM` |
| 环形缓冲 | `ringbuf.c/h`（纯字节缓冲，协议在 `proto.c`） |
| 坐标原点 | **肩关节轴中心 (0,0,0)**，z 轴竖直向上 |
| 连杆尺寸 | **L2=100.47mm、L3=86.68mm、L4=31.51mm** |
| 末端 | **无抓夹**，第 4 关节 = 末端（pitch） |
| 逆解 | **4 自由度**（位置 + 末端角 φ），解析法，自动选最近解 |
| 底座标定 | 500us→202.5°、750us→180°(-x)、1500us→112.5°、2500us→22.5°；脉宽↑角度↓ |
| 角度限位 | 按 HANDOFF 表；小臂/末端斜率用标称值近似 |
| 按键 | KEY1 模式 / KEY2 进入·退出 / KEY3 复位 / KEY4 急停 |
| 复位 | HANDOFF home 姿态，平滑到位（≈800ms） |
| 工作空间 | 离线采样算包围盒 → `robot_params.h` 常量 |
| 控制方式 | 板载按键 + 串口上位机 + 逆运动学坐标 + 轨迹插值 |

### 0.1 交接摘要（新会话先看这里）

**当前进度**

- 文档版本：**v1.4**
- 已完成：
  - `User/` 分层目录：`app / control / services / drivers / config / diagnostics`
  - 全部业务模块：`app / ui / cmd / motion / kinematics / joint / proto / ringbuf / uart_drv / servo_drv / key_drv / button / sched`
  - `User/Inc/config/robot_params.h`（含底座新标定、工作空间常量）
  - `CMakeLists.txt` 已加入 `User/Inc` 和全部 `User/Src/...`
  - `Test3.ioc` 已开 USART2（PA2/PA3）+ 全局中断
  - 底座标定修正：500us→202.5°、750us→180°(-x)、1500us→112.5°、2500us→22.5°
  - 工作空间按新底座限位重新采样
- 编译状态：**通过**（arm-none-eabi-gcc；RAM 3736B / 18.24%，FLASH 35168B / 53.66%）
- 尚未开始：整机硬件验收（750us 指向 -x、PWM/OLED/串口实测）
- 待办：上板验收、按新标定重新核对左右测试帧、必要时标定小臂/末端斜率

**下一步**

1. 上板确认底座 750us 指向 -x 轴、1500us 为 home 112.5°。
2. 用新标定重新发送左/右 MOVE 帧，核对 ACK 角度。
3. 验收 KEY1/KEY2/KEY3/KEY4、通信超时、OLED 页面。
4. 如需更高精度，补测小臂/末端斜率并更新 `robot_params.h`。

**关键约定（务必遵守）**

- 坐标原点 = **肩关节轴中心**；z 轴竖直向上；忽略 1.56mm 偏航偏移
- 连杆：**L2=100.47、L3=86.68、L4=31.51 mm**
- **无抓夹**，第 4 关节 = 末端（pitch），参与 **4 自由度 IK**
- 串口：USART2 PA2/PA3，115200；**二进制帧 + RxToIdle + ringbuf**
- `MOVE` 载荷：`x,y,z,φ`（int16 小端，φ 单位 0.1°），帧长 **12**
- 按键：**KEY1 模式 / KEY2 进入·退出 / KEY3 复位 / KEY4 急停**
- 底座标定：**500us→202.5°、750us→180°(-x)、1500us→112.5°、2500us→22.5°**
- 角度限位见 **4.1**；小臂/末端斜率为近似值（标称 11.11us/°）

**参考资源**

| 资源 | 路径 |
|---|---|
| 正式工程 | `Robotic_Arm_Main\1`（CMake 工程名 `Test3`） |
| 测试台工程 | `Robotic_Arm_Test\Test3` |
| 设计文档 | `Presentation（MD）\架构\design_idea.md` |
| 架构与代码地图 | `Presentation（MD）\架构\ARCHITECTURE.md` |
| 标定移交 | `Presentation（MD）\资料\ARM_HANDOFF.md` |
| 3D 模型（STEP） | `D:\Robot_Control\Robo1\Project\Robot_Arm\Build_File_Agent\相关建模文件\机械臂.STEP` |
| 环形缓冲参考工程 | `D:\STM32CubeMX\Project\Local_Project\STM32_HAL_Learning\Cyclic_Buffer\Core` |

**构建说明**

```
cmake --preset Debug
cmake --build --preset Debug
```
- 若 `build/Debug` 里是来自 `Robotic_Arm_Test\Test3` 的旧缓存（曾报 CMakeCache 路径不一致），先删除该目录再配置。
- 也可用独立目录验证：`build/manual`。

**新会话启动语**

> 先读 `Presentation（MD）\架构\design_idea.md`，重点看 0.1、第 3 章（架构/模块化）、第 4 章（详细设计）、第 5 章（协议）、第 9 章（参数/术语）；当前从 **M1** 开始实现。

### 架构相关章节导航

- **分层架构图（层次图）**：**第 3.1 节**
- 模块职责清单：3.2
- 依赖方向：3.3
- 数据流：3.4
- 主流程 / 执行链：3.5
- 启动时序与主循环：3.6

> 本文档里英文函数名、字段、宏的中文解释，统一见 **9.6 函数与术语中英对照**。

---

## 1. 项目背景与目标

### 1.1 硬件平台

| 项 | 值 |
|---|---|
| MCU | STM32F103C8T6 |
| 时钟 | HSE 8MHz，PLL×9 = 72MHz；APB1=/2，APB2=/1 |
| 工具链 | CMake + Ninja + arm-none-eabi-gcc |
| IDE | VS Code + STM32Cube 扩展；HAL |
| PWM | TIM1 CH1–CH4，Prescaler 72-1（1us/tick），Period 20000-1（20ms/50Hz），软件脉宽 500–2500us |
| 串口 | USART2，PA2(TX)/PA3(RX)，115200 8N1，接收中断 |
| 显示 | 软件 I2C OLED（SCL PB8 / SDA PB9，江协科技库） |
| 按键 | KEY1 PB15 / KEY2 PB14 / KEY3 PB13 / KEY4 PB12（内部上拉，低有效） |

**舵机 / 通道 / 引脚映射**

| 逻辑 | 名称 | 通道 | 引脚 | pwm_min | pwm_max | home |
|---|---|---|---|---|---|---|
| Servo1 | 底座 BASE | CH4 | PA11 | 500 | 2500 | 1500 |
| Servo2 | 大臂 UPPER | CH3 | PA10 | 500 | 800 | 500 |
| Servo3 | 小臂 FOREARM | CH2 | PA9 | 500 | 1000 | 500 |
| Servo4 | 末端 END（无抓夹） | CH1 | PA8 | 500 | 2000 | 1500 |

> `JointId` 顺序：`J_BASE=0, J_UPPER=1, J_FOREARM=2, J_END=3`，映射关系在 `joint.c` 内维护。

### 1.2 控制目标（四种能力）

1. **板载按键手动控制**：菜单选择模式、复位、急停。
2. **串口/上位机命令控制**：PC 下发二进制帧（坐标或复位指令），MCU 解析执行。
3. **逆运动学坐标控制**：给定末端 `x,y,z` 与末端角 `φ`，反解各关节角（4 自由度）。
4. **轨迹插值/平滑运动**：任意目标均从当前姿态平滑过渡，避免顿挫、保护舵机。

### 1.3 目标效果

```
根据限位正解算出 xyz 最大边界范围
        ↓
上位机发送坐标 / 复位指令 → 串口 → MCU
        ↓
MCU 解析指令 → 运动学逆解 → 舵机偏角与脉冲
        ↓
OLED 显示接收坐标 + 反解四角度（算法层的平滑运动控制）
        ↓
对应舵机执行
```

按键：**KEY1 模式选择；KEY2 进入/退出；KEY3 复位；KEY4 急停**。

---

## 2. 需求分析

### 2.1 功能需求

| 编号 | 需求 | 验收方式 |
|---|---|---|
| F1 | 上电后舵机平滑缓起到 home 姿态 | 上电目视，无同时堵转大电流 |
| F2 | KEY1 选择模式（当前仅 MODE1），KEY2 进入/退出 | 后台上电按键测试 |
| F3 | KEY3 触发复位（回 home），运动中可打断 | 任意姿态按 KEY3 回位 |
| F4 | 串口接收 `MOVE` 帧（x,y,z,φ）并逆解执行 | PC 发坐标帧，末端到位 |
| F5 | 串口接收 `HOME / STOP / QUERY` 帧 | 分别复位/急停/回读 |
| F6 | OLED 实时显示接收坐标与四关节角/脉宽 | 屏幕与串口数据一致 |
| F7 | 目标均平滑到达（可指定时长） | 运动连续无顿挫 |
| F8 | 越界/不可达/校验错有明确应答与提示 | 返回 `0x82` + 错误码 + 屏幕提示 |
| F9 | 通信超时自动急停（上位机模式） | 拔线后舵机停住 |
| F10 | 软限位：任何通路不超关节限位 | 强制下发越界值被夹紧 |
| F11 | KEY4 立即急停 | 运动中按 KEY4 立即停住 |

> 本机**无抓夹**，第 4 关节为末端俯仰，不再有"开合"需求。

### 2.2 非功能需求

| 类别 | 要求 |
|---|---|
| 实时性 | 插值任务 20ms 周期稳定；主循环无长阻塞；中断内不做浮点/OLED |
| 精度 | 位置精度受舵机开环限制；角度按 HANDOFF 线性公式（小臂/末端斜率为近似） |
| 安全 | 软限位 + 急停 + 上电缓起 + 通信看门狗 |
| 可维护 | 模块化、分层、接口稳定；纯逻辑模块可 PC 上单测 |
| 可调试 | 二进制帧 + 串口助手 HEX 收发 + OLED 状态页 + SWD |
| **模块化** | 单一职责、接口与实现分离；每个模块可独立编译/测试 |
| **可移植性** | 硬件相关代码集中在驱动层；换 MCU 只改驱动层与 `robot_params.h` |
| **可测试性** | 纯逻辑模块（joint/motion/kinematics/ringbuf）可在 PC 上单独 `gcc` 单测 |
| **接口稳定性** | 模块 `.h` 公开 API 保持稳定；内部实现一律 `static` |

### 2.3 约束

- **无 FPU**：F103 软件浮点，IK 仅在命令到达时计算一次，禁止放入 20ms 插值环。
- **引脚冲突**：PA9/PA10 已作 TIM1 CH2/CH3；串口改用 USART2 PA2/PA3。
- **舵机开环**：无编码器反馈，只能"发指令 + 限位保护"。
- **小臂/末端斜率未实测**：用标称 180° 舵机（≈11.11us/°）近似，后续可标定修正。
- **无抓夹**：第 4 关节为末端 pitch，参与逆解。

---

## 3. 系统架构设计

### 3.1 分层架构图（层次图）

> 代码级目录、函数调用树、数据流和 CMake 说明见 `Presentation（MD）\架构\ARCHITECTURE.md`。

```
┌──────────────────────────────────────────────────────────────┐
│ 应用层  App                                                    │
│   app.c   上电初始化 / 主循环 / 顶层模式状态机                 │
│   ui.c    OLED 界面（菜单页 / 坐标页 / 状态页）                 │
├──────────────────────────────────────────────────────────────┤
│ 控制与功能层  Control                                          │
│   cmd.c        统一命令队列 + 执行（角度↔脉宽）                 │
│   motion.c     轨迹生成：线性/缓动插值、限速、急停              │
│   kinematics.c 正解 FK / 逆解 IK（4 自由度）                    │
│   joint.c      关节模型：θ↔脉宽换算、软限位、标定参数          │
├──────────────────────────────────────────────────────────────┤
│ 服务层  Services                                               │
│   proto.c    二进制帧解析 / 组包 / 校验 / 应答                  │
│   ringbuf.c  纯字节环形缓冲（中断写 / 主循环读）                │
│   button.c   按键事件（去抖 / 短按 / 长按 / 连发）              │
│   sched.c    软定时器节拍（1 / 20 / 50ms 任务）                 │
├──────────────────────────────────────────────────────────────┤
│ 驱动层  BSP / Driver                                           │
│   servo_drv.c  TIM1 CH1-4 PWM 输出                             │
│   uart_drv.c   USART2 RxToIdle 中断 → ringbuf                  │
│   key_drv.c    GPIO 按键读取                                    │
│   oled.c / oled_data.c  软件 I2C 显示（江协库）                 │
├──────────────────────────────────────────────────────────────┤
│ 硬件抽象  HAL / CubeMX                                          │
│   main.c / stm32f1xx_hal* / SystemClock / TIM1 / USART2        │
└──────────────────────────────────────────────────────────────┘
   依赖方向：上层依赖下层；同层通过接口调用，禁止反向依赖
```

### 3.2 模块清单与职责

| 模块 | 文件 | 职责 | 依赖 |
|---|---|---|---|
| 关节模型 | `joint.c/h` | `theta↔us` 换算、软限位、home、标定表 | 无（纯逻辑） |
| 运动生成 | `motion.c/h` | 起终点插值、缓动、限速、急停 | joint |
| 运动学 | `kinematics.c/h` | FK/IK（4 自由度）、工作空间检查 | joint 常量 |
| 命令层 | `cmd.c/h` | 命令队列、模式分发 | motion, kinematics, joint |
| 协议 | `proto.c/h` | 二进制帧解析、组包、校验、应答 | ringbuf, cmd |
| 环形缓冲 | `ringbuf.c/h` | 纯字节环形缓冲、读写/计数 | 无（纯逻辑） |
| 按键 | `button.c/h` | 按键事件 | key_drv |
| 舵机驱动 | `servo_drv.c/h` | 脉宽写入 TIM1 | HAL |
| 串口驱动 | `uart_drv.c/h` | 接收回调、发送、环形缓冲入口 | HAL, ringbuf |
| 调度 | `sched.c/h` | 节拍任务 | HAL_GetTick |
| 应用 | `app.c/h` | 状态机、主循环 | 全部 |
| 界面 | `ui.c/h` | OLED 刷新 | oled, joint |

### 3.3 依赖方向

```
app ──► cmd / ui
cmd ──► motion / kinematics / joint
proto ──► ringbuf / cmd
uart_drv ──► ringbuf
motion / kinematics ──► joint
joint ──► (robot_params.h)
driver(servo/key/oled) ──► HAL
```

**规则**：上层依赖下层；下层不得 `#include` 上层的头文件；同层只经接口交互。

### 3.4 数据流

```
USART2 RX ──IRQ──► uart_drv(RxEvent 回调) ──► ringbuf ──► proto(找帧/校验)
                                                               │
                                                               ▼
                                                           cmd(入队/执行)
                                                               │
                                      ┌────────────────────────┼────────────────┐
                                      ▼                        ▼                ▼
                                kinematics(IK)            joint(θ↔us)      motion(插值)
                                      └────────────────────────┴────────────────┘
                                                               ▼
                                                         servo_drv(4路PWM)

OLED(ui) ◄── 状态 / 接收坐标 / 四关节角
```

**单一真值原则**：系统只维护"当前关节角 `theta[J_NUM]`"与"目标关节角"。手动/坐标/轨迹只是目标角的三种来源，运动层永远从当前插值到目标 —— 因此任何模式都天然平滑、可急停。

### 3.5 主流程 + 公共执行链

**公共执行链**

```
所有输入 ──► Command ──► 目标关节角 ──► Motion(插值/限速) ──► Joint(限位/θ↔脉宽) ──► Servo(4路PWM)
```

**顶层流程**

```
┌── 顶级菜单（KEY1 选模式，KEY2 进入/退出）──┐
│  MODE1 上位机（当前唯一模式）                │
└────────────────────────────────────────────┘
        │
MODE1: 串口二进制帧 MOVE / HOME / STOP / QUERY
        │  uart_drv → ringbuf → proto 解析
        ▼
   cmd.c: 包围盒粗判 → IK_Solve(x,y,z,φ) → 关节角限位细判
        │                          │
        │  不可达 → 0x82 + OLED 提示 │ 可达
        ▼                          ▼
   joint.c: θ→脉宽(HANDOFF公式)   motion.c: 平滑插值(20ms步)
        │                          │
        └────────► servo_drv.c (TIM1 4路PWM) ◄───────┘
                                     │
                              ui.c: OLED 显示 坐标+四角度
   KEY3 复位 → home 姿态（≈800ms 平滑）
   KEY4 急停 → Motion_Stop()
```

### 3.6 启动时序与主循环任务

```
main()
 ├ HAL_Init / SystemClock_Config / MX_GPIO_Init / MX_TIM1_Init / MX_USART2_UART_Init
 ├ App_Init()
 │   ├ Joint_Init()
 │   ├ Servo_Init()          写 home + 启动 4 路 PWM
 │   ├ OLED_Init()
 │   ├ RingBuf_Init()
 │   ├ Uart_Init()           启动 HAL_UARTEx_ReceiveToIdle_IT
 │   ├ Button_Init()
 │   ├ Motion_Init()
 │   ├ Proto_Init()
 │   └ UI_Init()
 └ while(1)
     ├ Proto_Poll()          每循环：ringbuf → 取帧 → 校验 → Cmd_Submit
     ├ Cmd_Task()            每循环：执行命令
     ├ Button_Scan()         10ms
     ├ Motion_Task()         20ms：插值
     ├ UI_Task()             50ms：OLED
     └ 通信看门狗            MODE1 下 >500ms 无有效帧 → Motion_Stop()
```

### 3.7 模块化与可移植性设计原则

1. **单一职责**：一个模块只做一件事，一个 `.c` 对应一个 `.h`。
2. **接口与实现分离**：
   - `.h` 只暴露 public API（函数原型、必要类型/宏）；
   - 模块内部变量/函数一律 `static`，禁止跨模块直接访问。
3. **单向依赖、禁止循环**：
   ```
   app → cmd/ui → motion/kinematics/joint → proto/button/servo/uart → HAL
   ```
   下层不得 `#include` 上层头文件；同层只经接口交互。
4. **硬件隔离**：
   - HAL / STM32 头文件只允许出现在驱动层（`servo_drv/uart_drv/key_drv/oled`）；
   - 控制层、应用层不得直接调用 HAL。
5. **纯逻辑模块零硬件依赖**：
   - `joint.c`、`motion.c`、`kinematics.c`、`ringbuf.c` 只依赖 `<stdint.h>/<math.h>/<string.h>`；
   - 可在 PC 上用 `gcc` 直接编译单测。
6. **配置集中**：
   - 连杆、限位、工作空间、节拍等全部放 `robot_params.h`；
   - 业务代码禁止魔法数字。
7. **初始化成对**：
   - 每个模块提供 `Xxx_Init()`（必要时 `Xxx_DeInit()`），不依赖隐式全局状态。
8. **中断与主循环解耦**：
   - ISR 只做搬运（`uart_drv → ringbuf`），解析/IK/OLED 全在主循环。
9. **可替换性**：
   - 换舵机/换串口/换显示屏，只改对应驱动层，不动 `cmd/kinematics/joint`。
10. **可测试性**：
    - 纯逻辑模块不依赖硬件，可单独编译 + 断言测试。

### 3.8 模块移植边界

| 模块 | 依赖硬件 | 换 MCU / 外设时要改 |
|---|---|---|
| `app / ui / cmd / motion / kinematics / joint / proto / button` | ❌ 纯逻辑 | 基本不用改 |
| `ringbuf` | ❌ 纯逻辑 | 不用改 |
| `servo_drv` | ✅ TIM1 PWM | 改定时器/通道 |
| `uart_drv` | ✅ USART2 + 中断 | 改串口/中断 |
| `key_drv` | ✅ GPIO | 改引脚 |
| `oled` | ✅ 软件 I2C | 改 I2C 引脚/时序 |
| `main / HAL` | ✅ | 重配时钟/外设 |

> 移植原则：**只动驱动层和 `robot_params.h`，纯逻辑模块原样复用。**

---

## 4. 详细设计

### 4.1 关节模型（joint.c/h）

```c
typedef enum { J_BASE=0, J_UPPER, J_FOREARM, J_END, J_NUM } JointId;

typedef struct {
    const char *name;
    uint16_t    pwm_min, pwm_max;     /* us */
    float       us_per_deg;           /* 斜率（小臂/末端为近似值） */
    float       theta_at_pwm_min;     /* pwm_min 对应角度（度） */
    int8_t      dir;                  /* 脉宽↑ → 角度方向：+1/-1 */
    float       theta_min, theta_max; /* 软限位（度） */
    uint16_t    home_us;
} JointCfg;

extern const JointCfg joint_cfg[J_NUM];

float    Joint_UsToTheta(JointId j, uint16_t us);  /* 脉宽 → 角度 */
uint16_t Joint_ThetaToUs(JointId j, float deg);    /* 角度 → 脉宽（含限位夹紧） */
void     Joint_Clamp(float theta[J_NUM]);          /* 关节角限位夹紧 */
```

**角度换算（us → 度）**

| 关节 | 公式 | 范围 | home |
|---|---|---|---|
| 底座 | `θ1 = 202.5 − (us−500)·180/2000` | 22.5°–202.5° | 1500 → 112.5° |
| 大臂 | `θ2 = 45 − (us−500)·90/2000` | 31.5°–45° | 500 → +45° |
| 小臂 | `θ3 = −[90 − k·(us−500)] = −90 + k·(us−500)`，k≈0.09°/us | −90°–−45°（近似） | 500 → −90° |
| 末端 | `θ4 = k4·(us−500)`，k4≈0.09°/us | 0°–135°（近似） | 1500 → +90°（近似） |

**方向约定**

- 底座：脉宽↑ → 角度↓（**顺时针**，500us→202.5°，750us→180°(-x 轴)，2500us→22.5°）。
- 大臂：脉宽↑ → 仰角↓（往下）。
- 小臂：脉宽↑ → 向大臂收（肘角减小，θ3 变大/更不负）。
- 末端：脉宽↑ → 向小臂收（θ4 增大）。

**角度约定**

- `θ2`：大臂相对水平面的仰角，**向上为正**，0° = 水平。
- `θ3`：小臂相对大臂的**相对角**，0° = 共线伸直，**向下弯为负**；home 时 θ3 = −90°（与大臂夹角 90°）。
- `θ4`：末端板相对小臂的**相对角**，0° = 沿小臂方向，向小臂收为正。

> 小臂/末端斜率 k、k4 未实测，按标称 180° 舵机 11.11us/° 近似；误差可接受，后续可两点标定修正。

### 4.2 正运动学 FK

坐标系：**原点在肩轴中心**，z 轴竖直向上；底座绕竖直轴旋转 θ1。

```
r = L2·cos(θ2) + L3·cos(θ2+θ3) + L4·cos(θ2+θ3+θ4)
z = L2·sin(θ2) + L3·sin(θ2+θ3) + L4·sin(θ2+θ3+θ4)
x = r·cos(θ1)
y = r·sin(θ1)
```

```c
typedef struct { float x, y, z, phi; } Pose;   /* 位姿：x,y,z 位置 + phi 末端板绝对角 */
uint8_t FK_Solve(const float theta[J_NUM], Pose *out);  /* 正运动学：关节角 → 末端位姿 */
```

> 原点在肩，故 `a1 = d1 = 0`，不再有底座偏移项。底座/转盘落在 `z ≈ −47.05mm`（转盘面）、`z ≈ −80.59mm`（底座底面），仅作参考，不参与 IK。

### 4.3 逆运动学 IK（4 自由度，解析法）

目标：末端位置 `(x,y,z)` + 末端板绝对角 `φ`，解 `θ1~θ4`。

```
① 底座
   θ1 = atan2(y, x)
   r  = √(x² + y²)

② 把末端 L4 从目标点减掉，得到“腕心”
   Wr = r  − L4·cosφ
   Wz = z  − L4·sinφ

③ 小臂（余弦定理）
   D = (Wr² + Wz² − L2² − L3²) / (2·L2·L3)
   若 |D| > 1  →  目标不可达，返回错误
   θ3 = ±acos(D)          // 两个肘解

④ 大臂
   θ2 = atan2(Wz, Wr) − atan2(L3·sinθ3, L2 + L3·cosθ3)

⑤ 末端
   θ4 = φ − θ2 − θ3
```

```c
uint8_t IK_Solve(const Pose *t, const float cur[J_NUM], float out[J_NUM]);
/* 逆运动学：末端位姿 → 关节角
 * t   : 目标位姿（x,y,z 位置 + phi 末端角）
 * cur : 当前关节角（用于选最近解）
 * out : 解出的四个关节角
 * 返回: 1=成功, 0=不可达/越界
 */
```

**关于“选哪一支解”**

`θ3 = ±acos(D)`：同一目标点有肘上/肘下两种姿态。本机采用**自动选最近**：两解都算，**先过滤掉超出关节限位的解**，再取与当前姿态关节移动量最小者，避免中途翻肘。

> 底座无多解：`θ1 = atan2(y,x)` 已消除 ±180° 歧义。
> 注意：小臂有效范围是 `θ3 ∈ [−90°, −45°]`（近似），正分支通常会被限位过滤掉。

### 4.4 运动平滑（motion.c/h）

```c
void    Motion_Init(void);                          /* 运动模块初始化 */
void    Motion_MoveTo(const float target[J_NUM], uint16_t dur_ms); /* 平滑移动到目标（0=瞬达） */
void    Motion_Stop(void);                          /* 急停：目标 = 当前 */
void    Motion_Task(void);                          /* 插值任务，每 20ms 调一次 */
uint8_t Motion_IsBusy(void);                        /* 是否还在运动：1=运动中 */
```

- 在**关节空间**做插值（避免在 MCU 上反复算 IK）。
- 缓动曲线 `ease-in-out`：`s = 3t² − 2t³`，消除启停顿挫。
- 急停：把目标角设为当前角，下一步立即停住。

### 4.5 顶层状态机与按键映射

```
顶层菜单
  KEY1 : 选择模式（当前仅 MODE1 上位机）
  KEY2 : 进入当前模式 / 退出

MODE1（上位机）
  串口控制；KEY2 长按 → 返回菜单

全局
  KEY3 : 复位 → home（平滑）
  KEY4 : 急停 → Motion_Stop()

急停来源：KEY4 / STOP 帧 / 通信超时
```

### 4.6 工作空间边界（离线采样）

"根据限位正解算 xyz 最大边界"需要遍历关节空间，F103 无 FPU，MCU 上跑很慢，故**离线计算并固化为常量**。

```
遍历 θ1 ∈ [22.5°, 202.5°]
     θ2 ∈ [31.5°, 45°]
     θ3 ∈ [−90°, −45°]     // 近似
     θ4 ∈ [0°, 135°]       // 近似
   FK → (x,y,z)
记录 min/max  →  生成 robot_params.h 宏
```

运行时只做：

1. **包围盒粗判**：`x/y/z` 是否在 `WS_*` 范围内（秒级判断）。
2. **关节限位细判**：IK 解出的关节角是否在限位内（最准，代价小）。

> 原点在肩轴，所以 `z` 的负值表示在肩轴下方。

### 4.7 环形缓冲（ringbuf.c/h）

纯字节缓冲，**不含协议**；生产者 = 串口接收中断，消费者 = 主循环。

```c
void     RingBuf_Init(void);                               /* 初始化读写指针为 0 */
uint8_t  RingBuf_Write(const uint8_t *data, uint8_t length); /* 写入数据，返回实际写入字节数 */
uint8_t  RingBuf_Peek(uint8_t i);      /* 查看第 i 个待读字节（不移动读指针） */
void     RingBuf_Skip(uint8_t length); /* 跳过 n 字节（移动读指针） */
uint8_t  RingBuf_GetLength(void);      /* 已用字节数 */
uint8_t  RingBuf_GetRemain(void);      /* 剩余空间 */
```

设计要点（相对参考 `command.c` 的修正）：

| 项 | 处理 |
|---|---|
| 容量 | `RINGBUF_SIZE = 128`，有效 127 |
| 满/空判定 | `GetRemain = (SIZE-1) - GetLength`，避免写满误判为空 |
| 索引 | `volatile uint8_t`，中断/主循环共享 |
| 并发模型 | 单生产者（ISR）/ 单消费者（主循环），8 位索引在 Cortex-M3 上原子 |
| 协议解耦 | 参考的 `Command_GetCommand` 逻辑迁到 `proto.c`，ringbuf 只管字节 |

### 4.8 关键数据结构与模块 API

```c
/* ===== cmd.h 命令层 ===== */
typedef enum {
    CMD_SET_POSE,    /* 设置末端位姿 (x,y,z,φ) */
    CMD_SET_JOINT,   /* 设置单个关节角 */
    CMD_HOME,        /* 复位到 home */
    CMD_STOP,        /* 急停 */
    CMD_QUERY        /* 查询当前角度 */
} CmdType;

typedef struct { CmdType type; uint8_t n; float arg[6]; } Command;
void    Cmd_Submit(const Command *c);   /* 提交命令到队列（满则丢最旧） */
uint8_t Cmd_Task(void);                 /* 从队列取出并执行一条命令 */

/* ===== proto.h 协议层 ===== */
void Proto_Init(void);                  /* 协议初始化 */
void Proto_Poll(void);                  /* 从 ringbuf 找帧 → 校验 → 提交命令 */
void Proto_SendAck(float theta[J_NUM]); /* 发送成功应答 0x81 */
void Proto_SendErr(uint8_t code);       /* 发送错误应答 0x82 */

/* ===== uart_drv.h 串口驱动 ===== */
void Uart_Init(void);                   /* 串口初始化并启动接收 */
void Uart_Send(const uint8_t *data, uint16_t len); /* 发送数据 */

/* ===== ringbuf.h 环形缓冲（见 4.7） ===== */

/* ===== button.h 按键 ===== */
typedef enum {
    BTN_NONE,     /* 无事件 */
    BTN_PRESS,    /* 按下 */
    BTN_REPEAT,   /* 长按连发 */
    BTN_RELEASE   /* 松开 */
} BtnEvent;
typedef struct { GPIO_TypeDef *port; uint16_t pin; /* ... */ } Button;
void    Button_Init(void);              /* 按键初始化 */
uint8_t Button_Scan(Button *b);         /* 扫描按键，返回 BtnEvent */

/* ===== servo_drv.h 舵机驱动 ===== */
void Servo_Init(void);                  /* 写 home 并启动 4 路 PWM */
void Servo_SetPulse(uint8_t ch, uint16_t us); /* 设置某通道脉宽 */
```

---

## 5. 接口与协议

### 5.1 串口

| 项 | 值 |
|---|---|
| 外设 | USART2 |
| 引脚 | PA2(TX) / PA3(RX) |
| 参数 | 115200 8N1 |
| 接收方式 | `HAL_UARTEx_ReceiveToIdle_IT`（总线空闲断帧） |
| 接收链路 | USART2 IRQ → `HAL_UARTEx_RxEventCallback` → `RingBuf_Write` → 重新挂接收 |
| 解析位置 | 主循环 `Proto_Poll`（中断内不做解析） |

> PA9/PA10 已作 TIM1 CH2/CH3，故不使用 USART1（默认 PA9/PA10）。

### 5.2 帧格式与校验

```
0xAA | LEN | CMD | PAYLOAD... | SUM
```

- `LEN`：整帧总字节数（含包头、长度、命令、数据、校验）
- `SUM`：前 `LEN-1` 字节累加和的低 8 位
- 最短 4 字节；多字节字段一律 **小端（little-endian）**
- 校验失败 / 长度非法：丢弃 1 字节后继续找下一个 `0xAA`

### 5.3 下行命令表

| CMD | 名称 | PAYLOAD | 帧长 LEN |
|---|---|---|---|
| `0x01` | MOVE | x, y, z, φ 各 int16 小端（mm / 0.1°） | 12 |
| `0x02` | HOME | — | 4 |
| `0x03` | STOP | — | 4 |
| `0x04` | QUERY | — | 4 |
| `0x05` | SET_JOINT | joint(u8) + deg(int16, 0.1°) | 7 |

> MOVE 的 `φ` 是末端板绝对角（相对水平面），单位 0.1°；例如 90° → 900 = `0x0384`。

### 5.4 上行应答

| CMD | 名称 | PAYLOAD | 帧长 LEN |
|---|---|---|---|
| `0x81` | OK | theta[0..3] 各 int16（0.1°） | 12 |
| `0x82` | ERR | code(u8) | 5 |

**错误码**

| code | 含义 |
|---|---|
| `0x01` | 目标超出工作空间 / IK 不可达 |
| `0x02` | 帧校验失败 |
| `0x03` | 参数非法 |
| `0x04` | 未知命令 |

### 5.5 帧示例

| 动作 | 帧 (HEX) |
|---|---|
| MOVE(120, 0, 80, φ=0°) | `AA 0C 01 78 00 00 00 50 00 00 00 7F` |
| MOVE(120, 0, 80, φ=90°) | `AA 0C 01 78 00 00 00 50 00 84 03 06` |
| HOME | `AA 04 02 B0` |
| STOP | `AA 04 03 B1` |
| QUERY | `AA 04 04 B2` |

> 坐标帧是示意；真实可达范围以工作空间包围盒为准。

### 5.6 超时看门狗

上位机模式（MODE1）下，若 > 500ms 未收到有效帧，则自动 `Motion_Stop()`，防止 PC 掉线导致失控。

---

## 6. 工程结构

### 6.1 文件 / 目录

```
Robotic_Arm_Main/1/
├─ Core/                              ← CubeMX 生成，仅保留生成文件
│  ├─ Inc/  main.h / stm32f1xx_it.h / stm32f1xx_hal_conf.h
│  └─ Src/  main.c / stm32f1xx_it.c / stm32f1xx_hal_msp.c
│            syscalls.c / sysmem.c / system_stm32f1xx.c
│
├─ User/                              ← 手写代码，全部在 Core 外面
│  ├─ Inc/
│  │  ├─ app/          app.h, ui.h
│  │  ├─ control/      joint.h, kinematics.h, motion.h, cmd.h
│  │  ├─ services/     proto.h, ringbuf.h, button.h, sched.h
│  │  ├─ drivers/      servo_drv.h, uart_drv.h, key_drv.h
│  │  ├─ drivers/display/  oled.h, oled_data.h
│  │  ├─ config/       robot_params.h
│  │  └─ diagnostics/  test.h
│  └─ Src/
│     ├─ app/          app.c, ui.c
│     ├─ control/      joint.c, kinematics.c, motion.c, cmd.c
│     ├─ services/     proto.c, ringbuf.c, button.c, sched.c
│     ├─ drivers/      servo_drv.c, uart_drv.c, key_drv.c
│     ├─ drivers/display/  oled.c, oled_data.c
│     └─ diagnostics/  test.c
│
├─ Drivers/                           ← STM32 HAL，原样保留
├─ cmake/                             ← CubeMX 生成
├─ CMakeLists.txt                     ← User/Inc + User/Src 列表
├─ Test3.ioc
└─ Presentation（MD）/                 ← 文档目录
   ├─ 架构/                           ← 本文档、ARCHITECTURE.md
   ├─ 学习/                           ← DAY1 清单、每日记录
   └─ 资料/                           ← ARM_HANDOFF.md
```

### 6.2 构建改动

- `main.c`：`USER CODE BEGIN 2` 调 `App_Init()`；`while(1)` 调 `App_Run()`。
- 根 `CMakeLists.txt`：
  - `target_include_directories` 增加 `User/Inc`；
  - `USER_SOURCES` 列出全部 `User/Src/...`；
  - `Core/Inc` 仍由 `cmake/stm32cubemx/CMakeLists.txt` 提供。
- `cmake/stm32cubemx/CMakeLists.txt`：CubeMX 生成文件，不把手写模块加进去。
- `Test3.ioc` / CubeMX：**USART2 已开启**（115200，PA2/PA3，全局中断）。
- `oled.c / oled_data.c`：移动到 `User/Src/drivers/display/`，只改 include 路径，库逻辑和字模不动。

### 6.3 移植指南

换 MCU / 换外设时按下面顺序操作：

1. **只改驱动层**：`servo_drv / uart_drv / key_drv / oled`；
2. **保持 `.h` 函数签名不变**，上层代码无需改动；
3. **改 `robot_params.h`**：引脚、限位、连杆尺寸、节拍；
4. **纯逻辑模块不动**：`joint / motion / kinematics / ringbuf / cmd / proto`；
5. **重新验证**：PC 单元测试 → 台架回环 → 系统验收。

---

## 7. 开发计划

### 7.1 里程碑

| 里程碑 | 内容 | 验证 |
|---|---|---|
| **M1** | 驱动层 `servo_drv/uart_drv/button` + `ringbuf` + `robot_params.h` + `App_Init/Run` 骨架 | 编译通过；USART2 回环；PWM 上电出 home |
| **M2** | `joint.c` + 菜单（KEY1/KEY2）+ KEY3 复位 + KEY4 急停 | 按键切模式、复位到 home、急停有效 |
| **M3** | `proto.c` + `cmd.c`：二进制帧 `MOVE/HOME/STOP/QUERY` | PC 发帧，舵机动、回 `0x81` |
| **M4** | `motion.c` 平滑插值 + 急停 | 运动平滑无顿挫，急停即刻停 |
| **M5** | `kinematics.c` 4 自由度 FK/IK（自动选最近）+ 越界判断 | FK/IK 互逆；越界报错 |
| **M6** | `ui.c`：OLED 显示接收坐标 + 反解四角度 | 屏幕数据与串口一致 |
| **M7** | 集成 / 标定 / 文档 / 交付 | 对照需求逐条验收 |

### 7.2 任务与验证要点

- **M1**：先跑通 `ringbuf + RxToIdle` 回环与 4 路 PWM，再搭应用骨架。
- **M2**：把 `test.c` 的按键去抖逻辑抽到 `button.c`；状态机做模式、复位、急停。
- **M3**：`Proto_Poll` 只负责找帧/校验 → `Cmd_Submit`，与执行解耦。
- **M4**：缓动曲线与急停是安全核心，必须先于 IK 完成。
- **M5**：用实测 L2/L3/L4 验证 4 自由度 IK；φ 分支与符号在 joint.c 内校正。
- **M6**：显示节流 50ms，避免刷屏拖慢主循环。

### 7.3 测试策略

| 层次 | 方式 |
|---|---|
| 纯逻辑模块（ringbuf/joint/motion/kinematics） | PC 上 `gcc` 编译跑断言，无需硬件 |
| 模块单测 | 每个模块单独编译 + 断言，验证接口独立、无隐藏耦合 |
| 驱动模块 | 示波器看 PWM、逻辑分析仪看串口 |
| 集成 | `Robotic_Arm_Test\Test3` 作测试台 |
| 系统 | 对照 F1–F11 需求逐条验收 |

---

## 8. 风险与注意事项

| 编号 | 风险 | 应对 |
|---|---|---|
| R1 | 小臂/末端斜率未实测（用标称近似） | `joint.c` 先用 11.11us/°；需要精度时两点标定修正 |
| R2 | 底座标定曾与实测差 22.5° | 已修正：500us→202.5°、750us→180°(-x)、1500us→112.5°、2500us→22.5°，脉宽↑角度↓ |
| R3 | 引脚冲突（PA9/PA10 已被 TIM1 占用） | 串口用 USART2 PA2/PA3；确认硬件未被占用 |
| R4 | 无 FPU，浮点慢 | IK 仅命令到达时算一次；插值在关节空间做 |
| R5 | 舵机开环无反馈 | 软限位 + 急停 + 上电缓起 + 通信看门狗 |
| R6 | 舵机大电流导致压降/复位 | 独立 5V 大电流供电 + 共地（HANDOFF 已列） |
| R7 | 帧长/缓冲溢出或半帧卡住 | `LEN` 上下界校验；ringbuf 满时丢弃新数据不覆盖 |
| R8 | 二进制帧手工调试不便 | 固定帧表 + 后续可加文本调试协议 |
| R9 | 第4关节按 pitch 建模（若非 pitch 需改公式） | 上电前确认；joint.c 中可调整符号/轴 |
| R10 | 模块耦合导致移植困难 | 遵守 3.7 依赖规则；硬件访问只放驱动层 |

**硬件复测项（HANDOFF TODO）**：舵机独立 5V 大电流供电 + 共地；3.3V 信号稳定性。

---

## 9. 附录

### 9.1 robot_params.h 模板（已填实测值）

```c
#ifndef __ROBOT_PARAMS_H
#define __ROBOT_PARAMS_H

/* ===== 连杆尺寸（实测，原点在肩轴）===== */
#define ARM_A1_MM        (0.0f)     /* 原点在肩轴，忽略 1.56mm 偏航偏移 */
#define ARM_D1_MM        (0.0f)     /* 原点在肩轴 */
#define ARM_L2_MM        (100.47f)  /* 肩 → 肘 */
#define ARM_L3_MM        (86.68f)   /* 肘 → 腕 */
#define ARM_L4_MM        (31.51f)   /* 腕 → 末端板端 */

/* ===== 关节角度限位（度）=====
 * 底座：θ1 = 202.5 − (us−500)·180/2000
 *       500→202.5°，750→180°(-x)，1500→112.5°，2500→22.5°
 */
#define J_BASE_THETA_MIN    (22.5f)
#define J_BASE_THETA_MAX    (202.5f)
#define J_UPPER_THETA_MIN   (31.5f)
#define J_UPPER_THETA_MAX   (45.0f)
#define J_FOREARM_THETA_MIN (-90.0f)   /* 近似 */
#define J_FOREARM_THETA_MAX (-45.0f)   /* 近似 */
#define J_END_THETA_MIN     (0.0f)     /* 近似 */
#define J_END_THETA_MAX     (135.0f)   /* 近似 */

/* ===== 关节脉宽限位（us）===== */
#define J_BASE_PWM_MIN      (500)
#define J_BASE_PWM_MAX      (2500)
#define J_UPPER_PWM_MIN     (500)
#define J_UPPER_PWM_MAX     (800)
#define J_FOREARM_PWM_MIN   (500)
#define J_FOREARM_PWM_MAX   (1000)
#define J_END_PWM_MIN       (500)
#define J_END_PWM_MAX       (2000)

/* ===== 工作空间包围盒（按底座 22.5°~202.5° 离线采样）===== */
#define WS_X_MIN        (-201.46f)
#define WS_X_MAX        (186.12f)
#define WS_Y_MIN        (-77.10f)
#define WS_Y_MAX        (201.46f)
#define WS_Z_MIN        (-48.28f)
#define WS_Z_MAX        (102.55f)

/* ===== 运动参数 ===== */
#define MOTION_STEP_MS   20       /* 插值节拍 */
#define MOTION_HOME_MS   800      /* 复位平滑时长 */
#define UART_TIMEOUT_MS  500      /* 上位机通信超时 */
#define RINGBUF_SIZE     128      /* 环形缓冲大小（ringbuf.c 内部定义）*/

#endif /* __ROBOT_PARAMS_H */
```

### 9.2 按键表

| 按键 | 引脚 | 功能 |
|---|---|---|
| KEY1 | PB15 | **模式选择**（当前仅 MODE1 上位机） |
| KEY2 | PB14 | **确认进入 / 退出** 当前模式 |
| KEY3 | PB13 | **复位**（回 home） |
| KEY4 | PB12 | **急停**（立即 Motion_Stop） |

### 9.3 ARM_HANDOFF 参数摘录

- PWM：TIM1 CH1–CH4，1us/tick，20ms(50Hz)，脉宽 500–2500us。
- home：`servo_home[4] = {1500, 500, 500, 1500}`。
- home 姿态：底座 112.5°；大臂 +45° 仰角；小臂与大臂夹角 90°；末端沿小臂方向起算的中间位（home 1500us）。
- 本机**无抓夹**，CH1 为末端 pitch。
- 坐标系：从机械臂后方看为正方向；z 轴竖直向上。
- OLED：软件 I2C，SCL/SDA = PB8/PB9。

### 9.4 二进制帧结构与校验（MOVE 示例）

```c
/* 帧布局（MOVE，共 12 字节）
 * [0]     0xAA
 * [1]     LEN   = 12
 * [2]     CMD   = 0x01
 * [3..4]  x  (int16 LE, mm)
 * [5..6]  y  (int16 LE, mm)
 * [7..8]  z  (int16 LE, mm)
 * [9..10] φ  (int16 LE, 0.1°)
 * [11]    SUM = (byte[0..10] 累加) & 0xFF
 */
uint8_t sum = 0;
for (uint8_t i = 0; i < len - 1; i++) sum += frame[i];
if (sum != frame[len - 1]) { /* 校验失败 */ }
```

- int16 小端还原：`x = (int16_t)(frame[3] | (frame[4] << 8));`
- 负数示例：`-120` → 16 位补码 `0xFF88` → 字节 `88 FF`。
- φ 示例：`90.0°` → `900 = 0x0384` → 字节 `84 03`。

### 9.5 实测尺寸与原点

| 项 | 值 | 来源 |
|---|---|---|
| 坐标原点 | 肩关节轴中心 (0,0,0) | 用户确认 |
| z 轴 | 竖直向上 | 用户确认 |
| 偏航偏移 a1 | 1.56mm（**忽略**） | SolidWorks 测量 |
| L2 肩→肘 | 100.47mm | SolidWorks 测量 |
| L3 肘→腕 | 86.68mm | SolidWorks 测量 |
| L4 腕→末端板端 | 31.51mm（竖直分量 30.77） | SolidWorks 测量 |
| 转盘面 z | ≈ −47.05mm | 参考，不参与 IK |
| 底座底面 z | ≈ −80.59mm | 参考，不参与 IK |
| 底座角度 | 500us→202.5°，750us→180°(-x 轴)，1500us→112.5°，2500us→22.5°（脉宽↑角度↓） | 用户实测 |
| 大臂角度 | 500us→+45°，800us→+31.5° | HANDOFF |
| 小臂角度 | 500us→与大臂夹角 90° | 用户确认 |
| 末端角度 | 500us→沿小臂方向（θ4=0） | 用户确认 |
| 工作空间包围盒 | x[-201.46,186.12]，y[-77.10,201.46]，z[-48.28,102.55]，φ[-58.5,135.0] | 按新底座限位离线采样 |

### 9.6 函数与术语中英对照

> 看代码/文档时遇到不认识的英文名，先查这里。

#### 系统 / HAL（底层）

| 名称 | 中文解释 |
|---|---|
| `HAL_Init` | HAL 库初始化（最底层，最先调用） |
| `SystemClock_Config` | 系统时钟配置（72MHz） |
| `MX_GPIO_Init` | GPIO 引脚初始化 |
| `MX_TIM1_Init` | TIM1（PWM 定时器）初始化 |
| `MX_USART2_UART_Init` | USART2 串口初始化 |
| `HAL_GetTick` | 获取上电以来的毫秒数（SysTick 计时） |
| `HAL_UARTEx_ReceiveToIdle_IT` | 启动"空闲中断接收"（收到一批数据/总线空闲时触发） |
| `HAL_UARTEx_RxEventCallback` | 串口接收事件回调（收到数据时自动调用） |
| `HAL_TIM_PWM_Start` | 启动某路 PWM 输出 |

#### 应用层

| 名称 | 中文解释 |
|---|---|
| `App_Init` | 应用总初始化（依次调用各模块 Init） |
| `App_Run` | 应用主循环（`while(1)` 里跑） |
| `UI_Init` | OLED 界面初始化 |
| `UI_Task` | OLED 刷新任务（节流 50ms） |

#### 关节 / 参数（joint）

| 名称 | 中文解释 |
|---|---|
| `Joint_Init` | 关节参数初始化 |
| `Joint_UsToTheta` | 脉宽（us）→ 角度（°） |
| `Joint_ThetaToUs` | 角度（°）→ 脉宽（us），含限位夹紧 |
| `Joint_Clamp` | 把关节角夹到允许范围内 |
| `J_BASE` / `J_UPPER` / `J_FOREARM` / `J_END` | 关节枚举：底座 / 大臂 / 小臂 / 末端 |
| `J_NUM` | 关节总数（=4） |
| `J_*_THETA_MIN/MAX` | 各关节的角度下限 / 上限 |
| `J_*_PWM_MIN/MAX` | 各关节的脉宽下限 / 上限 |
| `ARM_A1_MM` | 底座偏航轴到肩轴的水平偏移（本机忽略） |
| `ARM_D1_MM` | 底座到肩轴的垂直高度（原点在肩，故为 0） |
| `ARM_L2_MM` / `ARM_L3_MM` / `ARM_L4_MM` | 大臂 / 小臂 / 末端长度（mm） |
| `WS_X/Y/Z_MIN/MAX` | 工作空间（Workspace）包围盒的 X/Y/Z 最小/最大值 |

#### 运动学（kinematics）

| 名称 | 中文解释 |
|---|---|
| `FK_Solve` | 正运动学（Forward Kinematics）：关节角 → 末端位姿 |
| `IK_Solve` | 逆运动学（Inverse Kinematics）：末端位姿 → 关节角 |
| `Pose` | 位姿结构体：`x,y,z` 位置 + `phi` 末端角 |
| `phi` / `φ` | 末端板的绝对角度 |

#### 运动平滑（motion）

| 名称 | 中文解释 |
|---|---|
| `Motion_Init` | 运动模块初始化 |
| `Motion_MoveTo` | 平滑移动到目标关节角（可指定时长） |
| `Motion_Stop` | 急停（目标 = 当前，立即停下） |
| `Motion_Task` | 插值任务（每 20ms 调一次） |
| `Motion_IsBusy` | 是否还在运动（1=运动中） |

#### 命令层（cmd）

| 名称 | 中文解释 |
|---|---|
| `Cmd_Submit` | 提交一条命令到队列 |
| `Cmd_Task` | 从队列取一条命令并执行 |
| `CMD_SET_POSE` | 命令：设置末端位姿 (x,y,z,φ) |
| `CMD_SET_JOINT` | 命令：设置单个关节角 |
| `CMD_HOME` | 命令：复位到 home |
| `CMD_STOP` | 命令：急停 |
| `CMD_QUERY` | 命令：查询当前状态 |

#### 协议 / 串口（proto / uart）

| 名称 | 中文解释 |
|---|---|
| `Proto_Init` | 协议初始化 |
| `Proto_Poll` | 从环形缓冲里找帧、校验、提交命令 |
| `Proto_SendAck` | 发送"成功"应答（0x81） |
| `Proto_SendErr` | 发送"错误"应答（0x82） |
| `Uart_Init` | 串口初始化并启动接收 |
| `Uart_Send` | 发送数据 |
| `LEN` | 帧长度（整帧字节数） |
| `CMD` | 命令码（Command） |
| `SUM` | 校验和（前 LEN-1 字节累加的低 8 位） |

#### 环形缓冲（ringbuf）

| 名称 | 中文解释 |
|---|---|
| `RingBuf_Init` | 初始化读写指针为 0 |
| `RingBuf_Write` | 写入数据，返回实际写入字节数 |
| `RingBuf_Peek` | 查看第 i 个待读字节（不移动读指针） |
| `RingBuf_Skip` | 跳过 n 个字节（移动读指针） |
| `RingBuf_GetLength` | 已用字节数 |
| `RingBuf_GetRemain` | 剩余空间 |
| `Command_GetCommand` | 参考工程里的"取一帧"函数（已迁到 `proto.c`） |

#### 按键 / 舵机 / 显示

| 名称 | 中文解释 |
|---|---|
| `Button_Init` | 按键初始化 |
| `Button_Scan` | 扫描按键并返回事件 |
| `BTN_NONE` / `BTN_PRESS` / `BTN_REPEAT` / `BTN_RELEASE` | 无事件 / 按下 / 长按连发 / 松开 |
| `Servo_Init` | 舵机初始化（写 home + 启动 PWM） |
| `Servo_SetPulse` | 设置某通道的脉宽（us） |
| `OLED_Init` | OLED 屏幕初始化 |

#### 常用缩写

| 缩写 | 全称 | 中文 |
|---|---|---|
| PWM | Pulse Width Modulation | 脉宽调制（本机 50Hz，500–2500us） |
| USART | Universal Synchronous/Asynchronous Receiver/Transmitter | 通用同步/异步串口 |
| ISR | Interrupt Service Routine | 中断服务函数 |
| HAL | Hardware Abstraction Layer | 硬件抽象层 |
| FK / IK | Forward / Inverse Kinematics | 正运动学 / 逆运动学 |
| PWM min/max | — | 脉宽最小/最大值（us） |
| home | — | 复位/初始姿态 |
| e-stop | Emergency Stop | 急停 |

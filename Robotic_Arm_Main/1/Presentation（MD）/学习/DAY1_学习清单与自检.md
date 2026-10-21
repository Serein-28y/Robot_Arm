# DAY1｜带你重读一遍代码

> 这不是学习计划，是**带读稿**：我们坐下来，按代码真实执行顺序，把 `User/` 和 `Core/Src/main.c` 从头读一遍。
> 每段前面有标记，告诉你这段该怎么读：
>
> - **【必要】**：贴真代码，逐行讲。今天要能**合上书复述**它保证什么、为什么这么写。
> - **【黑盒】**：不进去。只记三样——输入 → 输出 → 它保证什么，外加"什么情况下必须打破黑盒"。
> - **【必要·短】**：属于【必要】，但只有几行，讲清就过，别在这里耗时间。
>
> 每段末尾的 **接下去** 一行，说的是数据从哪个变量、以什么值，传给下一个函数——这是把整棵树串起来的关键。

---

## 0. 全景：这套代码一共有什么

### 0.1 文件表（手写 + 必须懂的生成代码）

| 文件 | 行数 | 层 | 职责 | 标记 |
|---|---|---|---|---|
| `Core/Src/main.c` | 353 | 入口 | `HAL_Init` → 时钟 → 三个外设 → `App_Init` → `while(1) App_Run()` | 2 必要 / 4 黑盒（`main` 见 §0.3） |
| `Core/Src/stm32f1xx_it.c` | 218 | 入口 | 中断向量：`SysTick_Handler`、`USART2_IRQHandler` | 全黑盒（但要知道它转给谁） |
| `Core/Src/stm32f1xx_hal_msp.c` | 234 | 入口 | 外设时钟使能、PA8–11/PA2–3 引脚复用、NVIC 使能 | 全黑盒 |
| `User/Src/app/app.c` | 326 | App | 初始化顺序、模式机、主循环调度、PWM 输出 | 10 必要 |
| `User/Src/app/ui.c` | 324 | App | OLED 菜单页 / HOST 页，增量刷新 | 1 必要 / 2 接口 / 6 黑盒 |
| `User/Src/control/cmd.c` | 323 | Control | 命令队列、MOVE/HOME/STOP/QUERY/SET_JOINT 执行 | 11 必要 |
| `User/Src/control/motion.c` | 155 | Control | 关节插值、缓动、急停 | 8 必要 |
| `User/Src/control/joint.c` | 190 | Control | 关节参数表、角度↔脉宽、软限位、通道映射 | 7 必要 |
| `User/Src/control/kinematics.c` | 247 | Control | FK / IK / 工作空间粗判 | 4 必要 / 2 黑盒 |
| `User/Src/services/proto.c` | 266 | Services | 帧解析、校验、命令分发、ACK/ERR 组包 | 9 必要 |
| `User/Src/services/ringbuf.c` | 86 | Services | 128 B SPSC 环形缓冲 | 6 必要 |
| `User/Src/services/sched.c` | 56 | Services | 基于 `HAL_GetTick` 的软定时 | 3 必要 |
| `User/Src/services/button.c` | 123 | Services | 四键去抖 / 短按 / 长按 | 3 必要 |
| `User/Src/drivers/uart_drv.c` | 60 | Drivers | RxToIdle 中断接收 + 阻塞发送 | 3 必要 |
| `User/Src/drivers/servo_drv.c` | 107 | Drivers | TIM1 四路 PWM 与通道级限幅 | 3 必要 |
| `User/Src/drivers/key_drv.c` | 52 | Drivers | 读 4 路 GPIO（低有效→1） | 1 必要 / 1 黑盒 |
| `User/Src/drivers/display/oled.c` | 1428 | Drivers | 第三方 OLED 库（软 I2C、显存、绘制） | 全黑盒（用 8 个 API） |
| `User/Src/drivers/display/oled_data.c` | 406 | Drivers | 字模表（第三方） | 黑盒 |
| `User/Src/diagnostics/test.c` | 341 | 诊断 | 手动标定页，`Test()` **无人调用**但被编译进固件 | 黑盒（见附录 B） |
| `Core/Src/system_stm32f1xx.c` 等 3 个 | 655 | 生成 | 启动时钟/系统调用，无需读 | 黑盒 |

**一句话**：真正要读懂的是 `main.c` + `User/Src` 里 16 个文件；`oled*` 是第三方库，`test.c` 是没接线的旧代码。

### 0.2 四层，只能单向依赖（图 1）

```text
App       app.c（初始化顺序 + 模式机 + 主循环调度）   ui.c（OLED 页面）
            │  只能往下调，下面不会反过来调上面
Control   cmd.c（命令→目标角）  motion.c（插值）  joint.c（角↔脉宽）
          kinematics.c（FK/IK）            ← 纯逻辑，不碰 HAL，可在 PC 上单测
            │
Services  proto.c（帧解析/应答）  ringbuf.c（SPSC 缓冲）
          sched.c（软定时）  button.c（去抖状态机）
            │
Drivers   uart_drv.c（RxToIdle 中断）  servo_drv.c（TIM1 PWM）  key_drv.c  oled.c
            │
Config    robot_params.h（唯一参数入口：连杆、限位、脉宽、节拍、协议常量）
```

两个必须记住的后果：

1. `cmd.c` **不调** `proto.c`（否则循环依赖）→ 所以 ACK 是 `app.c` 拿到 `Cmd_Task` 的结果后自己发的。这解释了"为什么 ACK 立刻发、不等运动结束"。
2. `motion.c / joint.c / kinematics.c` 里**没有** `HAL_GetTick`（只有 `sched.c` 用）→ 所以它们能在 PC 上跑单测，这也是 `Motion_Task` 按"步数"而不是按"时间"推进的原因。

### 0.3 全局函数调用树（图 2）——这张图就是今天要背下来的东西

```text
main()                                  Core/Src/main.c:70
 ├─ HAL_Init()                          黑盒：SysTick 1 ms
 ├─ SystemClock_Config()                黑盒：HSE×9 = 72 MHz
 ├─ MX_GPIO_Init()                      按键 PB12–15、OLED PB8/9
 ├─ MX_TIM1_Init()                      4 路 PWM 的载体（PSC=71, ARR=19999, CCR=0）
 ├─ MX_USART2_UART_Init()               115200 8N1
 ├─ App_Init()                          app.c:243
 │   ├─ RingBuf_Init()      ringbuf.c:27
 │   ├─ Proto_Init()        proto.c:59
 │   ├─ Joint_Init()        joint.c:70     → s_home = [112.5, 45, −90, 90]
 │   ├─ Motion_Init()       motion.c:42    → current = start = target = s_home
 │   ├─ Servo_Init()        servo_drv.c:57 → CCR = [1500, 500, 500, 1500]
 │   ├─ Uart_Init()         uart_drv.c:22
 │   ├─ Sched_Init()        sched.c:17
 │   ├─ Button_Init()       button.c:32
 │   ├─ Cmd_Init()          cmd.c:92       → s_pose = (−59.2, 142.9, 32.0, 45.0)
 │   └─ UI_Init()           ui.c:236
 └─ while (1) App_Run()     app.c:272
     ├─ Proto_Poll()       proto.c:80     每轮（HOST 模式）
     ├─ Cmd_Task()         cmd.c:190      每轮（HOST 模式）
     ├─ button_task()      app.c:154      10 ms
     ├─ Motion_Task()      motion.c:96    20 ms ┐ 同一分支内
     ├─ apply_servo()      app.c:102      20 ms ┘
     ├─ watchdog_task()    app.c:202      50 ms（仅 HOST）
     ├─ update_ui()        app.c:122      50 ms ┐
     └─ UI_Task()          ui.c:288       50 ms ┘

中断里（唯一进入点）：
USART2_IRQHandler()                    stm32f1xx_it.c:204
 └─ HAL_UART_IRQHandler(&huart2)       黑盒（HAL 的 UART 状态机）
     └─ HAL_UARTEx_RxEventCallback()   uart_drv.c:46   ← 我们唯一的中断代码
         ├─ RingBuf_Write()            ringbuf.c:49
         └─ HAL_UARTEx_ReceiveToIdle_IT()  黑盒
```

**这张图回答三个问题**：谁启动了一切（`main`）、谁在循环里被反复调用（`App_Run` 的八条分支）、中断在哪里劈开主流程（只有 `USART2_IRQHandler` 这一条线，而且它只碰 `ringbuf`）。

换成"时间视角"再看一次（图 3）：同一份代码，跑在三个上下文里。

```text
① 中断上下文   USART2 ISR：收字节 → ringbuf          （只搬，不解析）
      │
      ▼
② 主循环·命令侧 每轮：Proto_Poll → Cmd_Task → Motion_MoveTo → 立刻回 ACK
      │
      ▼
③ 主循环·输出侧 每 20 ms：Motion_Task → apply_servo → CCR
```

> 后面所有"谁写谁读""会不会冲突""急停为什么安全"的问题，都是在这三个上下文的视角下回答的。

### 0.4 标记统计（今天的时间怎么分配）

| 标记 | 数量 | 要求 |
|---|---|---|
| 【必要】 | ≈ 70 个函数 | 能复述：输入/输出、逐行作用、写错的现象 |
| 【黑盒·接口】 | 6 个 | 能说出契约与"什么时候必须打破黑盒" |
| 【黑盒】 | ≈ 60 个（第三方库 38 + 生成代码 8 + 旧诊断 14） | 知道它是什么、不进去 |

### 0.5 今天的起点（实测，别用旧数字）

| 项 | 实测 | 说明 |
|---|---|---|
| 构建 | `cmake --build build/Debug` → no work to do | 源码与 `Test3.elf`（10-19 20:54）一致 |
| FLASH | text 35852 + data 92 = **35944 B / 64 KB ≈ 54.8%** | `arm-none-eabi-size` |
| RAM | data 92 + bss 3644 = **3736 B / 20 KB ≈ 18.2%** | 同上 |
| 文档漂移 | `../架构/design_idea.md` 0.1 写 FLASH 35168 B / 53.66% | RAM 对得上；FLASH 少算 776 B |
| Git | 最新提交 `de5f57b`，大量文件已 add 未 commit | **做实验前先 commit 基线** |

---

## 1. 复位之后，先跑的不是我们的代码

### 1.1【黑盒】启动文件 + `HAL_Init`

- **保证什么**：把 `.data` 从 Flash 搬到 RAM、把 `.bss` 清零、设好栈顶、跳到 `main`；`HAL_Init()` 打开 **SysTick 1 ms** 和 Flash 预取/等待周期。
- **依赖它的事实**：所有 `HAL_GetTick()` 的 1 ms 分辨率、所有 `Sched_IsDue` 的节拍、所有 500 ms 超时，都建立在这一个黑盒上。
- **什么时候打破黑盒**：换主频、改 tick 源、或用 `HAL_InitTick()` 改成别的周期——否则终生不用进去。

### 1.2【黑盒】`SystemClock_Config`（`main.c:126`）

- **保证什么**：HSE 8 MHz × PLL9 = **72 MHz** 系统时钟；AHB = 72、APB1 = 36、APB2 = 72 MHz。
- **为什么我们只需要这一个数**：TIM1 挂在 APB2，所以定时器输入时钟 = 72 MHz——第 2.2 节把 `PSC` 设成 71 就是靠这个 72。
- **打破黑盒的信号**：你要改 PWM 频率或者串口波特率对不上时。

### 1.3【黑盒】`SysTick_Handler` → `HAL_IncTick`（`stm32f1xx_it.c:183`）

- **保证什么**：每 1 ms 给 HAL 的全局 tick 加一，`HAL_GetTick()` 才能返回毫秒时间。
- **依赖它的事实**：`Sched_Now()` / `Sched_IsDue` / 看门狗 / ACK 之间的时序，全部以它为时间基准。

### 1.4【黑盒】中断入口链：`USART2_IRQHandler` → `HAL_UART_IRQHandler` → 回调

```text
串口收到字节 → NVIC → USART2_IRQHandler()          stm32f1xx_it.c:204
                        └─ HAL_UART_IRQHandler(&huart2)
                             └─ （HAL 内部判断是 RX 还是 IDLE）
                                  └─ HAL_UARTEx_RxEventCallback(huart, Size)  uart_drv.c:46
```

- **保证什么**：字节到达 / 总线空闲两种事件最终都以 `(huart, Size)` 的形式交给我们。
- **必须知道的一个细节**：`HAL_UART_MspInit` 里 `HAL_NVIC_SetPriority(USART2_IRQn, 0, 0)` —— 串口中断是**最高优先级**，它随时能打断主循环。这就是第 7 节"并发只有两方"的来源。
- **打破黑盒的信号**：出现丢字节、Overrun 错误、或者要换 DMA 时。

---

## 2. 三个外设初始化：硬件从哪一刻开始存在

### 2.1【必要·短】`MX_GPIO_Init`（`main.c:286`）

```c
__HAL_RCC_GPIOD_CLK_ENABLE(); __HAL_RCC_GPIOA_CLK_ENABLE(); __HAL_RCC_GPIOB_CLK_ENABLE();

HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_SET);   /* OLED I2C 空闲电平 = 高 */

GPIO_InitStruct.Pin = KEY4_Pin|KEY3_Pin|KEY2_Pin|KEY1_Pin;      /* PB12..PB15 */
GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
GPIO_InitStruct.Pull = GPIO_PULLUP;
HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;                    /* OLED SCL/SDA */
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;                     /* 开漏，软 I2C 必须 */
HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
```

- 按键是**上拉输入**：不按 = 高电平 = 读回 0；按下 = 低电平 = 读回 1（`key_drv.c` 里已经取反）。
- OLED 两线是**开漏输出**：靠外部上拉产生高电平，这是软件模拟 I2C 的标准接法。
- **写错的现象**：把按键配成下拉 → 上电就"一直按着"；把 PB8/9 配成推挽 → I2C 线和从机抢电平，屏幕花屏或全黑。

**接下去**：GPIO 配好之后，`KeyDrv_Read`（第 3.8 节）才有意义；`oled.c` 的软 I2C 才能拉线。

### 2.2【必要】`MX_TIM1_Init`（`main.c:166`）——四个数决定了后面所有脉宽

```c
htim1.Instance = TIM1;
htim1.Init.Prescaler = 72-1;        /* PSC */
htim1.Init.Period    = 20000-1;     /* ARR */
htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
HAL_TIM_Base_Init(&htim1);
HAL_TIM_PWM_Init(&htim1);
sConfigOC.OCMode = TIM_OCMODE_PWM1;
sConfigOC.Pulse  = 0;               /* ★ 四路 CCR 初值 = 0 */
HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1..4);
HAL_TIM_MspPostInit(&htim1);        /* PA8–PA11 配成复用推挽 */
```

三个换算，必须自己推一遍：

```text
1 tick = (PSC+1) / 72 MHz = 72 / 72 MHz = 1 μs      → CCR 的数值 = 脉宽微秒数
周期   = (ARR+1) × 1 μs = 20000 μs = 20 ms         → 50 Hz
占空比 = CCR / 20000                               → 写 1500 = 1500 μs 高电平（7.5%）
```

- **为什么要记 `Pulse = 0`**：这就是第 3.5 节"必须先写 CCR 再 `PWM_Start`"的全部理由。若先启动，第一个 20 ms 周期输出 0 μs 高电平，对 ZP25D 是无效脉宽（< 500 μs）→ 舵机不动、抖动或冲向极限。
- **写错的现象**：`PSC` 记成 720-1 → 周期变 200 ms（5 Hz），舵机全部抽筋；`ARR` 记成 2000-1 → 周期 2 ms（500 Hz），舵机不认。

**接下去**：`htim1` 这个句柄被 `Servo_Init` / `Servo_SetPulse` 使用；`PSC=71` 让 `robot_params.h` 里那些 500–2500 的"微秒"能直接当 CCR 写。

### 2.3【黑盒】`MX_USART2_UART_Init`（`main.c:253`）

- **保证什么**：USART2 配成 115200 8N1、收发都开；`HAL_UART_MspInit` 把 PA2 配 TX（复用推挽）、PA3 配 RX（浮空输入），并使能 `USART2_IRQn`。
- **我们依赖的两个数**：波特率 115200（1 字节 ≈ 87 μs，12 字节 ≈ 1.04 ms）、引脚 PA2/PA3（上位机接线必须交叉）。
- **打破黑盒的信号**：C3 一发命令就没回帧，先怀疑波特率和 PA2/PA3。

---

## 3. `App_Init`：十步，每一步给后面铺了什么

`App_Init` 只有 10 行调用，但每一行都在给后面的某个环节准备数据。下面按执行顺序，一步一个单元。
读法：先看 **改了什么变量（带值）**，再看 **给谁用**，最后看 **删掉会怎样**——这三样就是"理解初始化"的全部。

### 3.1【必要】第 1 步 `RingBuf_Init`（`ringbuf.c:27`）

```c
void RingBuf_Init(void)
{
    readIndex  = 0;      /* 消费者下标（主循环写） */
    writeIndex = 0;      /* 生产者下标（中断写） */
}
```

- **改了什么**：只有两个 `volatile uint8_t` 下标归零。128 B 的 `buffer[]` **不清**——因为"哪些字节有效"只由这两个下标决定。
- **给谁用**：`Uart_Init` 之后的接收中断往这里写；`Proto_Poll` 从这里读。
- **删掉会怎样**：现在恰好等于 `.bss` 清零，所以"删掉也能跑"——那是巧合。热复位、保留 RAM 调试、或以后把 buffer 放进别的段时，就会读到旧索引。
- **自检**：为什么 128 字节的有效容量是 127？（第 7.3 节）

**接下去**：`writeIndex` 只有中断写、`readIndex` 只有主循环写——这个"单生产者单消费者"约定贯穿第 5、7 节。

### 3.2【必要】第 2 步 `Proto_Init`（`proto.c:59`）

```c
static uint32_t s_last_valid;   /* 最近一次"整帧校验通过"的 tick */
void Proto_Init(void) { s_last_valid = 0; }
```

- **改了什么**：把"最近有效帧时间"标成 0，语义是**从没收到过有效帧**。
- **给谁用**：`watchdog_task` 用 `last_valid != s_prev_rx_tick` 判断"有没有新帧"（第 4.5 节）；`App_Init` 末尾 `s_prev_rx_tick = Proto_GetLastValidTick()` 拿到这个 0。
- **删掉会怎样**：靠 `.bss` 也是 0，功能不变；但"没收到过"和"收到过、时间戳恰好是 0"从此分不出来，看门狗第一次判断会变得不可解释。
- **自检**：为什么看门狗不能靠"ringbuf 里有没有字节"来判断上位机活着？（第 4.5 节）

### 3.3【必要】第 3 步 `Joint_Init`（`joint.c:70`）—— home 角在这一步被算出来

```c
static float s_home[J_NUM];
static uint8_t s_ready;

void Joint_Init(void)
{
    for (uint8_t i = 0; i < J_NUM; i++)
        s_home[i] = Joint_UsToTheta((JointId)i, joint_cfg[i].home_us);
    s_ready = 1;
}
```

代入 `θ = theta_at_pwm_min + dir × (us − pwm_min) / us_per_deg`：

| 关节 | `home_us` | 代入 | `s_home` |
|---|---|---|---|
| BASE | 1500 | 202.5 + (−1)(1500−500) ÷ (2000/180) | 202.5 − 90 = **112.5°** |
| UPPER | 500 | 45 + (−1)(500−500) ÷ (2000/90) | **45°** |
| FOREARM | 500 | −90 + (+1)(500−500) ÷ (1/0.09) | **−90°** |
| END | 1500 | 0 + (+1)(1500−500) × 0.09 | **90°** |

- **改了什么**：`s_home = [112.5, 45, −90, 90]`，`s_ready = 1`。
- **给谁用**：`Motion_Init`（当下一个 current）、`Cmd_Home`（KEY3 复位）、`Cmd_Task` 的 HOME 分支。
- **删掉会怎样**：不会崩——`Joint_GetHome` 里有惰性兜底（`if (!s_ready) Joint_Init();`）。但"home 在哪一步算出来"就不可控了，可能拖到 20 ms 任务里才算。
- **一个关键细节**：`Joint_UsToTheta` **只夹脉宽、不夹角度**（`joint.c:98-108`）。所以把 `J_BASE_DIR` 改成 `+1`，`s_home` 会算出 292.5°（超限也不报错）——这就是 B1 实验能引发上电跳变的原因。

**接下去**：`s_home` 是第 3.4 节 `Motion_Init` 的唯一输入；也是第 6.10 节"两条 home 路径"里逻辑层那一条。

### 3.4【必要】第 4 步 `Motion_Init`（`motion.c:42`）

```c
void Motion_Init(void)
{
    Joint_GetHome(s_current);       /* ← 读第 3 步算出来的 s_home */
    copy_joints(s_start,  s_current);
    copy_joints(s_target, s_current);
    s_steps_total = 0;
    s_step        = 0;
    s_busy        = 0;
}
```

- **改了什么**：`current = start = target = [112.5, 45, −90, 90]`；`busy = 0`。
- **给谁用**：`apply_servo` 读的 `current`；`Cmd_Init` 做 FK 的输入；`calc_duration` 的 `cur`。
- **删掉会怎样（具体推演）**：`current` 全是 0 → 第一次 `apply_servo` 把 0° 夹到各自限位边缘 → 写出 **BASE 2500 / UPPER 800 / FOREARM 1000 / END 500 μs** → 上电 20 ms 后四路一起猛动。
- **"上电不动"的原理**：`busy = 0` → `Motion_Task` 第一句 `if (!s_busy) return;` 直接返回；`apply_servo` 每 20 ms 仍跑，但算出的 `us` 与 `s_pulse` 相等 → 不写 CCR。**两个条件缺一不可**：`current` 对（这步）+ `s_pulse` 对（下一步）。

**接下去**：`current` 是第 6 节整条下行链的起点；`busy` 是第 4.5 节看门狗"运动中续命"的依据。

### 3.5【必要】第 5 步 `Servo_Init`（`servo_drv.c:57`）

```c
void Servo_Init(void)
{
    for (uint8_t i = 0; i < SERVO_NUM; i++) {
        s_pulse[i] = s_home_us[i];                                  /* [1500,500,500,1500] */
        __HAL_TIM_SET_COMPARE(&htim1, s_tim_channel[i], s_pulse[i]);/* 先写 CCR */
    }
    for (uint8_t i = 0; i < SERVO_NUM; i++)
        HAL_TIM_PWM_Start(&htim1, s_tim_channel[i]);                /* 再开输出 */
}
```

- **改了什么**：`s_pulse = [1500, 500, 500, 1500]`（CH1=末端、CH2=小臂、CH3=大臂、CH4=底座），随后四路 CCR 同步写成同样的值，最后才启动 PWM。
- **给谁用**：第一个 PWM 周期就是 home；`s_pulse` 同时是 `apply_servo`"没变不写"的比较基准。
- **为什么必须这个顺序**：`MX_TIM1_Init` 把 `Pulse` 设成 0（第 2.2 节）。先 Start 就先输出 0 μs——舵机会猛跳。
- **删掉会怎样**：四路无输出（舵机全软）；且 `s_pulse` 全 0 会让 `apply_servo` 每 20 ms 都写一次 CCR。

**接下去**：`s_pulse` 在第 6.8 节被 `Servo_SetPulse` 更新、被 `Servo_GetPulse` 读出比较。

### 3.6【必要】第 6 步 `Uart_Init`（`uart_drv.c:22`）

```c
void Uart_Init(void)
{
    HAL_UARTEx_ReceiveToIdle_IT(&huart2, s_rx_buf, sizeof(s_rx_buf));  /* 64 字节 */
}
```

- **改了什么**：打开 RX 中断 + 空闲（IDLE）中断，把接收缓冲指向 `s_rx_buf[64]`。
- **给谁用**：`Proto_Poll` 的唯一数据来源（字节先由中断搬进 ringbuf）。
- **删掉会怎样**：HOST 模式永远收不到字节 → 发什么都不回（C3 的失败现象）。
- **自检**：`HAL_UARTEx_ReceiveToIdle_IT` 是一次性的，为什么必须在回调里重新调用一次？（第 5.1 节）

### 3.7【必要·短】第 7 步 `Sched_Init`（`sched.c:17`）

```c
static uint32_t s_last[SCHED_NUM];
void Sched_Init(void)
{
    for (uint8_t i = 0; i < SCHED_NUM; i++) s_last[i] = HAL_GetTick();
}
```

- **改了什么**：四个任务的"上次执行时间"全部设成**现在**。
- **给谁用**：`Sched_IsDue`（第 4.2 节）的差值基准。
- **删掉会怎样**：`s_last` 全 0 → 上电第一圈 `now − 0 ≥ 10/20/50` 全部成立 → 四个任务同时触发一次（多刷一次屏、多扫一次键）。不坏，但相位乱。
- **顺带解释一个现象**：因为 `App_Init` 里 `Sched_Init` 排在硬件初始化之后，所以第一个 10 ms/20 ms 任务是从初始化结束那一刻才开始计时的。

### 3.8【必要】第 8 步 `Button_Init`（`button.c:32`）

```c
void Button_Init(uint32_t now_ms)
{
    KeyDrv_Init();                                  /* 黑盒：空函数，GPIO 已由 MX_GPIO_Init 配好 */
    for (uint8_t i = 0; i < KEY_NUM; i++) {
        uint8_t raw = KeyDrv_Read(i);               /* 1 = 按下 */
        s_button[i].last_raw   = raw;
        s_button[i].stable     = raw;               /* ← 以当前状态作为起点 */
        s_button[i].repeat_en  = 0;                 /* 本应用不用连发 */
        s_button[i].t_change   = now_ms;
        s_button[i].t_down     = now_ms;
        s_button[i].t_repeat   = now_ms;
    }
}
```

- **改了什么**：把四个键**此刻的原始电平**同时写进 `last_raw` 和 `stable`。
- **给谁用**：`Button_Scan` 的去抖状态机（第 4.3 节）。
- **删掉会怎样**：`stable` 全 0；若上电时 KEY2 正好被按住（或读成低电平），第一次扫描就会产生一次 `BTN_PRESS` → **无意识进入 HOST**。
- **自检**：`KeyDrv_Read` 返回 1 表示按下还是松开？在哪一行取的反？

### 3.9【必要】第 9 步 `Cmd_Init`（`cmd.c:92`）——用 FK 手算验证它跑过了

```c
void Cmd_Init(void)
{
    s_head = 0; s_tail = 0; s_count = 0; s_err = CMD_ERR_NONE;
    Motion_GetCurrent(cur);      /* ← 读第 4 步的 current */
    FK_Solve(cur, &s_pose);      /* ← 算出 UI 要显示的初始位姿 */
}
```

`FK_Solve([112.5, 45, −90, 90])`：θ2 + θ3 = −45°，θ2 + θ3 + θ4 = 45°，

```text
r = 100.47·cos45 + 86.68·cos(−45) + 31.51·cos45 = 0.7071 × 218.66 = 154.6 mm
z = 100.47·sin45 + 86.68·sin(−45) + 31.51·sin45 = 0.7071 × 45.30  =  32.0 mm
x = r·cos(112.5°) = 154.6 × (−0.3827) = −59.2
y = r·sin(112.5°) = 154.6 × ( 0.9239) = 142.9
phi = 45 + (−90) + 90 = 45.0°
```

- **改了什么**：`s_pose = (−59.2, 142.9, 32.0, 45.0)`；命令队列清空。
- **给谁用**：UI 显示的初始位姿、`Cmd_GetStatus` 的返回值。
- **删掉会怎样**：`s_pose` 全 0，菜单页显示 `(0.0, 0.0, 0.0, 0.0)`。
- **上板验证**：上电后菜单页位姿应接近这组数；QUERY 的 ACK 四角度应是 `65 04 C2 01 7C FC 84 03`（112.5 / 45 / −90 / 90）。两样都对 = 第 3/4/5/9 步都跑对了。

### 3.10【黑盒·接口】第 10 步 `UI_Init`（`ui.c:236`）

- **保证什么**：`OLED_Init()` 初始化屏幕；`s_state` 清零；`s_prev.status = 0xFF` 强制首帧重画；`s_page = MENU`；`draw_menu()` 画出菜单页。
- **我们依赖的契约**：之后每 50 ms 调一次 `UI_Task()`，它只在 `s_dirty` 为真时刷新；页面切换才整屏发送。
- **打破黑盒的信号**：屏幕花屏、整屏刷新太慢拖住主循环（那是 `oled.c` 的软 I2C 时序问题）。

### 3.11 十步之间的依赖：三条硬、三条软、一条能换（图 4）

```text
硬依赖（换了一定坏）
 ① Joint_Init → Motion_Init        s_home 是 current 的输入
 ② Motion_Init → Cmd_Init          Cmd_Init 要 Motion_GetCurrent 才能 FK
 ③ CubeMX → 各驱动 Init            MX_TIM1_Init → Servo_Init（要用 htim1）
                                   MX_USART2_UART_Init → Uart_Init（要用 huart2）
                                   MX_GPIO_Init → Button_Init（要用 PB12–15）

软约束（顺序错了不崩，但会把问题藏起来）
 ④ RingBuf_Init 在 Uart_Init 之前    中断可能在 Uart_Init 返回后立刻进来
 ⑤ Proto_Init 在 App_Init 最后一行之前  s_prev_rx_tick = Proto_GetLastValidTick()
 ⑥ Sched_Init 在第一次 App_Run 之前   否则第一圈四个任务全部到期

能换但别换
 ⑦ Motion_Init ↔ Servo_Init         今天互换也能跑（都只依赖 s_home，各写各的数组）
                                    但换过来会让人以为"PWM 先动、Motion 后知道"是允许的，
                                    恰好掩盖了"两条 home 路径必须同源"（第 6.10 节）
```

**记成四组更好记**：

```text
① 通信地基   RingBuf_Init → Proto_Init
② 控制数据   Joint_Init  → Motion_Init
③ 硬件与输入 Servo_Init  → Uart_Init → Sched_Init → Button_Init
④ 上层       Cmd_Init    → UI_Init
```

一句话：**顺序的本质不是"谁先谁后好看"，而是"谁给谁提供数据"**。以后加模块也按这四组找位置：动硬件/开中断的放 ③，读 control 层数据的放 ④。

---

## 4. `App_Run`：主循环里谁被调用、按什么节奏

### 4.1【必要】`App_Run`（`app.c:272`）

```c
void App_Run(void)
{
    now_ms = Sched_Now();                    /* ① 本圈时间戳，只取一次 */

    if (s_mode == APP_MODE_HOST) {           /* ② HOST：命令通路 */
        Proto_Poll(now_ms);                  /*    解析 ringbuf，可能入队 0~N 条 */
        if (Cmd_Task(&result)) {             /*    只取 1 条执行 */
            if (result.err == CMD_ERR_NONE) { Proto_SendAck(result.theta); s_estop = 0; }
            else                            { Proto_SendErr(result.err); }
        }
    } else {                                 /* ③ MENU：串口全丢 */
        RingBuf_Skip(RingBuf_GetLength());
        Cmd_Clear();
    }

    if (Sched_IsDue(SCHED_BUTTON, BUTTON_SCAN_MS)) { button_task(Sched_Now()); }      /* 10 ms */
    if (Sched_IsDue(SCHED_MOTION, MOTION_STEP_MS)) { Motion_Task(); apply_servo(); }  /* 20 ms */
    if (Sched_IsDue(SCHED_WDG,    WDG_PERIOD_MS))  { watchdog_task(Sched_Now()); }    /* 50 ms */
    if (Sched_IsDue(SCHED_UI,     UI_PERIOD_MS))   { update_ui(); UI_Task(); }        /* 50 ms */
}
```

- **①** `now_ms` 只喂给 `Proto_Poll`（写进 `s_last_valid`）。`button_task` / `watchdog_task` 内部又各自 `Sched_Now()` 取一次——同一圈里"现在"可能差几十微秒，对 500 ms 级判断无影响，但**不要以为它是同一个时间戳**。
- **②** `Proto_Poll` 一次可能把积压的完整帧全部入队，而 `Cmd_Task` 每圈只消化一条 → 连发 3 条命令会分布在 3 圈里执行（A13）。
- **③** MENU 分支**每圈都执行**，所以不按 KEY2 直接发命令**一定没反应**（数据被读走扔掉）。C3 必须先短按 KEY2。
- **④** 四个 `Sched_IsDue` 顺序固定：按键 → 运动 → 看门狗 → UI。UI 最慢（整屏刷新 ms 级），放最后，推迟不了别人。
- **⑤** `s_estop = 0` 只在"命令成功"时清 → KEY4 之后屏上的 `STOP` 会一直挂着，直到下一条成功命令。

**接下去**：`Proto_Poll` 的产出是队列，`Cmd_Task` 的产出是 `result.theta`（四角度），`apply_servo` 的产出是 CCR。三条产出分别在第 5、6 节展开。

### 4.2【必要·短】`Sched_IsDue`（`sched.c:38`）

```c
now = HAL_GetTick();
if ((uint32_t)(now - s_last[id]) >= period_ms) {
    s_last[id] = now;      /* ← 关键：= now，不是 += period */
    return 1;
}
return 0;
```

- **`= now` 的代价，量化一下**：主循环正常一圈 1–2 ms，但 OLED 整屏刷新那圈可能 30–40 ms。假设运动期间平均 25 ms/圈：20 ms 的 `SCHED_MOTION` **每圈都到期** → 每圈走一步 → 800 ms 的 home 复位 = 40 步 × 25 ms = **实际约 1000 ms**。
- **结论**：步数不变、总时长变长。看到的是"变慢"，不是"丢步/跳变"。这就是 A9。
- **无符号减法的意义**：`HAL_GetTick` 是 32 位毫秒，约 49.7 天回绕；`(uint32_t)(now - s_last)` 在回绕点附近依然得到正确差值。所以回绕**不用你处理**，但你要能说出为什么。
- **`+= period` 为什么不行**：慢了 30 ms 就要在下一圈补跑两次，越补越慢，雪崩。

### 4.3【必要】`button_task`（`app.c:154`）

```c
(void)Button_Scan(KEY_ID_1, now_ms);                 /* 预留，不用 */

event = Button_Scan(KEY_ID_2, now_ms);
if (event == BTN_PRESS && s_mode == APP_MODE_MENU) { enter_host(now_ms); s_key2_ignore_hold = 1; }
if (event == BTN_RELEASE)                          { s_key2_ignore_hold = 0; }
if (s_mode == APP_MODE_HOST && !s_key2_ignore_hold && Button_Held(KEY_ID_2, now_ms, BUTTON_HOLD_MS))
                                                   { exit_host(); }

if (Button_Scan(KEY_ID_3, now_ms) == BTN_PRESS)    { do_home(); }
if (Button_Scan(KEY_ID_4, now_ms) == BTN_PRESS)    { do_estop(); }
```

| 键 | 生效条件 | 调什么 | 具体改了哪些状态 |
|---|---|---|---|
| KEY1 | 只扫描 | — | 无（模式选择预留） |
| KEY2 | MENU 下 `BTN_PRESS` | `enter_host(now)` | `s_mode=HOST`；`Cmd_Clear()`；`Cmd_ReportError(0)`；`Proto_ResetWatchdog(now)`；`s_prev_rx_tick=now`；`s_last_activity=now`；清 `TO`/`STOP`；`s_key2_ignore_hold=1` |
| KEY2 | `BTN_RELEASE` | — | `s_key2_ignore_hold = 0` |
| KEY2 | HOST 下、未被屏蔽、稳定按住 ≥ 1000 ms | `exit_host()` | 先 `Cmd_StopNow()`，再 `s_mode=MENU`、清 `TO`/`STOP` |
| KEY3 | `BTN_PRESS`（两种模式都有效） | `do_home()` | `Cmd_Home()`：清队列 + `Motion_MoveTo(home, 800)` + `FK_Solve → s_pose`；清 `TO`/`STOP` |
| KEY4 | `BTN_PRESS`（两种模式都有效） | `do_estop()` | `Cmd_StopNow()`（清队列 + `Motion_Stop`）+ `s_estop=1` |

`enter_host` 里三个顺序不能乱：先 `Cmd_Clear()` 清掉进 HOST 之前的陈旧命令；再 `Proto_ResetWatchdog(now)` 把 `s_last_valid` 设成 now；最后 `s_prev_rx_tick = Proto_GetLastValidTick()` **与它对齐**。不对齐的话，下一次看门狗会把"复位"误判成"刚收到新帧"。

`do_home` / `do_estop` 都只是两三行：把"清队列 + 给 Motion 一条命令 + 更新 UI 位姿"打包。

**模式机（图 5）**：

```text
上电 ──► MENU（串口数据每圈被丢弃：RingBuf_Skip + Cmd_Clear）
           │ KEY2 短按 → enter_host：清队列、清错误、重挂看门狗、屏蔽长按
           ▼
         HOST（Proto_Poll → Cmd_Task → ACK/ERR）
           │ KEY2 按住 1 s 且已松开过一次 → exit_host：先急停再回 MENU
           ▼
         MENU
     任何时候：KEY3 → 回 home（800 ms 平滑）；KEY4 → 急停（屏显 STOP）
```

**接下去**：`exit_host` / `do_estop` 都落到 `Cmd_StopNow`（第 6 节），`do_home` 落到 `Motion_MoveTo`。

### 4.4【必要】`Motion_Task` + `apply_servo` 的轮廓（先用眼睛看一遍，第 6 节逐行）

```c
if (Sched_IsDue(SCHED_MOTION, MOTION_STEP_MS)) { Motion_Task(); apply_servo(); }
```

- `Motion_Task()`：推进插值一步，改 `current`。不忙时第一句就返回。
- `apply_servo()`：读 `current` → 换脉宽 → 变了才写 CCR。
- **这一对是"每 20 ms 的心脏"**：一个决定"现在应该在哪个角度"，另一个决定"这个角度对应多少微秒"。
- **写错的现象**：把 `apply_servo` 改成读 `target`，20 ms 后直接一步到位，`motion.c` 等于不存在。

### 4.5【必要】`watchdog_task`（`app.c:202`）

```c
if (s_mode != APP_MODE_HOST) return;             /* ① 只在 HOST 管 */
last_valid = Proto_GetLastValidTick();
if (last_valid != s_prev_rx_tick) {              /* ② 有新有效帧 */
    s_prev_rx_tick = last_valid; s_last_activity = now_ms; s_timeout_latched = 0;
}
if (Motion_IsBusy()) s_last_activity = now_ms;   /* ③ 运动期间续命 */
if (!s_timeout_latched && (uint32_t)(now_ms - s_last_activity) > UART_TIMEOUT_MS) {
    Cmd_StopNow(); s_timeout_latched = 1;        /* ④ 超时急停 + 锁存 */
}
```

- **② 为什么用"时间戳变了"而不是"有没有新字节"**：`s_last_valid` 只在**整帧通过校验**时更新（第 5.3 节第 ⑥ 步）。噪声、半帧、坏校验都不算"上位机还活着"。
- **③ 运动中续命**：一条 2000 ms 的 MOVE 远超 500 ms，不续命会被自己的看门狗杀掉。这是把"命令级超时"与"运动时长"解耦。
- **④ 锁存**：50 ms 周期内会连续命中十几次；锁存让急停只发生一次、`TO` 不闪。
- **确切时间**：进 HOST 时 `s_last_activity = now`，此后每 50 ms 检查一次，判断是**严格大于** 500 → 第一次命中在 **now + 550 ms** 左右。所以 C6 看到的是 500–550 ms。
- **边界（知道即可）**：两帧落在同一毫秒内时第二次不刷新 → 只影响 1 ms 窗口。

### 4.6【黑盒·接口】`update_ui` + `UI_Task`

- **保证什么**：`update_ui` 把 `Cmd_GetStatus`（位姿/当前角/moving/err）+ 模式/超时/急停打包成 `UiState`，`UI_SetState` 只拷贝并置脏；`UI_Task` 在有脏标记时局部刷新，**页面切换才整屏发送**。
- **我们依赖的契约**：50 ms 一次；同页只更新数字区域；整屏刷新是 ms 级的。
- **打破黑盒的信号**：页面切换时明显卡顿（`oled.c` 的 `OLED_Update` 发了 1024 字节）。

### 4.7 谁卡住谁（图 6）

```text
OLED 整屏刷新（UI_Task，页面切换才发生）
   │ 软件 I2C 发 1024 B，ms 级（未实测）
   ▼
Uart_Send → HAL_UART_Transmit 阻塞：12 B @115200 ≈ 1.0 ms（超时上限 100 ms）
   ▼
Proto_Poll 一次把积压帧全部入队 → Cmd_Task 每轮只消化一条（要几轮才清空）
   ▼
本轮 20 ms 的 Motion_Task 被推迟 → 步间隔变长
   ▼
步数不变、总时长变长：不补跑、不丢步、不跳变
```

---

## 5. 命令上行：一个字节走进来，怎么变成一条 `Command`

### 5.1【必要】`HAL_UARTEx_RxEventCallback`（`uart_drv.c:46`）—— 我们唯一的中断代码

```c
static uint8_t s_rx_buf[UART_RX_CHUNK];      /* 64 */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance != USART2) return;                  /* 只认 USART2 */
    if (Size > 0) RingBuf_Write(s_rx_buf, (uint8_t)Size);   /* ① 只搬字节 */
    HAL_UARTEx_ReceiveToIdle_IT(huart, s_rx_buf, sizeof(s_rx_buf)); /* ② 立刻重挂 */
}
```

- **输入**：`Size` = 本次收（或空闲）到的字节数；`s_rx_buf` 是 HAL 已经填好的 64 字节缓冲。
- **① 只搬字节**：不解析、不算校验、不碰浮点。中断里时间越短越好，协议逻辑留给主循环。
- **② 必须重挂**：`ReceiveToIdle_IT` 是**一次性**的，进回调就结束了。不重挂 → 第一批之后再也收不到（C3 无回帧的典型原因）。
- **返回值被忽略**：ringbuf 满时 `RingBuf_Write` 返回 0，这一批**整批丢弃**。设计上"丢新不覆盖旧"，避免把已缓存的完整帧截断（A4）。
- **写错的现象**：把解析写进中断 → 主循环被频繁打断，OLED 刷新和 UART 发送互相拖；漏掉重挂 → 只能收到上电后第一串字节。

**接下去**：字节在 `ringbuf` 里；主循环下一次 `Proto_Poll` 才去读。

### 5.2【必要】`RingBuf_Write`（`ringbuf.c:49`）—— 环形缓冲只有两条规则

```c
uint8_t RingBuf_Write(const uint8_t *data, uint8_t length)
{
    uint8_t firstLength;
    if (length == 0 || length > RingBuf_GetRemain()) return 0;   /* 规则一：装不下就整批拒绝 */

    if (writeIndex + length < RINGBUF_SIZE) {
        memcpy(buffer + writeIndex, data, length);
        writeIndex = (uint8_t)((writeIndex + length) % RINGBUF_SIZE);
    } else {
        firstLength = (uint8_t)(RINGBUF_SIZE - writeIndex);       /* 规则二：跨尾部就分两段 */
        memcpy(buffer + writeIndex, data, firstLength);
        memcpy(buffer, data + firstLength, length - firstLength);
        writeIndex = (uint8_t)((length - firstLength) % RINGBUF_SIZE);
    }
    return length;
}
```

- **规则一**：先看剩余空间，不够就返回 0、一个字节都不写。调用者（中断）忽略返回值 → 整批丢。
- **规则二**：`writeIndex + length == RINGBUF_SIZE` 这种边界走 else 分支，第一段正好写完、第二段长度 0，结果仍然正确。
- **`writeIndex` 只有中断写**：这是 SPSC 成立的一半（第 7 节）。

四个读接口（都在主循环里用，全在 `ringbuf.c`）：

| 函数 | 做什么 | 谁调 |
|---|---|---|
| `RingBuf_GetLength()` | `(writeIndex + SIZE − readIndex) % SIZE`，已用字节数 | `Proto_Poll` 的循环条件、`App_Run` 的 MENU 分支 |
| `RingBuf_GetRemain()` | `(SIZE−1) − length`，**故意少 1 字节**（区分空/满） | `RingBuf_Write` |
| `RingBuf_Peek(i)` | 看不移动指针，第 i 个待读字节 | `Proto_Poll` 找帧头/取字段 |
| `RingBuf_Skip(n)` | 移动 `readIndex` | `Proto_Poll` 的每一步、`App_Run` 的 MENU 分支 |

**接下去**：`Proto_Poll` 用 `GetLength` 判够不够、`Peek` 读字段、`Skip` 丢弃；三个动作构成第 5.3 节的六步。

### 5.3【必要】`Proto_Poll`（`proto.c:80`）—— 六步状态机

```c
while (RingBuf_GetLength() >= 4)                     /* 4 = 最短合法帧长 */
{
    if (RingBuf_Peek(0) != 0xAA) { RingBuf_Skip(1); continue; }

    len = RingBuf_Peek(1);
    if (len < 4 || len > PROTO_MAX_LEN) { RingBuf_Skip(1); continue; }

    if (RingBuf_GetLength() < len) break;            /* 半帧，等下一次 */

    for (i = 0; i < len; i++) frame[i] = RingBuf_Peek(i);

    if (frame_sum(frame, len) != frame[len - 1]) {
        send_error(CMD_ERR_CHECKSUM); RingBuf_Skip(1); continue;   /* 坏校验：只丢 1 字节 */
    }

    cmd = frame[2];
    /* 0x01 MOVE / 0x02 HOME / 0x03 STOP / 0x04 QUERY / 0x05 SET_JOINT */
    /* 已知命令但 LEN 不匹配 → send_error(PARAM) + Skip(len) */
    /* 未知命令          → send_error(UNKNOWN) + Skip(len)        */

    s_last_valid = now_ms;        /* 只有走到这里才算"上位机活着" */
    Cmd_Submit(&command);
    RingBuf_Skip(len);
}
```

六步，每步的"为什么这么丢"：

| 步 | 判断 | 失败动作 | 为什么 |
|---|---|---|---|
| ① | `Peek(0) != 0xAA` | `Skip(1)` | 0xAA 可能只是数据里的一个字节；丢 1 字节滑动重找，比丢整帧更不容易跳过真帧头 |
| ② | `len` 不在 4–16 | `Skip(1)` | 连 LEN 都不可信，不能按它跳 |
| ③ | `GetLength() < len` | `break` | 一帧可能分两批到达；半帧必须原地等，不能丢 |
| ④ | 校验和不等 | 回 `0x82/02` + `Skip(1)` | 同上：这个 0xAA 可能根本不是帧头 |
| ⑤ | 命令不认识 / LEN 不匹配 | 回 `0x82/03`/`04` + `Skip(len)` | 此时 LEN 可信、校验也过了，整帧确定是垃圾 |
| ⑥ | 全过 | 刷 `s_last_valid` → `Cmd_Submit` → `Skip(len)` | 只有完整合法的帧才算"上位机活动" |

两个副作用要记住：

- **`send_error` 里会调 `Uart_Send`（阻塞约 1 ms）**。一串噪声可能连续触发多次，把主循环拖慢——这是"简单可靠协议"的代价。
- **`Skip(1)` 会导致"连续回多个 0x82/02"**：流里若有多个"0xAA + 合法 LEN"的假帧头，每个都会失败一次（A5）。

**接下去**：`Cmd_Submit` 入队（第 5.5 节）；主循环下一轮 `Cmd_Task` 取一条执行（第 6 节）。

### 5.4【必要·短】三个辅助函数（`proto.c:32 / 38 / 52`）

```c
static int16_t read_le16(uint8_t lo, uint8_t hi)          /* 小端还原 */
{ return (int16_t)((uint16_t)lo | ((uint16_t)hi << 8)); }

static uint8_t frame_sum(const uint8_t *frame, uint8_t len)  /* 前 len−1 字节累加低 8 位 */
{ uint8_t s = 0; for (uint8_t i = 0; i < len - 1; i++) s += frame[i]; return s; }

static void send_error(uint8_t code)                      /* 一处出口：UI + 串口 */
{ Cmd_ReportError(code); Proto_SendErr(code); }
```

- `read_le16`：MOVE 的 x/y/z/phi 都是 int16 小端，`C5 FF` → `0xFFC5` = −59。
- `frame_sum`：**不是** CRC，就是累加和低 8 位。选它是因为上位机用 Python 也好算；代价是比 CRC 弱（错两个字节可能撞上）。
- `send_error`：把"记到 UI"和"回 0x82"绑在一处，避免两边漏一个。

`Proto_SendAck` / `Proto_SendErr` 在 `proto.c:216 / 247`：按 `0xAA LEN 0x81 …SUM` 组帧，四角度按 0.1° 转 int16 小端，然后 `Uart_Send`（阻塞）。**它回的是"目标角"，不是"已到位"**（A3）。

### 5.5【必要】`Cmd_Submit`（`cmd.c:154`）—— 队列的三条规矩

```c
uint8_t Cmd_Submit(const Command *c)
{
    if (c == 0) return 0;
    if (c->type == CMD_STOP) Cmd_Clear();                    /* 规矩一：STOP 清队 */
    if (s_count >= CMD_QUEUE_SIZE) {                         /* 规矩二：满则丢最旧 */
        s_head = (uint8_t)((s_head + 1) % CMD_QUEUE_SIZE);
        s_count--;
    }
    s_queue[s_tail] = *c;                                    /* 规矩三：环形入队 */
    s_tail = (uint8_t)((s_tail + 1) % CMD_QUEUE_SIZE);
    s_count++;
    return 1;
}
```

- **规矩一**：STOP 有最高优先级——先清空历史命令，再把自己放进去。所以"急停"不会被前面的 MOVE 堵住。
- **规矩二**：队列 4 条，满了丢**最旧**的（`s_head` 前移），而不是拒绝新的。
- **规矩三**：`head/tail/count` 三个变量配合，`count == 0` 表示空。
- **只在主循环调用**：中断只写 ringbuf，不入队——这是"协议层不做决定"的分工。

**接下去**：队列里现在有一条 `Command`（`type` + `arg[6]`），等 `Cmd_Task` 取走。

### 5.6 一帧在内存里的样子（图 7）

```text
帧格式： 0xAA │ LEN │ CMD │ PAYLOAD... │ SUM
        LEN = 整帧字节数（含 SUM）    SUM = 前 LEN−1 字节累加和低 8 位
        多字节字段一律小端
```

| 偏移 | 值 | 含义 |
|---|---|---|
| 0 | `AA` | 帧头 |
| 1 | LEN | 整帧字节数，合法范围 4–16 |
| 2 | CMD | `01` MOVE / `02` HOME / `03` STOP / `04` QUERY / `05` SET_JOINT |
| 3–10 | x,y,z,phi | 各 int16 小端：mm / mm / mm / 0.1°（仅 MOVE，LEN=12） |
| 11 | SUM | 前 11 字节累加和低 8 位 |

```text
MOVE 到 home 位姿  AA 0C 01 C5 FF 8F 00 20 00 C2 01 ED
                            x=-59  y=143  z=32  phi=45.0°        SUM=0xED
QUERY              AA 04 04 B2
ACK（home 角）     AA 0C 81 65 04 C2 01 7C FC 84 03 62
                            112.5°  45.0°  -90.0°  90.0°
ERR（校验失败）    AA 05 82 02 33      错误码：01 不可达 / 02 校验 / 03 参数 / 04 未知
```

**逐字节走一遍**（自己再算一次，不要只看）：

```text
AA 0C 01 C5 FF 8F 00 20 00 C2 01 ED
│  │  │  └──┬──┘ └──┬──┘ └──┬──┘ └──┬──┘ │
│  │  │    x=-59     y=143    z=32   phi=450(→45.0°)  SUM
│  │  └─ CMD=0x01 MOVE
│  └─ LEN=12
└─ 帧头
SUM：0xAA+0x0C+0x01+0xC5+0xFF+0x8F+0x00+0x20+0x00+0xC2+0x01 = 0x3ED → 低 8 位 0xED ✓
x  ：C5 FF → 0xFFC5 = −59 ✓（小端：低字节在前）
```

---

## 6. 命令下行：一条 `Command` 变成四路脉宽（今天最重的一节）

**全景（图 8）**：

```text
【中断】USART2 ISR ── RingBuf_Write ──► ringbuf（第 5 节）
【主循环·命令侧】Proto_Poll → Cmd_Submit → Cmd_Task
      └─ MOVE：包围盒 → IK_Solve → Motion_MoveTo(calc_duration)
      └─ 同一轮立刻回 ACK（目标角，非到位）
【主循环·输出侧】每 20 ms：Motion_Task → apply_servo
      └─ 4 ×（ServoChannel → ThetaToUs → [没变就不写] → Servo_SetPulse → CCR）
```

### 6.1【必要】`Cmd_Task`（`cmd.c:190`）—— 一条命令翻译成"目标角 + 时长"

```c
queue_pop(&c);                       /* 取一条 */
Motion_GetCurrent(cur);              /* 当前真实角度（不是目标） */
copy_joints(out, cur);               /* out 默认 = 当前角，失败时 ACK 用 */

switch (c.type) {
case CMD_SET_POSE: {                 /* MOVE */
    target.x = c.arg[0]; ... target.phi = c.arg[3];
    if (!Kinematics_InWorkspace(&target)) { r->err = CMD_ERR_WS; break; }   /* ① 粗判 */
    if (!IK_Solve(&target, cur, solved))  { r->err = CMD_ERR_WS; break; }   /* ② 精确解 */
    Motion_MoveTo(solved, calc_duration(cur, solved));                      /* ③ 交给 motion */
    copy_joints(out, solved);
    s_pose = target;
    break;
}
case CMD_SET_JOINT: /* 单关节赋值 → Joint_Clamp → MoveTo → FK */ break;
case CMD_HOME:      /* Joint_GetHome → MoveTo(home, 800) → FK */  break;
case CMD_STOP:      /* Motion_Stop → FK(cur) */                   break;
case CMD_QUERY:     /* FK(cur) 更新 s_pose */                     break;
}
```

- **① 粗判**：`Kinematics_InWorkspace` 只做 4 次比较，挡掉明显越界，不做三角函数。
- **② 精确解**：`IK_Solve` 内部用 `in_limits` 过滤两支解，返回的解**一定**在软限位内。
- **③ 交给 motion**：时长由 `calc_duration` 按最大关节差算：

```c
maxΔ = max(|target[i] − cur[i]|);
if (maxΔ < 0.05f) return 0;                       /* 几乎没动 → 立即到位 */
ms = maxΔ * 1000 / 60;                            /* 60°/s */
夹到 [MOTION_MIN_MS, MOTION_MAX_MS] = [300, 2000];  /* 整数四舍五入 */
```

- **失败时 ACK 里放什么**：`r->theta` 仍是当前角（`copy_joints(r->theta, cur)`），方便上位机看到"没动"。
- **`calc_duration` 返回 0 的后果**：`Motion_MoveTo` 走 `< MOTION_STEP_MS` 分支，直接 `current = target`，下一帧就到位——所以"发一条几乎相同的 MOVE，看起来完全不动"是正常的（B5 实验）。

**接下去**：`Motion_MoveTo(solved, dur)` 收到"目标角数组 + 时长"；`result.theta` 被 `App_Run` 拿去发 ACK。

### 6.2【必要·短】`Kinematics_InWorkspace`（`kinematics.c:65`）

```c
if (t->x < WS_X_MIN || t->x > WS_X_MAX) return 0;
if (t->y < WS_Y_MIN || t->y > WS_Y_MAX) return 0;
if (t->z < WS_Z_MIN || t->z > WS_Z_MAX) return 0;
if (t->phi < WS_PHI_MIN || t->phi > WS_PHI_MAX) return 0;
return 1;
```

- **它是"包围盒"不是"可达性"**：盒内的点仍可能被 IK 拒绝（例如底座角度落在 `(202.5°, 360°)`）。
- 数值来自 `robot_params.h`：x[−201.46, 186.12]、y[−77.10, 201.46]、z[−48.28, 102.55]、phi[−58.5, 135]（按底座 22.5°–202.5° 离线采样）。
- **为什么先做它**：省掉绝大多数越界目标的三角函数运算；代价是它必须保守（宁可放过，不可误杀）。

### 6.3【必要】`IK_Solve`（`kinematics.c:135`）—— 四步，最后一步是"选解"

```c
phi = deg2rad(t->phi);
t1 = atan2f(t->y, t->x);                          /* ① 底座方位 */
if (t1 < 0.0f) t1 += 2.0f * ARM_PI;               /*    归一到 [0,2π) */

r  = sqrtf(t->x * t->x + t->y * t->y);
wr = r  - ARM_L4_MM * cosf(phi);                  /* ② 减掉末端 L4 → 腕心 */
wz = t->z - ARM_L4_MM * sinf(phi);

d = (wr*wr + wz*wz - L2*L2 - L3*L3) / (2*L2*L3);  /* ③ 余弦定理求 θ3 */
if (d > 1+1e-5 || d < -1-1e-5) return 0;          /*    不可达 */
a = acosf(d);  cand[0] = a; cand[1] = -a;         /*    两肘解 */

for (i = 0; i < 2; i++) {                         /* ④ 两支都求 θ2/θ4 */
    t3 = cand[i];
    t2 = atan2f(wz, wr) - atan2f(L3*sinf(t3), L2 + L3*cosf(t3));
    t4 = phi - t2 - t3;
    if (!in_limits(theta)) continue;              /*    超限的解直接扔掉 */
    dist = Σ|theta[j] − cur[j]|;                  /*    与当前角的曼哈顿距离 */
    if (!found || dist < best_dist) { best = theta; }
}
```

- **① 归一化**：`atan2` 返回 (−π, π]，加 2π 归到 `[0, 2π)`，再和底座限位 `[22.5°, 202.5°]` 比。所以 `y < 0` 且 `x > 0` 的目标**必被拒**（A8）。
- **② 减 L4**：把末端那块"板"从目标点扣掉，三连杆就降成两连杆——这是标准做法，也是"为什么能直接用余弦定理"的原因。
- **③ 两支解**：`±acos(D)` 对应"肘上/肘下"。`|D| > 1` 说明目标在两个圆的间隙里，物理上够不着。
- **④ 选最近解**：`Σ|Δθ|` 最小的那支。目的是**避免中途翻肘**——如果每次选"数学上更优"的那支，机械臂可能为了省 5° 而整体翻转 180°。
- **返回 0 的三种原因**：`D` 越界、两支都超限、或参数为空指针。

`in_limits`（`kinematics.c:41`）：四个关节逐个和 `joint_cfg[]` 的 `theta_min/max` 比，容差 `1e-3`。**IK 返回的解一定过了这一关**，这是 `apply_servo` 里"角度已经合法"的前提。

**接下去**：`solved[4]` 是关节角（度），进 `Motion_MoveTo`。

### 6.4【必要】`FK_Solve`（`kinematics.c:96`）—— 反向验证 IK 的工具

```c
r = L2·cos(t2) + L3·cos(t2+t3) + L4·cos(t2+t3+t4);
z = L2·sin(t2) + L3·sin(t2+t3) + L4·sin(t2+t3+t4);
x = r·cos(t1);  y = r·sin(t1);
phi = t2+t3+t4;                       /* 末端板绝对角 */
```

- **它不在运动链上**：MOVE 只用 IK；FK 只被 `Cmd_Init` / `Cmd_Home` / `Cmd_StopNow` / QUERY 用来"把关节角算回位姿给 UI 看"。
- **它的真正用途是自检**：`FK(IK(目标))` 应该回到目标。上电时用它验证 `s_home` → `s_pose = (−59.2, 142.9, 32.0, 45.0)`（第 3.9 节）。
- **单位**：输入度，输出 mm / 度。

### 6.5【必要】`Motion_MoveTo` / `Motion_Task` / `Motion_Stop`（`motion.c:57 / 96 / 83`）

**三个数组的写入时机**（这张表是整节的骨架）：

| 数组 | 谁写 | 什么时候 | 谁读 |
|---|---|---|---|
| `s_start` | `Motion_MoveTo` | 每条新命令：`start = current`（**当前**，不是旧目标） | `Motion_Task` |
| `s_target` | `Motion_MoveTo` / `Motion_Stop` | 新命令时 `= 形参` 再 `Joint_Clamp`；急停时 `= current` | `Motion_Task`、`Motion_GetTarget` |
| `s_current` | `Motion_Task` / `Motion_MoveTo`（`dur<20ms`）/ `Motion_Init` | 每 20 ms 插值一步 | `apply_servo`、`Cmd_Task`、`Cmd_GetStatus` |

```c
void Motion_MoveTo(const float target[J_NUM], uint16_t dur_ms)
{
    copy_joints(s_start, s_current);      /* 从"现在"出发，不是从旧目标 */
    copy_joints(s_target, target);
    Joint_Clamp(s_target);                /* 目标先过软限位 */

    s_step = 0; s_steps_total = 0;
    if (dur_ms < MOTION_STEP_MS) {        /* < 20 ms：视为立即到位 */
        copy_joints(s_current, s_target);
        s_busy = 0;
        return;
    }
    s_steps_total = (uint16_t)((dur_ms + MOTION_STEP_MS - 1) / MOTION_STEP_MS);  /* 向上取整 */
    s_busy = 1;
}

void Motion_Task(void)
{
    if (!s_busy) return;
    if (s_step < s_steps_total) s_step++;
    if (s_step >= s_steps_total) { copy_joints(s_current, s_target); s_busy = 0; return; }  /* 硬对齐 */
    p = (float)s_step / s_steps_total;
    s = 3.0f*p*p - 2.0f*p*p*p;            /* ease-in-out */
    for (i = 0; i < J_NUM; i++) s_current[i] = s_start[i] + (s_target[i] - s_start[i]) * s;
}

void Motion_Stop(void)
{
    copy_joints(s_target, s_current);     /* ★ 只改 target，不动 current */
    s_step = 0; s_steps_total = 0; s_busy = 0;
}
```

- **为什么按步数不按时间（A9）**：纯逻辑、不依赖 HAL，能在 PC 上单测；代价是主循环变慢时"步间隔变长、总时长变长"。
- **为什么最后一步硬对齐**：`3p²−2p³` 在 `p=1` 理论上等于 1，但 float 会有 1e-7 残留 → `ThetaToUs` 会差 1 μs → `apply_servo` 的 `!=` 永远成立，静止时也一直写 CCR。
- **`Motion_Stop` 不改 `current`**：这是"急停不跳变"的全部依据（A7）。写错了，急停瞬间舵机会被拽回某个目标。
- **`MoveTo` 时 `start = current`**：如果中途来了新命令，从"现在的位置"平滑续接，而不是从上一个起点重来。

### 6.6【必要】`apply_servo`（`app.c:102`）

```c
Motion_GetCurrent(theta);                 /* ① 读插值结果，不是 target */
for (j = 0; j < J_NUM; j++) {
    ch = Joint_ServoChannel(j);           /* ② 逻辑关节 → 物理通道 */
    us = Joint_ThetaToUs(j, theta[j]);    /*    两次夹紧在里面 */
    if (Servo_GetPulse(ch) != us)         /* ③ 和"上次写进去的值"比 */
        Servo_SetPulse(ch, us);           /*    第三次夹紧 + 写 CCR */
}
```

- **① 读 `current`**：输出"此刻应该在哪"。改成 `target` 就变成阶跃，`motion.c` 白写。
- **② 映射只有一处**：`CH4=底座、CH3=大臂、CH2=小臂、CH1=末端`，与逻辑顺序反着。只能用 `joint_cfg[].servo_ch`，不能 `j+1`（A6）。
- **③ 比的是 `s_pulse[]` 缓存**：省一次寄存器读；代价是每次写 CCR 必须同步缓存（`Servo_Init` 和 `Servo_SetPulse` 都写了）。
- **静止时的真实开销**：每 20 ms 4 次浮点乘加 + 4 次比较，**不写 CCR**（约 200 次/秒，可忽略）。

### 6.7【必要】`joint.c` 的五个函数：角度 ↔ 脉宽（含三次夹紧）

```c
uint16_t Joint_ThetaToUs(JointId j, float deg)
{
    const JointCfg *c = &joint_cfg[j];
    d = clampf(deg, c->theta_min, c->theta_max);                       /* 夹① 角度软限位 */
    u = c->pwm_min + (float)c->dir * (d - c->theta_at_pwm_min) * c->us_per_deg;
    u = clampf(u, c->pwm_min, c->pwm_max);                             /* 夹② 该关节脉宽限位 */
    return (uint16_t)(u + 0.5f);                                        /* 四舍五入 */
}

float Joint_UsToTheta(JointId j, uint16_t us)
{
    u = clampf(us, c->pwm_min, c->pwm_max);                            /* 先夹脉宽（不夹角度！） */
    return c->theta_at_pwm_min + (float)c->dir * (u - c->pwm_min) / c->us_per_deg;
}
```

**三次夹紧，各管一段**：

```text
① cmd/motion 入口 Joint_Clamp(θ)    角度夹到 joint_cfg[].theta_min/max    ← 命令层
        ▼
② Joint_ThetaToUs(j, θ)             θ 再夹 + us 夹该关节 pwm_min/max       ← 逻辑层
        ▼
③ Servo_SetPulse(ch, us)            us 夹该通道 SERVO_CHx_PWM_MIN/MAX      ← 驱动层
```

三个辅助函数：

| 函数 | 作用 | 关键点 |
|---|---|---|
| `Joint_ServoChannel(j)` | 返回 `joint_cfg[j].servo_ch` | 唯一的"逻辑→物理"映射点 |
| `Joint_Clamp(theta[])` | 原地夹四个角 | `cmd.c`/`motion.c` 入口各用一次 |
| `Joint_GetHome(out[])` | 复制 `s_home[]` | `if (!s_ready) Joint_Init();` 惰性兜底 |

三组宏（关节限位 / 关节脉宽 / 通道脉宽）**必须互相一致**，改一处要同步另两处——这是 `robot_params.h` 最容易被改坏的地方。

### 6.8【必要】`Servo_SetPulse` / `Servo_GetPulse`（`servo_drv.c:77 / 98`）

```c
void Servo_SetPulse(uint8_t ch, uint16_t us)
{
    if (ch >= SERVO_NUM) return;
    us = clampf(us, s_pwm_min[ch], s_pwm_max[ch]);              /* 第三次夹紧 */
    s_pulse[ch] = us;                                           /* 缓存与寄存器同步 */
    __HAL_TIM_SET_COMPARE(&htim1, s_tim_channel[ch], us);       /* CCR = us（1 tick = 1 μs） */
}

uint16_t Servo_GetPulse(uint8_t ch) { return s_pulse[ch]; }      /* 给 apply_servo 比较用 */
```

- **`s_pulse` 必须和真实 CCR 一致**：任何绕过 `Servo_SetPulse` 直接写 CCR 的代码，都会让"没变不写"失准。
- **写错的现象**：去掉 `s_pulse[ch] = us` → 第二次同角度不会写，之后每次都会写（缓存是旧值）。

### 6.9 手算：一帧从字节到 4 个 CCR

**表 3-1 换算常数**（全部来自 `robot_params.h`）

| 关节 | us=500 时的角度 | 每度 us | 方向 | 软限位（°） | 硬件脉宽（μs） |
|---|---|---|---|---|---|
| BASE | 202.5° | 2000/180 ≈ 11.111 | 脉宽↑ 角度↓ | 22.5 – 202.5 | 500–2500 |
| UPPER | 45° | 2000/90 ≈ 22.222 | 脉宽↑ 仰角↓ | 31.5 – 45 | 500–800 |
| FOREARM | −90° | 1/0.09 ≈ 11.111 | 脉宽↑ 向内收 | −90 – −45 | 500–1000 |
| END | 0° | 1/0.09 ≈ 11.111 | 脉宽↑ 向内收 | 0 – 135 | 500–2000 |

**表 3-2 走一遍 home 位姿**（`AA 0C 01 C5 FF 8F 00 20 00 C2 01 ED` → IK 回到 112.5 / 45 / −90 / 90）

| 关节 | θ（°） | 代入 | us | 通道 | CCR |
|---|---|---|---|---|---|
| BASE | 112.5 | 500 + (−1)(112.5 − 202.5)×2000/180 | **1500** | CH4 | 1500 |
| UPPER | 45.0 | 500 + (−1)(45 − 45)×2000/90 | **500** | CH3 | 500 |
| FOREARM | −90.0 | 500 + (+1)(−90 + 90)×1/0.09 | **500** | CH2 | 500 |
| END | 90.0 | 500 + (+1)(90 − 0)×1/0.09 | **1500** | CH1 | 1500 |

**表 3-3 自己填一张**（用 `python tools/frame_tool.py move x y z phi` 生成帧）

| 关节 | IK θ（°） | 代入计算 | us | 通道 | CCR |
|---|---|---|---|---|---|
| BASE |  |  |  | CH4 |  |
| UPPER |  |  |  | CH3 |  |
| FOREARM |  |  |  | CH2 |  |
| END |  |  |  | CH1 |  |

> 换任意目标时 IK 角度会有零点几度差异、us 差几微秒——这是允许的。**别背数字，背公式和通道。**

**对答案**：`docs/0007-day1-check.html` 的"手算核对器"；或串口收到 ACK 后用 `python tools/frame_tool.py parse "AA 0C 81 ..."` 反解。

### 6.10 上电 home 的两条路径（图 9）—— 为什么会"先到 home，20 ms 后跳一下"

```text
路径 A（驱动层，决定"上电第一束脉冲"）
  SERVO_CH*_HOME_US ──► Servo_Init：s_pulse[i] = home_us
                        __HAL_TIM_SET_COMPARE(...) → 第一个 PWM 周期就是 home

路径 B（逻辑层，决定"Motion 认为现在在哪"）
  joint_cfg[].home_us ──► Joint_Init：s_home = Joint_UsToTheta(home_us)
                        ──► Motion_Init：current = start = target = s_home
                        ──► 第一次 apply_servo（20 ms 后）：
                              θ → us' = Joint_ThetaToUs(θ)
                                     │
                          us' == home_us ?
                             ├─ 是 → 不写 CCR（脉宽没变，静止）
                             └─ 否 → 写 CCR → 舵机在上电 20 ms 后跳一下
```

**两者同源的三个条件**（缺一个就跳）：

1. `joint_cfg[].home_us` 与 `SERVO_CH*_HOME_US` 用同一组常量；
2. `UsToTheta` 与 `ThetaToUs` 互为逆运算（`dir`、`us_per_deg`、`theta_at_pwm_min` 一致）；
3. 反向换算不能被软限位截断（`s_home` 落在 `[theta_min, theta_max]` 内）。

```text
B1 的最小复现（J_BASE_DIR 由 −1 改成 +1）
  s_home(BASE) = 202.5 + (1500−500)/11.111 = 292.5°      ← UsToTheta 不夹角度
        ▼ ThetaToUs 先把 292.5 夹到 202.5
  us' = 500 + (+1)(202.5 − 202.5)×11.111 = 500
        ▼ 上电写 1500，20 ms 后写 500 → 底座猛转 1000 μs 对应的角度
```

### 6.11 四个常见误区

| # | 误区 | 事实 |
|---|---|---|
| 1 | "收到 ACK 就说明到位了" | ACK 回的是**目标角**，`Cmd_Task` 成功时立刻发，此时 `Motion` 还没走第一步。判断到位只能轮询 QUERY |
| 2 | "`Motion_Stop` 会把角度拉到某处" | 它只写 `target = current`，`current` 不动 → CCR 不变、不跳变 |
| 3 | "三次夹紧是重复劳动" | 三层各管一段（命令/逻辑/驱动），各自面对不同的调用者，缺一层就有一条绕过路径 |
| 4 | "IK 返回的角度可能超限" | `IK_Solve` 返回的解一定通过了 `in_limits`；超限的解在里面就被 `continue` 掉了 |

---

## 7. 状态与并发：谁能改什么

### 7.1【必要】SPSC 成立的三个条件（图 10）

```text
【中断上下文】USART2 ISR
  HAL_UARTEx_RxEventCallback
    ├─ RingBuf_Write(...)       写 buffer、写 writeIndex
    └─ 重挂接收
         │
         ▼（共享区：volatile readIndex / writeIndex + buffer[128]）
【主循环】
  RingBuf_GetLength / Peek（只读）· RingBuf_Skip（写 readIndex）
         │
         ▼
  Proto_Poll → Cmd_Submit → Cmd_Task → Motion_MoveTo
```

```text
SPSC：单生产者（ISR 只写 writeIndex）+ 单消费者（主循环只写 readIndex）
  → 不需要关中断
GetLength 可能读到瞬时不一致：最坏只是提前/延后一次"写入成功"的判断，不破坏数据区
满时整批拒绝（返回 0），不覆盖旧帧 → 宁可丢新，不让已缓存的完整帧被截断
```

"为什么真的不用关中断"的三条理由：

1. **两个索引各自只有一个写者**，不会出现两个写者互相覆盖；
2. **数据区不会被并发读写同一字节**：生产者只写 `[writeIndex, …)` 空区，消费者只读 `[readIndex, …)` 已提交区，中间隔着一帧的距离；
3. **`volatile` 保证不了原子性**，但它保证"每次访问都真的去内存读/写"，不会被编译器缓存进寄存器导致死循环。8 位索引在 Cortex-M3 上单字节访问本身就是原子的，所以这里够用。

### 7.2【必要】变量归属表（这张表要能自己写出来）

| 变量 | 谁写 | 谁读 | 上下文 |
|---|---|---|---|
| `s_current` | `Motion_Task`、`Motion_MoveTo`（`dur<20 ms`）、`Motion_Init` | `apply_servo`、`Cmd_Task`、`Cmd_GetStatus` | 主循环 |
| `s_start` | `Motion_MoveTo` | `Motion_Task` | 主循环 |
| `s_target` | `Motion_MoveTo`、`Motion_Stop`（`target = current`，**不动 current**） | `Motion_Task`、`Motion_GetTarget` | 主循环 |
| `s_step / s_steps_total / s_busy` | `Motion_*` | `Motion_IsBusy`、`Motion_Task` | 主循环 |
| `s_pulse[4]` | `Servo_Init`、`Servo_SetPulse` | `Servo_GetPulse`（`apply_servo` 比较） | 主循环 |
| `readIndex` | `RingBuf_Skip` | `GetLength/Peek/Write` | 主循环 |
| `writeIndex` | `RingBuf_Write` | `GetLength/GetRemain/Write` | **中断** |
| `s_last_valid` | `Proto_Poll`（只认整帧通过校验的） | `Proto_GetLastValidTick` | 主循环 |
| `s_last_activity` | `watchdog_task`、`enter_host`、`App_Init` | `watchdog_task` | 主循环 |
| `s_pose` | `Cmd_Task`、`Cmd_Init`、`Cmd_Home`、`Cmd_StopNow` | `Cmd_GetStatus → UI` | 主循环 |
| `s_mode` | `enter_host`、`exit_host`、`App_Init` | `App_Run`、`button_task` | 主循环 |
| `s_queue[]` | `Cmd_Submit`（入队）、`queue_pop`（出队） | `Cmd_Task` | 主循环 |

**结论只有一句**：跨上下文被访问的只有 `writeIndex` 和 `buffer`，其余全是主循环内部的先后顺序问题，不需要任何同步机制。

### 7.3【必要·短】`ringbuf.c` 值得全读的三个理由

- **`GetRemain` 故意少 1**（`(SIZE−1) − length`）：读写索引相等时无法区分"空"和"满"，所以牺牲 1 字节，有效容量 127。
- **跨尾部环绕**分两段 `memcpy`，第二段目标地址回到 `buffer` 开头；`writeIndex + length == SIZE` 走 else 分支也正确（第二段长度 0）。
- **A14 的坑**：`ringbuf.c` 自己 `#define RINGBUF_SIZE 128`，**没有 include `robot_params.h`**。改配置里的值不会影响环形缓冲——"配置层唯一参数入口"在这里被打破。修法：include 配置头并删本地宏。

---

## 8. 安全链：出错时谁兜底

**（图 11）**

```text
① 软限位，三道夹紧（顺序固定）
   cmd/motion 入口 Joint_Clamp(θ)        角度夹到 joint_cfg[].theta_min/max
        ▼
   Joint_ThetaToUs(j, θ)                 θ 再夹一次 + us 夹该关节 pwm_min/max
        ▼
   Servo_SetPulse(ch, us)                us 夹该通道 SERVO_CHx_PWM_MIN/MAX → CCR
   └─ 三处限位来自 robot_params.h 的三组宏，数值应彼此一致，改一处要同步另两处
```

```text
② 急停：三个入口，落点只有一个
   KEY4 ──────────┐
   退出 HOST ─────┼──► Cmd_StopNow() ──► Cmd_Clear() + Motion_Stop()
   通信超时 ──────┘                          └─ target = current：CCR 不变、不跳变
                                                  下一次 MOVE 的 start 取当前 current，无缝续接
```

```text
③ 通信看门狗：只在 HOST，只认"整帧通过校验"
   every 50 ms（watchdog_task）:
     last_valid 变了？        → 刷新 s_last_activity、解锁 TO
     Motion_IsBusy()？        → s_last_activity = now（运动期间永不过期）
     now − s_last_activity > 500 ms 且未锁 → Cmd_StopNow() + s_timeout_latched = 1
```

```text
④ 上电安全：Servo_Init 先写 CCR 再 PWM_Start
   不这么做 → 第一个周期 CCR=0 → 无效脉宽 → 舵机从上电位置猛跳到某个极限再回 home
```

**拔线时序（图 12，对应 A10）**

```text
t0  最后一帧合法（或某条命令开始运动）
 │  运动 ≤ 2 s：busy = 1，每 50 ms 续命一次 → 不会超时
 ▼
t1  运动结束（busy = 0），t1 仍算"活动"
 │  之后 500 ms 内没有新有效帧
 ▼
t1 + 500 ms：Cmd_StopNow() → 舵机停在当前角，屏幕 TO，锁存（防重复急停）
 │
新帧到达且校验通过
 ▼
解锁、恢复计时；下一条命令正常执行
```

---

## 9. 上板：用现象反证理解（图 13）

前置：舵机**独立 5V 大电流供电 + 共地**；驱动板两排排针**跳线帽短接**；信号 3.3V。

**先做 C1 → C3 → C4**，一次覆盖三条不同的链：

```text
C1 上电不猛动     ← 验第 3.3–3.5 节：两条 home 路径同源
C3 QUERY 回 ACK   ← 验第 5 节整条上行：中断 → ringbuf → proto → cmd → ACK
C4 BASE=180°→−x   ← 验第 6.7 节：底座标定公式 + 逻辑关节→物理通道映射
C5 MOVE 回 home   ← 验第 6.1、6.3 节：IK 选解 + calc_duration
C2 按键           ← 验第 4.3 节：模式机 + 10 ms 任务
C6 不发帧 → TO    ← 验第 4.5 节：看门狗与锁存
```

| # | 操作 | 期望 | 对应单元 | 不对时先查 |
|---|---|---|---|---|
| C1 | 上电 | 底座 112.5°、大臂 +45°、小臂 −90°、末端 +90°；**无猛动** | 3.3–3.5 | `SERVO_CH*_HOME_US` 与 `joint_cfg[].home_us` 是否同源 → `dir` 符号 → 供电压降 |
| C2 | KEY2 短按 / 长按 1 s；KEY3；KEY4 | 进 HOST / 退出并急停 / 回 home / 屏显 `STOP` | 4.3 | `KeyDrv_Read` 低有效、去抖 20 ms、`s_key2_ignore_hold` |
| C3 | HOST 下发 `AA 04 04 B2` | 回 `AA 0C 81 65 04 C2 01 7C FC 84 03 62` | 5.1–5.4 | 波特率 115200、PA2/PA3、`RxToIdle` 是否重挂 |
| C4 | 下发 `AA 07 05 00 08 07 C5`（BASE=180°） | 底座对准 **−x 轴**；ACK 里 BASE≈180.0 | 6.7 | 底座公式 `θ1 = 202.5 − (us−500)×180/2000` → CH4 接线 → 750 μs 是否真指 −x |
| C5 | 下发 `AA 0C 01 C5 FF 8F 00 20 00 C2 01 ED` | 基本不动或极小修正；ACK≈home | 6.1、6.3 | IK 选解、`calc_duration` 是否返回 0 |
| C6 | 进 HOST 后不发帧 | 约 500–550 ms 后屏显 `TO`，舵机停 | 4.5 | `UART_TIMEOUT_MS`、`watchdog_task` 是否在跑 |

帧都可以用 `python tools/frame_tool.py` 重新生成，别手抄。

---

## 10. 收尾

### 10.1 全部内容压成一张图（图 14）

```text
                   上位机 115200 8N1：AA 0C 01 C5 FF 8F 00 20 00 C2 01 ED
                       ▼
USART2 IRQ ── RxEventCallback ── RingBuf_Write ──► [ringbuf 128B，SPSC]
                                                        │ 主循环每轮
                                                        ▼
                              Proto_Poll：0xAA → LEN → 半帧等 → 校验 → 分发
                                                        │ Cmd_Submit（队列 4，满丢最旧，STOP 清队）
                                                        ▼
                    Cmd_Task：包围盒 → IK_Solve → Motion_MoveTo(calc_duration)
                                        │                      │
                                        │ 立刻回 ACK（目标角）  │ start/current/target
                                        ▼                      ▼
                                   Uart_Send（阻塞 1 ms）   每 20 ms：Motion_Task（3p²−2p³）
                                                                   ▼
                                                    apply_servo：θ → us（夹两次）→ CCR（再夹）
                                                                   ▼
                                                    TIM1 CH1..4，50 Hz，500–2500 μs
```

一句话总结：**中断只搬字节，主循环做决定，20 ms 任务做插值，驱动层只认物理通道和脉宽。** 四层单向依赖，所以每一层都能单独讲、单独测、单独换。

### 10.2 表 4-1 要背下来的 12 个数字

| 项 | 值 | 位置 |
|---|---|---|
| 任务周期 | 按键 10 / 运动 20 / 看门狗 50 / UI 50 ms | `robot_params.h` |
| 运动时长 | `max(Δθ) × 1000 / 60`（60°/s），夹到 300–2000 ms；`< 0.05°` → 0 | `cmd.c` |
| home 复位 | 800 ms 平滑 | `robot_params.h` |
| 通信超时 | 500 ms（仅 HOST；运动中不计时；实际约 550 ms 触发） | `app.c` |
| 命令队列 | 4 条，满丢最旧；STOP 先清队 | `cmd.c` |
| 环形缓冲 | 128 B（有效 127），满则整批丢 | `ringbuf.c` |
| 帧长 | 最大 16 B；MOVE 12 / HOME·STOP·QUERY 4 / SET_JOINT 7 | `robot_params.h` |
| 串口 | USART2，115200 8N1，RxToIdle 中断，NVIC 优先级 0 | `uart_drv.c` |
| PWM | TIM1，PSC=71 → 1 μs/tick，ARR=19999 → 20 ms，500–2500 μs | `servo_drv.c` |
| 软限位 | BASE 22.5–202.5 / UPPER 31.5–45 / FOREARM −90–−45 / END 0–135 | `robot_params.h` |
| home 角 ↔ 脉宽 | 112.5 / 45 / −90 / 90° ↔ 1500 / 500 / 500 / 1500 μs（CH4/CH3/CH2/CH1） | `joint.c` |
| 工作空间 | x[−201.46, 186.12] y[−77.10, 201.46] z[−48.28, 102.55] phi[−58.5, 135] | `robot_params.h` |

### 10.3 表 4-2 五处"看着对其实不对"的坑

| # | 文档 / 直觉说 | 代码实际 | 影响 |
|---|---|---|---|
| 1 | `../架构/design_idea.md` 0.1：FLASH 35168 B / 53.66% | 实测 35944 B / 54.8% | 低：数字过期，顺手改 |
| 2 | 旧版清单说"`Motion_Stop` 会改 `current`" | 只写 `target = current`，`current` 不动 | **高**：A7"不跳变"靠它 |
| 3 | `../资料/ARM_HANDOFF.md`：CH1 = GRIP / 抓夹 | `joint_cfg[3]` = END（末端 pitch），CH1 = PA8 | 中：测试台与正式工程的命名差异，不是 bug |
| 4 | `robot_params.h` 的 `RINGBUF_SIZE` | `ringbuf.c` 自己 `#define 128`，没 include 配置层 | 中：改配置不生效（A14） |
| 5 | "ACK 表示命令执行完成" | ACK 只是"命令已接受"，`theta` 是目标角 | 中：上位机会误以为已到位（A3） |

### 10.4 A 组｜闭卷 15 题（先别翻答案）

1. `App_Run` 一帧里四个软任务的周期分别是多少？哪个任务最可能推迟其他任务？
2. 一个 12 字节 MOVE 帧，从进 USART2 到 CCR 改变，按顺序写出函数名，并标出哪一步在中断里。
3. ACK 里的四个角度，是"目标角"还是"当前实际角"？为什么？这会让上位机误判什么？
4. `RingBuf_Write` 在缓冲满时返回什么？调用者怎么处理？对协议帧的后果是什么？
5. 校验失败的帧，为什么 `RingBuf_Skip(1)` 而不是 `Skip(len)`？什么情况下会连续回多个 `0x82/02`？
6. 写出 4 个逻辑关节 → 物理通道 → 引脚 → 实际舵机的对应关系。哪个文档和代码在这里不一致？
7. 运动中按 KEY4 急停，四个 CCR 会不会跳变？为什么？下一次 MOVE 的起点是什么？
8. 底座限位 22.5°~202.5°，IK 里 `θ1` 归一化到 `[0,2π)`。举两个"算得出角度但一定不可达"的目标区域。
9. `Motion_Task` 为什么按"步数"推进而不是用 `HAL_GetTick` 算时间？主循环变慢时，运动时长会怎样？
10. 通信看门狗为什么在运动过程中不触发？上位机在运动中途死掉，实际会发生什么？
11. `app.c` 把 `s_mode` 直接塞进 `UiState.mode`，`ui.c` 又拿它和 `UI_PAGE_HOST` 比较。这两个常量分别定义在哪、值是多少？如果以后把 `APP_MODE_HOST` 改成 2 会怎样？
12. `Servo_Init` 为什么必须"先写 CCR 再 Start PWM"？不这么做，上电瞬间会看到什么？
13. 连续发两条 MOVE（不是 STOP），第二条会等第一条运动完吗？实际哪条生效？
14. `robot_params.h` 里的 `RINGBUF_SIZE` 被谁用了？改它会影响环形缓冲吗？这说明什么？
15. 上电时，"舵机实际输出的 home 脉宽"和"`Motion` 认为的 home 角度"是两条独立路径算出来的。分别是哪两条？它们什么时候会不一致？不一致时上电后 20 ms 内会看到什么？

### 10.5 A 组参考答案

**A1** 10 / 20 / 50 / 50 ms（BUTTON / MOTION / WDG / UI）。阻塞点：OLED 整屏刷新（页面切换时的 `OLED_Update`，软件 I2C 发 1024 字节，**ms 级，未实测**）、`Uart_Send`（`HAL_UART_Transmit` 阻塞，12 字节约 1 ms，超时上限 100 ms）、`Proto_Poll` 处理积压帧。它们推迟同一轮后面的任务，**运动按步数推进，所以变慢而不是丢步**。

**A2** 中断：`HAL_UARTEx_RxEventCallback → RingBuf_Write`。主循环：`App_Run → Proto_Poll（Peek/Skip/frame_sum）→ Cmd_Submit → Cmd_Task → Kinematics_InWorkspace → IK_Solve → Motion_MoveTo`；20 ms 后 `Motion_Task → apply_servo → Motion_GetCurrent → Joint_ThetaToUs → Servo_SetPulse → __HAL_TIM_SET_COMPARE`。

**A3** 目标角。`Cmd_Task` 成功时 `r->theta = out`（MOVE 是 `solved`，HOME 是 home，QUERY 是当前 `cur`），`app.c` 拿到结果**立即**发 ACK，此时 `Motion` 还没走第一步。所以 ACK 只能当"命令已接受"；要判断到位只能轮询 QUERY。

**A4** 返回 0（整批不写）。`RxEventCallback` 忽略返回值，这批数据**永久丢失**。后果：可能丢在帧中间，后续字节错位；`Proto_Poll` 靠找 `0xAA` + 校验和滑动重同步。设计上"丢新不覆盖旧"。

**A5** 校验失败说明"这个 0xAA 可能根本不是帧头"，丢 1 字节滑动重找；丢整帧反而可能跳过真正的帧头。若流里连续出现多个"`0xAA` + 合法 LEN"的假帧头，会每个都失败一次、各回一个 `0x82/02`。

**A6** BASE→CH4→PA11；UPPER→CH3→PA10；FOREARM→CH2→PA9；END→CH1→PA8。`../资料/ARM_HANDOFF.md` 第 3 节把 CH1 叫 `GRIP/抓夹`，而正式设计是**无抓夹、第 4 关节 = 末端 pitch**，代码里叫 `END`。测试台与正式工程的差异，不是 bug。

**A7** 不会跳变。`Motion_Stop` 把 `target` 设成 `current`，`current` 本身没动，`apply_servo` 算出的 `us` 和上次一样，`Servo_GetPulse != us` 不成立，**不写 CCR**。下一次 `Motion_MoveTo` 的 `start` 取当前 `current`，无缝续接。

**A8** `θ1 = atan2(y,x)` 归一到 `[0,360)`，限位只收 `[22.5, 202.5]`。不可达例子：① 第四象限 `x>0, y<0`（θ1 ∈ (202.5,360)）；② 第三象限偏下 `x<0, y<0` 且 θ1 > 202.5。也就是说 `y<0` 且 `x>0` 的目标**一定**被拒。

**A9** 为了纯逻辑、不依赖 HAL，能在 PC 上单测（见 `../架构/design_idea.md` 7.3）。每步固定 20 ms，但实际步进由 `Sched_IsDue` 驱动；主循环被阻塞时步间隔变长，**总步数不变 → 总时长变长**。所以 300–2000 ms 是"理想节拍下"的时长。

**A10** `watchdog_task` 里 `if (Motion_IsBusy()) s_last_activity = now_ms;`，运动中持续续命。上位机中途死掉：当前运动仍走完（≤2 s），运动结束 500 ms 后 `Cmd_StopNow()`，屏幕 `TO`，并 `s_timeout_latched=1` 防重复急停。

**A11** `APP_MODE_MENU/HOST` 定义在 `app.c`（0/1）；`UI_PAGE_MENU/HOST` 定义在 `ui.c`（0/1）。数值恰好相同，所以现在能工作。若 `APP_MODE_HOST` 改成 2，`ui.c` 的 `page = (s_state.mode == UI_PAGE_HOST)` 永远为假，**HOST 页永远不显示**，而且不会报错。属于隐式耦合，应改成显式映射。

**A12** 否则 `PWM_Start` 后的第一个周期 CCR=0，输出无效脉宽，舵机会从上电位置**猛跳到某个极限再回 home**。先写 CCR 保证第一个脉冲就是 home。

**A13** 不会等。`Proto_Poll` 一次 `while` 会把 ringbuf 里所有完整帧都解析入队；`Cmd_Task` 每个主循环取一条。第二条执行时 `current` 还没动（第一条要等 20 ms 才插值），于是第二条把 `target` 覆盖，`start` 取当前 `current`。**实际几乎只有最后一条生效**。命令队列 ≠ 运动队列。

**A14** 没人用。`ringbuf.c` 自己 `#define RINGBUF_SIZE 128`，且**没有 include `robot_params.h`**。改配置里的值不会影响环形缓冲——"配置层唯一参数入口"在这里被打破。修法：`ringbuf.c` include `robot_params.h` 并删本地宏。

**A15** 路径一：`Servo_Init()` 直接把 `SERVO_CH*_HOME_US` 写进 CCR；路径二：`Joint_Init()` 用 `Joint_UsToTheta(home_us)` 反算 `s_home[]`，`Motion_Init` 用它当 `current`，第一次 `apply_servo` 又把 `current` 转回 us。两条路径**只在"`joint_cfg[].home_us` 与 `SERVO_CH*_HOME_US` 同源、且标定公式正确"时吻合**。不一致时表现为**上电先到 home、20 ms 后突然跳一下**（B1 是最小复现）。

### 10.6 B 组｜改代码验证（5 个实验，改完 `git checkout` 回滚）

**先写预测，再动手。** 不预测就等于没做。

| # | 改什么 | 先预测，再验证 |
|---|---|---|
| B1 | `robot_params.h` 的 `J_BASE_DIR` 由 `-1` 改成 `+1` | `Joint_UsToTheta(1500)` 算出多少度？第一次 `apply_servo` 写多少 us？为什么上电 20 ms 内会跳？ |
| B2 | `app.c` 的 `apply_servo` 去掉 `if (Servo_GetPulse(ch) != us)` | CCR 写入频率变成多少？PWM 波形有影响吗？ |
| B3 | `MOTION_STEP_MS` 由 20 改 10 | 800 ms 复位还是 800 ms 吗？步数变多少？ |
| B4 | 发 `SET_JOINT(UPPER=180)`（帧 `AA 07 05 01 08 07 C6`） | 回 ACK 还是 ERR？ACK 里角度是多少？为什么？ |
| B5 | 发 MOVE 到当前位姿（`-59,143,32,45`） | 会运动吗？运动多久？ACK 为什么不等它？ |

> B2 需要逻辑分析仪/示波器；没有就临时加计数器数 `Servo_SetPulse` 的调用次数（OLED 或调试器看）。

### 10.7 今日自评：按【必要】单元打勾

| 单元 | 能复述？ |
|---|---|
| 0.3 全局调用树 | ☐ 能默写 / ☐ 只能照抄 / ☐ 画不出 |
| 3.1–3.10 `App_Init` 十步（每步改了什么、给谁用） | ☐ 全部 / ☐ 记得 6 步以上 / ☐ 记不清 |
| 3.11 顺序：三条硬依赖 | ☐ 能讲 / ☐ 看过 |
| 4.1–4.5 `App_Run`、`Sched_IsDue`、`button_task`、`watchdog_task` | ☐ 能讲 / ☐ 只有印象 |
| 5.1–5.5 上行五个函数 | ☐ 能讲 / ☐ 只有印象 |
| 6.1–6.8 下行八个函数 | ☐ 能讲 / ☐ 只有印象 |
| 6.9 手算一帧 → 4 个 CCR | ☐ 算对 / ☐ 差几十 μs / ☐ 不会 |
| 7.1–7.2 并发三条件 + 变量归属表 | ☐ 能重画 / ☐ 记得大概 |
| 8 三条安全链 | ☐ 能口述"拔线 3 秒" / ☐ 不能 |
| 10.4 A 组 15 题 | __ / 15 |

---

## 附录 A：黑盒清单（契约表）

| 黑盒 | 它保证什么 | 输入 → 输出 | 什么时候必须打破 |
|---|---|---|---|
| 启动文件 + `HAL_Init` | 栈/堆、`.bss` 清零、SysTick 1 ms | — | 换 tick 源、改启动流程 |
| `SystemClock_Config` | 72 MHz；APB1 36 / APB2 72 | — | 改主频、改 PWM 频率 |
| `Error_Handler` | 任一 `MX_*_Init` 失败时 `__disable_irq()` + 死循环 | — | 调试期想让它打印原因（可改成闪灯/串口输出） |
| `SysTick_Handler` | 每 1 ms `HAL_IncTick()` | — | 改 tick 周期 |
| `USART2_IRQHandler → HAL_UART_IRQHandler` | 把 RX/IDLE 事件转成回调 | `(huart, Size)` | 丢字节、Overrun、改 DMA |
| `HAL_MspInit` / `HAL_TIM_Base_MspInit` / `HAL_TIM_MspPostInit` | 开 AFIO/PWR/TIM1 时钟；PA8–PA11 复用推挽 | — | 换引脚 |
| `HAL_UART_MspInit` | 开 USART2 时钟；PA2=TX、PA3=RX；NVIC 优先级 0 | — | 换引脚/改优先级 |
| `MX_USART2_UART_Init` | 115200 8N1 收发 | — | 波特率对不上 |
| `KeyDrv_Init` | 空函数（GPIO 已配好） | — | 不需要 |
| `KeyDrv_Read(id)` | 1 = 按下（低电平已取反） | `id` → 0/1 | 改上下拉 |
| `OLED_Init` / `OLED_Update` / `OLED_UpdateArea` | 初始化屏幕；整屏/局部发送 | 坐标、宽高 | 花屏、刷新太慢 |
| `OLED_SetCursor` / `OLED_ShowString` / `OLED_ShowSignedNum` / `OLED_ShowNum` | 写显存：定位、写字、写有符号数、写无符号数 | 坐标/数字/位数 | 显示错位、字体不对 |
| `OLED_W_SCL` / `OLED_W_SDA` | 软 I2C 拉线（PB8/PB9 开漏） | 0/1 | 换 I2C 硬件 |
| `oled_data.c` 字模数组 | 8×16 与 6×8 点阵、中文索引 | 查表 | 换字体 |
| `OLED_ShowChar` / `ShowHexNum` / `ShowBinNum` / `ShowFloatNum` / `ShowChinese` / `ShowImage` / `Printf` / `DrawPoint`…`DrawArc`（共约 30 个） | 第三方库的完整绘图能力 | 各种形状参数 | 要画新界面时查 `oled.h` 注释，不进 `.c` |
| `draw_menu` / `draw_host_static` / `draw_values` / `round_i` / `status_code` / `status_text` | 画菜单骨架 / 画 HOST 骨架 / 只刷数字区 / 浮点四舍五入 / 状态码取值 / 状态码转文字 | 内部状态 | 改界面时 |
| `UI_Init` / `UI_Task` | 初始化第一屏；按脏标记增量刷新 | `UiState` | 改刷新策略 |
| `update_ui` | 打包 `CmdStatus` + 模式/超时/急停 → `UiState` | — | 改显示内容 |
| `deg2rad` / `rad2deg` | 度 → 弧度 / 反向 | float | 不需要 |
| `acosf` / `atan2f` / `sqrtf` | C 数学库，精度按 double 提升 | — | 数值精度出问题 |
| `test.c` 全部 13 个函数 | 手动标定页，`Test()` 无人调用 | — | 见附录 B |
| `system_stm32f1xx.c` / `syscalls.c` / `sysmem.c` | 系统时钟变量、`_sbrk` 等 | — | 用 printf/动态内存时 |

## 附录 B：`test.c` —— 编译进固件但没人调用的 341 行

- 它在 `CMakeLists.txt` 的 `USER_SOURCES` 里（第 66 行），所以**确实被编译并链接进 `Test3.elf`**。
- 它的入口 `Test()` 在整个工程里**没有任何调用点**（`main.c` 只调 `App_Init` / `App_Run`）；链接参数里也没有 `--gc-sections`，所以这些代码会占 FLASH。
- 它内部还定义了与正式模块**同名**的 `static` 函数（`App_Init`、`Servo_SetPulse`、`Button_Init` …）。因为都是 `static`，不会和正式实现冲突，但读代码时容易误以为"有两套实现"。
- **今天只读不改**。要处理的话有两个选项：从 `USER_SOURCES` 移除（最干净），或保留并在文件头写清"仅手动调试用，未接入主流程"。

---

> 明天的入口：底座 750 μs ↔ −x 的实测定标（C4），以及小臂/末端斜率的两点标定（`../资料/ARM_HANDOFF.md` TODO 2）。


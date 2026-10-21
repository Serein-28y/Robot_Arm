#include "main.h"
#include "test.h"
#include "oled.h"

#define STEP_US     10
#define CH_NUM      4

/* ===== 舵机规格（180 / 270 由此切换）===== */
typedef struct
{
    const char *name;      /* 显示用，如 "180" / "270" */
    uint16_t    pwm_min;   /* 最小脉宽 us */
    uint16_t    pwm_max;   /* 最大脉宽 us */
    uint16_t    range_deg; /* 行程角度 */
} ServoSpec;

static const ServoSpec SERVO_180 = { "180", 500, 2500, 180 };
static const ServoSpec SERVO_270 = { "270", 500, 2500, 270 };

/* 一键切换全部规格：改这一行即可（&SERVO_180 <-> &SERVO_270）*/
#define SERVO_DEFAULT   (&SERVO_180)

/* 每通道选规格（想单独换成 270，就把对应项改成 &SERVO_270）*/
static const ServoSpec *const servo_spec[CH_NUM] =
{
    SERVO_DEFAULT, SERVO_DEFAULT, SERVO_DEFAULT, SERVO_DEFAULT
};

/* 上电初始脉宽 / KEY4 回位目标（us）：CH1, CH2, CH3, CH4 */
static const uint16_t servo_home[CH_NUM] = { 1500, 500, 500, 1500 };

/* 按键事件 */
#define BTN_NONE    0
#define BTN_PRESS   1
#define BTN_REPEAT  2
#define BTN_RELEASE 3

/* 去抖/长按时间（单位：ms） */
#define DEBOUNCE_MS 20
#define REPEAT_MS   50
#define HOLD_MS     500
#define KEY4_LONG_MS 1000

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t      pin;
    uint8_t       last_raw;
    uint8_t       stable;
    uint32_t      t_change;
    uint32_t      t_down;
    uint32_t      t_repeat;
    uint8_t       repeat_en;
} Button;

static Button btns[4];

static uint8_t  current_ch;
static uint16_t pulse[CH_NUM];
static uint16_t pulse_min[CH_NUM];
static uint16_t pulse_max[CH_NUM];
static int8_t   last_dir[CH_NUM];
static uint8_t  key4_long;

/* 通道号(0~3) -> TIM 通道 */
static uint32_t ChannelTim(uint8_t ch)
{
    switch (ch)
    {
        case 0:  return TIM_CHANNEL_1;
        case 1:  return TIM_CHANNEL_2;
        case 2:  return TIM_CHANNEL_3;
        default: return TIM_CHANNEL_4;
    }
}

/* 按当前通道规格估算角度 */
static uint16_t Servo_Angle(uint8_t ch, uint16_t us)
{
    const ServoSpec *s = servo_spec[ch];
    if (us < s->pwm_min) us = s->pwm_min;
    if (us > s->pwm_max) us = s->pwm_max;
    return (uint16_t)((uint32_t)(us - s->pwm_min) * s->range_deg / (s->pwm_max - s->pwm_min));
}

static void Servo_SetPulse(uint8_t ch, uint16_t us)
{
    const ServoSpec *s = servo_spec[ch];
    if (us < s->pwm_min) us = s->pwm_min;
    if (us > s->pwm_max) us = s->pwm_max;
    pulse[ch] = us;
    __HAL_TIM_SET_COMPARE(&htim1, ChannelTim(ch), us);
}

static void Servo_InitPositions(void)
{
    uint8_t ch;
    for (ch = 0; ch < CH_NUM; ch++)
    {
        pulse[ch] = servo_home[ch];
        __HAL_TIM_SET_COMPARE(&htim1, ChannelTim(ch), servo_home[ch]);
    }
}

static void Servo_StartAll(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

static void Button_Init(void)
{
    uint8_t i;
    uint8_t raw;

    btns[0].port = KEY1_GPIO_Port; btns[0].pin = KEY1_Pin; btns[0].repeat_en = 0;
    btns[1].port = KEY2_GPIO_Port; btns[1].pin = KEY2_Pin; btns[1].repeat_en = 1;
    btns[2].port = KEY3_GPIO_Port; btns[2].pin = KEY3_Pin; btns[2].repeat_en = 1;
    btns[3].port = KEY4_GPIO_Port; btns[3].pin = KEY4_Pin; btns[3].repeat_en = 0;

    for (i = 0; i < 4; i++)
    {
        raw = (HAL_GPIO_ReadPin(btns[i].port, btns[i].pin) == GPIO_PIN_RESET) ? 1 : 0;
        btns[i].last_raw = raw;
        btns[i].stable   = raw;
        btns[i].t_change = HAL_GetTick();
        btns[i].t_down   = HAL_GetTick();
        btns[i].t_repeat = HAL_GetTick();
    }
}

/* 返回按键事件：BTN_NONE / PRESS / REPEAT / RELEASE */
static uint8_t Button_Update(Button *b)
{
    uint8_t  raw = (HAL_GPIO_ReadPin(b->port, b->pin) == GPIO_PIN_RESET) ? 1 : 0;
    uint32_t now = HAL_GetTick();
    uint8_t  event = BTN_NONE;

    if (raw != b->last_raw)
    {
        b->last_raw = raw;
        b->t_change = now;
        return BTN_NONE;
    }

    if ((now - b->t_change) < DEBOUNCE_MS)
    {
        return BTN_NONE;
    }

    if (raw != b->stable)
    {
        b->stable = raw;
        if (raw)
        {
            b->t_down   = now;
            b->t_repeat = now;
            event = BTN_PRESS;
        }
        else
        {
            event = BTN_RELEASE;
        }
    }
    else if (raw && b->repeat_en &&
             (now - b->t_down >= HOLD_MS) &&
             (now - b->t_repeat >= REPEAT_MS))
    {
        b->t_repeat = now;
        event = BTN_REPEAT;
    }

    return event;
}

static uint8_t Button_HeldMs(Button *b, uint32_t ms)
{
    return (uint8_t)(b->stable && (HAL_GetTick() - b->t_down >= ms));
}

static void UI_DrawStatic(void)
{
    OLED_Clear();
    OLED_ShowString(0, 0, "CH", OLED_8X16);
    OLED_ShowString(48, 0, "PWM", OLED_8X16);
    OLED_ShowString(0, 16, "ANG", OLED_8X16);
    OLED_ShowString(40, 16, "~", OLED_8X16);
    OLED_ShowString(88, 16, "R", OLED_8X16);
    OLED_ShowString(0, 32, "MIN", OLED_8X16);
    OLED_ShowString(64, 32, "MAX", OLED_8X16);
    OLED_ShowString(0, 48, "DIR", OLED_8X16);
    OLED_ShowString(64, 48, "STEP", OLED_8X16);
    OLED_ShowNum(96, 48, STEP_US, 2, OLED_8X16);
    OLED_Update();
}

static void UI_UpdateValues(void)
{
    OLED_ShowNum(24, 0, current_ch + 1, 1, OLED_8X16);
    OLED_ShowNum(80, 0, pulse[current_ch], 4, OLED_8X16);
    OLED_ShowNum(48, 16, Servo_Angle(current_ch, pulse[current_ch]), 3, OLED_8X16);
    OLED_ShowNum(96, 16, servo_spec[current_ch]->range_deg, 3, OLED_8X16);
    OLED_ShowNum(32, 32, pulse_min[current_ch], 4, OLED_8X16);
    OLED_ShowNum(96, 32, pulse_max[current_ch], 4, OLED_8X16);

    if (last_dir[current_ch] > 0)
        OLED_ShowChar(32, 48, '+', OLED_8X16);
    else if (last_dir[current_ch] < 0)
        OLED_ShowChar(32, 48, '-', OLED_8X16);
    else
        OLED_ShowChar(32, 48, ' ', OLED_8X16);

    OLED_UpdateArea(24, 0, 104, 16);
    OLED_UpdateArea(40, 16, 88, 16);
    OLED_UpdateArea(32, 32, 96, 16);
    OLED_UpdateArea(32, 48, 8, 16);
}

static void App_Loop(void)
{
    uint8_t ev;
    uint8_t changed = 0;

    /* KEY1: 切换通道 */
    ev = Button_Update(&btns[0]);
    if (ev == BTN_PRESS)
    {
        current_ch = (current_ch + 1) % CH_NUM;
        changed = 1;
    }

    /* KEY2: 脉宽 + */
    ev = Button_Update(&btns[1]);
    if (ev == BTN_PRESS || ev == BTN_REPEAT)
    {
        if (pulse[current_ch] + STEP_US <= servo_spec[current_ch]->pwm_max)
        {
            pulse[current_ch] += STEP_US;
            last_dir[current_ch] = 1;
            changed = 1;
        }
    }

    /* KEY3: 脉宽 - */
    ev = Button_Update(&btns[2]);
    if (ev == BTN_PRESS || ev == BTN_REPEAT)
    {
        if (pulse[current_ch] >= servo_spec[current_ch]->pwm_min + STEP_US)
        {
            pulse[current_ch] -= STEP_US;
            last_dir[current_ch] = -1;
            changed = 1;
        }
    }

    /* KEY4: 短按回位(home) / 长按清 MIN,MAX */
    ev = Button_Update(&btns[3]);
    if (ev == BTN_PRESS)
    {
        key4_long = 0;
    }
    if (!key4_long && Button_HeldMs(&btns[3], KEY4_LONG_MS))
    {
        key4_long = 1;
        pulse_min[current_ch] = pulse[current_ch];
        pulse_max[current_ch] = pulse[current_ch];
        changed = 1;
    }
    if (ev == BTN_RELEASE && !key4_long)
    {
        pulse[current_ch] = servo_home[current_ch];
        last_dir[current_ch] = 0;
        changed = 1;
    }

    if (changed)
    {
        if (pulse[current_ch] < pulse_min[current_ch]) pulse_min[current_ch] = pulse[current_ch];
        if (pulse[current_ch] > pulse_max[current_ch]) pulse_max[current_ch] = pulse[current_ch];
        Servo_SetPulse(current_ch, pulse[current_ch]);
        UI_UpdateValues();
    }
}

static void App_Init(void)
{
    uint8_t ch;

    OLED_Init();              /* 内部完成 PB8/PB9 软件 I2C 初始化 */

    Servo_InitPositions();    /* 先写各通道初始脉宽 */
    Servo_StartAll();         /* 再使能 PWM，首个脉冲即目标值 */

    current_ch = 0;
    key4_long  = 0;
    for (ch = 0; ch < CH_NUM; ch++)
    {
        pulse_min[ch] = servo_home[ch];
        pulse_max[ch] = servo_home[ch];
        last_dir[ch]  = 0;
    }

    Button_Init();
    UI_DrawStatic();
    UI_UpdateValues();
}

void Test(void)
{
    App_Init();

    while (1)
    {
        App_Loop();
    }
}

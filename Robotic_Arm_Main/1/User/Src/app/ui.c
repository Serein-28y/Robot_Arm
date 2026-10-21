/**
 * @file    ui.c
 * @layer   App（应用层，显示）
 * @brief   OLED 菜单页 + HOST 状态页，按变化区域刷新。
 * @deps    app/ui.h, drivers/display/oled.h
 * @callers app.c（UI_Init / UI_SetState / UI_Task）
 *
 * 页面：
 *   MENU：模式选择提示，KEY2 进入 HOST。
 *   HOST：状态字 + X/Y/Z/P + 四个当前关节角 + 错误码。
 *
 * 刷新策略：
 *   - app.c 每 50ms 调 UI_SetState() 提交快照，再调 UI_Task()。
 *   - 页面切换整屏刷新；同页只更新变化的数值区域，减少软件 I2C 阻塞。
 */
#include "app/ui.h"
#include "drivers/display/oled.h"

#define UI_PAGE_MENU (0)
#define UI_PAGE_HOST (1)

#define UI_STATUS_RUN     (0)
#define UI_STATUS_HOLD    (1)
#define UI_STATUS_ERR     (2)
#define UI_STATUS_TIMEOUT (3)
#define UI_STATUS_STOP    (4)

/* 上一次真正写到屏幕上的值；用于比较是否需要局部刷新。 */
typedef struct
{
    int32_t x;
    int32_t y;
    int32_t z;
    int32_t phi;
    int32_t angle[J_NUM];
    uint8_t status;
    uint8_t err;
} PrevState;

static UiState   s_state;              /* app 提交的最新快照 */
static PrevState s_prev;               /* 屏幕当前显示值 */
static uint8_t   s_page = 0xFF;        /* 当前页；0xFF 表示尚未绘制 */
static uint8_t   s_ready;
static uint8_t   s_dirty;              /* 1=有未刷新的新快照 */

/** @brief 四舍五入到整数，用于 OLED 显示。 */
static int32_t round_i(float value)
{
    if (value >= 0.0f)
    {
        return (int32_t)(value + 0.5f);
    }

    return (int32_t)(value - 0.5f);
}

/** @brief 根据快照决定状态字优先级：急停 > 超时 > 错误 > 运动中 > 待机。 */
static uint8_t status_code(void)
{
    if (s_state.estop)
    {
        return UI_STATUS_STOP;
    }
    if (s_state.timeout)
    {
        return UI_STATUS_TIMEOUT;
    }
    if (s_state.err)
    {
        return UI_STATUS_ERR;
    }
    if (s_state.moving)
    {
        return UI_STATUS_RUN;
    }

    return UI_STATUS_HOLD;
}

/** @brief 状态字 -> 4 字符固定宽度文本，便于覆盖旧内容。 */
static char *status_text(uint8_t status)
{
    switch (status)
    {
        case UI_STATUS_RUN:
            return "RUN ";
        case UI_STATUS_ERR:
            return "ERR ";
        case UI_STATUS_TIMEOUT:
            return "TO  ";
        case UI_STATUS_STOP:
            return "STOP";
        default:
            return "HOLD";
    }
}

/** @brief 绘制菜单页（整屏）。 */
static void draw_menu(void)
{
    OLED_Clear();
    OLED_ShowString(0, 0,  "ROBOT ARM", OLED_8X16);
    OLED_ShowString(0, 16, "MODE1 HOST", OLED_8X16);
    OLED_ShowString(0, 32, "KEY2 ENTER", OLED_8X16);
    OLED_ShowString(0, 48, "K3 HOME K4 STOP", OLED_8X16);
    OLED_Update();
}

/** @brief 绘制 HOST 页固定标签；数值由 draw_values() 填充。 */
static void draw_host_static(void)
{
    OLED_Clear();
    OLED_ShowString(0, 0,  "HOST", OLED_8X16);
    OLED_ShowString(0, 16, "X", OLED_8X16);
    OLED_ShowString(48, 16, "Y", OLED_8X16);
    OLED_ShowString(0, 32, "Z", OLED_8X16);
    OLED_ShowString(48, 32, "P", OLED_8X16);
    OLED_ShowString(0, 48, "A", OLED_6X8);
}

/**
 * @brief 绘制/刷新 HOST 数值。
 * @param force  1=忽略比较，全部重画（页面切换时用）
 * @param update 1=画完立即 OLED_UpdateArea；0=只写显示缓冲，由调用者整屏 Update
 */
static void draw_values(uint8_t force, uint8_t update)
{
    int32_t x;
    int32_t y;
    int32_t z;
    int32_t phi;
    int32_t angle[J_NUM];
    uint8_t status;
    uint8_t i;

    x   = round_i(s_state.pose.x);
    y   = round_i(s_state.pose.y);
    z   = round_i(s_state.pose.z);
    phi = round_i(s_state.pose.phi);

    for (i = 0; i < J_NUM; i++)
    {
        angle[i] = round_i(s_state.theta[i]);
    }

    status = status_code();

    /* 坐标区：X/Y 第一行，Z/P 第二行；每个字段固定宽度，覆盖旧值。 */
    if (force || x != s_prev.x)
    {
        OLED_ShowSignedNum(8, 16, x, 3, OLED_8X16);
        if (update)
        {
            OLED_UpdateArea(8, 16, 32, 16);
        }
    }

    if (force || y != s_prev.y)
    {
        OLED_ShowSignedNum(56, 16, y, 3, OLED_8X16);
        if (update)
        {
            OLED_UpdateArea(56, 16, 32, 16);
        }
    }

    if (force || z != s_prev.z)
    {
        OLED_ShowSignedNum(8, 32, z, 3, OLED_8X16);
        if (update)
        {
            OLED_UpdateArea(8, 32, 32, 16);
        }
    }

    if (force || phi != s_prev.phi)
    {
        OLED_ShowSignedNum(56, 32, phi, 3, OLED_8X16);
        if (update)
        {
            OLED_UpdateArea(56, 32, 32, 16);
        }
    }

    /* 状态字 + 错误码。 */
    if (force || status != s_prev.status || s_state.err != s_prev.err)
    {
        OLED_ShowString(40, 0, status_text(status), OLED_8X16);

        if (s_state.err)
        {
            OLED_ShowNum(88, 0, s_state.err, 2, OLED_8X16);
        }
        else
        {
            OLED_ShowString(88, 0, "  ", OLED_8X16);
        }

        if (update)
        {
            OLED_UpdateArea(40, 0, 64, 16);
        }
    }

    /* 四角度：6x8 字体，每个 3 位数字 + 符号，等宽排列。 */
    for (i = 0; i < J_NUM; i++)
    {
        if (force || angle[i] != s_prev.angle[i])
        {
            OLED_ShowSignedNum((uint8_t)(8 + 24 * i), 48, angle[i], 3, OLED_6X8);
            if (update)
            {
                OLED_UpdateArea((uint8_t)(8 + 24 * i), 48, 24, 8);
            }
        }
    }

    /* 记录本帧显示值，供下一次比较。 */
    s_prev.x      = x;
    s_prev.y      = y;
    s_prev.z      = z;
    s_prev.phi    = phi;
    s_prev.status = status;
    s_prev.err    = s_state.err;

    for (i = 0; i < J_NUM; i++)
    {
        s_prev.angle[i] = angle[i];
    }
}

/**
 * @brief 初始化 OLED、清空状态并绘制菜单页。
 * @note  必须在 App_Init 中、各模块初始化之后调用。
 */
void UI_Init(void)
{
    uint8_t i;

    OLED_Init();

    s_state.mode    = UI_PAGE_MENU;
    s_state.moving  = 0;
    s_state.err     = 0;
    s_state.timeout = 0;
    s_state.estop   = 0;
    s_state.pose.x   = 0.0f;
    s_state.pose.y   = 0.0f;
    s_state.pose.z   = 0.0f;
    s_state.pose.phi = 0.0f;

    for (i = 0; i < J_NUM; i++)
    {
        s_state.theta[i] = 0.0f;
        s_prev.angle[i]  = 0;
    }

    s_prev.x      = 0;
    s_prev.y      = 0;
    s_prev.z      = 0;
    s_prev.phi    = 0;
    s_prev.status = 0xFF;   /* 强制首次绘制状态区 */
    s_prev.err    = 0xFF;

    s_dirty = 0;
    s_ready = 1;
    s_page  = UI_PAGE_MENU;

    draw_menu();
}

/** @brief 提交新快照；真正刷屏在 UI_Task()。 */
void UI_SetState(const UiState *s)
{
    if (s == 0)
    {
        return;
    }

    s_state = *s;
    s_dirty = 1;
}

/**
 * @brief 50ms 刷新任务。
 * @note  页面切换整屏刷新；同页只刷变化区域。
 */
void UI_Task(void)
{
    uint8_t page;

    if (!s_dirty)
    {
        return;
    }

    s_dirty = 0;

    page = (s_state.mode == UI_PAGE_HOST) ? UI_PAGE_HOST : UI_PAGE_MENU;

    if (!s_ready || page != s_page)
    {
        if (page == UI_PAGE_MENU)
        {
            draw_menu();
        }
        else
        {
            draw_host_static();
            draw_values(1, 0);   /* 只写缓冲 */
            OLED_Update();       /* 整屏发送一次 */
        }

        s_page  = page;
        s_ready = 1;
        return;
    }

    if (page == UI_PAGE_HOST)
    {
        draw_values(0, 1);
    }
}

#include "pwm6.h"
#include <string.h>

/* 依赖 CubeMX 生成的句柄 */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

/* 简单的通道映射表 */
typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
} PwmBind;

/* 简单的通道映射表：1..6 分别对应 TIM1_CH1~CH4、TIM2_CH1~CH2 */
static PwmBind s_map[PWM6_NUM_CHANNELS] = {
    {&htim1, TIM_CHANNEL_1}, // ch1 -> TIM1_CH1
    {&htim1, TIM_CHANNEL_2}, // ch2 -> TIM1_CH2
    {&htim1, TIM_CHANNEL_3}, // ch3 -> TIM1_CH3
    {&htim1, TIM_CHANNEL_4}, // ch4 -> TIM1_CH4   <-- 新增映射到 TIM1_CH4
    {&htim2, TIM_CHANNEL_1}, // ch5 -> TIM2_CH1   <-- 改到 TIM2_CH1
    {&htim2, TIM_CHANNEL_2}, // ch6 -> TIM2_CH2   <-- 改到 TIM2_CH2
};

/* 取定时器时钟（Hz）——F1 的“定时器 x2”规则：APB prescaler >1 时，TIMCLK=2*APB */
static uint32_t _get_timer_clk_hz(TIM_HandleTypeDef *h)
{
    /* 依据实例判断属于 APB1 还是 APB2 */
    uint32_t pclk, ppre, timclk;
    if (h->Instance == TIM1) {
        pclk = HAL_RCC_GetPCLK2Freq();
        /* 读取 APB2 分频因子 */
        RCC_ClkInitTypeDef c;
        uint32_t f;
        HAL_RCC_GetClockConfig(&c, &f);
        ppre   = c.APB2CLKDivider;
        timclk = (ppre == RCC_HCLK_DIV1) ? pclk : (pclk * 2U);
    } else {
        pclk = HAL_RCC_GetPCLK1Freq();
        RCC_ClkInitTypeDef c;
        uint32_t f;
        HAL_RCC_GetClockConfig(&c, &f);
        ppre   = c.APB1CLKDivider;
        timclk = (ppre == RCC_HCLK_DIV1) ? pclk : (pclk * 2U);
    }
    return timclk;
}

static inline int _valid(uint8_t ch)
{
    return ch >= 1 && ch <= PWM6_NUM_CHANNELS;
}

void PWM6_Init(void)
{
    /* 此处无需做太多——定时器已由 MX_TIMx_Init 配好，
       我们只是在运行时计算时钟与 ARR 时需要它们 */
}

void PWM6_StartAll(void)
{
    for (uint8_t i = 1; i <= PWM6_NUM_CHANNELS; ++i) {
        PWM6_Start(i);
    }
}

void PWM6_StopAll(void)
{
    for (uint8_t i = 1; i <= PWM6_NUM_CHANNELS; ++i) {
        PWM6_Stop(i);
    }
}

void PWM6_Start(uint8_t ch)
{
    if (!_valid(ch)) return;
    PwmBind *b = &s_map[ch - 1];
    HAL_TIM_PWM_Start(b->htim, b->channel);
}

void PWM6_Stop(uint8_t ch)
{
    if (!_valid(ch)) return;
    PwmBind *b = &s_map[ch - 1];
    HAL_TIM_PWM_Stop(b->htim, b->channel);
}

uint32_t PWM6_GetTimerClkHz(uint8_t ch)
{
    if (!_valid(ch)) return 0;
    return _get_timer_clk_hz(s_map[ch - 1].htim);
}

uint32_t PWM6_GetARR(uint8_t ch)
{
    if (!_valid(ch)) return 0;
    TIM_HandleTypeDef *h = s_map[ch - 1].htim;
    return __HAL_TIM_GET_AUTORELOAD(h) + 1U; // ARR 寄存器 + 1 = 计数周期
}

void PWM6_SetDutyPercent(uint8_t ch, float duty_percent)
{
    if (!_valid(ch)) return;
    if (duty_percent < 0.0f) duty_percent = 0.0f;
    if (duty_percent > 100.0f) duty_percent = 100.0f;

    PwmBind *b         = &s_map[ch - 1];
    uint32_t arr_plus1 = __HAL_TIM_GET_AUTORELOAD(b->htim) + 1U;
    uint32_t ccr       = (uint32_t)((duty_percent * arr_plus1) / 100.0f);
    __HAL_TIM_SET_COMPARE(b->htim, b->channel, ccr);
}

void PWM6_SetPulseUs(uint8_t ch, uint32_t pulse_us)
{
    if (!_valid(ch)) return;
    PwmBind *b = &s_map[ch - 1];

    uint32_t timclk = _get_timer_clk_hz(b->htim);  // 定时器时钟 Hz
    uint32_t psc    = b->htim->Init.Prescaler + 1; // 预分频
    /* 计数器频率 = timclk / psc */
    uint32_t cnt_hz = timclk / psc;

    /* 将微秒换算成“计数值” */
    uint32_t ticks = (uint32_t)(((uint64_t)cnt_hz * pulse_us) / 1000000ULL);

    /* 限制不超过 ARR */
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(b->htim);
    if (ticks > arr) ticks = arr;

    __HAL_TIM_SET_COMPARE(b->htim, b->channel, ticks);
}

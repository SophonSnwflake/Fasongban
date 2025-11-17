#include "icx.h" // 引入本模块对应的头文件（结构体/原型）

// 绑定（附着）一个定时器句柄到 ICX 抽象句柄，并记录基础通道和计数时钟
void ICX_Attach(ICX_Handle *icx, TIM_HandleTypeDef *htim, uint32_t base_ch, uint32_t tick_hz)
{
    icx->htim    = htim;    // 保存 HAL 的定时器句柄（哪一个 TIMx）
    icx->base_ch = base_ch; // 保存“基准通道”：约定 base_ch=CH1 → CCR1=周期, CCR2=高电平
    icx->tick_hz = tick_hz; // 保存计数时钟频率（计数器的 Hz），比如 1MHz
}

// 启动输入捕获：这里同时启动 CH1 与 CH2（分别用来取周期和高电平）
HAL_StatusTypeDef ICX_Start(ICX_Handle *icx)
{
    HAL_StatusTypeDef s1 = HAL_TIM_IC_Start(icx->htim, TIM_CHANNEL_1); // 使能通道1输入捕获
    HAL_StatusTypeDef s2 = HAL_TIM_IC_Start(icx->htim, TIM_CHANNEL_2); // 使能通道2输入捕获
    return (s1 == HAL_OK && s2 == HAL_OK) ? HAL_OK : HAL_ERROR;        // 两个通道都成功才返回 OK
}

// 内联小工具：读某个通道的捕获比较寄存器（CCR）
static inline uint32_t _get_ccr(TIM_HandleTypeDef *htim, uint32_t ch)
{
    return __HAL_TIM_GET_COMPARE(htim, ch); // 读取 CCRx（x 由 ch 指定），单位是“计数器tick数”
}

// 计算频率：f = tick_hz / 周期tick
uint32_t ICX_GetFreq(const ICX_Handle *icx)
{
    // 约定 base_ch=CH1 → CCR1 存的是整周期（相邻上升沿之间的 tick 数）
    uint32_t period = _get_ccr(icx->htim, TIM_CHANNEL_1); // 读 CCR1：周期的 tick 计数
    if (period == 0) return 0;                            // 防止除零（还未捕获到有效周期时返回 0）
    return (uint32_t)(icx->tick_hz / period);             // 频率(Hz) = 计数频率 / 周期tick
}

// 计算占空比：D(%) = 高电平tick / 周期tick * 100
uint32_t ICX_GetDuty(const ICX_Handle *icx)
{
    // 约定 base_ch=CH1 → CCR2 存的是一个周期内的高电平持续 tick
    uint32_t period = _get_ccr(icx->htim, TIM_CHANNEL_1); // 读 CCR1：周期tick
    uint32_t high   = _get_ccr(icx->htim, TIM_CHANNEL_2); // 读 CCR2：高电平tick
    if (period == 0) return 0;                            // 防止除零（无有效周期）
    return (high * 100U) / period;                        // 返回占空比(整数%) = high/period * 100
}

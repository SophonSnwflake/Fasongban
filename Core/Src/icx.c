/**
 * @file    icx.c
 * @brief   通用 PWM 频率/占空比测量模块（基于 HAL 输入捕获）
 *
 * @details
 * 本模块对“定时器 + 输入捕获（PWMI 模式）”做了一层轻封装：
 *
 *   - 使用一个定时器的两个通道（例如：CH1=周期、CH2=高电平时间）；
 *   - 通过 ICX_GetFreq() 计算输入信号频率（Hz）；
 *   - 通过 ICX_GetDuty() 计算输入信号占空比（整数 %）。
 *
 * 使用前提（典型配置）：
 *   1. 在 CubeMX 中配置某个 TIMx，开启两个输入捕获通道（通常 CH1/CH2），
 *      并将其设置为 PWM Input 模式（即 PWMI）。
 *   2. 确保定时器的计数时钟频率已知，例如：
 *        - 通过手算：TIMCLK / (PSC + 1)
 *        - 或使用一个统一的“tick_hz”定义，例如 1 MHz。
 *   3. 在 main() 或初始化代码中调用：
 *        ICX_Attach(&ic3, &htim3, TIM_CHANNEL_1, TICK_HZ);
 *        ICX_Start(&ic3);
 *      然后即可在循环中调用 ICX_GetFreq(&ic3)、ICX_GetDuty(&ic3)。
 */

#include "icx.h" // 引入本模块对应的头文件（ICX_Handle 结构体 / 函数原型）

/*============================================================================
 *                           绑定 & 启动
 *===========================================================================*/

/**
 * @brief  将一个定时器句柄“附着”到 ICX 抽象句柄上。
 *
 * @param  icx      ICX 抽象句柄指针（由上层静态/全局定义）。
 * @param  htim     HAL 定时器句柄（例如 &htim3）。
 * @param  base_ch  基准通道（目前实现中假定为 TIM_CHANNEL_1）：
 *                  - 约定：base_ch = TIM_CHANNEL_1 →
 *                      CCR1 = 周期（一个信号周期的 tick 数）；
 *                      CCR2 = 高电平时间（一个周期内，高电平持续的 tick 数）。
 *                  - 你也可以扩展为其它通道，只需相应修改内部的 _get_ccr 使用方式。
 * @param  tick_hz  定时器计数时钟频率，单位 Hz。
 *                  例如：若 TIM3 计数频率为 1 MHz，则传入 1000000。
 *
 * @note   此函数仅保存关联信息，本身不启动定时器。
 *         启动捕获请调用 ICX_Start()。
 */
void ICX_Attach(ICX_Handle *icx,
                TIM_HandleTypeDef *htim,
                uint32_t base_ch,
                uint32_t tick_hz)
{
    icx->htim    = htim;    // 保存 HAL 的定时器句柄（指向具体某个 TIMx）
    icx->base_ch = base_ch; // 保存“基准通道”，当前约定 base_ch=CH1 → CCR1=周期, CCR2=高电平
    icx->tick_hz = tick_hz; // 保存计数时钟频率（计数器的 Hz），例如 1 MHz
}

/**
 * @brief  启动 ICX 对应定时器的输入捕获功能。
 *
 * @param  icx  已通过 ICX_Attach() 绑定好的 ICX 句柄。
 *
 * @retval HAL_OK   两个通道都成功启动。
 * @retval HAL_ERROR 至少有一个通道启动失败。
 *
 * @note   当前实现直接启动 CH1 与 CH2：
 *            - CH1 负责“整周期”测量；
 *            - CH2 负责“高电平时间”测量。
 *         因此你的 CubeMX 配置中需要：
 *            - TIMx_CH1、TIMx_CH2 都配置为输入捕获（通常为 PWMI）。
 */
HAL_StatusTypeDef ICX_Start(ICX_Handle *icx)
{
    HAL_StatusTypeDef s1 =
        HAL_TIM_IC_Start(icx->htim, TIM_CHANNEL_1); // 使能通道 1 输入捕获
    HAL_StatusTypeDef s2 =
        HAL_TIM_IC_Start(icx->htim, TIM_CHANNEL_2); // 使能通道 2 输入捕获

    // 两个通道都成功启动才返回 OK
    return (s1 == HAL_OK && s2 == HAL_OK) ? HAL_OK : HAL_ERROR;
}

/*============================================================================
 *                          内部小工具函数
 *===========================================================================*/

/**
 * @brief  读取指定通道的捕获比较寄存器（CCR）。
 *
 * @param  htim  HAL 定时器句柄。
 * @param  ch    通道号（TIM_CHANNEL_1 / 2 / 3 / 4）。
 *
 * @retval 当前 CCR 寄存器值，单位是“计数器 tick 数”。
 *
 * @note   该函数只是一层包装，实际使用了 HAL 提供的宏：
 *           __HAL_TIM_GET_COMPARE(htim, ch)
 */
static inline uint32_t _get_ccr(TIM_HandleTypeDef *htim, uint32_t ch)
{
    return __HAL_TIM_GET_COMPARE(htim, ch); // 返回 CCRx，x 由 ch 指定
}

/*============================================================================
 *                         频率 / 占空比 查询接口
 *===========================================================================*/

/**
 * @brief  获取输入信号的频率（Hz）。
 *
 * @param  icx  已附着且已经启动的 ICX 句柄。
 *
 * @retval 频率值，单位 Hz。若当前周期为 0（未捕获到有效周期），返回 0。
 *
 * @details
 * 公式：
 *     周期 tick 数：period = CCR1
 *     定时器计数频率：tick_hz
 *     频率：f = tick_hz / period
 *
 * 例如：
 *     若 tick_hz = 1,000,000 Hz（计数周期 1 us），
 *     且 CCR1 = 20,000，则周期为 20,000 us = 20 ms，对应 50 Hz。
 */
uint32_t ICX_GetFreq(const ICX_Handle *icx)
{
    // 当前约定 base_ch=CH1 → CCR1 存的是整周期（相邻上升沿之间的 tick 数）
    uint32_t period = _get_ccr(icx->htim, TIM_CHANNEL_1); // 读 CCR1：周期的 tick 计数
    if (period == 0U) {
        // 防止除零（还未捕获到有效周期或输入信号丢失时返回 0）
        return 0U;
    }

    // 频率(Hz) = 计数频率 / 周期tick
    return (uint32_t)(icx->tick_hz / period);
}

/**
 * @brief  获取输入信号的占空比（整数百分比）。
 *
 * @param  icx  已附着且已经启动的 ICX 句柄。
 *
 * @retval 占空比（0~100 之间的整数）。若周期为 0，则返回 0。
 *
 * @details
 * 公式：
 *     周期 tick 数：period = CCR1
 *     高电平 tick 数：high = CCR2
 *     占空比：D(%) = high / period * 100
 *
 * @note   为了简化，返回值为整数百分比（uint32_t），
 *         若需要更高精度，可在此基础上做浮点运算扩展。
 */
uint32_t ICX_GetDuty(const ICX_Handle *icx)
{
    // 约定 base_ch=CH1 → CCR1=周期间隔，CCR2=高电平持续 tick 数
    uint32_t period = _get_ccr(icx->htim, TIM_CHANNEL_1); // CCR1：周期 tick
    uint32_t high   = _get_ccr(icx->htim, TIM_CHANNEL_2); // CCR2：高电平 tick

    if (period == 0U) {
        // 防止除零（尚无有效周期，返回 0）
        return 0U;
    }

    // 返回占空比(整数%) = high / period * 100
    return (high * 100U) / period;
}

#include "fire.h"
#include "gpio.h"
#include "main.h"
#include "rc_process.h"

// 发射 GPIO：PA2 （高电平触发）
#define FIRE_GPIO_PORT GPIOA
#define FIRE_GPIO_PIN GPIO_PIN_2

// 10 发/秒 → 最快间隔 = 100ms
#define FIRE_INTERVAL_MS 50

// 保存上一次开关状态
static uint8_t last_switch = 0;

// 上次发射时间戳
static uint32_t last_fire_tick = 0;

// 短脉冲时间（发射触发高电平维持 15ms）
#define FIRE_PULSE_MS 50
static uint32_t fire_pulse_end_tick = 0;

/**
 * @brief 初始化发射 GPIO（PA2 输出）
 */
void Fire_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = FIRE_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(FIRE_GPIO_PORT, &GPIO_InitStruct);

    // 默认低电平
    HAL_GPIO_WritePin(FIRE_GPIO_PORT, FIRE_GPIO_PIN, GPIO_PIN_RESET);
}

/**
 * @brief 触发一次发射（内部使用）
 */
static void Fire_Trigger(void)
{
    uint32_t now = HAL_GetTick();

    // 功率节流：100ms 内只能打一发
    if (now - last_fire_tick < FIRE_INTERVAL_MS)
        return;

    last_fire_tick = now;

    // 输出高电平 → 发射
    HAL_GPIO_WritePin(FIRE_GPIO_PORT, FIRE_GPIO_PIN, GPIO_PIN_SET);

    // 记录脉冲结束时间
    fire_pulse_end_tick = now + FIRE_PULSE_MS;
}

/**
 * @brief 主控制循环（每帧 200Hz 调用）
 */
void Fire_ControlLoop(void)
{
    uint32_t now = HAL_GetTick();

    // 1) 若处于发射高电平时间，检查是否该恢复低电平
    if (fire_pulse_end_tick != 0)
    {
        if (now >= fire_pulse_end_tick)
        {
            HAL_GPIO_WritePin(FIRE_GPIO_PORT, FIRE_GPIO_PIN, GPIO_PIN_RESET);
            fire_pulse_end_tick = 0;
        }
    }

    // 2) 检测 d5（rc.fire_switch）的边沿
    uint8_t cur = rc.fire_switch;

    // 上升沿：0 → 1
    if (last_switch == 0 && cur == 1)
    {
        Fire_Trigger();
    }

    // 下降沿：1 → 0
    if (last_switch == 1 && cur == 0)
    {
        Fire_Trigger();
    }

    last_switch = cur;
}

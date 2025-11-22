#include "fire.h"
#include "gpio.h"
#include "main.h"
#include "rc_process.h"

// 发射 GPIO：PA2
#define FIRE_GPIO_PORT GPIOA
#define FIRE_GPIO_PIN GPIO_PIN_2

// 单发脉冲长度
#define FIRE_PULSE_MS 20

// d5 值
#define FIRE_MODE_LOW 85
#define FIRE_MODE_MID 127
#define FIRE_MODE_AUTO 1

static uint8_t last_d5 = FIRE_MODE_LOW;

// 单发脉冲结束时间戳
static uint32_t fire_pulse_end_tick = 0;

// 连发模式标志（d5=1 时）
static uint8_t auto_fire = 0;

/**
 * @brief 初始化 PA2
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
 * @brief 打单发（高电平维持 20ms）
 */
static void Fire_SingleShot(void)
{
    uint32_t now = HAL_GetTick();

    HAL_GPIO_WritePin(FIRE_GPIO_PORT, FIRE_GPIO_PIN, GPIO_PIN_SET);
    fire_pulse_end_tick = now + FIRE_PULSE_MS;
}

/**
 * @brief 每帧调用（200Hz）
 */
void Fire_ControlLoop(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t d5 = rc.fire_switch; // 实际为 1 / 85 / 127

    /************************************************************
     * ① 如果连发模式（d5 == 1）
     *    → 直接让 PA2 一直高电平
     ************************************************************/
    if (d5 == FIRE_MODE_AUTO)
    {
        auto_fire = 1;
        HAL_GPIO_WritePin(FIRE_GPIO_PORT, FIRE_GPIO_PIN, GPIO_PIN_SET);

        last_d5 = d5;
        return;
    }

    /************************************************************
     * ② 离开连发模式（d5 != 1）
     *    → 必须关闭高电平
     ************************************************************/
    if (auto_fire && d5 != FIRE_MODE_AUTO)
    {
        auto_fire = 0;
        HAL_GPIO_WritePin(FIRE_GPIO_PORT, FIRE_GPIO_PIN, GPIO_PIN_RESET);
    }

    /************************************************************
     * ③ 单发模式：85 ↔ 127 切换时打一发
     ************************************************************/
    if ((last_d5 == FIRE_MODE_LOW && d5 == FIRE_MODE_MID) ||
        (last_d5 == FIRE_MODE_MID && d5 == FIRE_MODE_LOW))
    {
        Fire_SingleShot();
    }

    last_d5 = d5;

    /************************************************************
     * ④ 单发脉冲结束计时
     ************************************************************/
    if (fire_pulse_end_tick && now >= fire_pulse_end_tick)
    {
        HAL_GPIO_WritePin(FIRE_GPIO_PORT, FIRE_GPIO_PIN, GPIO_PIN_RESET);
        fire_pulse_end_tick = 0;
    }
}

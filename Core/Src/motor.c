/**
 * @file    motor.c
 * @brief   直流电机方向与占空比控制模块（基于 HAL + pwm6）
 *
 * @details
 * 本模块实现：
 *   - 通过两路方向引脚（IN1/IN2）控制 H 桥，实现每路电机的正转/反转。
 *   - 通过 pwm6 模块（PWM6_SetDutyPercent）设置电机 PWM 占空比。
 *
 * 电机编号与 PWM 通道映射：
 *   - 电机 1 → PWM 通道 1
 *   - 电机 2 → PWM 通道 2
 *   - 电机 3 → PWM 通道 3
 *   - 电机 4 → PWM 通道 4
 *
 * 使用前提：
 *   1. 已在 CubeMX 中，将 Mx_INy 对应的引脚配置为推挽输出（GPIO Output, Push-Pull）。
 *   2. TIM1/TIM2 已在 CubeMX 中配置为 PWM 输出，且 pwm6.c 中已对定时器完成初始化。
 *   3. 在 main() 中需调用：
 *        PWM6_Init();
 *        PWM6_StartAll();     // 或 PWM6_Start(1..4)
 *      然后再调用 Motor_SetPWM() 控制电机。
 */

#include "motor.h"
#include "gpio.h" // CubeMX 生成的 GPIO 引脚定义与 HAL_GPIO_WritePin 等函数
#include "pwm6.h" // 自封装的 6 路 PWM 模块（提供 PWM6_SetDutyPercent 接口）

/*============================================================================
 *                         硬件引脚映射（方向控制）
 *===========================================================================*/
/**
 * @note
 *  每个电机有两路方向控制引脚 IN1 / IN2：
 *    - IN1 = 1, IN2 = 0 → 正转
 *    - IN1 = 0, IN2 = 1 → 反转
 *  具体正反含义依赖于你的 H 桥驱动和电机接线方向，如有需要可以互换。
 *
 *  若后期 PCB 改动，只需修改下列宏，不需要改 Motor_SetPWM 的逻辑。
 */

/* 电机 1 方向引脚 */
#define M1_IN1_GPIO GPIOB
#define M1_IN1_PIN  GPIO_PIN_0
#define M1_IN2_GPIO GPIOB
#define M1_IN2_PIN  GPIO_PIN_1

/* 电机 2 方向引脚 */
#define M2_IN1_GPIO GPIOB
#define M2_IN1_PIN  GPIO_PIN_10
#define M2_IN2_GPIO GPIOB
#define M2_IN2_PIN  GPIO_PIN_11

/* 电机 3 方向引脚 */
#define M3_IN1_GPIO GPIOC
#define M3_IN1_PIN  GPIO_PIN_13
#define M3_IN2_GPIO GPIOC
#define M3_IN2_PIN  GPIO_PIN_14

/* 电机 4 方向引脚 */
#define M4_IN1_GPIO GPIOC
#define M4_IN1_PIN  GPIO_PIN_15
#define M4_IN2_GPIO GPIOB
#define M4_IN2_PIN  GPIO_PIN_7

/*============================================================================
 *                             内部工具函数
 *===========================================================================*/

/**
 * @brief  求 16 位有符号数的绝对值。
 * @param  x  输入的有符号 16 位数。
 * @retval 无符号 16 位数，数值为 |x|。
 */
static inline uint16_t _abs16(int16_t x)
{
    return (uint16_t)(x >= 0 ? x : -x);
}

/*============================================================================
 *                         对外电机控制接口实现
 *===========================================================================*/

/**
 * @brief  控制指定电机的转向与占空比。
 *
 * @param  n    电机编号：
 *              - 1 ：电机 1（使用 PWM 通道 1）
 *              - 2 ：电机 2（使用 PWM 通道 2）
 *              - 3 ：电机 3（使用 PWM 通道 3）
 *              - 4 ：电机 4（使用 PWM 通道 4）
 *              传入其它编号时不做任何操作。
 *
 * @param  Duty 电机功率指令（有符号）：
 *              - 符号表示方向：
 *                  Duty >= 0 → 正转；
 *                  Duty <  0 → 反转。
 *              - 绝对值表示占空比幅度，推荐工程使用范围：
 *                  -100 ~ +100  → 对应 0% ~ 100% 占空比。
 *                但本函数不会强制限制，amp 直接使用 |Duty|。
 *
 * @note   内部实现策略：
 *              1. 先计算 amp = |Duty| 作为占空比百分数传给 PWM6_SetDutyPercent()。
 *              2. 再根据 Duty 的正负调整 Mx_IN1 / Mx_IN2 引脚，实现电机正反转。
 *
 * @warning 使用前请确保：
 *              - 已完成 GPIO 和定时器（pwm6）的初始化与启动；
 *              - H 桥驱动板与单片机的电平兼容且供电合理。
 */
void Motor_SetPWM(uint8_t n, int16_t Duty)
{
    /* 计算占空比幅度 = |Duty|。此处未做上限裁剪，如需限制可在上层或此处加上。 */
    uint16_t amp = _abs16(Duty);

    if (n == 1) {
        /*----------------------------- 电机 1 -----------------------------*/
        if (Duty >= 0) {
            /* 正转：IN1 = 1, IN2 = 0 */
            HAL_GPIO_WritePin(M1_IN1_GPIO, M1_IN1_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(M1_IN2_GPIO, M1_IN2_PIN, GPIO_PIN_RESET);
            PWM6_SetDutyPercent(1, amp); // 对应 PWM 通道 1
        } else {
            /* 反转：IN1 = 0, IN2 = 1 */
            HAL_GPIO_WritePin(M1_IN1_GPIO, M1_IN1_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(M1_IN2_GPIO, M1_IN2_PIN, GPIO_PIN_SET);
            PWM6_SetDutyPercent(1, amp);
        }

    } else if (n == 2) {
        /*----------------------------- 电机 2 -----------------------------*/
        if (Duty >= 0) {
            /* 正转：IN1 = 1, IN2 = 0 */
            HAL_GPIO_WritePin(M2_IN1_GPIO, M2_IN1_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(M2_IN2_GPIO, M2_IN2_PIN, GPIO_PIN_RESET);
            PWM6_SetDutyPercent(2, amp); // 对应 PWM 通道 2
        } else {
            /* 反转：IN1 = 0, IN2 = 1 */
            HAL_GPIO_WritePin(M2_IN1_GPIO, M2_IN1_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(M2_IN2_GPIO, M2_IN2_PIN, GPIO_PIN_SET);
            PWM6_SetDutyPercent(2, amp);
        }

    } else if (n == 3) {
        /*----------------------------- 电机 3 -----------------------------*/
        if (Duty >= 0) {
            /* 正转：IN1 = 1, IN2 = 0 */
            HAL_GPIO_WritePin(M3_IN1_GPIO, M3_IN1_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(M3_IN2_GPIO, M3_IN2_PIN, GPIO_PIN_RESET);
            PWM6_SetDutyPercent(3, amp); // 对应 PWM 通道 3
        } else {
            /* 反转：IN1 = 0, IN2 = 1 */
            HAL_GPIO_WritePin(M3_IN1_GPIO, M3_IN1_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(M3_IN2_GPIO, M3_IN2_PIN, GPIO_PIN_SET);
            PWM6_SetDutyPercent(3, amp);
        }

    } else if (n == 4) {
        /*----------------------------- 电机 4 -----------------------------*/
        if (Duty >= 0) {
            /* 正转：IN1 = 1, IN2 = 0 */
            HAL_GPIO_WritePin(M4_IN1_GPIO, M4_IN1_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(M4_IN2_GPIO, M4_IN2_PIN, GPIO_PIN_RESET);
            PWM6_SetDutyPercent(4, amp); // 对应 PWM 通道 4
        } else {
            /* 反转：IN1 = 0, IN2 = 1 */
            HAL_GPIO_WritePin(M4_IN1_GPIO, M4_IN1_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(M4_IN2_GPIO, M4_IN2_PIN, GPIO_PIN_SET);
            PWM6_SetDutyPercent(4, amp);
        }
    }

    /* 其它编号 (n != 1~4) 不处理；
       如将来扩展到 6 电机，可在此处继续增加 n == 5 / 6 的分支。*/
}

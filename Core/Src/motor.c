#include "motor.h"
#include "gpio.h" // 若你使用了 CubeMX 生成的 gpio.c/gpio.h
#include "pwm6.h"


// 方向引脚（保持与你旧工程一致）
#define M1_IN1_GPIO GPIOB
#define M1_IN1_PIN  GPIO_PIN_0
#define M1_IN2_GPIO GPIOB
#define M1_IN2_PIN  GPIO_PIN_1

#define M2_IN1_GPIO GPIOB
#define M2_IN1_PIN  GPIO_PIN_10
#define M2_IN2_GPIO GPIOB
#define M2_IN2_PIN  GPIO_PIN_11

#define M3_IN1_GPIO GPIOC
#define M3_IN1_PIN  GPIO_PIN_13
#define M3_IN2_GPIO GPIOC
#define M3_IN2_PIN  GPIO_PIN_14

#define M4_IN1_GPIO GPIOC
#define M4_IN1_PIN  GPIO_PIN_15
#define M4_IN2_GPIO GPIOB
#define M4_IN2_PIN  GPIO_PIN_7

static inline uint16_t _abs16(int16_t x)
{
    return (uint16_t)(x >= 0 ? x : -x);
}

// void Motor_Init(void)
// {
//     // 1) 使能 GPIOB 时钟（有的工程在 MX_GPIO_Init 里已做，这里再做一次也没问题）
//     __HAL_RCC_GPIOB_CLK_ENABLE();

//     // 2) 配置 PB12~PB15 为推挽输出（方向控制）
//     GPIO_InitTypeDef GPIO_InitStruct = {0};
//     GPIO_InitStruct.Pin              = M1_IN1_PIN | M1_IN2_PIN | M2_IN1_PIN | M2_IN2_PIN;
//     GPIO_InitStruct.Mode             = GPIO_MODE_OUTPUT_PP;
//     GPIO_InitStruct.Pull             = GPIO_NOPULL;
//     GPIO_InitStruct.Speed            = GPIO_SPEED_FREQ_HIGH;
//     HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

//     // 默认拉成“刹车/空闲”状态（可按你需要改成全低/全高）
//     HAL_GPIO_WritePin(M1_IN1_GPIO, M1_IN1_PIN, GPIO_PIN_RESET);
//     HAL_GPIO_WritePin(M1_IN2_GPIO, M1_IN2_PIN, GPIO_PIN_RESET);
//     HAL_GPIO_WritePin(M2_IN1_GPIO, M2_IN1_PIN, GPIO_PIN_RESET);
//     HAL_GPIO_WritePin(M2_IN2_GPIO, M2_IN2_PIN, GPIO_PIN_RESET);

//     // 3) 启动你的 PWM（保持你原先的接口）
//     PWM_Init(); // 由你已有的 HAL 版 PWM.c 提供
//     // 若需要：可在此调用 PWM_Start(TIM_CHANNEL_1/2) 之类的函数
// }

void Motor_SetPWM(uint8_t n, int16_t Duty)
{
    uint16_t amp = _abs16(Duty); // 占空比幅值 = |Duty|
    // （可选）这里也可以对 amp 做上限裁剪，比如限定在 [0, ARR]

    if (n == 1) {
        if (Duty >= 0) {
            // 电机1正转：IN1=1, IN2=0
            HAL_GPIO_WritePin(M1_IN1_GPIO, M1_IN1_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(M1_IN2_GPIO, M1_IN2_PIN, GPIO_PIN_RESET);
            PWM6_SetDutyPercent(1, amp); // CH1 占空比
        } else {
            // 电机1反转：IN1=0, IN2=1
            HAL_GPIO_WritePin(M1_IN1_GPIO, M1_IN1_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(M1_IN2_GPIO, M1_IN2_PIN, GPIO_PIN_SET);
            PWM6_SetDutyPercent(1, amp);
        }
    } else if (n == 2) {
        if (Duty >= 0) {
            // 电机2正转：IN1=1, IN2=0
            HAL_GPIO_WritePin(M2_IN1_GPIO, M2_IN1_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(M2_IN2_GPIO, M2_IN2_PIN, GPIO_PIN_RESET);
            PWM6_SetDutyPercent(2, amp); // CH2 占空比
        } else {
            // 电机2反转：IN1=0, IN2=1
            HAL_GPIO_WritePin(M2_IN1_GPIO, M2_IN1_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(M2_IN2_GPIO, M2_IN2_PIN, GPIO_PIN_SET);
            PWM6_SetDutyPercent(2, amp);
        }
    }
    else if(n ==3 )
    {
        if (Duty >= 0) {
            // 电机2正转：IN1=1, IN2=0
            HAL_GPIO_WritePin(M3_IN1_GPIO, M3_IN1_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(M3_IN2_GPIO, M3_IN2_PIN, GPIO_PIN_RESET);
            PWM6_SetDutyPercent(3, amp); // CH2 占空比
        } else {
            // 电机2反转：IN1=0, IN2=1
            HAL_GPIO_WritePin(M3_IN1_GPIO, M3_IN1_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(M3_IN2_GPIO, M3_IN2_PIN, GPIO_PIN_SET);
            PWM6_SetDutyPercent(3, amp);
        }
    } else if (n == 4) {
        if (Duty >= 0) {
            // 电机2正转：IN1=1, IN2=0
            HAL_GPIO_WritePin(M4_IN1_GPIO, M4_IN1_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(M4_IN2_GPIO, M4_IN2_PIN, GPIO_PIN_RESET);
            PWM6_SetDutyPercent(4, amp); // CH2 占空比
        } else {
            // 电机2反转：IN1=0, IN2=1
            HAL_GPIO_WritePin(M4_IN1_GPIO, M4_IN1_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(M4_IN2_GPIO, M4_IN2_PIN, GPIO_PIN_SET);
            PWM6_SetDutyPercent(4, amp);
        }
    } // 其它编号不处理；如需扩展可继续加分支
}

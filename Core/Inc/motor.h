#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "stm32f1xx_hal.h"

// // 初始化电机方向引脚（PB12~PB15），并调用你的 PWM_Init()
// void Motor_Init(void);

/**
 * @brief 设置电机 PWM 与方向
 * @param n    电机编号：1 或 2
 * @param Duty 有符号占空比：>=0 正转，<0 反转；幅值为占空比大小（单位与 PWM_SetCompareX 一致）
 *             例如：若定时器 ARR=999，则 Duty 的合理范围为 -999 ~ +999
 */
void Motor_SetPWM(uint8_t n, int16_t Duty);

#endif /* __MOTOR_H__ */

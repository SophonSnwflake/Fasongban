#ifndef PWM6_H
#define PWM6_H

#include "main.h"
#include "tim.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 逻辑通道映射：
 * 1→TIM1_CH1, 2→TIM1_CH2, 3→TIM1_CH3,
 * 4→TIM2_CH1, 5→TIM2_CH2, 6→TIM2_CH3
 */
#define PWM6_NUM_CHANNELS 6

/* 初始化：不创建定时器，只做参数读取与校验 */
void PWM6_Init(void);

/* 启动/停止所有通道（会调用 HAL_TIM_PWM_Start/Stop） */
void PWM6_StartAll(void);
void PWM6_StopAll(void);

/* 单独启动/停止某一路（1..6） */
void PWM6_Start(uint8_t ch);
void PWM6_Stop(uint8_t ch);

/* 设置占空比（0.0f~100.0f），超界会自动夹紧 */
void PWM6_SetDutyPercent(uint8_t ch, float duty_percent);

/* 设置脉宽（微秒）：依据当前定时器频率与 ARR 计算
   ——适用于 50Hz 舵机(500~2500us)等；两个定时器若频率不同也能各算各的 */
void PWM6_SetPulseUs(uint8_t ch, uint32_t pulse_us);

/* 读取当前通道的 ARR（Period+1）与定时器时钟（Hz） */
uint32_t PWM6_GetTimerClkHz(uint8_t ch);
uint32_t PWM6_GetARR(uint8_t ch);

#ifdef __cplusplus
}
#endif
#endif /* PWM6_H */

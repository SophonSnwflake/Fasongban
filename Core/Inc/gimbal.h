#ifndef __GIMBAL_H
#define __GIMBAL_H

#include <stdint.h>

/**
 * @brief 初始化云台舵机（Yaw + Pitch）
 */
void Gimbal_Init(void);

/**
 * @brief  云台控制主流程（每个控制周期调用）
 */
void Gimbal_ControlLoop(void);

#endif
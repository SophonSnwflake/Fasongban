#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "pid.h"
#include "spi_slave.h"

/**
 * @brief 麦克纳姆轮运动学计算（将底盘速度转换为四轮速度）
 * @param vx   前后速度（+前 / mm/s）
 * @param vy   左右速度（+左 / mm/s）
 * @param omega 旋转速度（+逆时针 / deg/s 或 rad/s）
 * @param wheel_out 四个轮子目标速度（mm/s）
 *
 * 注意：
 *   1) omega 是角速度，要统一单位（推荐用 rad/s）
 *   2) wheel_out[0] = FL
 *      wheel_out[1] = BL
 *      wheel_out[2] = BR
 *      wheel_out[3] = FR
 */
void Calc_Mecanum(float vx, float vy, float omega, float wheel_out[4]);

// 四轮 PID
void Chassis_PID_Init(void);
void Update_4Wheel_PID(float target[4], float measure[4]);

/* 编码器 → 轮速（mm/s） */
uint8_t GetWheelSpeed_FromSPI(SPI_SlaveCtx *ctx, float speed_out[4]);

/* 底盘模式控制（你之后新增的三个函数） */
void Chassis_Control_Move(void);
void Chassis_Control_Fire(void);
void Chassis_ControlLoop(void);

#endif

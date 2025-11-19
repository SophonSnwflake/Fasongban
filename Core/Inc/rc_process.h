#ifndef __RC_PROCESS_H
#define __RC_PROCESS_H

#include "stdint.h"

/**
 * @brief 遥控器解析后的控制量
 */
typedef struct
{
    float vx;           // 底盘前后速度 (mm/s)
    float vy;           // 底盘左右速度 (mm/s)
    float omega;        // 底盘旋转角速度 (deg/s)

    float pitch_speed;  // 云台 pitch 角速度 (deg/s)

    uint8_t mode;       // 0: 行进模式  1: 打靶模式
} RC_Ctrl_t;

// 声明一个全局遥控控制量
extern RC_Ctrl_t rc;

/**
 * @brief 从 d0~d4 更新 rc 结构体
 *        内部会做：归一化 + 映射到实际速度 + 模式判断
 */
void RC_Update(void);

#endif

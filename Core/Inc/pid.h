#ifndef __PID_H
#define __PID_H

#include "stdint.h"


typedef struct
{
    float Kp;
    float Ki;
    float Kd;

    float max_out;  // 输出限幅
    float max_iout; // 积分限幅

    float set;           // 目标
    float fdb;           // 反馈
    float err;           // 当前误差
    float last_err;      // 上一次误差
    float last_last_err; // 上上次

    float Pout;
    float Iout;
    float Dout;
    float out;
} PID_t;

// 初始化 PID
void PID_Init(PID_t *pid, float kp, float ki, float kd,float max_out, float max_iout);

// 清零 PID 数据
void PID_Clear(PID_t *pid);

// 核心计算（位置式 PID）
float PID_Calc(PID_t *pid, float set, float fdb);

#endif

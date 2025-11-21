#include "pid.h"

/**
 * @brief 限幅函数（内部使用）
 * @param x    当前值
 * @param max  最大允许值（正负）
 * @retval 限幅后的结果
 *
 * 作用：
 *   例如 max = 100，那么：
 *     x > 100  → 返回 100
 *     x < -100 → 返回 -100
 *     否则返回 x 本身
 */


// 限幅函数
static float Limit(float x, float max)
{
    if (x > max)
        return max;
    if (x < -max)
        return -max;
    return x;
}

/**
 * @brief PID 初始化（设置 PID 参数与限幅）
 * @param pid       PID 结构体指针
 * @param kp,ki,kd  PID 系数
 * @param max_out   输出限幅（避免电机过饱和）
 * @param max_iout  积分项限幅（避免积分爆炸）
 *
 * 初始化时会自动把误差、积分等清零。
 */

void PID_Init(PID_t *pid, float kp, float ki, float kd,
              float max_out, float max_iout)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;

    pid->max_out = max_out; // PID 输出最大值（例如 1000 对应 PWM）
    pid->max_iout = max_iout; // I 项最大值（例如 300）

    PID_Clear(pid);
}

void PID_Clear(PID_t *pid)
{
    pid->set = 0.0f;
    pid->fdb = 0.0f;
    pid->err = 0.0f;
    pid->last_err = 0.0f;
    pid->last_last_err = 0.0f;

    pid->Pout = 0.0f;
    pid->Iout = 0.0f;
    pid->Dout = 0.0f;
    pid->out = 0.0f;
}

/**
 * @brief 位置式 PID（主计算函数）
 * @param pid PID 控制器
 * @param set 设定值（目标值）
 * @param fdb 反馈值（实际测量值）
 * @retval PID 输出（已经过限幅）
 *
 * 内部流程：
 *   1) 计算误差 err = set - fdb
 *   2) 计算 P = Kp * err
 *   3) 计算 I = I + Ki * err（并对 I 做限幅）
 *   4) 计算 D = Kd * (err - last_err)
 *   5) 输出 = P + I + D（再做一次限幅）
 *   6) 保存 last_err
 *
 * 这是一套极稳定的标准 PID，用于电机速度环非常合适。
 */
float PID_Calc(PID_t *pid, float set, float fdb)
{
    pid->set = set;
    pid->fdb = fdb;

    // 1. 误差计算
    pid->err = pid->set - pid->fdb;

    // 2. P 项（比例）—— 解决响应速度
    pid->Pout = pid->Kp * pid->err;

    // 3. I 项（积分）—— 解决稳态误差
    pid->Iout += pid->Ki * pid->err;
    pid->Iout = Limit(pid->Iout, pid->max_iout); // 防止积分过大

    // 4. D 项（微分）—— 解决动态超调
    pid->Dout = pid->Kd * (pid->err - pid->last_err);

    // 5. 最终 PID 输出
    pid->out = pid->Pout + pid->Iout + pid->Dout;

    // 6. 输出限幅
    pid->out = Limit(pid->out, pid->max_out);

    // 7. 误差保存，便于下次计算 D
    pid->last_err = pid->err;

   

    return pid->out;
}
#include "gimbal.h"
#include "rc_process.h"
#include "pwm6.h"
#include "OLED.h"

// ---------------- SG90 标准参数 ----------------
#define SG90_DUTY_MIN 5.0f  // 0°
#define SG90_DUTY_MAX 10.0f // 180°

#define SG90_RANGE_DUTY (SG90_DUTY_MAX - SG90_DUTY_MIN)
#define SG90_ANGLE_MIN 0.0f
#define SG90_ANGLE_MAX 180.0f

// ---------------- 云台限制角度 ----------------
// 可根据实际机械结构修改
#define YAW_MIN_ANGLE 0.0f
#define YAW_MAX_ANGLE 180.0f
#define PIT_MIN_ANGLE 0.0f  // 防止往下撞结构
#define PIT_MAX_ANGLE 100.0f // 防止往上撞

// ---------------- 当前云台角度 ----------------
static float yaw_angle = 90.0f; // 中位
static float pit_angle = 90.0f; // 中位,可根据情况改为水平位置角度，需要同步修改初始化函数中的初始值

//--------------------------------------------------
// 工具函数：角度转 duty
//--------------------------------------------------
static float angle_to_duty(float ang)
{
    return 7.5 + (ang / 180.0f) * 5.0f;
}

void Gimbal_Init(void)
{
    yaw_angle = 90.0f;
    pit_angle = 90.0f;

    PWM6_SetDutyPercent(5, angle_to_duty(yaw_angle)); // 云台 YAW
    PWM6_SetDutyPercent(6, angle_to_duty(pit_angle)); // 云台 PITCH
}

//--------------------------------------------------
// 云台控制主流程（每 5ms 调用）
//--------------------------------------------------
void Gimbal_ControlLoop(void)
{
    extern RC_Ctrl_t rc;

    // 1) YAW：由遥控器 d0 控制 rc.omega（映射在 rc_process）
    yaw_angle += rc.omega * 0.05f; // 云台移动速度（每周期 5ms）
    if (yaw_angle < YAW_MIN_ANGLE)
        yaw_angle = YAW_MIN_ANGLE;
    if (yaw_angle > YAW_MAX_ANGLE)
        yaw_angle = YAW_MAX_ANGLE;

    // 2) PITCH：使用 rc.pitch_speed
    pit_angle = rc.pitch_speed ;
    OLED_ShowSignedNum(1, 41, (int)pit_angle, 4, OLED_6X8);
    if (pit_angle < PIT_MIN_ANGLE)
        pit_angle = PIT_MIN_ANGLE;
    if (pit_angle > PIT_MAX_ANGLE)
        pit_angle = PIT_MAX_ANGLE;

    // 3) 输出到舵机
    PWM6_SetDutyPercent(5, angle_to_duty(yaw_angle));
    PWM6_SetDutyPercent(6, angle_to_duty(pit_angle));
}
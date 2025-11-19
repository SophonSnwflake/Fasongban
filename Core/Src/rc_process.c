#include "rc_process.h"
#include "robot_config.h"
#include "spi_slave.h"

// 在这里定义全局的 rc（声明在 rc_process.h 里）
RC_Ctrl_t rc;

// 简单的归一化工具：把 [min, max] 映射到 [-1, +1]
static float rc_norm(uint8_t val, uint8_t min, uint8_t max)
{
    float mid = (min + max) * 0.5f;
    float half = (max - min) * 0.5f;
    return (val - mid) / half; // 得到 -1 ~ +1
}

void RC_Update(void)
{
    // 这些变量是底层同学在别的地方定义并更新的
    extern uint8_t d0, d1, d2, d3, d4, d5;

    // 1) 底盘速度映射
    rc.vx = rc_norm(d1, 85, 169) * CHASSIS_VX_MAX;       // 前后
    rc.vy = rc_norm(d3, 85, 169) * CHASSIS_VY_MAX;       // 左右
    rc.omega = rc_norm(d0, 77, 178) * CHASSIS_OMEGA_MAX; // 底盘旋转速度 (deg/s)

    // 2) 云台 pitch 速度（以后 gimbal 用）
    rc.pitch_speed = rc_norm(d2, 85, 169) * GIMBAL_PITCH_SPEED_MAX;

    // 3) 模式切换：d4 是二挡开关
    //    d4 小 ≈ 一档，d4 大 ≈ 二档（具体数值 85 / 169）
    if (d4 > 120)
        rc.mode = 1; // 打靶模式
    else
        rc.mode = 0; // 行进模式
}

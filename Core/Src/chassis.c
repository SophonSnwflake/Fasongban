#include "chassis.h"
#include "robot_config.h"
#include "pid.h"
#include "changecaculate.h"
#include "spi_slave.h"
#include "motor.h"
#include "rc_process.h"
#include "OLED.h"

extern SPI_SlaveCtx spi2_ctx;

// 全局缓存数组
float wheel_target[4];
float wheel_speed[4];

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
void Calc_Mecanum(float vx, float vy, float omega, float wheel_out[4])
{
    // 将角速度从 deg/s 转成 rad/s（如果你输入的本来就是 rad/s，可以删掉这一行）
    float omega_rad = omega * 0.01745329252f; // = PI / 180

    // R = Lx + Ly（在 robot_config.h 中定义）
    float R = CHASSIS_ROTATE_R;

    // 麦轮运动学（标准四麦轮公式）
    // wheel_out[0] = -(vx - vy - omega_rad * R)/8; // FL 左前
    // wheel_out[1] = (vx + vy + omega_rad * R) / 8; // BL 左后
    // wheel_out[2] = (vx - vy - omega_rad * R) / 8; // BR 右后
    // wheel_out[3] = (vx + vy + omega_rad * R) / 8; // FR 右前


    wheel_out[0] = -(vx + vy + omega_rad * R)/8; // FL 左前
    wheel_out[1] = (vx - vy + omega_rad * R) / 8; // BL 左后
    wheel_out[2] = (vx + vy - omega_rad * R) / 8; // BR 右后
    wheel_out[3] = (vx - vy - omega_rad * R) / 8; // FR 右前
}

// -------------------- 四个轮子的 PID 控制器 --------------------
static PID_t pid_wheel[4];

// -------------------- 初始化四个轮子的 PID --------------------
void Chassis_PID_Init(void)
{
    // Kp Ki Kd 根据后面调试修改。PWM 范围为 -100~100。
    float kp = 1.0f;
    float ki = 0.0f;
    float kd = 0.0f;

    for (int i = 0; i < 4; i++)
    {
        PID_Init(&pid_wheel[i],
                 kp, ki, kd,
                 MOTOR_PWM_MAX,    // 输出限幅：±100
                 MOTOR_PWM_MAX / 2 // 积分限幅：±50
        );
    }
}

/**
 * @brief  四轮速度 PID 控制（生成 PWM）
 * @param  target[4]  四个轮子的目标速度（mm/s）
 * @param  measure[4] 四个轮子的实际速度（mm/s）
 *
 * 输出过程：
 *   1) 对每个轮执行 PID
 *   2) 限幅 ±100
 *   3) 调用底层 Motor_SetPWM(id, duty)
 */
void Update_4Wheel_PID(float target[4], float measure[4])
{
    int pwm_out;

    for (int i = 0; i < 4; i++)
    {
        float pwm = PID_Calc(&pid_wheel[i], target[i], measure[i]);

        // 限幅（双保险）
        if (pwm > MOTOR_PWM_MAX)

            pwm = MOTOR_PWM_MAX;
        if (pwm < -MOTOR_PWM_MAX)
            pwm = -MOTOR_PWM_MAX;

        pwm_out = (int)pwm;

        

        // 映射到实际的 motor id（1~4）
        Motor_SetPWM(i + 1, pwm_out);
        if (i == 0)
        {
            /* code */
            OLED_ShowSignedNum(1, 9, pwm_out, 4, OLED_6X8);
        }
        else if (i == 1)
        {
            OLED_ShowSignedNum(1, 17, pwm_out, 4, OLED_6X8);
        }
        else if (i == 2)
        {
            OLED_ShowSignedNum(1, 25, pwm_out, 4, OLED_6X8);
        }
        else if (i == 3)
        {
            OLED_ShowSignedNum(1, 33, pwm_out, 4, OLED_6X8);
        }
        
        OLED_ShowString(1, 1, "PID:", OLED_6X8);
        
        
        
        
        OLED_Update();
    }
        
}

/**
 * @brief 读取 SPI 编码器并转换为轮速（严格使用原逻辑）
 * @param ctx    SPI2 的上下文
 * @param speed_out[4]  输出：四个轮子的速度（mm/s）
 *        0=FL, 1=BL, 2=BR, 3=FR
 * @retval 1：成功读取  0：无新数据
 */
uint8_t GetWheelSpeed_FromSPI(SPI_SlaveCtx *ctx, float speed_out[4])
{
    uint8_t k0, k1, k2, k3;

    // 尝试获取 4 个 u8 原始速度数据（0~255）
    if (!SPI_Slave_TryGet4U8(ctx, &k0, &k1, &k2, &k3))
        return 0;

    // 严格使用原公式转换
    speed_out[0] = caculateWheelSpeed(WHEEL_RADIUS_MM, k0, ENCODER_MUTIPLE); // FL
    speed_out[1] = caculateWheelSpeed(WHEEL_RADIUS_MM, k1, ENCODER_MUTIPLE); // BL
    speed_out[2] = caculateWheelSpeed(WHEEL_RADIUS_MM, k2, ENCODER_MUTIPLE); // BR
    speed_out[3] = caculateWheelSpeed(WHEEL_RADIUS_MM, k3, ENCODER_MUTIPLE); // FR

    // OLED_ShowString(35, 1, "Spe:", OLED_6X8);
    // OLED_ShowSignedNum(35, 9, wheel_speed[0], 4, OLED_6X8);
    // OLED_ShowSignedNum(35, 17, wheel_speed[1], 4, OLED_6X8);
    // OLED_ShowSignedNum(35, 25, wheel_speed[2], 4, OLED_6X8);
    // OLED_ShowSignedNum(35, 33, wheel_speed[3], 4, OLED_6X8);
    // OLED_Update();

    return 1;
}

/**
 * @brief 行进模式：底盘三向运动 + 旋转，云台 YAW 锁死
 */
void Chassis_Control_Move(void)
{
    float vx = rc.vx;
    float vy = rc.vy;
    float omega = rc.omega; // 注意：单位是 deg/s，Calc_Mecanum 里会转 rad

    // 1) 运动学求目标轮速
    Calc_Mecanum(vx, vy, omega, wheel_target);

    

    // 2) 读取当前轮速（mm/s）
    GetWheelSpeed_FromSPI(&spi2_ctx, wheel_speed);

    // 3) 四轮 PID 控制
    Update_4Wheel_PID(wheel_target, wheel_speed);

    OLED_ShowSignedNum(70, 9, (int)wheel_target[0], 4, OLED_6X8);
    OLED_ShowSignedNum(70, 17, (int)wheel_target[1], 4, OLED_6X8);
    OLED_ShowSignedNum(70, 25, (int)wheel_target[2], 4, OLED_6X8);
    OLED_ShowSignedNum(70, 33, (int)wheel_target[3], 4, OLED_6X8);

    // OLED_ShowSignedNum(70, 25, vx, 3, OLED_6X8);
    // OLED_ShowSignedNum(70, 33, vy, 3, OLED_6X8);
    OLED_Update();

}

/**
 * @brief 打靶模式：底盘不自转（omega=0），云台 YAW 由遥控控制
 */
void Chassis_Control_Fire(void)
{
    float vx = rc.vx;
    float vy = rc.vy;
    float omega = 0.0f; // 锁死底盘旋转

    Calc_Mecanum(vx, vy, omega, wheel_target);
    GetWheelSpeed_FromSPI(&spi2_ctx, wheel_speed);
    // wheel_speed[3] = wheel_speed[3]/80000009000000000;
    Update_4Wheel_PID(wheel_target, wheel_speed);
}

/**
 * @brief 底盘总控制循环（外部每次调用执行一帧控制）
 */
void Chassis_ControlLoop(void)
{
    RC_Update(); // 先根据 d0~d4 更新 rc 内的 vx,vy,omega,mode

    if (rc.mode == 0)
    {
        // 行进模式
        Chassis_Control_Move();
    }
    else
    {
        // 打靶模式
        Chassis_Control_Fire();
    }
}
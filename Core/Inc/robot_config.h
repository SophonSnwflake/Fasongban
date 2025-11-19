#ifndef __ROBOT_CONFIG_H
#define __ROBOT_CONFIG_H

#include "stdint.h"

/************************************************************
 *  小车底盘配置文件
 *  所有底盘运动学相关的参数都在这里配置
 *  后期调试或更换机械结构时，只需要改这里即可
 ************************************************************/

/******************* 1. 轮子相关参数 ************************/

/**
 * @brief 轮子半径（单位：mm）
 * 常见麦克纳姆轮尺寸：
 *   48mm / 65mm / 75mm
 * 对应轮子半径一般是直径的一半。
 *
 * 我们直径 55mm
 * 因此轮子半径 ≈ 22.5mm
 */
#define WHEEL_RADIUS_MM 25.5f // 55mm 麦轮

/**
 * @brief 轮周长（用于脉冲 → mm 的换算，可选）
 */
#define WHEEL_CIRCUM_MM (2.0f * 3.1415926f * WHEEL_RADIUS_MM)

/******************* 2. 底盘尺寸（车中心到四轮的距离） **********************/

/**
 * @brief 小车中心到左右轮中心的水平距离 Lx（单位 mm）
 *
 * 一般 4 麦轮小车：
 *   宽度（左右轮中心距）约 170~200mm
 *   则 Lx = 宽度 / 2
 *
 * 推荐值：Lx = 90mm
 */
#define CHASSIS_LX 90.0f

/**
 * @brief 小车中心到前后轮中心的垂直距离 Ly（单位 mm）
 *
 * 前后距离一般略小于左右距离，可以设为 80~100mm。
 *
 * 推荐值：Ly = 90mm
 */
#define CHASSIS_LY 90.0f

/**
 * @brief 麦轮运动学中用于旋转的有效半径
 *
 * R = Lx + Ly
 * 这个是麦轮底盘旋转运动分量计算的重要参数
 */
#define CHASSIS_ROTATE_R (CHASSIS_LX + CHASSIS_LY)

/******************* 3. 底盘控制参数（死区等） ************************/

/**
 * @brief 底盘跟随云台时的死区角（单位：度）
 *
 * 小于该角度 → 底盘不跟随（视角左右微调不带动车体）
 * 大于该角度 → 底盘开始旋转去跟视角
 *
 * 标准值：15°
 */
#define GIMBAL_FOLLOW_DEADZONE_DEG 15.0f

/******************* 4. 云台参数 ********************************/

/**
 * @brief 云台角速度（单位：度/秒）
 *
 * 遥控摇杆映射到的最大旋转速度
 * 150°/s 是 RM 常见值，速度快但不抖
 */
#define GIMBAL_MAX_YAW_SPEED_DEG   150.0f
#define GIMBAL_MAX_PITCH_SPEED_DEG 150.0f

/**
 * @brief 云台角度软件限位（不允许超出这个范围）
 *
 * 避免 SG90 舵机打齿、卡死、抖动
 */
#define GIMBAL_YAW_MAX_DEG   90.0f
#define GIMBAL_YAW_MIN_DEG   -90.0f

#define GIMBAL_PITCH_MAX_DEG 45.0f
#define GIMBAL_PITCH_MIN_DEG -35.0f

/******************* 5. 电机 PWM 输出范围 **********************/

/**
 * @brief 对 JGA25-370 电机驱动时常用 PWM 范围
 *
 * 一般 PWM 范围为 -1000 ~ +1000
 * 正负代表电机正反转
 */
#define MOTOR_PWM_MAX 100.0f

/******************* 6. 编码器速度比例因子 **********************/

#define ENCODER_MUTIPLE 1.39008526f // 编码器速度比例因子（0.22123894*2*pi）

/******************* 7. 底盘最大线速度（mm/s） **********************/
#define CHASSIS_VX_MAX 800.0f
#define CHASSIS_VY_MAX 800.0f

/******************* 8. 底盘最大旋转速度（deg/s，行进模式底盘旋转用的是速控） **********************/
#define CHASSIS_OMEGA_MAX 180.0f

/******************* 9. 云台 pitch 最大角速度（deg/s，先给个温和值，后期你可以调） **********************/
#define GIMBAL_PITCH_SPEED_MAX 60.0f

#endif

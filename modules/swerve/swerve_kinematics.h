/**
 * @file swerve_kinematics.h
 * @brief 四舵轮运动学接口，根据车体速度和当前舵角求取轮组目标
 */
#ifndef SWERVE_KINEMATICS_H
#define SWERVE_KINEMATICS_H

#include <stdint.h>

/**
 * @brief 四舵轮正方形布局与传动参数，由应用持有并传入
 * @note 三个参数均须为有限正数；减速比为输出轴转速/转子转速。
 */
typedef struct
{
    float center_to_wheel_m; ///< 底盘中心到轮心距离，m。
    float wheel_perimeter_m; ///< 驱动轮周长，m。
    float drive_reduction;   ///< 输出轴/转子转速比，无量纲。
} SwerveGeometry_s;

/**
 * @brief 单个轮组的舵角和驱动转速目标
 * @note angle_deg与输入的多圈实际舵角连续；speed_rpm对应电机转子，不是输出轴。
 */
typedef struct
{
    float angle_deg;  ///< 与当前舵角连续的多圈目标，度。
    float speed_rpm;  ///< 驱动电机转子目标转速，rpm，带方向。
    uint8_t reversed; ///< 为减少转向行程而反转驱动时为 1。
} SwerveTarget;

/**
 * @brief 将弧度角差归一化到[-PI, PI]
 * @param angle 有限角度值，单位rad
 * @return float 归一化后的角差，单位rad
 * @attention 不检查NaN或无穷值，调用方应先验证输入。
 */
float SwerveNormalizeRad(float angle);

/**
 * @brief 计算四轮平移与旋转的合成速度，并选择舵向最短转动和驱动正反转
 * @param geometry 应用提供的有效几何与传动参数，只读且不可为NULL
 * @param forward_velocity_mps 车体X向前速度，单位m/s
 * @param left_velocity_mps 车体Y向左速度，单位m/s，注意与应用层Y向右约定相反
 * @param yaw_rate_dps 俯视逆时针为正的车体旋转速度，单位度/秒
 * @param steering_actual_deg 四轮校零后的实际多圈舵角，单位度，轮序为左前/右前/右后/左后
 * @param wheel_targets 四轮目标数组，作为输出；全零速度时只清轮速，保留原舵角和反转标志
 * @attention 两个数组均至少有4项且指针有效；输入必须为有限数，target应在首次调用前初始化。
 * @note 函数不检查电机在线状态、不执行轮速限幅、不发送报文，这些由底盘应用负责。
 *       采用正方形轮心布局；轮心距离、轮周长和减速比均来自geometry，不依赖应用宏。
 */
void SwerveSolve(const SwerveGeometry_s *geometry, float forward_velocity_mps, float left_velocity_mps,
                 float yaw_rate_dps, const float steering_actual_deg[4], SwerveTarget wheel_targets[4]);

#endif

/**
 * @file swerve_kinematics.c
 * @brief 四舵轮逆运动学及最短转向算法，不依赖遥控、电机实例或RTOS
 *
 * 先由每个轮心的位置把车体旋转速度换为轮心切向速度，再与平移矢量叠加。
 * 每轮以atan2求目标方向，以矢量模长求轮速；超过90度的转向可由驱动反转等效替代。
 * 输出角度接在当前多圈舵角附近，避免编码器跨圈时目标突然跳转。
 */
#include "swerve_kinematics.h"
#include <math.h>

#define PI 3.14159265358979323846f

// 将弧度差限制到 [-PI, PI]，用于寻找当前多圈舵角附近的最短转向。
float SwerveNormalizeRad(float angle)
{
    while (angle > PI)
    {
        angle -= 2.0f * PI;
    }
    while (angle < -PI)
    {
        angle += 2.0f * PI;
    }
    return angle;
}

void SwerveSolve(const SwerveGeometry_s *geometry, float forward_velocity_mps, float left_velocity_mps,
                 float yaw_rate_dps, const float steering_actual_deg[4], SwerveTarget wheel_targets[4])
{
    unsigned i;
    /* 轮心位于正方形四角，theta为45度，中心到轮心的距离为配置半径。
     * 单轮矢量：wheel_vx_i = forward_velocity_mps - wz*py_i，wheel_vy_i = left_velocity_mps + wz*px_i。
     * 下方按左前、右前、右后、左后展开轮心坐标，wz先从度/秒换为rad/s。 */
    const float wheel_position_angle_rad = atan(1.0 / 1.0);
    /* 这里保留原成品解算的 3.14 换算系数，避免改动已经调试过的运动参数。 */
    const float rotation_tangent_speed_mps = yaw_rate_dps * 3.14f / 180.0f * geometry->center_to_wheel_m;
    // 轮缘线速度 m/s -> 输出轴转速 -> 电机转子 rpm。
    const float rotor_rpm_per_mps = 60.0f / (geometry->wheel_perimeter_m * geometry->drive_reduction);
    const float wheel_forward_velocity_mps[4] = {
        forward_velocity_mps - rotation_tangent_speed_mps * sinf(wheel_position_angle_rad),
        forward_velocity_mps + rotation_tangent_speed_mps * sinf(wheel_position_angle_rad),
        forward_velocity_mps + rotation_tangent_speed_mps * sinf(wheel_position_angle_rad),
        forward_velocity_mps - rotation_tangent_speed_mps * sinf(wheel_position_angle_rad)};
    const float wheel_left_velocity_mps[4] = {
        left_velocity_mps + rotation_tangent_speed_mps * cosf(wheel_position_angle_rad),
        left_velocity_mps + rotation_tangent_speed_mps * cosf(wheel_position_angle_rad),
        left_velocity_mps - rotation_tangent_speed_mps * cosf(wheel_position_angle_rad),
        left_velocity_mps - rotation_tangent_speed_mps * cosf(wheel_position_angle_rad)};
    // 全零速度只清轮速，保留上一轮舵角目标，避免松杆时舵轮反复归中。
    if (forward_velocity_mps == 0 && left_velocity_mps == 0 && yaw_rate_dps == 0)
    {
        for (i = 0; i < 4; i++)
        {
            wheel_targets[i].speed_rpm = 0;
        }
        return;
    }
    for (i = 0; i < 4; i++)
    {
        float steering_actual_rad = steering_actual_deg[i] * PI / 180.0f;
        // 当前舵角可能已经跨越多圈，只归一化“目标方向-当前舵角”的差值。
        // 最终再把最短角差加回当前角，保留与实际多圈角一致的圈数。
        float steering_delta_rad =
            SwerveNormalizeRad(atan2(wheel_left_velocity_mps[i], wheel_forward_velocity_mps[i]) - steering_actual_rad);
        wheel_targets[i].reversed = 0;
        // 目标方向与当前方向相差大于90度时，舵角少转180度，同时驱动转速反号。
        // 两者组合产生相同的轮缘速度矢量，并把单次舵向行程限制在正负90度内。
        if (steering_delta_rad > PI / 2.0f)
        {
            steering_delta_rad -= PI;
            wheel_targets[i].reversed = 1;
        }
        else if (steering_delta_rad < -PI / 2.0f)
        {
            steering_delta_rad += PI;
            wheel_targets[i].reversed = 1;
        }
        wheel_targets[i].angle_deg = (steering_actual_rad + steering_delta_rad) * 180.0f / PI;
        wheel_targets[i].speed_rpm =
            sqrt(pow(wheel_left_velocity_mps[i], 2) + pow(wheel_forward_velocity_mps[i], 2)) * rotor_rpm_per_mps;
        if (wheel_targets[i].reversed)
        {
            wheel_targets[i].speed_rpm = -wheel_targets[i].speed_rpm;
        }
    }
}

/**
 * @file swerve_kinematics.c
 * @brief 四舵轮正运动学及最短转向算法，不依赖遥控、电机实例或RTOS
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
    while (angle > PI) angle -= 2.0f * PI;
    while (angle < -PI) angle += 2.0f * PI;
    return angle;
}

void SwerveSolve(const SwerveGeometry_s *geometry, float vx, float vy, float wz_dps, const float actual_deg[4],
                 SwerveTarget target[4])
{
    unsigned i;
    /* 轮心位于正方形四角，theta为45度，中心到轮心的距离为配置半径。
     * 单轮矢量：wheel_vx_i = vx - wz*py_i，wheel_vy_i = vy + wz*px_i。
     * 下方按左前、右前、右后、左后展开轮心坐标，wz先从度/秒换为rad/s。 */
    const float theta = atan(1.0 / 1.0);
    /* 这里保留原成品解算的 3.14 换算系数，避免改动已经调试过的运动参数。 */
    const float vw = wz_dps * 3.14f / 180.0f * geometry->center_to_wheel_m;
    // 轮缘线速度 m/s -> 输出轴转速 -> 电机转子 rpm。
    const float ratio = 60.0f / (geometry->wheel_perimeter_m * geometry->drive_reduction);
    const float x[4] = {vx - vw * sinf(theta), vx + vw * sinf(theta),
                        vx + vw * sinf(theta), vx - vw * sinf(theta)};
    const float y[4] = {vy + vw * cosf(theta), vy + vw * cosf(theta),
                        vy - vw * cosf(theta), vy - vw * cosf(theta)};
    // 全零速度只清轮速，保留上一轮舵角目标，避免松杆时舵轮反复归中。
    if (vx == 0 && vy == 0 && wz_dps == 0)
    {
        for (i = 0; i < 4; i++) target[i].speed_rpm = 0;
        return;
    }
    for (i = 0; i < 4; i++)
    {
        float current = actual_deg[i] * PI / 180.0f;
        // 当前舵角可能已经跨越多圈，只归一化“目标方向-当前舵角”的差值。
        // 最终再把最短角差加回当前角，保留与实际多圈角一致的圈数。
        float diff = SwerveNormalizeRad(atan2(y[i], x[i]) - current);
        target[i].reversed = 0;
        // 目标方向与当前方向相差大于90度时，舵角少转180度，同时驱动转速反号。
        // 两者组合产生相同的轮缘速度矢量，并把单次舵向行程限制在正负90度内。
        if (diff > PI / 2.0f) { diff -= PI; target[i].reversed = 1; }
        else if (diff < -PI / 2.0f) { diff += PI; target[i].reversed = 1; }
        target[i].angle_deg = (current + diff) * 180.0f / PI;
        target[i].speed_rpm = sqrt(pow(y[i], 2) + pow(x[i], 2)) * ratio;
        if (target[i].reversed) target[i].speed_rpm = -target[i].speed_rpm;
    }
}

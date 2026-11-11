/**
 * @file chassis_motion.c
 * @brief 仅处理底盘旋转参考和坐标变换，保留已验证的模式数值及正负方向。
 */
#include "chassis_motion.h"
#include "general_def.h"
#include "swerve_kinematics.h"
#include <math.h>

/** @brief 对目标作关于零的对称限幅，limit为非负幅值。 */
static float ClampMotionValue(float value, float limit)
{
    return fmaxf(-limit, fminf(limit, value));
}

/** @brief 重置旋转参考；非旋转模式每轮更新参考，但不改变当轮平移输出。 */
static void ResetSpinReference(ChassisMotionState_s *state, const ChassisMotionConfig_s *config, float yaw_deg)
{
    PID_Init_Config_s controller_config = {
        .Kp = config->spin_correction_gain,
        .MaxOut = config->spin_correction_radps,
    };
    PIDInit(&state->spin_rate_controller, &controller_config);
    state->spin_rate_reference_radps = 0;
    state->spin_reference_yaw_deg = yaw_deg;
}

void ChassisMotionReset(ChassisMotionState_s *state, const ChassisMotionConfig_s *config, float yaw_deg)
{
    ResetSpinReference(state, config, yaw_deg);
    state->relative_yaw_deg = 0;
    state->forward_velocity_mmps = 0;
    state->right_velocity_mmps = 0;
    state->yaw_rate_radps = 0;
}

/**
 * @brief 按旋转模式生成角速度，保留原先爬升再叠加反馈修正的顺序。
 * @return rad/s，未施加最终整车旋转上限。
 */
static float CalculateYawRate(ChassisMotionState_s *state, const ChassisMotionConfig_s *config,
                              const Chassis_Ctrl_Cmd_s *command, chassis_mode_e previous_mode, float yaw_deg,
                              float gyro_z_radps, uint32_t elapsed_ms)
{
    if (command->chassis_mode == CHASSIS_ROTATE)
    {
        if (previous_mode != CHASSIS_ROTATE)
        {
            ResetSpinReference(state, config, yaw_deg);
        }
        float maximum_rate_step = config->spin_acceleration_radps2 * elapsed_ms * 0.001f;
        state->spin_rate_reference_radps +=
            ClampMotionValue(command->chassis_spin_speed - state->spin_rate_reference_radps, maximum_rate_step);
        return state->spin_rate_reference_radps +
               PIDCalculate(&state->spin_rate_controller, gyro_z_radps, state->spin_rate_reference_radps);
    }

    ResetSpinReference(state, config, yaw_deg);
    if (command->chassis_mode == CHASSIS_NO_FOLLOW)
    {
        return 0;
    }
    if (command->chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW)
    {
        return config->follow_gain_radps_per_deg * command->offset_angle;
    }
    return command->wz;
}

/** @brief 按命令中的参考系编码取得相对航向；数值定义见robot_def.h。 */
static float CalculateReferenceAngle(const ChassisMotionState_s *state, const Chassis_Ctrl_Cmd_s *command,
                                     float yaw_deg)
{
    float reference_angle_deg = 0;
    if (command->mode == 1)
    {
        reference_angle_deg = yaw_deg - command->offset_angle;
    }
    else if (command->mode == 2)
    {
        reference_angle_deg = command->offset_angle;
    }
    else if (command->mode == 3 && command->chassis_mode == CHASSIS_ROTATE)
    {
        reference_angle_deg = yaw_deg - state->spin_reference_yaw_deg;
    }
    // 只归一化角差，跨越±180deg时平移方向仍连续。
    return SwerveNormalizeRad(reference_angle_deg * DEGREE_2_RAD) * RAD_2_DEGREE;
}

void ChassisMotionUpdate(ChassisMotionState_s *state, const ChassisMotionConfig_s *config,
                         const Chassis_Ctrl_Cmd_s *command, chassis_mode_e previous_mode, float yaw_deg,
                         float gyro_z_radps, uint32_t elapsed_ms)
{
    float yaw_rate_radps = CalculateYawRate(state, config, command, previous_mode, yaw_deg, gyro_z_radps, elapsed_ms);
    state->relative_yaw_deg = CalculateReferenceAngle(state, command, yaw_deg);
    float reference_angle_rad = state->relative_yaw_deg * DEGREE_2_RAD;
    state->forward_velocity_mmps = command->vx * cosf(reference_angle_rad) - command->vy * sinf(reference_angle_rad);
    state->right_velocity_mmps = command->vx * sinf(reference_angle_rad) + command->vy * cosf(reference_angle_rad);
    state->yaw_rate_radps = ClampMotionValue(yaw_rate_radps, config->maximum_yaw_rate_radps);
}

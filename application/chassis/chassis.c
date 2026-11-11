/**
 * @file chassis.c
 * @brief 四舵轮底盘应用，负责模式处理、坐标变换、运动学解算和底盘状态反馈
 *
 * 数据流：CMD发布chassis_cmd -> 本应用订阅 -> SwerveSolve解算四轮目标
 *         -> DJIMotorSetRef更新电机目标 -> 原MotorControlTask闭环并发送CAN报文。
 * 本文件不直接发送电机控制帧，驱动轮由PowerControl管理，舵向轮由DJIMotor管理。
 *
 * 应用层速度约定：X向前、Y向右，平移单位mm/s，旋转单位rad/s，俯视逆时针为正。
 * 解算模块约定：X向前、Y向左，平移单位m/s，旋转输入为度/秒；在ApplyWheelTargets中转换。
 * 所有轮子数组均按左前、右前、右后、左后排列，索引对应0、1、2、3。
 */
#include "chassis.h"
#include "chassis_motion.h"
#include "robot_def.h"
#include "swerve_config.h"
#include "swerve_kinematics.h"
#include "power_control.h"
#include "message_center.h"
#include "referee_task.h"
#include "referee_UI.h"
#include "super_cap.h"
#include "master_process.h"
#include "general_def.h"
#include "ins_task.h"
#include <math.h>
#include <string.h>

/* 底盘拥有的模块实例和消息，初始化后由本应用保存；电机反馈由CAN接收回调更新。 */
static DJIMotorInstance *drive_motors[4];           ///< M3508驱动电机，轮序左前/右前/右后/左后。
static DJIMotorInstance *steering_motors[4];        ///< GM6020舵向电机，与驱动轮轮序一一对应。
static Subscriber_t *chassis_command_subscriber;    ///< 订阅CMD发布的chassis_cmd话题
static Publisher_t *chassis_feedback_publisher;     ///< 发布chassis_feed，供CMD读取底盘反馈
static Chassis_Ctrl_Cmd_s chassis_command;          ///< 本轮收到的底盘目标，消息为空时不能沿用旧目标
static Chassis_Upload_Data_s chassis_feedback_data; ///< 由实际轮速、舵角反算的车体速度
static referee_info_t *referee_data;                ///< 裁判系统反馈，读取底盘功率上限
static Referee_Interactive_info_t ui_data;          ///< 原裁判UI模块使用的交互数据
static attitude_t chassis_imu_snapshot;             ///< 本轮INS快照，必须配合ins_online使用
static uint32_t last_chassis_task_ms;               ///< 上轮底盘任务时间，用于转速爬升和任务超时检查

/* 运行状态只在底盘应用内部使用；调试快照不作为控制输入。 */
typedef struct
{
    chassis_mode_e mode;          ///< 当前实际执行模式，保护停机时为无力。
    uint8_t command_online;       ///< 本轮是否收到 CMD 消息，不代表遥控器在线。
    uint8_t motors_online;        ///< 八电机全部有有效且未过期的反馈时为 1。
    uint8_t ins_online;           ///< INS 快照新鲜且航向、角速度数值正常。
    uint8_t drive_online[4];      ///< 四个 M3508 反馈状态。
    uint8_t steering_online[4];   ///< 四个 GM6020 反馈状态。
    float yaw_deg;                ///< 底盘 INS 航向，度。
    float gyro_z_radps;           ///< Z 轴角速度，rad/s，俯视逆时针为正。
    float relative_yaw_deg;       ///< 本轮速度坐标转换使用的相对角，度。
    float spin_reference_yaw_deg; ///< 进入小陀螺时捕获的底盘朝向，度。
    float spin_rate_ref_radps;    ///< 经角加速度限制后的旋转参考，rad/s。
    float vx_body_mmps;           ///< 车体系 X 向前目标，mm/s。
    float vy_body_mmps;           ///< 车体系 Y 向右目标，mm/s。
    float wz_radps;               ///< 实际送入运动学的角速度目标，rad/s。
    float wheel_target_rpm[4];    ///< 驱动转子目标，经整车限速和舵向误差补偿。
    float wheel_actual_rpm[4];    ///< 驱动转子实际 rpm，尚未乘减速比。
    float steering_target_deg[4]; ///< 舵向多圈目标，度。
    float steering_actual_deg[4]; ///< 舵向实际多圈角，已处理机械零位和方向，度。
} ChassisState_s;
static ChassisState_s chassis_state;
static ChassisMotionState_s chassis_motion_state; ///< 小陀螺参考与坐标变换状态，独立于设备实例。
static const ChassisMotionConfig_s chassis_motion_config = {
    .spin_acceleration_radps2 = SWERVE_SPIN_ACCEL_DPS2 * DEGREE_2_RAD,
    .spin_correction_gain = SWERVE_SPIN_RATE_KP,
    .spin_correction_radps = SWERVE_SPIN_CORRECTION_DPS * DEGREE_2_RAD,
    .maximum_yaw_rate_radps = SWERVE_MAX_WZ_RADPS,
    .follow_gain_radps_per_deg = 0.15f,
};
SwerveDebug_s swerve_debug;
static const SwerveGeometry_s swerve_geometry = {.center_to_wheel_m = CHASSIS_CENTER_TO_WHEEL,
                                                 .wheel_perimeter_m = CHASSIS_MOTOT_PER,
                                                 .drive_reduction = SWERVE_DRIVE_REDUCTION};
static SwerveTarget wheel_targets[4];  ///< 运动学输出，零速时保留其中上一轮的舵向目标
static float steering_feedback_deg[4]; ///< 校零且修正方向的多圈舵角，同时作为角度环反馈
static const uint16_t steering_zero_encoders[4] = SWERVE_ZERO_ENCODERS; ///< 四个轮子机械朝前时的编码器刻度

/** @brief 将本轮私有运行状态复制到观测快照；控制路径不读取快照。 */
static void PublishChassisDebug(void)
{
    swerve_debug.mode = chassis_state.mode;
    swerve_debug.command_online = chassis_state.command_online;
    swerve_debug.motors_online = chassis_state.motors_online;
    swerve_debug.ins_online = chassis_state.ins_online;
    memcpy(swerve_debug.drive_online, chassis_state.drive_online, sizeof(swerve_debug.drive_online));
    memcpy(swerve_debug.steering_online, chassis_state.steering_online, sizeof(swerve_debug.steering_online));
    swerve_debug.yaw_deg = chassis_state.yaw_deg;
    swerve_debug.gyro_z_radps = chassis_state.gyro_z_radps;
    swerve_debug.relative_yaw_deg = chassis_state.relative_yaw_deg;
    swerve_debug.spin_reference_yaw_deg = chassis_state.spin_reference_yaw_deg;
    swerve_debug.spin_rate_ref_radps = chassis_state.spin_rate_ref_radps;
    swerve_debug.vx_body_mmps = chassis_state.vx_body_mmps;
    swerve_debug.vy_body_mmps = chassis_state.vy_body_mmps;
    swerve_debug.wz_radps = chassis_state.wz_radps;
    memcpy(swerve_debug.wheel_target_rpm, chassis_state.wheel_target_rpm, sizeof(swerve_debug.wheel_target_rpm));
    memcpy(swerve_debug.wheel_actual_rpm, chassis_state.wheel_actual_rpm, sizeof(swerve_debug.wheel_actual_rpm));
    memcpy(swerve_debug.steering_target_deg, chassis_state.steering_target_deg,
           sizeof(swerve_debug.steering_target_deg));
    memcpy(swerve_debug.steering_actual_deg, chassis_state.steering_actual_deg,
           sizeof(swerve_debug.steering_actual_deg));
}

referee_info_t *get_referr_data_pr(void)
{
    return referee_data;
}

/**
 * @brief 检查单个电机是否已收到有效反馈，且反馈尚未过期
 * @param motor 已完成初始化的电机实例，不可为NULL
 * @return uint8_t 1:反馈有效；0:尚未收帧或超过SWERVE_MOTOR_TIMEOUT_MS
 * @note 在读取电机反馈的临界区中调用，不将上电时全零的测量结构视为在线。
 */
static uint8_t IsChassisMotorOnline(DJIMotorInstance *motor)
{
    return motor->feedback_seen && (uint32_t)(HAL_GetTick() - motor->last_rx_ms) <= SWERVE_MOTOR_TIMEOUT_MS;
}

/**
 * @brief 重置小陀螺转速修正器，将当前航向记为下一次旋转的平移参考
 * @note 本应用在初始化与无力时调用；模式入口/非旋转时的参考维护由ChassisMotion处理。
 *       复位后角速度参考从零开始爬升，避免直接给定完整转速。
 */
static void ResetChassisMotion(void)
{
    ChassisMotionReset(&chassis_motion_state, &chassis_motion_config, chassis_state.yaw_deg);
    chassis_state.spin_rate_ref_radps = chassis_motion_state.spin_rate_reference_radps;
    chassis_state.spin_reference_yaw_deg = chassis_motion_state.spin_reference_yaw_deg;
}

/**
 * @brief 将八电机置为停止状态，并复位目标缓存与小陀螺状态
 * @note DJIMotorStop使电机任务发送零控制量；仅设置零轮速仍会闭环制动，不是无力模式。
 *       舵角目标缓存跟随当前实际舵角，恢复后不追赶停机前的旧目标。
 */
static void StopChassisMotors(void)
{
    for (unsigned i = 0; i < 4; ++i)
    {
        DJIMotorStop(drive_motors[i]);
        DJIMotorStop(steering_motors[i]);
        DJIMotorSetRef(drive_motors[i], 0);
        DJIMotorSetRef(steering_motors[i], steering_feedback_deg[i]);
        wheel_targets[i].speed_rpm = 0;
        wheel_targets[i].angle_deg = steering_feedback_deg[i];
        chassis_state.wheel_target_rpm[i] = 0;
        chassis_state.steering_target_deg[i] = steering_feedback_deg[i];
    }
    chassis_state.mode = CHASSIS_ZERO_FORCE;
    chassis_state.vx_body_mmps = chassis_state.vy_body_mmps = chassis_state.wz_radps = 0;
    ResetChassisMotion();
}

void ChassisInit(void)
{
    // 驱动轮走原 PowerControl 的 rpm 速度环，CAN1 ID1~4。
    Motor_Init_Config_s motor_config = {
        .can_init_config.can_handle = CHASSIS_DRIVE_CAN_HANDLE,
        .controller_param_init_config.speed_PID =
            {
                .Kp = SWERVE_DRIVE_KP,
                .MaxOut = SWERVE_DRIVE_MAX_OUT,
            },
        .controller_setting_init_config =
            {
                .outer_loop_type = SPEED_LOOP,
                .close_loop_type = SPEED_LOOP,
            },
        .motor_type = M3508,
    };
    for (unsigned i = 0; i < 4; ++i)
    {
        motor_config.can_init_config.tx_id = i + 1;
        drive_motors[i] = PowerControlInit(&motor_config);
        DJIMotorStop(drive_motors[i]);
    }
    // 舵向走原 DJIMotor 的角度环，CAN2 ID1~4；外部反馈为校零后的多圈角度。
    memset(&motor_config, 0, sizeof(motor_config));
    motor_config.motor_type = GM6020;
    motor_config.can_init_config.can_handle = CHASSIS_STEERING_CAN_HANDLE;
    motor_config.controller_setting_init_config.outer_loop_type = ANGLE_LOOP;
    motor_config.controller_setting_init_config.close_loop_type = ANGLE_LOOP;
    motor_config.controller_setting_init_config.angle_feedback_source = OTHER_FEED;
    // 此输出反向与下方编码器反馈取负号一起，沿用已调通的舵向正负关系。
    motor_config.controller_setting_init_config.feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE;
    motor_config.controller_param_init_config.angle_PID.Kp = SWERVE_STEERING_KP;
    motor_config.controller_param_init_config.angle_PID.MaxOut = SWERVE_STEERING_MAX_OUT;
    for (unsigned i = 0; i < 4; ++i)
    {
        motor_config.can_init_config.tx_id = i + 1;
        motor_config.controller_param_init_config.other_angle_feedback_ptr = &steering_feedback_deg[i];
        steering_motors[i] = DJIMotorInit(&motor_config);
        DJIMotorStop(steering_motors[i]);
    }
    INS_Init(); // 使用全向轮原 BMI088/EKF/DWT 实测时间间隔，上电保持静止完成校准。
    referee_data = UITaskInit(&huart6, &ui_data);
    SuperCap_Init_Config_s super_capacitor_config = {
        .can_config = {.can_handle = CHASSIS_CAP_CAN_HANDLE, .tx_id = CHASSIS_CAP_TX_ID, .rx_id = CHASSIS_CAP_RX_ID}};
    SuperCapInstance *super_capacitor = SuperCapInit(&super_capacitor_config);
    // MotorTask可能早于首次ChassisTask执行；上电即使用无裁判时的45W/0J回退值。
    super_capacitor->control.referee_power_limit = SWERVE_FALLBACK_POWER_W;
    super_capacitor->control.referee_energy_buffer = 0;
    SetPowerLimit(SWERVE_FALLBACK_POWER_W);
    chassis_command_subscriber = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feedback_publisher = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
    last_chassis_task_ms = HAL_GetTick();
    ResetChassisMotion();
    PublishChassisDebug();
}

/**
 * @brief 更新电机/INS在线状态，将实际轮速和舵角反算为底盘速度
 * @note 电机测量结构由CAN中断写入，此处在临界区内复制应用需要的字段。
 *       关中断仅防止某个测量结构在读取中途被修改，不保证八电机同时采样。
 *       反算结果写入chassis_feedback_data，并更新USB遥测中的底盘速度。
 */
static void UpdateChassisFeedback(void)
{
    float sum_forward_velocity_mps = 0;
    float sum_left_velocity_mps = 0;
    float sum_rotation_projection = 0;
    const float wheel_axis_offset_m = CHASSIS_CENTER_TO_WHEEL * 0.7071067811865475f;
    // 轮心位置使用解算的X向前、Y向左坐标，wheel_axis_offset_m为中心到轮心在单轴上的投影。
    const float wheel_position_x_m[4] = {wheel_axis_offset_m, wheel_axis_offset_m, -wheel_axis_offset_m,
                                         -wheel_axis_offset_m};
    const float wheel_position_y_m[4] = {wheel_axis_offset_m, -wheel_axis_offset_m, -wheel_axis_offset_m,
                                         wheel_axis_offset_m};
    chassis_state.motors_online = 1;
    // 读取期间禁止CAN回调改写测量结构，完成后恢复进入临界区前的中断状态。
    uint32_t interrupt_mask = __get_PRIMASK();
    __disable_irq();
    for (unsigned i = 0; i < 4; ++i)
    {
        chassis_state.drive_online[i] = IsChassisMotorOnline(drive_motors[i]);
        chassis_state.steering_online[i] = IsChassisMotorOnline(steering_motors[i]);
        if (!chassis_state.drive_online[i] || !chassis_state.steering_online[i])
        {
            chassis_state.motors_online = 0;
        }
        // GM6020 编码器减去机械零位并取反；保留多圈角，避免过零跳变。
        steering_feedback_deg[i] =
            -(steering_motors[i]->measure.total_angle - steering_zero_encoders[i] * (360.0f / 8192.0f));
        chassis_state.steering_actual_deg[i] = steering_feedback_deg[i];
        // 原电机模块反馈是转子度/秒，除以 6 才是 rpm。
        chassis_state.wheel_actual_rpm[i] = drive_motors[i]->measure.speed_aps / 6.0f;
    }
    __set_PRIMASK(interrupt_mask);
    // 单轮速度投影：wheel_forward_velocity_mps = v*cos(舵角)，wheel_left_velocity_mps = v*sin(舵角)。
    // 对称布局下，平移取四轮平均；旋转由sum(-y_i*wheel_forward_velocity_mps +
    // x_i*wheel_left_velocity_mps)/sum(r_i^2)得到。
    for (unsigned i = 0; i < 4; ++i)
    {
        float wheel_speed_mps =
            chassis_state.wheel_actual_rpm[i] * (CHASSIS_MOTOT_PER * SWERVE_DRIVE_REDUCTION) / 60.0f;
        float steering_angle_rad = steering_feedback_deg[i] * DEGREE_2_RAD;
        float wheel_forward_velocity_mps = wheel_speed_mps * cosf(steering_angle_rad);
        float wheel_left_velocity_mps = wheel_speed_mps * sinf(steering_angle_rad);
        sum_forward_velocity_mps += wheel_forward_velocity_mps;
        sum_left_velocity_mps += wheel_left_velocity_mps;
        sum_rotation_projection +=
            -wheel_position_y_m[i] * wheel_forward_velocity_mps + wheel_position_x_m[i] * wheel_left_velocity_mps;
    }
    // 250 = 1000/4：四轮求平均并将m/s换成mm/s；Y取负号转换回应用约定。
    chassis_feedback_data.real_vx = sum_forward_velocity_mps * 250.0f;
    chassis_feedback_data.real_vy = -sum_left_velocity_mps * 250.0f;
    chassis_feedback_data.real_wz = sum_rotation_projection / (4.0f * wheel_axis_offset_m * wheel_axis_offset_m * 2.0f);
    VisionSetChassisVel(chassis_feedback_data.real_vx, chassis_feedback_data.real_vy, chassis_feedback_data.real_wz);
    // 原 INS 发布完整快照；除新鲜度外还检查数值范围，拒绝 NaN 和异常姿态。
    chassis_state.ins_online = INS_GetSnapshot(&chassis_imu_snapshot) && isfinite(chassis_imu_snapshot.Yaw) &&
                               fabsf(chassis_imu_snapshot.Yaw) <= 180.1f && isfinite(chassis_imu_snapshot.Gyro[2]) &&
                               fabsf(chassis_imu_snapshot.Gyro[2]) <= 35.0f;
    if (chassis_state.ins_online)
    {
        chassis_state.yaw_deg = SWERVE_INS_YAW_SIGN * chassis_imu_snapshot.Yaw;
        chassis_state.gyro_z_radps = SWERVE_INS_YAW_SIGN * chassis_imu_snapshot.Gyro[2];
    }
}

/**
 * @brief 检查本轮控制消息中的有限数、幅值范围和模式编码
 * @return uint8_t 1:允许进入解算；0:消息含异常值或超出约定范围
 * @note 检查范围按robot_def.h中的单位约定解释，不能自动识别发送方是否使用了错误单位。
 */
static uint8_t IsChassisCommandValid(void)
{
    return isfinite(chassis_command.vx) && isfinite(chassis_command.vy) && isfinite(chassis_command.wz) &&
           isfinite(chassis_command.offset_angle) && isfinite(chassis_command.chassis_spin_speed) &&
           fabsf(chassis_command.vx) <= SWERVE_MAX_VX_MMPS && fabsf(chassis_command.vy) <= SWERVE_MAX_VY_MMPS &&
           fabsf(chassis_command.wz) <= SWERVE_MAX_WZ_RADPS &&
           fabsf(chassis_command.chassis_spin_speed) <= SWERVE_MAX_WZ_RADPS &&
           fabsf(chassis_command.offset_angle) <= 180.0f && chassis_command.mode <= 3 &&
           chassis_command.chassis_mode >= CHASSIS_ZERO_FORCE && chassis_command.chassis_mode <= CHASSIS_REMOTE;
}

/**
 * @brief 将车体目标解算为四组轮速/舵角目标，限幅后交给原电机框架
 * @note SwerveSolve已处理最短转向和驱动反转，此处再做四轮同比例限速及舵向误差补偿。
 *       本函数只更新参考值和使能状态，实际闭环和CAN发送在MotorControlTask中执行。
 */
static void ApplyWheelTargets(void)
{
    float rotor_rpm_per_mps = 60.0f / (CHASSIS_MOTOT_PER * SWERVE_DRIVE_REDUCTION);
    float maximum_rotor_rpm = SWERVE_MAX_WHEEL_MPS * rotor_rpm_per_mps;
    float peak_rotor_rpm = maximum_rotor_rpm;
    // 应用层 mm/s、Y 向右、rad/s，解算层 m/s、Y 向左、度/秒。
    SwerveSolve(&swerve_geometry, chassis_state.vx_body_mmps * 0.001f, -chassis_state.vy_body_mmps * 0.001f,
                chassis_state.wz_radps * RAD_2_DEGREE, steering_feedback_deg, wheel_targets);
    // 按最大轮速对四轮同比例缩放，保持平移与旋转的合成比例。
    for (unsigned i = 0; i < 4; ++i)
    {
        peak_rotor_rpm = fmaxf(peak_rotor_rpm, fabsf(wheel_targets[i].speed_rpm));
    }
    for (unsigned i = 0; i < 4; ++i)
    {
        float steering_error_rad = (wheel_targets[i].angle_deg - steering_feedback_deg[i]) * DEGREE_2_RAD;
        // 舵向未转到位时，只保留沿目标方向的牵引分量；误差达到90度则暂不给驱动速度。
        // 该补偿减小转向过程的横向拖拽，不是位置闭环，也不能消除INS航向漂移。
        float steering_alignment_ratio = fmaxf(0, cosf(steering_error_rad));
        chassis_state.wheel_target_rpm[i] =
            wheel_targets[i].speed_rpm * maximum_rotor_rpm / peak_rotor_rpm * steering_alignment_ratio;
        chassis_state.steering_target_deg[i] = wheel_targets[i].angle_deg;
        DJIMotorSetRef(drive_motors[i], chassis_state.wheel_target_rpm[i]);
        DJIMotorSetRef(steering_motors[i], wheel_targets[i].angle_deg);
        DJIMotorEnable(steering_motors[i]);
        DJIMotorEnable(drive_motors[i]);
    }
}

/** @brief 更新驱动功率上限和电容的裁判功率/缓冲；离线使用既有45W回退。 */
static void UpdateChassisPower(void)
{
    float referee_power_limit_w = RefereeHeatIsOnline() ? referee_data->GameRobotState.chassis_power_limit : 0;
    SetPowerLimit(referee_power_limit_w > 0 ? referee_power_limit_w : SWERVE_FALLBACK_POWER_W);
    SuperCapInstance *super_capacitor = GetSuperCapInstance();
    if (super_capacitor != NULL)
    {
        // 原电容协议通过CAN携带裁判限制，不改变电容收发ID。
        super_capacitor->control.referee_power_limit =
            referee_power_limit_w > 0 ? referee_power_limit_w : SWERVE_FALLBACK_POWER_W;
        super_capacitor->control.referee_energy_buffer =
            RefereeHeatIsOnline() ? referee_data->PowerHeatData.buffer_energy : 0;
    }
}

/** @brief 发布速度、姿态、在线状态及裁判热量；无力时也持续回传。 */
static void PublishChassisFeedback(void)
{
    // 两端共享实际轮速和裁判热量；未收裁判首帧时不能拿零值当作允许发射。
    chassis_feedback_data.motors_online = chassis_state.motors_online;
    chassis_feedback_data.ins_online = chassis_state.ins_online;
    chassis_feedback_data.yaw_deg = chassis_state.yaw_deg;
    uint32_t interrupt_mask = __get_PRIMASK();
    __disable_irq();
    chassis_feedback_data.referee_online = RefereeHeatIsOnline();
    chassis_feedback_data.heat_limit = referee_data->GameRobotState.shooter_barrel_heat_limit;
    chassis_feedback_data.current_heat = referee_data->PowerHeatData.shooter_17mm_barrel_heat;
    chassis_feedback_data.current_hp = referee_data->GameRobotState.current_HP;
    __set_PRIMASK(interrupt_mask);
    unsigned remaining_heat = chassis_feedback_data.heat_limit > chassis_feedback_data.current_heat
                                  ? chassis_feedback_data.heat_limit - chassis_feedback_data.current_heat
                                  : 0;
    chassis_feedback_data.rest_heat = remaining_heat > 255u ? 255u : remaining_heat;
    PubPushMessage(chassis_feedback_publisher, &chassis_feedback_data);
}

void ChassisTask(void)
{
    uint32_t current_time_ms = HAL_GetTick();
    uint32_t elapsed_ms = (uint32_t)(current_time_ms - last_chassis_task_ms);
    chassis_mode_e previous_mode = chassis_state.mode;
    last_chassis_task_ms = current_time_ms;
    /* 1. 更新实际状态，随后读取本轮CMD消息；消息为空不能继续执行旧目标。 */
    UpdateChassisFeedback();
    // command_online 只代表本轮收到 CMD 消息；遥控失联时 CMD 仍会发布无力指令。
    chassis_state.command_online = SubGetMessage(chassis_command_subscriber, &chassis_command);
    /* 2. 统一保护判定。任一轮失联即停止八电机，避免只靠剩余轮组牵引。
     * elapsed检查的是本任务两次执行间隔；小陀螺和导航坐标变换还依赖有效INS。 */
    if (!chassis_state.command_online || !IsChassisCommandValid() || !chassis_state.motors_online ||
        elapsed_ms > SWERVE_COMMAND_TIMEOUT_MS || chassis_command.chassis_mode == CHASSIS_ZERO_FORCE ||
        ((chassis_command.chassis_mode == CHASSIS_ROTATE || chassis_command.mode == 1) && !chassis_state.ins_online))
    {
        StopChassisMotors();
    }
    else
    {
        chassis_state.mode = chassis_command.chassis_mode;
        ChassisMotionUpdate(&chassis_motion_state, &chassis_motion_config, &chassis_command, previous_mode,
                            chassis_state.yaw_deg, chassis_state.gyro_z_radps, elapsed_ms);
        chassis_state.relative_yaw_deg = chassis_motion_state.relative_yaw_deg;
        chassis_state.spin_reference_yaw_deg = chassis_motion_state.spin_reference_yaw_deg;
        chassis_state.spin_rate_ref_radps = chassis_motion_state.spin_rate_reference_radps;
        chassis_state.vx_body_mmps = chassis_motion_state.forward_velocity_mmps;
        chassis_state.vy_body_mmps = chassis_motion_state.right_velocity_mmps;
        chassis_state.wz_radps = chassis_motion_state.yaw_rate_radps;
        ApplyWheelTargets();
    }
    /* 更新驱动功率上限并发布底盘反馈；无力模式下也保持状态消息流转。 */
    UpdateChassisPower();
    PublishChassisFeedback();
    PublishChassisDebug();
}

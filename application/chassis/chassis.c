/**
 * @file chassis.c
 * @brief 四舵轮底盘应用，负责模式处理、坐标变换、运动学解算和底盘状态反馈
 *
 * 数据流：CMD发布chassis_cmd -> 本应用订阅 -> SwerveSolve解算四轮目标
 *         -> DJIMotorSetRef更新电机目标 -> 原MotorControlTask闭环并发送CAN报文。
 * 本文件不直接发送电机控制帧，驱动轮由PowerControl管理，舵向轮由DJIMotor管理。
 *
 * 应用层速度约定：X向前、Y向右，平移单位mm/s，旋转单位rad/s，俯视逆时针为正。
 * 解算模块约定：X向前、Y向左，平移单位m/s，旋转输入为度/秒；在apply_targets中转换。
 * 所有轮子数组均按左前、右前、右后、左后排列，索引对应0、1、2、3。
 */
#include "chassis.h"
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
static DJIMotorInstance *drive[4], *steer[4];        // 四个驱动轮和四个舵向轮，轮序一一对应
static Subscriber_t *chassis_sub;                   // 订阅CMD发布的chassis_cmd话题
static Publisher_t *chassis_pub;                    // 发布chassis_feed，供CMD读取底盘反馈
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 本轮收到的底盘目标，消息为空时不能沿用旧目标
static Chassis_Upload_Data_s chassis_feedback_data; // 由实际轮速、舵角反算的车体速度
static referee_info_t *referee_data;                // 裁判系统反馈，读取底盘功率上限
static Referee_Interactive_info_t ui_data;          // 原裁判UI模块使用的交互数据
static attitude_t imu;                             // 本轮INS快照，必须配合ins_online使用
static PIDInstance spin_pid;                       // 小陀螺角速度误差的比例修正，输入和输出均为rad/s
static uint32_t last_task_tick;                     // 上轮底盘任务时间，用于转速爬升和任务超时检查

/* 运行状态只在底盘应用内部使用；调试快照不作为控制输入。 */
typedef struct
{
    chassis_mode_e mode;                 // 当前实际执行模式，保护停机时为无力。
    uint8_t command_online;              // 本轮是否收到 CMD 消息，不代表遥控器在线。
    uint8_t motors_online;               // 八电机全部有有效且未过期的反馈时为 1。
    uint8_t ins_online;                  // INS 快照新鲜且航向、角速度数值正常。
    uint8_t drive_online[4];            // 四个 M3508 反馈状态。
    uint8_t steering_online[4];         // 四个 GM6020 反馈状态。
    float yaw_deg;                      // 底盘 INS 航向，度。
    float gyro_z_radps;                 // Z 轴角速度，rad/s，俯视逆时针为正。
    float relative_yaw_deg;             // 本轮速度坐标转换使用的相对角，度。
    float spin_reference_yaw_deg;       // 进入小陀螺时捕获的底盘朝向，度。
    float spin_rate_ref_radps;          // 经角加速度限制后的旋转参考，rad/s。
    float vx_body_mmps;                 // 车体系 X 向前目标，mm/s。
    float vy_body_mmps;                 // 车体系 Y 向右目标，mm/s。
    float wz_radps;                     // 实际送入运动学的角速度目标，rad/s。
    float wheel_target_rpm[4];          // 驱动转子目标，经整车限速和舵向误差补偿。
    float wheel_actual_rpm[4];          // 驱动转子实际 rpm，尚未乘减速比。
    float steering_target_deg[4];       // 舵向多圈目标，度。
    float steering_actual_deg[4];       // 舵向实际多圈角，已处理机械零位和方向，度。
} ChassisState_s;
static ChassisState_s chassis_state;
SwerveDebug_s swerve_debug;
static const SwerveGeometry_s swerve_geometry = {
    .center_to_wheel_m = CHASSIS_CENTER_TO_WHEEL,
    .wheel_perimeter_m = CHASSIS_MOTOT_PER,
    .drive_reduction = SWERVE_DRIVE_REDUCTION};
static SwerveTarget target[4];                     // 运动学输出，零速时保留其中上一轮的舵向目标
static float steer_feedback_deg[4];                // 校零且修正方向的多圈舵角，同时作为角度环反馈
static const uint16_t zero_ecd[4] = SWERVE_ZERO_ENCODERS; // 四个轮子机械朝前时的编码器刻度

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
    memcpy(swerve_debug.steering_target_deg, chassis_state.steering_target_deg, sizeof(swerve_debug.steering_target_deg));
    memcpy(swerve_debug.steering_actual_deg, chassis_state.steering_actual_deg, sizeof(swerve_debug.steering_actual_deg));
}

/**
 * @brief 对数值进行关于零对称的限幅
 * @param value 待限幅数值
 * @param limit 非负幅值上限，单位与value相同
 * @return float 限制在[-limit, limit]内的数值
 */
static float clamp(float value, float limit)
{
    return fmaxf(-limit, fminf(limit, value));
}

/**
 * @brief 将角度差归一化到[-180, 180]，供速度坐标变换使用
 * @param value 角度差，单位为度
 * @return float 归一化后的角度差，单位为度
 */
static float wrap_deg(float value)
{
    return SwerveNormalizeRad(value * DEGREE_2_RAD) * RAD_2_DEGREE;
}

referee_info_t *get_referr_data_pr(void) { return referee_data; }

/**
 * @brief 检查单个电机是否已收到有效反馈，且反馈尚未过期
 * @param motor 已完成初始化的电机实例，不可为NULL
 * @return uint8_t 1:反馈有效；0:尚未收帧或超过SWERVE_MOTOR_TIMEOUT_MS
 * @note 在读取电机反馈的临界区中调用，不将上电时全零的测量结构视为在线。
 */
static uint8_t motor_online(DJIMotorInstance *motor)
{
    return motor->feedback_seen &&
        (uint32_t)(HAL_GetTick() - motor->last_rx_ms) <= SWERVE_MOTOR_TIMEOUT_MS;
}

/**
 * @brief 重置小陀螺转速修正器，将当前航向记为下一次旋转的平移参考
 * @note 无力、非旋转模式和刚进入小陀螺时都会调用；持续旋转期间不重复捕获航向。
 *       复位后角速度参考从零开始爬升，避免直接给定完整转速。
 */
static void spin_reset(void)
{
    PID_Init_Config_s cfg = {.Kp = SWERVE_SPIN_RATE_KP,
        .MaxOut = SWERVE_SPIN_CORRECTION_DPS * DEGREE_2_RAD};
    PIDInit(&spin_pid, &cfg);
    chassis_state.spin_rate_ref_radps = 0;
    chassis_state.spin_reference_yaw_deg = chassis_state.yaw_deg;
}

/**
 * @brief 将八电机置为停止状态，并复位目标缓存与小陀螺状态
 * @note DJIMotorStop使电机任务发送零控制量；仅设置零轮速仍会闭环制动，不是无力模式。
 *       舵角目标缓存跟随当前实际舵角，恢复后不追赶停机前的旧目标。
 */
static void stop_all(void)
{
    for (unsigned i = 0; i < 4; ++i)
    {
        DJIMotorStop(drive[i]);
        DJIMotorStop(steer[i]);
        DJIMotorSetRef(drive[i], 0);
        DJIMotorSetRef(steer[i], steer_feedback_deg[i]);
        target[i].speed_rpm = 0;
        target[i].angle_deg = steer_feedback_deg[i];
        chassis_state.wheel_target_rpm[i] = 0;
        chassis_state.steering_target_deg[i] = steer_feedback_deg[i];
    }
    chassis_state.mode = CHASSIS_ZERO_FORCE;
    chassis_state.vx_body_mmps = chassis_state.vy_body_mmps = chassis_state.wz_radps = 0;
    spin_reset();
}

void ChassisInit(void)
{
    // 驱动轮走原 PowerControl 的 rpm 速度环，CAN1 ID1~4。
    Motor_Init_Config_s cfg = {
        .can_init_config.can_handle = CHASSIS_DRIVE_CAN_HANDLE,
        .controller_param_init_config.speed_PID = {
            .Kp = SWERVE_DRIVE_KP, .MaxOut = SWERVE_DRIVE_MAX_OUT},
        .controller_setting_init_config = {
            .outer_loop_type = SPEED_LOOP, .close_loop_type = SPEED_LOOP},
        .motor_type = M3508};
    for (unsigned i = 0; i < 4; ++i)
    {
        cfg.can_init_config.tx_id = i + 1;
        drive[i] = PowerControlInit(&cfg);
        DJIMotorStop(drive[i]);
    }
    // 舵向走原 DJIMotor 的角度环，CAN2 ID1~4；外部反馈为校零后的多圈角度。
    memset(&cfg, 0, sizeof(cfg));
    cfg.motor_type = GM6020;
    cfg.can_init_config.can_handle = CHASSIS_STEERING_CAN_HANDLE;
    cfg.controller_setting_init_config.outer_loop_type = ANGLE_LOOP;
    cfg.controller_setting_init_config.close_loop_type = ANGLE_LOOP;
    cfg.controller_setting_init_config.angle_feedback_source = OTHER_FEED;
    // 此输出反向与下方编码器反馈取负号一起，沿用已调通的舵向正负关系。
    cfg.controller_setting_init_config.feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE;
    cfg.controller_param_init_config.angle_PID.Kp = SWERVE_STEERING_KP;
    cfg.controller_param_init_config.angle_PID.MaxOut = SWERVE_STEERING_MAX_OUT;
    for (unsigned i = 0; i < 4; ++i)
    {
        cfg.can_init_config.tx_id = i + 1;
        cfg.controller_param_init_config.other_angle_feedback_ptr = &steer_feedback_deg[i];
        steer[i] = DJIMotorInit(&cfg);
        DJIMotorStop(steer[i]);
    }
    INS_Init(); // 使用全向轮原 BMI088/EKF/DWT 实测时间间隔，上电保持静止完成校准。
    referee_data = UITaskInit(&huart6, &ui_data);
    SuperCap_Init_Config_s cap_cfg = {
        .can_config = {.can_handle = CHASSIS_CAP_CAN_HANDLE,
            .tx_id = CHASSIS_CAP_TX_ID, .rx_id = CHASSIS_CAP_RX_ID}};
    SuperCapInstance *cap = SuperCapInit(&cap_cfg);
    // MotorTask可能早于首次ChassisTask执行；上电即使用无裁判时的45W/0J回退值。
    cap->control.referee_power_limit = SWERVE_FALLBACK_POWER_W;
    cap->control.referee_energy_buffer = 0;
    SetPowerLimit(SWERVE_FALLBACK_POWER_W);
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
    last_task_tick = HAL_GetTick();
    spin_reset();
    PublishChassisDebug();
}

/**
 * @brief 更新电机/INS在线状态，将实际轮速和舵角反算为底盘速度
 * @note 电机测量结构由CAN中断写入，此处在临界区内复制应用需要的字段。
 *       关中断仅防止某个测量结构在读取中途被修改，不保证八电机同时采样。
 *       反算结果写入chassis_feedback_data，并更新USB遥测中的底盘速度。
 */
static void update_feedback(void)
{
    float sum_x = 0, sum_y = 0, sum_w = 0;
    const float a = CHASSIS_CENTER_TO_WHEEL * 0.7071067811865475f;
    // 轮心位置使用解算的X向前、Y向左坐标，a为中心到轮心在单轴上的投影。
    const float px[4] = {a, a, -a, -a}, py[4] = {a, -a, -a, a};
    chassis_state.motors_online = 1;
    // 读取期间禁止CAN回调改写测量结构，完成后恢复进入临界区前的中断状态。
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    for (unsigned i = 0; i < 4; ++i)
    {
        chassis_state.drive_online[i] = motor_online(drive[i]);
        chassis_state.steering_online[i] = motor_online(steer[i]);
        if (!chassis_state.drive_online[i] || !chassis_state.steering_online[i])
            chassis_state.motors_online = 0;
        // GM6020 编码器减去机械零位并取反；保留多圈角，避免过零跳变。
        steer_feedback_deg[i] = -(steer[i]->measure.total_angle -
            zero_ecd[i] * (360.0f / 8192.0f));
        chassis_state.steering_actual_deg[i] = steer_feedback_deg[i];
        // 原电机模块反馈是转子度/秒，除以 6 才是 rpm。
        chassis_state.wheel_actual_rpm[i] = drive[i]->measure.speed_aps / 6.0f;
    }
    __set_PRIMASK(irq);
    // 单轮速度投影：wheel_x = v*cos(舵角)，wheel_y = v*sin(舵角)。
    // 对称布局下，平移取四轮平均；旋转由sum(-y_i*wheel_x + x_i*wheel_y)/sum(r_i^2)得到。
    for (unsigned i = 0; i < 4; ++i)
    {
        float speed = chassis_state.wheel_actual_rpm[i] *
            (CHASSIS_MOTOT_PER * SWERVE_DRIVE_REDUCTION) / 60.0f;
        float angle = steer_feedback_deg[i] * DEGREE_2_RAD;
        float wheel_x = speed * cosf(angle), wheel_y = speed * sinf(angle);
        sum_x += wheel_x;
        sum_y += wheel_y;
        sum_w += -py[i] * wheel_x + px[i] * wheel_y;
    }
    // 250 = 1000/4：四轮求平均并将m/s换成mm/s；Y取负号转换回应用约定。
    chassis_feedback_data.real_vx = sum_x * 250.0f;
    chassis_feedback_data.real_vy = -sum_y * 250.0f;
    chassis_feedback_data.real_wz = sum_w / (4.0f * a * a * 2.0f);
    VisionSetChassisVel(chassis_feedback_data.real_vx,
        chassis_feedback_data.real_vy, chassis_feedback_data.real_wz);
    // 原 INS 发布完整快照；除新鲜度外还检查数值范围，拒绝 NaN 和异常姿态。
    chassis_state.ins_online = INS_GetSnapshot(&imu) && isfinite(imu.Yaw) && fabsf(imu.Yaw) <= 180.1f &&
        isfinite(imu.Gyro[2]) && fabsf(imu.Gyro[2]) <= 35.0f;
    if (chassis_state.ins_online)
    {
        chassis_state.yaw_deg = SWERVE_INS_YAW_SIGN * imu.Yaw;
        chassis_state.gyro_z_radps = SWERVE_INS_YAW_SIGN * imu.Gyro[2];
    }
}

/**
 * @brief 检查本轮控制消息中的有限数、幅值范围和模式编码
 * @return uint8_t 1:允许进入解算；0:消息含异常值或超出约定范围
 * @note 检查范围按robot_def.h中的单位约定解释，不能自动识别发送方是否使用了错误单位。
 */
static uint8_t command_valid(void)
{
    return isfinite(chassis_cmd_recv.vx) && isfinite(chassis_cmd_recv.vy) &&
        isfinite(chassis_cmd_recv.wz) && isfinite(chassis_cmd_recv.offset_angle) &&
        isfinite(chassis_cmd_recv.chassis_spin_speed) &&
        fabsf(chassis_cmd_recv.vx) <= SWERVE_MAX_VX_MMPS &&
        fabsf(chassis_cmd_recv.vy) <= SWERVE_MAX_VY_MMPS &&
        fabsf(chassis_cmd_recv.wz) <= SWERVE_MAX_WZ_RADPS &&
        fabsf(chassis_cmd_recv.chassis_spin_speed) <= SWERVE_MAX_WZ_RADPS &&
        fabsf(chassis_cmd_recv.offset_angle) <= 180.0f &&
        chassis_cmd_recv.mode <= 3 && chassis_cmd_recv.chassis_mode >= CHASSIS_ZERO_FORCE &&
        chassis_cmd_recv.chassis_mode <= CHASSIS_REMOTE;
}

/**
 * @brief 将车体目标解算为四组轮速/舵角目标，限幅后交给原电机框架
 * @note SwerveSolve已处理最短转向和驱动反转，此处再做四轮同比例限速及舵向误差补偿。
 *       本函数只更新参考值和使能状态，实际闭环和CAN发送在MotorControlTask中执行。
 */
static void apply_targets(void)
{
    float ratio = 60.0f / (CHASSIS_MOTOT_PER * SWERVE_DRIVE_REDUCTION);
    float max_rpm = SWERVE_MAX_WHEEL_MPS * ratio, peak = max_rpm;
    // 应用层 mm/s、Y 向右、rad/s，解算层 m/s、Y 向左、度/秒。
    SwerveSolve(&swerve_geometry, chassis_state.vx_body_mmps * 0.001f,
        -chassis_state.vy_body_mmps * 0.001f, chassis_state.wz_radps * RAD_2_DEGREE,
        steer_feedback_deg, target);
    // 按最大轮速对四轮同比例缩放，保持平移与旋转的合成比例。
    for (unsigned i = 0; i < 4; ++i) peak = fmaxf(peak, fabsf(target[i].speed_rpm));
    for (unsigned i = 0; i < 4; ++i)
    {
        float error = (target[i].angle_deg - steer_feedback_deg[i]) * DEGREE_2_RAD;
        // 舵向未转到位时，只保留沿目标方向的牵引分量；误差达到90度则暂不给驱动速度。
        // 该补偿减小转向过程的横向拖拽，不是位置闭环，也不能消除INS航向漂移。
        float alignment = fmaxf(0, cosf(error));
        chassis_state.wheel_target_rpm[i] = target[i].speed_rpm * max_rpm / peak * alignment;
        chassis_state.steering_target_deg[i] = target[i].angle_deg;
        DJIMotorSetRef(drive[i], chassis_state.wheel_target_rpm[i]);
        DJIMotorSetRef(steer[i], target[i].angle_deg);
        DJIMotorEnable(steer[i]);
        DJIMotorEnable(drive[i]);
    }
}

void ChassisTask(void)
{
    uint32_t now = HAL_GetTick(), elapsed = (uint32_t)(now - last_task_tick);
    chassis_mode_e previous = chassis_state.mode;
    last_task_tick = now;
    /* 1. 更新实际状态，随后读取本轮CMD消息；消息为空不能继续执行旧目标。 */
    update_feedback();
    // command_online 只代表本轮收到 CMD 消息；遥控失联时 CMD 仍会发布无力指令。
    chassis_state.command_online = SubGetMessage(chassis_sub, &chassis_cmd_recv);
    /* 2. 统一保护判定。任一轮失联即停止八电机，避免只靠剩余轮组牵引。
     * elapsed检查的是本任务两次执行间隔；小陀螺和导航坐标变换还依赖有效INS。 */
    if (!chassis_state.command_online || !command_valid() || !chassis_state.motors_online ||
        elapsed > SWERVE_COMMAND_TIMEOUT_MS || chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE ||
        ((chassis_cmd_recv.chassis_mode == CHASSIS_ROTATE || chassis_cmd_recv.mode == 1) &&
         !chassis_state.ins_online))
        stop_all();
    else
    {
        float angle = 0;
        float wz = chassis_cmd_recv.wz;
        chassis_state.mode = chassis_cmd_recv.chassis_mode;
        /* 3. 先确定旋转目标：小陀螺用转速参考加反馈修正，其余模式沿用底盘模式约定。 */
        if (chassis_state.mode == CHASSIS_ROTATE)
        {
            // 只在模式入口重捕获朝向；切换左拨杆不影响参考方向。
            if (previous != CHASSIS_ROTATE) spin_reset();
            // 用实际任务间隔限制角加速度，再以陀螺仪角速度反馈补偿转速误差。
            float step = SWERVE_SPIN_ACCEL_DPS2 * DEGREE_2_RAD * elapsed * 0.001f;
            chassis_state.spin_rate_ref_radps += clamp(chassis_cmd_recv.chassis_spin_speed -
                chassis_state.spin_rate_ref_radps, step);
            wz = chassis_state.spin_rate_ref_radps + PIDCalculate(&spin_pid,
                chassis_state.gyro_z_radps, chassis_state.spin_rate_ref_radps);
        }
        else
        {
            spin_reset();
            if (chassis_state.mode == CHASSIS_NO_FOLLOW) wz = 0;
            if (chassis_state.mode == CHASSIS_FOLLOW_GIMBAL_YAW)
                wz = 0.15f * chassis_cmd_recv.offset_angle;
        }
        /* 4. 选择平移参考系，再把输入速度投影到车体系。
         * mode=0不旋转输入速度；1使用INS与外部参考角；2使用外部相对角；
         * mode=3使用进入小陀螺时捕获的航向，本地遥控只发布0和3。 */
        if (chassis_cmd_recv.mode == 1)
            angle = chassis_state.yaw_deg - chassis_cmd_recv.offset_angle;
        else if (chassis_cmd_recv.mode == 2)
            angle = chassis_cmd_recv.offset_angle;
        else if (chassis_cmd_recv.mode == 3 && chassis_state.mode == CHASSIS_ROTATE)
            angle = chassis_state.yaw_deg - chassis_state.spin_reference_yaw_deg;
        // 先归一化相对航向，跨越 ±180 度时平移映射仍连续。
        chassis_state.relative_yaw_deg = wrap_deg(angle);
        angle = chassis_state.relative_yaw_deg * DEGREE_2_RAD;
        chassis_state.vx_body_mmps = chassis_cmd_recv.vx * cosf(angle) - chassis_cmd_recv.vy * sinf(angle);
        chassis_state.vy_body_mmps = chassis_cmd_recv.vx * sinf(angle) + chassis_cmd_recv.vy * cosf(angle);
        chassis_state.wz_radps = clamp(wz, SWERVE_MAX_WZ_RADPS);
        /* 5. 四舵轮解算、整车轮速限幅和舵向误差补偿，最后更新八电机目标。 */
        apply_targets();
    }
    /* 6. 更新驱动功率上限并发布底盘反馈；无力模式下也保持状态消息流转。 */
    float limit = RefereeHeatIsOnline() ? referee_data->GameRobotState.chassis_power_limit : 0;
    SetPowerLimit(limit > 0 ? limit : SWERVE_FALLBACK_POWER_W);
    SuperCapInstance *cap = GetSuperCapInstance();
    if (cap != NULL)
    {
        // 原电容协议通过CAN携带裁判限制，不改变电容收发ID。
        cap->control.referee_power_limit = limit > 0 ? limit : SWERVE_FALLBACK_POWER_W;
        cap->control.referee_energy_buffer = RefereeHeatIsOnline() ?
            referee_data->PowerHeatData.buffer_energy : 0;
    }
    // 两端共享实际轮速和裁判热量；未收裁判首帧时不能拿零值当作允许发射。
    chassis_feedback_data.motors_online = chassis_state.motors_online;
    chassis_feedback_data.ins_online = chassis_state.ins_online;
    chassis_feedback_data.yaw_deg = chassis_state.yaw_deg;
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    chassis_feedback_data.referee_online = RefereeHeatIsOnline();
    chassis_feedback_data.heat_limit = referee_data->GameRobotState.shooter_barrel_heat_limit;
    chassis_feedback_data.current_heat = referee_data->PowerHeatData.shooter_17mm_barrel_heat;
    chassis_feedback_data.current_hp = referee_data->GameRobotState.current_HP;
    __set_PRIMASK(irq);
    unsigned remaining = chassis_feedback_data.heat_limit > chassis_feedback_data.current_heat ?
        chassis_feedback_data.heat_limit - chassis_feedback_data.current_heat : 0;
    chassis_feedback_data.rest_heat = remaining > 255u ? 255u : remaining;
    PubPushMessage(chassis_pub, &chassis_feedback_data);
    PublishChassisDebug();
}

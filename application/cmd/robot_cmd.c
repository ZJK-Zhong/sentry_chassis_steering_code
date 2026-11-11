/**
 * @file robot_cmd.c
 * @brief 底盘指令适配层，将本地DBUS或云台板CAN命令转换为底盘目标
 *
 * SWERVE_COMMAND_SOURCE选择唯一指令来源。双板模式沿用原CANComm帧和打包消息，
 * 底盘CAN1以0x312接收云台命令、以0x311回传底盘反馈；物理端口见board_config.h。
 *
 * RobotCMDTask由RobotTask先于ChassisTask调用，通过原message_center发布chassis_cmd。
 * 本应用不访问电机实例；八电机保护、INS要求和运动学由底盘应用处理。
 */
#include "robot_def.h"
#include "board_link.h"
#include "robot_cmd.h"
#include "message_center.h"
#include "swerve_config.h"
#include "string.h"

#if SWERVE_COMMAND_SOURCE == SWERVE_COMMAND_SOURCE_LOCAL_RC
#include "remote_control.h"
#elif SWERVE_COMMAND_SOURCE == SWERVE_COMMAND_SOURCE_GIMBAL_CAN
#include "board_transport.h"
#include "math.h"
#endif

/* 每轮只有所选编译分支会初始化并读取对应的控制来源。 */
static Publisher_t *chassis_command_publisher; ///< 发布统一底盘目标话题，保持ChassisTask与控制源解耦。
#if SWERVE_COMMAND_SOURCE == SWERVE_COMMAND_SOURCE_LOCAL_RC
static Subscriber_t *chassis_feedback_subscriber;       ///< 订阅底盘状态，消息类型为Chassis_Upload_Data_s
static Chassis_Upload_Data_s chassis_feedback_snapshot; ///< 取出底盘反馈，当前不用于改变控制模式
static uint8_t local_remote_armed;                      ///< 上电/遥控无效时为0，只有收到右下档才置1

/**
 * @brief 将遥控量转换后的目标限制在正负同一幅值内
 * @param value 待限幅目标
 * @param limit 非负上限，单位与value一致
 * @return float 限幅后的目标
 */
static float ClampLocalCommand(float value, float limit)
{
    return value > limit ? limit : (value < -limit ? -limit : value);
}

/**
 * @brief 对遥控原始通道施加中位死区，抑制松杆后的微小速度指令
 * @param value 减去DBUS中位偏置后的有符号通道值
 * @return int16_t 死区内返回0，死区外保留原值，不做重新缩放
 */
static int16_t ApplyLocalRemoteDeadband(int16_t value)
{
    return value > SWERVE_RC_DEADBAND || value < -SWERVE_RC_DEADBAND ? value : 0;
}
#elif SWERVE_COMMAND_SOURCE == SWERVE_COMMAND_SOURCE_GIMBAL_CAN
static Subscriber_t *chassis_feedback_subscriber;       ///< 订阅底盘任务发布的实际速度反馈。
static Chassis_Upload_Data_s chassis_feedback_snapshot; ///< CAN反馈的固定长度发送缓存。
static Chassis_Ctrl_Cmd_s gimbal_command;               ///< 最近一次校验通过的云台命令。
static uint8_t gimbal_command_valid;                    ///< 首帧前或通信超时后为0，禁止重放旧命令。

/**
 * @brief 验证云台板发来的底盘命令数值、单位范围和模式编码
 * @param command 已从CANComm接收缓冲区复制出的完整命令
 * @return uint8_t 1表示可以交给底盘任务，0表示必须清空目标
 * @note CRC由CANComm检查；此处防止有效CRC但字段越界或包含NaN/Inf的消息进入控制环。
 */
static uint8_t IsGimbalCommandValid(const Chassis_Ctrl_Cmd_s *command)
{
    return isfinite(command->vx) && isfinite(command->vy) && isfinite(command->wz) && isfinite(command->offset_angle) &&
           isfinite(command->chassis_spin_speed) && fabsf(command->vx) <= SWERVE_MAX_VX_MMPS &&
           fabsf(command->vy) <= SWERVE_MAX_VY_MMPS && fabsf(command->wz) <= SWERVE_MAX_WZ_RADPS &&
           fabsf(command->chassis_spin_speed) <= SWERVE_MAX_WZ_RADPS && fabsf(command->offset_angle) <= 180.0f &&
           command->mode <= 3 && command->chassis_mode >= CHASSIS_ZERO_FORCE && command->chassis_mode <= CHASSIS_REMOTE;
}
#endif

void RobotCMDInit(void)
{
    // 保留原USB/INS遥测初始化；当前CMD不通过上位机串口下发运动目标。
    VisionInit(&huart1);
#if SWERVE_COMMAND_SOURCE == SWERVE_COMMAND_SOURCE_LOCAL_RC
    // 接收机安装在底盘板时，遥控目标经原消息中心交给ChassisTask。
    RemoteControlInit(&huart3);
    chassis_command_publisher = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feedback_subscriber = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
    local_remote_armed = 0;
#elif SWERVE_COMMAND_SOURCE == SWERVE_COMMAND_SOURCE_GIMBAL_CAN
    // 通信实例、端口、超时和发送周期由板间适配服务管理。
    BoardTransportInit();
    chassis_command_publisher = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feedback_subscriber = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
    memset(&gimbal_command, 0, sizeof(gimbal_command));
    gimbal_command_valid = 0;
#endif
}

void RobotCMDTask(void)
{
#if SWERVE_COMMAND_SOURCE == SWERVE_COMMAND_SOURCE_LOCAL_RC
    RC_ctrl_t remote;
    Chassis_Ctrl_Cmd_s chassis_command = {0}; ///< 默认无力；任何失联或未解锁状态均发布零指令。
    SubGetMessage(chassis_feedback_subscriber, &chassis_feedback_snapshot);
    /* 1. 读取遥控快照并维护解锁状态。
     * 无效/过期帧立即撤销解锁；右下档只解锁，当前这一轮仍发布无力命令。
     * 下一轮收到中档或上档才允许运动，避免上电时拨杆已在运动档而直接起转。 */
    if (!RemoteControlSnapshot(&remote))
    {
        local_remote_armed = 0;
    }
    else if (switch_is_down(remote.rc.switch_right))
    {
        local_remote_armed = 1;
    }
    else if (local_remote_armed)
    {
        /* 2. 摇杆映射：ch1前后、ch0左右，先过死区，再乘比例并限幅。
         * X向前为正，Y向右为正；CMD只使用mm/s，轮速rpm在底盘解算后产生。 */
        chassis_command.vx =
            ClampLocalCommand(ApplyLocalRemoteDeadband(remote.rc.rocker_r1) * 6.0f, SWERVE_MAX_VX_MMPS);
        chassis_command.vy =
            ClampLocalCommand(ApplyLocalRemoteDeadband(remote.rc.rocker_r_) * 2.0f, SWERVE_MAX_VY_MMPS);
        /* 3. 仅由右拨杆选择运动模式；左拨杆只在遥控模块中校验编码合法性。 */
        if (switch_is_mid(remote.rc.switch_right))
        {
            chassis_command.chassis_mode = CHASSIS_REMOTE;
            // ch2 的负号对应已确认的实车转向，角速度换算成 rad/s。
            chassis_command.wz = -ApplyLocalRemoteDeadband(remote.rc.rocker_l_) * 0.2f * 0.01745329251994329577f;
        }
        else if (switch_is_up(remote.rc.switch_right))
        {
            chassis_command.chassis_mode = CHASSIS_ROTATE;
            chassis_command.mode = 3; ///< 平移以进入小陀螺时的底盘朝向为参考，参考角由底盘捕获。
            chassis_command.chassis_spin_speed = SWERVE_SPIN_DPS * 0.01745329251994329577f;
        }
    }
    // 每轮都发布消息，包括无力状态；底盘可据此区分“CMD停止”与“本轮没有CMD消息”。
    PubPushMessage(chassis_command_publisher, &chassis_command);
#elif SWERVE_COMMAND_SOURCE == SWERVE_COMMAND_SOURCE_GIMBAL_CAN
    Chassis_Ctrl_Cmd_s received_command;
    Chassis_Ctrl_Cmd_s chassis_command = {0}; ///< 未收到首帧、帧无效或通信超时均发送无力目标。

    /* 快照在关中断的短临界区复制完整报文，并按实际收帧时间判断50ms超时。
     * CMD每轮发布消息不能充当链路心跳，否则CAN断线后会一直重放旧目标。 */
    if (BoardTransportReadCommand(&received_command) && IsGimbalCommandValid(&received_command))
    {
        gimbal_command = received_command;
        gimbal_command_valid = 1;
        chassis_command = gimbal_command;
    }
    else
    {
        memset(&gimbal_command, 0, sizeof(gimbal_command));
        gimbal_command_valid = 0;
    }

    // 每轮仍经消息中心发布，底盘任务无需知道目标来自本地遥控还是另一块主控板。
    PubPushMessage(chassis_command_publisher, &chassis_command);

    // 底盘实际速度由ChassisTask发布；以50Hz回传，限制与驱动轮及Yaw共享总线的负载。
    SubGetMessage(chassis_feedback_subscriber, &chassis_feedback_snapshot);
    BoardTransportSendFeedback(&chassis_feedback_snapshot);
#endif
}

/**
 * @file chassis.h
 * @brief 四舵轮底盘应用的初始化、周期任务和调试数据接口
 *
 * 底盘应用通过message_center收发控制/反馈消息，外部不应直接修改电机实例。
 * chassis_move按驱动/舵向分组汇总八电机完整反馈；swerve_debug保留原运动状态入口。
 */
#ifndef CHASSIS_H
#define CHASSIS_H

#include "rm_referee.h"
#include "robot_def.h"
#include "dji_motor.h"

/**
 * @brief 底盘应用调试数据，包含实际执行模式、在线状态、车体目标及八电机目标/反馈
 * @note 数组轮序统一为0左前、1右前、2右后、3左后；结构体不参与消息或CAN通信。
 *       由ChassisTask在本轮结束后复制，控制路径不读取；异步Watch仍可能看到复制中的字段。
 */
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
} SwerveDebug_s;

extern SwerveDebug_s swerve_debug; ///< 应用层统一观测入口，调试时以读取为主

/**
 * @brief 单个M3508驱动电机的观测副本，不包含可改写控制实例的指针。
 * @note measure.speed_aps为转子deg/s；actual_speed_rpm已除以6，均未乘减速比。
 */
typedef struct
{
    DJI_Motor_Measure_s measure;    ///< 完整原反馈：编码器、单/多圈角度、转速、电流和温度。
    float actual_speed_rpm;         ///< 驱动转子实测转速，rpm，按本次measure换算。
    float target_speed_rpm;         ///< 应用输出目标，rpm，已做轮速限幅与舵角补偿。
    uint32_t last_rx_ms;            ///< 最近有效反馈的HAL时刻，ms；失联后保留便于诊断。
    uint8_t feedback_seen;          ///< 至少收到过一次有效反馈；首帧前不能视为在线。
    uint8_t online;                 ///< 在本次采集时，反馈已收到且未超过电机超时阈值。
    uint8_t can_bus;                ///< 本板CAN端口编号，1或2。
    uint16_t motor_id;              ///< 电调ID；区别于CAN反馈报文ID。
    Motor_Working_Type_e stop_flag; ///< 应用启停状态；最终电流还受MotorTask/功率模块保护。
} ChassisDriveMotorDebug_s;

/**
 * @brief 单个GM6020舵向电机的观测副本，原反馈与校零后的控制角分开保存。
 * @note measure.total_angle为电机原多圈角；actual_angle_deg已减机械零位并修正方向。
 */
typedef struct
{
    DJI_Motor_Measure_s measure;    ///< 完整原反馈，电流数值沿用DJI协议，不直接解释为安培。
    float actual_angle_deg;         ///< 校零并修正方向的多圈实际舵角，deg。
    float target_angle_deg;         ///< 应用输出的多圈舵角目标，deg。
    uint16_t zero_encoder;          ///< 当前轮机械朝前时的编码器零位，0～8191。
    uint32_t last_rx_ms;            ///< 最近有效反馈的HAL时刻，ms。
    uint8_t feedback_seen;          ///< 是否已收到首帧有效反馈。
    uint8_t online;                 ///< 本次采集时反馈是否仍有效。
    uint8_t can_bus;                ///< 本板CAN端口编号，1或2。
    uint16_t motor_id;              ///< 电调ID。
    Motor_Working_Type_e stop_flag; ///< 应用启停状态，不代表已实测电机电流为零。
} ChassisSteeringMotorDebug_s;

/**
 * @brief 兼顾旧chassis_move使用习惯的统一Watch入口，只保存观测副本。
 * @note drive_motors和steering_motors均按左前、右前、右后、左后排列。
 *       由ChassisTask每轮结束时刷新；控制、PID、功率模块不读取本结构。
 *       不是旧控制结构的二进制兼容替代，也不作为消息/CAN载荷发送。
 *       在线标志为0时measure保留末帧值；异步Watch不保证整结构原子一致。
 */
typedef struct
{
    SwerveDebug_s state;                            ///< 底盘模式、姿态、坐标变换及八电机运动状态。
    Chassis_Ctrl_Cmd_s command;                     ///< 底盘应用本轮读取的命令，结合state.command_online解释。
    ChassisDriveMotorDebug_s drive_motors[4];       ///< 四个驱动电机完整反馈、目标与通信状态。
    ChassisSteeringMotorDebug_s steering_motors[4]; ///< 四个舵向电机完整反馈、目标、零位与通信状态。
    uint32_t updated_at_ms;                         ///< 本次电机反馈采集的HAL时刻，ms。
} ChassisMoveDebug_s;

extern ChassisMoveDebug_s chassis_move; ///< Orbit只需添加chassis_move，再展开两个电机数组。

/**
 * @brief 初始化四个驱动电机、四个舵向电机、底盘INS、裁判系统和超级电容
 * @attention 仅由RobotInit在BSP初始化后、创建任务前调用一次。
 *            初始化期间使用原BMI088零偏校准流程，底盘应保持静止。
 * @note 同时注册chassis_cmd订阅者和chassis_feed发布者，电机初始处于停止状态。
 */
void ChassisInit(void);

/**
 * @brief 获取本底盘应用初始化的裁判系统数据指针
 * @return referee_info_t* 裁判系统数据实例；ChassisInit执行前为NULL
 * @note 返回的是原模块持有的共享数据，字段可能由串口接收回调更新，调用方不释放该指针。
 */
referee_info_t *get_referr_data_pr(void);

/**
 * @brief 执行一轮底盘反馈采集、保护判定、模式处理和四舵轮目标解算
 * @attention 在ChassisInit之后由RobotTask周期调用，并在同一轮RobotCMDTask之后执行。
 * @note 原机器人任务每轮末尾osDelay(5)，实际间隔包含计算和调度时间。
 *       本函数只设置电机参考值，闭环及CAN报文发送由独立MotorControlTask完成。
 */
void ChassisTask(void);

#endif

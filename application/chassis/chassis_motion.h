/**
 * @file chassis_motion.h
 * @brief 底盘模式计算：旋转参考爬升、转速修正和速度坐标变换。
 * @note 不访问CAN、电机或INS服务，测量和实际周期由调用方提供。
 *       复用原PID，其初始化会读取DWT计数；该计数不用于运动参考爬升。
 */
#ifndef CHASSIS_MOTION_H
#define CHASSIS_MOTION_H

#include "controller.h"
#include "robot_def.h"

/** @brief 运动处理参数；由底盘应用从本车配置构造，初始化后只读。 */
typedef struct
{
    float spin_acceleration_radps2;  ///< 小陀螺角加速度上限，rad/s²。
    float spin_correction_gain;      ///< 角速度反馈修正比例。
    float spin_correction_radps;     ///< 反馈修正上限，rad/s。
    float maximum_yaw_rate_radps;    ///< 最终旋转目标上限，rad/s。
    float follow_gain_radps_per_deg; ///< 跟随模式的相对角比例系数。
} ChassisMotionConfig_s;

/**
 * @brief 一套独立运动状态及本轮输出，不作为板间报文使用。
 * @note 参考方向只在旋转入口捕获；此状态与Ozone快照独立。
 */
typedef struct
{
    PIDInstance spin_rate_controller; ///< 原PID实现，参数由应用传入。
    float spin_reference_yaw_deg;     ///< 进入小陀螺时的底盘航向，deg。
    float spin_rate_reference_radps;  ///< 限加速度后的角速度参考，rad/s。
    float relative_yaw_deg;           ///< 本轮坐标变换的相对航向，deg。
    float forward_velocity_mmps;      ///< 车体系X向前速度目标，mm/s。
    float right_velocity_mmps;        ///< 车体系Y向右速度目标，mm/s。
    float yaw_rate_radps;             ///< 最终俯视逆时针旋转目标，rad/s。
} ChassisMotionState_s;

/**
 * @brief 清空输出、复位转速修正器并记录当前参考航向。
 * @param state 非空状态，首次使用前及无力停机时调用。
 * @param config 有限且有效的非空参数。
 * @param yaw_deg 当前底盘航向，deg。
 */
void ChassisMotionReset(ChassisMotionState_s *state, const ChassisMotionConfig_s *config, float yaw_deg);

/**
 * @brief 由已验证的底盘命令和测量计算当前车体系速度目标。
 * @param state 非空且已初始化的状态，只属于当前底盘实例。
 * @param config 非空运动参数。
 * @param command 非空且已通过数值、模式及执行联锁检查的目标。
 * @param previous_mode 上轮实际执行模式，用于识别小陀螺入口。
 * @param yaw_deg 当前底盘航向，deg。
 * @param gyro_z_radps 当前Z轴角速度，rad/s，俯视逆时针为正。
 * @param elapsed_ms 本任务实际周期，ms，调用前已检查超时。
 * @note 不判断电机/INS在线，不发布消息、不设置电机参考。
 */
void ChassisMotionUpdate(ChassisMotionState_s *state, const ChassisMotionConfig_s *config,
                         const Chassis_Ctrl_Cmd_s *command, chassis_mode_e previous_mode, float yaw_deg,
                         float gyro_z_radps, uint32_t elapsed_ms);

#endif

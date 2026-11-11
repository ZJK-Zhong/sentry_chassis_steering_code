/**
 * @file robot_cmd.h
 * @brief 底盘控制源适配应用的初始化和周期执行接口
 */
#ifndef ROBOT_CMD_H
#define ROBOT_CMD_H

/**
 * @brief 初始化所选控制源、原USB遥测以及底盘控制/反馈通信接口
 * @attention 由RobotInit在BSP初始化后、创建实时任务前调用一次。
 * @note SWERVE_COMMAND_SOURCE_LOCAL_RC下接收机位于USART3，启动后需右下解锁。
 *       默认SWERVE_COMMAND_SOURCE_GIMBAL_CAN使用底盘CAN1的0x312/0x311。
 */
void RobotCMDInit(void);

/**
 * @brief 读取所选控制源，将底盘目标发布到chassis_cmd并按需回传底盘反馈
 * @attention 在RobotCMDInit之后，由RobotTask每轮先于ChassisTask调用。
 * @note 本地遥控失效时发布无力；CAN模式无有效首帧或通信超过约50ms时发布无力。
 *       两种来源都发布相同的Chassis_Ctrl_Cmd_s，平移单位mm/s、角速度单位rad/s。
 */
void RobotCMDTask(void);

#endif

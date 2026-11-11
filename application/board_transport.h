/**
 * @file board_transport.h
 * @brief 双板命令与反馈的通信适配接口；CMD无需访问CANComm实例
 * @note 由RobotCMDTask调用，不创建新任务。线上布局、ID、周期及超时保持board_link.h约定。
 */
#ifndef BOARD_TRANSPORT_H
#define BOARD_TRANSPORT_H
#include "robot_def.h"

/** @brief 板间接收观测副本；不作为解锁或运动判据，控制应使用读取接口的返回值。 */
typedef struct
{
    uint8_t rx_online;    // 最近一次读取时，完整消息是否在有效期内。
    uint32_t last_rx_ms;  // 完整CRC正确消息的实际接收时间，ms；首帧前为0。
} BoardTransportDebug_s;
extern BoardTransportDebug_s board_link_debug;

/** @brief 初始化本板通信端点；在创建任务前调用一次，角色与端口来自board_config.h。 */
void BoardTransportInit(void);

#ifdef CHASSIS_BOARD
/**
 * @brief 读取50ms内的完整底盘命令；不刷新接收时间
 * @param output 非NULL输出，失败时清零；字段合法性仍由底盘CMD校验。
 * @return 1:收到新鲜完整消息；0:首帧未到、CRC错误或超时。
 */
uint8_t BoardTransportReadCommand(Chassis_Ctrl_Cmd_s *output);
/** @brief 提交非NULL底盘反馈；内部按20ms周期发送，无力时也保持反馈。 */
void BoardTransportSendFeedback(const Chassis_Upload_Data_s *feedback);
#endif

#ifdef GIMBAL_BOARD
/**
 * @brief 读取100ms内且数值有效的底盘反馈；不刷新接收时间
 * @param output 非NULL输出，失效时清零。
 * @return 1:反馈有效；0:首帧未到、CRC错误、超时或字段非法。
 */
uint8_t BoardTransportReadFeedback(Chassis_Upload_Data_s *output);
/**
 * @brief 提交非NULL底盘命令；常态10ms发送一次，模式切换或保护停机可立即发送
 * @param command 本轮控制目标，只读，接口不保存调用方指针。
 * @param immediate 非0时绕过本次周期节流，不绕过CANComm打包。
 */
void BoardTransportSendCommand(const Chassis_Ctrl_Cmd_s *command, uint8_t immediate);
#endif
#endif

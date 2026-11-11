/**
 * @file board_link.h
 * @brief 双板CANComm协议常量，两工程必须使用相同的消息定义与本文件
 * @note 速度mm/s、角速度rad/s、角度deg。CANComm负责分包和CRC8，线上模式用固定宽度整数。
 */
#ifndef BOARD_LINK_H
#define BOARD_LINK_H
#include "robot_def.h"
#define BOARD_CHASSIS_TX_ID 0x311u
#define BOARD_GIMBAL_TX_ID 0x312u
#define BOARD_COMMAND_TIMEOUT_MS 50u
#define BOARD_COMMAND_PERIOD_MS 10u
#define BOARD_FEEDBACK_TIMEOUT_MS 100u
#define BOARD_FEEDBACK_PERIOD_MS 20u
#define BOARD_MAX_VX_MMPS 2000.0f
#define BOARD_MAX_VY_MMPS 1500.0f
#define BOARD_MAX_WZ_RADPS 3.14159265358979323846f
_Static_assert(sizeof(float) == 4, "Board protocol requires 32-bit floats");
_Static_assert(sizeof(Chassis_Ctrl_Cmd_s) == 26, "Chassis command wire layout changed");
_Static_assert(sizeof(Chassis_Upload_Data_s) == 28, "Chassis feedback wire layout changed");
#endif

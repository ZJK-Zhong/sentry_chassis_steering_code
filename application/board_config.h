/**
 * @file board_config.h
 * @brief 双板底盘角色与板间CAN接口，按旧舵轮实际收发代码配置
 * @note CAN1连接四个驱动轮和云台CAN1；CAN2仅连接舵向轮及电容。
 */
#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H
#define CHASSIS_BOARD
#define BOARD_LINK_CAN_HANDLE (&hcan1)
#define CHASSIS_DRIVE_CAN_HANDLE (&hcan1)
#define CHASSIS_STEERING_CAN_HANDLE (&hcan2)
#define CHASSIS_CAP_CAN_HANDLE (&hcan2)
#define CHASSIS_CAP_TX_ID 0x061u
#define CHASSIS_CAP_RX_ID 0x052u
#endif

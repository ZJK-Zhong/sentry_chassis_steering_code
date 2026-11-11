/**
 * @file power_control.h
 * @author neozng
 * @brief DJI驱动电机速度闭环、功率模型和功率限制接口
 * @version 0.2
 * @date 2022-11-01
 *
 * @todo  1. 给不同的电机设置不同的低通滤波器惯性系数而不是统一使用宏
          2. 为M2006和M3508增加开环的零位校准函数,并在初始化时调用(根据用户配置决定是否调用)

 * @copyright Copyright (c) 2022 HNU YueLu EC all rights reserved
 *
 */

#ifndef POWER_CONTROL_H
#define POWER_CONTROL_H

#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "stdint.h"
#include "daemon.h"
#include "dji_motor.h"


/**
 * @brief 注册由原功率控制模块管理的DJI驱动电机
 * @param config 电机CAN、型号及控制器配置，本底盘使用CAN1的四个M3508
 * @return DJIMotorInstance* 已注册的电机实例，可使用DJIMotorSetRef/Stop/Enable更新状态
 * @attention 在创建任务前调用；同一电机不能同时重复注册到DJIMotorInit。
 * @note 本模块驱动速度参考采用转子rpm，measure.speed_aps仍保留原反馈的转子度/秒。
 */
DJIMotorInstance *PowerControlInit(Motor_Init_Config_s *config);

/**
 * @brief 为已注册的驱动电机计算速度闭环和功率分配，并按CAN组发送控制报文
 * @attention 由原MotorControlTask周期调用；反馈和目标均需在20ms内保持更新。
 * @note 停止或超时的电机不参与本轮闭环并发送零控制量。
 *       原功率模型仅统计注册到本模块的驱动电机，不包含独立DJIMotor管理的舵向功耗。
 */
void PowerControl(void);

/**
 * @brief 设置电机功率限制
 *
 * @param power_limit 基础功率上限，单位W，由底盘应用提供裁判值或缺省值
 * @note 只更新模块变量，最终功率分配还受原超级电容策略影响，不立即发送报文。
 */
void SetPowerLimit(float power_limit);
#endif // POWER_CONTROL_H

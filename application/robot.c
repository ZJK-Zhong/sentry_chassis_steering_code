/**
 * @file robot.c
 * @brief 组织双板底盘从控的初始化顺序和应用调用顺序
 *
 * 初始化：BSPInit -> RobotCMDInit -> ChassisInit -> OSTaskInit。
 * 机器人任务只调用CMD和底盘应用；电机闭环、INS解算、守护检测和UI由原基础任务执行。
 */
#include "bsp_init.h"
#include "robot.h"
#include "robot_def.h"
#include "robot_task.h"
#include "referee_task.h"
#include "chassis.h"
#include "robot_cmd.h"

// 提醒修改硬件参数；本工程只作为底盘板使用。
#ifndef ROBOT_DEF_PARAM_WARNING
#define ROBOT_DEF_PARAM_WARNING
#pragma message "check if you have configured the parameters in robot_def.h, IF NOT, please refer to the comments AND DO IT, otherwise the robot will have FATAL ERRORS!!!"
#endif

void RobotInit(void)
{
    // 初始化沿用全向轮框架；中断关闭期间只能使用 DWT 延时。
    __disable_irq();
    BSPInit();
    RobotCMDInit(); // 板间CAN输入适配和消息中心发布端。
    ChassisInit();  // 八电机、底盘 INS、裁判系统及超级电容。
    OSTaskInit();   // 保留原框架的 INS、电机、守护、机器人和 UI 任务。
    __enable_irq();
}

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t RobotTask_high_water;
#endif

void RobotTask(void)
{
    // 同一轮先发布统一底盘目标，再由底盘取出并解算，避免额外等待一个任务周期。
    RobotCMDTask();
    ChassisTask();

#if INCLUDE_uxTaskGetStackHighWaterMark
    RobotTask_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
}

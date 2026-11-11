/**
 * @file robot.h
 * @brief 双板底盘从控的系统初始化和机器人周期任务入口
 */
#ifndef ROBOT_H
#define ROBOT_H

/**
 * @brief 初始化BSP、底盘CMD与底盘应用，并创建原框架的基础任务
 * @attention 由main在外设初始化完成后、启动RTOS调度前调用一次。
 * @note 本工程只初始化底盘相关应用；INS、电机、守护和UI保留独立任务。
 */
void RobotInit();

/**
 * @brief 执行一轮底盘指令生成和底盘控制，顺序为RobotCMDTask -> ChassisTask
 * @attention 由StartROBOTTASK循环调用，不能与另一个任务并发调用此入口。
 * @note 本函数不包含延时；周期由robot_task.h中的任务循环和RTOS调度共同决定。
 */
void RobotTask();

#endif

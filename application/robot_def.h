/**
 * @file robot_def.h
 * @author NeoZeng neozng1@hnu.edu.cn
 * @author Even
 * @details 两端共享原框架模式与消息类型。硬件映射在各自board_config.h，舵轮参数在swerve_config.h。
 *          底盘不注册云台/发射电机；云台不注册底盘电机。线上字段使用固定宽度类型。
 * @version 0.1
 * @date 2022-12-02
 *
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 *
 */
#pragma once // 可以用#pragma once代替#ifndef ROBOT_DEF_H(header guard)
#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include "ins_task.h"
#include "master_process.h"
#include "stdint.h"

/* 板间消息在两端保持完全相同，仅角色和CAN接口由各工程board_config.h决定。 */
#include "board_config.h"

#define VISION_USE_VCP  // 使用虚拟串口发送视觉数据
// #define VISION_USE_UART // 使用串口发送视觉数据

/* 机器人重要参数定义,注意根据不同机器人进行修改,浮点数需要以.0或f结尾,无符号以u结尾 */
// 机器人底盘修改的参数,单位为mm(毫米)
#define WHEEL_BASE 305.47013f       // 216mm回转半径、正方形布局
#define TRACK_WIDTH 305.47013f
#define RADIUS_WHEEL (380.0f / (2.0f * 3.14159265358979323846f))
#define REDUCTION_RATIO_WHEEL (1.0f / 0.0526f)
#define CHASSIS_CENTER 216.0f

#pragma pack(1) // 压缩结构体,取消字节对齐,下面的数据都可能被传输
/* -------------------------基本控制模式和数据类型定义-------------------------*/
/**
 * @brief 这些枚举类型和结构体会作为CMD控制数据和各应用的反馈数据的一部分
 *
 */
// 机器人状态
typedef enum
{
    ROBOT_STOP = 0,
    ROBOT_READY,
} Robot_Status_e;

// 应用状态
typedef enum
{
    APP_OFFLINE = 0,
    APP_ONLINE,
    APP_ERROR,
} App_Status_e;

/**
 * @brief 底盘执行模式，原模式编码保留，本地遥控新增CHASSIS_REMOTE
 * @note 本地右拨杆只发布ZERO_FORCE、REMOTE和ROTATE。
 *       双板按rm-sentry左右组合分发整车模式，定义见云台robot_cmd.h。
 */
typedef enum
{
    CHASSIS_ZERO_FORCE = 0,    // 电流零输入
    CHASSIS_ROTATE,            // 小陀螺模式
    CHASSIS_NO_FOLLOW,         // 不跟随，允许全向平移
    CHASSIS_FOLLOW_GIMBAL_YAW, // 跟随模式，底盘叠加角度环控制
    CHASSIS_REMOTE,           // 本地遥控，车体系平移和旋转
} chassis_mode_e;

// 以下模式由云台/发射应用与底盘裁判UI共用，不表示底盘板拥有对应电机。
// 云台执行/显示模式
typedef enum
{
    GIMBAL_ZERO_FORCE = 0, // 两轴零电流。
    GIMBAL_FREE_MODE,      // 保留模式编码，本遥控入口不生成。
    GIMBAL_GYRO_MODE,      // INS反馈的角度/角速度闭环。
} gimbal_mode_e;

// 发射执行/显示模式
typedef enum
{
    SHOOT_OFF = 0,
    SHOOT_ON,
} shoot_mode_e;
typedef enum
{
    FRICTION_OFF = 0, // 摩擦轮关闭
    FRICTION_ON,      // 摩擦轮开启
} friction_mode_e;

typedef enum
{
    LID_OPEN = 0, // 弹舱盖打开
    LID_CLOSE,    // 弹舱盖关闭
} lid_mode_e;

/** @brief 原rm-sentry发射机构拨盘模式；无力状态必须使用LOAD_STOP。 */
typedef enum
{
    LOAD_STOP = 0,
    LOAD_REVERSE,
    LOAD_1_BULLET,
    LOAD_3_BULLET,
    LOAD_BURSTFIRE,
} loader_mode_e;

/** @brief 云台角度目标，单位deg；只在云台板通过消息中心传递。 */
typedef struct
{
    float yaw;
    float pitch;
    float chassis_rotate_wz; // rad/s，保留原框架字段。
    gimbal_mode_e gimbal_mode;
} Gimbal_Ctrl_Cmd_s;

/** @brief 发射机构目标，只在云台板使用，不经CAN发送电机命令。 */
typedef struct
{
    shoot_mode_e shoot_mode;
    loader_mode_e load_mode;
    lid_mode_e lid_mode;
    friction_mode_e friction_mode;
    Bullet_Speed_e bullet_speed;
    uint8_t rest_heat;
    float shoot_rate; // 发/秒，原框架保留字段。
} Shoot_Ctrl_Cmd_s;

/**
 * @brief 原裁判UI使用的底盘功率显示数据，单位W
 * @note 该结构只供显示，不是PowerControl的闭环输入。
 */
typedef struct
{ // 功率控制
    float chassis_power_mx;
} Chassis_Power_Data_s;

/* CMD 通过原消息中心发布底盘目标，底盘任务订阅。 */
/**
 * @brief CMD发布到chassis_cmd话题的底盘目标
 * @note vx/vy/wz分别为mm/s、mm/s和rad/s；X向前、Y向右，俯视逆时针为正。
 *       mode只选择平移坐标转换方式，chassis_mode选择底盘实际执行方式，两者含义不同。
 *       本地手动使用mode=0、小陀螺使用mode=3；双板遥控使用mode=2云台系，保持pack(1)。
 */
typedef struct
{
    // 控制部分
    float vx;           // mm/s, 前进为正
    float vy;           // mm/s, 向右为正
    float wz;           // rad/s, 俯视逆时针为正
    float offset_angle; // 底盘坐标转换/跟随用角度，单位度；本地遥控保持为 0
    uint8_t chassis_mode; // chassis_mode_e编码，通信字段固定1字节，避免枚举ABI差异。
    int32_t chassis_speed_buff;
    // UI部分
    //  ...

    uint8_t mode;       // 0:车体系, 1:导航, 2:云台系, 3:进入小陀螺时的参考系
    float chassis_spin_speed; // 小陀螺目标角速度，rad/s
} Chassis_Ctrl_Cmd_s;

/**
 * @brief 底盘发布到chassis_feed话题的实际速度和原框架共享反馈字段
 * @note real_vx/real_vy由四轮实际轮速及舵角反算，real_wz为运动学估计，不直接等于陀螺仪读数。
 *       热量与血量来自底盘裁判USART6，仅在referee_online有效时可用于发射判定。
 */
typedef struct
{
    float real_vx; // 车体系 X 速度，mm/s
    float real_vy; // 车体系 Y 向右速度，mm/s
    float real_wz; // 俯视逆时针角速度，rad/s

    uint8_t rest_heat;           // 剩余枪口热量
    uint8_t bullet_speed; // Bullet_Speed_e编码，固定1字节。
    uint8_t enemy_color;  // Enemy_Color_e编码，固定1字节。

    uint8_t motors_online;   // 八电机均已收帧且未超时。
    uint8_t ins_online;      // 底盘INS快照有效。
    float yaw_deg;          // 底盘航向，deg。
    uint16_t heat_limit;    // 裁判允许热量；未收到有效裁判数据时为0。
    uint16_t current_heat;  // 当前17mm热量。
    uint8_t referee_online; // 性能和热量两种裁判报文均未超过500ms。
    uint16_t current_hp;    // 原CMD巡查逻辑使用的实际剩余血量。

} Chassis_Upload_Data_s;

/** @brief 云台应用快照；yaw电机角度保留float，避免原uint16_t截断角度。 */
typedef struct
{
    attitude_t gimbal_imu_data;
    float yaw_motor_single_round_angle; // deg，0~360。
    uint8_t online; // INS和两电机反馈均有效。
} Gimbal_Upload_Data_s;

/** @brief 发射应用反馈，用于CMD联锁和调试，不作为底盘CAN报文发送。 */
typedef struct
{
    uint8_t motors_online;
    uint8_t friction_ready;
    uint8_t heat_allowed;
    uint8_t loader_running;
} Shoot_Upload_Data_s;


#pragma pack() // 开启字节对齐,结束前面的#pragma pack(1)

#endif // !ROBOT_DEF_H

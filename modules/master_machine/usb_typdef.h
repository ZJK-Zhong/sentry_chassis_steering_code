#ifndef USB_TYPEDEF_H
#define USB_TYPEDEF_H

#include "attribute_typedef.h"
#include "remote_control.h"
#include "stdbool.h"
#include "stdint.h"

#define DEBUG_PACKAGE_NUM 10

#define DATA_DOMAIN_OFFSET 0x08

// clang-format off
#define SEND_SOF    ((uint8_t)0x5A)
#define RECEIVE_SOF ((uint8_t)0x5A)

#define DEBUG_DATA_SEND_ID        ((uint8_t)0x01)
#define IMU_DATA_SEND_ID          ((uint8_t)0x02)
#define ROBOT_STATE_INFO_DATA_SEND_ID   ((uint8_t)0x03)
#define EVENT_DATA_SEND_ID        ((uint8_t)0x04)
#define PID_DEBUG_DATA_SEND_ID    ((uint8_t)0x05)
#define ALL_ROBOT_HP_SEND_ID      ((uint8_t)0x06)
#define GAME_STATUS_SEND_ID       ((uint8_t)0x07) 
#define ROBOT_MOTION_DATA_SEND_ID ((uint8_t)0x08)
#define GROUND_ROBOT_POSITION_SEND_ID ((uint8_t)0x09)
#define RFID_STATUS_SEND_ID       ((uint8_t)0x0A)
#define ROBOT_STATUS_SEND_ID      ((uint8_t)0x0B)
#define JOINT_STATE_SEND_ID       ((uint8_t)0x0C)
#define BUFF_SEND_ID              ((uint8_t)0x0D)
#define SENTRY_INFO_SEND_ID       ((uint8_t)0x0E)

#define ROBOT_CMD_DATA_RECEIVE_ID  ((uint8_t)0x01)
#define PID_DEBUG_DATA_RECEIVE_ID  ((uint8_t)0x02)
#define VIRTUAL_RC_DATA_RECEIVE_ID ((uint8_t)0x03)
#define SENTRYCMD_ID               ((uint8_t)0x04)






#define AX_X 0  // x轴
#define AX_Y 1  // y轴
#define AX_Z 2  // z轴

#define AX_ROLL AX_X   // roll轴
#define AX_PITCH AX_Y  // pitch轴
#define AX_YAW AX_Z    // yaw轴



// clang-format off
// 可用底盘硬件类型
#define CHASSIS_NONE            0  // 无底盘
#define CHASSIS_MECANUM_WHEEL   1  // 麦克纳姆轮底盘
#define CHASSIS_OMNI_WHEEL      2  // 全向轮底盘
#define CHASSIS_STEERING_WHEEL  3  // 舵轮底盘
#define CHASSIS_BALANCE         4  // 平衡底盘

// 可用云台硬件类型
#define GIMBAL_NONE                0  // 无云台
#define GIMBAL_YAW_PITCH_DIRECT    1  // yaw-pitch电机直连云台

// 可用的发射机构硬件类型
#define SHOOT_NONE               0  // 无发射机构
#define SHOOT_FRIC_TRIGGER       1  // 摩擦轮+拨弹盘发射机构
#define SHOOT_PNEUMATIC_TRIGGER  2  // 气动+拨弹盘发射机构

// 可用机械臂硬件类型
#define MECHANICAL_ARM_NONE              0  // 无机械臂
#define MECHANICAL_ARM_PENGUIN_MINI_ARM  1  // 企鹅mini机械臂
#define MECHANICAL_ARM_ENGINEER_ARM      2  // 工程机械臂

// 控制类型（板间通信时用到）
#define CHASSIS_ONLY       0  // 只控制底盘
#define GIMBAL_ONLY        1  // 只控制云台
#define CHASSIS_AND_GIMBAL 2  // 控制底盘和云台

#define CHASSIS_TYPE CHASSIS_STEERING_WHEEL      // 选择底盘类型
#define GIMBAL_TYPE GIMBAL_YAW_PITCH_DIRECT      // 选择云台类型
#define SHOOT_TYPE SHOOT_NONE                    // 选择发射机构类型
#define CONTROL_TYPE CHASSIS_AND_GIMBAL          // 选择控制类型
#define MECHANICAL_ARM_TYPE MECHANICAL_ARM_NONE  //选择机械臂类型


// clang-format on

typedef struct  // 底盘速度向量结构体
{
    float vx;  // (m/s) x方向速度
    float vy;  // (m/s) y方向速度
    float wz;  // (rad/s) 旋转速度
} ChassisSpeedVector_t;

typedef struct __Imu
{
    float angle[3];  // rad 欧拉角数据
    float gyro[3];   // rad/s 陀螺仪数据
    float accel[3];  // m/s^2 加速度计数据
} Imu_t;

typedef struct
{
    ChassisSpeedVector_t speed_vector;
    struct
    {
        float roll;
        float pitch;
        float yaw;
        float leg_length;
    } chassis;

    struct
    {
        float pitch;
        float yaw;
    } gimbal;

    struct
    {
        bool fire;
        bool fric_on;
    } shoot;

} RobotCmdData_t;




typedef struct {
  uint8_t sentry_cmd;  // bit 0-31
} sentrycmd_t;



typedef struct
{
    uint8_t sof;  // 数据帧起始字节，固定值为 0x5A
    uint8_t len;  // 数据段长度
    uint8_t id;   // 数据段id
    uint8_t crc;  // 数据帧头的 CRC8 校验
} __packed__ FrameHeader_t;
/*-------------------- Send --------------------*/

// 串口调试数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x01
    uint32_t time_stamp;
    struct
    {
        uint8_t name[10];
        uint8_t type;
        float data;
    } __packed__ packages[DEBUG_PACKAGE_NUM];
    uint16_t checksum;
} __packed__ SendDataDebug_s;

// IMU 数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x02
    uint32_t time_stamp;
    struct
    {
        float yaw;    // rad
        float pitch;  // rad
        float roll;   // rad

        float yaw_vel;    // rad/s
        float pitch_vel;  // rad/s
        float roll_vel;   // rad/s

        // float x_accel;  // m/s^2
        // float y_accel;  // m/s^2
        // float z_accel;  // m/s^2
    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataImu_s;

// 机器人信息数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x03
    uint32_t time_stamp;
    struct
    {
        /// @brief 机器人部位类型 2 bytes
        struct
        {
            uint16_t chassis : 3;
            uint16_t gimbal : 3;
            uint16_t shoot : 3;
            uint16_t arm : 3;
            uint16_t custom_controller : 3;
            uint16_t reserve : 1;
        } __packed__ type;
        /// @brief 机器人部位状态 1 byte
        /// @note 0: 正常，1: 错误
        struct
        {
            uint8_t chassis : 1;
            uint8_t gimbal : 1;
            uint8_t shoot : 1;
            uint8_t arm : 1;
            uint8_t custom_controller : 1;
            uint8_t reserve : 3;
        } __packed__ state;
        // /// @brief 机器人裁判系统信息 7 bytes
        // struct
        // {
        //     uint8_t id;
        //     uint8_t color;  // 0-red 1-blue 2-unknown
        //     bool attacked;
        //     uint16_t hp;
        //     uint16_t heat;
        // } __packed__ referee;
    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataRobotStateInfo_s;

// 事件数据包

typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x04
    uint32_t time_stamp;

    struct
    {
        uint8_t non_overlapping_supply_zone : 1;
        uint8_t overlapping_supply_zone : 1;
        uint8_t supply_zone : 1;

        uint8_t small_energy : 1;
        uint8_t big_energy : 1;

        uint8_t central_highland : 2;
        uint8_t reserved1 : 1;
        uint8_t trapezoidal_highland : 2;

        uint8_t center_gain_zone : 2;
        uint8_t reserved2 : 4;
    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataEvent_s;

// PID调参数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x05
    uint32_t time_stamp;
    struct
    {
        float fdb;
        float ref;
        float pid_out;
    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataPidDebug_s;

// 全场机器人hp信息数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x06
    uint32_t time_stamp;
    struct
    {
        uint16_t ally_1_robot_HP;
        uint16_t ally_2_robot_HP;
        uint16_t ally_3_robot_HP;
        uint16_t ally_4_robot_HP;
        uint16_t reserved;
        uint16_t ally_7_robot_HP;
        uint16_t ally_outpost_HP;
        uint16_t ally_base_HP;
    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataAllRobotHp_s;

// 比赛信息数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x07
    uint32_t time_stamp;
    struct
    {
        uint8_t game_progress;
        uint16_t stage_remain_time;
    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataGameStatus_s;

// 地面机器人位置数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x08
    uint32_t time_stamp;
    
    struct
    {
        float hero_x;
        float hero_y;

        float engineer_x;
        float engineer_y;

        float standard_3_x;
        float standard_3_y;

        float standard_4_x;
        float standard_4_y;

        float standard_5_x;
        float standard_5_y;

        float reserved1;
        float reserved2;

    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataGroundRobotPosition_s;

// 机器人运动数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x09
    uint32_t time_stamp;
    struct
    {
        struct
        {
            float vx;
            float vy;
            float wz;
        } __packed__ speed_vector;
    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataRobotMotion_s;

// RFID状态数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x0A
    uint32_t time_stamp;

  struct
  {
    bool base_gain_point;                     // 己方基地增益点
    bool circular_highland_gain_point;        // 己方环形高地增益点
    bool enemy_circular_highland_gain_point;  // 对方环形高地增益点
    bool friendly_r3_b3_gain_point;           // 己方 R3/B3 梯形高地增益点
    bool enemy_r3_b3_gain_point;              // 对方 R3/B3 梯形高地增益点
    bool friendly_r4_b4_gain_point;           // 己方 R4/B4 梯形高地增益点
    bool enemy_r4_b4_gain_point;              // 对方 R4/B4 梯形高地增益点
    bool energy_mechanism_gain_point;         // 己方能量机关激活点
    bool friendly_fly_ramp_front_gain_point;  // 己方飞坡增益点（靠近己方一侧飞坡前）
    bool friendly_fly_ramp_back_gain_point;  // 己方飞坡增益点（靠近己方一侧飞坡后）
    bool enemy_fly_ramp_front_gain_point;  // 对方飞坡增益点（靠近对方一侧飞坡前）
    bool enemy_fly_ramp_back_gain_point;   // 对方飞坡增益点（靠近对方一侧飞坡后）
    bool friendly_outpost_gain_point;      // 己方前哨站增益点
    bool friendly_healing_point;           // 己方补血点（检测到任一均视为激活）
    bool friendly_sentry_patrol_area;      // 己方哨兵巡逻区
    bool enemy_sentry_patrol_area;         // 对方哨兵巡逻区
    bool friendly_big_resource_island;     // 己方大资源岛增益点
    bool enemy_big_resource_island;        // 对方大资源岛增益点
    bool friendly_exchange_area;           // 己方兑换区
    bool center_gain_point;  // 中心增益点 RFID 卡状态（仅 RMUL 适用），1 为已检测到
  } __attribute__((packed)) data;
    uint16_t crc;            
} __packed__ SendDataRfidStatus_s;



typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x0E
    uint32_t time_stamp;

    struct {
            uint32_t sentry_info;    // bit 0-31
            uint16_t sentry_info_2;  // bit 0-15
        } data;


    uint16_t crc;            
} __packed__ SendDataSentryInfo_s;



// 机器人状态数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x0B
    uint32_t time_stamp;

    struct
    {
        uint8_t robot_id;
        uint8_t robot_level;
        uint16_t current_up;
        uint16_t maximum_hp;
        uint16_t shooter_barrel_cooling_value;
        uint16_t shooter_barrel_heat_limit;

        uint16_t shooter_17mm_1_barrel_heat;

        float robot_pos_x;
        float robot_pos_y;
        float robot_pos_angle;

        uint8_t armor_id : 4;
        uint8_t hp_deduction_reason : 4;

        uint16_t projectile_allowance_17mm;
        uint16_t remaining_gold_coin;      
    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataRobotStatus_s;

// 云台状态数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x0C
    uint32_t time_stamp;
    struct
    {
        float pitch;
        float yaw;

    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataJointState_s;

// 机器人增益和底盘能量数据包
typedef struct 
{
    FrameHeader_t frame_header;
    uint32_t time_stamp;

    struct
    {
        uint8_t recovery_buff;
        uint8_t cooling_buff;
        uint8_t defence_buff;
        uint8_t vulnerability_buff;
        uint16_t attack_buff;
        uint8_t remaining_energy;
    } __packed__ data;

    uint16_t crc;
} __packed__ SendDataBuff_s;
/*-------------------- Receive --------------------*/
typedef struct RobotCmdData
{
    FrameHeader_t frame_header;  // 数据段id = 0x01
    uint32_t time_stamp;
    struct
    {
        struct
        {
            float vx;
            float vy;
            float wz;
        } __packed__ speed_vector;
        struct
        {
            float roll;
            float pitch;
            float yaw;
            float leg_lenth;
        } __packed__ chassis;
        struct
        {
            float pitch;
            float yaw;
        } __packed__ gimbal;
        struct
        {
            uint8_t fire;
            uint8_t fric_on;
        } __packed__ shoot;
        struct
        {
            bool tracking;
        } __packed__ tracking;
    } __packed__ data;
    uint16_t checksum;
} __packed__ ReceiveDataRobotCmd_s;

// PID调参数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x02
    uint32_t time_stamp;
    struct
    {
        float kp;
        float ki;
        float kd;
        float max_out;
        float max_iout;
    } __packed__ data;
    uint16_t crc;
} __packed__ ReceiveDataPidDebug_s;

// 虚拟遥控器数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x03
    uint32_t time_stamp;
    RC_ctrl_t data;
    uint16_t crc;
} __packed__ ReceiveDataVirtualRc_s;
#endif  // USB_TYPEDEF_H

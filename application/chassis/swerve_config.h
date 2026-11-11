/**
 * @file swerve_config.h
 * @brief 舵轮硬件、运动限制、小陀螺参数和超时阈值
 * @attention 编码器零位必须与轮序对应；修改方向、减速比或PID后应先架空验证。
 * @note 文件中的长度和速度单位按每个参数标明；本文件不执行运行时控制。
 */
#ifndef SWERVE_CONFIG_H
#define SWERVE_CONFIG_H

/* 硬件顺序：左前、右前、右后、左后；M3508 在 CAN1，GM6020 在 CAN2，均为 ID1~4。 */
#define SWERVE_ZERO_ENCODERS {4976, 701, 7532, 688} // 各舵向轮朝前时的编码器零位。
#define CHASSIS_CENTER_TO_WHEEL 0.216f              // 底盘中心到轮心距离，m，采用正方形布局。
#define CHASSIS_MOTOT_PER 0.38f                     // 驱动轮周长，m；保留原解算的宏名。
#define SWERVE_DRIVE_REDUCTION 0.0526f              // 输出轴转速 / 电机转子转速。
/* 电机控制参数：驱动闭环使用转子rpm，舵向闭环使用校零后的多圈角度。 */
#define SWERVE_DRIVE_KP 5.0f             // 驱动 rpm 速度环比例系数。
#define SWERVE_STEERING_KP 360.0f        // 舵向角度环比例系数，反馈单位度。
#define SWERVE_DRIVE_MAX_OUT 16000.0f    // M3508 协议输出上限。
#define SWERVE_STEERING_MAX_OUT 30000.0f // GM6020 协议输出上限。
/* 运动上限：应用层使用mm/s和rad/s，轮组解算使用m/s和度/秒。 */
#define SWERVE_MAX_WHEEL_MPS 4.0f                   // 合成轮速上限，超出时四轮同比例缩放。
#define SWERVE_MAX_VX_MMPS 2000.0f                  // 前后速度上限，mm/s。
#define SWERVE_MAX_VY_MMPS 1500.0f                  // 左右速度上限，mm/s。
#define SWERVE_MAX_WZ_RADPS 3.14159265358979323846f // 旋转目标上限，rad/s。
/* 小陀螺：角速度参考先爬升，再叠加由陀螺仪角速度反馈得到的限幅修正。 */
#define SWERVE_SPIN_DPS 120.0f           // 小陀螺稳定目标，度/秒。
#define SWERVE_SPIN_ACCEL_DPS2 60.0f     // 小陀螺角加速度限制，度/秒平方。
#define SWERVE_SPIN_RATE_KP 0.3f         // 原 PID 框架内的角速度修正比例。
#define SWERVE_SPIN_CORRECTION_DPS 30.0f // 角速度反馈修正量上限，度/秒。
#define SWERVE_INS_YAW_SIGN 1.0f         // INS 到底盘逆时针正方向的符号。
/* 超时与输入保护；电机模块的独立20ms看门狗仍在MotorControlTask中执行。 */
#define SWERVE_MOTOR_TIMEOUT_MS 20u   // 任一电机反馈过期则停全部八电机。
#define SWERVE_COMMAND_TIMEOUT_MS 20u // 底盘任务间隔异常时停止输出。
#define SWERVE_RC_DEADBAND 10         // 遥控摇杆中位死区。
#define SWERVE_FALLBACK_POWER_W 45.0f // 无裁判功率上限时的默认限制，W。

/* 本双板工程默认CAN输入；本地DBUS分支保留作软件回归，不能用拨杆切换编译角色。 */
#define SWERVE_COMMAND_SOURCE_LOCAL_RC 0
#define SWERVE_COMMAND_SOURCE_GIMBAL_CAN 1
#ifndef SWERVE_COMMAND_SOURCE
#define SWERVE_COMMAND_SOURCE SWERVE_COMMAND_SOURCE_GIMBAL_CAN
#endif

/* 双板CAN：底盘CAN1收0x312控制、发0x311反馈；看门狗按10ms daemon周期计数。 */
#define SWERVE_CHASSIS_CAN_TX_ID 0x311u
#define SWERVE_CHASSIS_CAN_RX_ID 0x312u
#define SWERVE_CHASSIS_CAN_DAEMON_TICKS 5u        // 约50ms无有效控制帧时判通信离线。
#define SWERVE_CHASSIS_CAN_FEEDBACK_PERIOD_MS 20u // 反馈20ms发送一次，降低共享总线负载。

#if (SWERVE_COMMAND_SOURCE != SWERVE_COMMAND_SOURCE_LOCAL_RC) &&                                                       \
    (SWERVE_COMMAND_SOURCE != SWERVE_COMMAND_SOURCE_GIMBAL_CAN)
#error Invalid SWERVE_COMMAND_SOURCE value.
#endif

#endif

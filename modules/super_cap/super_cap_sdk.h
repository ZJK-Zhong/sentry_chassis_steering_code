#ifndef SUPER_CAP_SDK_H
#define SUPER_CAP_SDK_H

#include <stdint.h>
#include <stdbool.h>

// 发送给超级电容的控制数据 (Chassis -> Supercap)

#define SUPERCAP_RECEIVE_CAN_ID 0x052
#define SUPERCAP_SEND_CAN_ID 0x061

typedef struct {
    // 开关控制
    bool enable_dcdc;                // 1: 开启DCDC输出, 0: 关闭
    bool system_restart;             // 1: 重启超级电容系统
    bool clear_error;                // 1: 清除错误
    bool enable_active_charging_limit; // 1: 启用主动充电限制策略

    // 裁判系统数据转发
    uint16_t referee_power_limit;       // 裁判限制功率 (W)
    uint16_t referee_energy_buffer;     // 裁判剩余缓冲能量 (J)
    
    // 策略参数
    float active_charging_limit_ratio; // 主动充电限制比例 (0-1.0)
} SuperCap_Control_t;

typedef enum {
    SUPERCAP_NO_ERROR = 0,               // 无错误
    SUPERCAP_ERROR_RECOVER_AUTO = 1,     // 错误，过时自动恢复
    SUPERCAP_ERROR_RECOVER_MANUAL = 2,   // 错误，按板载按钮恢复
    SUPERCAP_ERROR_UNRECOVERABLE = 3,    // 错误，不可恢复
    // WARNING不会被发送
} SuperCapErrorLevel_t;

typedef enum {
    SUPERCAP_REFEREE_POWER = 0,
    SUPERCAP_CAPARR_VOLTAGE_MAX = 1,
    SUPERCAP_CAPARR_VOLTAGE_NORMAL = 2,
    SUPERCAP_IB_POSITIVE_OR_IB_NEGATIVE = 3,
} SuperCapLimitFactor_t;

// 从超级电容接收的反馈数据 (Supercap -> Chassis)
typedef struct {
    // 状态标志
    bool dcdc_enabled;               // 当前DCDC是否开启
    bool wpt_status; // TODO
    SuperCapLimitFactor_t limit_factor;
    SuperCapErrorLevel_t error_flag;

    // 物理量 (已转换为实际单位)
    float chassis_power_w;              // 底盘实际功率 (W)
    float referee_power_w;              // 裁判系统功率 (W)
    uint16_t chassis_power_limit_w;     // 当前底盘最大可用功率 (W)，这主要是基于裁判系统电流限制算出来的，如果不放心可以再加个缩放系数0.9什么的
    float cap_energy_percent;           // 电容剩余能量百分比，注意这个是可以超过1的（以电容冲到28.8V为1计算，最后一点为保护和能量回收预留）
} SuperCap_Feedback_t;



void SuperCap_InitDefaultControl(SuperCap_Control_t* control);

void SuperCap_PackTxData(const SuperCap_Control_t* control, uint8_t* tx_buffer);

void SuperCap_ParseRxData(const uint8_t* rx_buffer, SuperCap_Feedback_t* feedback);

#endif // SUPERCAP_SDK_H
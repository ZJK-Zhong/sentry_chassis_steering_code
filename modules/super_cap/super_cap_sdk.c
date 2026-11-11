#include "super_cap_sdk.h"

void SuperCap_InitDefaultControl(SuperCap_Control_t *control) {
    control->enable_dcdc = true;
    control->system_restart = false;
    control->clear_error = false;
    control->enable_active_charging_limit = false;
    control->referee_power_limit = 37;
    control->referee_energy_buffer = 57;
    control->active_charging_limit_ratio = 0;
}

#define TX_FLAH_ENABLE_DCDC (1 << 0)
#define TX_FLAH_SYSTEM_RESTART (1 << 1)
// bits 2-4 reserved
#define TX_FLAH_CLEAR_ERROR (1 << 5)
#define TX_FLAH_ENABLE_LIMIT (1 << 6)
#define TX_FLAH_USE_NEW_MSG (1 << 7)

void SuperCap_PackTxData(const SuperCap_Control_t *control,
                         uint8_t *tx_buffer) {
    if (!control || !tx_buffer) return;

    // Byte 0: 标志位
    uint8_t byte0 = 0;
    if (control->enable_dcdc) byte0 |= TX_FLAH_ENABLE_DCDC;
    if (control->system_restart) byte0 |= TX_FLAH_SYSTEM_RESTART;
    if (control->clear_error) byte0 |= TX_FLAH_CLEAR_ERROR;
    if (control->enable_active_charging_limit) byte0 |= TX_FLAH_ENABLE_LIMIT;

    // 强制设置使用新协议标志
    byte0 |= TX_FLAH_USE_NEW_MSG;

    tx_buffer[0] = byte0;

    // Byte 1-2: 裁判系统功率限制
    tx_buffer[1] = (uint8_t)(control->referee_power_limit & 0xFF);
    tx_buffer[2] = (uint8_t)((control->referee_power_limit >> 8) & 0xFF);

    // Byte 3-4: 裁判系统剩余缓冲能量
    tx_buffer[3] = (uint8_t)(control->referee_energy_buffer & 0xFF);
    tx_buffer[4] = (uint8_t)((control->referee_energy_buffer >> 8) & 0xFF);

    // Byte 5: 主动充电限制比例
    tx_buffer[5] = (uint8_t)(control->active_charging_limit_ratio * 255.0f);

    // Byte 6-7: Reserved
    tx_buffer[6] = 0;
    tx_buffer[7] = 0;
}


#define RX_FLAG_DCDC_ENABLED (1 << 7)
#define RX_FLAG_NEW_MSG (1 << 6)
// #define RX_FLAG_WPT_STATUS (0b11 << 4)
// #define RX_FLAG_LIMIT_FACTOR (0b11 << 2)
// #define RX_FLAG_ERROR_LEVEL (0b11 << 0)
#define RX_FLAG_WPT_STATUS (0x3 << 4)
#define RX_FLAG_LIMIT_FACTOR (0x3 << 2)
#define RX_FLAG_ERROR_LEVEL (0x3 << 0)



void SuperCap_ParseRxData(const uint8_t *rx_buffer,
                          SuperCap_Feedback_t *feedback) {
    if (!rx_buffer || !feedback) return;

    // Byte 0 flag
    uint8_t status = rx_buffer[0];
    feedback->dcdc_enabled = (status & RX_FLAG_DCDC_ENABLED) != 0;
    feedback->wpt_status = (status & RX_FLAG_WPT_STATUS) >> 4;
    feedback->limit_factor = (status & RX_FLAG_LIMIT_FACTOR) >> 2;
    feedback->error_flag = (status & RX_FLAG_ERROR_LEVEL);

    // Byte 1-2: 底盘功率
    uint16_t raw_chassis_power =
        (uint16_t)rx_buffer[1] | ((uint16_t)rx_buffer[2] << 8);
    feedback->chassis_power_w = ((float)raw_chassis_power - 16384) / 64.0f;

    // Byte 3-4: 裁判系统功率
    uint16_t raw_referee_power =
        (uint16_t)rx_buffer[3] | ((uint16_t)rx_buffer[4] << 8);
    feedback->referee_power_w = ((float)raw_referee_power - 16384) / 64.0f;

    // Byte 5-6: 底盘功率限制
    feedback->chassis_power_limit_w =
        (uint16_t)rx_buffer[5] | ((uint16_t)rx_buffer[6] << 8);

    // Byte 7: 电容能量
    uint8_t raw_energy = rx_buffer[7];
    feedback->cap_energy_percent = (float)raw_energy / 250.0f;
}
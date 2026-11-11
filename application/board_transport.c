/**
 * @file board_transport.c
 * @brief 将整车消息适配到原CANComm，集中管理角色、超时、反馈合法性和发送节流
 */
#include "board_transport.h"
#include "board_link.h"
#include "can_comm.h"
#include "can.h"
#include <math.h>
#include <string.h>

_Static_assert(sizeof(Chassis_Upload_Data_s) <= CAN_COMM_MAX_BUFFSIZE, "Feedback exceeds CANComm buffer");
_Static_assert(sizeof(Chassis_Ctrl_Cmd_s) <= CAN_COMM_MAX_BUFFSIZE, "Command exceeds CANComm buffer");
static CANCommInstance *board_comm;
static uint32_t last_tx_ms;
BoardTransportDebug_s board_link_debug;

void BoardTransportInit(void)
{
    CANComm_Init_Config_s config = {.can_config.can_handle = BOARD_LINK_CAN_HANDLE};
#ifdef CHASSIS_BOARD
    config.can_config.tx_id = BOARD_CHASSIS_TX_ID;
    config.can_config.rx_id = BOARD_GIMBAL_TX_ID;
    config.send_data_len = sizeof(Chassis_Upload_Data_s);
    config.recv_data_len = sizeof(Chassis_Ctrl_Cmd_s);
    config.daemon_count = BOARD_COMMAND_TIMEOUT_MS / 10u;
#else
    config.can_config.tx_id = BOARD_GIMBAL_TX_ID;
    config.can_config.rx_id = BOARD_CHASSIS_TX_ID;
    config.send_data_len = sizeof(Chassis_Ctrl_Cmd_s);
    config.recv_data_len = sizeof(Chassis_Upload_Data_s);
    config.daemon_count = BOARD_FEEDBACK_TIMEOUT_MS / 10u;
#endif
    board_comm = CANCommInit(&config);
    last_tx_ms = HAL_GetTick();
    memset(&board_link_debug, 0, sizeof(board_link_debug));
}

/** @brief 只记录快照读取结果；不以调试变量延长通信有效期。 */
static void UpdateLinkDebug(uint8_t online)
{
    board_link_debug.rx_online = online;
    board_link_debug.last_rx_ms = board_comm->last_rx_ms;
}

#ifdef CHASSIS_BOARD
uint8_t BoardTransportReadCommand(Chassis_Ctrl_Cmd_s *output)
{
    uint8_t valid = CANCommSnapshot(board_comm, output, BOARD_COMMAND_TIMEOUT_MS);
    if (!valid) memset(output, 0, sizeof(*output));
    UpdateLinkDebug(valid);
    return valid;
}

void BoardTransportSendFeedback(const Chassis_Upload_Data_s *feedback)
{
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - last_tx_ms) >= BOARD_FEEDBACK_PERIOD_MS)
    {
        CANCommSend(board_comm, (uint8_t *)feedback);
        last_tx_ms = now;
    }
}
#else
uint8_t BoardTransportReadFeedback(Chassis_Upload_Data_s *output)
{
    uint8_t valid = CANCommSnapshot(board_comm, output, BOARD_FEEDBACK_TIMEOUT_MS);
    if (valid)
        valid = isfinite(output->real_vx) && isfinite(output->real_vy) &&
            isfinite(output->real_wz) && isfinite(output->yaw_deg) && output->motors_online <= 1 &&
            output->ins_online <= 1 && output->referee_online <= 1;
    if (!valid) memset(output, 0, sizeof(*output));
    UpdateLinkDebug(valid);
    return valid;
}

void BoardTransportSendCommand(const Chassis_Ctrl_Cmd_s *command, uint8_t immediate)
{
    uint32_t now = HAL_GetTick();
    if (immediate || (uint32_t)(now - last_tx_ms) >= BOARD_COMMAND_PERIOD_MS)
    {
        CANCommSend(board_comm, (uint8_t *)command);
        last_tx_ms = now;
    }
}
#endif

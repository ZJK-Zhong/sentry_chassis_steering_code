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
static CANCommInstance *board_link_instance; ///< 本板唯一板间链路实例，由传输服务持有。
static uint32_t last_board_tx_ms;            ///< 最近提交发送的HAL时刻，ms，用于常态节流。
BoardTransportDebug_s board_link_debug;      ///< 公开观测副本，不参与超时或发送判断。

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
    board_link_instance = CANCommInit(&config);
    last_board_tx_ms = HAL_GetTick();
    memset(&board_link_debug, 0, sizeof(board_link_debug));
}

/** @brief 复制本地端口和诊断数据，不改变通信在线标志、错误寄存器或控制状态。 */
static void UpdateTrafficDebug(void)
{
    CANInstance *can = board_link_instance->can_ins;
    uint32_t interrupt_mask = __get_PRIMASK();
    __disable_irq();
    board_link_debug.can_bus = can->can_handle == &hcan1 ? 1u : 2u;
    board_link_debug.tx_id = can->txconf.StdId;
    board_link_debug.rx_id = can->rx_id;
    board_link_debug.tx_data_len = board_link_instance->send_data_len;
    board_link_debug.rx_data_len = board_link_instance->recv_data_len;
    board_link_debug.rx_partial_len = board_link_instance->cur_recv_len;
    board_link_debug.last_part_ms = board_link_instance->last_part_ms;
    board_link_debug.can_error_code = can->can_handle->ErrorCode;
    board_link_debug.can_error_status = can->can_handle->Instance != NULL ? can->can_handle->Instance->ESR : 0;
    board_link_debug.traffic = board_link_instance->diagnostics;
    __set_PRIMASK(interrupt_mask);
}

/** @brief 只记录快照读取结果；不以调试变量延长通信有效期。 */
static void UpdateLinkDebug(uint8_t online)
{
    board_link_debug.rx_online = online;
    board_link_debug.last_rx_ms = board_link_instance->last_rx_ms;
    UpdateTrafficDebug();
}

#ifdef CHASSIS_BOARD
uint8_t BoardTransportReadCommand(Chassis_Ctrl_Cmd_s *output)
{
    uint8_t valid = CANCommSnapshot(board_link_instance, output, BOARD_COMMAND_TIMEOUT_MS);
    if (!valid)
    {
        memset(output, 0, sizeof(*output));
    }
    UpdateLinkDebug(valid);
    return valid;
}

void BoardTransportSendFeedback(const Chassis_Upload_Data_s *feedback)
{
    uint32_t current_time_ms = HAL_GetTick();
    if ((uint32_t)(current_time_ms - last_board_tx_ms) >= BOARD_FEEDBACK_PERIOD_MS)
    {
        CANCommSend(board_link_instance, (uint8_t *)feedback);
        last_board_tx_ms = current_time_ms;
        UpdateTrafficDebug();
    }
}
#else
uint8_t BoardTransportReadFeedback(Chassis_Upload_Data_s *output)
{
    uint8_t valid = CANCommSnapshot(board_link_instance, output, BOARD_FEEDBACK_TIMEOUT_MS);
    if (valid)
    {
        valid = isfinite(output->real_vx) && isfinite(output->real_vy) && isfinite(output->real_wz) &&
                isfinite(output->yaw_deg) && output->motors_online <= 1 && output->ins_online <= 1 &&
                output->referee_online <= 1;
    }
    if (!valid)
    {
        memset(output, 0, sizeof(*output));
    }
    UpdateLinkDebug(valid);
    return valid;
}

void BoardTransportSendCommand(const Chassis_Ctrl_Cmd_s *command, uint8_t immediate)
{
    uint32_t current_time_ms = HAL_GetTick();
    if (immediate || (uint32_t)(current_time_ms - last_board_tx_ms) >= BOARD_COMMAND_PERIOD_MS)
    {
        CANCommSend(board_link_instance, (uint8_t *)command);
        last_board_tx_ms = current_time_ms;
        UpdateTrafficDebug();
    }
}
#endif

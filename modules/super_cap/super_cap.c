/*
 * @Descripttion:
 * @version:
 * @Author: Chenfu
 * @Date: 2022-12-02 21:32:47
 * @LastEditTime: 2022-12-05 15:29:49
 */
#include "super_cap.h"
#include "super_cap_sdk.h" // 引入新版SDK
#include "memory.h"
#include "stdlib.h"
#include "string.h" // 使用 memset, memcpy

static SuperCapInstance *super_cap_instance = NULL; // 可以由app保存此指针

static void SuperCapRxCallback(CANInstance *_instance)
{
    if (super_cap_instance != NULL && _instance->rx_len == 8) {
        // 使用新版 SDK 解析接收到的数据
        SuperCap_ParseRxData(_instance->rx_buff, &super_cap_instance->feedback);
        super_cap_instance->last_rx_ms = HAL_GetTick();
        super_cap_instance->feedback_seen = 1;
    }
}

SuperCapInstance *SuperCapInit(SuperCap_Init_Config_s *supercap_config)
{
    super_cap_instance = (SuperCapInstance *)malloc(sizeof(SuperCapInstance));
    memset(super_cap_instance, 0, sizeof(SuperCapInstance));
    
    supercap_config->can_config.can_module_callback = SuperCapRxCallback;
    super_cap_instance->can_ins = CANRegister(&supercap_config->can_config);
    
    // 初始化一个默认的控制结构体，防止全0导致参数异常
    SuperCap_InitDefaultControl(&super_cap_instance->control);

    return super_cap_instance;
}

// 发送时传入控制参数结构体指针
void SuperCapSend(SuperCapInstance *instance, SuperCap_Control_t *control_data)
{
    // 使用新版 SDK 打包发送数据
    SuperCap_PackTxData(control_data, instance->can_ins->tx_buff);
    
    // 触发底层的 CAN 发送
    CANTransmit(instance->can_ins, 1);
}

// 获取反馈数据
SuperCap_Feedback_t SuperCapGet(SuperCapInstance *instance)
{
    return instance->feedback;
}

// 提供一个接口让外部获取超级电容实例指针，以便在其他模块中使用
SuperCapInstance *GetSuperCapInstance(void)
{
    return super_cap_instance;
}

uint8_t SuperCapSnapshot(SuperCapInstance *instance, SuperCap_Feedback_t *feedback)
{
    if (instance == NULL || feedback == NULL) return 0;
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    uint8_t valid = instance->feedback_seen &&
        (uint32_t)(HAL_GetTick() - instance->last_rx_ms) <= 100u;
    if (valid) *feedback = instance->feedback;
    __set_PRIMASK(irq);
    return valid;
}



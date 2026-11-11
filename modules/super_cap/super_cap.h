/**
 * @file super_cap.h
 * @brief 原超级电容SDK的CAN实例、控制发送和新鲜反馈快照
 */
#ifndef SUPER_CAP_H
#define SUPER_CAP_H
#include "bsp_can.h"
#include "super_cap_sdk.h"

/** @brief 单电容实例；反馈由CAN回调写入，控制字段由底盘应用维护。 */
typedef struct
{
    CANInstance *can_ins;
    SuperCap_Feedback_t feedback;
    SuperCap_Control_t control;
    uint32_t last_rx_ms; // 实际收到8字节反馈的毫秒时间，读取不刷新。
    uint8_t feedback_seen; // 禁止将初始化零值当成真实电容反馈。
} SuperCapInstance;

/** @brief 端口和标准ID由应用填写，方向以本板为参照。 */
typedef struct
{
    CAN_Init_Config_s can_config;
} SuperCap_Init_Config_s;

/**
 * @brief 注册单个电容CAN实例及接收回调，装入SDK默认控制项
 * @param supercap_config 应用填写的本板CAN端口、TX/RX标准ID
 * @return SuperCapInstance* 实例由模块持有；本系统只初始化一次。
 * @note 调度前应用应覆盖裁判功率/缓冲初值，避免使用SDK演示值。
 */
SuperCapInstance *SuperCapInit(SuperCap_Init_Config_s *supercap_config);

/**
 * @brief 按原SDK打包并发送一条8字节控制帧
 * @param instance 初始化返回的实例
 * @param control_data 含裁判功率W、缓冲J和电容策略参数的有效指针
 * @note 调用方负责节流；本项目由PowerControl每100ms发送。
 */
void SuperCapSend(SuperCapInstance *instance, SuperCap_Control_t *control_data);

/**
 * @brief 返回最近缓存，不检查新鲜度
 * @param instance 初始化返回的非空实例
 * @return SuperCap_Feedback_t 缓存副本；控制逻辑应使用SuperCapSnapshot。
 */
SuperCap_Feedback_t SuperCapGet(SuperCapInstance *instance);

/**
 * @brief 获取单电容实例，不转移所有权
 * @return SuperCapInstance* 未初始化时为NULL。
 */
SuperCapInstance *GetSuperCapInstance(void);

/**
 * @brief 在短临界区读取未超过100ms的完整电容反馈快照
 * @param instance 初始化返回的实例；NULL视为无反馈
 * @param feedback 非空反馈目标空间；无效时不改写
 * @return uint8_t 1:反馈新鲜；0:首帧未到、指针为空或反馈过期
 */
uint8_t SuperCapSnapshot(SuperCapInstance *instance, SuperCap_Feedback_t *feedback);
#endif

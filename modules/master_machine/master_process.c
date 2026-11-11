/**
 * @file master_process.c
 * @author neozng
 * @brief  module for recv&send vision data
 * @version beta
 * @date 2022-11-03
 * @todo 增加对串口调试助手协议的支持,包括vofa和serial debug
 * @copyright Copyright (c) 2022
 *
 */
#include "stdint.h"
#include "master_process.h"
#include "seasky_protocol.h"
#include "daemon.h"
#include "bsp_log.h"
#include "robot_def.h"
#include "rm_referee.h"
#include "referee_task.h"
#include "chassis.h"


static Vision_Recv_s recv_data;
static Vision_Send_s send_data;
static DaemonInstance *vision_daemon_instance;
static volatile uint8_t vision_cmd_new_frame = 0;
static volatile uint32_t vision_cmd_last_rx_time_ms = 0;

#define USB_RX_DATA_SIZE 256  // byte
#define USB_RECEIVE_LEN 150   // byte
#define HEADER_SIZE 4         // byte

// Variable Declarations
static uint8_t USB_RX_BUF[USB_RX_DATA_SIZE];

static const Imu_t * IMU;
static const ChassisSpeedVector_t * FDB_SPEED_VECTOR;

// 判断USB连接状态用到的一些变量
static bool USB_OFFLINE = true;
static uint32_t RECEIVE_TIME = 0;
static uint32_t LATEST_RX_TIMESTAMP = 0;
static uint32_t CONTINUE_RECEIVE_CNT = 0;

// 数据发送结构体
// clang-format off
static SendDataDebug_s       SEND_DATA_DEBUG;
static SendDataImu_s         SEND_DATA_IMU;
static SendDataRobotStateInfo_s   SEND_DATA_ROBOT_STATE_INFO;
static SendDataEvent_s       SEND_DATA_EVENT;
static SendDataPidDebug_s    SEND_DATA_PID;
static SendDataAllRobotHp_s  SEND_DATA_ALL_ROBOT_HP;
static SendDataGameStatus_s  SEND_DATA_GAME_STATUS;
static SendDataRobotMotion_s SEND_ROBOT_MOTION_DATA;
static SendDataGroundRobotPosition_s SEND_GROUND_ROBOT_POSITION_DATA;
static SendDataRfidStatus_s  SEND_RFID_STATUS_DATA;
static SendDataRobotStatus_s SEND_ROBOT_STATUS_DATA;
static SendDataJointState_s  SEND_JOINT_STATE_DATA;
static SendDataBuff_s        SEND_BUFF_DATA;
static SendDataSentryInfo_s   SEND_SENTRY_INFO_DATA;

// clang-format on  
// 数据接收结构体
static ReceiveDataRobotCmd_s RECEIVE_ROBOT_CMD_DATA;
static ReceiveDataPidDebug_s RECEIVE_PID_DEBUG_DATA;
static ReceiveDataVirtualRc_s RECEIVE_VIRTUAL_RC_DATA;

// 机器人控制指令数据
RobotCmdData_t ROBOT_CMD_DATA;
static RC_ctrl_t VIRTUAL_RC_CTRL;



// static Publisher_t *masterProcess_pub;
// static Subscriber_t *masterProcess_sub;



//裁判系统数据接收
static referee_info_t * refer_data;







//打包模式，弹速，敌方颜色
void VisionSetFlag(Enemy_Color_e enemy_color, Work_Mode_e work_mode, Bullet_Speed_e bullet_speed)
{
    send_data.enemy_color = enemy_color;
    send_data.work_mode = work_mode;
    send_data.bullet_speed = bullet_speed;
}


//打包姿态角
void VisionSetAltitude(float yaw, float pitch, float roll)
{
    send_data.yaw = yaw;
    send_data.pitch = pitch;
    send_data.roll = roll;
}


//打包姿态角速度
void VisionSetAltitudeVel(float yaw_vel,float pitch_vel,float roll_vel)
{
    send_data.yaw_vel = yaw_vel;
    send_data.pitch_vel = pitch_vel;
    send_data.roll_vel = roll_vel;
}

//打包解算值
void VisionSetChassisVel(float vx,float vy,float wz)
{
    send_data.vx = vx;
    send_data.vy = vy;
    send_data.wz = wz; 
}

uint8_t VisionCmdHasNewFrame(void)
{
    uint8_t has_new = vision_cmd_new_frame;
    vision_cmd_new_frame = 0;
    return has_new;
}

uint32_t VisionCmdLastRxTimeMs(void)
{
    return vision_cmd_last_rx_time_ms;
}

/**
 * @brief 离线回调函数,将在daemon.c中被daemon task调用
 * @attention 由于HAL库的设计问题,串口开启DMA接收之后同时发送有概率出现__HAL_LOCK()导致的死锁,使得无法
 *            进入接收中断.通过daemon判断数据更新,重新调用服务启动函数以解决此问题.
 *
 * @param id vision_usart_instance的地址,此处没用.
 */
static void VisionOfflineCallback(void *id)
{
#ifdef VISION_USE_UART
    USARTServiceInit(vision_usart_instance);
#endif // !VISION_USE_UART
    LOGWARNING("[vision] vision offline, restart communication.");
}

#ifdef VISION_USE_UART

#include "bsp_usart.h"

static USARTInstance *vision_usart_instance;

/**
 * @brief 接收解包回调函数,将在bsp_usart.c中被usart rx callback调用
 * @todo  1.提高可读性,将get_protocol_info的第四个参数增加一个float类型buffer
 *        2.添加标志位解码
 */
static void DecodeVision()
{
    uint16_t flag_register;
    DaemonReload(vision_daemon_instance); // 喂狗
    get_protocol_info(vision_usart_instance->recv_buff, &flag_register, (uint8_t *)&recv_data.pitch);
    // TODO: code to resolve flag_register;
}

Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = DecodeVision;
    conf.recv_buff_size = VISION_RECV_SIZE;
    conf.usart_handle = _handle;
    vision_usart_instance = USARTRegister(&conf);

    // 为master process注册daemon,用于判断视觉通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback, // 离线时调用的回调函数,会重启串口接收
        .owner_id = vision_usart_instance,
        .reload_count = 10,
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    return &recv_data;
}

/**
 * @brief 发送函数
 *
 * @param send 待发送数据
 *
 */
void VisionSend()
{
    // buff和txlen必须为static,才能保证在函数退出后不被释放,使得DMA正确完成发送
    // 析构后的陷阱需要特别注意!
    static uint16_t flag_register;
    static uint8_t send_buff[VISION_SEND_SIZE];
    static uint16_t tx_len;
    // TODO: code to set flag_register
    flag_register = 30 << 8 | 0b00000001;
    // 将数据转化为seasky协议的数据包
    get_protocol_send_data(0x02, flag_register, &send_data.yaw, 3, send_buff, &tx_len);
    USARTSend(vision_usart_instance, send_buff, tx_len, USART_TRANSFER_DMA); // 和视觉通信使用IT,防止和接收使用的DMA冲突
    // 此处为HAL设计的缺陷,DMASTOP会停止发送和接收,导致再也无法进入接收中断.
    // 也可在发送完成中断中重新启动DMA接收,但较为复杂.因此,此处使用IT发送.
    // 若使用了daemon,则也可以使用DMA发送.
}

#endif // VISION_USE_UART

#ifdef VISION_USE_VCP

#include "bsp_usb.h"
static uint8_t *vis_recv_buff;


void UsbSendData(void);
static void UsbReceiveData(uint16_t recv_len);
static void UsbSendDebugData(void);
static void UsbSendImuData(void);
static void UsbSendRobotStateInfoData(void);
static void UsbSendEventData(void);
static void UsbSendAllRobotHpData(void);
static void UsbSendGameStatusData(void);
static void UsbSendRobotMotionData(void);
static void UsbSendGroundRobotPositionData(void);
static void UsbSendRfidStatusData(void);
static void UsbSendRobotStatusData(void);
static void UsbSendJointStateData(void);
static void UsbSendBuffData(void);
static void UsbSendSentryInfoData(void);
static void GetCmdData(void);
static void GetVirtualRcCtrlData(void);

// static void DecodeVision(uint16_t recv_len)
// {
//     uint16_t flag_register;
//     get_protocol_info(vis_recv_buff, &flag_register, (uint8_t *)&recv_data.pitch);
//     // TODO: code to resolve flag_register;
// }

/* 视觉通信初始化 */
Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
    UNUSED(_handle); // 仅为了消除警告
    //初始化USB，注册USB回调
    USB_Init_Config_s conf = {.rx_cbk = UsbReceiveData};
    vis_recv_buff = USBInit(conf);


    //接受裁判系统数据指针
    refer_data=GetRefereeInfo();


    // masterProcess_pub = SubRegister("master_cmd",sizeof(ReceiveDataRobotCmd_s));

    // 数据置零
    memset(&RECEIVE_ROBOT_CMD_DATA, 0, sizeof(ReceiveDataRobotCmd_s));
    memset(&RECEIVE_PID_DEBUG_DATA, 0, sizeof(ReceiveDataPidDebug_s));
    memset(&RECEIVE_VIRTUAL_RC_DATA, 0, sizeof(ReceiveDataVirtualRc_s));
    memset(&ROBOT_CMD_DATA, 0, sizeof(RobotCmdData_t));
    memset(&VIRTUAL_RC_CTRL, 0, sizeof(RC_ctrl_t));

    /*******************************************************************************/
    /* Serial                                                                     */
    /*******************************************************************************/
    
    // 1.初始化调试数据包
    // 帧头部分
    SEND_DATA_DEBUG.frame_header.sof = SEND_SOF;
    SEND_DATA_DEBUG.frame_header.len = (uint8_t)(sizeof(SendDataDebug_s) - 6);
    SEND_DATA_DEBUG.frame_header.id = DEBUG_DATA_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_DEBUG.frame_header), sizeof(SEND_DATA_DEBUG.frame_header));
    // 数据部分
    for (uint8_t i = 0; i < DEBUG_PACKAGE_NUM; i++) {
        SEND_DATA_DEBUG.packages[i].type = 1;
        SEND_DATA_DEBUG.packages[i].name[0] = '\0';
    }
    
    // 2.初始化IMU数据包
    SEND_DATA_IMU.frame_header.sof = SEND_SOF;
    SEND_DATA_IMU.frame_header.len = (uint8_t)(sizeof(SendDataImu_s) - 6);
    SEND_DATA_IMU.frame_header.id = IMU_DATA_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_IMU.frame_header), sizeof(SEND_DATA_IMU.frame_header));
    /*******************************************************************************/
    /* Referee                                                                     */
    /*******************************************************************************/
    
    // 3.初始化机器人信息数据包
    // 帧头部分
    SEND_DATA_ROBOT_STATE_INFO.frame_header.sof = SEND_SOF;
    SEND_DATA_ROBOT_STATE_INFO.frame_header.len = (uint8_t)(sizeof(SendDataRobotStateInfo_s) - 6);
    SEND_DATA_ROBOT_STATE_INFO.frame_header.id = ROBOT_STATE_INFO_DATA_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_ROBOT_STATE_INFO.frame_header), sizeof(SEND_DATA_ROBOT_STATE_INFO.frame_header));
    // 数据部分
    SEND_DATA_ROBOT_STATE_INFO.data.type.chassis = CHASSIS_TYPE;
    SEND_DATA_ROBOT_STATE_INFO.data.type.gimbal = GIMBAL_TYPE;
    SEND_DATA_ROBOT_STATE_INFO.data.type.shoot = SHOOT_TYPE;
    SEND_DATA_ROBOT_STATE_INFO.data.type.arm = MECHANICAL_ARM_TYPE;
    
    // 4.初始化事件数据包
    SEND_DATA_EVENT.frame_header.sof = SEND_SOF;
    SEND_DATA_EVENT.frame_header.len = (uint8_t)(sizeof(SendDataEvent_s) - 6);
    SEND_DATA_EVENT.frame_header.id = EVENT_DATA_SEND_ID;
    append_CRC8_check_sum
        ((uint8_t *)(&SEND_DATA_EVENT.frame_header), sizeof(SEND_DATA_EVENT.frame_header));

    // 5.初始化pid调参数据
    SEND_DATA_PID.frame_header.sof = SEND_SOF;
    SEND_DATA_PID.frame_header.len = (uint8_t)(sizeof(SendDataPidDebug_s) - 6);
    SEND_DATA_PID.frame_header.id = PID_DEBUG_DATA_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_PID.frame_header), sizeof(SEND_DATA_PID.frame_header));

    // 6.初始化所有机器人血量数据
    SEND_DATA_ALL_ROBOT_HP.frame_header.sof = SEND_SOF;
    SEND_DATA_ALL_ROBOT_HP.frame_header.len = (uint8_t)(sizeof(SendDataAllRobotHp_s) - 6);
    SEND_DATA_ALL_ROBOT_HP.frame_header.id = ALL_ROBOT_HP_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_ALL_ROBOT_HP.frame_header),
        sizeof(SEND_DATA_ALL_ROBOT_HP.frame_header));

    // 7.初始化比赛状态数据
    SEND_DATA_GAME_STATUS.frame_header.sof = SEND_SOF;
    SEND_DATA_GAME_STATUS.frame_header.len = (uint8_t)(sizeof(SendDataGameStatus_s) - 6);
    SEND_DATA_GAME_STATUS.frame_header.id = GAME_STATUS_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_GAME_STATUS.frame_header),
        sizeof(SEND_DATA_GAME_STATUS.frame_header));
    
    // 8.初始化机器人运动数据
    SEND_ROBOT_MOTION_DATA.frame_header.sof = SEND_SOF;
    SEND_ROBOT_MOTION_DATA.frame_header.len = (uint8_t)(sizeof(SendDataRobotMotion_s) - 6);
    SEND_ROBOT_MOTION_DATA.frame_header.id = ROBOT_MOTION_DATA_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_ROBOT_MOTION_DATA.frame_header),
        sizeof(SEND_ROBOT_MOTION_DATA.frame_header));
    
    // 9.初始化地面机器人位置数据
    SEND_GROUND_ROBOT_POSITION_DATA.frame_header.sof = SEND_SOF;
    SEND_GROUND_ROBOT_POSITION_DATA.frame_header.len =(uint8_t)(sizeof(SendDataGroundRobotPosition_s) - 6);
    SEND_GROUND_ROBOT_POSITION_DATA.frame_header.id = GROUND_ROBOT_POSITION_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_GROUND_ROBOT_POSITION_DATA.frame_header),
        sizeof(SEND_GROUND_ROBOT_POSITION_DATA.frame_header));

    // 10.初始化RFID状态数据
    SEND_RFID_STATUS_DATA.frame_header.sof = SEND_SOF;
    SEND_RFID_STATUS_DATA.frame_header.len = (uint8_t)(sizeof(SendDataRfidStatus_s) - 6);
    SEND_RFID_STATUS_DATA.frame_header.id = RFID_STATUS_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_RFID_STATUS_DATA.frame_header),
        sizeof(SEND_RFID_STATUS_DATA.frame_header));

    // 11.初始化机器人状态数据
    SEND_ROBOT_STATUS_DATA.frame_header.sof = SEND_SOF;
    SEND_ROBOT_STATUS_DATA.frame_header.len = (uint8_t)(sizeof(SendDataRobotStatus_s) - 6);
    SEND_ROBOT_STATUS_DATA.frame_header.id = ROBOT_STATUS_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_ROBOT_STATUS_DATA.frame_header),
        sizeof(SEND_ROBOT_STATUS_DATA.frame_header));
    
    // 12.初始化云台状态数据
    SEND_JOINT_STATE_DATA.frame_header.sof = SEND_SOF;
    SEND_JOINT_STATE_DATA.frame_header.len = (uint8_t)(sizeof(SendDataJointState_s) - 6);
    SEND_JOINT_STATE_DATA.frame_header.id = JOINT_STATE_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_JOINT_STATE_DATA.frame_header),
        sizeof(SEND_JOINT_STATE_DATA.frame_header));

    // 13.初始化机器人增益和底盘能量数据
    SEND_BUFF_DATA.frame_header.sof = SEND_SOF;
    SEND_BUFF_DATA.frame_header.len = (uint8_t)(sizeof(SendDataBuff_s) - 6);
    SEND_BUFF_DATA.frame_header.id = BUFF_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_BUFF_DATA.frame_header),
        sizeof(SEND_BUFF_DATA.frame_header));
    

    // 14.初始化哨兵信息数据
    SEND_SENTRY_INFO_DATA.frame_header.sof = SEND_SOF;
    SEND_SENTRY_INFO_DATA.frame_header.len = (uint8_t)(sizeof(SendDataSentryInfo_s) - 6);
    SEND_SENTRY_INFO_DATA.frame_header.id = SENTRY_INFO_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_SENTRY_INFO_DATA.frame_header),
        sizeof(SEND_SENTRY_INFO_DATA.frame_header));

    // 为master process注册daemon,用于判断视觉通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback, // 离线时调用的回调函数,会重启串口接收
        .owner_id = NULL,
        .reload_count = 5, // 50ms
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    return &recv_data;
}


//发送给视觉？
void VisionSend()
{
    static uint16_t flag_register;
    static uint8_t send_buff[VISION_SEND_SIZE];
    static uint16_t tx_len;
    // TODO: code to set flag_register
    flag_register = 30 << 8 | 0b00000001;
    // 将数据转化为seasky协议的数据包
    get_protocol_send_data(0x02, flag_register, &send_data.yaw, 3, send_buff, &tx_len);
    USBTransmit(send_buff, tx_len);
}


/**
 * @brief      用USB发送数据
 * @param      None
 * @retval     None
 */
void UsbSendData(void)
{
    // 发送Debug数据
    // UsbSendDebugData();
    // 发送Imu数据
    UsbSendImuData();    
    // 发送RobotStateInfo数据
    // UsbSendRobotStateInfoData();   //后续修改
    // // 发送Event数据
    // UsbSendEventData();
    // 发送PidDebug数据
    // CheckDurationAndSend(Pid);
    // // 发送AllRobotHp数据
    // UsbSendAllRobotHpData(); 
    // 发送GameStatus数据
    UsbSendGameStatusData();    //比赛状态
    // // 发送RobotMotion数据
    // UsbSendRobotMotionData();
    // // 发送GroundRobotPosition数据
    // UsbSendGroundRobotPositionData();
    // 发送RfidStatus数据
    UsbSendRfidStatusData();
    // 发送RobotStatus数据
    UsbSendRobotStatusData();   //机器人状态
    // 发送JointState数据
    // UsbSendJointStateData();
    // // 发送Buff数据
    // UsbSendBuffData();
    // // 发送SentryInfo数据
    // UsbSendSentryInfoData();
}


/**
 * @brief      USB接收数据
 * @param      None
 * @retval     None
 */
static void UsbReceiveData(uint16_t recv_len)
{
    if (recv_len > USB_RX_DATA_SIZE)
    {
        recv_len = USB_RX_DATA_SIZE;
    }

    // 仅解析本次回调真实收到的数据，避免读取缓冲区残留的旧内容
    if (recv_len < HEADER_SIZE)
    {
        return;
    }
    
    // 直接使用vis_recv_buff中的数据进行解析
    uint8_t * sof_address = vis_recv_buff;
    uint8_t * rx_data_end_address = vis_recv_buff + recv_len - 1;
    
    // 解析缓冲区中的所有数据包
    while (sof_address <= rx_data_end_address) {
        // 寻找帧头位置
        while ((sof_address <= rx_data_end_address) && (*(sof_address) != RECEIVE_SOF)) {
            sof_address++;
        }
        // 判断是否超出接收数据范围
        if (sof_address > rx_data_end_address) {
            break;  // 退出循环
        }
        // 检查CRC8校验
        bool crc8_ok = verify_CRC8_check_sum(sof_address, HEADER_SIZE);
        if (crc8_ok) {
            uint8_t data_len = sof_address[1];
            uint8_t data_id = sof_address[2];
            uint16_t packet_len = (uint16_t)(HEADER_SIZE + data_len + 2);

            if ((uint32_t)(rx_data_end_address - sof_address + 1) < packet_len)
            {
                break;
            }

            // 检查整包CRC16校验 4: header size, 2: crc16 size
            bool crc16_ok = verify_CRC16_check_sum(sof_address, packet_len);
            if (crc16_ok) {
                switch (data_id) {
                    case ROBOT_CMD_DATA_RECEIVE_ID: {    //视觉端
                        if (packet_len >= sizeof(ReceiveDataRobotCmd_s))
                        {
                            memcpy(&recv_data.cmd, sof_address, sizeof(ReceiveDataRobotCmd_s));
                            vision_cmd_last_rx_time_ms = DWT_GetTimeline_ms();
                            vision_cmd_new_frame = 1;
                            DaemonReload(vision_daemon_instance);
                        }
                    } break;
                    case PID_DEBUG_DATA_RECEIVE_ID: {
                        if (packet_len >= sizeof(ReceiveDataPidDebug_s))
                        {
                            memcpy(&RECEIVE_PID_DEBUG_DATA, sof_address, sizeof(ReceiveDataPidDebug_s));
                        }
                    } break;
                    case VIRTUAL_RC_DATA_RECEIVE_ID: {
                        if (packet_len >= sizeof(ReceiveDataVirtualRc_s))
                        {
                            memcpy(&RECEIVE_VIRTUAL_RC_DATA, sof_address, sizeof(ReceiveDataVirtualRc_s));
                        }
                    } break;
                    case SENTRYCMD_ID: {
                        if (packet_len >= sizeof(FrameHeader_t) + sizeof(uint32_t) + sizeof(sentrycmd_t) + sizeof(uint16_t))
                        {
                            memcpy(&recv_data.Sentry_Cmd, sof_address + sizeof(FrameHeader_t) + sizeof(uint32_t), sizeof(sentrycmd_t));
                        }
                    } break;
                    default:
                        break;
                }
                if (*((uint32_t *)(&sof_address[4])) > LATEST_RX_TIMESTAMP) {
                    LATEST_RX_TIMESTAMP = *((uint32_t *)(&sof_address[4]));
                    RECEIVE_TIME = DWT_GetTimeline_ms();
                }
            }
            sof_address += packet_len;
        } else {
            sof_address++;
        }
    }
}

/*******************************************************************************/
/* Send Function                                                               */
/*******************************************************************************/


/**
 * @brief 发送DEBUG数据
 * @param duration 发送周期
 */
static void UsbSendDebugData(void)
{
    SEND_DATA_DEBUG.time_stamp = DWT_GetTimeline_ms();
    append_CRC16_check_sum((uint8_t *)&SEND_DATA_DEBUG, sizeof(SendDataDebug_s));
    USBTransmit((uint8_t *)&SEND_DATA_DEBUG, sizeof(SendDataDebug_s));
}




/**
 * @brief 发送IMU数据
 * @param duration 发送周期
 */
static void UsbSendImuData(void)
{
    // if (IMU == NULL) {
    //     return;
    // }

    SEND_DATA_IMU.time_stamp = DWT_GetTimeline_ms();

    SEND_DATA_IMU.data.yaw = send_data.yaw;
    SEND_DATA_IMU.data.pitch = send_data.pitch;
    SEND_DATA_IMU.data.roll = send_data.roll;

    SEND_DATA_IMU.data.yaw_vel = send_data.yaw_vel;
    SEND_DATA_IMU.data.pitch_vel = send_data.pitch_vel;
    SEND_DATA_IMU.data.roll_vel = send_data.roll_vel;

    append_CRC16_check_sum((uint8_t *)&SEND_DATA_IMU, sizeof(SendDataImu_s));
    USBTransmit((uint8_t *)&SEND_DATA_IMU, sizeof(SendDataImu_s));
}

/**
 * @brief 发送机器人信息数据
 * @param duration 发送周期   yangf
 */
static void UsbSendRobotStateInfoData(void)
{
    SEND_DATA_ROBOT_STATE_INFO.time_stamp = DWT_GetTimeline_ms();


    append_CRC16_check_sum((uint8_t *)&SEND_DATA_ROBOT_STATE_INFO, sizeof(SendDataRobotStateInfo_s));
    USBTransmit((uint8_t *)&SEND_DATA_ROBOT_STATE_INFO, sizeof(SendDataRobotStateInfo_s));
}


/**
 * @brief 发送事件数据
 * @param duration 发送周期
 */
static void UsbSendEventData(void)
{
    SEND_DATA_EVENT.time_stamp = DWT_GetTimeline_ms();
    append_CRC16_check_sum((uint8_t *)&SEND_DATA_EVENT, sizeof(SendDataEvent_s));
    USBTransmit((uint8_t *)&SEND_DATA_EVENT, sizeof(SendDataEvent_s));
}

/**
 * @brief 发送PidDubug数据
 * @param duration 发送周期
 */
// static void UsbSendPidDebugData(void)
// {
//     SEND_DATA_PID.time_stamp = DWT_GetTimeline_ms();
//     append_CRC16_check_sum((uint8_t *)&SEND_DATA_PID, sizeof(SendDataPidDebug_s));
// }

/**
 * @brief 发送全场机器人hp信息数据
 * @param duration 发送周期
 */
static void UsbSendAllRobotHpData(void)
{
    SEND_DATA_ALL_ROBOT_HP.time_stamp = DWT_GetTimeline_ms();

    SEND_DATA_ALL_ROBOT_HP.data.ally_1_robot_HP = refer_data->GameRobotHP.ally_1_robot_HP;
    SEND_DATA_ALL_ROBOT_HP.data.ally_2_robot_HP = refer_data->GameRobotHP.ally_2_robot_HP;
    SEND_DATA_ALL_ROBOT_HP.data.ally_3_robot_HP = refer_data->GameRobotHP.ally_3_robot_HP;
    SEND_DATA_ALL_ROBOT_HP.data.ally_4_robot_HP = refer_data->GameRobotHP.ally_4_robot_HP;
    SEND_DATA_ALL_ROBOT_HP.data.reserved = refer_data->GameRobotHP.reserved;
    SEND_DATA_ALL_ROBOT_HP.data.ally_7_robot_HP = refer_data->GameRobotHP.ally_7_robot_HP;
    SEND_DATA_ALL_ROBOT_HP.data.ally_outpost_HP = refer_data->GameRobotHP.ally_outpost_HP;
    SEND_DATA_ALL_ROBOT_HP.data.ally_base_HP = refer_data->GameRobotHP.ally_base_HP;


    append_CRC16_check_sum((uint8_t *)&SEND_DATA_ALL_ROBOT_HP, sizeof(SendDataAllRobotHp_s));
    USBTransmit((uint8_t *)&SEND_DATA_ALL_ROBOT_HP, sizeof(SendDataAllRobotHp_s));
}

/**
 * @brief 发送比赛状态数据
 * @param duration 发送周期
 */
static void UsbSendGameStatusData(void)
{
    SEND_DATA_GAME_STATUS.time_stamp = DWT_GetTimeline_ms();

    SEND_DATA_GAME_STATUS.data.game_progress = refer_data->GameState.game_progress;
    SEND_DATA_GAME_STATUS.data.stage_remain_time = refer_data->GameState.stage_remain_time;

    append_CRC16_check_sum((uint8_t *)&SEND_DATA_GAME_STATUS, sizeof(SendDataGameStatus_s));
    USBTransmit((uint8_t *)&SEND_DATA_GAME_STATUS, sizeof(SendDataGameStatus_s));
}

/**
 * @brief 发送机器人运动数据
 * @param duration 发送周期
 */
static void UsbSendRobotMotionData(void)
{

    SEND_ROBOT_MOTION_DATA.time_stamp = DWT_GetTimeline_ms();

    SEND_ROBOT_MOTION_DATA.data.speed_vector.vx = send_data.vx;
    SEND_ROBOT_MOTION_DATA.data.speed_vector.vy = send_data.vy;
    SEND_ROBOT_MOTION_DATA.data.speed_vector.wz = send_data.wz;

    append_CRC16_check_sum((uint8_t *)&SEND_ROBOT_MOTION_DATA, sizeof(SendDataRobotMotion_s));
    USBTransmit((uint8_t *)&SEND_ROBOT_MOTION_DATA, sizeof(SendDataRobotMotion_s));
}

/**
 * @brief 发送地面机器人位置数据
 * @param duration 发送周期
 */
static void UsbSendGroundRobotPositionData(void)
{
    SEND_GROUND_ROBOT_POSITION_DATA.time_stamp = DWT_GetTimeline_ms();

    SEND_GROUND_ROBOT_POSITION_DATA.data.hero_x = refer_data->GroundRobotPos.hero_x;
    SEND_GROUND_ROBOT_POSITION_DATA.data.hero_y = refer_data->GroundRobotPos.hero_y;
    SEND_GROUND_ROBOT_POSITION_DATA.data.engineer_x = refer_data->GroundRobotPos.engineer_x;
    SEND_GROUND_ROBOT_POSITION_DATA.data.engineer_y = refer_data->GroundRobotPos.engineer_y;
    SEND_GROUND_ROBOT_POSITION_DATA.data.standard_3_x = refer_data->GroundRobotPos.standard_3_x;
    SEND_GROUND_ROBOT_POSITION_DATA.data.standard_3_y = refer_data->GroundRobotPos.standard_3_y;
    SEND_GROUND_ROBOT_POSITION_DATA.data.standard_4_x = refer_data->GroundRobotPos.standard_4_x;
    SEND_GROUND_ROBOT_POSITION_DATA.data.standard_4_y = refer_data->GroundRobotPos.standard_4_y;
    SEND_GROUND_ROBOT_POSITION_DATA.data.reserved1 = refer_data->GroundRobotPos.reserved1;
    SEND_GROUND_ROBOT_POSITION_DATA.data.reserved2 = refer_data->GroundRobotPos.reserved2;

    append_CRC16_check_sum((uint8_t *)&SEND_GROUND_ROBOT_POSITION_DATA, sizeof(SendDataGroundRobotPosition_s));
    USBTransmit((uint8_t *)&SEND_GROUND_ROBOT_POSITION_DATA, sizeof(SendDataGroundRobotPosition_s));
}

/**
 * @brief 发送RFID状态数据
 * @param duration 发送周期
 */
static void UsbSendRfidStatusData(void)
{
    SEND_RFID_STATUS_DATA.time_stamp = DWT_GetTimeline_ms();

    // 基地与高地
    SEND_RFID_STATUS_DATA.data.base_gain_point = (refer_data->RFIDStatus.rfid_status & (1U << 0)) != 0;
    SEND_RFID_STATUS_DATA.data.circular_highland_gain_point = (refer_data->RFIDStatus.rfid_status & (1U << 1)) != 0;
    SEND_RFID_STATUS_DATA.data.enemy_circular_highland_gain_point = (refer_data->RFIDStatus.rfid_status & (1U << 2)) != 0;
    
    // 梯形高地（新版对应原R3/B3）
    SEND_RFID_STATUS_DATA.data.friendly_r3_b3_gain_point = (refer_data->RFIDStatus.rfid_status & (1U << 3)) != 0;
    SEND_RFID_STATUS_DATA.data.enemy_r3_b3_gain_point = (refer_data->RFIDStatus.rfid_status & (1U << 4)) != 0;
    
    // 新版已取消的地形，直接赋0 (false)
    SEND_RFID_STATUS_DATA.data.friendly_r4_b4_gain_point = 0;
    SEND_RFID_STATUS_DATA.data.enemy_r4_b4_gain_point = 0;
    SEND_RFID_STATUS_DATA.data.energy_mechanism_gain_point = 0;
    
    // 飞坡增益点
    SEND_RFID_STATUS_DATA.data.friendly_fly_ramp_front_gain_point = (refer_data->RFIDStatus.rfid_status & (1U << 5)) != 0;
    SEND_RFID_STATUS_DATA.data.friendly_fly_ramp_back_gain_point = (refer_data->RFIDStatus.rfid_status & (1U << 6)) != 0;
    SEND_RFID_STATUS_DATA.data.enemy_fly_ramp_front_gain_point = (refer_data->RFIDStatus.rfid_status & (1U << 7)) != 0;
    SEND_RFID_STATUS_DATA.data.enemy_fly_ramp_back_gain_point = (refer_data->RFIDStatus.rfid_status & (1U << 8)) != 0;
    
    // 前哨站
    SEND_RFID_STATUS_DATA.data.friendly_outpost_gain_point = (refer_data->RFIDStatus.rfid_status & (1U << 18)) != 0;
    
    // 补血点（新版为补给区，bit 19 或 bit 20 任一激活即视为激活）
    SEND_RFID_STATUS_DATA.data.friendly_healing_point = ((refer_data->RFIDStatus.rfid_status & (1U << 19)) != 0) || ((refer_data->RFIDStatus.rfid_status & (1U << 20)) != 0);
    
    // 新版已取消的地形（哨兵区、资源岛、兑换区），直接赋0 (false)
    SEND_RFID_STATUS_DATA.data.friendly_sentry_patrol_area = 0;
    SEND_RFID_STATUS_DATA.data.enemy_sentry_patrol_area = 0;
    SEND_RFID_STATUS_DATA.data.friendly_big_resource_island = 0;
    SEND_RFID_STATUS_DATA.data.enemy_big_resource_island = 0;
    SEND_RFID_STATUS_DATA.data.friendly_exchange_area = 0;
    
    // 中心增益点 (仅 RMUL 适用)
    SEND_RFID_STATUS_DATA.data.center_gain_point = (refer_data->RFIDStatus.rfid_status & (1U << 23)) != 0;


    append_CRC16_check_sum((uint8_t *)&SEND_RFID_STATUS_DATA, sizeof(SendDataRfidStatus_s));
    USBTransmit((uint8_t *)&SEND_RFID_STATUS_DATA, sizeof(SendDataRfidStatus_s));
}

/**
 * @brief 发送机器人状态数据
 * @param duration 发送周期
 */
static void UsbSendRobotStatusData(void)
{
    SEND_ROBOT_STATUS_DATA.time_stamp = DWT_GetTimeline_ms();
    SEND_ROBOT_STATUS_DATA.data.robot_id = refer_data->GameRobotState.robot_id;
    SEND_ROBOT_STATUS_DATA.data.robot_level = refer_data->GameRobotState.robot_level;
    SEND_ROBOT_STATUS_DATA.data.current_up = refer_data->GameRobotState.current_HP;
    SEND_ROBOT_STATUS_DATA.data.maximum_hp= refer_data->GameRobotState.maximum_HP;
    SEND_ROBOT_STATUS_DATA.data.shooter_barrel_cooling_value= refer_data->GameRobotState.shooter_barrel_cooling_value;
    SEND_ROBOT_STATUS_DATA.data.shooter_barrel_heat_limit= refer_data->GameRobotState.shooter_barrel_heat_limit;
    SEND_ROBOT_STATUS_DATA.data.shooter_17mm_1_barrel_heat=refer_data->PowerHeatData.shooter_17mm_barrel_heat;
    SEND_ROBOT_STATUS_DATA.data.robot_pos_x=refer_data->GameRobotPos.x;
    SEND_ROBOT_STATUS_DATA.data.robot_pos_y=refer_data->GameRobotPos.y;
    SEND_ROBOT_STATUS_DATA.data.robot_pos_angle=refer_data->GameRobotPos.angle;
    SEND_ROBOT_STATUS_DATA.data.armor_id=refer_data->RobotHurt.armor_id;
    SEND_ROBOT_STATUS_DATA.data.hp_deduction_reason=refer_data->RobotHurt.hurt_type;
    SEND_ROBOT_STATUS_DATA.data.projectile_allowance_17mm=refer_data->ProjectileAllowance.projectile_allowance_17mm;
    SEND_ROBOT_STATUS_DATA.data.remaining_gold_coin=refer_data->ProjectileAllowance.remaining_gold_coin;
    
    append_CRC16_check_sum((uint8_t *)&SEND_ROBOT_STATUS_DATA, sizeof(SendDataRobotStatus_s));
    USBTransmit((uint8_t *)&SEND_ROBOT_STATUS_DATA, sizeof(SendDataRobotStatus_s));
}

/**
 * @brief 发送云台状态数据
 * @param duration 发送周期
 */
static void UsbSendJointStateData(void)
{
    SEND_JOINT_STATE_DATA.time_stamp = DWT_GetTimeline_ms();
    SEND_JOINT_STATE_DATA.data.pitch = send_data.pitch;
    SEND_JOINT_STATE_DATA.data.yaw = send_data.yaw;
    append_CRC16_check_sum((uint8_t *)&SEND_JOINT_STATE_DATA, sizeof(SendDataJointState_s));
    USBTransmit((uint8_t *)&SEND_JOINT_STATE_DATA, sizeof(SendDataJointState_s));
}

/**
 * @brief 发送机器人增益和底盘能量数据
 * @param duration 发送周期
 */
static void UsbSendBuffData(void)
{
    SEND_BUFF_DATA.time_stamp = DWT_GetTimeline_ms();
    SEND_BUFF_DATA.data.attack_buff=refer_data->BuffMusk.attack_buff;
    SEND_BUFF_DATA.data.defence_buff=refer_data->BuffMusk.defence_buff;
    SEND_BUFF_DATA.data.cooling_buff=refer_data->BuffMusk.cooling_buff;
    SEND_BUFF_DATA.data.recovery_buff=refer_data->BuffMusk.recovery_buff;
    SEND_BUFF_DATA.data.vulnerability_buff=refer_data->BuffMusk.vulnerability_buff;
    SEND_BUFF_DATA.data.remaining_energy=refer_data->BuffMusk.remaining_energy;

    append_CRC16_check_sum((uint8_t *)&SEND_BUFF_DATA, sizeof(SendDataBuff_s));
    USBTransmit((uint8_t *)&SEND_BUFF_DATA, sizeof(SendDataBuff_s));
}




static void UsbSendSentryInfoData(void)
{
    SEND_SENTRY_INFO_DATA.time_stamp = DWT_GetTimeline_ms();

    SEND_SENTRY_INFO_DATA.data.sentry_info=refer_data->SentryInfo.sentry_info;
    SEND_SENTRY_INFO_DATA.data.sentry_info_2=refer_data->SentryInfo.sentry_info_2;


    append_CRC16_check_sum((uint8_t *)&SEND_SENTRY_INFO_DATA, sizeof(SendDataSentryInfo_s));
    USBTransmit((uint8_t *)&SEND_SENTRY_INFO_DATA, sizeof(SendDataSentryInfo_s));

}
/*******************************************************************************/
/* Receive Function                                                            */
/*******************************************************************************/

static void GetCmdData(void)
{
    ROBOT_CMD_DATA.speed_vector.vx = RECEIVE_ROBOT_CMD_DATA.data.speed_vector.vx;
    ROBOT_CMD_DATA.speed_vector.vy = RECEIVE_ROBOT_CMD_DATA.data.speed_vector.vy;
    ROBOT_CMD_DATA.speed_vector.wz = RECEIVE_ROBOT_CMD_DATA.data.speed_vector.wz;

    ROBOT_CMD_DATA.chassis.yaw = RECEIVE_ROBOT_CMD_DATA.data.chassis.yaw;
    ROBOT_CMD_DATA.chassis.pitch = RECEIVE_ROBOT_CMD_DATA.data.chassis.pitch;
    ROBOT_CMD_DATA.chassis.roll = RECEIVE_ROBOT_CMD_DATA.data.chassis.roll;
    ROBOT_CMD_DATA.chassis.leg_length = RECEIVE_ROBOT_CMD_DATA.data.chassis.leg_lenth;

    ROBOT_CMD_DATA.gimbal.yaw = RECEIVE_ROBOT_CMD_DATA.data.gimbal.yaw;
    ROBOT_CMD_DATA.gimbal.pitch = RECEIVE_ROBOT_CMD_DATA.data.gimbal.pitch;

    ROBOT_CMD_DATA.shoot.fire = RECEIVE_ROBOT_CMD_DATA.data.shoot.fire;
    ROBOT_CMD_DATA.shoot.fric_on = RECEIVE_ROBOT_CMD_DATA.data.shoot.fric_on;
}

static void GetVirtualRcCtrlData(void)
{
    memcpy(&VIRTUAL_RC_CTRL, &RECEIVE_VIRTUAL_RC_DATA.data, sizeof(RC_ctrl_t));
}

/*******************************************************************************/
/* Public Function                                                             */
/*******************************************************************************/

/**
 * @brief 修改调试数据包
 * @param index 索引
 * @param data  发送数据
 * @param name  数据名称
 */
void ModifyDebugDataPackage(uint8_t index, float data, const char * name)
{
    SEND_DATA_DEBUG.packages[index].data = data;

    if (SEND_DATA_DEBUG.packages[index].name[0] != '\0') {
        return;
    }

    uint8_t i = 0;
    while (name[i] != '\0' && i < 10) {
        SEND_DATA_DEBUG.packages[index].name[i] = name[i];
        i++;
    }

    //TODO:添加对数据名称的一些检查工作
}

/**
 * @brief 获取上位机控制指令：云台姿态，基于欧拉角 r×p×y
 * @param axis 轴id，可配合定义好的轴id宏 AX_PITCH,AX_YAW 使用
 * @return (rad) 云台姿态
 */
inline float GetScCmdGimbalAngle(uint8_t axis)
{
    if (axis == AX_YAW) {
        return ROBOT_CMD_DATA.gimbal.yaw;
    } else if (axis == AX_PITCH) {
        return ROBOT_CMD_DATA.gimbal.pitch;
    }
    return 0.0f;
}

/**
 * @brief 获取上位机控制指令：底盘坐标系下axis方向运动线速度
 * @param axis 轴id，可配合定义好的轴id宏使用
 * @return float (m/s) 底盘坐标系下axis方向运动线速度
 */
inline float GetScCmdChassisSpeed(uint8_t axis)
{
    if (axis == AX_X)
    {
        return ROBOT_CMD_DATA.speed_vector.vx;
    } 
    else if (axis == AX_Y) 
    {
        return ROBOT_CMD_DATA.speed_vector.vy;
    }
    else if (axis == AX_Z)
    {
        return 0;
    }
    return 0.0f;
}

/**
 * @brief 获取上位机控制指令：底盘坐标系下axis方向运动角速度
 * @param axis 轴id，可配合定义好的轴id宏使用
 * @return float (rad/s) 底盘坐标系下axis方向运动角速度
 */
inline float GetScCmdChassisVelocity(uint8_t axis)
{
    if (axis == AX_Z)
    {
        return ROBOT_CMD_DATA.speed_vector.wz;
    } 
    return 0.0f;
}


/**
 * @brief 获取上位机控制指令：底盘离地高度，平衡底盘中可用作腿长参数
 * @param void
 * @return (m) 底盘离地高度
 */
inline float GetScCmdChassisHeight(void)
{
    return ROBOT_CMD_DATA.chassis.leg_length;
}

/**
 * @brief 获取上位机控制指令：开火
 * @param void
 * @return bool 是否开火
 */
inline bool GetScCmdFire(void)
{
    return ROBOT_CMD_DATA.shoot.fire;
}


/**
 * @brief 获取上位机控制指令：启动摩擦轮
 * @param void
 * @return bool 是否启动摩擦轮
 */
inline bool GetScCmdFricOn(void)
{
    return ROBOT_CMD_DATA.shoot.fric_on;
}





#endif // VISION_USE_VCP

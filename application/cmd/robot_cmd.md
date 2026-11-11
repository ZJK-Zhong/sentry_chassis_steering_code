# 底盘 CMD / 双板命令适配

## 总体说明

本双板底盘默认 `SWERVE_COMMAND_SOURCE_GIMBAL_CAN`，用原 CANComm 从底盘CAN1
接收云台CAN1的0x312命令，发布到原消息中心 `chassis_cmd`。
`ChassisTask` 发布的 `chassis_feed` 每20ms通过0x311回传。
物理端口取自 `board_config.h`，报文和完整设备列表见 [CAN分配](../../CAN_ALLOCATION.md)。

本地DBUS分支保留用于回归测试，通过编译宏选择；遥控拨杆选择运动模式，不改变板角色。
单板实车继续使用独立的Steering-drive-chassis项目。

## 外部接口和数据

`RobotCMDInit(void)` 由RobotInit在BSP后调用一次；初始化CANComm、视觉通信和消息中心。
`RobotCMDTask(void)` 由RobotTask先于ChassisTask调用，检查实际接收时间并发布目标。
控制26字节、反馈28字节，固定宽度字段和pack(1)，两端头文件保持一致。

速度单位mm/s、旋转rad/s、偏角deg。合法输入才进入底盘解算：
X±2000、Y±1500、旋转±π、偏角±180，所有浮点数必须有限，模式编码需在约定范围。
`mode=2` 表示云台坐标系，`chassis_mode` 决定无力、跟随或小陀螺。

## 私有状态和执行条件

`chassis_can_comm` 保存收发实例，`chassis_cmd_recv` 为本轮目标，
`chassis_feedback_data` 为从消息中心读取的状态缓存。
完整消息年龄超过50ms、无首帧、CRC错误或目标非法时发布零力。
CANCommSnapshot复制数据不刷新接收时间；半包不算新命令。
应用目标与电机目标还有各自的20ms看门狗。

`SubGetMessage` 读取本地消息不等于CAN在线；以完整收帧的新鲜度判定控制源。
调用链保持 `CANComm → CMD → message_center → ChassisTask → MotorControlTask`，
本文件不直接发送任何电机电流。

## 通信与控制的边界

CAN实例、超时快照和发送周期已移入[板间通信适配](../board_transport.md)。
CMD负责目标与策略；底盘执行层继续只订阅消息中心。

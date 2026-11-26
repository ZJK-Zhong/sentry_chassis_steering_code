# 板间通信适配

## 职责与依赖

`board_transport.c/.h`位于应用层，依赖共享消息、板级配置和原CANComm。
它负责选择本板CAN端点、初始化报文长度、读取完整消息、执行接收超时和发送节流。
CANComm仍负责分包、CRC和实际接收时间；BSP负责CAN收发和过滤器。
CMD只使用类型明确的读写接口，保留拨杆、解锁、视觉和整车模式决策。
本服务不创建任务，不访问电机，不修改报文ID、字段布局或控制模式。

## 接口

- `BoardTransportInit()`：RobotCMDInit内、创建任务前调用一次。
- 底盘调用`BoardTransportReadCommand()`，失败时得到清零命令；随后由CMD检查数值与模式范围。
- 底盘调用`BoardTransportSendFeedback()`，内部每20ms发送一次。
- 云台调用`BoardTransportReadFeedback()`，完整消息超过100ms或数值非法时返回0并清零输出。
- 云台调用`BoardTransportSendCommand()`，常态10ms节流；模式变化或保护停止时传immediate=1。

底盘命令有效期仍为50ms。读取快照不会延长报文寿命，CRC坏包立即撤销接收有效标志。
本版云台CAN1与底盘CAN1互连，0x312命令、0x311反馈，布局仍为26/28字节。
两个工程的board_transport、board_link和robot_def必须配套；`tests/can_layout.py`校验公共文件一致性。

## 观测

两端均可观察`board_link_debug.rx_online`和`board_link_debug.last_rx_ms`。
它们只展示最近一次接口读取结果，不参与控制判定，也不能替代电机或INS在线标志。
底盘`gimbal_command_valid`进一步表示命令通过了应用范围校验。
云台`gimbal_control.chassis_online`表示有效底盘反馈，`armed`表示整车联锁和解锁结果。
无需再从CMD文件里寻找CANComm实例；私有board_comm仅用于通信模块内部。

`board_link_debug`还包含本机实际注册的`can_bus`、`tx_id/rx_id`、`tx_data_len/rx_data_len`，
以及`can_error_code`（HAL缓存）、`can_error_status`（bxCAN ESR实时快照）。
正常云台为CAN1、发送0x312/26字节、接收0x311/28字节；底盘方向和长度相反。
`traffic`只记录本地通信诊断，不进入线上报文，不参与解锁或运动判定：

| 字段 | 含义与定位方法 |
| --- | --- |
| `tx_message_count` | 应用提交给CANComm的消息数，应持续增长 |
| `tx_frame_queued_count` | 成功入发送邮箱的分包数；不等于对端已收到或确认 |
| `tx_frame_fail_count` | 发送调用失败数，增加时先检查本板CAN状态与负载 |
| `rx_frame_count` | 到达该通信实例回调的原始帧数，含无法组包的帧 |
| `rx_message_count` | 通过完整长度、尾标和CRC检查的消息数 |
| `rx_crc_error_count` | 完整消息CRC错误数 |
| `rx_format_error_count` | 声明长度、总长度、尾标或DLC错误数 |
| `rx_partial_timeout_count` | 半包超时被丢弃次数 |
| `last_rx_dlc / last_rx_frame` | 最近一帧的长度与原始内容；不是完整消息 |

若`last_rx_ms=0`且`rx_frame_count=0`，说明尚无帧到达这个接收实例，
应结合对端发送计数检查实际固件、端口、ID、过滤器、总线与中断；
若原始帧增长而完整消息不增长，则结合长度、CRC和半包超时计数检查组包。
`rx_partial_len`和`last_part_ms`提供当前半包进度；所有计数上电清零，自然回绕。

## 验证边界

原集成测试继续覆盖分包、CRC、断包恢复、50/100ms超时、模式变化即时零力和20ms反馈节流。
这些是仿真验证，实车接线、总线负载和机械响应仍需双板联调。

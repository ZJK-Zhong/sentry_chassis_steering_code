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

## 验证边界

原集成测试继续覆盖分包、CRC、断包恢复、50/100ms超时、模式变化即时零力和20ms反馈节流。
这些是仿真验证，实车接线、总线负载和机械响应仍需双板联调。

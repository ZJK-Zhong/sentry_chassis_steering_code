# 舵轮双板工程的模块边界与阅读顺序

## 数据流

```text
云台USART3遥控 ─┐
视觉传输 → VisionCommand ─┼→ 云台CMD采集 → RobotControl → 云台CMD分发
本板gimbal_feed ──────────┤                       ├→ gimbal_cmd → GimbalTask
底盘CAN反馈 → BoardTransport ┘                       ├→ shoot_cmd → ShootTask
                                                  └→ BoardTransport → CANComm → CAN1
底盘CAN1 → CANComm → BoardTransport → 底盘CMD → chassis_cmd
    → ChassisTask（反馈/保护）→ ChassisMotion（参考系/转速）
    → SwerveSolve（四轮逆运动学）→ 电机目标 → 原MotorTask（PID/功率分配/CAN）
底盘状态 → chassis_feed → 底盘CMD → BoardTransport → 云台CMD → 本板chassis_feed
```

两端仍使用修改版rm-sentry的BSP、消息中心、INS、电机和任务框架。
单板工程`Steering-drive-chassis`是独立交付，本次整理对象为两个双板工程。

## 各层负责什么

| 文件/模块 | 负责 | 调用边界 |
| --- | --- | --- |
| 云台`cmd/robot_cmd.c` | 采集本轮快照、调用策略、发布结果 | 可用遥控/视觉/板间服务和消息中心，不计算电机目标 |
| 云台`cmd/robot_control.c` | 双下解锁、模式选择、巡查、三类目标生成 | 输入快照+调用方持有的状态；不读外设、不读时钟、不发布话题 |
| 云台`cmd/robot_control_config.h` | 通道比例、巡查增量、软件限幅、安装补偿 | 每项说明单位；每轮增量对应当前机器人任务周期 |
| 云台`cmd/vision_command.c` | 首帧、新鲜度、有限数和协议字段检查 | 封装原视觉模块，DWT时间轴120ms；失败清零快照 |
| 两端`board_transport.c` | 唯一链路实例、CANComm快照、收发周期及超时 | 不选操作模式、不持有电机对象 |
| 底盘`cmd/robot_cmd.c` | 所选控制源适配、完整命令校验与消息转发 | 不访问底盘电机；默认来源仍为云台CAN |
| 底盘`chassis/chassis.c` | 设备初始化、反馈、执行联锁、轮速限幅、功率及反馈发布 | 应用层负责硬件服务，运动层只得到明确的输入 |
| 底盘`chassis/chassis_motion.c` | 小陀螺参考捕获、转速爬升、角速度修正、坐标变换 | 测量、配置、实际毫秒间隔显式传入；不读取CAN/INS；复用原PID |
| 底盘`modules/swerve` | 轮心速度矢量、最短转向、驱动反转 | 仅依赖标准C库与显式几何，不依赖BSP或应用宏 |
| 云台`gimbal/gimbal.c` | 本板IMU/电机反馈及两轴执行联锁 | 通过gimbal_cmd/gimbal_feed与策略解耦 |
| 云台`shoot/shoot.c` | 摩擦轮预热/实测速度、热量、拨弹与反转解堵 | 从话题读取请求和裁判反馈，独立执行最终发射联锁 |

`RobotControlState_s`和`ChassisMotionState_s`由调用方持有，不使用隐藏的全局策略目标。
头文件说明参数、单位、所有权和调用条件；私有实现的步骤及原因写在对应C文件。

## 命名与格式

- 变量用`lower_snake_case`，函数用动词开头的`PascalCase`。
- 物理量在内部变量名标明单位：`steering_feedback_deg`、`yaw_rate_radps`、`wheel_speed_mps`。
- 对象名体现角色：`drive_motors`、`steering_motors`、`gimbal_command_subscriber`。
- 公共数据类型和接口用中文Doxygen；字段用`///<`，说明含义及单位，避免只重复变量名。
- `.clang-format`固定四空格、Allman大括号、120列和尾注释对齐。只格式化本工程应用及舵轮模块；
  Cube/HAL/FreeRTOS及保留的原框架文件不整库重排。
- 原框架API、线上协议字段、既有Watch入口保持其名称；不为了命名统一破坏接口。

## 调试入口

| Watch表达式 | 内容 |
| --- | --- |
| `swerve_debug` | 底盘模式、在线状态、八电机目标/反馈、姿态和旋转参考 |
| `gimbal_control` | 拨杆请求、实际执行模式、解锁状态及三类目标 |
| `gimbal_feedback_data.gimbal_imu_data` | 云台板本地完整INS姿态，展开一个结构即可查看 |
| `INS` | 原INS模块数据；使用时同时核对在线/新鲜度 |
| `board_link_debug` | 板间接收状态和最近完整收帧时间 |

`swerve_debug`、`gimbal_control`、`board_link_debug`是观测副本，控制不读取它们。
异步Watch不能保证跨字段原子一致；改写观测副本不能改变解锁状态或电机目标。
`gimbal_feedback_data`是云台应用发布的反馈，不是板间发送的IMU报文。
板间端口、报文ID及全部设备接线见[CAN分配](CAN_ALLOCATION.md)。

## 验证与剩余耦合

`tests/run.ps1`运行ARM仿真：模式组合、解锁/失联、真实消息中心与CANComm、反馈联锁、
旋转参考与爬升、发射预热/热量/解堵，以及改写公开Watch不影响控制。
新增交错更新两套控制/运动上下文的回归，检查解锁、连续目标与旋转参考没有跨实例污染。
底盘另有仅依赖标准C头文件的逆运动学回归；`tests/can_layout.py`核对13电机、超级电容、
共享协议一致性和同物理总线的ID/控制槽位冲突。

仍保留原`robot_def.h`对INS/视觉类型头文件、遥控快照类型对原遥控头文件的依赖，以及PowerControl与超级电容的关联。
运动模块复用原PID；其初始化会读取DWT计数，角加速度爬升仍只使用应用传入的毫秒间隔。
策略层不调用硬件服务，但目前不能宣称整个工程完全无硬件头文件依赖。
软件验证不代替实车CAN、机械响应及实际周期测试。

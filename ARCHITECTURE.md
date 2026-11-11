# 舵轮工程模块边界

## 数据流与职责

```text
遥控/视觉 -> 云台CMD策略 -> BoardTransport -> CANComm -> BSP CAN
底盘BSP CAN -> CANComm -> BoardTransport -> 底盘CMD -> chassis_cmd话题
chassis_cmd -> ChassisTask -> SwerveSolve(显式几何参数) -> 电机参考
电机参考 -> 原MotorControlTask -> PID/功率分配 -> CAN电流帧
底盘状态 -> chassis_feed话题 -> 底盘CMD -> BoardTransport -> 云台
云台CMD -> gimbal_cmd/shoot_cmd话题 -> GimbalTask/ShootTask -> 本板电机参考
```

单板工程默认用本地遥控CMD发布同一个chassis_cmd话题，不运行双板应用服务。
单板保留的旧可选CAN入口及接法未与双板交付混用；双板实验应成对烧录两个sentry工程。

| 边界 | 规则 |
| --- | --- |
| CMD/执行应用 | 用原message_center传递目标与反馈，不访问其他应用的电机指针 |
| 双板通信/整车策略 | BoardTransport处理角色、快照与周期，CMD处理模式和解锁 |
| 解算/硬件 | SwerveSolve只接收几何、速度和舵角，不包含swerve_config或任何BSP头文件 |
| 应用/电机输出 | 应用设参考，原电机任务闭环并合帧发送 |
| 控制/调试 | 私有状态负责计算，公开Watch结构仅接收本轮副本 |

## 调试快照

底盘继续看`swerve_debug`，云台继续看`gimbal_control`，字段和单位保持原约定。
快照不再保存控制状态；本轮结束后复制。异步读取不能保证跨字段原子一致。
板间通信可看`board_link_debug.rx_online/last_rx_ms`；它也不参与控制判定。
应用内部的电机反馈、INS新鲜度和命令合法性校验保持原位置和时限。

## 配置与行为

SwerveGeometry_s由底盘应用从swerve_config.h构造，传入SwerveSolve。
保留当前方形布局、方向约定、减速比和原算法3.14换算系数；本次只解除参数来源依赖。
拨杆组合、PID、零位、通信端口/ID、报文布局、任务周期与保护动作均保持原配置。
接口、参数单位、调用条件和观测限制在各头文件以中文Doxygen说明。

## 回归与范围

tests/run.ps1包含原ARM仿真回归；两个底盘工程另有独立解算构建，只使用模块头目录和标准C库。
测试交错使用不同几何参数，并在运行中改写公开Watch快照，检查参考角、转速爬升与解锁状态不受影响。
双板工程继续执行tests/can_layout.py，检查共享协议/实现一致性和CAN分配。

本轮集中处理控制快照、解算依赖和双板通信边界。
原PowerControl与电容协议的关联、robot_def对INS/视觉头文件的依赖仍保留，尚未称为完全独立模块。
上述验证不包含实车烧录、实际CAN负载或电机机械响应。

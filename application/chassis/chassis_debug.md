# 底盘八电机统一调试入口

本说明适用于重构后的双板底盘`sentry_chassis_steering_code`。
重新编译、烧录底盘固件后，Orbit加载同次构建的ELF，在Watch只添加：

```c
chassis_move
```

结构体声明在`application/chassis/chassis.h`，实例和刷新函数在`chassis.c`。
驱动和舵向电机使用对称的分组名称，轮序都是0左前、1右前、2右后、3左后：

| 展开位置 | 内容 |
| --- | --- |
| `drive_motors[0..3]` | 四个M3508驱动电机 |
| `steering_motors[0..3]` | 四个GM6020舵向电机 |
| `state` | 原swerve_debug的模式、在线状态、姿态和运动目标 |
| `command` | 本轮底盘应用读取的命令，需结合state.command_online解释 |
| `updated_at_ms` | 本次完整反馈采集的HAL毫秒时刻 |

每个电机的`measure`都包含原反馈：`ecd`、`last_ecd`、`angle_single_round`、`total_angle`、
`total_round`、`speed_aps`、`real_current`、`temperature`。`real_current`沿用电机协议数值，不能直接当安培。

驱动分组另外提供`actual_speed_rpm`和`target_speed_rpm`，均为转子rpm，未乘减速比。
舵向分组另外提供`actual_angle_deg`、`target_angle_deg`和`zero_encoder`；
actual_angle_deg已经减去机械零位并修正安装方向，区别于measure中的电机原多圈角。
两组都包含`can_bus`、`motor_id`、`feedback_seen`、`online`、`last_rx_ms`和`stop_flag`。

示例：展开`chassis_move.drive_motors[0].measure`可看左前驱动电流/温度/编码器；
展开`chassis_move.steering_motors[0]`可对照左前实际舵角与目标。
不需要像旧指针结构一样再解引用电机测量指针。

每轮ChassisTask结束刷新；上电未收首帧时feedback_seen/online均为0。
失联后保留末帧measure供排查，但online为0，不能把残留转速当作当前实测值。
反馈在短临界区复制，八电机并非同时采样；异步Watch也不能保证整结构原子一致。
原swerve_debug保留；chassis_move只保存副本，没有控制指针，控制/PID/功率模块不读取它。
stop_flag表示应用启停请求，实际输出还受电机任务超时和功率分配影响。
该结构不经CAN发送，未改变线上协议或原控制链路，也不兼容旧chassis_move_t的全部成员布局。

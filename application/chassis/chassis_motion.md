# 底盘运动处理

## 职责与文件

`chassis_motion.h/.c`处理旋转参考、转速爬升、反馈修正和输入速度到车体系的转换。
命令、INS测量、实际任务间隔和配置由Chassis应用显式传入；不直接调用CAN、电机或INS服务。原PID初始化会读取DWT计数，爬升周期仍由应用显式传入。
四舵轮逆运动学仍在独立`modules/swerve`，设备保护、功率和电机使能仍由`chassis.c`负责。

## 类型与接口

`ChassisMotionConfig_s`保存角加速度、修正增益/上限、最终转速上限和跟随增益；单位在头文件逐项说明。
`ChassisMotionState_s`保存PID、入口航向、爬升参考及本轮速度输出；每套底盘实例独立持有。

`ChassisMotionReset(state,config,yaw_deg)`在首次使用或无力时清输出并复位参考。
`ChassisMotionUpdate(...)`接收已通过保护/数值校验的命令；`previous_mode`为上轮实际模式，
`elapsed_ms`为应用测得且已通过超时检查的实际毫秒间隔。

小陀螺只在模式入口重新捕获航向，每轮按实际间隔爬升角速度，再叠加原PID角速度误差修正。
非旋转模式重置旋转参考；NO_FOLLOW角速度为零，FOLLOW按外部相对角生成旋转目标。

| mode字段 | 平移参考角 |
| --- | --- |
| 0 | 车体系，不旋转输入速度 |
| 1 | 底盘INS航向减外部参考角 |
| 2 | 外部云台相对角 |
| 3 | 小陀螺时底盘INS航向减入口航向；其余模式按车体系 |

## 使用示例

```c
ChassisMotionState_s motion = {0};
ChassisMotionReset(&motion, &config, yaw_deg);
ChassisMotionUpdate(&motion, &config, &command, previous_mode,
                    yaw_deg, gyro_z_radps, elapsed_ms);
/* forward_velocity_mmps、right_velocity_mmps、yaw_rate_radps交给轮组解算。 */
```

应用约定X前、Y右、俯视逆时针为正。mm/s到m/s、Y取反和rad/s到deg/s的轮组适配发生在
`ApplyWheelTargets`中。转速反馈不是位置闭环，也不能消除INS绝对航向漂移。

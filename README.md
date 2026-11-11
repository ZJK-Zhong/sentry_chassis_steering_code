# sentry_chassis_steering_code

基于修改版 `E:/RM/rm-sentry` 框架的双板舵轮底盘从控，默认从 **底盘CAN1**
接收云台CAN1命令；接收机插云台USART3。

本板运行CMD、四舵轮应用、INS、原电机/功率控制、裁判USART6/UI和超级电容CAN2。
双板模式由云台按rm-sentry左右拨杆组合选择，左右均下为无力/解锁；
移动、小陀螺、摩擦轮、打弹与视觉模式见README_STEERING.md。
全部八个底盘电机和舵零位沿用已调试舵轮配置。

- [全部CAN设备及物理接线](CAN_ALLOCATION.md)：十三电机、板间报文、电容、裁判数据流与负载前提。
- [调用链、遥控、调试和验证](README_STEERING.md)。
- [底盘应用](application/chassis/chassis.md)、[CAN命令适配](application/cmd/robot_cmd.md)。

构建：`mingw32-make -j24`，产物 `build/sentry_chassis_steering_code.elf`。
本工程需要同级云台项目成对使用；单板遥控继续使用已有 `Steering-drive-chassis`。

模块职责、命名注释规范、控制/调试隔离及回归范围见[架构说明](ARCHITECTURE.md)。

旋转参考和坐标变换已独立为[运动处理](application/chassis/chassis_motion.md)，
设备采集、执行保护和功率处理仍由底盘应用编排。

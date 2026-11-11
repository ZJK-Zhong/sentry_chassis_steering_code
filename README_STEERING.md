# 双板舵轮工程开发说明

## 来源和职责

框架沿用修改版 `E:/RM/rm-sentry` 的 BSP、Module、FreeRTOS、消息中心、DJIMotor、
PID、BMI088/QuaternionEKF、裁判、电容与构建方式。舵轮解算和已验证的轮组参数来自旧舵轮工程。
这两个工程继续单板重构成果，并分别增加底盘从控和云台主控应用，不是旧双板仓库的复制品。
现有 `Steering-drive-chassis` 仍为默认本地遥控的单板工程。

| 工程 | 本板任务 |
| --- | --- |
| sentry_chassis_steering_code | CAN命令 → CMD → chassis_cmd → ChassisTask → 舵轮解算/轮组目标；裁判/UI、电容 |
| sentry_gimbal_steering_code | USART3遥控 → CMD → CAN底盘命令；本地 GimbalTask、ShootTask → 电机目标 |

RobotTask 每轮末延时 5ms；MotorTask 延时 2ms，由原电机模块合帧发送；
INS 延时 1ms，Daemon 延时 10ms。实际周期还包括计算和调度时间。
云台不创建底盘应用/UI任务；底盘不注册云台或发射电机。
USB视觉传输用于比赛/云台打弹模式；模式选择、视觉瞄准/巡查与超时处理沿用rm-sentry的应用逻辑。

硬件总线、全部十三台电机、超级电容及裁判数据流见 [CAN分配说明](CAN_ALLOCATION.md)。

## 控制和参考系

接收机插云台 C 板 USART3，双板整车采用rm-sentry的左右三段开关组合：

| 左拨杆 ＼ 右拨杆 | 下 | 中 | 上 |
| --- | --- | --- | --- |
| 下 | 无力模式 | 摩擦轮单开 | 遥控打弹 |
| 中 | 比赛模式（视觉） | 安全模式 | 安全模式 |
| 上 | 遥控移动/底盘跟随 | 小陀螺 | 云台打弹（视觉） |

上电或遥控/CAN/本地云台反馈失效、拨轮急停恢复后，必须先**左右都拨下**解锁。
只有右下不能解锁：左中右下是比赛，左上右下是移动。
拨轮大于300进入急停并撤销解锁；两种安全组合输出全零力，但不代替双下解锁。
进入安全/无力后再恢复工作，先按当前INS对齐云台目标，避免追赶旧姿态。

移动：底盘跟随云台，手动平移/云台，关闭发射。
小陀螺：手动平移/云台，底盘120度/秒、按60度/秒平方爬升，关闭发射。
摩擦轮单开和遥控打弹：底盘可手动平移但不跟随Yaw，云台手动；
前者只开摩擦轮，后者请求连发，实际拨弹仍受轮速/启动时间/热量联锁。

比赛：使用原USB视觉命令控制导航平移、旋转、云台及发射；
云台打弹：底盘零力，仅视觉控制云台及发射（按rm-sentry实际执行代码）。
两者要求完整视觉命令在120ms内更新；超时保持云台目标，底盘零速度、发射关闭。
仅在视觉通信在线且tracking=0时进行原巡查轨迹，不把通信断线当作丢目标巡查。
视觉Yaw/Pitch安装补偿沿用rm-sentry的0.65/16.5度，实车仍需标定。

右摇杆前后/左右控制底盘X/Y，左摇杆水平/垂直控制Yaw/Pitch。
手动平移保留舵轮已验证方向与6/2比例，死区±10，X/Y限幅2000/1500mm/s。
手动模式使用mode=2：**平移X正方向沿云台前方**，用Yaw相对归中3938的角度转换到车体系。
比赛使用mode=1，视觉vx/vy按原-1400/+1400比例、旋转按2倍映射，再限制在本舵轮上限内。
小陀螺保留当前120度/秒，所有旋转目标不超过πrad/s；Pitch保留±25度软件限幅，
不照搬全向轮更大的转速与自动模式±35度范围。

本模式表仅适用于双板两个工程。现有单板Steering-drive-chassis继续使用原右拨杆三档；
其小陀螺仍以进入模式时的底盘方向为参考。

## 硬件与参数

轮序：左前、右前、右后、左后；驱动CAN1 ID1~4，舵向CAN2 ID1~4。
舵零位为 `4976, 701, 7532, 688`；轮心半径0.216m，轮周长0.38m，输出轴/转子转速比0.0526。
驱动原速度环使用rpm、P=5，舵向角度环单位deg、P=360；
最短转向、驱动反转、同比例限速和舵角误差余弦降速在原重构实现上保留。

云台 CAN 和 ID、Yaw 归中3938取自旧舵轮；云台/发射 PID 和 INS 轴约定取自 rm-sentry。
Yaw采用角度+速度环，Pitch保留 rm-sentry 实际开启的角度环。
Pitch目标暂限±25度，**不是实测机械限位**；正负号、安装方向、零位和增益仍需架空调试。

两个摩擦轮保持相反方向、速度30000deg/s，拨盘连发10000deg/s；
摩擦轮启动至少300ms且两轮实测达80%后才允许拨弹，
还必须收到真实裁判热量并满足 `current_heat + 50 < heat_limit`。
不启用旧代码中未完成的单发/三发位置环。

## 保护与观测

- 底盘命令首帧/50ms有效期、云台反馈100ms有效期均基于接收时间；CRC坏包撤销有效标志。
- 任一底盘电机反馈超过20ms，八电机停止；云台两轴或发射三电机分别检查反馈与任务目标有效性。
- MotorTask 独立检查目标/反馈20ms，RobotTask异常后不会持续输出旧目标。
- 零力使用停止标志发送零电流，不是速度零目标的闭环刹车。
- INS要通过真实初始化/快照检查，上电校准期间保持静止。
- 裁判失联禁止拨弹；电容失联退回应用功率上限，模型仍只涵盖驱动轮。

底盘 Watch：`swerve_debug`、`drive[i]->measure`、`steer[i]->measure`。
云台 Watch：`gimbal_control`、`yaw_motor->measure`、`pitch_motor->measure`。
发射 Watch：`shoot_feedback`、`friction_l->measure`、`friction_r->measure`、`loader->measure`。
文件内 static 变量需要在对应源码作用域观察；反馈单位以字段注释为准。
旧裁判 UI 的演示枚举不代表实际云台反馈，调试以应用结构为准。

## 构建和测试

在对应工程根目录执行：

```powershell
$env:PATH = 'D:\msys64\mingw64\bin;' + $env:PATH
mingw32-make -j24
```

产物为各自 `build/<工程目录名>.elf/.hex/.bin`。
保留 CMake 入口，可使用独立 `build-cmake` 目录；本机没有找到 CMake，尚未验证该入口。
Make已用ARM GCC 13.3.0验证。newlib-nosys的_read/_write/_close/_lseek占位链接警告仍存在。

```powershell
python -m pip install -r tests/requirements.txt
powershell -NoProfile -ExecutionPolicy Bypass -File tests/run.ps1
python tests/can_layout.py
```

run.ps1 也接受 `-PythonExecutable` 和 `-RuntimePath`，用于已安装依赖。
底盘覆盖真实解算/CMD/pubsub/PID、左右拨杆、本地/CAN两输入、INS/RC/电机失联，
以及真实电容SDK的裁判转发与新鲜度；底盘测试中CANComm传输本身用桩。
两端还以真实CAN BSP测试过滤器全部ID槽初始化、FIFO连续收帧与非法帧拒绝。
云台测试覆盖真实CMD/云台/发射/DJIMotor/CANComm，包含分包、CRC、断包恢复、
九种拨杆组合、双下解锁、失联/急停归零、视觉模式/巡查/120ms超时、摩擦轮准备与热量联锁。
物理UART/CAN/INS和视觉输入接口由测试模拟，没有连接真实上位机。

以上为软件验证，没有进行实车烧录。先按CAN分配确认接线和反馈速率，
架空验证所有零电流状态、方向、机械限位，再开展低速跟随和小陀螺实验。

## 模块边界优化

详见[架构说明](ARCHITECTURE.md)。Watch入口保持原名称，控制改用私有状态；
解算接收显式几何参数。双板通信适配服务只在成对的双板工程中启用。

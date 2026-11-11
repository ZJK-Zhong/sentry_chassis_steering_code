# swerve_kinematics

## 总体说明

四舵轮运动学模块将车体平移/旋转目标分解为四组轮缘速度矢量，再求舵角和驱动转速。
仅复用原舵轮成品算法及硬件参数，不带入其任务、遥控、INS或电机驱动框架。

模块不持有电机实例，也不发送CAN报文。电机在线检查、车体坐标转换、轮速限幅、
舵向误差补偿和参考值下发由底盘应用完成。

## 代码结构

- `swerve_kinematics.h`：目标类型和两个计算接口，包含输入单位及调用条件。
- `swerve_kinematics.c`：弧度归一化、四轮矢量合成、最短转向和驱动反转。
- `SwerveGeometry_s`：应用通过参数传入几何与传动信息；本模块不包含应用配置头文件。

## 外部接口与类型

`SwerveTarget` 是单个轮组的解算结果：

| 字段 | 含义 |
| --- | --- |
| `angle_deg` | 接在当前实际多圈舵角附近的目标，度 |
| `speed_rpm` | 驱动电机转子转速，rpm，正负表示转向 |
| `reversed` | 为缩短舵向行程而选择驱动反转时为1 |

`SwerveNormalizeRad(angle)` 将有限弧度值折算到[-PI, PI]。
`SwerveSolve(&geometry, vx, vy, wz_dps, actual_deg, target)` 输出四轮目标：

- geometry包含中心到轮心距离(m)、轮周长(m)、输出轴/转子转速比，均须为有限正数。
- vx、vy为m/s，X向前、Y向左。
- wz_dps为度/秒，俯视逆时针为正。
- actual_deg为已校零且修正方向的实际多圈舵角，度。
- 两个数组都至少有4项，索引依次为左前、右前、右后、左后。

接口不校验指针、数组长度或非有限数，调用方应在进入计算前保证输入有效。

## 计算流程与中间变量

设中心到轮心距离为r，正方形布局在单轴的投影为a=r/sqrt(2)。
轮心位置依次为(a,a)、(a,-a)、(-a,-a)、(-a,a)，使用X向前、Y向左坐标。

对位置(px_i, py_i)处的轮子：

```text
wheel_vx_i = vx - wz_radps * py_i
wheel_vy_i = vy + wz_radps * px_i
wheel_speed = sqrt(wheel_vx_i^2 + wheel_vy_i^2)
wheel_angle = atan2(wheel_vy_i, wheel_vx_i)
```

实现保留成品代码的3.14角度换算系数。轮速换算为转子rpm：

```text
rotor_rpm = wheel_speed * 60 / (轮周长 * 输出轴转速/转子转速)
```

每轮只归一化“目标方向 - 当前舵角”的差值。如果差值超过正负90度，
将舵角少转180度，并将驱动转速反号。这样得到相同轮缘矢量，同时减少舵向行程。
最后将角差加回当前多圈角，避免跨编码器零位时产生一整圈目标跳变。

theta、vw、ratio、x/y为单次调用的局部中间量；模块没有持续运行的私有任务或电机缓存。

## 使用范例

以下输入约定属于解算模块，应用层mm/s和Y向右的速度应先转换：

```c
const SwerveGeometry_s geometry = {0.216f, 0.38f, 0.0526f};
float actual_deg[4] = {0.0f, 0.0f, 0.0f, 0.0f};
SwerveTarget target[4] = {0};

// 1m/s正前进、无旋转；四轮目标舵角为0度，驱动转速同向。
SwerveSolve(&geometry, 1.0f, 0.0f, 0.0f, actual_deg, target);

// 停止驱动，保留target里上一轮的舵角；因此target应跨调用保存。
SwerveSolve(&geometry, 0.0f, 0.0f, 0.0f, actual_deg, target);
```

全零输入只把speed_rpm置零，angle_deg和reversed保留原值；首次调用前必须初始化target。
底盘应用将target作为静态缓存，并在无力状态把舵角目标同步为实际角度。

## 注意事项

- 当前几何展开假定正方形轮心布局，改成长方形时不能只改一个半径参数。
- 多圈舵角的零位和正方向应在应用层统一，模块不会读取编码器或处理机械零位。
- 驱动限速及转向未到位时的牵引补偿不在本函数内，详见底盘apply_targets。
- 软件解算正确不等于实车零位、方向和长期INS航向已经校准。

可运行`tests/run.ps1`中的独立解算测试；该构建只包含本模块头目录和标准C库，
不提供应用、BSP或RTOS头文件，并交错验证两套几何配置，防止隐含全局参数依赖。

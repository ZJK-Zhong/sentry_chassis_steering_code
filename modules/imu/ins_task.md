# ins_task

<p align='right'>neozng1@hnu.edu.cn</p>

## 硬触发流程

![image-20221113212706633](.assets\image-20221113212706633.png)

`times%10` 是固定相机的采集频率为100hz，请根据视觉算法实际能达到的最大帧率调整。

## 算法解析

介绍EKF四元数姿态解算的教程在:[四元数EKF姿态更新算法](https://zhuanlan.zhihu.com/p/454155643)

## 模块移植

由于历史遗留问题,IMU模块耦合程度高.后续实现BSP_SPI,将bmi088 middleware删除.仅保留BMI088读取的协议和寄存器定义等,单独实现IMU模块.
> 移植已经完成,请转而使用module/BMI088的模块. 当前文件夹将在beta1.2停止支持, 1.5之后删除. INS_Task届时会被放到algorithm中,以提供对不同IMU的兼容.

## Orbit结构体成员地址异常

若展开INS后，q、Gyro、Accel等数组显示同一地址，Roll/Pitch/Yaw也完全相同，
先检查ELF调试信息，不能据此认定BMI088三轴采样或姿态算法故障。
Orbit 1.2.2将DWARF2的成员位置表达式（例如`2 byte block: 23 68`）
用parseInt读成2，导致多个成员都指向结构体首址+2。

Makefile、Makefile.upgrade与CMake的调试构建统一使用`-gdwarf-4`，
让成员位置以常量字节偏移保存。INS_t的Roll/Pitch/Yaw偏移应为104/108/112，
attitude_t中对应偏移为24/28/32。此调整只影响调试信息格式。

修改构建配置后须重新编译所有对象文件；Makefile改动会自动触发该工程全量重编译。
云台普通版与`BENCH_TEST=1`空板版需分别构建。空板版不启动INS，观察陀螺仪须用普通版。
结束旧Orbit会话，确认板上固件与ELF对应，
再以该工程正确的Orbit配置重新启动；若Watch仍保存旧展开结果，删除INS并重新添加。

可用以下命令检查普通固件ELF的结构体成员偏移；空板版将参数替换为build-bench中的ELF：

```text
python tests/orbit_dwarf_regression.py build/<工程名>.elf
```

空板版检查时加`--bench`，只要求保留的attitude_t类型；该版本不提供运行中的INS数据。
脚本仅检查实际ELF中的结构体布局，不连接或控制硬件。

<div align="center">

# Tea-Picking-Dual-Arm

面向双臂茶叶采摘示范与数据采集的 C++17 主从遥操作系统

[![C++17](https://img.shields.io/badge/C%2B%2B-17-00599C?style=flat-square)](https://isocpp.org/)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E?style=flat-square)](https://docs.ros.org/en/humble/)
[![Leader](https://img.shields.io/badge/Leader-HX--10HM-4C8BF5?style=flat-square)](#硬件连接)
[![Slave](https://img.shields.io/badge/Slave-RM65--B-00A896?style=flat-square)](#硬件连接)

</div>

## 项目说明

Tea-Picking-Dual-Arm 用于茶叶采摘主从遥操作与示范数据采集

当前已完成右主臂控制链路，HX-10HM 主臂由 SerialArm-Core 负责状态读取、Software MIT、重力补偿与安全失能，Tea Teleop 负责把主臂关节状态映射到 RM65-B

主链路如下

```text
HX-10HM × 6
    ↓
SerialArm Hiwonder Backend
    ↓
Software MIT + COMPLIANT_DRAG + GRAVITY
    ↓
q_leader
    ↓
Tea Teleop
    ↓
关节方向映射
    ↓
RM65-B Native SDK
```

## 当前能力

- 读取右主臂关节状态
- 读取 RM65-B 关节状态
- 主从关节状态对照
- 右主臂自动归零
- RM65-B 自动归零
- 主从臂依次归零
- 慢速遥操作
- 正常遥操作
- RM65-B 软件停止
- 主臂安全卸力
- SerialArm 重力补偿
- Ctrl+C 中断后停止从臂并失能主臂

## 目录结构

```text
src/
├── SerialArm-Core/
│   ├── src/serial_arm/
│   └── src/robot_supports/
│       ├── hardware/hiwonder/
│       ├── protocol/hiwonder_bus_servo/
│       ├── profiles/
│       └── robots/tea_leader/
├── bringup/
│   └── tea_teleop/
└── hardware/
    └── rm65b/
        ├── rm_bringup/
        ├── rm_driver/
        └── rm_ros_interfaces/
```

HX 通信和控制统一由 SerialArm-Core 提供，不再保留 Tea 工程内另一套 Hiwonder 串口与舵机驱动

`rm_driver` 与 `rm_ros_interfaces` 保留作为睿尔曼官方 SDK 与 ROS 2 资源

## 右主臂 Robot Profile

右主臂使用

```text
tea_leader_right
```

主要配置位于

```text
src/SerialArm-Core/src/robot_supports/robots/tea_leader/description/
├── config/
│   ├── core/right.yaml
│   ├── hardware/right.yaml
│   └── ros2_controllers_right.yaml
└── model/right/
    ├── meshes/
    └── urdf/
```

右主臂当前使用

```text
model_feedforward_mode = GRAVITY
gravity_scale = 0.3
COMPLIANT_DRAG kp = 0
```

`gravity_scale=0.3` 来自当前实机调试结果

该比例用于补偿当前 URDF 动力学模型与实际机构之间的误差，不代表理论模型必须使用 1.0

## 构建

```bash
cd ~/AgroTech/ROS2-Workspace/Tea-Picking-Dual-Arm

source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

如果工作区存在已删除功能包的安装残留，建议先清理工作区

```bash
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 运行

正式入口只有一个

```bash
ros2 run tea_teleop teleop
```

无需额外启动 `serial_arm_terminal`

`serial_arm_terminal` 仅用于 SerialArm-Core 独立调试

主菜单

```text
1. 读取主臂
2. 读取从臂
3. 主从臂读取对照
4. 主臂归零
5. 从臂归零
6. 主从臂归零
7. 慢速遥操作
8. 遥操作
9. 修改运行配置
10. 从臂软件停止
11. 主臂卸力
0. 退出
```

菜单操作不再要求输入额外确认字符串

## 主臂归零

主臂归零不再调用 HX-10HM 原生位置模式

归零过程直接复用 SerialArm-Core

```text
tea_leader_right
    ↓
RIGID_TRACKING
    ↓
当前位置作为初始参考
    ↓
按设定速度连续生成 q_ref
    ↓
q_ref → 0
    ↓
RIGID_TRACKING + 小幅静摩擦补偿
    ↓
Software MIT + Gravity
    ↓
到位后 PWM=0 + Torque OFF
```

这种方式不会依赖舵机退出遥操作后是否仍处于 PWM Open-Loop 模式

默认参数

```text
归零目标        6 轴 q = 0 rad
归零速度        10 deg/s
到位容差        2 deg
超时            30 s
```

归零过程中按 Ctrl+C 会立即进入安全失能流程

## 遥操作

遥操作时 Tea Teleop 自己创建 SerialArm Robot

不要同时启动另一个进程占用相同主臂串口

运行过程

```text
加载 tea_leader_right
    ↓
主臂 activate
    ↓
COMPLIANT_DRAG + GRAVITY
    ↓
独立 100 Hz SerialArm 控制线程
    ↓
缓存最新 q_leader
    ↓
50 Hz 遥操作发送线程
    ↓
映射到 RM65-B
```

慢速遥操作会限制单周期从臂目标变化量

正常遥操作直接使用 RM65-B CANFD 跟随接口

如果主从初始角差较大但仍处于允许范围，正常遥操作会先让从臂低速对齐到当前主臂姿态

从臂网络调用和低速对齐不会阻塞主臂 100 Hz 控制线程，因此主臂重力补偿不会因为 RM65-B 通信等待而触发状态超时

## 关节映射

主臂到从臂的默认方向映射

```text
J1  +1
J2  +1
J3  -1
J4  +1
J5  +1
J6  +1
```

该映射只负责主臂坐标到 RM65-B 坐标的转换

SerialArm Hardware direction、URDF axis 与 Tea Teleop 映射属于不同层级，不应互相替代

## 安全行为

正常遥操作结束或 Ctrl+C

```text
RM65-B stop
    ↓
HX PWM = 0
    ↓
Torque OFF
```

主臂卸力菜单直接通过 SerialArm Hiwonder Backend 建立只读连接

Backend 在连接阶段会确认 Torque OFF，因此不再依赖另一套 HX 驱动

## 双臂扩展

左主臂建议新增独立 Robot Profile

```text
tea_leader_left
tea_leader_right
```

左右主臂分别拥有独立 SerialArm Robot 与独立串口总线

最终双臂遥操作由同一个 Tea Teleop 进程统一管理两个主臂实例与两个 RM65-B，从而统一处理启动、停止、故障和数据同步

## License

见 [LICENSE](LICENSE)

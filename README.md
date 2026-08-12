<div align="center">

# Tea-Picking-Dual-Arm

面向双臂茶叶采摘示范与数据采集的 C++17 主从遥操作系统

[![C++17](https://img.shields.io/badge/C%2B%2B-17-00599C?style=flat-square)](https://isocpp.org/)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E?style=flat-square)](https://docs.ros.org/en/humble/)
[![Leader](https://img.shields.io/badge/Leader-HX--10HM-4C8BF5?style=flat-square)](#主臂配置)
[![Follower](https://img.shields.io/badge/Follower-RM65--B-00A896?style=flat-square)](#从臂与遥操作配置)

</div>

## 项目说明

Tea-Picking-Dual-Arm 用于茶叶采摘主从遥操作与示范数据采集

HX-10HM 主臂由 SerialArm-Core 负责状态读取、Software MIT、重力补偿和安全失能，Tea Teleop 负责读取补偿状态下的主臂关节位置并映射到 RM65-B

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

当前提供左右主臂两个独立 Robot Profile

```text
tea_leader_right
tea_leader_left
```

当前 `tea_teleop` 仍按单主臂到单 RM65-B 的方式运行，默认使用右主臂

## SerialArm-Core 基线

```text
Upstream: https://github.com/Kaede-Rei/SerialArm-Core
Tag: v0.3.0
```

## 当前能力

- 读取主臂关节状态
- 读取 RM65-B 关节状态
- 主从关节状态对照
- 主臂自动归零
- RM65-B 自动归零
- 主从臂依次归零
- 慢速遥操作
- 正常遥操作
- RM65-B 软件停止
- 主臂安全卸力
- SerialArm 重力补偿
- Ctrl+C 中断后停止从臂并失能主臂
- 右主臂 Robot Profile
- 左主臂 Robot Profile
- 可选 HX-10HM 末端回弹 Tool Button
- Tool Button 与 Joint1~Joint6 共用同一 Hiwonder Bus Servo 串口

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
│       └── config/teleop.yaml
└── hardware/
    └── rm65b/
        ├── rm_bringup/
        ├── rm_driver/
        └── rm_ros_interfaces/
```

HX 通信和控制统一由 SerialArm-Core 提供；`rm_driver` 与 `rm_ros_interfaces` 保留作为睿尔曼官方 SDK 与 ROS 2 资源

## 构建

```bash
cd ~/AgroTech/ROS2-Workspace/Tea-Picking-Dual-Arm

source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

如果工作区存在已经删除功能包的安装残留，先清理后重新构建

```bash
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 配置

配置分为两层

```text
主臂 SerialArm 配置
└── src/SerialArm-Core/src/robot_supports/robots/tea_leader/description/config/

从臂与遥操作配置
└── src/bringup/tea_teleop/config/teleop.yaml
```

主臂的串口、舵机 ID、零位、重力补偿和控制参数不要写到 `teleop.yaml`

从臂的 IP、端口和主从映射不要写到 SerialArm Hardware YAML

### 主臂配置

右主臂配置位于

```text
src/SerialArm-Core/src/robot_supports/robots/tea_leader/description/
├── config/
│   ├── core/right.yaml
│   ├── hardware/right.yaml
│   └── ros2_controllers_right.yaml
└── model/right/
    ├── meshes/
    └── urdf/
        ├── tea_leader_right.urdf
        └── tea_leader_right.ros2_control.xacro
```

左主臂配置位于

```text
src/SerialArm-Core/src/robot_supports/robots/tea_leader/description/
├── config/
│   ├── core/left.yaml
│   ├── hardware/left.yaml
│   └── ros2_controllers_left.yaml
└── model/left/
    ├── meshes/
    └── urdf/
        ├── tea_leader_left.urdf
        └── tea_leader_left.ros2_control.xacro
```

主臂 Profile 在这里注册

```text
src/SerialArm-Core/src/robot_supports/profiles/config/robot_profiles.yaml
```

当前 Profile

```text
tea_leader_right
tea_leader_left
```

左右主臂的关节轴符号保持同一约定，只有 J1 的 Z 轴方向相反

```text
                 J1   J2   J3   J4   J5   J6
tea_leader_right -Z   -Z   -Z   +Z   +Z   -Z
tea_leader_left  -Z   -Z   -Z   +Z   +Z   -Z
```

URDF `axis` 表示模型关节正方向，Hardware `direction` 表示执行器原始方向到 SerialArm 执行器方向的映射，两者不要混用

#### 配置主臂串口

右主臂编辑

```text
src/SerialArm-Core/src/robot_supports/robots/tea_leader/description/config/hardware/right.yaml
```

```yaml
hiwonder:
  serial_port: /dev/ttyACM0
  baudrate: 1000000
```

左主臂编辑

```text
src/SerialArm-Core/src/robot_supports/robots/tea_leader/description/config/hardware/left.yaml
```

```yaml
hiwonder:
  serial_port: /dev/ttyACM1
  baudrate: 1000000
```

如果 Linux 的 `/dev/ttyACM0` 与 `/dev/ttyACM1` 枚举顺序不稳定，可以将 `serial_port` 改为对应的 `/dev/serial/by-id/...` 设备路径

#### 配置舵机 ID 与零位

编辑对应侧的 `config/hardware/*.yaml`

```yaml
actuators:
  - name: right_hx10hm_1
    joint_name: right_joint1
    servo_id: 1
    raw_zero: 2048
    direction: 1
```

主要字段

```text
servo_id       Hiwonder 总线 ID
raw_zero       机械零位对应的原始位置值
direction      HX 原始正方向到 SerialArm 执行器正方向的符号
max_effort     Software MIT 力矩软件上限
max_kp/max_kd  Software MIT 增益上限
pwm_limit      PWM Open-Loop 输出上限
```

#### 配置末端回弹 Tool Button

Tool Button 配置位于对应侧的 `config/hardware/*.yaml`，与六轴使用同一个 `serial_port`

```yaml
tool_button:
  enabled: false
  servo_id: 7
  raw_zero: 2048
  direction: 1
  press_threshold_rad: 0.20
  release_threshold_rad: 0.12
  kp: 0.10
  kd: 0.01
  max_effort: 0.08
  positive_gain: 1019.716213
  negative_gain: 1019.716213
  positive_offset: 0.0
  negative_offset: 0.0
  torque_deadband_nm: 0.0
  pwm_limit: 80
```

`enabled: false` 保持原六轴行为不变，安装并确认 ID7、零位和按压方向后再改为 `true`

Tool Button 不属于第 7 个 Robot joint，不改变 `tea_leader_right` / `tea_leader_left` 的 6DOF、MotorBus size、HardwareCapabilities 和 Robot ActuatorState 维度

#### 配置重力补偿与控制参数

编辑对应侧的 `config/core/*.yaml`

```yaml
model:
  gravity: [0.0, 0.0, -9.81]
  gravity_scale: [0.3, 0.3, 0.3, 0.3, 0.3, 0.3]

control:
  runtime:
    ctrl_frequency_hz: 100.0
    write_enabled: true
    model_feedforward_mode: GRAVITY
```

阻抗参数位于

```yaml
control:
  controller:
    rigid_hold:
    rigid_tracking:
    compliant_hold:
    compliant_drag:
    compliant_tracking:
```

遥操作主臂主要使用 `compliant_drag`

主臂归零主要使用 `rigid_tracking`

Safety 相关的软件限幅位于

```yaml
safety_policy:
```

修改 `max_effort_override`、`max_kp_override`、`max_kd_override` 时应同时检查 URDF limit 与 Hardware capability，最终有效限制取各层允许范围中的严格值

### 从臂与遥操作配置

从臂 RM65-B 的连接参数和遥操作参数统一位于

```text
src/bringup/tea_teleop/config/teleop.yaml
```

默认配置

```yaml
leader:
  profile: tea_leader_right

follower:
  ip: 192.168.1.18
  port: 8080
  home_speed_percent: 10

teleop:
  duration_s: -1
  period_ms: 20
  slow_max_step_degree: 1.0
  max_start_error_degree: 30.0
  mapping_direction: [1, 1, -1, 1, 1, 1]

leader_home:
  speed_degree_s: 10.0
  tolerance_degree: 2.0
  timeout_s: 30
```

```text
J1  +1
J2  +1
J3  -1
J4  +1
J5  +1
J6  +1
```

菜单中的“修改运行配置”只修改当前进程中的参数，不会回写 `teleop.yaml`

需要长期保存的参数应直接修改 YAML

也可以使用另一份遥操作配置启动

```bash
ros2 run tea_teleop teleop --config /path/to/teleop.yaml
```

### 主臂独立检查

显示右主臂模型

```bash
ros2 launch serial_arm_ros2_control display.launch.py robot_profile:=tea_leader_right
```

显示左主臂模型

```bash
ros2 launch serial_arm_ros2_control display.launch.py robot_profile:=tea_leader_left
```

主臂硬件、动力学和 Software MIT 的独立调试使用 `serial_arm_terminal`

```bash
serial_arm_terminal --robot-profile tea_leader_right
```

或

```bash
serial_arm_terminal --robot-profile tea_leader_left
```

正式遥操作时不要同时启动另一个进程占用同一主臂串口

## 运行

默认使用 `src/bringup/tea_teleop/config/teleop.yaml`

```bash
ros2 run tea_teleop teleop
```

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

## 主臂归零

主臂归零复用 SerialArm-Core 的 Software MIT 控制链路

```text
当前 q
    ↓
RIGID_TRACKING
    ↓
连续生成 q_ref
    ↓
q_ref → 0
    ↓
重力前馈 + 位置控制 + 静摩擦辅助
    ↓
到位后 PWM=0 + Torque OFF
```

默认归零参数由 `config/teleop.yaml` 的 `leader_home` 配置

```yaml
leader_home:
  speed_degree_s: 10.0
  tolerance_degree: 2.0
  timeout_s: 30
```

## 遥操作

遥操作时 Tea Teleop 自己创建 SerialArm Robot

```text
加载 leader.profile
    ↓
主臂 activate
    ↓
COMPLIANT_DRAG + GRAVITY
    ↓
独立 100 Hz SerialArm 控制线程
    ↓
缓存最新 q_leader
    ↓
按 teleop.period_ms 向 RM65-B 发送目标
```

慢速遥操作会限制单周期从臂目标变化量

正常遥操作直接使用 RM65-B CANFD 跟随接口

如果主从初始角差较大但仍处于允许范围，遥操作会先让从臂低速对齐到当前主臂姿态

## 安全行为

正常遥操作结束或 Ctrl+C

```text
RM65-B stop
    ↓
HX PWM = 0
    ↓
Torque OFF
```

## License

见 [LICENSE](LICENSE)

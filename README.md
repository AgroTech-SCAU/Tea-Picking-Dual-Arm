<div align="center">

# Tea-Picking-Dual-Arm

C++17 master-slave teleoperation system for dual-arm tea-picking data collection

面向双臂茶叶采摘示范与数据采集的 C++17 主从遥操作系统

[![C++17](https://img.shields.io/badge/C%2B%2B-17-00599C?style=flat-square)](https://isocpp.org/)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E?style=flat-square)](https://docs.ros.org/en/humble/)
[![Master](https://img.shields.io/badge/Master-HX--10HM-4C8BF5?style=flat-square)](#硬件连接)
[![Slave](https://img.shields.io/badge/Slave-RM65--B-00A896?style=flat-square)](#硬件连接)

</div>

## 项目简介

Tea-Picking-Dual-Arm 面向双臂茶叶采摘示范操作与数据采集，当前版本首先完成单侧 6-DOF 主从遥操作链路，并以同一套接口扩展到双臂系统

当前主臂由 6 个幻尔 HX-10HM 总线舵机构成，主臂只负责获取操作者关节动作并在归零阶段短时主动运动；从臂使用睿尔曼 RM65-B，通过 RealMan 原生 C++ SDK 接收六关节目标角度

项目当前运行链路不依赖 ROS topic / service；ROS 2 Humble 与 ament/colcon 主要用于工作区构建和安装组织

项目边界是：

- HX-10HM 串口通信与协议解析
- 主臂六关节角度读取与自动归零
- RM65-B 原生 SDK 封装
- 主从关节映射
- 低跟随 / 高跟随 Teleop
- 启动安全检查与归零流程
- 菜单式运行配置、状态读取与异常恢复

## 核心设计

```mermaid
flowchart LR
    Operator["Operator"] --> Master["HX-10HM Master Arm ×6"]
    Master --> Serial["SerialPort\n1 Mbps / 8N1"]
    Serial --> HX["Hx10hm Driver"]
    HX --> Mapper["Joint Mapping\n2048 -> 0 deg"]
    Mapper --> Teleop["TeaTeleop"]
    Teleop --> RM["Rm65bBringup"]
    RM --> SDK["RealMan Native C++ SDK"]
    SDK --> Slave["RM65-B Slave Arm"]
```

主链路保持为：

```text
HX-10HM ×6
  ↓
SerialPort
  ↓
Hx10hm
  ↓
raw position -> degree -> direction mapping
  ↓
TeaTeleop
  ↓
Rm65bBringup
  ↓
RealMan Native SDK
  ↓
RM65-B
```

`rm_driver` 与 `rm_ros_interfaces` 当前保留作为 RealMan 官方 ROS 2 参考代码与 SDK 资源来源，正式 Teleop 不通过 ROS topic 控制 RM65-B

## 当前能力

| 能力 | 状态 |
| --- | --- |
| C++17 Teleop | 已实现 |
| HX-10HM 6-DOF 主臂读取 | 已实现 |
| HX-10HM 自动归零 | 已实现，Torque ACK + Sync Write + 到位确认 + Torque OFF |
| RM65-B 六关节读取 | 已实现 |
| RM65-B 规划归零 | 已实现 |
| 主从角度映射 | 已实现，支持逐关节 `+1 / -1` 方向配置 |
| 慢速 Teleop | 已实现，上层单周期限速 + `follow=false` |
| 全速 Teleop | 已实现，无上层单周期限速 + `follow=true` |
| 主从启动误差检查 | 已实现，默认最大 `30 deg` |
| 主臂异常卸力 | 已实现，可从菜单独立执行 |
| 运行时参数修改 | 已实现，菜单内修改，本次进程有效 |
| 双臂同步 Teleop | Planned |
| 示范数据记录 | Planned |
| 主臂重力补偿 | Planned |

## SerialArm-Core 基线

- Upstream: https://github.com/Kaede-Rei/SerialArm-Core
- Base: `v0.2.0`
- `src/SerialArm-Core` 由本仓库直接追踪，用于 HX-10HM Hardware Backend 的项目内二次开发，不是 Git submodule

## 仓库结构

```text
src/
├── bringup/
│   └── tea_teleop/
│       ├── include/tea_teleop/
│       │   └── teleop.hpp
│       └── src/
│           └── teleop.cpp
└── hardware/
    ├── hiwonder/
    │   ├── serial_port/           # 通用 Linux/POSIX 串口层
    │   └── hiwonder_driver/       # HX-10HM 协议、读取、归零与 Torque 控制
    └── rm65b/
        ├── rm_bringup/            # RM65-B 原生 C++ SDK 最小封装
        ├── rm_driver/             # RealMan 官方 ROS 2 driver 与 Native SDK 资源
        └── rm_ros_interfaces/     # RealMan 官方 ROS 2 interfaces
```

分层职责保持为：

```text
Transport          SerialPort
Protocol / Device  Hx10hm
Robot SDK Wrapper  Rm65bBringup
Application        TeaTeleop
```

## 硬件连接

### 主臂

| 项目 | 默认配置 |
| --- | --- |
| Servo | HX-10HM ×6 |
| Servo ID | `1 ~ 6` |
| Serial device | `/dev/ttyACM0` |
| Baud rate | `1,000,000` |
| Serial format | `8N1` |
| Position resolution | 12-bit / `0 ~ 4095` |
| Project center | `2048 -> 0 deg` |

HX-10HM 使用独立供电，并通过调试板 / USB 与上位机连接

如果当前用户没有串口权限：

```bash
sudo usermod -aG dialout $USER
```

重新登录后再运行程序

### 从臂

| 项目 | 默认配置 |
| --- | --- |
| Robot | RealMan RM65-B |
| Controller IP | `192.168.1.18` |
| TCP port | `8080` |
| PC Ethernet | 推荐 `192.168.1.100/24` |
| Subnet mask | `255.255.255.0` |

RM65-B 控制器默认有线地址为 `192.168.1.18`，TCP 控制端口为 `8080`；PC 有线网口必须配置到同一 IPv4 网段，本项目推荐固定为 `192.168.1.100/24`

#### Ubuntu 有线网络配置

先确认电脑实际的 Ethernet 设备名：

```bash
nmcli device status
```

例如本项目当前测试主机使用：

```text
enp4s0  ethernet
```

设备名以当前机器实际输出为准，不要直接照抄 `enp4s0`

先确认物理链路：

```bash
sudo ethtool enp4s0 | grep "Link detected"
```

正常应输出：

```text
Link detected: yes
```

如果为 `no`，优先检查控制器网口、网线和电脑网口，不需要继续排查 SDK

本项目的 RM65-B 直连链路采用静态 IPv4，不依赖 `Automatic (DHCP)`；如果有线连接长期显示 `connecting` 后又变回 `disconnected / connect off`，优先检查 PC 是否没有获得 `192.168.1.x` IPv4 地址

查看已有 NetworkManager 连接：

```bash
nmcli connection show
```

假设有线连接名称为 `有线连接 1`，绑定网卡为 `enp4s0`，可直接配置静态 IPv4：

```bash
sudo nmcli connection modify "有线连接 1" \
  connection.interface-name enp4s0 \
  ipv4.method manual \
  ipv4.addresses 192.168.1.100/24 \
  ipv4.gateway "" \
  ipv4.dns "" \
  ipv6.method disabled

sudo nmcli connection up "有线连接 1"
```

本机直连 RM65-B 时不需要额外配置 Gateway 或 DNS

配置后确认有线网卡已经获得 IPv4：

```bash
ip -br addr show enp4s0
```

期望包含：

```text
enp4s0  UP  192.168.1.100/24
```

只有 `fe80::...` 表示当前只有 IPv6 link-local 地址，仍未完成 RM65-B 所需的 IPv4 配置

#### 连通性检查

首先确认到机械臂的路由确实走有线网卡：

```bash
ip route get 192.168.1.18
```

期望类似：

```text
192.168.1.18 dev enp4s0 src 192.168.1.100
```

然后测试 IP 层：

```bash
ping -c 4 192.168.1.18
```

Ping 正常后再测试 SDK 使用的 TCP 端口：

```bash
nc -vz 192.168.1.18 8080
```

正常情况下应看到 `8080` 连接成功；随后再启动 `tea_teleop`

如果未安装 `nc`：

```bash
sudo apt install netcat-openbsd
```

#### Wi-Fi 与有线网段冲突

电脑可以同时保持 Wi-Fi 联网，但如果 Wi-Fi 也处于 `192.168.1.0/24` 网段，Linux 可能把访问 `192.168.1.18` 的流量错误地发往 Wi-Fi

始终以：

```bash
ip route get 192.168.1.18
```

确认目标路由为 Ethernet 设备；如果显示 `dev wlo1` 或其他 Wi-Fi 设备，可临时关闭 Wi-Fi 验证：

```bash
nmcli radio wifi off
```

测试结束后恢复：

```bash
nmcli radio wifi on
```

#### RM65-B 网络故障快速判断

| 检查结果 | 说明 | 优先处理 |
| --- | --- | --- |
| `Link detected: no` | Ethernet 物理链路未建立 | 检查网线、控制器网口、电脑网口 |
| `Link detected: yes`，但没有 `192.168.1.x` | PC IPv4 未配置 | 设置静态 `192.168.1.100/24` |
| `ip route get` 走 Wi-Fi | 路由冲突 | 修正路由或临时关闭 Wi-Fi |
| Ping 不通 | IP 层未打通 | 检查 PC 网段和机械臂实际 IP |
| Ping 通但 `8080` 不通 | TCP 服务不可达 | 检查控制器状态、端口和机械臂 IP |
| Ping 与 `8080` 都正常但 SDK 失败 | 网络基本正常 | 再检查 RealMan SDK 调用和程序参数 |

若程序出现：

```text
[rm_create_robot_arm] socket connect err!
[FAILED] rm_create_robot_arm failed: 192.168.1.18:8080
```

不要先修改 Teleop 控制逻辑，应按上表从物理链路 → IPv4 → 路由 → Ping → TCP 8080 逐层检查

如果机械臂有线 IP 已被修改，则 README 中的 `192.168.1.18` 不再适用；应先通过示教器或已有连接确认控制器当前实际 IP，再同步修改 Teleop 配置

睿尔曼官方网络配置参考：<https://develop.realman-robotics.com/en/robot/teachingPendant/setting/>

## Quick Start

### 1. 构建

当前工作区使用 ROS 2 Humble + colcon 组织构建：

```bash
source /opt/ros/humble/setup.bash

colcon build --symlink-install
source install/setup.bash
```

### 2. 运行

通过 ROS 2 package 安装入口运行：

```bash
ros2 run tea_teleop teleop
```

也可以直接运行安装后的可执行文件：

```bash
./install/tea_teleop/lib/tea_teleop/teleop
```

程序默认使用：

```text
RM65-B IP : 192.168.1.18
HX serial : /dev/ttyACM0
```

也可以从命令行覆盖：

```bash
./install/tea_teleop/lib/tea_teleop/teleop \
  192.168.1.18 \
  /dev/ttyACM0
```

参数顺序为：

```text
argv[1] = RM65-B IP
argv[2] = HX-10HM serial device
```

## Teleop 菜单

启动后进入统一交互菜单：

```text
 1. 读取主臂
 2. 读取从臂
 3. 主从臂读取对照
 4. 主臂慢速归零
 5. 从臂慢速归零
 6. 主从臂慢速归零
 7. 慢速 Teleop
 8. 全速 Teleop
 9. 修改运行配置
10. RM65-B 软件停止
11. 主臂卸力 / 异常恢复
 0. 退出
```

主臂、从臂和主从对照读取均支持：

- 单次读取打印
- 持续读取打印

持续读取和 Teleop 运行期间使用 `Ctrl+C` 返回主菜单

## 默认配置

```text
serial_device              = /dev/ttyACM0
rm_ip                      = 192.168.1.18
rm_port                    = 8080
teleop_duration_s          = -1
read_print_period_ms       = 100
teleop_period_ms           = 20
slow_max_step_degree       = 1.0
max_start_error_degree     = 30.0
master_home_tolerance_deg  = 2.0
master_home_speed          = 100 steps/s
master_home_timeout_s      = 30 s
slave_home_speed_percent   = 10%
mapping_direction          = [1, 1, -1, 1, 1, 1]
```

`teleop_duration_s = -1` 表示持续运行直到用户中断

所有配置都可以在菜单中修改，当前版本只影响本次运行，不写回配置文件

## 主从映射

HX-10HM 使用 12-bit 磁编码器，当前项目将 `2048` 定义为主臂关节零位

```text
raw = 2048  -> 0 deg
raw < 2048  -> negative degree
raw > 2048  -> positive degree
```

基础角度换算为：

```text
master_degree = (raw - 2048) * 360 / 4096
```

最终发送到从臂前再应用逐关节方向：

```text
slave_target[i] = direction[i] * master_degree[i]
```

当前默认方向：

```text
[+1, +1, -1, +1, +1, +1]
```

方向只用于补偿主从机械安装方向，不改变 `2048 -> 0 deg` 的主臂零位定义

## Teleop 模式

### 慢速 Teleop

```text
Master Target
  ↓
单周期最大角度变化限制
  ↓
rm_movej_canfd(..., follow=false)
```

默认每周期最大变化 `1 deg`，用于首次联调、映射验证和低速测试

### 全速 Teleop

```text
Master Target
  ↓
无上层单周期角度限速
  ↓
rm_movej_canfd(..., follow=true)
```

全速模式使用 RM65-B CANFD 高跟随，程序会将实际控制周期限制到不大于 `10 ms`

如果主从初始误差仍在允许范围内但大于全速启动对齐阈值，程序会先使用 `movej` 低速将从臂对齐到当前主臂映射姿态，再进入高跟随

## 自动归零

### 主臂

主臂自动归零不是直接连续写六个舵机，而是使用确认式事务：

```text
读取当前位置
  ↓
ID1~6 Torque ON
每个 WRITE DATA 等待并验证 ACK
  ↓
SYNC WRITE -> raw 2048
  ↓
持续 READ DATA 确认真实位置
  ↓
进入归零容差
  ↓
ID1~6 Torque OFF
每个 WRITE DATA 等待并验证 ACK
```

SYNC WRITE 使用广播 ID，不等待状态包；到位判断始终使用读取到的真实当前位置

如果主臂归零被中断或发生异常，程序会尽可能逐个执行 Torque OFF

如果仍无法确认全部卸力，应停止 Teleop 并检查主臂，必要时断电重启后再继续

### 从臂

RM65-B 归零使用控制器自身关节空间规划：

```text
rm_movej([0, 0, 0, 0, 0, 0], low_speed)
```

归零和启动前对齐不使用 CANFD 透传

## 安全策略

当前保留以下运行约束：

- Teleop 启动前可选择主从臂自动慢速归零
- 不归零时，任一关节主从初始角差超过默认 `30 deg` 直接拒绝启动
- 主臂自动归零完成后必须确认 Torque OFF
- 慢速 Teleop 默认限制单周期最大变化 `1 deg`
- 全速 Teleop 启动前进行主从姿态检查与必要的低速对齐
- 通信异常时尝试停止 RM65-B 当前轨迹
- `Ctrl+C` 中断当前持续任务并返回菜单
- 菜单提供独立 RM65-B 软件停止
- 菜单提供独立 HX-10HM 主臂卸力 / 异常恢复

首次实机建议先完成单关节方向验证，再逐步进入六关节全速遥操作

## 许可证

以仓库当前 LICENSE 为准

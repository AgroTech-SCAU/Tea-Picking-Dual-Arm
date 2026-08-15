# HX-10HM Software MIT 验证工具

`serial_arm_hiwonder_calibration` 用于验证 HX-10HM 在 PWM 开环模式下的软件 MIT 控制链路

默认标称配置安装到：

```text
share/serial_arm_hardware_hiwonder/config/hx10hm_nominal.yaml
```

该配置中的力矩、速度和 PWM 参数主要来自厂家参数换算与保守工程限制，不代表主臂最终机械限位或精确力矩标定结果

## 安全要求

- 测试时必须支撑机械臂，并保留可立即断电的物理手段
- 首先使用 `read` 模式确认位置、速度、故障、电流、负载、温度和电压
- `tau` 与 `damping` 每次只测试一个关节
- 所有会使能舵机的模式都必须显式传入 `--confirm-motion`
- 工具硬限制为 `|tau| <= 0.2941995 N·m`、`|PWM| <= 300`
- SIGINT、SIGTERM、通信错误和诊断阈值触发后会尽力执行零 PWM、Torque OFF 和资源清理

## 位置反馈说明

HX-10HM 的 `0x1F` 为位置校正寄存器，范围 `[-2047, 2047]`，BIT11 为方向位

位置校正会平移 `0x38` 当前坐标，例如校正为 `+1825` 时，PWM Open-Loop 下单圈反馈可能表现为 `-1825..2270`

Backend 在 `connect()` 时逐轴读取 `0x1F`，再用 `wrap4096(position_raw + position_calibration)` 恢复统一的 `0..4095` 单圈编码器坐标

因此不需要在 YAML 重复维护位置校正值，也不需要针对某个舵机写特殊分支

`raw_zero` 仍填写机械零位在 HX 校正后 `0x38` 坐标中的读数，通常为 `2048`，Backend 会对零位应用同一位置校正后再计算关节角

## 常用命令

```bash
serial_arm_hiwonder_calibration \
  --config /path/to/hx10hm_nominal.yaml \
  --mode read --duration 10 --period-ms 100

serial_arm_hiwonder_calibration \
  --config /path/to/hx10hm_nominal.yaml \
  --mode tau --joint 2 --tau 0.02 --duration 2 \
  --confirm-motion --csv /tmp/hx_tau.csv

serial_arm_hiwonder_calibration \
  --config /path/to/hx10hm_nominal.yaml \
  --mode damping --joint 2 --kd 0.01 --duration 2 \
  --confirm-motion --csv /tmp/hx_damping.csv

serial_arm_hiwonder_calibration \
  --config /path/to/hx10hm_nominal.yaml \
  --mode benchmark --duration 10 --period-ms 10 \
  --confirm-motion --csv /tmp/hx_benchmark.csv
```

阻尼测试建议从 `kd=0.01` 开始，确认方向正确后再考虑 `0.02` 或 `0.05`

如果出现助推、自激或振荡，应立即停止测试

CSV 字段：

```text
timestamp_s,joint,q_rad,dq_rad_s,position_raw,position_calibration,encoder_raw,current_ma,load_raw,tau_cmd_nm,pwm_cmd,kp,kd,online,enabled,fault
```

## 通信稳定性

实时控制只依赖 `0x38` 起始的状态块，电流 `0x45` 当前仅用于诊断，不参与 Software MIT 或 Safety

Hiwonder Backend 因此采用以下策略：

- 控制周期每帧执行一次六轴状态 `SYNC READ`
- 电流读取通过 `current_read_divider` 降频，100 Hz 下默认每 10 帧更新一次
- 实时状态出现瞬态 `TIMEOUT`、校验错误或报文异常时，按 `read_retry_count` 立即重试
- 电流诊断读取失败不会直接使 Robot 进入 FAULT，下一次诊断周期继续更新缓存
- 每次新事务仍会清理输入缓冲区，避免延迟应答污染下一帧

推荐初值：

```yaml
read_timeout_ms: 8
read_retry_count: 1
current_read_divider: 10
```

这些参数用于吸收 USB CDC、线程调度和舵机应答的短时抖动，不应被用来掩盖持续掉线、供电异常或物理总线故障


## Tea Leader Tool Button

Tea Leader 可以在 Joint1~Joint6 之外增加一只 HX-10HM 作为可按压回弹的 Tool Button

Tool Button 不是第 7 个 SerialArm Robot joint，不改变 MotorBus 六轴 Contract

Hardware YAML 中使用独立配置块

```yaml
hiwonder:
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

`enabled=false` 时原六轴行为保持不变

启用后 Joint1~Joint6 与 Tool Button 仍由同一个 `Hx10hmMotorBus`、同一个 `HiwonderBusServo` 和同一个 `SerialPort` 管理

实时读取使用同一次多 ID `SYNC READ`，PWM 输出使用同一次 `SYNC WRITE`

Tool Button 状态通过 `Hx10hmMotorBus::tool_button_state()` 读取

```cpp
const auto state = bus.tool_button_state();

state.pos_rad;
state.vel_rad_s;
state.pressed;
state.online;
```

只读真机检查

```bash
serial_arm_hiwonder_tool_button --config /path/to/hardware/right.yaml
```

低扭矩回弹测试

```bash
serial_arm_hiwonder_tool_button \
  --config /path/to/hardware/right.yaml \
  --spring --confirm-spring
```

`--spring` 会让 Joint1~Joint6 保持 Torque Enable + zero PWM，同时 Tool Button 使用 YAML 中的低 `kp`、`kd`、`max_effort` 和 `pwm_limit` 执行零位虚拟弹簧

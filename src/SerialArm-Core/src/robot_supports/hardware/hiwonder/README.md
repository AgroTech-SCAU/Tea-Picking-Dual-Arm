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

HX-10HM 的 `0x38` 当前位置信息按 16 位有符号绝对位置处理

厂家资料中单圈 `0..4095` 对应 `0..360°`，绝对位置相关范围可扩展到约 `-30719..30719` 步

因此 Backend 不再把 `position_raw > 4095` 直接判定为非法状态，避免关节跨过单圈边界后误报 `INVALID_STATE`

当前 `raw_zero=2048` 与执行器级 `[-π, π)` 配置仍保留，用于主臂零位和初始安全范围定义

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
timestamp_s,joint,q_rad,dq_rad_s,position_raw,current_ma,load_raw,tau_cmd_nm,pwm_cmd,kp,kd,online,enabled,fault
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


# HX-10HM Software MIT validation

`serial_arm_hiwonder_calibration` 是用于软件MIT的有界验证工具，在 HX-10HM PWM 开环上运行

已安装的标称配置是 `share/serial_arm_hardware_hiwonder/
config/hx10hm_nominal.yaml`；其六轴数值是理论或保守的执行器级参数，而不是主臂的机械极限

## Safety

- 保持手臂支撑并提供物理紧急停止
- 首先运行 `read` 并验证位置、速度、故障、电流、负载、温度和电压
- 每次只测试一个关节的 `tau` 和 `damping`
- 每种 Torque ON 模式都需要 `confirm motion`
- 工具会拒绝超过 `0 2941995 N m` 或 `300 PWM` 的配置
- SIGINT、SIGTERM、错误、故障及配置的诊断阈值会导致尽最大努力设为零 PWM、Torque OFF，并进行清理

## Commands

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

对于阻尼，从 `kd=0 01` 开始，然后在检查符号后才考虑 `0 02` 和 `0 05`。如果运动变得自激或振荡，立即停止；基准发送六轴零 MIT 命令，并报告 `样本数`、平均值、最小值、最大值、周期时间、有效频率、超时计数和超限计数

CSV 字段包括：

```text
timestamp_s, joint, q_rad, dq_rad_s, current_ma, load_raw, tau_cmd_nm, pwm_cmd, kp, kd, online, enabled, fault
```

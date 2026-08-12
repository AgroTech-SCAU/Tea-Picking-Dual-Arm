#pragma once

#include "serial_arm/hardware/motor_bus.hpp"
#include "serial_arm/transport/serial_port.hpp"
#include "serial_arm_protocol_hiwonder_bus_servo/bus.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace serial_arm {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

/**
 * @brief HX-10HM 速度反馈编码
 */
enum class HiwonderVelocityEncoding {
    Bit15SignMagnitude,   ///< BIT15 为方向位，BIT0~14 为步/秒幅值
};

/**
 * @brief 单个 HX-10HM 执行器配置
 */
struct HiwonderActuatorCfg {
    std::string name;              ///< 执行器名称
    std::string joint_name;        ///< 关联关节名称
    std::uint8_t servo_id{ 0 };    ///< 舵机 ID
    std::uint16_t raw_zero{ 2048 }; ///< 关节零位在 HX 校正后 0x38 坐标中的位置
    int direction{ 1 };            ///< 机械关节方向，只允许 +1 或 -1
    double min_pos{ 0.0 };         ///< 最小位置，rad
    double max_pos{ 0.0 };         ///< 最大位置，rad
    double max_vel{ 0.0 };         ///< 最大速度，rad/s
    double max_effort{ 0.0 };      ///< 软件安全力矩上限，N·m
    double max_kp{ 0.0 };          ///< 软件 MIT 最大 kp，N·m/rad
    double max_kd{ 0.0 };          ///< 软件 MIT 最大 kd，N·m·s/rad
    double positive_gain{ 0.0 };   ///< 正向力矩到 PWM 增益
    double negative_gain{ 0.0 };   ///< 负向力矩到 PWM 增益
    double positive_offset{ 0.0 }; ///< 正向 PWM 启动偏置
    double negative_offset{ 0.0 }; ///< 负向 PWM 启动偏置
    double torque_deadband_nm{ 0.0 }; ///< 力矩死区，N·m
    std::int16_t pwm_limit{ 0 };   ///< 单轴 PWM 安全限幅，最大 1000
};

/**
 * @brief HX-10HM 总线配置
 */
struct HiwonderBusCfg {
    std::string serial_port{ "/dev/ttyACM0" };      ///< 串口设备路径
    int baudrate{ 1000000 };                         ///< 串口波特率
    std::chrono::milliseconds read_timeout{ 5 };    ///< 单应答读取超时
    std::chrono::milliseconds write_timeout{ 20 };  ///< 写入及 ACK 超时
    std::chrono::milliseconds feedback_timeout{ 50 }; ///< 状态新鲜度上限
    std::size_t startup_read_cycles{ 3 };            ///< 激活后状态确认次数
    std::size_t read_retry_count{ 1 };               ///< 实时状态 SYNC READ 瞬态失败重试次数
    std::size_t current_read_divider{ 10 };          ///< 电流诊断读取分频，100 Hz 控制时 10 表示约 10 Hz
    bool restore_position_mode_on_deactivate{ false }; ///< 停用时是否恢复位置模式
    HiwonderVelocityEncoding velocity_encoding{ HiwonderVelocityEncoding::Bit15SignMagnitude }; ///< 速度编码
    std::string torque_feedback_mode{ "unavailable_zero" }; ///< 未标定力矩反馈策略
    std::vector<HiwonderActuatorCfg> actuators;      ///< 固定六个执行器配置
};

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

/**
 * @brief HX-10HM SerialArm MotorBus Hardware Backend
 */
class Hx10hmMotorBus final : public MotorBus {
public:
    /**
     * @brief 析构并安全释放串口资源
     */
    ~Hx10hmMotorBus() override { cleanup(); }

    /**
     * @brief 从 YAML 读取并校验 Backend 配置
     * @param config_path Backend 配置文件路径
     * @return 成功时返回空结果，否则返回错误码
     */
    tl::expected<void, MotorBusErr> configure(const std::string& config_path) override;

    /**
     * @brief 使用结构化参数配置 Backend
     * @param cfg Backend 配置
     * @return 成功时返回空结果，否则返回错误码
     */
    tl::expected<void, MotorBusErr> configure(const HiwonderBusCfg& cfg);

    /**
     * @brief 打开串口、确保 Torque OFF 并验证六轴通信
     * @return 成功时返回空结果，否则返回错误码
     */
    tl::expected<void, MotorBusErr> connect() override;

    /**
     * @brief 同步读取六轴状态并转换为 Hardware Contract 单位
     * @return 成功时返回 ActuatorState，否则返回错误码
     */
    tl::expected<ActuatorState, MotorBusErr> read() override;

    /**
     * @brief 以零 PWM 安全切换到 PWM Open-Loop 并使能六轴
     * @return 成功时返回空结果，否则返回错误码
     */
    tl::expected<void, MotorBusErr> activate() override;

    /**
     * @brief 执行 Software MIT、Torque -> PWM 并同步下发六轴命令
     * @param cmd Core 输出的 MIT 五元组命令
     * @return 成功时返回空结果，否则返回错误码
     */
    tl::expected<void, MotorBusErr> write(const ActuatorCtrlCmd& cmd) override;

    /**
     * @brief 使用六轴 SYNC WRITE 将 PWM 安全归零
     * @return 成功时返回空结果，否则返回错误码
     */
    tl::expected<void, MotorBusErr> stop() override;

    /**
     * @brief 零 PWM、Torque OFF 并按配置恢复位置模式
     * @return 成功时返回空结果，否则返回错误码
     */
    tl::expected<void, MotorBusErr> deactivate() override;

    /**
     * @brief 执行零 PWM、Torque OFF、flush 并重新验证通信与模式
     * @return 成功时返回空结果，否则返回错误码
     */
    tl::expected<void, MotorBusErr> recover() override;

    /**
     * @brief 释放资源，可重复调用且不抛异常
     */
    void cleanup() noexcept override;

    /**
     * @brief 返回执行器数量
     * @return 已配置的执行器数量
     */
    std::size_t size() const noexcept override;

    /**
     * @brief 返回配置产生的 HardwareCapabilities
     * @return HardwareCapabilities 只读引用
     */
    const HardwareCapabilities& capabilities() const noexcept override;

    /**
     * @brief 验证 MIT 命令维度、有限值与能力范围
     * @param cmd 待验证命令
     * @return 成功时返回空结果，否则返回 INVALID_CMD
     */
    tl::expected<void, MotorBusErr> validate_command(const ActuatorCtrlCmd& cmd) const;

    /**
     * @brief 计算标量 Software MIT 力矩
     * @param tor_ff 前馈力矩，N·m
     * @param kp 位置刚度，N·m/rad
     * @param pos_desired 目标位置，rad
     * @param pos_measured 实测位置，rad
     * @param kd 速度阻尼，N·m·s/rad
     * @param vel_desired 目标速度，rad/s
     * @param vel_measured 实测速度，rad/s
     * @return 未限幅的 MIT 力矩，N·m
     */
    static double calculate_mit_torque(
        double tor_ff,
        double kp,
        double pos_desired,
        double pos_measured,
        double kd,
        double vel_desired,
        double vel_measured) noexcept;

    /**
     * @brief 将单轴力矩执行限幅、死区和分段线性 PWM 映射
     * @param index 执行器索引
     * @param tau_cmd 未限幅 MIT 力矩，N·m
     * @return 成功时返回有符号 PWM，否则返回错误码
     */
    tl::expected<std::int16_t, MotorBusErr> torque_to_pwm(
        std::size_t index,
        double tau_cmd) const;

    /**
     * @brief 使用指定状态和反馈年龄构造六轴 PWM 命令
     * @param cmd Core 输出的 MIT 命令
     * @param state 用于 MIT 计算的执行器状态
     * @param feedback_age 状态年龄
     * @return 成功时返回按配置舵机顺序排列的 PWM 命令
     */
    tl::expected<std::vector<protocol::hiwonder_bus_servo::PwmCommand>, MotorBusErr>
    build_pwm_commands(
        const ActuatorCtrlCmd& cmd,
        const ActuatorState& state,
        std::chrono::milliseconds feedback_age) const;

    /**
     * @brief 使用舵机内部位置校正恢复 0~4095 单圈编码器坐标
     * @param reported_raw 0x38 当前舵机报告位置
     * @param position_calibration 0x1F 位置校正参数
     * @return 归一化后的单圈位置 [0, 4095]
     */
    static std::uint16_t normalize_position_raw(
        std::int32_t reported_raw,
        std::int16_t position_calibration) noexcept;

    /**
     * @brief 将归一化单圈位置转换为关节弧度
     * @param encoder_raw 归一化单圈位置 [0, 4095]
     * @param encoder_zero 关节零位对应的归一化单圈位置
     * @param direction 机械方向 +1 或 -1
     * @return 关节位置，rad，使用最短环形差值避免 4095/0 回绕跳变
     */
    static double raw_position_to_rad(
        std::uint16_t encoder_raw,
        std::uint16_t encoder_zero,
        int direction) noexcept;

    /**
     * @brief 将 BIT15 方向位的原始速度转换为 rad/s
     * @param raw 原始速度字
     * @param direction 机械方向 +1 或 -1
     * @return 关节速度，rad/s
     */
    static double raw_velocity_to_rad_per_second(
        std::uint16_t raw,
        int direction) noexcept;

    /**
     * @brief 返回最近一次 read() 保存的 HX 原始诊断状态
     * @return 按配置舵机顺序排列的只读原始状态
     */
    const std::vector<protocol::hiwonder_bus_servo::RawState>&
    raw_diagnostics() const noexcept;

private:
    /**
     * @brief 验证 Backend 配置
     * @param cfg 待验证配置
     * @return 成功时返回空结果，否则返回 INVALID_CFG
     */
    tl::expected<void, MotorBusErr> validate_cfg(const HiwonderBusCfg& cfg) const;

    /**
     * @brief 获取按配置顺序排列的舵机 ID
     * @return 舵机 ID 列表
     */
    std::vector<std::uint8_t> servo_ids() const;

    /**
     * @brief 同步下发六轴零 PWM
     * @return 成功时返回空结果，否则返回写入错误
     */
    tl::expected<void, MotorBusErr> write_zero_pwm();

    /**
     * @brief 逐轴设置 Torque Enable 并消费 ACK
     * @param enable true 为上力，false 为卸力
     * @return 成功时返回空结果，否则返回错误码
     */
    tl::expected<void, MotorBusErr> set_all_torque(bool enable);

    /**
     * @brief 仅在当前值不同时写入 NVS 运行模式
     * @param id 舵机 ID
     * @param mode 目标模式
     * @return 成功时返回空结果，否则返回错误码
     */
    tl::expected<void, MotorBusErr> ensure_run_mode(
        std::uint8_t id,
        protocol::hiwonder_bus_servo::RunMode mode);

    /**
     * @brief 将协议读取错误转换为 MotorBusErr
     * @param error 协议错误
     * @return MotorBus 读取错误
     */
    static MotorBusErr map_read_error(protocol::hiwonder_bus_servo::Err error) noexcept;

    /**
     * @brief 将协议写入错误转换为 MotorBusErr
     * @param error 协议错误
     * @return MotorBus 写入错误
     */
    static MotorBusErr map_write_error(protocol::hiwonder_bus_servo::Err error) noexcept;

    /**
     * @brief 尽力零 PWM 并 Torque OFF
     */
    void safe_disable_noexcept() noexcept;

    /**
     * @brief 释放连接资源
     * @param keep_config 是否保留已校验配置
     */
    void release_connection_noexcept(bool keep_config) noexcept;

private:
    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;

private:
    HiwonderBusCfg cfg_;                                      ///< Backend 配置
    transport::SerialPort serial_;                            ///< SerialArm-Core 串口
    std::unique_ptr<protocol::hiwonder_bus_servo::HiwonderBusServo> protocol_; ///< HX 协议层
    HardwareCapabilities capabilities_;                      ///< Core 所需执行器能力
    ActuatorState last_state_;                               ///< 最近一次有效状态
    std::vector<protocol::hiwonder_bus_servo::RawState> raw_states_; ///< 最近原始反馈
    std::vector<std::uint8_t> online_;                       ///< 执行器在线状态
    std::vector<std::uint8_t> enabled_;                      ///< 执行器使能状态
    std::vector<std::uint16_t> current_cache_ma_;            ///< 低频电流诊断缓存，控制环不依赖该值
    std::vector<std::int16_t> position_calibration_raw_;     ///< connect() 从 0x1F 读取的位置校正参数
    std::vector<std::uint16_t> encoder_zero_raw_;            ///< raw_zero 加位置校正后恢复的单圈零位
    std::size_t read_cycle_count_{ 0 };                      ///< read() 调用计数，用于诊断分频
    TimePoint last_feedback_time_{};                         ///< 最近六轴同步反馈时间
    bool configured_{ false };                               ///< 是否已配置
    bool connected_{ false };                                ///< 是否已连接
    bool active_{ false };                                   ///< 是否已激活 PWM 模式
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //

} // namespace serial_arm

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
    std::uint8_t servo_id{ 0 };    ///< 舅机 ID
    std::uint16_t raw_zero{ 2048 }; ///< 关节零位对应的原始位置
    int direction{ 1 };            ///< 机械关节方向，只允许 +1 或 -1
    double min_pos{ 0.0 };         ///< 最小位置，rad
    double max_pos{ 0.0 };         ///< 最大位置，rad
    double max_vel{ 0.0 };         ///< 最大速度，rad/s
    double max_effort{ 0.0 };      ///< 软件安全力矩上限，N·m
    double max_kp{ 0.0 };          ///< 软件 MIT 最大 kp，N·m/rad
    double max_kd{ 0.0 };          ///< 软件 MIT 最大 kd，N·m·s/rad
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
     * @brief 校验 MIT 命令并下发本阶段的安全零 PWM
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
     * @brief 将 HX 原始位置转换为弧度
     * @param raw 原始位置步数
     * @param raw_zero 零位步数
     * @param direction 机械方向 +1 或 -1
     * @return 关节位置，rad
     */
    static double raw_position_to_rad(
        std::uint16_t raw,
        std::uint16_t raw_zero,
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

private:
    /**
     * @brief 验证 Backend 配置
     * @param cfg 待验证配置
     * @return 成功时返回空结果，否则返回 INVALID_CFG
     */
    tl::expected<void, MotorBusErr> validate_cfg(const HiwonderBusCfg& cfg) const;

    /**
     * @brief 获取按配置顺序排列的舅机 ID
     * @return 舅机 ID 列表
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
     * @param id 舅机 ID
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
    TimePoint last_feedback_time_{};                         ///< 最近六轴同步反馈时间
    bool configured_{ false };                               ///< 是否已配置
    bool connected_{ false };                                ///< 是否已连接
    bool active_{ false };                                   ///< 是否已激活 PWM 模式
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //

} // namespace serial_arm

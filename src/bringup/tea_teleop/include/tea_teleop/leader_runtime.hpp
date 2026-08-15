#pragma once

#include <memory>
#include <string>

#include "serial_arm/config/config.hpp"
#include "serial_arm/core/joint_actuator_mapper.hpp"
#include "serial_arm/core/types.hpp"
#include "serial_arm/dynamics/dynamics.hpp"
#include "serial_arm/hardware/hardware_loader.hpp"
#include "serial_arm/hardware/motor_bus.hpp"
#include "serial_arm/robot.hpp"
#include "serial_arm_hardware_hiwonder/hx10hm_motor_bus.hpp"

namespace tea_teleop {

/**
 * @brief 主臂只读会话
 *
 * 打开 Hardware Backend 后保持 Torque OFF，仅用于读取关节状态和安全卸力
 */
class LeaderReadSession final {
public:
    LeaderReadSession() = default;
    ~LeaderReadSession();

    LeaderReadSession(const LeaderReadSession&) = delete;
    LeaderReadSession& operator=(const LeaderReadSession&) = delete;

    void open(const std::string& profile_name);
    serial_arm::JointState read();
    void close() noexcept;

private:
    serial_arm::RobotCfg cfg_;
    serial_arm::JointActuatorMapper mapper_;
    serial_arm::HardwareLoader loader_;
    std::unique_ptr<serial_arm::MotorBus> motor_bus_;
};

/**
 * @brief 主臂 SerialArm 控制会话
 *
 * 负责加载 Robot Profile、动力学和 Hardware Backend，并向遥操作层提供统一控制周期
 */
class LeaderRuntime final {
public:
    LeaderRuntime() = default;
    ~LeaderRuntime();

    LeaderRuntime(const LeaderRuntime&) = delete;
    LeaderRuntime& operator=(const LeaderRuntime&) = delete;

    void initialize(const std::string& profile_name);
    void activate(serial_arm::JointImpedanceMode mode);
    void set_impedance_mode(serial_arm::JointImpedanceMode mode);
    void set_cmd(
        const serial_arm::JointCmd& cmd,
        serial_arm::Robot::TimePoint now = serial_arm::Robot::Clock::now());

    serial_arm::RobotCycleOutput cycle(
        serial_arm::Robot::TimePoint now = serial_arm::Robot::Clock::now());

    bool safe_deactivate() noexcept;

    [[nodiscard]] double control_frequency_hz() const noexcept;
    [[nodiscard]] serial_arm::RobotState state() const noexcept;
    [[nodiscard]] const serial_arm::JointState& joint_state() const noexcept;
    [[nodiscard]] const serial_arm::RobotCfg& config() const noexcept;
    [[nodiscard]] bool tool_button_pressed() const noexcept;
    [[nodiscard]] serial_arm::ToolButtonState tool_button_state() const noexcept;

private:
    serial_arm::RobotCfg cfg_;
    serial_arm::Dynamics dynamics_;
    serial_arm::HardwareLoader loader_;
    serial_arm::Robot robot_;
    serial_arm::Hx10hmMotorBus* hiwonder_bus_{ nullptr }; ///< Robot 持有 Backend，此处仅用于读取 Tool Button 状态
    bool initialized_{ false };
};

} // namespace tea_teleop

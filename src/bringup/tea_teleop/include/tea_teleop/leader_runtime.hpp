#pragma once

#include <memory>
#include <string>
#include <thread>
#include <mutex>

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

/**
 * @brief 主臂后台控制循环
 *
 * SerialArm 保持独立 100 Hz 控制周期，RM65-B 的网络调用即使偶发阻塞
 * 也不会让主臂状态超时
 */
class LeaderCycleWorker final {
public:
    explicit LeaderCycleWorker(tea_teleop::LeaderRuntime& leader)
        : leader_(leader) {}

    ~LeaderCycleWorker() {
        stop();
    }

    LeaderCycleWorker(const LeaderCycleWorker&) = delete;
    LeaderCycleWorker& operator=(const LeaderCycleWorker&) = delete;

    void start() {
        if(worker_.joinable()) throw std::logic_error("主臂后台控制已经启动");

        const double frequency_hz = leader_.control_frequency_hz();
        if(!std::isfinite(frequency_hz) || frequency_hz <= 0.0) {
            throw std::runtime_error("主臂控制频率无效");
        }

        period_ = std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(1.0 / frequency_hz));
        stop_requested_.store(false);
        worker_ = std::thread([this]() { run(); });
    }

    void stop() noexcept {
        stop_requested_.store(true);
        if(worker_.joinable()) worker_.join();
    }

    serial_arm::RobotCycleOutput latest() const {
        std::lock_guard<std::mutex> lock(mutex_);
        if(fault_) std::rethrow_exception(fault_);
        if(!latest_) throw std::runtime_error("主臂后台控制尚未产生有效状态");
        return *latest_;
    }

    void wait_until_ready(std::chrono::milliseconds timeout) const {
        const auto deadline = Clock::now() + timeout;
        for(;;) {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if(fault_) std::rethrow_exception(fault_);
                if(latest_) return;
            }

            if(Clock::now() >= deadline) {
                throw std::runtime_error("等待主臂后台控制状态超时");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds{ 2 });
        }
    }

    void rethrow_if_failed() const {
        std::lock_guard<std::mutex> lock(mutex_);
        if(fault_) std::rethrow_exception(fault_);
    }

    [[nodiscard]] std::uint64_t cycle_count() const noexcept {
        return cycle_count_.load();
    }

    [[nodiscard]] std::uint64_t overrun_count() const noexcept {
        return overrun_count_.load();
    }

private:
    using Clock = std::chrono::steady_clock;

    void run() noexcept {
        auto next_tick = Clock::now();

        while(!stop_requested_.load()) {
            const auto cycle_start = Clock::now();

            try {
                auto output = leader_.cycle(cycle_start);
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    latest_ = std::move(output);
                }
                ++cycle_count_;
            }
            catch(...) {
                std::lock_guard<std::mutex> lock(mutex_);
                fault_ = std::current_exception();
                break;
            }

            next_tick += period_;
            const auto after_work = Clock::now();
            if(next_tick > after_work) {
                std::this_thread::sleep_until(next_tick);
            }
            else {
                ++overrun_count_;
                next_tick = after_work;
            }
        }
    }

    tea_teleop::LeaderRuntime& leader_;
    Clock::duration period_{};
    std::atomic<bool> stop_requested_{ false };
    std::atomic<std::uint64_t> cycle_count_{ 0U };
    std::atomic<std::uint64_t> overrun_count_{ 0U };

    mutable std::mutex mutex_;
    std::optional<serial_arm::RobotCycleOutput> latest_;
    std::exception_ptr fault_;
    std::thread worker_;
};

} // namespace tea_teleop

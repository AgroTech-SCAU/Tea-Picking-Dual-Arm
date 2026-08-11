#include "tea_teleop/teleop.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <exception>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "serial_arm/core/types.hpp"
#include "tea_teleop/leader_runtime.hpp"

namespace {

constexpr std::size_t kJointCount = 6;
constexpr float kFullAlignToleranceDegree = 2.0F;
constexpr double kRadToDegree = 57.2957795130823208768;
constexpr double kHomeStaticAssistNm = 0.04;
constexpr float kAlignMaxStepDegree = 0.25F;
constexpr float kAlignSettleToleranceDegree = 0.5F;
constexpr auto kAlignTimeout = std::chrono::seconds{ 8 };
constexpr auto kTeleopPeriod = std::chrono::milliseconds{ 20 };

volatile std::sig_atomic_t g_interrupt_requested = 0;

void signal_handler(int) {
    g_interrupt_requested = 1;
}

void clear_interrupt() {
    g_interrupt_requested = 0;
}

bool interrupted() {
    return g_interrupt_requested != 0;
}

std::string prompt_line(const std::string& prompt) {
    std::cout << prompt;
    std::cout.flush();

    std::string line;
    if(!std::getline(std::cin, line)) {
        throw std::runtime_error("标准输入已关闭");
    }
    return line;
}

int prompt_int(const std::string& prompt) {
    for(;;) {
        const std::string text = prompt_line(prompt);
        try {
            std::size_t parsed = 0;
            const int value = std::stoi(text, &parsed);
            if(parsed == text.size()) return value;
        }
        catch(const std::exception&) {
        }
        std::cout << "输入无效，请重新输入\n";
    }
}

float prompt_float(const std::string& prompt) {
    for(;;) {
        const std::string text = prompt_line(prompt);
        try {
            std::size_t parsed = 0;
            const float value = std::stof(text, &parsed);
            if(parsed == text.size()) return value;
        }
        catch(const std::exception&) {
        }
        std::cout << "输入无效，请重新输入\n";
    }
}

void print_degree_array(
    const std::string& name,
    const std::array<float, kJointCount>& value) {
    std::cout << name << "  ";
    for(std::size_t i = 0; i < value.size(); ++i) {
        std::cout
            << "J" << (i + 1)
            << "[" << std::fixed << std::setprecision(1)
            << value[i] << "deg] ";
    }
    std::cout << "\n";
}

void print_difference_line(
    const std::array<float, kJointCount>& leader,
    const std::array<float, kJointCount>& slave) {
    std::array<float, kJointCount> difference{};
    for(std::size_t i = 0; i < kJointCount; ++i) {
        difference[i] = leader[i] - slave[i];
    }
    print_degree_array("差值", difference);
}

float max_abs_difference(
    const std::array<float, kJointCount>& lhs,
    const std::array<float, kJointCount>& rhs) {
    float result = 0.0F;
    for(std::size_t i = 0; i < kJointCount; ++i) {
        result = std::max(result, std::abs(lhs[i] - rhs[i]));
    }
    return result;
}

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

} // namespace

TeaTeleop::TeaTeleop(TeleopConfig config)
    : config_(std::move(config)) {}

int TeaTeleop::run() {
    std::signal(SIGINT, signal_handler);

    for(;;) {
        clear_interrupt();
        print_main_menu();
        const int option = prompt_int("选择功能: ");

        try {
            switch(option) {
                case 0: return 0;
                case 1: read_leader(); break;
                case 2: read_slave(); break;
                case 3: read_compare(); break;
                case 4: home_leader_menu(); break;
                case 5: home_slave_menu(); break;
                case 6: home_both_menu(); break;
                case 7: teleop(TeleopMode::Slow); break;
                case 8: teleop(TeleopMode::Full); break;
                case 9: config_menu(); break;
                case 10: software_stop(); break;
                case 11: release_leader_menu(); break;
                default: std::cout << "未知选项\n"; break;
            }
        }
        catch(const std::exception& error) {
            std::cerr << "[失败] " << error.what() << "\n";
        }
    }
}

void TeaTeleop::print_main_menu() const {
    std::cout
        << "\n============================================================\n"
        << " Tea-Picking-Dual-Arm\n"
        << "============================================================\n"
        << " 主臂: " << config_.leader_profile << "\n"
        << " 从臂: " << config_.rm_ip << ":" << config_.rm_port << "\n"
        << "------------------------------------------------------------\n"
        << " 1. 读取主臂\n"
        << " 2. 读取从臂\n"
        << " 3. 主从臂读取对照\n"
        << " 4. 主臂归零\n"
        << " 5. 从臂归零\n"
        << " 6. 主从臂归零\n"
        << " 7. 慢速遥操作\n"
        << " 8. 遥操作\n"
        << " 9. 修改运行配置\n"
        << "10. 从臂软件停止\n"
        << "11. 主臂卸力\n"
        << " 0. 退出\n"
        << "============================================================\n"
        << "遥操作和归零过程中可按 Ctrl+C 中断\n";
}

void TeaTeleop::print_config() const {
    std::cout
        << "\n---------------- 当前配置 ----------------\n"
        << "主臂 Profile             = " << config_.leader_profile << "\n"
        << "RM65-B 地址              = " << config_.rm_ip << ":" << config_.rm_port << "\n"
        << "遥操作持续时间           = " << config_.teleop_duration_s << " s\n"
        << "慢速单周期最大变化       = " << config_.slow_max_step_degree << " deg\n"
        << "启动最大允许角差         = " << config_.max_start_error_degree << " deg\n"
        << "主臂归零容差             = " << config_.leader_home_tolerance_degree << " deg\n"
        << "主臂归零速度             = " << config_.leader_home_speed_degree_s << " deg/s\n"
        << "主臂归零超时             = " << config_.leader_home_timeout_s << " s\n"
        << "从臂归零/对齐速度        = " << config_.slave_home_speed_percent << "%\n"
        << "映射方向                 = [";

    for(std::size_t i = 0; i < kJointCount; ++i) {
        std::cout << static_cast<int>(config_.mapping_direction[i]);
        if(i + 1 != kJointCount) std::cout << ", ";
    }
    std::cout << "]\n------------------------------------------\n";
}

void TeaTeleop::config_menu() {
    for(;;) {
        print_config();
        std::cout
            << " 1. 修改 RM65-B IP\n"
            << " 2. 修改 RM65-B 端口\n"
            << " 3. 修改关节映射方向\n"
            << " 4. 修改慢速遥操作步长\n"
            << " 5. 修改主臂归零速度\n"
            << " 6. 修改主臂归零容差\n"
            << " 7. 修改主臂归零超时\n"
            << " 8. 修改从臂归零/对齐速度\n"
            << " 9. 修改遥操作持续时间\n"
            << "10. 恢复默认配置\n"
            << " 0. 返回\n";

        switch(prompt_int("选择配置项: ")) {
            case 0:
                return;

            case 1: {
                const std::string value = prompt_line("新的 RM65-B IP: ");
                if(!value.empty()) {
                    rm_.disconnect();
                    config_.rm_ip = value;
                }
                break;
            }

            case 2: {
                const int value = prompt_int("新的 TCP 端口 [1~65535]: ");
                if(value < 1 || value > 65535) {
                    std::cout << "端口范围无效\n";
                    break;
                }
                rm_.disconnect();
                config_.rm_port = value;
                break;
            }

            case 3:
                mapping_direction_menu();
                break;

            case 4: {
                const float value = prompt_float("慢速模式单周期最大变化 deg (0, 5]: ");
                if(value > 0.0F && value <= 5.0F) config_.slow_max_step_degree = value;
                else std::cout << "范围无效\n";
                break;
            }

            case 5: {
                const float value = prompt_float("主臂归零速度 deg/s [2~30]: ");
                if(value >= 2.0F && value <= 30.0F) config_.leader_home_speed_degree_s = value;
                else std::cout << "范围无效\n";
                break;
            }

            case 6: {
                const float value = prompt_float("主臂归零容差 deg [0.5~5]: ");
                if(value >= 0.5F && value <= 5.0F) config_.leader_home_tolerance_degree = value;
                else std::cout << "范围无效\n";
                break;
            }

            case 7: {
                const int value = prompt_int("主臂归零超时 s [5~60]: ");
                if(value >= 5 && value <= 60) config_.leader_home_timeout_s = value;
                else std::cout << "范围无效\n";
                break;
            }

            case 8: {
                const int value = prompt_int("从臂归零/对齐速度百分比 [1~30]: ");
                if(value >= 1 && value <= 30) config_.slave_home_speed_percent = value;
                else std::cout << "范围无效\n";
                break;
            }

            case 9: {
                const int value = prompt_int("遥操作持续时间 s (-1 或 >=1): ");
                if(value == -1 || value >= 1) config_.teleop_duration_s = value;
                else std::cout << "只允许 -1 或正整数\n";
                break;
            }

            case 10:
                rm_.disconnect();
                config_ = TeleopConfig{};
                std::cout << "已恢复默认配置\n";
                break;

            default:
                std::cout << "未知选项\n";
                break;
        }
    }
}

void TeaTeleop::mapping_direction_menu() {
    for(;;) {
        std::cout << "当前方向: ";
        for(std::size_t i = 0; i < kJointCount; ++i) {
            std::cout
                << "J" << (i + 1)
                << "=" << static_cast<int>(config_.mapping_direction[i]) << " ";
        }
        std::cout << "\n";

        const int joint = prompt_int("选择关节 [1~6]，0 返回，7 恢复默认方向: ");
        if(joint == 0) return;
        if(joint == 7) {
            config_.mapping_direction = TeleopConfig{}.mapping_direction;
            continue;
        }
        if(joint < 1 || joint > 6) {
            std::cout << "关节编号无效\n";
            continue;
        }

        const int direction = prompt_int("方向输入 1 或 -1: ");
        if(direction != 1 && direction != -1) {
            std::cout << "方向只能是 1 或 -1\n";
            continue;
        }
        config_.mapping_direction[static_cast<std::size_t>(joint - 1)] =
            static_cast<float>(direction);
    }
}

void TeaTeleop::read_leader() {
    tea_teleop::LeaderReadSession leader;
    leader.open(config_.leader_profile);
    const auto degree = leader_joint_to_degree(leader.read().pos);
    print_degree_array("主臂", degree);
}

void TeaTeleop::read_slave() {
    ensure_rm_connected();
    print_degree_array("从臂", rm_.read_all_degree());
}

void TeaTeleop::read_compare() {
    tea_teleop::LeaderReadSession leader;
    leader.open(config_.leader_profile);
    ensure_rm_connected();

    const auto leader_degree = leader_joint_to_degree(leader.read().pos);
    const auto slave_degree = rm_.read_all_degree();
    print_degree_array("主臂", leader_degree);
    print_degree_array("从臂", slave_degree);
    print_difference_line(leader_degree, slave_degree);
}

void TeaTeleop::home_leader_menu() {
    (void)home_leader();
}

void TeaTeleop::home_slave_menu() {
    (void)home_slave();
}

void TeaTeleop::home_both_menu() {
    if(!home_slave()) return;
    (void)home_leader();
}

void TeaTeleop::release_leader_menu() {
    tea_teleop::LeaderReadSession leader;
    leader.open(config_.leader_profile);
    leader.close();
    std::cout << "主臂已卸力\n";
}

bool TeaTeleop::home_leader() {
    tea_teleop::LeaderRuntime leader;
    leader.initialize(config_.leader_profile);

    using Clock = std::chrono::steady_clock;
    const double frequency_hz = leader.control_frequency_hz();
    if(!std::isfinite(frequency_hz) || frequency_hz <= 0.0) {
        throw std::runtime_error("主臂控制频率无效");
    }

    const auto period = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(1.0 / frequency_hz));
    const double speed_rad_s =
        static_cast<double>(config_.leader_home_speed_degree_s) / kRadToDegree;
    const double tolerance_rad =
        static_cast<double>(config_.leader_home_tolerance_degree) / kRadToDegree;

    std::cout
        << "主臂归零开始  速度=" << config_.leader_home_speed_degree_s
        << " deg/s  容差=" << config_.leader_home_tolerance_degree
        << " deg\n";

    clear_interrupt();
    try {
        leader.activate(serial_arm::JointImpedanceMode::RIGID_TRACKING);

        serial_arm::JointVector reference = leader.joint_state().pos;
        if(reference.size() != kJointCount) {
            throw std::runtime_error("主臂关节状态数量不是 6");
        }

        serial_arm::JointVector actual_position = reference;
        serial_arm::JointVector reference_vel(kJointCount, 0.0);
        serial_arm::JointVector static_assist(kJointCount, 0.0);
        const auto deadline = Clock::now() +
            std::chrono::seconds{ config_.leader_home_timeout_s };
        auto next_tick = Clock::now();
        auto last_print = next_tick;
        std::size_t settled_cycles = 0U;

        while(!interrupted()) {
            const auto now = Clock::now();
            if(now >= deadline) throw std::runtime_error("主臂归零超时");

            const double max_step = speed_rad_s / frequency_hz;
            for(std::size_t i = 0; i < kJointCount; ++i) {
                const double error = -reference[i];
                const double step = std::clamp(error, -max_step, max_step);
                reference[i] += step;
                reference_vel[i] = step * frequency_hz;
                if(std::abs(error) <= max_step) {
                    reference[i] = 0.0;
                    reference_vel[i] = 0.0;
                }
            }

            for(std::size_t i = 0; i < kJointCount; ++i) {
                const double actual_error = -actual_position[i];
                static_assist[i] =
                    std::abs(actual_error) > tolerance_rad ?
                    std::copysign(kHomeStaticAssistNm, actual_error) :
                    0.0;
            }

            leader.set_cmd(
                serial_arm::JointPosVelTorCmd{
                    reference,
                    reference_vel,
                    static_assist,
                },
                now);
            const auto output = leader.cycle(now);
            actual_position = output.joint_state.pos;

            bool settled = true;
            for(std::size_t i = 0; i < kJointCount; ++i) {
                if(std::abs(output.joint_state.pos[i]) > tolerance_rad ||
                    std::abs(output.joint_state.vel[i]) > 0.10) {
                    settled = false;
                    break;
                }
            }
            settled_cycles = settled ? settled_cycles + 1U : 0U;

            if(now - last_print >= std::chrono::milliseconds{ 500 }) {
                print_degree_array(
                    "主臂当前位置",
                    leader_joint_to_degree(output.joint_state.pos));
                last_print = now;
            }

            if(settled_cycles >= 10U) {
                if(!leader.safe_deactivate()) {
                    throw std::runtime_error("主臂到达零点，但失能失败");
                }
                std::cout << "主臂归零完成并已卸力\n";
                clear_interrupt();
                return true;
            }

            next_tick += period;
            const auto after_work = Clock::now();
            if(next_tick > after_work) std::this_thread::sleep_until(next_tick);
            else next_tick = after_work;
        }

        if(!leader.safe_deactivate()) {
            throw std::runtime_error("主臂归零已中断，但失能失败");
        }
        std::cout << "主臂归零已中断并已卸力\n";
        clear_interrupt();
        return false;
    }
    catch(...) {
        (void)leader.safe_deactivate();
        clear_interrupt();
        throw;
    }
}

bool TeaTeleop::home_slave() {
    ensure_rm_connected();
    print_degree_array("从臂归零前", rm_.read_all_degree());

    const std::array<float, kJointCount> zero{};
    rm_.movej_degree(zero, config_.slave_home_speed_percent, true);

    print_degree_array("从臂归零后", rm_.read_all_degree());
    std::cout << "从臂归零完成\n";
    return true;
}

void TeaTeleop::teleop(TeleopMode mode) {
    tea_teleop::LeaderRuntime leader;
    leader.initialize(config_.leader_profile);
    ensure_rm_connected();

    std::cout
        << (mode == TeleopMode::Slow ? "慢速遥操作开始\n" : "遥操作开始\n")
        << "主臂使用 COMPLIANT_DRAG + GRAVITY，Ctrl+C 结束\n";

    clear_interrupt();
    bool rm_stop_ok = true;
    std::unique_ptr<LeaderCycleWorker> worker;

    auto safe_shutdown = [&]() noexcept {
        try {
            rm_.stop();
        }
        catch(const std::exception& error) {
            rm_stop_ok = false;
            std::cerr << "[警告] 从臂停止失败: " << error.what() << "\n";
        }

        if(worker) worker->stop();

        const bool leader_stop_ok = leader.safe_deactivate();
        if(!leader_stop_ok) std::cerr << "[警告] 主臂安全失能失败\n";
        return rm_stop_ok && leader_stop_ok;
    };

    try {
        leader.activate(serial_arm::JointImpedanceMode::COMPLIANT_DRAG);

        worker = std::make_unique<LeaderCycleWorker>(leader);
        worker->start();
        worker->wait_until_ready(std::chrono::milliseconds{ 500 });

        // 让重力补偿先稳定几个周期，再读取主臂姿态
        std::this_thread::sleep_for(std::chrono::milliseconds{ 200 });
        worker->rethrow_if_failed();

        auto output = worker->latest();
        auto target = leader_joint_to_degree(output.joint_state.pos);
        auto slave_start = rm_.read_all_degree();
        print_difference_line(target, slave_start);
        validate_start_error(target, slave_start);

        if(mode == TeleopMode::Full &&
            max_abs_difference(target, slave_start) > kFullAlignToleranceDegree) {
            std::cout
                << "从臂正在平滑对齐到当前主臂姿态"
                << "，对齐期间可按 Ctrl+C 中断\n";

            auto align_command = slave_start;
            auto align_next_tick = std::chrono::steady_clock::now();
            const auto align_deadline = align_next_tick + kAlignTimeout;
            std::size_t align_settled_cycles = 0U;

            while(!interrupted()) {
                if(std::chrono::steady_clock::now() >= align_deadline) {
                    throw std::runtime_error("从臂对齐超时");
                }

                worker->rethrow_if_failed();
                output = worker->latest();
                target = leader_joint_to_degree(output.joint_state.pos);

                for(std::size_t i = 0; i < kJointCount; ++i) {
                    const float error = target[i] - align_command[i];
                    align_command[i] += std::clamp(
                        error,
                        -kAlignMaxStepDegree,
                        kAlignMaxStepDegree);
                }

                rm_.write_all_degree(align_command, false);

                const bool aligned =
                    max_abs_difference(target, align_command) <=
                    kAlignSettleToleranceDegree;
                align_settled_cycles = aligned ? align_settled_cycles + 1U : 0U;
                if(align_settled_cycles >= 10U) break;

                align_next_tick += kTeleopPeriod;
                const auto after_align_work = std::chrono::steady_clock::now();
                if(align_next_tick > after_align_work) {
                    std::this_thread::sleep_until(align_next_tick);
                }
                else {
                    align_next_tick = after_align_work;
                }
            }

            if(interrupted()) {
                (void)safe_shutdown();
                clear_interrupt();
                return;
            }

            worker->rethrow_if_failed();
            output = worker->latest();
            target = leader_joint_to_degree(output.joint_state.pos);
            slave_start = rm_.read_all_degree();
            print_difference_line(target, slave_start);
        }

        auto previous_command = slave_start;
        const auto start_time = std::chrono::steady_clock::now();
        auto last_status = start_time;
        auto next_tick = start_time;
        std::uint64_t teleop_cycle_count = 0U;

        while(!interrupted()) {
            const auto cycle_start = std::chrono::steady_clock::now();
            if(config_.teleop_duration_s >= 0 &&
                cycle_start - start_time >=
                    std::chrono::seconds{ config_.teleop_duration_s }) {
                break;
            }

            worker->rethrow_if_failed();
            output = worker->latest();
            target = leader_joint_to_degree(output.joint_state.pos);

            const auto command = mode == TeleopMode::Slow ?
                limit_slow_command(target, previous_command) : target;

            rm_.write_all_degree(command, mode == TeleopMode::Full);
            previous_command = command;
            ++teleop_cycle_count;

            const auto after_work = std::chrono::steady_clock::now();
            if(after_work - last_status >= std::chrono::seconds{ 1 }) {
                std::cout
                    << "遥操作运行中  发送周期=" << teleop_cycle_count
                    << "  主臂周期=" << worker->cycle_count()
                    << "  主臂超期=" << worker->overrun_count()
                    << "  J1=" << std::fixed << std::setprecision(1)
                    << target[0] << " deg"
                    << "  J2重力前馈=" << output.model_feedforward[1] << " N.m\n";
                last_status = after_work;
            }

            next_tick += kTeleopPeriod;
            if(next_tick > after_work) {
                std::this_thread::sleep_until(next_tick);
            }
            else {
                next_tick = after_work;
            }
        }

        if(!safe_shutdown()) {
            throw std::runtime_error("遥操作结束时安全停止未完全成功");
        }

        std::cout
            << "遥操作结束  发送周期=" << teleop_cycle_count
            << "  主臂周期=" << (worker ? worker->cycle_count() : 0U)
            << "  主臂超期=" << (worker ? worker->overrun_count() : 0U)
            << "\n";
    }
    catch(...) {
        (void)safe_shutdown();
        clear_interrupt();
        throw;
    }

    clear_interrupt();
}

void TeaTeleop::software_stop() {
    ensure_rm_connected();
    rm_.stop();
    std::cout << "从臂软件停止命令已发送\n";
}

void TeaTeleop::ensure_rm_connected() {
    if(rm_.is_connected()) return;
    rm_.connect(config_.rm_ip, config_.rm_port);
    std::cout << "从臂已连接: " << config_.rm_ip << ":" << config_.rm_port << "\n";
}

std::array<float, kJointCount> TeaTeleop::leader_joint_to_degree(
    const std::vector<double>& joint_position) const {
    if(joint_position.size() != kJointCount) {
        throw std::runtime_error("主臂关节状态数量不是 6");
    }

    std::array<float, kJointCount> degree{};
    for(std::size_t i = 0; i < kJointCount; ++i) {
        degree[i] = config_.mapping_direction[i] *
            static_cast<float>(joint_position[i] * kRadToDegree);
    }
    return degree;
}

std::array<float, kJointCount> TeaTeleop::limit_slow_command(
    const std::array<float, kJointCount>& target,
    const std::array<float, kJointCount>& previous) const {
    std::array<float, kJointCount> command{};
    for(std::size_t i = 0; i < kJointCount; ++i) {
        command[i] = std::clamp(
            target[i],
            previous[i] - config_.slow_max_step_degree,
            previous[i] + config_.slow_max_step_degree);
    }
    return command;
}

void TeaTeleop::validate_start_error(
    const std::array<float, kJointCount>& leader_degree,
    const std::array<float, kJointCount>& slave_degree) const {
    std::ostringstream error;
    bool failed = false;

    for(std::size_t i = 0; i < kJointCount; ++i) {
        const float diff = std::abs(leader_degree[i] - slave_degree[i]);
        if(diff > config_.max_start_error_degree) {
            failed = true;
            error
                << "J" << (i + 1)
                << "=" << std::fixed << std::setprecision(2)
                << diff << "deg ";
        }
    }

    if(failed) {
        throw std::runtime_error(
            "主从初始角差超过 " +
            std::to_string(config_.max_start_error_degree) +
            " deg，拒绝启动遥操作: " + error.str());
    }
}

int main(int argc, char** argv) {
    TeleopConfig config;

    if(argc >= 2 && std::string(argv[1]) == "--check-leader") {
        try {
            tea_teleop::LeaderRuntime runtime;
            runtime.initialize(config.leader_profile);
            std::cout
                << "主臂配置检查通过: " << config.leader_profile
                << "  控制频率=" << runtime.control_frequency_hz() << " Hz\n";
            return 0;
        }
        catch(const std::exception& error) {
            std::cerr << "主臂配置检查失败: " << error.what() << "\n";
            return 1;
        }
    }

    if(argc != 1) {
        std::cerr << "用法: ros2 run tea_teleop teleop\n";
        return 2;
    }

    try {
        TeaTeleop app(config);
        return app.run();
    }
    catch(const std::exception& error) {
        std::cerr << "[致命错误] " << error.what() << "\n";
        return 1;
    }
}

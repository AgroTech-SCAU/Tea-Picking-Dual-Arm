#include "tea_teleop/dual_teleop.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

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
constexpr auto kStatusInterval = std::chrono::seconds{ 2 };

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

const YAML::Node require_map(
    const YAML::Node& parent,
    const char* key,
    const char* context) {
    const YAML::Node node = parent[key];
    if(!node || !node.IsMap()) {
        throw std::runtime_error(
            std::string("双臂遥操作配置缺少映射项: ") + context + "." + key);
    }
    return node;
}

template <typename T>
T require_value(
    const YAML::Node& parent,
    const char* key,
    const char* context) {
    const YAML::Node node = parent[key];
    if(!node) {
        throw std::runtime_error(
            std::string("双臂遥操作配置缺少参数: ") + context + "." + key);
    }

    try {
        return node.as<T>();
    }
    catch(const YAML::Exception&) {
        throw std::runtime_error(
            std::string("双臂遥操作配置参数类型错误: ") + context + "." + key);
    }
}

DualArmConfig load_arm_config(
    const YAML::Node& root,
    const char* side_name) {
    const YAML::Node side = require_map(root, side_name, "root");
    const YAML::Node follower = require_map(side, "follower", side_name);

    DualArmConfig config;
    config.leader_profile =
        require_value<std::string>(side, "leader_profile", side_name);
    config.follower_ip =
        require_value<std::string>(follower, "ip", side_name);
    config.follower_port =
        require_value<int>(follower, "port", side_name);
    config.follower_home_speed_percent =
        require_value<int>(follower, "home_speed_percent", side_name);

    const YAML::Node mapping = side["mapping_direction"];
    if(!mapping || !mapping.IsSequence() || mapping.size() != kJointCount) {
        throw std::runtime_error(
            std::string(side_name) +
            ".mapping_direction 必须包含 6 个方向值");
    }

    for(std::size_t i = 0; i < kJointCount; ++i) {
        const float value = mapping[i].as<float>();
        if(value != 1.0F && value != -1.0F) {
            throw std::runtime_error(
                std::string(side_name) +
                ".mapping_direction 只允许 1 或 -1");
        }
        config.mapping_direction[i] = value;
    }

    if(config.leader_profile.empty()) {
        throw std::runtime_error(std::string(side_name) + ".leader_profile 不能为空");
    }
    if(config.follower_ip.empty()) {
        throw std::runtime_error(std::string(side_name) + ".follower.ip 不能为空");
    }
    if(config.follower_port < 1 || config.follower_port > 65535) {
        throw std::runtime_error(
            std::string(side_name) + ".follower.port 必须位于 [1, 65535]");
    }
    if(config.follower_home_speed_percent < 1 ||
        config.follower_home_speed_percent > 30) {
        throw std::runtime_error(
            std::string(side_name) +
            ".follower.home_speed_percent 必须位于 [1, 30]");
    }

    return config;
}

DualTeleopConfig load_dual_teleop_config(const std::string& path) {
    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    }
    catch(const YAML::Exception& error) {
        throw std::runtime_error(
            "双臂遥操作配置加载失败: " + path +
            " (" + error.what() + ")");
    }

    DualTeleopConfig config;
    config.left = load_arm_config(root, "left");
    config.right = load_arm_config(root, "right");

    const YAML::Node teleop = require_map(root, "teleop", "root");
    const YAML::Node leader_home = require_map(root, "leader_home", "root");

    config.teleop_duration_s =
        require_value<int>(teleop, "duration_s", "teleop");
    config.teleop_period_ms =
        require_value<int>(teleop, "period_ms", "teleop");
    config.slow_max_step_degree =
        require_value<float>(teleop, "slow_max_step_degree", "teleop");
    config.max_start_error_degree =
        require_value<float>(teleop, "max_start_error_degree", "teleop");

    config.leader_home_speed_degree_s =
        require_value<float>(leader_home, "speed_degree_s", "leader_home");
    config.leader_home_tolerance_degree =
        require_value<float>(leader_home, "tolerance_degree", "leader_home");
    config.leader_home_timeout_s =
        require_value<int>(leader_home, "timeout_s", "leader_home");

    if(config.teleop_duration_s != -1 && config.teleop_duration_s < 1) {
        throw std::runtime_error("teleop.duration_s 只允许 -1 或正整数");
    }
    if(config.teleop_period_ms < 5 || config.teleop_period_ms > 10) {
        throw std::runtime_error("teleop.period_ms 必须位于 [5, 10]");
    }
    if(config.slow_max_step_degree <= 0.0F || config.slow_max_step_degree > 5.0F) {
        throw std::runtime_error("teleop.slow_max_step_degree 必须位于 (0, 5]");
    }
    if(config.max_start_error_degree <= 0.0F ||
        config.max_start_error_degree > 180.0F) {
        throw std::runtime_error("teleop.max_start_error_degree 必须位于 (0, 180]");
    }
    if(config.leader_home_speed_degree_s < 2.0F ||
        config.leader_home_speed_degree_s > 30.0F) {
        throw std::runtime_error("leader_home.speed_degree_s 必须位于 [2, 30]");
    }
    if(config.leader_home_tolerance_degree < 0.5F ||
        config.leader_home_tolerance_degree > 5.0F) {
        throw std::runtime_error("leader_home.tolerance_degree 必须位于 [0.5, 5]");
    }
    if(config.leader_home_timeout_s < 5 || config.leader_home_timeout_s > 60) {
        throw std::runtime_error("leader_home.timeout_s 必须位于 [5, 60]");
    }

    return config;
}

std::string default_dual_teleop_config_path() {
    const std::filesystem::path share =
        ament_index_cpp::get_package_share_directory("tea_teleop");
    return (share / "config" / "dual_teleop.yaml").string();
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
    const std::string& name,
    const std::array<float, kJointCount>& leader,
    const std::array<float, kJointCount>& follower) {
    std::array<float, kJointCount> difference{};
    for(std::size_t i = 0; i < kJointCount; ++i) {
        difference[i] = leader[i] - follower[i];
    }
    print_degree_array(name, difference);
}

struct DifferenceSummary {
    float max_abs_degree{ 0.0F };
    std::size_t joint_index{ 0U };
};

DifferenceSummary summarize_difference(
    const std::array<float, kJointCount>& lhs,
    const std::array<float, kJointCount>& rhs) {
    DifferenceSummary result;
    for(std::size_t i = 0; i < kJointCount; ++i) {
        const float value = std::abs(lhs[i] - rhs[i]);
        if(value > result.max_abs_degree) {
            result.max_abs_degree = value;
            result.joint_index = i;
        }
    }
    return result;
}

float max_abs_difference(
    const std::array<float, kJointCount>& lhs,
    const std::array<float, kJointCount>& rhs) {
    return summarize_difference(lhs, rhs).max_abs_degree;
}

void step_toward(
    std::array<float, kJointCount>& command,
    const std::array<float, kJointCount>& target,
    float max_step_degree) {
    for(std::size_t i = 0; i < kJointCount; ++i) {
        const float error = target[i] - command[i];
        command[i] += std::clamp(error, -max_step_degree, max_step_degree);
    }
}

std::array<float, kJointCount> joint_to_degree(
    const std::vector<double>& joint_position,
    const DualArmConfig& arm) {
    if(joint_position.size() != kJointCount) {
        throw std::runtime_error("主臂关节状态数量不是 6");
    }

    std::array<float, kJointCount> degree{};
    for(std::size_t i = 0; i < kJointCount; ++i) {
        degree[i] = arm.mapping_direction[i] *
            static_cast<float>(joint_position[i] * kRadToDegree);
    }
    return degree;
}

bool home_one_leader(
    const DualArmConfig& arm,
    const DualTeleopConfig& common,
    const char* side_label) {
    tea_teleop::LeaderRuntime leader;
    leader.initialize(arm.leader_profile);

    using Clock = std::chrono::steady_clock;
    const double frequency_hz = leader.control_frequency_hz();
    if(!std::isfinite(frequency_hz) || frequency_hz <= 0.0) {
        throw std::runtime_error("主臂控制频率无效");
    }

    const auto period = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(1.0 / frequency_hz));
    const double speed_rad_s =
        static_cast<double>(common.leader_home_speed_degree_s) / kRadToDegree;
    const double tolerance_rad =
        static_cast<double>(common.leader_home_tolerance_degree) / kRadToDegree;

    std::cout
        << side_label << "主臂归零开始  Profile=" << arm.leader_profile
        << "  速度=" << common.leader_home_speed_degree_s
        << " deg/s  容差=" << common.leader_home_tolerance_degree
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
            std::chrono::seconds{ common.leader_home_timeout_s };
        auto next_tick = Clock::now();
        auto last_print = next_tick;
        std::size_t settled_cycles = 0U;

        while(!interrupted()) {
            const auto now = Clock::now();
            if(now >= deadline) {
                throw std::runtime_error(std::string(side_label) + "主臂归零超时");
            }

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
                    std::string(side_label) + "主臂当前位置",
                    joint_to_degree(output.joint_state.pos, arm));
                last_print = now;
            }

            if(settled_cycles >= 10U) {
                if(!leader.safe_deactivate()) {
                    throw std::runtime_error(
                        std::string(side_label) + "主臂到达零点，但失能失败");
                }
                std::cout << side_label << "主臂归零完成并已卸力\n";
                clear_interrupt();
                return true;
            }

            next_tick += period;
            const auto after_work = Clock::now();
            if(next_tick > after_work) {
                std::this_thread::sleep_until(next_tick);
            }
            else {
                next_tick = after_work;
            }
        }

        if(!leader.safe_deactivate()) {
            throw std::runtime_error(
                std::string(side_label) + "主臂归零已中断，但失能失败");
        }
        std::cout << side_label << "主臂归零已中断并已卸力\n";
        clear_interrupt();
        return false;
    }
    catch(...) {
        (void)leader.safe_deactivate();
        clear_interrupt();
        throw;
    }
}

} // namespace

DualTeaTeleop::DualTeaTeleop(DualTeleopConfig config)
    : config_(std::move(config)) {}

int DualTeaTeleop::run() {
    std::signal(SIGINT, signal_handler);

    for(;;) {
        clear_interrupt();
        print_main_menu();
        const int option = prompt_int("选择功能: ");

        try {
            switch(option) {
                case 0: return 0;
                case 1: read_all(); break;
                case 2: read_compare(); break;
                case 3: (void)home_leaders(); break;
                case 4: (void)home_followers(); break;
                case 5: (void)home_all(); break;
                case 6: teleop(TeleopMode::Slow); break;
                case 7: teleop(TeleopMode::Full); break;
                case 8: software_stop_all(); break;
                case 9: release_leaders(); break;
                default: std::cout << "未知选项\n"; break;
            }
        }
        catch(const std::exception& error) {
            std::cerr << "[失败] " << error.what() << "\n";
        }
    }
}

void DualTeaTeleop::print_main_menu() const {
    std::cout
        << "\n============================================================\n"
        << " Tea-Picking-Dual-Arm | Dual Teleop\n"
        << "============================================================\n"
        << " Left  : " << config_.left.leader_profile
        << " -> " << config_.left.follower_ip
        << ":" << config_.left.follower_port << "\n"
        << " Right : " << config_.right.leader_profile
        << " -> " << config_.right.follower_ip
        << ":" << config_.right.follower_port << "\n"
        << "------------------------------------------------------------\n"
        << " 1. 读取双主臂 / 双从臂\n"
        << " 2. 左右主从状态对照\n"
        << " 3. 双主臂归零\n"
        << " 4. 双从臂归零\n"
        << " 5. 全部归零\n"
        << " 6. 双臂慢速遥操作\n"
        << " 7. 双臂遥操作\n"
        << " 8. 双从臂软件停止\n"
        << " 9. 双主臂卸力\n"
        << " 0. 退出\n"
        << "============================================================\n"
        << "遥操作和归零过程中可按 Ctrl+C 中断\n";
}

void DualTeaTeleop::print_config() const {
    std::cout
        << "\n---------------- 当前双臂配置 ----------------\n"
        << "左主臂 Profile           = " << config_.left.leader_profile << "\n"
        << "左 RM65-B 地址           = " << config_.left.follower_ip
        << ":" << config_.left.follower_port << "\n"
        << "右主臂 Profile           = " << config_.right.leader_profile << "\n"
        << "右 RM65-B 地址           = " << config_.right.follower_ip
        << ":" << config_.right.follower_port << "\n"
        << "双臂轮次周期             = " << config_.teleop_period_ms << " ms (~"
        << std::fixed << std::setprecision(1)
        << (1000.0 / static_cast<double>(config_.teleop_period_ms)) << " Hz)\n"
        << "遥操作持续时间           = " << config_.teleop_duration_s << " s\n"
        << "慢速单周期最大变化       = " << config_.slow_max_step_degree << " deg\n"
        << "启动最大允许角差         = " << config_.max_start_error_degree << " deg\n"
        << "主臂归零容差             = " << config_.leader_home_tolerance_degree << " deg\n"
        << "主臂归零速度             = " << config_.leader_home_speed_degree_s << " deg/s\n"
        << "主臂归零超时             = " << config_.leader_home_timeout_s << " s\n"
        << "左从臂归零/对齐速度      = "
        << config_.left.follower_home_speed_percent << "%\n"
        << "右从臂归零/对齐速度      = "
        << config_.right.follower_home_speed_percent << "%\n"
        << "------------------------------------------------\n";
}

void DualTeaTeleop::read_all() {
    tea_teleop::LeaderReadSession left_leader;
    tea_teleop::LeaderReadSession right_leader;
    left_leader.open(config_.left.leader_profile);
    right_leader.open(config_.right.leader_profile);
    ensure_followers_connected();

    print_degree_array(
        "左主臂",
        leader_joint_to_degree(left_leader.read().pos, config_.left));
    print_degree_array(
        "右主臂",
        leader_joint_to_degree(right_leader.read().pos, config_.right));
    print_degree_array("左从臂", left_rm_.read_all_degree());
    print_degree_array("右从臂", right_rm_.read_all_degree());
}

void DualTeaTeleop::read_compare() {
    tea_teleop::LeaderReadSession left_leader;
    tea_teleop::LeaderReadSession right_leader;
    left_leader.open(config_.left.leader_profile);
    right_leader.open(config_.right.leader_profile);
    ensure_followers_connected();

    const auto left_leader_degree =
        leader_joint_to_degree(left_leader.read().pos, config_.left);
    const auto right_leader_degree =
        leader_joint_to_degree(right_leader.read().pos, config_.right);
    const auto left_follower_degree = left_rm_.read_all_degree();
    const auto right_follower_degree = right_rm_.read_all_degree();

    print_degree_array("左主臂", left_leader_degree);
    print_degree_array("左从臂", left_follower_degree);
    print_difference_line("左差值", left_leader_degree, left_follower_degree);

    print_degree_array("右主臂", right_leader_degree);
    print_degree_array("右从臂", right_follower_degree);
    print_difference_line("右差值", right_leader_degree, right_follower_degree);
}

bool DualTeaTeleop::home_leaders() {
    if(!home_one_leader(config_.left, config_, "左")) {
        return false;
    }

    if(!home_one_leader(config_.right, config_, "右")) {
        return false;
    }

    return true;
}

bool DualTeaTeleop::home_followers() {
    ensure_followers_connected();

    const std::array<float, kJointCount> zero{};

    std::cout << "左从臂归零\n";
    left_rm_.movej_degree(
        zero,
        config_.left.follower_home_speed_percent,
        true);

    std::cout << "右从臂归零\n";
    right_rm_.movej_degree(
        zero,
        config_.right.follower_home_speed_percent,
        true);

    std::cout << "双从臂归零完成\n";
    return true;
}

bool DualTeaTeleop::home_all() {
    if(!home_followers()) return false;
    return home_leaders();
}

void DualTeaTeleop::release_leaders() {
    tea_teleop::LeaderReadSession left_leader;
    tea_teleop::LeaderReadSession right_leader;
    left_leader.open(config_.left.leader_profile);
    right_leader.open(config_.right.leader_profile);
    left_leader.close();
    right_leader.close();
    std::cout << "双主臂已卸力\n";
}

void DualTeaTeleop::software_stop_all() {
    ensure_followers_connected();

    bool stop_ok = true;
    try {
        left_rm_.stop();
    }
    catch(const std::exception& error) {
        stop_ok = false;
        std::cerr << "[警告] 左从臂停止失败: " << error.what() << "\n";
    }

    try {
        right_rm_.stop();
    }
    catch(const std::exception& error) {
        stop_ok = false;
        std::cerr << "[警告] 右从臂停止失败: " << error.what() << "\n";
    }

    if(!stop_ok) {
        throw std::runtime_error("双从臂软件停止未完全成功");
    }
    std::cout << "双从臂软件停止命令已发送\n";
}

void DualTeaTeleop::teleop(TeleopMode mode) {
    tea_teleop::LeaderRuntime left_leader;
    tea_teleop::LeaderRuntime right_leader;

    left_leader.initialize(config_.left.leader_profile);
    right_leader.initialize(config_.right.leader_profile);

    ensure_followers_connected();

    tea_teleop::LeaderCycleWorker left_worker(left_leader);
    tea_teleop::LeaderCycleWorker right_worker(right_leader);

    const double follower_target_hz =
        1000.0 / static_cast<double>(config_.teleop_period_ms);
    std::cout
        << (mode == TeleopMode::Slow ? "双臂慢速遥操作开始" : "双臂遥操作开始")
        << " | 双臂轮次目标=" << std::fixed << std::setprecision(1)
        << follower_target_hz << " Hz"
        << " | 左主臂目标=" << left_leader.control_frequency_hz() << " Hz"
        << " | 右主臂目标=" << right_leader.control_frequency_hz() << " Hz\n"
        << "主臂模式: COMPLIANT_DRAG + GRAVITY，Ctrl+C 结束\n";

    clear_interrupt();

    bool followers_stop_ok = true;
    bool left_worker_started = false;
    bool right_worker_started = false;
    bool left_leader_active = false;
    bool right_leader_active = false;

    auto safe_shutdown = [&]() noexcept {
        try {
            left_rm_.stop();
        }
        catch(const std::exception& error) {
            followers_stop_ok = false;
            std::cerr << "[警告] 左从臂停止失败: " << error.what() << "\n";
        }

        try {
            right_rm_.stop();
        }
        catch(const std::exception& error) {
            followers_stop_ok = false;
            std::cerr << "[警告] 右从臂停止失败: " << error.what() << "\n";
        }

        if(left_worker_started) {
            left_worker.stop();
            left_worker_started = false;
        }

        if(right_worker_started) {
            right_worker.stop();
            right_worker_started = false;
        }

        bool leaders_stop_ok = true;
        if(left_leader_active) {
            leaders_stop_ok &= left_leader.safe_deactivate();
            left_leader_active = false;
        }

        if(right_leader_active) {
            leaders_stop_ok &= right_leader.safe_deactivate();
            right_leader_active = false;
        }

        if(!leaders_stop_ok) {
            std::cerr << "[警告] 双主臂安全失能未完全成功\n";
        }

        return followers_stop_ok && leaders_stop_ok;
        };

    try {
        left_leader.activate(serial_arm::JointImpedanceMode::COMPLIANT_DRAG);
        left_leader_active = true;

        try {
            right_leader.activate(serial_arm::JointImpedanceMode::COMPLIANT_DRAG);
            right_leader_active = true;
        }
        catch(...) {
            (void)left_leader.safe_deactivate();
            left_leader_active = false;
            throw;
        }

        left_worker.start();
        left_worker_started = true;

        try {
            right_worker.start();
            right_worker_started = true;
        }
        catch(...) {
            left_worker.stop();
            left_worker_started = false;
            (void)left_leader.safe_deactivate();
            left_leader_active = false;
            (void)right_leader.safe_deactivate();
            right_leader_active = false;
            throw;
        }

        left_worker.wait_until_ready(std::chrono::milliseconds{ 500 });
        right_worker.wait_until_ready(std::chrono::milliseconds{ 500 });

        std::this_thread::sleep_for(std::chrono::milliseconds{ 200 });

        left_worker.rethrow_if_failed();
        right_worker.rethrow_if_failed();

        auto left_output = left_worker.latest();
        auto right_output = right_worker.latest();

        auto left_target = leader_joint_to_degree(
            left_output.joint_state.pos,
            config_.left);
        auto right_target = leader_joint_to_degree(
            right_output.joint_state.pos,
            config_.right);

        auto left_follower = left_rm_.read_all_degree();
        auto right_follower = right_rm_.read_all_degree();

        const auto left_initial_difference =
            summarize_difference(left_target, left_follower);
        const auto right_initial_difference =
            summarize_difference(right_target, right_follower);

        std::cout
            << "左侧初始最大角差: " << std::fixed << std::setprecision(1)
            << left_initial_difference.max_abs_degree << " deg (J"
            << (left_initial_difference.joint_index + 1U) << ")\n"
            << "右侧初始最大角差: "
            << right_initial_difference.max_abs_degree << " deg (J"
            << (right_initial_difference.joint_index + 1U) << ")\n";

        validate_start_error(
            "Left",
            left_target,
            left_follower);
        validate_start_error(
            "Right",
            right_target,
            right_follower);

        const bool needs_full_align =
            mode == TeleopMode::Full &&
            (left_initial_difference.max_abs_degree > kFullAlignToleranceDegree ||
                right_initial_difference.max_abs_degree > kFullAlignToleranceDegree);

        if(needs_full_align) {
            std::cout
                << "双从臂启动对齐开始 | Ctrl+C 中断\n";

            auto left_command = left_follower;
            auto right_command = right_follower;
            auto align_next_tick = std::chrono::steady_clock::now();
            const auto align_deadline = align_next_tick + kAlignTimeout;
            std::size_t align_settled_cycles = 0U;

            while(!interrupted()) {
                if(std::chrono::steady_clock::now() >= align_deadline) {
                    throw std::runtime_error("双从臂启动对齐超时");
                }

                left_worker.rethrow_if_failed();
                right_worker.rethrow_if_failed();

                left_output = left_worker.latest();
                right_output = right_worker.latest();

                left_target = leader_joint_to_degree(
                    left_output.joint_state.pos,
                    config_.left);
                right_target = leader_joint_to_degree(
                    right_output.joint_state.pos,
                    config_.right);

                step_toward(left_command, left_target, kAlignMaxStepDegree);
                step_toward(right_command, right_target, kAlignMaxStepDegree);

                left_rm_.write_all_degree(left_command, false);
                right_rm_.write_all_degree(right_command, false);

                const bool left_ok =
                    max_abs_difference(left_target, left_command) <=
                    kAlignSettleToleranceDegree;
                const bool right_ok =
                    max_abs_difference(right_target, right_command) <=
                    kAlignSettleToleranceDegree;

                align_settled_cycles =
                    (left_ok && right_ok) ? align_settled_cycles + 1U : 0U;

                if(align_settled_cycles >= 10U) break;

                align_next_tick += std::chrono::milliseconds{
                    config_.teleop_period_ms
                };
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

            left_worker.rethrow_if_failed();
            right_worker.rethrow_if_failed();
            left_output = left_worker.latest();
            right_output = right_worker.latest();
            left_target = leader_joint_to_degree(
                left_output.joint_state.pos,
                config_.left);
            right_target = leader_joint_to_degree(
                right_output.joint_state.pos,
                config_.right);
            left_follower = left_rm_.read_all_degree();
            right_follower = right_rm_.read_all_degree();

            std::cout
                << "双从臂启动对齐完成 | 左最大角差="
                << std::fixed << std::setprecision(1)
                << max_abs_difference(left_target, left_follower)
                << " deg | 右最大角差="
                << max_abs_difference(right_target, right_follower)
                << " deg\n";
        }

        auto left_previous = left_follower;
        auto right_previous = right_follower;
        const auto start_time = std::chrono::steady_clock::now();
        auto last_status = start_time;
        auto next_tick = start_time;
        const std::uint64_t left_leader_cycle_start = left_worker.cycle_count();
        const std::uint64_t right_leader_cycle_start = right_worker.cycle_count();
        const std::uint64_t left_leader_overrun_start = left_worker.overrun_count();
        const std::uint64_t right_leader_overrun_start = right_worker.overrun_count();
        std::uint64_t previous_send_count = 0U;
        std::uint64_t previous_left_leader_count = left_leader_cycle_start;
        std::uint64_t previous_right_leader_count = right_leader_cycle_start;
        std::uint64_t teleop_cycle_count = 0U;
        std::uint64_t teleop_overrun_count = 0U;

        while(!interrupted()) {
            const auto cycle_start = std::chrono::steady_clock::now();
            if(config_.teleop_duration_s >= 0 &&
                cycle_start - start_time >=
                std::chrono::seconds{ config_.teleop_duration_s }) {
                break;
            }

            left_worker.rethrow_if_failed();
            right_worker.rethrow_if_failed();

            left_output = left_worker.latest();
            right_output = right_worker.latest();

            left_target = leader_joint_to_degree(
                left_output.joint_state.pos,
                config_.left);
            right_target = leader_joint_to_degree(
                right_output.joint_state.pos,
                config_.right);

            const auto left_command =
                mode == TeleopMode::Slow ?
                limit_slow_command(left_target, left_previous) :
                left_target;

            const auto right_command =
                mode == TeleopMode::Slow ?
                limit_slow_command(right_target, right_previous) :
                right_target;

            left_rm_.write_all_degree(
                left_command,
                mode == TeleopMode::Full);

            right_rm_.write_all_degree(
                right_command,
                mode == TeleopMode::Full);

            left_previous = left_command;
            right_previous = right_command;
            ++teleop_cycle_count;

            const auto after_work = std::chrono::steady_clock::now();
            if(after_work - last_status >= kStatusInterval) {
                const double interval_s =
                    std::chrono::duration<double>(after_work - last_status).count();
                const double elapsed_s =
                    std::chrono::duration<double>(after_work - start_time).count();
                const std::uint64_t left_leader_count = left_worker.cycle_count();
                const std::uint64_t right_leader_count = right_worker.cycle_count();
                const double follower_hz =
                    static_cast<double>(teleop_cycle_count - previous_send_count) /
                    interval_s;
                const double left_leader_hz =
                    static_cast<double>(
                        left_leader_count - previous_left_leader_count) /
                    interval_s;
                const double right_leader_hz =
                    static_cast<double>(
                        right_leader_count - previous_right_leader_count) /
                    interval_s;

                std::cout
                    << "[双臂遥操作 " << std::fixed << std::setprecision(1)
                    << elapsed_s << " s] "
                    << "双臂轮次=" << follower_hz << " Hz"
                    << " | 左主臂=" << left_leader_hz << " Hz"
                    << " | 右主臂=" << right_leader_hz << " Hz"
                    << " | 超期 双臂=" << teleop_overrun_count
                    << " 左主臂=" << (left_worker.overrun_count() -
                        left_leader_overrun_start)
                    << " 右主臂=" << (right_worker.overrun_count() -
                        right_leader_overrun_start)
                    << "\n";

                previous_send_count = teleop_cycle_count;
                previous_left_leader_count = left_leader_count;
                previous_right_leader_count = right_leader_count;
                last_status = after_work;
            }

            next_tick += std::chrono::milliseconds{ config_.teleop_period_ms };
            if(next_tick > after_work) {
                std::this_thread::sleep_until(next_tick);
            }
            else {
                ++teleop_overrun_count;
                next_tick = after_work;
            }
        }

        const auto end_time = std::chrono::steady_clock::now();
        const double elapsed_s =
            std::chrono::duration<double>(end_time - start_time).count();
        const std::uint64_t left_leader_cycle_total =
            left_worker.cycle_count() - left_leader_cycle_start;
        const std::uint64_t right_leader_cycle_total =
            right_worker.cycle_count() - right_leader_cycle_start;
        const std::uint64_t left_leader_overrun_total =
            left_worker.overrun_count() - left_leader_overrun_start;
        const std::uint64_t right_leader_overrun_total =
            right_worker.overrun_count() - right_leader_overrun_start;
        const double follower_average_hz = elapsed_s > 0.0 ?
            static_cast<double>(teleop_cycle_count) / elapsed_s : 0.0;
        const double left_leader_average_hz = elapsed_s > 0.0 ?
            static_cast<double>(left_leader_cycle_total) / elapsed_s : 0.0;
        const double right_leader_average_hz = elapsed_s > 0.0 ?
            static_cast<double>(right_leader_cycle_total) / elapsed_s : 0.0;

        if(!safe_shutdown()) {
            throw std::runtime_error("双臂遥操作结束时安全停止未完全成功");
        }

        std::cout
            << "双臂遥操作结束 | 时长=" << std::fixed << std::setprecision(1)
            << elapsed_s << " s"
            << " | 双臂轮次平均=" << follower_average_hz << " Hz"
            << " 超期=" << teleop_overrun_count
            << " | 左主臂平均=" << left_leader_average_hz << " Hz"
            << " 超期=" << left_leader_overrun_total
            << " | 右主臂平均=" << right_leader_average_hz << " Hz"
            << " 超期=" << right_leader_overrun_total
            << "\n";
    }
    catch(...) {
        (void)safe_shutdown();
        clear_interrupt();
        throw;
    }

    clear_interrupt();
}

void DualTeaTeleop::ensure_followers_connected() {
    if(!left_rm_.is_connected()) {
        left_rm_.connect(
            config_.left.follower_ip,
            config_.left.follower_port);

        std::cout
            << "左从臂已连接: "
            << config_.left.follower_ip << ":"
            << config_.left.follower_port << "\n";
    }

    try {
        if(!right_rm_.is_connected()) {
            right_rm_.connect(
                config_.right.follower_ip,
                config_.right.follower_port);

            std::cout
                << "右从臂已连接: "
                << config_.right.follower_ip << ":"
                << config_.right.follower_port << "\n";
        }
    }
    catch(...) {
        left_rm_.disconnect();
        throw;
    }
}

std::array<float, kJointCount> DualTeaTeleop::leader_joint_to_degree(
    const std::vector<double>& joint_position,
    const DualArmConfig& arm) const {
    return joint_to_degree(joint_position, arm);
}

std::array<float, kJointCount> DualTeaTeleop::limit_slow_command(
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

void DualTeaTeleop::validate_start_error(
    const char* side,
    const std::array<float, kJointCount>& leader_degree,
    const std::array<float, kJointCount>& follower_degree) const {
    std::ostringstream error;
    bool failed = false;

    for(std::size_t i = 0; i < kJointCount; ++i) {
        const float diff = std::abs(leader_degree[i] - follower_degree[i]);
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
            std::string(side) +
            " 主从初始角差超过 " +
            std::to_string(config_.max_start_error_degree) +
            " deg，拒绝启动双臂遥操作: " + error.str());
    }
}

int main(int argc, char** argv) {
    try {
        std::string config_path;

        for(int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];

            if(arg == "--config") {
                if(i + 1 >= argc) {
                    std::cerr << "--config 后必须提供 YAML 路径\n";
                    return 2;
                }
                config_path = argv[++i];
            }
            else if(arg == "--help" || arg == "-h") {
                std::cout
                    << "用法: ros2 run tea_teleop dual_teleop "
                    << "[--config <dual_teleop.yaml>]\n";
                return 0;
            }
            else {
                std::cerr << "未知参数: " << arg << "\n";
                return 2;
            }
        }

        if(config_path.empty()) {
            config_path = default_dual_teleop_config_path();
        }

        DualTeleopConfig config = load_dual_teleop_config(config_path);
        DualTeaTeleop app(std::move(config));
        return app.run();
    }
    catch(const std::exception& error) {
        std::cerr << "[致命错误] " << error.what() << "\n";
        return 1;
    }
}

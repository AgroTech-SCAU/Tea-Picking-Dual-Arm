#include "tea_teleop/teleop.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "hiwonder_driver/hx10hm.hpp"
#include "serial_port/serial_port.hpp"

namespace {

constexpr std::size_t kJointCount = 6;
constexpr float kFullAlignToleranceDegree = 2.0F;

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
        throw std::runtime_error("stdin 已关闭");
    }
    return line;
}

int prompt_int(const std::string& prompt) {
    for(;;) {
        const std::string text = prompt_line(prompt);

        try {
            std::size_t parsed = 0;
            const int value = std::stoi(text, &parsed);
            if(parsed == text.size()) {
                return value;
            }
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
            if(parsed == text.size()) {
                return value;
            }
        }
        catch(const std::exception&) {
        }

        std::cout << "输入无效，请重新输入\n";
    }
}

bool confirm_token(const std::string& expected, const std::string& prompt) {
    return prompt_line(prompt) == expected;
}

bool prompt_yes_no(const std::string& prompt) {
    const std::string answer = prompt_line(prompt);
    return answer == "y" || answer == "Y" || answer == "yes" || answer == "YES";
}

SerialPort::Config make_serial_config() {
    SerialPort::Config config;
    config.baud_rate = Hx10hm::BAUDRATE;
    config.data_bits = 8;
    config.parity = SerialPort::Parity::None;
    config.stop_bits = SerialPort::StopBits::One;
    config.flow_control = SerialPort::FlowControl::None;
    config.read_timeout = std::chrono::milliseconds{ 10 };
    config.write_timeout = std::chrono::milliseconds{ 20 };
    config.flush_on_open = true;
    return config;
}

void print_degree_array(
    const std::string& name,
    const std::array<float, kJointCount>& value) {
    std::cout << name << "\n";

    for(std::size_t i = 0; i < value.size(); ++i) {
        std::cout
            << "  J" << (i + 1)
            << " = " << std::fixed << std::setprecision(2)
            << value[i] << " deg\n";
    }
}

void print_master_line(
    const std::array<std::uint16_t, kJointCount>& raw,
    const std::array<float, kJointCount>& degree) {
    std::cout << "MASTER  ";

    for(std::size_t i = 0; i < kJointCount; ++i) {
        std::cout
            << "J" << (i + 1)
            << "[" << raw[i]
            << "," << std::fixed << std::setprecision(1)
            << degree[i] << "deg] ";
    }
    std::cout << "\n";
}

void print_slave_line(const std::array<float, kJointCount>& degree) {
    std::cout << "SLAVE   ";

    for(std::size_t i = 0; i < kJointCount; ++i) {
        std::cout
            << "J" << (i + 1)
            << "[" << std::fixed << std::setprecision(1)
            << degree[i] << "deg] ";
    }
    std::cout << "\n";
}

void print_difference_line(
    const std::array<float, kJointCount>& master,
    const std::array<float, kJointCount>& slave) {
    std::cout << "DIFF    ";

    for(std::size_t i = 0; i < kJointCount; ++i) {
        std::cout
            << "J" << (i + 1)
            << "[" << std::fixed << std::setprecision(1)
            << (master[i] - slave[i]) << "deg] ";
    }
    std::cout << "\n";
}

bool all_master_near_zero(
    const std::array<std::uint16_t, kJointCount>& raw,
    float tolerance_degree) {
    for(const auto value : raw) {
        if(std::abs(static_cast<float>(Hx10hm::raw_to_degree(value))) > tolerance_degree) {
            return false;
        }
    }
    return true;
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

}  // namespace

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
                case 0:
                    return 0;

                case 1:
                    read_master_menu();
                    break;

                case 2:
                    read_slave_menu();
                    break;

                case 3:
                    read_compare_menu();
                    break;

                case 4:
                    home_master_menu();
                    break;

                case 5:
                    home_slave_menu();
                    break;

                case 6:
                    home_both_menu();
                    break;

                case 7:
                    teleop(TeleopMode::Slow);
                    break;

                case 8:
                    teleop(TeleopMode::Full);
                    break;

                case 9:
                    config_menu();
                    break;

                case 10:
                    software_stop();
                    break;

                default:
                    std::cout << "未知选项\n";
                    break;
            }
        }
        catch(const std::exception& e) {
            std::cerr << "[FAILED] " << e.what() << "\n";
        }
    }
}

void TeaTeleop::print_main_menu() const {
    std::cout
        << "\n============================================================\n"
        << " Tea-Picking-Dual-Arm Teleoperation\n"
        << "============================================================\n"
        << " Master : " << config_.serial_device << " @ " << Hx10hm::BAUDRATE << "\n"
        << " Slave  : " << config_.rm_ip << ":" << config_.rm_port << "\n"
        << " Duration: "
        << (config_.teleop_duration_s < 0
            ? std::string("-1 / until Ctrl+C")
            : std::to_string(config_.teleop_duration_s) + " s")
        << "\n"
        << " Direction: [";

    for(std::size_t i = 0; i < kJointCount; ++i) {
        std::cout << static_cast<int>(config_.mapping_direction[i]);
        if(i + 1 != kJointCount) {
            std::cout << ", ";
        }
    }

    std::cout
        << "]\n"
        << "------------------------------------------------------------\n"
        << " 1. 读取主臂\n"
        << " 2. 读取从臂\n"
        << " 3. 主从臂读取对照\n"
        << " 4. 主臂零位检查（人工摆零）\n"
        << " 5. 从臂慢速归零\n"
        << " 6. 主从臂归零准备\n"
        << " 7. 慢速 Teleop\n"
        << " 8. 全速 Teleop\n"
        << " 9. 修改运行配置\n"
        << "10. RM65-B 软件停止\n"
        << " 0. 退出\n"
        << "============================================================\n"
        << "说明: 慢速 Teleop = 上层单周期限速 + RM65-B CANFD 低跟随 follow=false\n"
        << "      全速 Teleop = 无上层单周期限速 + RM65-B CANFD 高跟随 follow=true\n"
        << "      连续读取/Teleop 运行中按 Ctrl+C 返回菜单\n";
}

void TeaTeleop::print_config() const {
    std::cout
        << "\n---------------- 当前配置 ----------------\n"
        << "serial_device              = " << config_.serial_device << "\n"
        << "rm_ip                      = " << config_.rm_ip << "\n"
        << "rm_port                    = " << config_.rm_port << "\n"
        << "teleop_duration_s          = " << config_.teleop_duration_s << "\n"
        << "read_print_period_ms       = " << config_.read_print_period_ms << "\n"
        << "teleop_period_ms           = " << config_.teleop_period_ms << "\n"
        << "slow_max_step_degree       = " << config_.slow_max_step_degree << "\n"
        << "max_start_error_degree     = " << config_.max_start_error_degree << " (安全阈值)\n"
        << "master_zero_tolerance_deg  = " << config_.master_home_tolerance_degree << "\n"
        << "slave_home_speed_percent   = " << config_.slave_home_speed_percent << "%\n"
        << "mapping_direction          = [";

    for(std::size_t i = 0; i < kJointCount; ++i) {
        std::cout << static_cast<int>(config_.mapping_direction[i]);
        if(i + 1 != kJointCount) {
            std::cout << ", ";
        }
    }
    std::cout << "]\n------------------------------------------\n";
}

void TeaTeleop::config_menu() {
    for(;;) {
        print_config();

        std::cout
            << " 1. 修改主臂串口\n"
            << " 2. 修改 RM65-B IP\n"
            << " 3. 修改 RM65-B 端口\n"
            << " 4. 修改 Teleop 持续时间 (-1=无限)\n"
            << " 5. 修改关节映射方向\n"
            << " 6. 修改连续读取打印周期\n"
            << " 7. 修改 Teleop 期望周期\n"
            << " 8. 修改慢速 Teleop 单周期最大角度\n"
            << " 9. 修改主臂人工零位检查容差\n"
            << "10. 修改从臂归零/对齐速度百分比\n"
            << "11. 恢复默认配置\n"
            << " 0. 返回\n";

        const int option = prompt_int("选择配置项: ");

        switch(option) {
            case 0:
                return;

            case 1: {
                const std::string value = prompt_line("新的串口设备: ");
                if(!value.empty()) {
                    config_.serial_device = value;
                }
                break;
            }

            case 2: {
                const std::string value = prompt_line("新的 RM65-B IP: ");
                if(!value.empty()) {
                    rm_.disconnect();
                    config_.rm_ip = value;
                }
                break;
            }

            case 3: {
                const int value = prompt_int("新的 TCP 端口 [1~65535]: ");
                if(value < 1 || value > 65535) {
                    std::cout << "端口范围无效\n";
                    break;
                }
                rm_.disconnect();
                config_.rm_port = value;
                break;
            }

            case 4: {
                const int value = prompt_int("Teleop 持续时间 s (-1 或 >=1): ");
                if(value == -1 || value >= 1) {
                    config_.teleop_duration_s = value;
                }
                else {
                    std::cout << "只允许 -1 或正整数\n";
                }
                break;
            }

            case 5:
                mapping_direction_menu();
                break;

            case 6: {
                const int value = prompt_int("连续读取打印周期 ms [20~2000]: ");
                if(value >= 20 && value <= 2000) {
                    config_.read_print_period_ms = value;
                }
                else {
                    std::cout << "范围无效\n";
                }
                break;
            }

            case 7: {
                const int value = prompt_int("Teleop 期望周期 ms [5~100]: ");
                if(value >= 5 && value <= 100) {
                    config_.teleop_period_ms = value;
                }
                else {
                    std::cout << "范围无效\n";
                }
                break;
            }

            case 8: {
                const float value = prompt_float("慢速模式单周期最大变化 deg (0, 5]: ");
                if(value > 0.0F && value <= 5.0F) {
                    config_.slow_max_step_degree = value;
                }
                else {
                    std::cout << "范围无效\n";
                }
                break;
            }

            case 9: {
                const float value = prompt_float("主臂人工零位检查容差 deg [0.5~10]: ");
                if(value >= 0.5F && value <= 10.0F) {
                    config_.master_home_tolerance_degree = value;
                }
                else {
                    std::cout << "范围无效，仅允许 0.5~10 deg\n";
                }
                break;
            }

            case 10: {
                const int value = prompt_int("从臂归零/对齐速度百分比 [1~30]: ");
                if(value >= 1 && value <= 30) {
                    config_.slave_home_speed_percent = value;
                }
                else {
                    std::cout << "为保证归零安全，菜单仅允许 1~30%\n";
                }
                break;
            }

            case 11:
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
                << "=" << static_cast<int>(config_.mapping_direction[i])
                << " ";
        }
        std::cout << "\n";

        const int joint = prompt_int("选择关节 [1~6]，0 返回，7 恢复默认方向: ");
        if(joint == 0) {
            return;
        }
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

void TeaTeleop::read_master_menu() {
    std::cout
        << "\n1. 单次读取打印\n"
        << "2. 持续读取打印\n"
        << "0. 返回\n";

    const int option = prompt_int("选择主臂读取方式: ");
    if(option == 1) {
        read_master(ReadMode::Once);
    }
    else if(option == 2) {
        read_master(ReadMode::Continuous);
    }
}

void TeaTeleop::read_slave_menu() {
    std::cout
        << "\n1. 单次读取打印\n"
        << "2. 持续读取打印\n"
        << "0. 返回\n";

    const int option = prompt_int("选择从臂读取方式: ");
    if(option == 1) {
        read_slave(ReadMode::Once);
    }
    else if(option == 2) {
        read_slave(ReadMode::Continuous);
    }
}

void TeaTeleop::read_compare_menu() {
    std::cout
        << "\n1. 单次读取对照\n"
        << "2. 持续读取对照\n"
        << "0. 返回\n";

    const int option = prompt_int("选择主从对照方式: ");
    if(option == 1) {
        read_compare(ReadMode::Once);
    }
    else if(option == 2) {
        read_compare(ReadMode::Continuous);
    }
}

void TeaTeleop::read_master(ReadMode mode) {
    SerialPort serial(config_.serial_device, make_serial_config());
    Hx10hm master(serial);

    clear_interrupt();

    do {
        const auto raw = master.read_all_pos_raw();
        const auto degree = master_raw_to_degree(raw);
        print_master_line(raw, degree);

        if(mode == ReadMode::Once) {
            break;
        }

        std::this_thread::sleep_for(
            std::chrono::milliseconds{ config_.read_print_period_ms });
    }
    while(!interrupted());

    if(interrupted()) {
        std::cout << "主臂持续读取已中断\n";
    }
    clear_interrupt();
}

void TeaTeleop::read_slave(ReadMode mode) {
    ensure_rm_connected();
    clear_interrupt();

    do {
        print_slave_line(rm_.read_all_degree());

        if(mode == ReadMode::Once) {
            break;
        }

        std::this_thread::sleep_for(
            std::chrono::milliseconds{ config_.read_print_period_ms });
    }
    while(!interrupted());

    if(interrupted()) {
        std::cout << "从臂持续读取已中断\n";
    }
    clear_interrupt();
}

void TeaTeleop::read_compare(ReadMode mode) {
    SerialPort serial(config_.serial_device, make_serial_config());
    Hx10hm master(serial);
    ensure_rm_connected();
    clear_interrupt();

    do {
        const auto raw = master.read_all_pos_raw();
        const auto master_degree = master_raw_to_degree(raw);
        const auto slave_degree = rm_.read_all_degree();

        print_master_line(raw, master_degree);
        print_slave_line(slave_degree);
        print_difference_line(master_degree, slave_degree);

        if(mode == ReadMode::Once) {
            break;
        }

        std::this_thread::sleep_for(
            std::chrono::milliseconds{ config_.read_print_period_ms });
    }
    while(!interrupted());

    if(interrupted()) {
        std::cout << "主从持续对照已中断\n";
    }
    clear_interrupt();
}

void TeaTeleop::home_master_menu() {
    SerialPort serial(config_.serial_device, make_serial_config());
    Hx10hm master(serial);
    (void)home_master(master, true);
}

void TeaTeleop::home_slave_menu() {
    (void)home_slave(true);
}

void TeaTeleop::home_both_menu() {
    if(!confirm_token(
        "HOME_BOTH",
        "从臂将慢速回零，主臂需要人工摆到 2048 零位附近，确认安全后输入 HOME_BOTH: ")) {
        std::cout << "已取消\n";
        return;
    }

    if(!home_slave(false)) {
        return;
    }

    SerialPort serial(config_.serial_device, make_serial_config());
    Hx10hm master(serial);
    (void)home_master(master, false);
}

bool TeaTeleop::home_master(Hx10hm& master, bool require_confirmation) {
    if(require_confirmation &&
        !confirm_token(
            "CHECK_MASTER",
            "请手动将主臂摆到零位附近，输入 CHECK_MASTER 开始检查: ")) {
        std::cout << "已取消\n";
        return false;
    }

    std::cout
        << "主臂零位检查已开始\n"
        << "- HX-10HM 全程只读，不发送 Torque/Position/Mode 写命令\n"
        << "- 请手动移动各关节，使其进入 +/- "
        << config_.master_home_tolerance_degree
        << " deg 范围\n"
        << "- 按 Ctrl+C 可取消并返回菜单\n";

    clear_interrupt();

    while(!interrupted()) {
        const auto raw = master.read_all_pos_raw();
        const auto degree = master_raw_to_degree(raw);
        print_master_line(raw, degree);

        if(all_master_near_zero(raw, config_.master_home_tolerance_degree)) {
            std::cout << "主臂已进入人工零位允许范围\n";
            clear_interrupt();
            return true;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds{ 200 });
    }

    std::cout << "主臂零位检查已由用户中断\n";
    clear_interrupt();
    return false;
}

bool TeaTeleop::home_slave(bool require_confirmation) {
    ensure_rm_connected();

    if(require_confirmation &&
        !confirm_token(
            "HOME_SLAVE",
            "RM65-B 将使用规划运动慢速回到 [0,0,0,0,0,0] deg，输入 HOME_SLAVE: ")) {
        std::cout << "已取消\n";
        return false;
    }

    const auto before = rm_.read_all_degree();
    print_degree_array("RM65-B before home", before);

    const std::array<float, kJointCount> zero{};
    rm_.movej_degree(zero, config_.slave_home_speed_percent, true);

    const auto after = rm_.read_all_degree();
    print_degree_array("RM65-B after home", after);

    std::cout << "从臂归零完成\n";
    return true;
}

void TeaTeleop::teleop(TeleopMode mode) {
    SerialPort serial(config_.serial_device, make_serial_config());
    Hx10hm master(serial);
    ensure_rm_connected();

    std::cout
        << "\nTeleop 启动前检查\n"
        << "- 主臂 2048 对应 0 deg\n"
        << "- 不归零时，任一关节主从初始角差 > "
        << config_.max_start_error_degree
        << " deg 将拒绝启动\n";

    const bool return_zero = prompt_yes_no("开始 Teleop 前是否执行归零准备（从臂回零 + 主臂人工摆零）? [y/N]: ");

    if(return_zero) {
        if(!confirm_token(
            "ZERO",
            "确认从臂回零路径安全，并准备人工摆正主臂后输入 ZERO: ")) {
            std::cout << "已取消 Teleop\n";
            return;
        }

        // 先让从臂规划回零，再等待用户人工摆正主臂
        if(!home_slave(false)) {
            return;
        }
        if(!home_master(master, false)) {
            return;
        }
    }

    auto first_raw = master.read_all_pos_raw();
    auto first_target = master_raw_to_degree(first_raw);
    auto slave_start = rm_.read_all_degree();

    print_master_line(first_raw, first_target);
    print_slave_line(slave_start);
    print_difference_line(first_target, slave_start);

    validate_start_error(first_target, slave_start);

    std::cout
        << "模式: "
        << (mode == TeleopMode::Slow ? "慢速" : "全速")
        << "\n持续时间: "
        << (config_.teleop_duration_s < 0
            ? std::string("-1 / Ctrl+C 停止")
            : std::to_string(config_.teleop_duration_s) + " s")
        << "\n期望周期: " << config_.teleop_period_ms << " ms\n";

    if(mode == TeleopMode::Slow) {
        std::cout
            << "慢速模式单周期最大变化: "
            << config_.slow_max_step_degree
            << " deg\n";
    }
    else {
        std::cout
            << "全速模式不做上层单周期角度限速\n"
            << "RM65-B CANFD 使用高跟随 follow=true\n"
            << "启动前若存在小于等于 30 deg 的初始角差，将先使用规划运动慢速对齐\n";
    }

    const std::string token =
        mode == TeleopMode::Slow ? "SLOW" : "FULL";

    if(!confirm_token(
        token,
        "确认实体急停可用且机械臂周围安全后输入 " + token + ": ")) {
        std::cout << "已取消\n";
        return;
    }

    // 用户确认后重新读取，避免确认过程中主臂姿态已经变化
    first_raw = master.read_all_pos_raw();
    first_target = master_raw_to_degree(first_raw);
    slave_start = rm_.read_all_degree();
    validate_start_error(first_target, slave_start);

    if(mode == TeleopMode::Full &&
        max_abs_difference(first_target, slave_start) > kFullAlignToleranceDegree) {
        std::cout
            << "全速模式启动前先慢速对齐从臂到当前主臂映射姿态\n";
        rm_.movej_degree(
            first_target,
            config_.slave_home_speed_percent,
            true);
        slave_start = rm_.read_all_degree();
    }

    clear_interrupt();

    auto previous_command = slave_start;
    auto next_tick = std::chrono::steady_clock::now();
    const auto start_time = next_tick;
    auto last_status = start_time;
    std::uint64_t cycle_count = 0;
    std::uint64_t overrun_count = 0;

    try {
        while(!interrupted()) {
            const auto now = std::chrono::steady_clock::now();

            if(config_.teleop_duration_s >= 0 &&
                now - start_time >= std::chrono::seconds{ config_.teleop_duration_s }) {
                break;
            }

            const auto raw = master.read_all_pos_raw();
            const auto target = master_raw_to_degree(raw);

            const auto command =
                mode == TeleopMode::Slow
                ? limit_slow_command(target, previous_command)
                : target;

            // 慢速模式: follow=false，保留控制器低跟随，并叠加上层单周期角度限速
            // 全速模式: follow=true，取消上层角度限速，直接使用 RM65-B CANFD 高跟随
            const bool high_follow = mode == TeleopMode::Full;
            rm_.write_all_degree(command, high_follow);
            previous_command = command;
            ++cycle_count;

            const auto status_now = std::chrono::steady_clock::now();
            if(status_now - last_status >= std::chrono::seconds{ 1 }) {
                const auto elapsed_ms =
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        status_now - start_time)
                    .count();

                std::cout
                    << "[Teleop] elapsed=" << elapsed_ms / 1000.0
                    << "s cycles=" << cycle_count
                    << " overruns=" << overrun_count
                    << " target_J1=" << std::fixed << std::setprecision(1)
                    << target[0] << "deg\n";

                last_status = status_now;
            }

            next_tick += std::chrono::milliseconds{ config_.teleop_period_ms };
            const auto after_work = std::chrono::steady_clock::now();

            if(next_tick > after_work) {
                std::this_thread::sleep_until(next_tick);
            }
            else {
                ++overrun_count;
                next_tick = after_work;
            }
        }
    }
    catch(...) {
        // 通信异常时尝试停止当前 RM 轨迹，随后把原始异常继续抛给菜单层
        try {
            rm_.stop();
        }
        catch(const std::exception&) {
        }
        clear_interrupt();
        throw;
    }

    if(interrupted()) {
        std::cout << "Teleop 已由 Ctrl+C 中断\n";
    }
    else {
        std::cout << "Teleop 已达到配置持续时间\n";
    }

    std::cout
        << "Teleop 结束: cycles=" << cycle_count
        << " overruns=" << overrun_count << "\n";

    clear_interrupt();
}

void TeaTeleop::software_stop() {
    if(!confirm_token(
        "STOP",
        "确认要发送 RM65-B 软件停止命令后输入 STOP: ")) {
        std::cout << "已取消\n";
        return;
    }

    ensure_rm_connected();
    rm_.stop();
    std::cout << "RM65-B 软件停止命令已发送\n";
}

void TeaTeleop::ensure_rm_connected() {
    if(!rm_.is_connected()) {
        rm_.connect(config_.rm_ip, config_.rm_port);
        std::cout
            << "RM65-B connection established: "
            << config_.rm_ip << ":" << config_.rm_port << "\n";
    }
}

std::array<float, kJointCount> TeaTeleop::master_raw_to_degree(
    const std::array<std::uint16_t, kJointCount>& raw) const {
    std::array<float, kJointCount> degree{};

    for(std::size_t i = 0; i < kJointCount; ++i) {
        degree[i] =
            config_.mapping_direction[i] *
            static_cast<float>(Hx10hm::raw_to_degree(raw[i]));
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
    const std::array<float, kJointCount>& master_degree,
    const std::array<float, kJointCount>& slave_degree) const {
    std::ostringstream error;
    bool failed = false;

    for(std::size_t i = 0; i < kJointCount; ++i) {
        const float diff = std::abs(master_degree[i] - slave_degree[i]);
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
            " deg，拒绝启动 Teleop: " + error.str());
    }
}

int main(int argc, char** argv) {
    TeleopConfig config;

    // 保留 teleop_test.cpp 原先的命令行兼容方式：
    // argv[1] = RM65-B IP
    // argv[2] = HX-10HM 串口设备
    if(argc >= 2) {
        config.rm_ip = argv[1];
    }
    if(argc >= 3) {
        config.serial_device = argv[2];
    }

    try {
        TeaTeleop app(config);
        return app.run();
    }
    catch(const std::exception& e) {
        std::cerr << "[FATAL] " << e.what() << "\n";
        return 1;
    }
}

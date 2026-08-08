#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

#include "hiwonder_driver/hx10hm.hpp"
#include "rm_bringup/rm65b.hpp"
#include "serial_port/serial_port.hpp"

namespace {

constexpr std::size_t kJointCount = 6;

// 当前主臂机械安装方向尚未最终标定
constexpr std::array<float, kJointCount> kDirection{
    1.0F,
    1.0F,
    -1.0F,
    1.0F,
    1.0F,
    1.0F,
};

constexpr auto kTeleopPeriod = std::chrono::milliseconds{ 20 };
constexpr float kMaxStepDegree = 1.0F;
constexpr float kMaxStartErrorDegree = 30.0F;
constexpr float kMaxDeviationFromStartDegree = 15.0F;

std::atomic_bool g_stop_requested{ false };

void signal_handler(int) {
    g_stop_requested.store(true);
}

std::string prompt_line(const std::string& prompt) {
    std::cout << prompt;

    std::string line;
    std::getline(std::cin, line);
    return line;
}

int prompt_int(const std::string& prompt) {
    for(;;) {
        try {
            return std::stoi(prompt_line(prompt));
        }
        catch(const std::exception&) {
            std::cout << "输入无效，请重新输入\n";
        }
    }
}

float prompt_float(const std::string& prompt) {
    for(;;) {
        try {
            return std::stof(prompt_line(prompt));
        }
        catch(const std::exception&) {
            std::cout << "输入无效，请重新输入\n";
        }
    }
}

bool confirm(const std::string& expected, const std::string& prompt) {
    return prompt_line(prompt) == expected;
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

std::array<float, kJointCount> master_raw_to_degree(
    const std::array<std::uint16_t, kJointCount>& raw) {
    std::array<float, kJointCount> degree{};

    for(std::size_t i = 0; i < raw.size(); ++i) {
        degree[i] =
            kDirection[i] *
            static_cast<float>(Hx10hm::raw_to_degree(raw[i]));
    }

    return degree;
}

void ensure_rm_connected(Rm65bBringup& rm, const std::string& rm_ip) {
    if(!rm.is_connected()) {
        rm.connect(rm_ip, 8080);
        std::cout << "RM65-B connection established; it will be reused until exit\n";
    }
}

std::array<float, kJointCount> limit_command(
    const std::array<float, kJointCount>& target,
    const std::array<float, kJointCount>& previous,
    const std::array<float, kJointCount>& start) {
    std::array<float, kJointCount> command{};

    for(std::size_t i = 0; i < target.size(); ++i) {
        const float workspace_limited = std::clamp(
            target[i],
            start[i] - kMaxDeviationFromStartDegree,
            start[i] + kMaxDeviationFromStartDegree);

        command[i] = std::clamp(
            workspace_limited,
            previous[i] - kMaxStepDegree,
            previous[i] + kMaxStepDegree);
    }

    return command;
}

void test_serial(const std::string& serial_device) {
    SerialPort serial(serial_device, make_serial_config());

    std::cout
        << "串口打开成功\n"
        << "device = " << serial.port() << "\n"
        << "baud   = " << serial.config().baud_rate << "\n";
}

void test_hx_single(const std::string& serial_device) {
    const int id = prompt_int("舵机 ID [1~6]: ");
    if(id < 1 || id > 6) {
        throw std::invalid_argument("V1 主臂舵机 ID 必须为 1~6");
    }

    SerialPort serial(serial_device, make_serial_config());
    Hx10hm master(serial);

    const auto raw = master.read_pos_raw(static_cast<std::uint8_t>(id));
    const auto degree = Hx10hm::raw_to_degree(raw);

    std::cout
        << "ID     = " << id << "\n"
        << "raw    = " << raw << "\n"
        << "degree = " << std::fixed << std::setprecision(2)
        << degree << " deg\n";
}

void test_hx_all(const std::string& serial_device) {
    SerialPort serial(serial_device, make_serial_config());
    Hx10hm master(serial);

    const auto raw = master.read_all_pos_raw();

    for(std::size_t i = 0; i < raw.size(); ++i) {
        std::cout
            << "J" << (i + 1)
            << " raw=" << raw[i]
            << " degree=" << std::fixed << std::setprecision(2)
            << Hx10hm::raw_to_degree(raw[i])
            << " deg\n";
    }
}

void test_mapping_preview(const std::string& serial_device) {
    SerialPort serial(serial_device, make_serial_config());
    Hx10hm master(serial);

    const auto raw = master.read_all_pos_raw();
    const auto target = master_raw_to_degree(raw);

    std::cout << "该测试不会连接 RM65-B\n";

    for(std::size_t i = 0; i < raw.size(); ++i) {
        std::cout
            << "J" << (i + 1)
            << " raw=" << raw[i]
            << " mapped=" << std::fixed << std::setprecision(2)
            << target[i] << " deg"
            << " direction=" << kDirection[i]
            << "\n";
    }
}

void test_rm_read(Rm65bBringup& rm, const std::string& rm_ip) {
    ensure_rm_connected(rm, rm_ip);

    const auto joint = rm.read_all_degree();
    print_degree_array("RM65-B current joint", joint);
}

void test_rm_small_step(Rm65bBringup& rm, const std::string& rm_ip) {
    ensure_rm_connected(rm, rm_ip);

    const auto current = rm.read_all_degree();
    print_degree_array("RM65-B current joint", current);

    const int joint_id = prompt_int("测试关节 [1~6]: ");
    if(joint_id < 1 || joint_id > 6) {
        throw std::invalid_argument("关节编号必须为 1~6");
    }

    const float delta = prompt_float("变化角度 [-1.0, 1.0] deg: ");
    if(std::abs(delta) > 1.0F) {
        throw std::invalid_argument("首次测试单次变化不能超过 1 deg");
    }

    auto target = current;
    target[static_cast<std::size_t>(joint_id - 1)] += delta;

    print_degree_array("RM65-B target joint", target);

    if(!confirm("MOVE", "确认机械臂周围安全后输入 MOVE: ")) {
        std::cout << "已取消\n";
        return;
    }

    rm.write_all_degree(target, false);
    std::this_thread::sleep_for(std::chrono::milliseconds{ 500 });

    const auto after = rm.read_all_degree();
    print_degree_array("RM65-B after command", after);
}

void test_master_slave_compare(
    Rm65bBringup& rm,
    const std::string& serial_device,
    const std::string& rm_ip) {
    SerialPort serial(serial_device, make_serial_config());
    Hx10hm master(serial);

    ensure_rm_connected(rm, rm_ip);

    const auto raw = master.read_all_pos_raw();
    const auto master_degree = master_raw_to_degree(raw);
    const auto slave_degree = rm.read_all_degree();

    std::cout << "该测试只读，不发送运动命令\n";
    print_degree_array("Master mapped joint", master_degree);
    print_degree_array("RM65-B current joint", slave_degree);

    std::cout << "Difference master - slave\n";
    for(std::size_t i = 0; i < kJointCount; ++i) {
        std::cout
            << "  J" << (i + 1)
            << " = " << std::fixed << std::setprecision(2)
            << (master_degree[i] - slave_degree[i])
            << " deg\n";
    }
}

void test_low_follow_teleop(
    Rm65bBringup& rm,
    const std::string& serial_device,
    const std::string& rm_ip) {
    SerialPort serial(serial_device, make_serial_config());
    Hx10hm master(serial);

    ensure_rm_connected(rm, rm_ip);

    const auto first_raw = master.read_all_pos_raw();
    const auto first_target = master_raw_to_degree(first_raw);
    const auto start = rm.read_all_degree();

    print_degree_array("Master initial target", first_target);
    print_degree_array("RM65-B initial joint", start);

    for(std::size_t i = 0; i < kJointCount; ++i) {
        const float error = std::abs(first_target[i] - start[i]);

        if(error > kMaxStartErrorDegree) {
            throw std::runtime_error(
                "J" + std::to_string(i + 1) +
                " 主从初始角差超过 " +
                std::to_string(kMaxStartErrorDegree) +
                " deg，拒绝启动连续遥操作");
        }
    }

    int duration_s = prompt_int("测试时长 [1~120] s: ");
    duration_s = std::clamp(duration_s, 1, 120);

    if(!confirm("TELEOP", "确认实体急停可用且机械臂周围安全后输入 TELEOP: ")) {
        std::cout << "已取消\n";
        return;
    }

    g_stop_requested.store(false);

    auto previous = start;
    auto next_tick = std::chrono::steady_clock::now();
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds{ duration_s };

    try {
        while(
            !g_stop_requested.load() &&
            std::chrono::steady_clock::now() < deadline) {
            next_tick += kTeleopPeriod;

            const auto raw = master.read_all_pos_raw();
            const auto target = master_raw_to_degree(raw);
            const auto command = limit_command(target, previous, start);

            // V1 固定低跟随
            rm.write_all_degree(command, false);
            previous = command;

            std::this_thread::sleep_until(next_tick);
        }
    }
    catch(...) {
        try {
            rm.stop();
        }
        catch(const std::exception&) {
        }
        throw;
    }

    std::cout << "连续遥操作测试结束\n";
}

void test_rm_stop(Rm65bBringup& rm, const std::string& rm_ip) {
    if(!confirm("STOP", "确认要发送 RM65-B 软件停止命令后输入 STOP: ")) {
        std::cout << "已取消\n";
        return;
    }

    ensure_rm_connected(rm, rm_ip);
    rm.stop();

    std::cout << "停止命令已发送\n";
}

void print_menu(
    const std::string& serial_device,
    const std::string& rm_ip) {
    std::cout
        << "\n==================================================\n"
        << " Tea-Picking-Dual-Arm V1 Test Menu\n"
        << "==================================================\n"
        << " serial: " << serial_device << "\n"
        << " RM65-B: " << rm_ip << ":8080\n"
        << "--------------------------------------------------\n"
        << " 1. 串口打开测试\n"
        << " 2. HX-10HM 单舵机读取\n"
        << " 3. HX-10HM 六舵机读取\n"
        << " 4. 主臂角度映射预览\n"
        << " 5. RM65-B 连接与关节只读\n"
        << " 6. RM65-B 单关节 +/-1 deg 小步运动\n"
        << " 7. 主臂 / 从臂只读对照\n"
        << " 8. 低跟随遥操作测试\n"
        << " 9. RM65-B 停止命令\n"
        << " 0. 退出\n"
        << "==================================================\n";
}

}  // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, signal_handler);

    std::string rm_ip = "192.168.1.18";
    std::string serial_device = "/dev/ttyACM0";

    if(argc >= 2) {
        rm_ip = argv[1];
    }
    if(argc >= 3) {
        serial_device = argv[2];
    }

    Rm65bBringup rm;

    for(;;) {
        print_menu(serial_device, rm_ip);

        const int option = prompt_int("选择测试项: ");

        if(option == 0) {
            return 0;
        }

        try {
            switch(option) {
                case 1:
                    test_serial(serial_device);
                    break;

                case 2:
                    test_hx_single(serial_device);
                    break;

                case 3:
                    test_hx_all(serial_device);
                    break;

                case 4:
                    test_mapping_preview(serial_device);
                    break;

                case 5:
                    test_rm_read(rm, rm_ip);
                    break;

                case 6:
                    test_rm_small_step(rm, rm_ip);
                    break;

                case 7:
                    test_master_slave_compare(rm, serial_device, rm_ip);
                    break;

                case 8:
                    test_low_follow_teleop(rm, serial_device, rm_ip);
                    break;

                case 9:
                    test_rm_stop(rm, rm_ip);
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

#include "serial_arm_hardware_hiwonder/hx10hm_motor_bus.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstddef>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

constexpr double NOMINAL_MAX_EFFORT_NM = 0.2941995;
constexpr std::int16_t NOMINAL_MAX_PWM = 300;

volatile std::sig_atomic_t stop_requested = 0;

enum class Mode {
    ReadOnly,
    TauOnly,
    Damping,
    Benchmark,
};

struct Options {
    std::string config_path;
    std::string csv_path;
    Mode mode{ Mode::ReadOnly };
    double duration_s{ 10.0 };
    double tau_nm{ 0.0 };
    double kd{ 0.01 };
    double max_current_ma{ 0.0 };
    double max_temperature_c{ 0.0 };
    std::chrono::milliseconds period{ 10 };
    std::size_t joint_index{ 0U };
    bool joint_was_set{ false };
    bool tau_was_set{ false };
    bool confirm_motion{ false };
    bool show_help{ false };
};

struct CycleStats {
    std::size_t sample_count{ 0U };
    std::size_t timeout_count{ 0U };
    std::size_t overrun_count{ 0U };
    double total_cycle_us{ 0.0 };
    double min_cycle_us{ std::numeric_limits<double>::infinity() };
    double max_cycle_us{ 0.0 };
};

// ! ========================= 私 有 量 / 工 具 函 数 实 现 ========================= ! //

/**
 * @brief 仅在 signal handler 中设置停止标志
 */
void handle_signal(int) noexcept {
    stop_requested = 1;
}

/**
 * @brief 输出命令行帮助
 */
void print_usage(const char* executable) {
    std::cout
        << "Usage:\n"
        << "  " << executable << " --config FILE --mode read [options]\n"
        << "  " << executable
        << " --config FILE --mode tau --joint 1..6 --tau NM --confirm-motion [options]\n"
        << "  " << executable
        << " --config FILE --mode damping --joint 1..6 [--kd VALUE] --confirm-motion [options]\n"
        << "  " << executable
        << " --config FILE --mode benchmark --confirm-motion [options]\n\n"
        << "Modes:\n"
        << "  read       Torque-OFF six-axis state and diagnostics read\n"
        << "  tau        Single-joint tau-only test; every other joint commands zero\n"
        << "  damping    Single-joint kd-only test with kp=tor=vel_desired=0\n"
        << "  benchmark  Six-axis read + zero MIT + SYNC WRITE timing benchmark\n\n"
        << "Options:\n"
        << "  --duration SEC          Test duration, (0, 60], default 10\n"
        << "  --period-ms MS          Requested loop period, default 10\n"
        << "  --csv FILE              Optional per-joint CSV output\n"
        << "  --max-current-ma MA     Optional immediate-stop threshold\n"
        << "  --max-temperature-c C   Optional immediate-stop threshold\n"
        << "  --confirm-motion        Required for every Torque-ON mode\n"
        << "  --help                  Show this message\n\n"
        << "Hard tool limits: |tau| <= 0.2941995 N.m and |PWM| <= 300.\n";
}

/**
 * @brief 将字符串解析为有限 double
 * @param text 输入字符串
 * @param name 选项名称
 * @return 解析值
 */
double parse_double(const std::string& text, const char* name) {
    std::size_t parsed = 0U;
    const double value = std::stod(text, &parsed);
    if(parsed != text.size() || !std::isfinite(value)) {
        throw std::invalid_argument(std::string("invalid ") + name);
    }
    return value;
}

/**
 * @brief 将字符串解析为正整数
 * @param text 输入字符串
 * @param name 选项名称
 * @return 解析值
 */
std::size_t parse_size(const std::string& text, const char* name) {
    std::size_t parsed = 0U;
    const unsigned long long value = std::stoull(text, &parsed);
    if(parsed != text.size() || value == 0ULL ||
        value > static_cast<unsigned long long>(std::numeric_limits<std::size_t>::max())) {
        throw std::invalid_argument(std::string("invalid ") + name);
    }
    return static_cast<std::size_t>(value);
}

/**
 * @brief 读取下一个命令行参数
 */
std::string require_value(int argc, char** argv, int& index, const char* option) {
    if(index + 1 >= argc) throw std::invalid_argument(std::string("missing value for ") + option);
    return argv[++index];
}

/**
 * @brief 解析并校验命令行选项
 */
Options parse_options(int argc, char** argv) {
    Options options;
    for(int i = 1; i < argc; ++i) {
        const std::string argument = argv[i];
        if(argument == "--help" || argument == "-h") {
            options.show_help = true;
        }
        else if(argument == "--config") {
            options.config_path = require_value(argc, argv, i, "--config");
        }
        else if(argument == "--mode") {
            const std::string mode = require_value(argc, argv, i, "--mode");
            if(mode == "read") options.mode = Mode::ReadOnly;
            else if(mode == "tau") options.mode = Mode::TauOnly;
            else if(mode == "damping") options.mode = Mode::Damping;
            else if(mode == "benchmark") options.mode = Mode::Benchmark;
            else throw std::invalid_argument("--mode must be read, tau, damping, or benchmark");
        }
        else if(argument == "--duration") {
            options.duration_s = parse_double(
                require_value(argc, argv, i, "--duration"), "--duration");
        }
        else if(argument == "--period-ms") {
            const auto value = parse_size(
                require_value(argc, argv, i, "--period-ms"), "--period-ms");
            if(value > 1000U) throw std::invalid_argument("--period-ms must be <= 1000");
            options.period = std::chrono::milliseconds(value);
        }
        else if(argument == "--joint") {
            const auto value = parse_size(require_value(argc, argv, i, "--joint"), "--joint");
            if(value > 6U) throw std::invalid_argument("--joint must be in [1, 6]");
            options.joint_index = value - 1U;
            options.joint_was_set = true;
        }
        else if(argument == "--tau") {
            options.tau_nm = parse_double(require_value(argc, argv, i, "--tau"), "--tau");
            options.tau_was_set = true;
        }
        else if(argument == "--kd") {
            options.kd = parse_double(require_value(argc, argv, i, "--kd"), "--kd");
        }
        else if(argument == "--csv") {
            options.csv_path = require_value(argc, argv, i, "--csv");
        }
        else if(argument == "--max-current-ma") {
            options.max_current_ma = parse_double(
                require_value(argc, argv, i, "--max-current-ma"), "--max-current-ma");
        }
        else if(argument == "--max-temperature-c") {
            options.max_temperature_c = parse_double(
                require_value(argc, argv, i, "--max-temperature-c"), "--max-temperature-c");
        }
        else if(argument == "--confirm-motion") {
            options.confirm_motion = true;
        }
        else {
            throw std::invalid_argument("unknown option: " + argument);
        }
    }

    if(options.show_help) return options;
    if(options.config_path.empty()) throw std::invalid_argument("--config is required");
    if(options.duration_s <= 0.0 || options.duration_s > 60.0) {
        throw std::invalid_argument("--duration must be in (0, 60]");
    }
    if(options.max_current_ma < 0.0 || options.max_temperature_c < 0.0) {
        throw std::invalid_argument("diagnostic thresholds cannot be negative");
    }

    const bool motion_mode = options.mode != Mode::ReadOnly;
    if(motion_mode && !options.confirm_motion) {
        throw std::invalid_argument("Torque-ON modes require --confirm-motion");
    }
    if((options.mode == Mode::TauOnly || options.mode == Mode::Damping) &&
        !options.joint_was_set) {
        throw std::invalid_argument("single-joint modes require --joint 1..6");
    }
    if(options.mode == Mode::TauOnly && !options.tau_was_set) {
        throw std::invalid_argument("tau mode requires --tau");
    }
    if(std::abs(options.tau_nm) > NOMINAL_MAX_EFFORT_NM + 1e-12) {
        throw std::invalid_argument("|--tau| exceeds the 0.2941995 N.m tool limit");
    }
    if(options.kd <= 0.0) throw std::invalid_argument("--kd must be positive");
    return options;
}

/**
 * @brief 将 MotorBusErr 输出为可读名称并保留整数值
 */
std::string error_text(serial_arm::MotorBusErr error) {
    using serial_arm::MotorBusErr;
    const char* name = "UNKNOWN";
    switch(error) {
        case MotorBusErr::NOT_CONFIGURED: name = "NOT_CONFIGURED"; break;
        case MotorBusErr::NOT_CONNECTED: name = "NOT_CONNECTED"; break;
        case MotorBusErr::NOT_ACTIVE: name = "NOT_ACTIVE"; break;
        case MotorBusErr::INVALID_CFG: name = "INVALID_CFG"; break;
        case MotorBusErr::OPEN_FAILED: name = "OPEN_FAILED"; break;
        case MotorBusErr::READ_FAILED: name = "READ_FAILED"; break;
        case MotorBusErr::WRITE_FAILED: name = "WRITE_FAILED"; break;
        case MotorBusErr::INVALID_STATE: name = "INVALID_STATE"; break;
        case MotorBusErr::INVALID_CMD: name = "INVALID_CMD"; break;
        case MotorBusErr::ACTUATOR_OFFLINE: name = "ACTUATOR_OFFLINE"; break;
        case MotorBusErr::ACTUATOR_FAULT: name = "ACTUATOR_FAULT"; break;
        case MotorBusErr::TIMEOUT: name = "TIMEOUT"; break;
        case MotorBusErr::ENABLE_FAILED: name = "ENABLE_FAILED"; break;
        case MotorBusErr::MODE_SWITCH_FAILED: name = "MODE_SWITCH_FAILED"; break;
        case MotorBusErr::STOP_FAILED: name = "STOP_FAILED"; break;
        case MotorBusErr::DISABLE_FAILED: name = "DISABLE_FAILED"; break;
        case MotorBusErr::RECOVER_FAILED: name = "RECOVER_FAILED"; break;
    }
    return std::string(name) + "(" + std::to_string(static_cast<int>(error)) + ")";
}

/**
 * @brief 在所有退出路径尽力执行零 PWM、Torque OFF 与 cleanup
 */
class SafeSession final {
public:
    explicit SafeSession(serial_arm::Hx10hmMotorBus& bus) noexcept : bus_(bus) {}

    ~SafeSession() {
        if(active_) (void)bus_.stop();
        if(connected_) (void)bus_.deactivate();
        bus_.cleanup();
    }

    void set_connected() noexcept { connected_ = true; }
    void set_active() noexcept { active_ = true; }

private:
    serial_arm::Hx10hmMotorBus& bus_;
    bool connected_{ false };
    bool active_{ false };
};

/**
 * @brief 创建以当前状态为目标的六轴零输出 MIT 命令
 */
serial_arm::ActuatorCtrlCmd make_zero_command(const serial_arm::ActuatorState& state) {
    serial_arm::ActuatorCtrlCmd command;
    command.pos = state.pos;
    command.vel.assign(state.pos.size(), 0.0);
    command.tor.assign(state.pos.size(), 0.0);
    command.kp.assign(state.pos.size(), 0.0);
    command.kd.assign(state.pos.size(), 0.0);
    return command;
}

/**
 * @brief 检查故障、温度和电流安全条件
 */
bool diagnostics_safe(
    const serial_arm::ActuatorState& state,
    const std::vector<serial_arm::protocol::hiwonder_bus_servo::RawState>& raw,
    const Options& options) {
    if(raw.size() != state.pos.size()) return false;
    for(std::size_t i = 0; i < state.pos.size(); ++i) {
        if(state.online[i] == 0U || state.err_code[i] != 0) return false;
        if(options.max_current_ma > 0.0 &&
            static_cast<double>(raw[i].current_raw_ma) > options.max_current_ma) {
            std::cerr << "current threshold exceeded at joint " << i + 1U << '\n';
            return false;
        }
        if(options.max_temperature_c > 0.0 &&
            static_cast<double>(raw[i].temperature_raw) > options.max_temperature_c) {
            std::cerr << "temperature threshold exceeded at joint " << i + 1U << '\n';
            return false;
        }
    }
    return true;
}

/**
 * @brief 验证 actuation 配置不会突破本工具的 nominal 硬限制
 */
bool validate_tool_limits(serial_arm::Hx10hmMotorBus& bus) {
    const auto& capabilities = bus.capabilities();
    for(std::size_t i = 0; i < capabilities.size(); ++i) {
        if(capabilities[i].max_effort > NOMINAL_MAX_EFFORT_NM + 1e-12) return false;
        const auto positive = bus.torque_to_pwm(i, capabilities[i].max_effort);
        const auto negative = bus.torque_to_pwm(i, -capabilities[i].max_effort);
        if(!positive || !negative || std::abs(static_cast<int>(*positive)) > NOMINAL_MAX_PWM ||
            std::abs(static_cast<int>(*negative)) > NOMINAL_MAX_PWM) {
            return false;
        }
    }
    return true;
}

/**
 * @brief 写入一轮 CSV 状态与命令
 */
void write_csv_rows(
    std::ofstream& csv,
    double timestamp_s,
    serial_arm::Hx10hmMotorBus& bus,
    const serial_arm::ActuatorState& state,
    const serial_arm::ActuatorCtrlCmd& command) {
    if(!csv) return;
    const auto& raw = bus.raw_diagnostics();
    for(std::size_t i = 0; i < state.pos.size(); ++i) {
        const double tau_cmd = serial_arm::Hx10hmMotorBus::calculate_mit_torque(
            command.tor[i], command.kp[i], command.pos[i], state.pos[i],
            command.kd[i], command.vel[i], state.vel[i]);
        const auto pwm = bus.torque_to_pwm(i, tau_cmd);
        csv << std::setprecision(12) << timestamp_s << ',' << i + 1U << ','
            << state.pos[i] << ',' << state.vel[i] << ',' << raw[i].position_raw << ','
            << raw[i].position_calibration_raw << ',' << raw[i].encoder_position_raw << ','
            << raw[i].current_raw_ma << ',' << raw[i].load_raw << ','
            << tau_cmd << ',' << (pwm ? *pwm : 0) << ','
            << command.kp[i] << ',' << command.kd[i] << ','
            << static_cast<int>(state.online[i]) << ',' << static_cast<int>(state.enabled[i])
            << ',' << state.err_code[i] << '\n';
    }
}

/**
 * @brief 输出六轴只读状态
 */
void print_state(
    const serial_arm::ActuatorState& state,
    const std::vector<serial_arm::protocol::hiwonder_bus_servo::RawState>& raw) {
    for(std::size_t i = 0; i < state.pos.size(); ++i) {
        std::cout << "J" << i + 1U << " q=" << state.pos[i]
            << " dq=" << state.vel[i] << " position_raw=" << raw[i].position_raw
            << " position_calibration=" << raw[i].position_calibration_raw
            << " encoder_raw=" << raw[i].encoder_position_raw
            << " current_ma=" << raw[i].current_raw_ma
            << " load_raw=" << raw[i].load_raw
            << " voltage_v=" << static_cast<double>(raw[i].voltage_raw) * 0.1
            << " temperature_c=" << static_cast<int>(raw[i].temperature_raw)
            << " fault=" << state.err_code[i] << '\n';
    }
}

/**
 * @brief 输出周期统计
 */
void print_stats(const CycleStats& stats, double elapsed_s) {
    const double average = stats.sample_count == 0U ? 0.0 :
        stats.total_cycle_us / static_cast<double>(stats.sample_count);
    const double minimum = stats.sample_count == 0U ? 0.0 : stats.min_cycle_us;
    const double effective_hz = elapsed_s <= 0.0 ? 0.0 :
        static_cast<double>(stats.sample_count) / elapsed_s;
    std::cout << "sample_count=" << stats.sample_count << '\n'
        << "average_cycle_us=" << average << '\n'
        << "min_cycle_us=" << minimum << '\n'
        << "max_cycle_us=" << stats.max_cycle_us << '\n'
        << "effective_hz=" << effective_hz << '\n'
        << "timeout_count=" << stats.timeout_count << '\n'
        << "overrun_count=" << stats.overrun_count << '\n';
}

/**
 * @brief 执行选定的有限时长验证循环
 */
int run_loop(serial_arm::Hx10hmMotorBus& bus, const Options& options) {
    using Clock = std::chrono::steady_clock;
    const auto start = Clock::now();
    const auto end = start + std::chrono::duration<double>(options.duration_s);
    auto deadline = start;
    CycleStats stats;
    std::ofstream csv;
    if(!options.csv_path.empty()) {
        csv.open(options.csv_path, std::ios::out | std::ios::trunc);
        if(!csv) {
            std::cerr << "failed to open CSV: " << options.csv_path << '\n';
            return EXIT_FAILURE;
        }
        csv << "timestamp_s,joint,q_rad,dq_rad_s,position_raw,position_calibration,encoder_raw,"
            << "current_ma,load_raw,tau_cmd_nm,"
            << "pwm_cmd,kp,kd,online,enabled,fault\n";
    }

    bool failed = false;
    while(stop_requested == 0 && Clock::now() < end) {
        deadline += options.period;
        const auto cycle_start = Clock::now();
        const auto state = bus.read();
        if(!state) {
            if(state.error() == serial_arm::MotorBusErr::TIMEOUT) ++stats.timeout_count;
            std::cerr << "read failed, MotorBusErr=" << error_text(state.error()) << '\n';
            failed = true;
            break;
        }
        const auto& raw = bus.raw_diagnostics();
        if(!diagnostics_safe(*state, raw, options)) {
            std::cerr << "fault or diagnostic safety limit detected\n";
            failed = true;
            break;
        }

        auto command = make_zero_command(*state);
        if(options.mode == Mode::TauOnly) command.tor[options.joint_index] = options.tau_nm;
        if(options.mode == Mode::Damping) command.kd[options.joint_index] = options.kd;

        if(options.mode != Mode::ReadOnly) {
            const auto written = bus.write(command);
            if(!written) {
                if(written.error() == serial_arm::MotorBusErr::TIMEOUT) ++stats.timeout_count;
                std::cerr << "write failed, MotorBusErr=" << error_text(written.error()) << '\n';
                failed = true;
                break;
            }
        }

        const double timestamp_s = std::chrono::duration<double>(Clock::now() - start).count();
        write_csv_rows(csv, timestamp_s, bus, *state, command);
        if(options.mode == Mode::ReadOnly) print_state(*state, raw);

        const auto cycle_end = Clock::now();
        const double cycle_us = std::chrono::duration<double, std::micro>(
            cycle_end - cycle_start).count();
        ++stats.sample_count;
        stats.total_cycle_us += cycle_us;
        stats.min_cycle_us = std::min(stats.min_cycle_us, cycle_us);
        stats.max_cycle_us = std::max(stats.max_cycle_us, cycle_us);

        if(cycle_end > deadline) {
            ++stats.overrun_count;
            deadline = cycle_end;
        }
        else {
            std::this_thread::sleep_until(deadline);
        }
    }

    const double elapsed_s = std::chrono::duration<double>(Clock::now() - start).count();
    print_stats(stats, elapsed_s);
    return failed ? EXIT_FAILURE : EXIT_SUCCESS;
}

} // namespace

// ! ========================= 主 函 数 ========================= ! //

int main(int argc, char** argv) {
    try {
        const Options options = parse_options(argc, argv);
        if(options.show_help) {
            print_usage(argv[0]);
            return EXIT_SUCCESS;
        }

        std::signal(SIGINT, handle_signal);
        std::signal(SIGTERM, handle_signal);

        serial_arm::Hx10hmMotorBus bus;
        SafeSession session(bus);
        const auto configured = bus.configure(options.config_path);
        if(!configured) {
            std::cerr << "configure failed, MotorBusErr=" << error_text(configured.error()) << '\n';
            return EXIT_FAILURE;
        }
        if(options.mode != Mode::ReadOnly && !validate_tool_limits(bus)) {
            std::cerr << "config exceeds calibration tool hard limits\n";
            return EXIT_FAILURE;
        }
        if(options.mode == Mode::TauOnly &&
            std::abs(options.tau_nm) > bus.capabilities()[options.joint_index].max_effort + 1e-12) {
            std::cerr << "requested tau exceeds configured joint max_effort\n";
            return EXIT_FAILURE;
        }
        if(options.mode == Mode::Damping &&
            options.kd > bus.capabilities()[options.joint_index].max_kd + 1e-12) {
            std::cerr << "requested kd exceeds configured joint max_kd\n";
            return EXIT_FAILURE;
        }

        const auto connected = bus.connect();
        if(!connected) {
            std::cerr << "connect failed, MotorBusErr=" << error_text(connected.error()) << '\n';
            return EXIT_FAILURE;
        }
        session.set_connected();

        if(options.mode != Mode::ReadOnly) {
            const auto active = bus.activate();
            if(!active) {
                std::cerr << "activate failed, MotorBusErr=" << error_text(active.error()) << '\n';
                return EXIT_FAILURE;
            }
            session.set_active();
        }

        return run_loop(bus, options);
    }
    catch(const std::exception& error) {
        std::cerr << "argument/error: " << error.what() << '\n';
        print_usage(argv[0]);
        return EXIT_FAILURE;
    }
}

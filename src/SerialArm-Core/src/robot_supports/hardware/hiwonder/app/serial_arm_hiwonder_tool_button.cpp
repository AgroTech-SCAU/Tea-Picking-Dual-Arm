#include "serial_arm_hardware_hiwonder/hx10hm_motor_bus.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

namespace {

constexpr double RAD_TO_DEG = 57.2957795130823208768;
std::atomic_bool stop_requested{ false };

struct Options {
    std::string config_path;
    std::chrono::milliseconds period{ 50 };
    bool spring{ false };
    bool confirm_spring{ false };
};

void signal_handler(int) {
    stop_requested.store(true);
}

void print_usage(const char* program) {
    std::cout
        << "Usage: " << program << " --config <hardware.yaml> [options]\n"
        << "\n"
        << "Options:\n"
        << "  --period-ms <ms>     print/control period, default 50\n"
        << "  --spring             enable low-torque Tool Button zero-position spring\n"
        << "  --confirm-spring     required together with --spring\n"
        << "  --help                show this help\n";
}

Options parse_options(int argc, char** argv) {
    Options options;
    for(int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if(arg == "--help") {
            print_usage(argv[0]);
            std::exit(0);
        }
        if(arg == "--config" && i + 1 < argc) {
            options.config_path = argv[++i];
            continue;
        }
        if(arg == "--period-ms" && i + 1 < argc) {
            const long value = std::stol(argv[++i]);
            if(value <= 0) throw std::invalid_argument("--period-ms must be positive");
            options.period = std::chrono::milliseconds(value);
            continue;
        }
        if(arg == "--spring") {
            options.spring = true;
            continue;
        }
        if(arg == "--confirm-spring") {
            options.confirm_spring = true;
            continue;
        }
        throw std::invalid_argument("unknown or incomplete argument: " + arg);
    }

    if(options.config_path.empty()) throw std::invalid_argument("--config is required");
    if(options.spring && !options.confirm_spring) {
        throw std::invalid_argument("--spring requires --confirm-spring");
    }
    return options;
}

serial_arm::ActuatorCtrlCmd zero_joint_pwm_command(const serial_arm::ActuatorState& state) {
    serial_arm::ActuatorCtrlCmd cmd;
    cmd.pos = state.pos;
    cmd.vel.assign(state.pos.size(), 0.0);
    cmd.tor.assign(state.pos.size(), 0.0);
    cmd.kp.assign(state.pos.size(), 0.0);
    cmd.kd.assign(state.pos.size(), 0.0);
    return cmd;
}

} // namespace

int main(int argc, char** argv) {
    try {
        const auto options = parse_options(argc, argv);
        std::signal(SIGINT, signal_handler);
        std::signal(SIGTERM, signal_handler);

        serial_arm::Hx10hmMotorBus bus;
        const auto configured = bus.configure(options.config_path);
        if(!configured) {
            std::cerr << "configure failed, MotorBusErr="
                      << static_cast<int>(configured.error()) << '\n';
            return 2;
        }
        if(!bus.tool_button_enabled()) {
            std::cerr << "Tool Button is disabled in hardware YAML\n";
            return 2;
        }

        const auto connected = bus.connect();
        if(!connected) {
            std::cerr << "connect failed, MotorBusErr="
                      << static_cast<int>(connected.error())
                      << "; check Joint1~Joint6 and Tool Button on the shared HX bus\n";
            return 3;
        }

        if(options.spring) {
            std::cout
                << "spring mode enabled: Joint1~Joint6 are torque-enabled with zero PWM, "
                << "Tool Button uses configured low-torque zero-position spring\n";
            const auto active = bus.activate();
            if(!active) {
                std::cerr << "activate failed, MotorBusErr="
                          << static_cast<int>(active.error()) << '\n';
                bus.cleanup();
                return 4;
            }
        }
        else {
            std::cout << "read-only mode: all HX servos remain Torque OFF\n";
        }

        while(!stop_requested.load()) {
            const auto state = bus.read();
            if(!state) {
                std::cerr << "read failed, MotorBusErr="
                          << static_cast<int>(state.error())
                          << "; Tool Button online=0 or shared HX bus read failed\n";
                break;
            }

            if(options.spring) {
                const auto written = bus.write(zero_joint_pwm_command(*state));
                if(!written) {
                    std::cerr << "write failed, MotorBusErr="
                              << static_cast<int>(written.error())
                              << "; Tool Button spring stopped\n";
                    break;
                }
            }

            const auto button = bus.tool_button_state();
            std::cout
                << "button q=" << button.pos_rad * RAD_TO_DEG << " deg"
                << "  dq=" << button.vel_rad_s * RAD_TO_DEG << " deg/s"
                << "  pressed=" << (button.pressed ? 1 : 0)
                << "  online=" << (button.online ? 1 : 0)
                << '\n';

            std::this_thread::sleep_for(options.period);
        }

        if(options.spring) {
            const auto deactivated = bus.deactivate();
            if(!deactivated) {
                std::cerr << "deactivate failed, MotorBusErr="
                          << static_cast<int>(deactivated.error()) << '\n';
                bus.cleanup();
                return 5;
            }
        }
        else {
            bus.cleanup();
        }
        return 0;
    }
    catch(const std::exception& error) {
        std::cerr << "argument/error: " << error.what() << '\n';
        return 1;
    }
}

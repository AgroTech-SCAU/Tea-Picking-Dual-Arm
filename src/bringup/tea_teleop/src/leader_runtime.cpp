#include "tea_teleop/leader_runtime.hpp"

#include <sstream>
#include <stdexcept>
#include <utility>

#include "serial_arm/config/robot_profile.hpp"

namespace tea_teleop {

namespace {

const char* robot_err_name(serial_arm::RobotErr error) noexcept {
    using E = serial_arm::RobotErr;
    switch(error) {
        case E::NOT_CONFIGURED: return "NOT_CONFIGURED";
        case E::ALREADY_CONFIGURED: return "ALREADY_CONFIGURED";
        case E::INVALID_CFG: return "INVALID_CFG";
        case E::NULL_MOTOR_BUS: return "NULL_MOTOR_BUS";
        case E::MOTOR_BUS_SIZE_MISMATCH: return "MOTOR_BUS_SIZE_MISMATCH";
        case E::WRITE_DISABLED: return "WRITE_DISABLED";
        case E::NOT_ACTIVE: return "NOT_ACTIVE";
        case E::NOT_INACTIVE: return "NOT_INACTIVE";
        case E::ALREADY_ACTIVE: return "ALREADY_ACTIVE";
        case E::FAULTED: return "FAULTED";
        case E::NOT_FAULTED: return "NOT_FAULTED";
        case E::INVALID_TIME: return "INVALID_TIME";
        case E::MOTOR_BUS_CONNECT_FAILED: return "MOTOR_BUS_CONNECT_FAILED";
        case E::MOTOR_BUS_ACTIVATE_FAILED: return "MOTOR_BUS_ACTIVATE_FAILED";
        case E::MOTOR_BUS_READ_FAILED: return "MOTOR_BUS_READ_FAILED";
        case E::MOTOR_BUS_WRITE_FAILED: return "MOTOR_BUS_WRITE_FAILED";
        case E::MOTOR_BUS_DEACTIVATE_FAILED: return "MOTOR_BUS_DEACTIVATE_FAILED";
        case E::MOTOR_BUS_RECOVER_FAILED: return "MOTOR_BUS_RECOVER_FAILED";
        case E::MAPPER_FAILED: return "MAPPER_FAILED";
        case E::CTRLLER_FAILED: return "CTRLLER_FAILED";
        case E::SAFETY_FAILED: return "SAFETY_FAILED";
        case E::MODEL_FEEDFORWARD_FAILED: return "MODEL_FEEDFORWARD_FAILED";
        case E::INVALID_MODEL_FEEDFORWARD: return "INVALID_MODEL_FEEDFORWARD";
        case E::FAULT_RECOVERY_NOT_ALLOWED: return "FAULT_RECOVERY_NOT_ALLOWED";
    }
    return "UNKNOWN";
}

const char* motor_bus_err_name(serial_arm::MotorBusErr error) noexcept {
    using E = serial_arm::MotorBusErr;
    switch(error) {
        case E::NOT_CONFIGURED: return "NOT_CONFIGURED";
        case E::NOT_CONNECTED: return "NOT_CONNECTED";
        case E::NOT_ACTIVE: return "NOT_ACTIVE";
        case E::INVALID_CFG: return "INVALID_CFG";
        case E::OPEN_FAILED: return "OPEN_FAILED";
        case E::READ_FAILED: return "READ_FAILED";
        case E::WRITE_FAILED: return "WRITE_FAILED";
        case E::INVALID_STATE: return "INVALID_STATE";
        case E::INVALID_CMD: return "INVALID_CMD";
        case E::ACTUATOR_OFFLINE: return "ACTUATOR_OFFLINE";
        case E::ACTUATOR_FAULT: return "ACTUATOR_FAULT";
        case E::TIMEOUT: return "TIMEOUT";
        case E::ENABLE_FAILED: return "ENABLE_FAILED";
        case E::MODE_SWITCH_FAILED: return "MODE_SWITCH_FAILED";
        case E::STOP_FAILED: return "STOP_FAILED";
        case E::DISABLE_FAILED: return "DISABLE_FAILED";
        case E::RECOVER_FAILED: return "RECOVER_FAILED";
    }
    return "UNKNOWN";
}

const char* safety_err_name(serial_arm::SafetyErr error) noexcept {
    using E = serial_arm::SafetyErr;
    switch(error) {
        case E::NOT_CONFIGURED: return "NOT_CONFIGURED";
        case E::INVALID_CFG: return "INVALID_CFG";
        case E::INVALID_DT: return "INVALID_DT";
        case E::INVALID_STATE_AGE: return "INVALID_STATE_AGE";
        case E::INVALID_CMD_AGE: return "INVALID_CMD_AGE";
        case E::STATE_TIMEOUT: return "STATE_TIMEOUT";
        case E::CMD_TIMEOUT: return "CMD_TIMEOUT";
        case E::INVALID_JOINT_STATE_SIZE: return "INVALID_JOINT_STATE_SIZE";
        case E::INVALID_ACTUATOR_STATE_SIZE: return "INVALID_ACTUATOR_STATE_SIZE";
        case E::NON_FINITE_JOINT_STATE: return "NON_FINITE_JOINT_STATE";
        case E::NON_FINITE_ACTUATOR_STATE: return "NON_FINITE_ACTUATOR_STATE";
        case E::JOINT_POS_LIMIT: return "JOINT_POS_LIMIT";
        case E::JOINT_VEL_LIMIT: return "JOINT_VEL_LIMIT";
        case E::ACTUATOR_OFFLINE: return "ACTUATOR_OFFLINE";
        case E::ACTUATOR_NOT_ENABLED: return "ACTUATOR_NOT_ENABLED";
        case E::ACTUATOR_FAULT: return "ACTUATOR_FAULT";
        case E::INVALID_CMD_SIZE: return "INVALID_CMD_SIZE";
        case E::NON_FINITE_CMD: return "NON_FINITE_CMD";
        case E::CMD_POS_LIMIT: return "CMD_POS_LIMIT";
        case E::CMD_VEL_LIMIT: return "CMD_VEL_LIMIT";
        case E::CMD_EFFORT_LIMIT: return "CMD_EFFORT_LIMIT";
        case E::CMD_KP_LIMIT: return "CMD_KP_LIMIT";
        case E::CMD_KD_LIMIT: return "CMD_KD_LIMIT";
        case E::CMD_POS_STEP_LIMIT: return "CMD_POS_STEP_LIMIT";
        case E::CMD_VEL_STEP_LIMIT: return "CMD_VEL_STEP_LIMIT";
    }
    return "UNKNOWN";
}

const char* model_feedforward_err_name(serial_arm::ModelFeedforwardErr error) noexcept {
    using E = serial_arm::ModelFeedforwardErr;
    switch(error) {
        case E::NOT_CONFIGURED: return "NOT_CONFIGURED";
        case E::INVALID_INPUT: return "INVALID_INPUT";
        case E::INVALID_MODE: return "INVALID_MODE";
        case E::COMPUTE_FAILED: return "COMPUTE_FAILED";
    }
    return "UNKNOWN";
}

std::string robot_fault_text(const char* operation, const serial_arm::RobotFault& fault) {
    std::ostringstream message;
    message << operation << "失败: " << robot_err_name(fault.code);

    switch(fault.code) {
        case serial_arm::RobotErr::MOTOR_BUS_CONNECT_FAILED:
        case serial_arm::RobotErr::MOTOR_BUS_ACTIVATE_FAILED:
        case serial_arm::RobotErr::MOTOR_BUS_READ_FAILED:
        case serial_arm::RobotErr::MOTOR_BUS_WRITE_FAILED:
        case serial_arm::RobotErr::MOTOR_BUS_DEACTIVATE_FAILED:
        case serial_arm::RobotErr::MOTOR_BUS_RECOVER_FAILED:
            message << ", MotorBus=" << motor_bus_err_name(fault.motor_bus_err);
            break;

        case serial_arm::RobotErr::SAFETY_FAILED:
            message
                << ", Safety=" << safety_err_name(fault.safety_fault.code)
                << ", index=" << fault.safety_fault.index
                << ", value=" << fault.safety_fault.value
                << ", limit=" << fault.safety_fault.limit;
            break;

        case serial_arm::RobotErr::MODEL_FEEDFORWARD_FAILED:
        case serial_arm::RobotErr::INVALID_MODEL_FEEDFORWARD:
            message
                << ", ModelFeedforward="
                << model_feedforward_err_name(fault.model_feedforward_err);
            break;

        default:
            break;
    }

    return message.str();
}

std::string motor_bus_error_text(const char* operation, serial_arm::MotorBusErr error) {
    std::ostringstream message;
    message << operation << "失败: MotorBus=" << motor_bus_err_name(error);
    return message.str();
}

std::string with_tool_button_context(
    std::string message,
    const serial_arm::Hx10hmMotorBus* bus) {
    if(bus != nullptr && bus->tool_button_enabled() && !bus->tool_button_state().online) {
        message += ", ToolButton=OFFLINE";
    }
    return message;
}

} // namespace

LeaderReadSession::~LeaderReadSession() {
    close();
}

void LeaderReadSession::open(const std::string& profile_name) {
    if(motor_bus_) throw std::logic_error("主臂只读会话已经打开");

    const auto profile = serial_arm::load_robot_profile_core(profile_name);
    if(!profile) {
        throw std::runtime_error("主臂 Profile 加载失败: " + profile.error().message);
    }

    auto motor_bus = loader_.load(profile->hardware_plugin, profile->hardware_config_path);
    if(!motor_bus) {
        throw std::runtime_error(
            "主臂 Hardware Backend 加载失败: " +
            std::to_string(static_cast<int>(motor_bus.error())));
    }

    const auto config = serial_arm::load_robot_cfg(
        profile->core_config_path,
        motor_bus.value()->capabilities());
    if(!config) throw std::runtime_error("主臂配置加载失败: " + config.error().message);
    cfg_ = config.value();

    const auto mapper = mapper_.configure(cfg_.mapper);
    if(!mapper) {
        throw std::runtime_error(
            "主臂关节映射配置失败: " +
            std::to_string(static_cast<int>(mapper.error())));
    }

    const auto connected = motor_bus.value()->connect();
    if(!connected) {
        throw std::runtime_error(motor_bus_error_text("主臂只读连接", connected.error()));
    }

    motor_bus_ = std::move(motor_bus.value());
}

serial_arm::JointState LeaderReadSession::read() {
    if(!motor_bus_) throw std::logic_error("主臂只读会话尚未打开");

    const auto actuator_state = motor_bus_->read();
    if(!actuator_state) {
        throw std::runtime_error(motor_bus_error_text("主臂读取", actuator_state.error()));
    }

    const auto joint_state = mapper_.to_joint_state(actuator_state.value());
    if(!joint_state) {
        throw std::runtime_error(
            "主臂关节状态转换失败: " +
            std::to_string(static_cast<int>(joint_state.error())));
    }
    return joint_state.value();
}

void LeaderReadSession::close() noexcept {
    if(!motor_bus_) return;
    motor_bus_->cleanup();
    motor_bus_.reset();
}

LeaderRuntime::~LeaderRuntime() {
    (void)safe_deactivate();
}

void LeaderRuntime::initialize(const std::string& profile_name) {
    if(initialized_) throw std::logic_error("主臂控制会话已经初始化");

    const auto profile = serial_arm::load_robot_profile_core(profile_name);
    if(!profile) {
        throw std::runtime_error("主臂 Profile 加载失败: " + profile.error().message);
    }

    auto motor_bus = loader_.load(profile->hardware_plugin, profile->hardware_config_path);
    if(!motor_bus) {
        throw std::runtime_error(
            "主臂 Hardware Backend 加载失败: " +
            std::to_string(static_cast<int>(motor_bus.error())));
    }

    const auto config = serial_arm::load_robot_cfg(
        profile->core_config_path,
        motor_bus.value()->capabilities());
    if(!config) throw std::runtime_error("主臂配置加载失败: " + config.error().message);
    cfg_ = config.value();
    hiwonder_bus_ = dynamic_cast<serial_arm::Hx10hmMotorBus*>(motor_bus.value().get());

    const auto dynamics = dynamics_.configure(cfg_.dynamics);
    if(!dynamics) {
        throw std::runtime_error(
            "主臂动力学配置失败: " +
            std::to_string(static_cast<int>(dynamics.error())));
    }

    serial_arm::ModelFeedforwardFn model_feedforward = [this](
        serial_arm::ModelFeedforwardMode mode,
        const serial_arm::JointState& state,
        const serial_arm::JointVector& acc,
        const serial_arm::JointVector& ref_acc,
        double) -> tl::expected<serial_arm::JointVector, serial_arm::ModelFeedforwardErr> {
        const auto updated = dynamics_.update(state, acc, ref_acc);
        if(!updated) {
            return tl::make_unexpected(serial_arm::ModelFeedforwardErr::COMPUTE_FAILED);
        }

        switch(mode) {
            case serial_arm::ModelFeedforwardMode::NONE:
                return serial_arm::JointVector(state.pos.size(), 0.0);
            case serial_arm::ModelFeedforwardMode::GRAVITY:
                return dynamics_.get_gravity_compensation();
            case serial_arm::ModelFeedforwardMode::FULL_INVERSE_DYNAMICS:
                return dynamics_.get_inverse_dynamics();
        }
        return tl::make_unexpected(serial_arm::ModelFeedforwardErr::INVALID_MODE);
    };

    const auto configured = robot_.configure(
        cfg_,
        std::move(motor_bus.value()),
        std::move(model_feedforward));
    if(!configured) {
        throw std::runtime_error(robot_fault_text("主臂配置", configured.error()));
    }

    const auto feedforward = robot_.set_model_feedforward_mode(
        cfg_.runtime.model_feedforward_mode);
    if(!feedforward) {
        throw std::runtime_error(robot_fault_text("主臂模型前馈设置", feedforward.error()));
    }

    initialized_ = true;
}

void LeaderRuntime::activate(serial_arm::JointImpedanceMode mode) {
    if(!initialized_) throw std::logic_error("主臂控制会话尚未初始化");

    const auto active = robot_.activate();
    if(!active) {
        throw std::runtime_error(with_tool_button_context(
            robot_fault_text("主臂使能", active.error()),
            hiwonder_bus_));
    }

    const auto mode_result = robot_.set_impedance_mode(mode);
    if(!mode_result) {
        (void)robot_.force_deactivate();
        throw std::runtime_error(robot_fault_text("主臂模式切换", mode_result.error()));
    }
}

void LeaderRuntime::set_impedance_mode(serial_arm::JointImpedanceMode mode) {
    const auto result = robot_.set_impedance_mode(mode);
    if(!result) throw std::runtime_error(robot_fault_text("主臂模式切换", result.error()));
}

void LeaderRuntime::set_cmd(
    const serial_arm::JointCmd& cmd,
    serial_arm::Robot::TimePoint now) {
    const auto result = robot_.set_cmd(cmd, now);
    if(!result) throw std::runtime_error(robot_fault_text("主臂命令", result.error()));
}

serial_arm::RobotCycleOutput LeaderRuntime::cycle(serial_arm::Robot::TimePoint now) {
    const auto output = robot_.cycle(now);
    if(!output) {
        throw std::runtime_error(with_tool_button_context(
            robot_fault_text("主臂控制周期", output.error()),
            hiwonder_bus_));
    }
    return output.value();
}

bool LeaderRuntime::safe_deactivate() noexcept {
    if(!initialized_) return true;
    if(robot_.get_state() == serial_arm::RobotState::INACTIVE) return true;

    const auto deactivated = robot_.deactivate();
    if(deactivated) return true;
    const auto forced = robot_.force_deactivate();
    return static_cast<bool>(forced);
}

double LeaderRuntime::control_frequency_hz() const noexcept {
    return cfg_.runtime.ctrl_frequency_hz;
}

serial_arm::RobotState LeaderRuntime::state() const noexcept {
    return robot_.get_state();
}

const serial_arm::JointState& LeaderRuntime::joint_state() const noexcept {
    return robot_.get_joint_state();
}

const serial_arm::RobotCfg& LeaderRuntime::config() const noexcept {
    return cfg_;
}

bool LeaderRuntime::tool_button_pressed() const noexcept {
    return hiwonder_bus_ != nullptr && hiwonder_bus_->tool_button_state().online &&
        hiwonder_bus_->tool_button_state().pressed;
}

serial_arm::ToolButtonState LeaderRuntime::tool_button_state() const noexcept {
    if(hiwonder_bus_ == nullptr) return {};
    return hiwonder_bus_->tool_button_state();
}

} // namespace tea_teleop

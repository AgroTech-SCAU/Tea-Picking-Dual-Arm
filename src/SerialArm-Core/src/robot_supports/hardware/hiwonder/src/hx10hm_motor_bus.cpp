#include "serial_arm_hardware_hiwonder/hx10hm_motor_bus.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <utility>

namespace serial_arm {

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //



// ! ========================= 私 有 量 / 工 具 函 数 实 现 ========================= ! //

namespace {

constexpr std::size_t HX10HM_ACTUATOR_COUNT = 6;
constexpr std::uint16_t HX10HM_MAX_POSITION_RAW = 4095;
constexpr double TWO_PI = 6.28318530717958647692;
constexpr double RAD_PER_STEP = TWO_PI / 4096.0;

/**
 * @brief 检查向量中的所有值是否为有限值
 */
bool finite_vector(const std::vector<double>& values) {
    return std::all_of(values.begin(), values.end(), [](double value) {
        return std::isfinite(value);
    });
}

/**
 * @brief 从 YAML 节点读取必填字段
 */
template<typename T>
tl::expected<T, MotorBusErr> require_as(const YAML::Node& parent, const char* key) {
    const YAML::Node node = parent[key];
    if(!node) return tl::make_unexpected(MotorBusErr::INVALID_CFG);
    try {
        return node.as<T>();
    }
    catch(const YAML::Exception&) {
        return tl::make_unexpected(MotorBusErr::INVALID_CFG);
    }
}

/**
 * @brief 解析速度编码名称
 */
tl::expected<HiwonderVelocityEncoding, MotorBusErr> parse_velocity_encoding(const std::string& value) {
    if(value == "bit15_sign_magnitude") return HiwonderVelocityEncoding::Bit15SignMagnitude;
    return tl::make_unexpected(MotorBusErr::INVALID_CFG);
}

} // namespace

// ! ========================= 接 口 类 方 法 / 函 数 实 现 ========================= ! //

/**
 * @brief 从 YAML 读取并校验 Backend 配置
 */
tl::expected<void, MotorBusErr> Hx10hmMotorBus::configure(const std::string& config_path) {
    if(connected_) return tl::make_unexpected(MotorBusErr::INVALID_STATE);

    try {
        const YAML::Node root = YAML::LoadFile(config_path);
        const YAML::Node hiwonder = root["hiwonder"] ? root["hiwonder"] : root;
        if(!hiwonder || !hiwonder.IsMap()) {
            return tl::make_unexpected(MotorBusErr::INVALID_CFG);
        }

        HiwonderBusCfg cfg;
        auto serial_port = require_as<std::string>(hiwonder, "serial_port");
        auto baudrate = require_as<int>(hiwonder, "baudrate");
        auto read_timeout_ms = require_as<long long>(hiwonder, "read_timeout_ms");
        auto write_timeout_ms = require_as<long long>(hiwonder, "write_timeout_ms");
        auto feedback_timeout_ms = require_as<long long>(hiwonder, "feedback_timeout_ms");
        auto startup_read_cycles = require_as<std::size_t>(hiwonder, "startup_read_cycles");
        auto restore_mode = require_as<bool>(hiwonder, "restore_position_mode_on_deactivate");
        auto velocity_encoding_name = require_as<std::string>(hiwonder, "velocity_encoding");
        auto torque_feedback_mode = require_as<std::string>(hiwonder, "torque_feedback_mode");
        if(!serial_port || !baudrate || !read_timeout_ms || !write_timeout_ms ||
            !feedback_timeout_ms || !startup_read_cycles || !restore_mode ||
            !velocity_encoding_name || !torque_feedback_mode) {
            return tl::make_unexpected(MotorBusErr::INVALID_CFG);
        }
        const auto velocity_encoding = parse_velocity_encoding(*velocity_encoding_name);
        if(!velocity_encoding) return tl::make_unexpected(velocity_encoding.error());

        cfg.serial_port = *serial_port;
        cfg.baudrate = *baudrate;
        cfg.read_timeout = std::chrono::milliseconds(*read_timeout_ms);
        cfg.write_timeout = std::chrono::milliseconds(*write_timeout_ms);
        cfg.feedback_timeout = std::chrono::milliseconds(*feedback_timeout_ms);
        cfg.startup_read_cycles = *startup_read_cycles;
        cfg.restore_position_mode_on_deactivate = *restore_mode;
        cfg.velocity_encoding = *velocity_encoding;
        cfg.torque_feedback_mode = *torque_feedback_mode;

        const YAML::Node actuators = hiwonder["actuators"];
        if(!actuators || !actuators.IsSequence()) {
            return tl::make_unexpected(MotorBusErr::INVALID_CFG);
        }
        for(const auto& item : actuators) {
            if(!item.IsMap()) return tl::make_unexpected(MotorBusErr::INVALID_CFG);

            auto name = require_as<std::string>(item, "name");
            auto joint_name = require_as<std::string>(item, "joint_name");
            auto servo_id = require_as<int>(item, "servo_id");
            auto raw_zero = require_as<int>(item, "raw_zero");
            auto direction = require_as<int>(item, "direction");
            auto min_pos = require_as<double>(item, "min_pos");
            auto max_pos = require_as<double>(item, "max_pos");
            auto max_vel = require_as<double>(item, "max_vel");
            auto max_effort = require_as<double>(item, "max_effort");
            auto max_kp = require_as<double>(item, "max_kp");
            auto max_kd = require_as<double>(item, "max_kd");
            if(!name || !joint_name || !servo_id || !raw_zero || !direction ||
                !min_pos || !max_pos || !max_vel || !max_effort || !max_kp || !max_kd ||
                *servo_id < 0 || *servo_id >= protocol::hiwonder_bus_servo::BROADCAST_ID ||
                *raw_zero < 0 || *raw_zero > HX10HM_MAX_POSITION_RAW) {
                return tl::make_unexpected(MotorBusErr::INVALID_CFG);
            }

            HiwonderActuatorCfg actuator;
            actuator.name = *name;
            actuator.joint_name = *joint_name;
            actuator.servo_id = static_cast<std::uint8_t>(*servo_id);
            actuator.raw_zero = static_cast<std::uint16_t>(*raw_zero);
            actuator.direction = *direction;
            actuator.min_pos = *min_pos;
            actuator.max_pos = *max_pos;
            actuator.max_vel = *max_vel;
            actuator.max_effort = *max_effort;
            actuator.max_kp = *max_kp;
            actuator.max_kd = *max_kd;
            cfg.actuators.push_back(std::move(actuator));
        }
        return configure(cfg);
    }
    catch(const YAML::Exception&) {
        return tl::make_unexpected(MotorBusErr::INVALID_CFG);
    }
}

/**
 * @brief 使用结构化参数配置 Backend
 */
tl::expected<void, MotorBusErr> Hx10hmMotorBus::configure(const HiwonderBusCfg& cfg) {
    if(connected_) return tl::make_unexpected(MotorBusErr::INVALID_STATE);
    const auto valid = validate_cfg(cfg);
    if(!valid) return tl::make_unexpected(valid.error());

    cfg_ = cfg;
    capabilities_.clear();
    capabilities_.reserve(cfg_.actuators.size());
    for(const auto& actuator : cfg_.actuators) {
        ActuatorCapability capability;
        capability.actuator_name = actuator.name;
        capability.min_pos = actuator.min_pos;
        capability.max_pos = actuator.max_pos;
        capability.max_vel = actuator.max_vel;
        capability.max_effort = actuator.max_effort;
        capability.max_kp = actuator.max_kp;
        capability.max_kd = actuator.max_kd;
        capabilities_.push_back(std::move(capability));
    }

    last_state_ = ActuatorState{};
    raw_states_.clear();
    online_.assign(cfg_.actuators.size(), 0U);
    enabled_.assign(cfg_.actuators.size(), 0U);
    last_feedback_time_ = TimePoint{};
    configured_ = true;
    active_ = false;
    return {};
}

/**
 * @brief 打开串口、确保 Torque OFF 并验证六轴通信
 */
tl::expected<void, MotorBusErr> Hx10hmMotorBus::connect() {
    if(!configured_) return tl::make_unexpected(MotorBusErr::NOT_CONFIGURED);
    if(connected_) return {};

    try {
        transport::SerialPort::Config serial_cfg;
        serial_cfg.baud_rate = static_cast<std::uint32_t>(cfg_.baudrate);
        serial_cfg.data_bits = 8;
        serial_cfg.parity = transport::SerialPort::Parity::None;
        serial_cfg.stop_bits = transport::SerialPort::StopBits::One;
        serial_cfg.flow_control = transport::SerialPort::FlowControl::None;
        serial_cfg.read_timeout = cfg_.read_timeout;
        serial_cfg.write_timeout = cfg_.write_timeout;
        serial_cfg.flush_on_open = true;
        serial_.open(cfg_.serial_port, serial_cfg);
        protocol_ = std::make_unique<protocol::hiwonder_bus_servo::HiwonderBusServo>(serial_);
        connected_ = true;

        online_.assign(cfg_.actuators.size(), 0U);
        enabled_.assign(cfg_.actuators.size(), 0U);
        last_state_.pos.assign(cfg_.actuators.size(), 0.0);
        last_state_.vel.assign(cfg_.actuators.size(), 0.0);
        last_state_.tor.assign(cfg_.actuators.size(), 0.0);
        last_state_.online = online_;
        last_state_.enabled = enabled_;
        last_state_.err_code.assign(cfg_.actuators.size(), 0);

        const auto disabled = set_all_torque(false);
        if(!disabled) {
            release_connection_noexcept(true);
            return tl::make_unexpected(disabled.error());
        }

        const auto state = read();
        if(!state) {
            release_connection_noexcept(true);
            return tl::make_unexpected(state.error());
        }
        if(std::any_of(state->online.begin(), state->online.end(), [](std::uint8_t value) {
            return value == 0U;
        })) {
            release_connection_noexcept(true);
            return tl::make_unexpected(MotorBusErr::ACTUATOR_OFFLINE);
        }
        if(std::any_of(state->err_code.begin(), state->err_code.end(), [](int value) {
            return value != 0;
        })) {
            release_connection_noexcept(true);
            return tl::make_unexpected(MotorBusErr::ACTUATOR_FAULT);
        }
        return {};
    }
    catch(...) {
        release_connection_noexcept(true);
        return tl::make_unexpected(MotorBusErr::OPEN_FAILED);
    }
}

/**
 * @brief 同步读取六轴状态并转换为 Hardware Contract 单位
 */
tl::expected<ActuatorState, MotorBusErr> Hx10hmMotorBus::read() {
    if(!connected_ || !protocol_) return tl::make_unexpected(MotorBusErr::NOT_CONNECTED);

    const auto raw = protocol_->sync_read_states(servo_ids(), cfg_.read_timeout);
    if(!raw) return tl::make_unexpected(map_read_error(raw.error()));
    if(raw->size() != cfg_.actuators.size()) {
        return tl::make_unexpected(MotorBusErr::READ_FAILED);
    }

    ActuatorState state;
    state.pos.resize(raw->size());
    state.vel.resize(raw->size());
    state.tor.assign(raw->size(), 0.0);
    state.online.assign(raw->size(), 1U);
    state.enabled = enabled_;
    state.err_code.resize(raw->size());

    for(std::size_t i = 0; i < raw->size(); ++i) {
        if((*raw)[i].id != cfg_.actuators[i].servo_id ||
            (*raw)[i].position_raw > HX10HM_MAX_POSITION_RAW) {
            return tl::make_unexpected(MotorBusErr::INVALID_STATE);
        }
        state.pos[i] = raw_position_to_rad(
            (*raw)[i].position_raw,
            cfg_.actuators[i].raw_zero,
            cfg_.actuators[i].direction);
        state.vel[i] = raw_velocity_to_rad_per_second(
            (*raw)[i].velocity_raw,
            cfg_.actuators[i].direction);
        state.err_code[i] = static_cast<int>((*raw)[i].fault);
    }

    if(!finite_vector(state.pos) || !finite_vector(state.vel) || !finite_vector(state.tor)) {
        return tl::make_unexpected(MotorBusErr::INVALID_STATE);
    }

    raw_states_ = *raw;
    online_ = state.online;
    last_feedback_time_ = Clock::now();
    last_state_ = state;
    return state;
}

/**
 * @brief 以零 PWM 安全切换到 PWM Open-Loop 并使能六轴
 */
tl::expected<void, MotorBusErr> Hx10hmMotorBus::activate() {
    if(!connected_ || !protocol_) return tl::make_unexpected(MotorBusErr::NOT_CONNECTED);
    if(active_) return {};

    const auto initial_zero = write_zero_pwm();
    if(!initial_zero) {
        safe_disable_noexcept();
        return tl::make_unexpected(MotorBusErr::STOP_FAILED);
    }
    const auto disabled = set_all_torque(false);
    if(!disabled) {
        safe_disable_noexcept();
        return tl::make_unexpected(MotorBusErr::DISABLE_FAILED);
    }

    for(const auto& actuator : cfg_.actuators) {
        const auto mode = ensure_run_mode(
            actuator.servo_id,
            protocol::hiwonder_bus_servo::RunMode::PwmOpenLoop);
        if(!mode) {
            safe_disable_noexcept();
            return tl::make_unexpected(MotorBusErr::MODE_SWITCH_FAILED);
        }
    }

    const auto mode_zero = write_zero_pwm();
    if(!mode_zero) {
        safe_disable_noexcept();
        return tl::make_unexpected(MotorBusErr::STOP_FAILED);
    }
    const auto enabled = set_all_torque(true);
    if(!enabled) {
        safe_disable_noexcept();
        return tl::make_unexpected(MotorBusErr::ENABLE_FAILED);
    }

    for(std::size_t cycle = 0; cycle < cfg_.startup_read_cycles; ++cycle) {
        const auto state = read();
        if(!state || std::any_of(state->online.begin(), state->online.end(), [](std::uint8_t value) {
            return value == 0U;
        })) {
            safe_disable_noexcept();
            return tl::make_unexpected(MotorBusErr::ACTUATOR_OFFLINE);
        }
        if(std::any_of(state->err_code.begin(), state->err_code.end(), [](int value) {
            return value != 0;
        })) {
            safe_disable_noexcept();
            return tl::make_unexpected(MotorBusErr::ACTUATOR_FAULT);
        }
    }

    active_ = true;
    return {};
}

/**
 * @brief 校验 MIT 命令并下发本阶段的安全零 PWM
 */
tl::expected<void, MotorBusErr> Hx10hmMotorBus::write(const ActuatorCtrlCmd& cmd) {
    if(!active_) return tl::make_unexpected(MotorBusErr::NOT_ACTIVE);
    const auto valid = validate_command(cmd);
    if(!valid) return tl::make_unexpected(valid.error());
    return write_zero_pwm();
}

/**
 * @brief 使用六轴 SYNC WRITE 将 PWM 安全归零
 */
tl::expected<void, MotorBusErr> Hx10hmMotorBus::stop() {
    if(!active_) return tl::make_unexpected(MotorBusErr::NOT_ACTIVE);
    return write_zero_pwm();
}

/**
 * @brief 零 PWM、Torque OFF 并按配置恢复位置模式
 */
tl::expected<void, MotorBusErr> Hx10hmMotorBus::deactivate() {
    if(!connected_ || !protocol_) return tl::make_unexpected(MotorBusErr::NOT_CONNECTED);

    const auto stopped = write_zero_pwm();
    const auto disabled = set_all_torque(false);
    bool mode_failed = false;
    if(cfg_.restore_position_mode_on_deactivate) {
        for(const auto& actuator : cfg_.actuators) {
            if(!ensure_run_mode(actuator.servo_id, protocol::hiwonder_bus_servo::RunMode::Position)) {
                mode_failed = true;
            }
        }
    }

    active_ = false;
    std::fill(enabled_.begin(), enabled_.end(), 0U);
    last_state_.enabled = enabled_;
    if(!disabled) return tl::make_unexpected(MotorBusErr::DISABLE_FAILED);
    if(!stopped) return tl::make_unexpected(MotorBusErr::STOP_FAILED);
    if(mode_failed) return tl::make_unexpected(MotorBusErr::MODE_SWITCH_FAILED);
    return {};
}

/**
 * @brief 执行零 PWM、Torque OFF、flush 并重新验证通信与模式
 */
tl::expected<void, MotorBusErr> Hx10hmMotorBus::recover() {
    if(!configured_) return tl::make_unexpected(MotorBusErr::NOT_CONFIGURED);
    if(!connected_) {
        const auto connected = connect();
        if(!connected) return tl::make_unexpected(connected.error());
    }

    const auto stopped = write_zero_pwm();
    const auto disabled = set_all_torque(false);
    active_ = false;
    std::fill(enabled_.begin(), enabled_.end(), 0U);
    last_state_.enabled = enabled_;

    try {
        serial_.flush(transport::SerialPort::FlushDirection::Input);
    }
    catch(...) {
        return tl::make_unexpected(MotorBusErr::RECOVER_FAILED);
    }

    bool mode_valid = true;
    for(const auto& actuator : cfg_.actuators) {
        const auto mode = protocol_->read_register(
            actuator.servo_id,
            protocol::hiwonder_bus_servo::RUN_MODE_ADDR,
            1U,
            cfg_.read_timeout);
        if(!mode || mode->size() != 1U || (*mode)[0] > 2U) mode_valid = false;
    }
    const auto state = read();
    if(!stopped || !disabled || !mode_valid || !state) {
        return tl::make_unexpected(MotorBusErr::RECOVER_FAILED);
    }
    return {};
}

/**
 * @brief 释放资源，可重复调用且不抛异常
 */
void Hx10hmMotorBus::cleanup() noexcept {
    release_connection_noexcept(false);
}

/**
 * @brief 返回执行器数量
 */
std::size_t Hx10hmMotorBus::size() const noexcept {
    return cfg_.actuators.size();
}

/**
 * @brief 返回配置产生的 HardwareCapabilities
 */
const HardwareCapabilities& Hx10hmMotorBus::capabilities() const noexcept {
    return capabilities_;
}

/**
 * @brief 验证 MIT 命令维度、有限值与能力范围
 */
tl::expected<void, MotorBusErr> Hx10hmMotorBus::validate_command(const ActuatorCtrlCmd& cmd) const {
    if(!configured_) return tl::make_unexpected(MotorBusErr::NOT_CONFIGURED);
    const std::size_t n = cfg_.actuators.size();
    if(cmd.pos.size() != n || cmd.vel.size() != n || cmd.tor.size() != n ||
        cmd.kp.size() != n || cmd.kd.size() != n ||
        !finite_vector(cmd.pos) || !finite_vector(cmd.vel) || !finite_vector(cmd.tor) ||
        !finite_vector(cmd.kp) || !finite_vector(cmd.kd)) {
        return tl::make_unexpected(MotorBusErr::INVALID_CMD);
    }

    constexpr double epsilon = 1e-9;
    for(std::size_t i = 0; i < n; ++i) {
        const auto& capability = capabilities_[i];
        if(cmd.pos[i] < capability.min_pos - epsilon ||
            cmd.pos[i] > capability.max_pos + epsilon ||
            std::abs(cmd.vel[i]) > capability.max_vel + epsilon ||
            std::abs(cmd.tor[i]) > capability.max_effort + epsilon ||
            cmd.kp[i] < 0.0 || cmd.kp[i] > capability.max_kp + epsilon ||
            cmd.kd[i] < 0.0 || cmd.kd[i] > capability.max_kd + epsilon) {
            return tl::make_unexpected(MotorBusErr::INVALID_CMD);
        }
    }
    return {};
}

/**
 * @brief 将 HX 原始位置转换为弧度
 */
double Hx10hmMotorBus::raw_position_to_rad(
    std::uint16_t raw,
    std::uint16_t raw_zero,
    int direction) noexcept {
    const auto delta = static_cast<int>(raw) - static_cast<int>(raw_zero);
    return static_cast<double>(direction * delta) * RAD_PER_STEP;
}

/**
 * @brief 将 BIT15 方向位的原始速度转换为 rad/s
 */
double Hx10hmMotorBus::raw_velocity_to_rad_per_second(
    std::uint16_t raw,
    int direction) noexcept {
    const auto magnitude = static_cast<int>(raw & 0x7FFFU);
    const int signed_steps_per_second = (raw & 0x8000U) != 0U ? -magnitude : magnitude;
    return static_cast<double>(direction * signed_steps_per_second) * RAD_PER_STEP;
}

// ! ========================= 私 有 类 方 法 实 现 ========================= ! //

/**
 * @brief 验证 Backend 配置
 */
tl::expected<void, MotorBusErr> Hx10hmMotorBus::validate_cfg(const HiwonderBusCfg& cfg) const {
    if(cfg.serial_port.empty() || cfg.baudrate != 1000000 ||
        cfg.read_timeout.count() <= 0 || cfg.write_timeout.count() <= 0 ||
        cfg.feedback_timeout.count() <= 0 || cfg.startup_read_cycles == 0U ||
        cfg.actuators.size() != HX10HM_ACTUATOR_COUNT ||
        cfg.velocity_encoding != HiwonderVelocityEncoding::Bit15SignMagnitude ||
        cfg.torque_feedback_mode != "unavailable_zero") {
        return tl::make_unexpected(MotorBusErr::INVALID_CFG);
    }

    std::vector<std::uint8_t> ids;
    ids.reserve(cfg.actuators.size());
    for(const auto& actuator : cfg.actuators) {
        const double endpoint_zero = raw_position_to_rad(0U, actuator.raw_zero, actuator.direction);
        const double endpoint_max = raw_position_to_rad(
            HX10HM_MAX_POSITION_RAW,
            actuator.raw_zero,
            actuator.direction);
        const double representable_min = std::min(endpoint_zero, endpoint_max);
        const double representable_max = std::max(endpoint_zero, endpoint_max);
        if(actuator.name.empty() || actuator.joint_name.empty() ||
            actuator.servo_id >= protocol::hiwonder_bus_servo::BROADCAST_ID ||
            actuator.raw_zero > HX10HM_MAX_POSITION_RAW ||
            (actuator.direction != 1 && actuator.direction != -1) ||
            !std::isfinite(actuator.min_pos) || !std::isfinite(actuator.max_pos) ||
            !std::isfinite(actuator.max_vel) || !std::isfinite(actuator.max_effort) ||
            !std::isfinite(actuator.max_kp) || !std::isfinite(actuator.max_kd) ||
            actuator.min_pos >= actuator.max_pos || actuator.max_vel <= 0.0 ||
            actuator.max_effort <= 0.0 || actuator.max_kp < 0.0 || actuator.max_kd < 0.0 ||
            actuator.min_pos < representable_min || actuator.max_pos > representable_max ||
            std::find(ids.begin(), ids.end(), actuator.servo_id) != ids.end()) {
            return tl::make_unexpected(MotorBusErr::INVALID_CFG);
        }
        ids.push_back(actuator.servo_id);
    }
    return {};
}

/**
 * @brief 获取按配置顺序排列的舅机 ID
 */
std::vector<std::uint8_t> Hx10hmMotorBus::servo_ids() const {
    std::vector<std::uint8_t> ids;
    ids.reserve(cfg_.actuators.size());
    for(const auto& actuator : cfg_.actuators) ids.push_back(actuator.servo_id);
    return ids;
}

/**
 * @brief 同步下发六轴零 PWM
 */
tl::expected<void, MotorBusErr> Hx10hmMotorBus::write_zero_pwm() {
    if(!protocol_) return tl::make_unexpected(MotorBusErr::NOT_CONNECTED);

    std::vector<protocol::hiwonder_bus_servo::PwmCommand> commands;
    commands.reserve(cfg_.actuators.size());
    for(const auto& actuator : cfg_.actuators) {
        commands.push_back(protocol::hiwonder_bus_servo::PwmCommand{ actuator.servo_id, 0 });
    }
    const auto written = protocol_->sync_write_pwm(commands);
    if(!written) return tl::make_unexpected(map_write_error(written.error()));
    return {};
}

/**
 * @brief 逐轴设置 Torque Enable 并消费 ACK
 */
tl::expected<void, MotorBusErr> Hx10hmMotorBus::set_all_torque(bool enable) {
    if(!protocol_) return tl::make_unexpected(MotorBusErr::NOT_CONNECTED);

    bool failed = false;
    MotorBusErr first_error = enable ? MotorBusErr::ENABLE_FAILED : MotorBusErr::DISABLE_FAILED;
    for(std::size_t i = 0; i < cfg_.actuators.size(); ++i) {
        const auto result = protocol_->set_torque_enable(
            cfg_.actuators[i].servo_id,
            enable,
            cfg_.write_timeout);
        if(!result) {
            if(!failed) first_error = map_write_error(result.error());
            failed = true;
        }
        else {
            const auto confirmed = protocol_->read_register(
                cfg_.actuators[i].servo_id,
                protocol::hiwonder_bus_servo::TORQUE_ENABLE_ADDR,
                1U,
                cfg_.read_timeout);
            if(!confirmed || confirmed->size() != 1U ||
                (*confirmed)[0] != static_cast<std::uint8_t>(enable ? 1U : 0U)) {
                if(!failed) {
                    first_error = confirmed ? MotorBusErr::INVALID_STATE :
                        map_read_error(confirmed.error());
                }
                failed = true;
            }
            else if(i < enabled_.size()) {
                enabled_[i] = enable ? 1U : 0U;
            }
        }
    }
    if(failed) return tl::make_unexpected(first_error);
    return {};
}

/**
 * @brief 仅在当前值不同时写入 NVS 运行模式
 */
tl::expected<void, MotorBusErr> Hx10hmMotorBus::ensure_run_mode(
    std::uint8_t id,
    protocol::hiwonder_bus_servo::RunMode mode) {
    if(!protocol_) return tl::make_unexpected(MotorBusErr::NOT_CONNECTED);
    const auto current = protocol_->read_register(
        id,
        protocol::hiwonder_bus_servo::RUN_MODE_ADDR,
        1U,
        cfg_.read_timeout);
    if(!current) return tl::make_unexpected(map_read_error(current.error()));
    if(current->size() != 1U || (*current)[0] > 2U) {
        return tl::make_unexpected(MotorBusErr::INVALID_STATE);
    }
    if((*current)[0] == static_cast<std::uint8_t>(mode)) return {};

    const auto written = protocol_->set_run_mode(id, mode, cfg_.write_timeout);
    if(!written) return tl::make_unexpected(map_write_error(written.error()));
    const auto confirmed = protocol_->read_register(
        id,
        protocol::hiwonder_bus_servo::RUN_MODE_ADDR,
        1U,
        cfg_.read_timeout);
    if(!confirmed) return tl::make_unexpected(map_read_error(confirmed.error()));
    if(confirmed->size() != 1U || (*confirmed)[0] != static_cast<std::uint8_t>(mode)) {
        return tl::make_unexpected(MotorBusErr::INVALID_STATE);
    }
    return {};
}

/**
 * @brief 将协议读取错误转换为 MotorBusErr
 */
MotorBusErr Hx10hmMotorBus::map_read_error(protocol::hiwonder_bus_servo::Err error) noexcept {
    using ProtocolErr = protocol::hiwonder_bus_servo::Err;
    if(error == ProtocolErr::NOT_OPEN) return MotorBusErr::NOT_CONNECTED;
    if(error == ProtocolErr::TIMEOUT) return MotorBusErr::TIMEOUT;
    if(error == ProtocolErr::DEVICE_ERROR) return MotorBusErr::ACTUATOR_FAULT;
    return MotorBusErr::READ_FAILED;
}

/**
 * @brief 将协议写入错误转换为 MotorBusErr
 */
MotorBusErr Hx10hmMotorBus::map_write_error(protocol::hiwonder_bus_servo::Err error) noexcept {
    using ProtocolErr = protocol::hiwonder_bus_servo::Err;
    if(error == ProtocolErr::NOT_OPEN) return MotorBusErr::NOT_CONNECTED;
    if(error == ProtocolErr::TIMEOUT) return MotorBusErr::TIMEOUT;
    if(error == ProtocolErr::DEVICE_ERROR) return MotorBusErr::ACTUATOR_FAULT;
    return MotorBusErr::WRITE_FAILED;
}

/**
 * @brief 尽力零 PWM 并 Torque OFF
 */
void Hx10hmMotorBus::safe_disable_noexcept() noexcept {
    if(protocol_ && serial_.is_open()) {
        try {
            (void)write_zero_pwm();
            (void)set_all_torque(false);
            serial_.flush(transport::SerialPort::FlushDirection::Input);
        }
        catch(...) {
        }
    }
    active_ = false;
    std::fill(enabled_.begin(), enabled_.end(), 0U);
    last_state_.enabled = enabled_;
}

/**
 * @brief 释放连接资源
 */
void Hx10hmMotorBus::release_connection_noexcept(bool keep_config) noexcept {
    safe_disable_noexcept();
    protocol_.reset();
    serial_.close();
    raw_states_.clear();
    online_.clear();
    enabled_.clear();
    last_state_ = ActuatorState{};
    last_feedback_time_ = TimePoint{};
    connected_ = false;
    active_ = false;
    if(!keep_config) {
        cfg_ = HiwonderBusCfg{};
        capabilities_.clear();
        configured_ = false;
    }
}

} // namespace serial_arm

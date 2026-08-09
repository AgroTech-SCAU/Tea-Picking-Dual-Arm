#include "serial_arm_protocol_hiwonder_bus_servo/bus.hpp"

#include <array>

namespace serial_arm::protocol::hiwonder_bus_servo {

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //



// ! ========================= 私 有 量 / 工 具 函 数 实 现 ========================= ! //

namespace {

constexpr std::size_t MIN_PACKET_SIZE = 6;
constexpr std::size_t MAX_PARAMETER_SIZE = 253;
constexpr std::size_t STATE_BLOCK_SIZE = 10;
constexpr std::uint16_t DIRECTION_BIT_10 = 0x0400;
constexpr std::uint16_t MAGNITUDE_MASK_10 = 0x03FF;

/**
 * @brief 检查单舅机 ID 是否合法
 */
bool valid_servo_id(std::uint8_t id) noexcept {
    return id < BROADCAST_ID;
}

/**
 * @brief 检查 ID 列表非空、合法且无重复
 */
bool valid_servo_ids(const std::vector<std::uint8_t>& ids) noexcept {
    if(ids.empty()) return false;

    std::array<bool, BROADCAST_ID> seen{};
    for(const auto id : ids) {
        if(!valid_servo_id(id) || seen[id]) return false;
        seen[id] = true;
    }
    return true;
}

/**
 * @brief 构造通用 HX 指令包
 */
tl::expected<Buffer, Err> encode_instruction_packet(
    std::uint8_t id,
    std::uint8_t instruction,
    const Buffer& parameters) {
    if(parameters.size() > MAX_PARAMETER_SIZE) {
        return tl::make_unexpected(Err::INVALID_ARGUMENT);
    }

    Buffer packet;
    packet.reserve(parameters.size() + 6U);
    packet.push_back(0xFF);
    packet.push_back(0xFF);
    packet.push_back(id);
    packet.push_back(static_cast<std::uint8_t>(parameters.size() + 2U));
    packet.push_back(instruction);
    packet.insert(packet.end(), parameters.begin(), parameters.end());
    packet.push_back(checksum(packet));
    return packet;
}

/**
 * @brief 按低字节在前解码 16 位无符号数
 */
std::uint16_t decode_u16_le(const Buffer& data, std::size_t offset) noexcept {
    return static_cast<std::uint16_t>(data[offset]) |
        static_cast<std::uint16_t>(static_cast<std::uint16_t>(data[offset + 1U]) << 8U);
}

/**
 * @brief 按 BIT10 方向位解码符号幅值
 */
std::int16_t decode_signed_magnitude_10(std::uint16_t wire) noexcept {
    const auto magnitude = static_cast<std::int16_t>(wire & MAGNITUDE_MASK_10);
    return (wire & DIRECTION_BIT_10) != 0U ? static_cast<std::int16_t>(-magnitude) : magnitude;
}

/**
 * @brief 按 BIT10 方向位编码符号幅值
 */
tl::expected<std::uint16_t, Err> encode_signed_magnitude_10(std::int16_t value) {
    if(value < -1000 || value > 1000) {
        return tl::make_unexpected(Err::INVALID_ARGUMENT);
    }
    const auto magnitude = static_cast<std::uint16_t>(value < 0 ? -value : value);
    return value < 0 ? static_cast<std::uint16_t>(magnitude | DIRECTION_BIT_10) : magnitude;
}

/**
 * @brief 检查状态包的 ID、ERROR 和参数长度
 */
tl::expected<void, Err> validate_status(
    const StatusPacket& status,
    std::uint8_t expected_id,
    std::size_t expected_parameter_size) {
    if(status.id != expected_id) return tl::make_unexpected(Err::UNEXPECTED_ID);
    if(status.error != 0U) return tl::make_unexpected(Err::DEVICE_ERROR);
    if(status.parameters.size() != expected_parameter_size) {
        return tl::make_unexpected(Err::MALFORMED_PACKET);
    }
    return {};
}

} // namespace

// ! ========================= 接 口 类 方 法 / 函 数 实 现 ========================= ! //

/**
 * @brief 计算 HX 报文校验和
 */
std::uint8_t checksum(const Buffer& packet_without_checksum) noexcept {
    std::uint8_t sum = 0;
    if(packet_without_checksum.size() <= 2U) return static_cast<std::uint8_t>(~sum);
    for(std::size_t i = 2; i < packet_without_checksum.size(); ++i) {
        sum = static_cast<std::uint8_t>(sum + packet_without_checksum[i]);
    }
    return static_cast<std::uint8_t>(~sum);
}

/**
 * @brief 构造 READ DATA 指令包
 */
tl::expected<Buffer, Err> encode_read_packet(
    std::uint8_t id,
    std::uint8_t address,
    std::uint8_t length) {
    if(!valid_servo_id(id) || length == 0U || length > MAX_PARAMETER_SIZE) {
        return tl::make_unexpected(Err::INVALID_ARGUMENT);
    }
    return encode_instruction_packet(id, READ_DATA, { address, length });
}

/**
 * @brief 构造 WRITE DATA 指令包
 */
tl::expected<Buffer, Err> encode_write_packet(
    std::uint8_t id,
    std::uint8_t address,
    const Buffer& data) {
    if(!valid_servo_id(id) || data.empty() || data.size() > 252U) {
        return tl::make_unexpected(Err::INVALID_ARGUMENT);
    }

    Buffer parameters;
    parameters.reserve(data.size() + 1U);
    parameters.push_back(address);
    parameters.insert(parameters.end(), data.begin(), data.end());
    return encode_instruction_packet(id, WRITE_DATA, parameters);
}

/**
 * @brief 构造 SYNC READ 指令包
 */
tl::expected<Buffer, Err> encode_sync_read_packet(
    const std::vector<std::uint8_t>& ids,
    std::uint8_t address,
    std::uint8_t length) {
    if(!valid_servo_ids(ids) || ids.size() > 251U ||
        length == 0U || length > MAX_PARAMETER_SIZE) {
        return tl::make_unexpected(Err::INVALID_ARGUMENT);
    }

    Buffer parameters;
    parameters.reserve(ids.size() + 2U);
    parameters.push_back(address);
    parameters.push_back(length);
    parameters.insert(parameters.end(), ids.begin(), ids.end());
    return encode_instruction_packet(BROADCAST_ID, SYNC_READ, parameters);
}

/**
 * @brief 构造 SYNC WRITE 指令包
 */
tl::expected<Buffer, Err> encode_sync_write_packet(
    std::uint8_t address,
    std::uint8_t data_length,
    const std::vector<SyncWriteEntry>& entries) {
    if(entries.empty() || data_length == 0U) {
        return tl::make_unexpected(Err::INVALID_ARGUMENT);
    }

    std::vector<std::uint8_t> ids;
    ids.reserve(entries.size());
    const std::size_t parameter_size = 2U + entries.size() * (static_cast<std::size_t>(data_length) + 1U);
    if(parameter_size > MAX_PARAMETER_SIZE) {
        return tl::make_unexpected(Err::INVALID_ARGUMENT);
    }
    for(const auto& entry : entries) {
        ids.push_back(entry.id);
        if(entry.data.size() != data_length) {
            return tl::make_unexpected(Err::INVALID_ARGUMENT);
        }
    }
    if(!valid_servo_ids(ids)) return tl::make_unexpected(Err::INVALID_ARGUMENT);

    Buffer parameters;
    parameters.reserve(parameter_size);
    parameters.push_back(address);
    parameters.push_back(data_length);
    for(const auto& entry : entries) {
        parameters.push_back(entry.id);
        parameters.insert(parameters.end(), entry.data.begin(), entry.data.end());
    }
    return encode_instruction_packet(BROADCAST_ID, SYNC_WRITE, parameters);
}

/**
 * @brief 解析并校验状态应答包
 */
tl::expected<StatusPacket, Err> parse_status_packet(const Buffer& packet) {
    if(packet.size() < MIN_PACKET_SIZE || packet[0] != 0xFF || packet[1] != 0xFF ||
        packet[2] >= BROADCAST_ID || packet[3] < 2U ||
        packet.size() != static_cast<std::size_t>(packet[3]) + 4U) {
        return tl::make_unexpected(Err::MALFORMED_PACKET);
    }

    const Buffer without_checksum(packet.begin(), packet.end() - 1);
    if(checksum(without_checksum) != packet.back()) {
        return tl::make_unexpected(Err::CHECKSUM_MISMATCH);
    }

    StatusPacket status;
    status.id = packet[2];
    status.error = packet[4];
    status.parameters.assign(packet.begin() + 5, packet.end() - 1);
    return status;
}

/**
 * @brief 解码 HX-10HM 原始状态块
 */
tl::expected<RawState, Err> decode_raw_state(
    const StatusPacket& state_packet,
    const StatusPacket& current_packet) {
    if(state_packet.id != current_packet.id) {
        return tl::make_unexpected(Err::UNEXPECTED_ID);
    }
    if(state_packet.error != 0U || current_packet.error != 0U) {
        return tl::make_unexpected(Err::DEVICE_ERROR);
    }
    if(state_packet.parameters.size() != STATE_BLOCK_SIZE || current_packet.parameters.size() != 2U) {
        return tl::make_unexpected(Err::MALFORMED_PACKET);
    }

    RawState state;
    state.id = state_packet.id;
    state.position_raw = decode_u16_le(state_packet.parameters, 0U);
    state.velocity_raw = decode_u16_le(state_packet.parameters, 2U);
    state.load_raw = decode_signed_magnitude_10(decode_u16_le(state_packet.parameters, 4U));
    state.voltage_raw = state_packet.parameters[6];
    state.temperature_raw = state_packet.parameters[7];
    state.registered = state_packet.parameters[8];
    state.fault = state_packet.parameters[9];
    state.current_raw_ma = decode_u16_le(current_packet.parameters, 0U);
    return state;
}

/**
 * @brief 绑定 SerialArm-Core 串口传输对象
 */
HiwonderBusServo::HiwonderBusServo(transport::SerialPort& serial) noexcept
    : serial_(serial) {}

/**
 * @brief 读取单舅机连续寄存器
 */
tl::expected<Buffer, Err> HiwonderBusServo::read_register(
    std::uint8_t id,
    std::uint8_t address,
    std::uint8_t length,
    std::chrono::milliseconds timeout) {
    const auto request = encode_read_packet(id, address, length);
    if(!request) return tl::make_unexpected(request.error());
    const auto prepared = prepare_transaction(timeout);
    if(!prepared) return tl::make_unexpected(prepared.error());
    const auto sent = transmit(*request);
    if(!sent) return tl::make_unexpected(sent.error());

    const auto status = receive_status_packet();
    if(!status) return tl::make_unexpected(status.error());
    const auto valid = validate_status(*status, id, length);
    if(!valid) return tl::make_unexpected(valid.error());
    return status->parameters;
}

/**
 * @brief 写入单舅机连续寄存器并消费状态 ACK
 */
tl::expected<void, Err> HiwonderBusServo::write_register(
    std::uint8_t id,
    std::uint8_t address,
    const Buffer& data,
    std::chrono::milliseconds timeout) {
    const auto request = encode_write_packet(id, address, data);
    if(!request) return tl::make_unexpected(request.error());
    const auto prepared = prepare_transaction(timeout);
    if(!prepared) return tl::make_unexpected(prepared.error());
    const auto sent = transmit(*request);
    if(!sent) return tl::make_unexpected(sent.error());

    const auto status = receive_status_packet();
    if(!status) return tl::make_unexpected(status.error());
    return validate_status(*status, id, 0U);
}

/**
 * @brief 同步读取多舅机连续寄存器
 */
tl::expected<std::vector<StatusPacket>, Err> HiwonderBusServo::sync_read(
    const std::vector<std::uint8_t>& ids,
    std::uint8_t address,
    std::uint8_t length,
    std::chrono::milliseconds timeout) {
    const auto request = encode_sync_read_packet(ids, address, length);
    if(!request) return tl::make_unexpected(request.error());
    const auto prepared = prepare_transaction(timeout);
    if(!prepared) return tl::make_unexpected(prepared.error());
    const auto sent = transmit(*request);
    if(!sent) return tl::make_unexpected(sent.error());

    std::vector<StatusPacket> result;
    result.reserve(ids.size());
    for(const auto id : ids) {
        const auto status = receive_status_packet();
        if(!status) return tl::make_unexpected(status.error());
        const auto valid = validate_status(*status, id, length);
        if(!valid) return tl::make_unexpected(valid.error());
        result.push_back(*status);
    }
    return result;
}

/**
 * @brief 同步写入多舅机连续寄存器
 */
tl::expected<void, Err> HiwonderBusServo::sync_write(
    std::uint8_t address,
    std::uint8_t data_length,
    const std::vector<SyncWriteEntry>& entries) {
    const auto request = encode_sync_write_packet(address, data_length, entries);
    if(!request) return tl::make_unexpected(request.error());
    if(!serial_.is_open()) return tl::make_unexpected(Err::NOT_OPEN);
    try {
        serial_.flush(transport::SerialPort::FlushDirection::Input);
    }
    catch(...) {
        return tl::make_unexpected(Err::READ_FAILED);
    }
    return transmit(*request);
}

/**
 * @brief 设置舅机运行模式
 */
tl::expected<void, Err> HiwonderBusServo::set_run_mode(
    std::uint8_t id,
    RunMode mode,
    std::chrono::milliseconds timeout) {
    const auto value = static_cast<std::uint8_t>(mode);
    if(value > static_cast<std::uint8_t>(RunMode::PwmOpenLoop)) {
        return tl::make_unexpected(Err::INVALID_ARGUMENT);
    }
    return write_register(id, RUN_MODE_ADDR, { value }, timeout);
}

/**
 * @brief 设置舅机扭矩使能
 */
tl::expected<void, Err> HiwonderBusServo::set_torque_enable(
    std::uint8_t id,
    bool enable,
    std::chrono::milliseconds timeout) {
    return write_register(id, TORQUE_ENABLE_ADDR, { static_cast<std::uint8_t>(enable ? 1U : 0U) }, timeout);
}

/**
 * @brief 写入单舅机 PWM 开环命令
 */
tl::expected<void, Err> HiwonderBusServo::write_pwm(
    std::uint8_t id,
    std::int16_t pwm,
    std::chrono::milliseconds timeout) {
    const auto encoded = encode_signed_magnitude_10(pwm);
    if(!encoded) return tl::make_unexpected(encoded.error());
    return write_register(id, PWM_COMMAND_ADDR, {
        static_cast<std::uint8_t>(*encoded & 0xFFU),
        static_cast<std::uint8_t>((*encoded >> 8U) & 0xFFU),
        }, timeout);
}

/**
 * @brief 使用一帧 SYNC WRITE 写入多舅机 PWM
 */
tl::expected<void, Err> HiwonderBusServo::sync_write_pwm(const std::vector<PwmCommand>& commands) {
    std::vector<SyncWriteEntry> entries;
    entries.reserve(commands.size());
    for(const auto& command : commands) {
        const auto encoded = encode_signed_magnitude_10(command.pwm);
        if(!encoded) return tl::make_unexpected(encoded.error());
        entries.push_back(SyncWriteEntry{ command.id, {
            static_cast<std::uint8_t>(*encoded & 0xFFU),
            static_cast<std::uint8_t>((*encoded >> 8U) & 0xFFU),
        } });
    }
    return sync_write(PWM_COMMAND_ADDR, 2U, entries);
}

/**
 * @brief 读取原始位置
 */
tl::expected<std::uint16_t, Err> HiwonderBusServo::read_position(
    std::uint8_t id,
    std::chrono::milliseconds timeout) {
    const auto data = read_register(id, PRESENT_POSITION_ADDR, 2U, timeout);
    if(!data) return tl::make_unexpected(data.error());
    return decode_u16_le(*data, 0U);
}

/**
 * @brief 读取原始速度字
 */
tl::expected<std::uint16_t, Err> HiwonderBusServo::read_velocity(
    std::uint8_t id,
    std::chrono::milliseconds timeout) {
    const auto data = read_register(id, PRESENT_VELOCITY_ADDR, 2U, timeout);
    if(!data) return tl::make_unexpected(data.error());
    return decode_u16_le(*data, 0U);
}

/**
 * @brief 读取有符号原始负载
 */
tl::expected<std::int16_t, Err> HiwonderBusServo::read_load(
    std::uint8_t id,
    std::chrono::milliseconds timeout) {
    const auto data = read_register(id, PRESENT_LOAD_ADDR, 2U, timeout);
    if(!data) return tl::make_unexpected(data.error());
    return decode_signed_magnitude_10(decode_u16_le(*data, 0U));
}

/**
 * @brief 读取舅机故障位图
 */
tl::expected<std::uint8_t, Err> HiwonderBusServo::read_fault(
    std::uint8_t id,
    std::chrono::milliseconds timeout) {
    const auto data = read_register(id, FAULT_ADDR, 1U, timeout);
    if(!data) return tl::make_unexpected(data.error());
    return (*data)[0];
}

/**
 * @brief 读取舅机原始电流
 */
tl::expected<std::uint16_t, Err> HiwonderBusServo::read_current(
    std::uint8_t id,
    std::chrono::milliseconds timeout) {
    const auto data = read_register(id, PRESENT_CURRENT_ADDR, 2U, timeout);
    if(!data) return tl::make_unexpected(data.error());
    return decode_u16_le(*data, 0U);
}

/**
 * @brief 使用两次 SYNC READ 读取多舅机原始状态与电流
 */
tl::expected<std::vector<RawState>, Err> HiwonderBusServo::sync_read_states(
    const std::vector<std::uint8_t>& ids,
    std::chrono::milliseconds timeout) {
    const auto states = sync_read(ids, PRESENT_POSITION_ADDR, STATE_BLOCK_SIZE, timeout);
    if(!states) return tl::make_unexpected(states.error());
    const auto currents = sync_read(ids, PRESENT_CURRENT_ADDR, 2U, timeout);
    if(!currents) return tl::make_unexpected(currents.error());

    std::vector<RawState> result;
    result.reserve(ids.size());
    for(std::size_t i = 0; i < ids.size(); ++i) {
        const auto state = decode_raw_state((*states)[i], (*currents)[i]);
        if(!state) return tl::make_unexpected(state.error());
        result.push_back(*state);
    }
    return result;
}

// ! ========================= 私 有 类 方 法 实 现 ========================= ! //

/**
 * @brief 清理输入缓冲区并设置读取超时
 */
tl::expected<void, Err> HiwonderBusServo::prepare_transaction(std::chrono::milliseconds timeout) {
    if(!serial_.is_open()) return tl::make_unexpected(Err::NOT_OPEN);
    if(timeout.count() <= 0) return tl::make_unexpected(Err::INVALID_ARGUMENT);
    try {
        serial_.set_read_timeout(timeout);
        serial_.flush(transport::SerialPort::FlushDirection::Input);
        return {};
    }
    catch(...) {
        return tl::make_unexpected(Err::READ_FAILED);
    }
}

/**
 * @brief 发送完整协议报文
 */
tl::expected<void, Err> HiwonderBusServo::transmit(const Buffer& packet) {
    if(!serial_.is_open()) return tl::make_unexpected(Err::NOT_OPEN);
    try {
        if(serial_.write(packet) != packet.size()) {
            return tl::make_unexpected(Err::TIMEOUT);
        }
        serial_.drain();
        return {};
    }
    catch(...) {
        return tl::make_unexpected(Err::WRITE_FAILED);
    }
}

/**
 * @brief 从串口接收并解析一个状态包
 */
tl::expected<StatusPacket, Err> HiwonderBusServo::receive_status_packet() {
    try {
        std::uint8_t byte = 0;
        bool first_header = false;
        while(true) {
            if(serial_.read_exact(&byte, 1U) != 1U) {
                return tl::make_unexpected(Err::TIMEOUT);
            }
            if(byte == 0xFF) {
                if(first_header) break;
                first_header = true;
            }
            else {
                first_header = false;
            }
        }

        std::array<std::uint8_t, 2> id_and_length{};
        if(serial_.read_exact(id_and_length.data(), id_and_length.size()) != id_and_length.size()) {
            return tl::make_unexpected(Err::TIMEOUT);
        }
        if(id_and_length[0] >= BROADCAST_ID || id_and_length[1] < 2U) {
            return tl::make_unexpected(Err::MALFORMED_PACKET);
        }

        Buffer tail(id_and_length[1]);
        if(serial_.read_exact(tail.data(), tail.size()) != tail.size()) {
            return tl::make_unexpected(Err::TIMEOUT);
        }

        Buffer packet{ 0xFF, 0xFF, id_and_length[0], id_and_length[1] };
        packet.insert(packet.end(), tail.begin(), tail.end());
        return parse_status_packet(packet);
    }
    catch(...) {
        return tl::make_unexpected(Err::READ_FAILED);
    }
}

} // namespace serial_arm::protocol::hiwonder_bus_servo

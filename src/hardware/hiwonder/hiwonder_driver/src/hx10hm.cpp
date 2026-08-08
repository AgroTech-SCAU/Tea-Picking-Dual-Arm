#include "hiwonder_driver/hx10hm.hpp"

#include <stdexcept>
#include <string>

namespace {

void validate_servo_id(std::uint8_t id) {
    if(id > 0xFD) {
        throw std::invalid_argument("HX-10HM id 范围必须为 [0, 253]");
    }
}

void validate_position_and_speed(std::uint16_t raw, std::uint16_t speed) {
    if(raw > 4095U) {
        throw std::out_of_range("HX-10HM 目标位置范围必须为 [0, 4095]");
    }
    if(speed > 3400U) {
        throw std::out_of_range("HX-10HM 位置模式速度范围必须为 [0, 3400]");
    }
}

}  // namespace

std::uint16_t Hx10hm::read_pos_raw(std::uint8_t id) {
    validate_servo_id(id);

    std::vector<std::uint8_t> tx{
        0xFF,
        0xFF,
        id,
        0x04,
        READ_DATA,
        PRESENT_POS_ADDR,
        0x02,
    };
    tx.push_back(check_sum(tx));

    serial_.flush(SerialPort::FlushDirection::Input);

    if(serial_.write(tx) != tx.size()) {
        throw std::runtime_error("HX-10HM READ DATA 请求写入超时, id=" + std::to_string(id));
    }
    serial_.drain();

    const auto rx = serial_.read_exact(8);
    if(rx.size() != 8) {
        throw std::runtime_error("HX-10HM READ DATA 响应超时, id=" + std::to_string(id));
    }
    if(rx[0] != 0xFF || rx[1] != 0xFF || rx[2] != id || rx[3] != 0x04) {
        throw std::runtime_error(
            "HX-10HM READ DATA 响应 header/id/length 不匹配, id=" + std::to_string(id));
    }

    const std::uint8_t expected =
        check_sum(std::vector<std::uint8_t>(rx.begin(), rx.end() - 1));
    if(expected != rx.back()) {
        throw std::runtime_error("HX-10HM READ DATA 校验和不匹配, id=" + std::to_string(id));
    }
    if(rx[4] != 0x00) {
        throw std::runtime_error(
            "HX-10HM READ DATA 错误码=" + std::to_string(rx[4]) +
            ", id=" + std::to_string(id));
    }

    return static_cast<std::uint16_t>(rx[5]) |
        (static_cast<std::uint16_t>(rx[6]) << 8U);
}

std::array<std::uint16_t, Hx10hm::JOINT_COUNT> Hx10hm::read_all_pos_raw() {
    std::array<std::uint16_t, JOINT_COUNT> result{};

    for(std::uint8_t id = 1; id <= JOINT_COUNT; ++id) {
        result[static_cast<std::size_t>(id - 1U)] = read_pos_raw(id);
    }

    return result;
}

void Hx10hm::write_pos_raw(
    std::uint8_t id,
    std::uint16_t raw,
    std::uint16_t speed,
    std::uint16_t time) {
    validate_servo_id(id);
    validate_position_and_speed(raw, speed);

    const std::vector<std::uint8_t> data{
        static_cast<std::uint8_t>(raw & 0xFFU),
        static_cast<std::uint8_t>((raw >> 8U) & 0xFFU),
        static_cast<std::uint8_t>(time & 0xFFU),
        static_cast<std::uint8_t>((time >> 8U) & 0xFFU),
        static_cast<std::uint8_t>(speed & 0xFFU),
        static_cast<std::uint8_t>((speed >> 8U) & 0xFFU),
    };

    write_data(id, GOAL_POSITION_ADDR, data);
}

void Hx10hm::sync_write_all_pos_raw(
    const std::array<std::uint16_t, JOINT_COUNT>& raw,
    std::uint16_t speed,
    std::uint16_t time) {
    for(const auto value : raw) {
        validate_position_and_speed(value, speed);
    }

    std::vector<std::uint8_t> tx;
    tx.reserve(7U + JOINT_COUNT * 7U);
    tx.push_back(0xFF);
    tx.push_back(0xFF);
    tx.push_back(BROADCAST_ID);

    const std::size_t parameter_count = 2U + JOINT_COUNT * 7U;
    tx.push_back(static_cast<std::uint8_t>(parameter_count + 2U));
    tx.push_back(SYNC_WRITE);
    tx.push_back(GOAL_POSITION_ADDR);
    tx.push_back(0x06);

    for(std::uint8_t id = 1; id <= JOINT_COUNT; ++id) {
        const auto value = raw[static_cast<std::size_t>(id - 1U)];
        tx.push_back(id);
        tx.push_back(static_cast<std::uint8_t>(value & 0xFFU));
        tx.push_back(static_cast<std::uint8_t>((value >> 8U) & 0xFFU));
        tx.push_back(static_cast<std::uint8_t>(time & 0xFFU));
        tx.push_back(static_cast<std::uint8_t>((time >> 8U) & 0xFFU));
        tx.push_back(static_cast<std::uint8_t>(speed & 0xFFU));
        tx.push_back(static_cast<std::uint8_t>((speed >> 8U) & 0xFFU));
    }

    tx.push_back(check_sum(tx));

    serial_.flush(SerialPort::FlushDirection::Input);
    if(serial_.write(tx) != tx.size()) {
        throw std::runtime_error("HX-10HM SYNC WRITE 写入超时");
    }
    serial_.drain();
}

void Hx10hm::set_torque(std::uint8_t id, bool enable) {
    validate_servo_id(id);
    write_data(
        id,
        TORQUE_ENABLE_ADDR,
        { static_cast<std::uint8_t>(enable ? 1U : 0U) });
}

void Hx10hm::set_all_torque(bool enable) {
    for(std::uint8_t id = 1; id <= JOINT_COUNT; ++id) {
        set_torque(id, enable);
    }
}

double Hx10hm::raw_to_degree(std::uint16_t raw) {
    if(raw > 4095U) {
        throw std::out_of_range("HX-10HM 原始数据范围必须为 [0, 4095]");
    }

    constexpr double kDegreePerCount = 360.0 / 4096.0;
    return (static_cast<int>(raw) - static_cast<int>(CENTER_RAW)) * kDegreePerCount;
}

std::uint8_t Hx10hm::check_sum(const std::vector<std::uint8_t>& packet) {
    if(packet.size() < 5) {
        throw std::invalid_argument("HX-10HM 数据包太短");
    }

    std::uint8_t sum = 0;
    for(std::size_t i = 2; i < packet.size(); ++i) {
        sum = static_cast<std::uint8_t>(sum + packet[i]);
    }
    return static_cast<std::uint8_t>(~sum);
}

void Hx10hm::write_data(
    std::uint8_t id,
    std::uint8_t start_addr,
    const std::vector<std::uint8_t>& data) {
    validate_servo_id(id);

    if(data.empty()) {
        throw std::invalid_argument("HX-10HM WRITE DATA 不能为空");
    }
    if(data.size() > 252U) {
        throw std::invalid_argument("HX-10HM WRITE DATA 参数过长");
    }

    const auto length = static_cast<std::uint8_t>(data.size() + 3U);

    std::vector<std::uint8_t> tx;
    tx.reserve(data.size() + 7U);
    tx.push_back(0xFF);
    tx.push_back(0xFF);
    tx.push_back(id);
    tx.push_back(length);
    tx.push_back(WRITE_DATA);
    tx.push_back(start_addr);
    tx.insert(tx.end(), data.begin(), data.end());
    tx.push_back(check_sum(tx));

    serial_.flush(SerialPort::FlushDirection::Input);

    if(serial_.write(tx) != tx.size()) {
        throw std::runtime_error("HX-10HM WRITE DATA 写入超时, id=" + std::to_string(id));
    }
    serial_.drain();

    read_write_status(id);
}

void Hx10hm::read_write_status(std::uint8_t expected_id) {
    const auto rx = serial_.read_exact(6);
    if(rx.size() != 6) {
        throw std::runtime_error(
            "HX-10HM WRITE DATA ACK 超时, id=" + std::to_string(expected_id));
    }

    if(rx[0] != 0xFF || rx[1] != 0xFF ||
        rx[2] != expected_id || rx[3] != 0x02) {
        throw std::runtime_error(
            "HX-10HM WRITE DATA ACK header/id/length 不匹配, id=" +
            std::to_string(expected_id));
    }

    const std::uint8_t expected =
        check_sum(std::vector<std::uint8_t>(rx.begin(), rx.end() - 1));
    if(expected != rx.back()) {
        throw std::runtime_error(
            "HX-10HM WRITE DATA ACK 校验和不匹配, id=" +
            std::to_string(expected_id));
    }

    if(rx[4] != 0x00) {
        throw std::runtime_error(
            "HX-10HM WRITE DATA ERROR=" + std::to_string(rx[4]) +
            ", id=" + std::to_string(expected_id));
    }
}

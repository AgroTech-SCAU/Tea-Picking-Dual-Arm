#include "hiwonder_driver/hx10hm.hpp"

#include <stdexcept>
#include <string>

namespace {

void validate_servo_id(std::uint8_t id) {
    if(id > 0xFD) {
        throw std::invalid_argument("HX-10HM id 范围必须为 [0, 253]");
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
        READ_DATA_ADDR,
        PRESENT_POS_ADDR,
        0x02,
    };
    tx.push_back(check_sum(tx));

    // 每次读请求前清掉旧回复，避免上一次 WRITE 状态包或残留字节污染本次解析
    serial_.flush(SerialPort::FlushDirection::Input);

    if(serial_.write(tx) != tx.size()) {
        throw std::runtime_error("HX-10HM 请求写入超时");
    }
    serial_.drain();

    auto rx = serial_.read_exact(8);
    if(rx.size() != 8) {
        throw std::runtime_error("HX-10HM 响应超时, id=" + std::to_string(id));
    }
    if(rx[0] != 0xFF || rx[1] != 0xFF || rx[2] != id || rx[3] != 0x04) {
        throw std::runtime_error("HX-10HM 响应 header/id/length 不匹配");
    }

    const std::uint8_t expected = check_sum(std::vector<std::uint8_t>(rx.begin(), rx.end() - 1));
    if(expected != rx.back()) {
        throw std::runtime_error("HX-10HM 校验和不匹配");
    }
    if(rx[4] != 0x00) {
        throw std::runtime_error("HX-10HM 错误码=" + std::to_string(rx[4]));
    }

    return static_cast<std::uint16_t>(rx[5]) | (static_cast<std::uint16_t>(rx[6]) << 8U);
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
    std::uint8_t acceleration) {
    validate_servo_id(id);

    if(raw > 4095U) {
        throw std::out_of_range("HX-10HM V1 目标位置范围必须为 [0, 4095]");
    }
    if(speed > 3400U) {
        throw std::out_of_range("HX-10HM 位置模式速度范围必须为 [0, 3400]");
    }

    const std::vector<std::uint8_t> data{
        acceleration,
        static_cast<std::uint8_t>(raw & 0xFFU),
        static_cast<std::uint8_t>((raw >> 8U) & 0xFFU),
        0x00,
        0x00,
        static_cast<std::uint8_t>(speed & 0xFFU),
        static_cast<std::uint8_t>((speed >> 8U) & 0xFFU),
    };

    write_data(id, GOAL_ACC_ADDR, data);
}

void Hx10hm::write_all_pos_raw(
    const std::array<std::uint16_t, JOINT_COUNT>& raw,
    std::uint16_t speed,
    std::uint8_t acceleration) {
    for(std::uint8_t id = 1; id <= JOINT_COUNT; ++id) {
        write_pos_raw(id, raw[static_cast<std::size_t>(id - 1U)], speed, acceleration);
    }
}

void Hx10hm::set_torque(std::uint8_t id, bool enable) {
    validate_servo_id(id);
    write_data(id, TORQUE_ENABLE_ADDR, { static_cast<std::uint8_t>(enable ? 1U : 0U) });
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
    tx.push_back(WRITE_DATA_ADDR);
    tx.push_back(start_addr);
    tx.insert(tx.end(), data.begin(), data.end());
    tx.push_back(check_sum(tx));

    // 写命令前清空旧输入，避免不同状态返回级别下残留的 ACK 影响后续读操作
    serial_.flush(SerialPort::FlushDirection::Input);

    if(serial_.write(tx) != tx.size()) {
        throw std::runtime_error("HX-10HM WRITE DATA 写入超时");
    }
    serial_.drain();
}

#include "hiwonder_driver/hx10hm.hpp"

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //



// ! ========================= 私 有 量 / 工 具 函 数 实 现 ========================= ! //



// ! ========================= 接 口 类 方 法 / 函 数 实 现 ========================= ! //

/**
 * @brief 读取指定舵机的原始位置
 * @param id 舵机编号
 * @return 舵机原始位置计数值
 */
uint16_t Hx10hm::read_pos_raw(uint8_t id) {
    if(id > 0xFD) {
        throw std::invalid_argument("HX-10HM id 范围必须为 [0, 254)");
    }

    std::vector<uint8_t> tx{ 0xFF, 0xFF, id, 0x04, READ_DATA_ADDR, PRESENT_POS_ADDR, 0x02, };
    tx.push_back(check_sum(tx));

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

    const uint8_t expected = check_sum(std::vector<uint8_t>(rx.begin(), rx.end() - 1));
    if(expected != rx.back()) {
        throw std::runtime_error("HX-10HM 总数不匹配");
    }
    if(rx[4] != 0x00) {
        throw std::runtime_error("HX-10HM 错误码为=" + std::to_string(rx[4]));
    }

    return static_cast<uint16_t>(rx[5]) | (static_cast<uint16_t>(rx[6]) << 8U);
}

/**
 * @brief 读取六个舵机的原始位置
 * @return 按舵机编号顺序排列的原始位置数组
 */
std::array<uint16_t, 6> Hx10hm::read_all_pos_raw() {
    std::array<uint16_t, 6> result{};
    for(uint8_t i = 1; i <= 6; ++i) {
        result[i] = read_pos_raw(i);
    }
    return result;
}

/**
 * @brief 将原始位置转换为角度
 * @param raw 原始位置计数值
 * @return 相对于中心位置的角度
 */
double Hx10hm::raw_to_degree(uint16_t raw) {
    if(raw > 4095U) {
        throw std::out_of_range("HX-10HM 原始数据范围必须为 [0, 4096)");
    }

    constexpr double kDegreePerCount = 360.0 / 4096.0;
    return (static_cast<int>(raw) - 2048) * kDegreePerCount;
}

// ! ========================= 私 有 类 方 法 实 现 ========================= ! //

/**
 * @brief 计算 HX 10HM 数据包校验和
 * @param packet 不含校验和字段的数据包
 * @return 计算得到的校验和
 */
uint8_t Hx10hm::check_sum(const std::vector<uint8_t>& packet) {
    if(packet.size() < 5) {
        throw std::invalid_argument("HX-10HM 数据包太短");
    }

    uint8_t sum = 0;
    for(std::size_t i = 2; i < packet.size(); ++i) {
        sum = static_cast<uint8_t>(sum + packet[i]);
    }
    return static_cast<uint8_t>(~sum);
}

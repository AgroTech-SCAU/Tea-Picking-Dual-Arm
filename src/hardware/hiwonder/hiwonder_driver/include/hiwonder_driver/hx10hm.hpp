#pragma once

#include <array>

#include "serial_port/serial_port.hpp"

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //



// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

class Hx10hm {
public:
    static constexpr uint32_t BAUDRATE = 1000000;
    static constexpr uint8_t READ_DATA_ADDR = 0x02;
    static constexpr uint8_t PRESENT_POS_ADDR = 0x38;

    explicit Hx10hm(SerialPort& serial) : serial_(serial) {}

    uint16_t read_pos_raw(uint8_t id);
    std::array<uint16_t, 6> read_all_pos_raw();

    static double raw_to_degree(uint16_t raw);

private:
    static uint8_t check_sum(const std::vector<uint8_t>& packet);

private:
    SerialPort& serial_;
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //



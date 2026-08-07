#pragma once

#include <array>

#include "serial_port/serial_port.hpp"

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //



// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

/**
 * @brief HX 10HM 总线舵机驱动
 *
 * 提供单个舵机和全部舵机的位置读取接口
 */
class Hx10hm {
public:
    /**
     * @brief HX 10HM 默认波特率
     */
    static constexpr uint32_t BAUDRATE = 1000000;

    /**
     * @brief 读取数据指令地址
     */
    static constexpr uint8_t READ_DATA_ADDR = 0x02;

    /**
     * @brief 当前位置信息地址
     */
    static constexpr uint8_t PRESENT_POS_ADDR = 0x38;

    /**
     * @brief 构造 HX 10HM 驱动
     * @param serial 用于通信的串口对象
     */
    explicit Hx10hm(SerialPort& serial) : serial_(serial) {}

    /**
     * @brief 读取指定舵机的原始位置
     * @param id 舵机编号
     * @return 舵机原始位置计数值
     */
    uint16_t read_pos_raw(uint8_t id);

    /**
     * @brief 读取六个舵机的原始位置
     * @return 按舵机编号顺序排列的原始位置数组
     */
    std::array<uint16_t, 6> read_all_pos_raw();

    /**
     * @brief 将原始位置转换为角度
     * @param raw 原始位置计数值
     * @return 相对于中心位置的角度
     */
    static double raw_to_degree(uint16_t raw);

private:
    /**
     * @brief 计算 HX 10HM 数据包校验和
     * @param packet 不含校验和字段的数据包
     * @return 计算得到的校验和
     */
    static uint8_t check_sum(const std::vector<uint8_t>& packet);

private:
    /**
     * @brief 用于驱动通信的串口引用
     */
    SerialPort& serial_;
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //


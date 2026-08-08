#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "serial_port/serial_port.hpp"

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //



// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

/**
 * @brief HX-10HM 总线舵机驱动
 *
 * 当前项目约定：原始位置 2048 对应主臂关节 0 deg
 * raw < 2048 为负角，raw > 2048 为正角
 */
class Hx10hm {
public:
    static constexpr std::uint32_t BAUDRATE = 1000000;
    static constexpr std::uint8_t READ_DATA = 0x02;
    static constexpr std::uint8_t WRITE_DATA = 0x03;
    static constexpr std::uint8_t SYNC_WRITE = 0x83;
    static constexpr std::uint8_t BROADCAST_ID = 0xFE;

    static constexpr std::uint8_t TORQUE_ENABLE_ADDR = 0x28;
    static constexpr std::uint8_t GOAL_POSITION_ADDR = 0x2A;
    static constexpr std::uint8_t PRESENT_POS_ADDR = 0x38;

    static constexpr std::uint16_t CENTER_RAW = 2048;
    static constexpr std::size_t JOINT_COUNT = 6;

    explicit Hx10hm(SerialPort& serial) : serial_(serial) {}

    /**
     * @brief 读取指定舵机的原始位置
     * @param id 舵机 ID
     * @return 原始位置计数值
     */
    std::uint16_t read_pos_raw(std::uint8_t id);

    /**
     * @brief 依次读取 ID 1~6 的原始位置
     * @return 下标 0~5 分别对应 J1~J6
     */
    std::array<std::uint16_t, JOINT_COUNT> read_all_pos_raw();

    /**
     * @brief 通过普通 WRITE DATA 控制单个舵机转到目标位置
     * @param id 舵机 ID
     * @param raw 目标位置，当前项目仅允许 [0, 4095]
     * @param speed 速度，单位 steps/s，范围 [0, 3400]
     * @param time 时间字段，位置模式下通常设 0
     *
     * @note 本函数会等待并严格校验舵机返回的 6 字节状态包
     * @note 只有收到 ERROR=0 的 ACK 才认为写命令成功
     */
    void write_pos_raw(std::uint8_t id, std::uint16_t raw, std::uint16_t speed, std::uint16_t time = 0);

    /**
     * @brief 使用 SYNC WRITE 同时控制 ID 1~6 到各自目标位置
     * @param raw J1~J6 目标位置
     * @param speed 六个舵机统一速度，单位 steps/s
     * @param time 时间字段，位置模式下通常设 0
     *
     * @note 协议规定 SYNC WRITE 使用广播 ID，因此不会返回状态包
     * @note 该接口只负责一次同步下发，是否到位由调用方继续读取当前位置确认
     */
    void sync_write_all_pos_raw(const std::array<std::uint16_t, JOINT_COUNT>& raw, std::uint16_t speed, std::uint16_t time = 0);

    /**
     * @brief 设置单个舵机扭矩使能
     * @param id 舵机 ID
     * @param enable true 上力，false 卸力
     *
     * @note 使用非广播 WRITE DATA，并等待 6 字节 ACK
     */
    void set_torque(std::uint8_t id, bool enable);

    /**
     * @brief 对 ID 1~6 逐个设置扭矩使能并确认 ACK
     * @param enable true 上力，false 卸力
     *
     * @warning 任一 ID 写失败会抛异常；异常恢复时调用方应继续逐个尝试 Torque OFF
     */
    void set_all_torque(bool enable);

    /**
     * @brief 将原始位置转换为项目定义的相对角度
     * @param raw 原始位置计数值 [0, 4095]
     * @return 以 2048 为 0 deg 的相对角度
     */
    static double raw_to_degree(std::uint16_t raw);

private:
    /**
     * @brief 计算 HX-10HM 协议数据包校验和
     * @param packet 数据包内容，至少包含 5 字节
     * @return 校验和字节
     */
    static std::uint8_t check_sum(const std::vector<std::uint8_t>& packet);

    /**
     * @brief 向舵机控制表连续写入数据
     * @param id 舵机 ID
     * @param start_addr 起始寄存器地址
     * @param data 连续写入的数据
     */
    void write_data(std::uint8_t id, std::uint8_t start_addr, const std::vector<std::uint8_t>& data);

    /**
     * @brief 读取并验证写命令状态包 FF FF ID 02 ERROR CHECKSUM
     */
    void read_write_status(std::uint8_t expected_id);

private:
    SerialPort& serial_;    ///< 用于底层通信的通用串口对象
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //


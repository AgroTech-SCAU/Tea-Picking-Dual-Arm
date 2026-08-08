#pragma once

#include <array>

#include "serial_port/serial_port.hpp"

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //



// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

/**
 * @brief HX 10HM 总线舵机驱动
 *
 * 项目约定：原始位置 2048 对应主臂关节 0 deg，raw < 2048 为负角，
 * raw > 2048 为正角
 */
class Hx10hm {
public:
    static constexpr std::uint32_t BAUDRATE = 1000000;          ///< HX-10HM 当前工作区使用的通信波特率
    static constexpr std::uint8_t READ_DATA_ADDR = 0x02;        ///< 协议 READ DATA 指令
    static constexpr std::uint8_t WRITE_DATA_ADDR = 0x03;       ///< 协议 WRITE DATA 指令
    static constexpr std::uint8_t TORQUE_ENABLE_ADDR = 0x28;    ///< 协议 WRITE DATA 指令
    static constexpr std::uint8_t GOAL_ACC_ADDR = 0x29;         ///< 加速度寄存器起始地址，位置扩展写命令从此处连续写入 7 字节
    static constexpr std::uint8_t PRESENT_POS_ADDR = 0x38;      ///< 当前位置信息高字节地址
    static constexpr std::uint16_t CENTER_RAW = 2048;           ///< 主臂软件零点，对应 180 deg 机械中位
    static constexpr std::size_t JOINT_COUNT = 6;               ///< 主臂关节数量

    /**
     * @brief 构造 HX-10HM 驱动
     * @param serial 已经打开并配置为 1 Mbps 8N1 的串口对象
     */
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
     * @brief 写入单个舵机位置目标
     * @param id 舵机 ID
     * @param raw 目标原始位置，当前 V1 仅允许 [0, 4095]
     * @param speed 位置模式速度，单位 steps/s，允许 [0, 3400]
     * @param acceleration 加速度参数，单位 100 steps/s^2
     *
     * @note 该接口只负责下发位置命令，不等待舵机到位
     */
    void write_pos_raw(std::uint8_t id, std::uint16_t raw, std::uint16_t speed, std::uint8_t acceleration = 0);

    /**
     * @brief 为 ID 1~6 下发各自的位置目标
     * @param raw J1~J6 原始目标位置
     * @param speed 所有关节使用的统一速度
     * @param acceleration 所有关节使用的统一加速度
     */
    void write_all_pos_raw(const std::array<std::uint16_t, JOINT_COUNT>& raw, std::uint16_t speed, std::uint8_t acceleration = 0);

    /**
     * @brief 设置单个舵机扭矩使能
     * @param id 舵机 ID
     * @param enable true 使能输出扭矩，false 卸力，可手动拖动
     */
    void set_torque(std::uint8_t id, bool enable);

    /**
     * @brief 对 ID 1~6 统一设置扭矩使能
     * @param enable true 使能，false 卸力
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

private:
    SerialPort& serial_;    ///< 用于底层通信的通用串口对象
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //


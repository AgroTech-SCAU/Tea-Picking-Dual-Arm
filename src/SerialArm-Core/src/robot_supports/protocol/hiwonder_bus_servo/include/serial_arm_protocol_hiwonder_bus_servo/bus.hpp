#pragma once

#include "serial_arm/transport/serial_port.hpp"
#include "tl/expected.hpp"

namespace serial_arm::protocol::hiwonder_bus_servo {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

using Buffer = std::vector<std::uint8_t>;

constexpr std::uint8_t BROADCAST_ID = 0xFE;
constexpr std::uint8_t READ_DATA = 0x02;
constexpr std::uint8_t WRITE_DATA = 0x03;
constexpr std::uint8_t SYNC_READ = 0x82;
constexpr std::uint8_t SYNC_WRITE = 0x83;

constexpr std::uint8_t RUN_MODE_ADDR = 0x21;
constexpr std::uint8_t TORQUE_ENABLE_ADDR = 0x28;
constexpr std::uint8_t PWM_COMMAND_ADDR = 0x2C;
constexpr std::uint8_t PRESENT_POSITION_ADDR = 0x38;
constexpr std::uint8_t PRESENT_VELOCITY_ADDR = 0x3A;
constexpr std::uint8_t PRESENT_LOAD_ADDR = 0x3C;
constexpr std::uint8_t FAULT_ADDR = 0x41;
constexpr std::uint8_t PRESENT_CURRENT_ADDR = 0x45;

/**
 * @brief Hiwonder Bus Servo 协议错误类型
 */
enum class Err {
    NOT_OPEN,              ///< 串口未打开
    INVALID_ARGUMENT,     ///< 输入参数不符合协议约束
    WRITE_FAILED,         ///< 底层串口写入失败
    READ_FAILED,          ///< 底层串口读取失败
    TIMEOUT,              ///< 传输或应答超时
    MALFORMED_PACKET,     ///< 报文头或长度非法
    CHECKSUM_MISMATCH,    ///< 报文校验和不匹配
    UNEXPECTED_ID,        ///< 应答舵机 ID 不匹配
    DEVICE_ERROR,         ///< 应答包 ERROR 非零
};

/**
 * @brief HX-10HM 运行模式寄存器取值
 */
enum class RunMode : std::uint8_t {
    Position = 0,             ///< 位置伺服模式
    VelocityClosedLoop = 1,   ///< 恒速闭环模式
    PwmOpenLoop = 2,          ///< PWM 开环模式
};

/**
 * @brief 舵机状态应答包
 */
struct StatusPacket {
    std::uint8_t id{ 0 };          ///< 应答舵机 ID
    std::uint8_t error{ 0 };       ///< 协议 ERROR 字段
    Buffer parameters;            ///< 原始参数字节
};

/**
 * @brief SYNC WRITE 单舵机数据项
 */
struct SyncWriteEntry {
    std::uint8_t id{ 0 };      ///< 舵机 ID
    Buffer data;               ///< 该舵机的连续寄存器数据
};

/**
 * @brief 单舵机 PWM 原始命令
 */
struct PwmCommand {
    std::uint8_t id{ 0 };      ///< 舵机 ID
    std::int16_t pwm{ 0 };     ///< 有符号 PWM 命令，范围 [-1000, 1000]
};

/**
 * @brief HX-10HM 原始反馈状态
 */
struct RawState {
    std::uint8_t id{ 0 };                  ///< 舵机 ID
    std::int16_t position_raw{ 0 };        ///< 0x38 有符号绝对位置步数
    std::uint16_t velocity_raw{ 0 };       ///< 0x3A 原始速度字，方向编码未解释
    std::int16_t load_raw{ 0 };            ///< 0x3C 原始负载，0.1% 且 BIT10 为方向位
    std::uint8_t voltage_raw{ 0 };         ///< 0x3E 原始电压，0.1 V
    std::uint8_t temperature_raw{ 0 };     ///< 0x3F 原始温度，摄氏度
    std::uint8_t registered{ 0 };          ///< 0x40 异步写标志
    std::uint8_t fault{ 0 };               ///< 0x41 舵机故障位图
    std::uint16_t current_raw_ma{ 0 };     ///< 0x45 原始电流，1 mA
};

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

/**
 * @brief 计算 HX 报文校验和
 * @param packet_without_checksum 从 ID 字段开始参与校验的完整报文
 * @return 按字节求和后取反的校验和
 */
std::uint8_t checksum(const Buffer& packet_without_checksum) noexcept;

/**
 * @brief 构造 READ DATA 指令包
 * @param id 舵机 ID
 * @param address 起始寄存器地址
 * @param length 读取字节数
 * @return 成功时返回报文，否则返回错误码
 */
tl::expected<Buffer, Err> encode_read_packet(
    std::uint8_t id,
    std::uint8_t address,
    std::uint8_t length);

/**
 * @brief 构造 WRITE DATA 指令包
 * @param id 舵机 ID
 * @param address 起始寄存器地址
 * @param data 连续写入数据
 * @return 成功时返回报文，否则返回错误码
 */
tl::expected<Buffer, Err> encode_write_packet(
    std::uint8_t id,
    std::uint8_t address,
    const Buffer& data);

/**
 * @brief 构造 SYNC READ 指令包
 * @param ids 应答顺序中的舵机 ID
 * @param address 起始寄存器地址
 * @param length 每个舵机读取字节数
 * @return 成功时返回报文，否则返回错误码
 */
tl::expected<Buffer, Err> encode_sync_read_packet(
    const std::vector<std::uint8_t>& ids,
    std::uint8_t address,
    std::uint8_t length);

/**
 * @brief 构造 SYNC WRITE 指令包
 * @param address 起始寄存器地址
 * @param data_length 每个舵机写入字节数
 * @param entries 舵机 ID 与数据列表
 * @return 成功时返回报文，否则返回错误码
 */
tl::expected<Buffer, Err> encode_sync_write_packet(
    std::uint8_t address,
    std::uint8_t data_length,
    const std::vector<SyncWriteEntry>& entries);

/**
 * @brief 解析并校验状态应答包
 * @param packet 完整状态报文
 * @return 成功时返回状态包，否则返回错误码
 */
tl::expected<StatusPacket, Err> parse_status_packet(const Buffer& packet);

/**
 * @brief 解码 HX-10HM 0x38 起始的 10 字节实时状态块
 * @param state_packet 从 0x38 读取 10 字节的状态包
 * @param current_raw_ma 最近一次电流诊断值，单位 mA
 * @return 成功时返回原始状态，否则返回错误码
 */
tl::expected<RawState, Err> decode_state_block(
    const StatusPacket& state_packet,
    std::uint16_t current_raw_ma = 0U);

/**
 * @brief 解码 HX-10HM 原始状态块与独立电流包
 * @param state_packet 从 0x38 读取 10 字节的状态包
 * @param current_packet 从 0x45 读取 2 字节的电流包
 * @return 成功时返回原始状态，否则返回错误码
 */
tl::expected<RawState, Err> decode_raw_state(
    const StatusPacket& state_packet,
    const StatusPacket& current_packet);

/**
 * @brief Hiwonder Bus Servo 协议层
 */
class HiwonderBusServo final {
public:
    /**
     * @brief 绑定 SerialArm-Core 串口传输对象
     * @param serial 已由 Hardware Backend 管理的串口对象
     */
    explicit HiwonderBusServo(transport::SerialPort& serial) noexcept;

    /**
     * @brief 读取单舵机连续寄存器
     * @param id 舵机 ID
     * @param address 起始地址
     * @param length 读取长度
     * @param timeout 应答超时
     * @return 成功时返回原始寄存器数据，否则返回错误码
     */
    tl::expected<Buffer, Err> read_register(
        std::uint8_t id,
        std::uint8_t address,
        std::uint8_t length,
        std::chrono::milliseconds timeout);

    /**
     * @brief 写入单舵机连续寄存器并消费状态 ACK
     * @param id 舵机 ID
     * @param address 起始地址
     * @param data 写入数据
     * @param timeout ACK 超时
     * @return 成功时返回空结果，否则返回错误码
     */
    tl::expected<void, Err> write_register(
        std::uint8_t id,
        std::uint8_t address,
        const Buffer& data,
        std::chrono::milliseconds timeout);

    /**
     * @brief 同步读取多舵机连续寄存器
     * @param ids 舵机 ID 与应答顺序
     * @param address 起始地址
     * @param length 每个舵机读取长度
     * @param timeout 每个应答包超时
     * @return 成功时返回按 ids 排列的状态包，否则返回错误码
     */
    tl::expected<std::vector<StatusPacket>, Err> sync_read(
        const std::vector<std::uint8_t>& ids,
        std::uint8_t address,
        std::uint8_t length,
        std::chrono::milliseconds timeout);

    /**
     * @brief 同步写入多舵机连续寄存器
     * @param address 起始地址
     * @param data_length 每个舵机写入长度
     * @param entries 舵机 ID 与写入数据
     * @return 成功时返回空结果，否则返回错误码
     */
    tl::expected<void, Err> sync_write(
        std::uint8_t address,
        std::uint8_t data_length,
        const std::vector<SyncWriteEntry>& entries);

    /**
     * @brief 设置舵机运行模式
     * @param id 舵机 ID
     * @param mode 运行模式
     * @param timeout ACK 超时
     * @return 成功时返回空结果，否则返回错误码
     */
    tl::expected<void, Err> set_run_mode(
        std::uint8_t id,
        RunMode mode,
        std::chrono::milliseconds timeout);

    /**
     * @brief 设置舵机扭矩使能
     * @param id 舵机 ID
     * @param enable true 为上力，false 为卸力
     * @param timeout ACK 超时
     * @return 成功时返回空结果，否则返回错误码
     */
    tl::expected<void, Err> set_torque_enable(
        std::uint8_t id,
        bool enable,
        std::chrono::milliseconds timeout);

    /**
     * @brief 写入单舵机 PWM 开环命令
     * @param id 舵机 ID
     * @param pwm 有符号 PWM 命令，范围 [-1000, 1000]
     * @param timeout ACK 超时
     * @return 成功时返回空结果，否则返回错误码
     */
    tl::expected<void, Err> write_pwm(
        std::uint8_t id,
        std::int16_t pwm,
        std::chrono::milliseconds timeout);

    /**
     * @brief 使用一帧 SYNC WRITE 写入多舵机 PWM
     * @param commands 舵机 ID 与有符号 PWM 命令
     * @return 成功时返回空结果，否则返回错误码
     */
    tl::expected<void, Err> sync_write_pwm(const std::vector<PwmCommand>& commands);

    /**
     * @brief 读取原始位置
     * @param id 舵机 ID
     * @param timeout 应答超时
     * @return 0x38 的有符号绝对位置步数
     */
    tl::expected<std::int16_t, Err> read_position(
        std::uint8_t id,
        std::chrono::milliseconds timeout);

    /**
     * @brief 读取原始速度字
     * @param id 舵机 ID
     * @param timeout 应答超时
     * @return 0x3A 的原始 16 位值
     */
    tl::expected<std::uint16_t, Err> read_velocity(
        std::uint8_t id,
        std::chrono::milliseconds timeout);

    /**
     * @brief 读取有符号原始负载
     * @param id 舵机 ID
     * @param timeout 应答超时
     * @return 0x3C 按 BIT10 方向位解码的负载值
     */
    tl::expected<std::int16_t, Err> read_load(
        std::uint8_t id,
        std::chrono::milliseconds timeout);

    /**
     * @brief 读取舵机故障位图
     * @param id 舵机 ID
     * @param timeout 应答超时
     * @return 0x41 的原始故障字节
     */
    tl::expected<std::uint8_t, Err> read_fault(
        std::uint8_t id,
        std::chrono::milliseconds timeout);

    /**
     * @brief 读取舵机原始电流
     * @param id 舵机 ID
     * @param timeout 应答超时
     * @return 0x45 的原始电流，单位 1 mA
     */
    tl::expected<std::uint16_t, Err> read_current(
        std::uint8_t id,
        std::chrono::milliseconds timeout);

    /**
     * @brief 使用一次 SYNC READ 读取控制循环所需的多舵机状态块
     * @param ids 舵机 ID 顺序
     * @param timeout 每个应答包超时
     * @return 成功时返回按 ids 排列的原始状态，电流字段保持 0
     */
    tl::expected<std::vector<RawState>, Err> sync_read_state_blocks(
        const std::vector<std::uint8_t>& ids,
        std::chrono::milliseconds timeout);

    /**
     * @brief 使用一次 SYNC READ 读取多舵机电流诊断值
     * @param ids 舵机 ID 顺序
     * @param timeout 每个应答包超时
     * @return 成功时返回按 ids 排列的电流值，单位 mA
     */
    tl::expected<std::vector<std::uint16_t>, Err> sync_read_currents(
        const std::vector<std::uint8_t>& ids,
        std::chrono::milliseconds timeout);

    /**
     * @brief 兼容接口：连续执行状态块与电流两次 SYNC READ
     * @param ids 舵机 ID 顺序
     * @param timeout 每个应答包超时
     * @return 成功时返回按 ids 排列的原始状态
     */
    tl::expected<std::vector<RawState>, Err> sync_read_states(
        const std::vector<std::uint8_t>& ids,
        std::chrono::milliseconds timeout);

private:
    /**
     * @brief 清理输入缓冲区并设置读取超时
     * @param timeout 读取超时
     * @return 成功时返回空结果，否则返回错误码
     */
    tl::expected<void, Err> prepare_transaction(std::chrono::milliseconds timeout);

    /**
     * @brief 发送完整协议报文
     * @param packet 待发送报文
     * @return 成功时返回空结果，否则返回错误码
     */
    tl::expected<void, Err> transmit(const Buffer& packet);

    /**
     * @brief 从串口接收并解析一个状态包
     * @return 成功时返回状态包，否则返回错误码
     */
    tl::expected<StatusPacket, Err> receive_status_packet();

private:
    transport::SerialPort& serial_;   ///< SerialArm-Core 底层串口
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //

} // namespace serial_arm::protocol::hiwonder_bus_servo

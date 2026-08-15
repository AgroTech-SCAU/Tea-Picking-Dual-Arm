#pragma once

#include "serial_arm/transport/bus.hpp"

#include <memory>
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <iostream> // IWYU pragma: keep
#include <unistd.h>

#define POS_MODE 0x100
#define SPEED_MODE 0x200
#define POSI_MODE 0x300

#define POS_CSP_MODE 0x400
#define SPEED_CSP_MODE 0x500
#define TOR_CSP_MODE 0x600

namespace damiao {
#pragma pack(1)
using MotorId = uint32_t;

constexpr uint8_t PARAM_READ_CMD = 51;
constexpr uint8_t PARAM_WRITE_CMD = 85;
constexpr uint8_t PARAM_SAVE_CMD = 170;
constexpr auto DEFAULT_FEEDBACK_TIMEOUT = std::chrono::milliseconds(20);
constexpr auto DEFAULT_PARAMETER_TIMEOUT = std::chrono::milliseconds(250);
constexpr auto RECEIVE_SLICE = std::chrono::milliseconds(2);

/**
 * @brief Motor Type 电机类型
 */
enum DmMotorType {
    DM4310,
    DM4310_48V,
    DM4340,
    DM4340_48V,
    DM6006,
    DM6248P,
    DM8006,
    DM8009,
    DM10010L,
    DM10010,
    DMH3510,
    DMH6215,
    DMG6220,
    DMJH11,
    Num_Of_Motor
};

/**
 * @brief 电机控制模式
 * @note 这是改控制模式对应的编码
 */
enum DmControlMode {
    MIT_MODE = 1,
    POS_VEL_MODE = 2,
    VEL_MODE = 3,
    POS_FORCE_MODE = 4,

    POS_VEL_CSP_MODE = 5,
    VEL_CSP_MODE = 6,
    TORQUE_CSP_MODE = 7,
};

/**
 * @brief 寄存器列表 具体参考达妙手册
 */
enum DmReg {
    UV_Value = 0,
    KT_Value = 1,
    OT_Value = 2,
    OC_Value = 3,
    ACC = 4,
    DEC = 5,
    MAX_SPD = 6,
    MST_ID = 7,
    ESC_ID = 8,
    TIMEOUT = 9,
    CTRL_MODE = 10,
    Damp = 11,
    Inertia = 12,
    hw_ver = 13,
    sw_ver = 14,
    SN = 15,
    NPP = 16,
    Rs = 17,
    LS = 18,
    Flux = 19,
    Gr = 20,
    PMAX = 21,
    VMAX = 22,
    TMAX = 23,
    I_BW = 24,
    KP_ASR = 25,
    KI_ASR = 26,
    KP_APR = 27,
    KI_APR = 28,
    OV_Value = 29,
    GREF = 30,
    Deta = 31,
    V_BW = 32,
    IQ_c1 = 33,
    VL_c1 = 34,
    can_br = 35,
    sub_ver = 36,
    u_off = 50,
    v_off = 51,
    k1 = 52,
    k2 = 53,
    m_off = 54,
    dir = 55,
    p_m = 80,
    xout = 81,
};

#pragma pack()

typedef struct {
    float q_max;
    float dq_max;
    float tau_max;
} LimitParam;

// 电机 PMAX/DQMAX/TAUMAX 参数
static LimitParam limit_param[Num_Of_Motor] =
{
    {12.5, 30, 10 },    // DM4310
    {12.5, 50, 10 },    // DM4310_48V
    {12.5, 8, 28 },     // DM4340
    {12.5, 10, 28 },    // DM4340_48V
    {12.5, 45, 20 },    // DM6006
    {12.566, 20, 120 }, // DM6248P
    {12.5, 45, 40 },    // DM8006
    {12.5, 45, 54 },    // DM8009
    {12.5,25,  200},    // DM10010L
    {12.5,20, 200},     // DM10010
    {12.5,28,1},        // DMH3510
    {12.5,45,10},       // DMH6215
    {12.5,45,10},      // DMG6220
    {12.5,10,12}        // DMJH11
};

class Motor {
private:
    MotorId master_id;
    MotorId slave_id;
    float state_q = 0;
    float state_dq = 0;
    float state_tau = 0;
    uint64_t state_seq = 0;
    LimitParam limit_param{};
    DmMotorType motor_type;

    union ValueUnion {
        float float_value;
        uint32_t uint32_value;
    };

    struct ValueType {
        ValueUnion value;
        bool is_float;
    };

    std::unordered_map<uint32_t, ValueType> param_map;

public:
    /**
     * @brief Construct a new Motor object
     *
     * @param motor_type 电机类型
     * @param slave_id can_id 从机ID即电机ID
     * @param master_id 主机ID建议主机ID不要都设为0x00
     *
     */
    Motor(DmMotorType motor_type, MotorId slave_id, MotorId master_id)
        : master_id(master_id), slave_id(slave_id), motor_type(motor_type) {
        this->limit_param = damiao::limit_param[motor_type];
    }

    Motor() : master_id(0x01), slave_id(0x11), motor_type(DM4310) {
        this->limit_param = damiao::limit_param[DM4310];
    }

    void receive_data(float q, float dq, float tau) {
        this->state_q = q;
        this->state_dq = dq;
        this->state_tau = tau;
        ++this->state_seq;
    }

    DmMotorType get_motor_type() const { return this->motor_type; }

    /**
     * @brief get master id 获取主机ID
     * @return MasterID
     */
    MotorId get_master_id() const { return this->master_id; }

    /**
     * @brief get motor slave id(can id)  获取电机CAN ID
     * @return SlaveID
     */
    MotorId get_slave_id() const { return this->slave_id; }

    /**
     * @brief get motor position 获取电机位置
     * @return motor position 电机位置
     */
    float get_position() const { return this->state_q; }

    /**
     * @brief get motor velocity 获取电机速度
     * @return motor velocity 电机速度
     */
    float get_velocity() const { return this->state_dq; }

    /**
     * @brief get torque of the motor  获取电机实际输出扭矩
     * @return motor torque 电机实际输出扭矩
     */
    float get_tau() const { return this->state_tau; }

    uint64_t get_state_seq() const { return this->state_seq; }

    /**
     * @brief get limit param 获取电机限制参数
     * @return limit_param 电机限制参数
     */
    LimitParam get_limit_param() { return limit_param; }

    void set_param(int key, float value) {
        ValueType v{};
        v.value.float_value = value;
        v.is_float = true;
        param_map[key] = v;
    }

    void set_param(int key, uint32_t value) {
        ValueType v{};
        v.value.uint32_value = value;
        v.is_float = false;
        param_map[key] = v;
    }

    float get_param_as_float(int key) const {
        auto it = param_map.find(key);
        if(it != param_map.end()) {
            if(it->second.is_float) {
                return it->second.value.float_value;
            }
            else {
                return 0;
            }
        }
        return 0;
    }

    uint32_t get_param_as_uint32(int key) const {
        auto it = param_map.find(key);
        if(it != param_map.end()) {
            if(!it->second.is_float) {
                return it->second.value.uint32_value;
            }
            else {
                return 0;
            }
        }
        return 0;
    }

    bool has_param(int key) const {
        return param_map.find(key) != param_map.end();
    }

    bool is_have_param(int key) const {
        return has_param(key);
    }

    void clear_param(int key) {
        param_map.erase(key);
    }

    void clear_all_params() {
        param_map.clear();
    }
};


/**
 * @brief motor control class 电机控制类
 * 使用USB转CAN进行通信，linux做虚拟串口
 */
class MotorControl {
public:
    MotorControl(std::shared_ptr<serial_arm::transport::CanChannel> channel) : channel_(std::move(channel)) {
        if(channel_ == nullptr) throw std::invalid_argument("CAN channel is null");
    }

    ~MotorControl() = default;

    MotorId last_parameter_reply_can_id() const noexcept {
        return last_parameter_reply_can_id_;
    }

    bool enable(const Motor& motor) {
        const bool sent = control_cmd(motor.get_slave_id(), 0xFC);
        usleep(100000);
        this->receive();
        return sent;
    }

    void enable_old(const Motor& motor, DmControlMode mode) {
        const uint32_t id = ((mode - 1) << 2) + motor.get_slave_id();
        control_cmd(id, 0xFC);
        usleep(100000);
        this->receive();
    }

    bool refresh_motor_status(
        const Motor& motor,
        std::chrono::milliseconds timeout = DEFAULT_FEEDBACK_TIMEOUT) {
        const uint32_t id = 0x7FF;
        const uint8_t can_low = motor.get_slave_id() & 0xff;
        const uint8_t can_high = (motor.get_slave_id() >> 8) & 0xff;
        const std::array<uint8_t, 8> data_buf = { can_low, can_high, 0xCC, 0x00, 0x00, 0x00, 0x00, 0x00 };
        if(!send_frame(id, data_buf)) return false;
        return receive_feedback_for(motor, timeout);
    }

    bool disable(const Motor& motor) {
        const bool sent = control_cmd(motor.get_slave_id(), 0xFD);
        usleep(100000);
        this->receive();
        return sent;
    }

    void set_zero_position(const Motor& motor) {
        control_cmd(motor.get_slave_id(), 0xFE);
        usleep(100000);
        this->receive();
    }

    bool control_mit(Motor& motor, float kp, float kd, float q, float dq, float tau, bool receive_feedback = true) {
        static auto float_to_uint = [](float x, float xmin, float xmax, uint8_t bits) -> uint16_t {
            x = std::clamp(x, xmin, xmax);
            const float span = xmax - xmin;
            const float data_norm = (x - xmin) / span;
            return static_cast<uint16_t>(data_norm * ((1u << bits) - 1));
        };
        const MotorId id = motor.get_slave_id();
        if(motors_by_slave_id_.find(id) == motors_by_slave_id_.end()) {
            throw std::runtime_error("MotorControl id not found");
        }
        auto& m = motors_by_slave_id_.at(id);
        const uint16_t kp_uint = float_to_uint(kp, 0, 500, 12);
        const uint16_t kd_uint = float_to_uint(kd, 0, 5, 12);
        const LimitParam limit_param_cmd = m->get_limit_param();
        const uint16_t q_uint = float_to_uint(q, -limit_param_cmd.q_max, limit_param_cmd.q_max, 16);
        const uint16_t dq_uint = float_to_uint(dq, -limit_param_cmd.dq_max, limit_param_cmd.dq_max, 12);
        const uint16_t tau_uint = float_to_uint(tau, -limit_param_cmd.tau_max, limit_param_cmd.tau_max, 12);

        std::array<uint8_t, 8> data_buf{};
        data_buf[0] = (q_uint >> 8) & 0xff;
        data_buf[1] = q_uint & 0xff;
        data_buf[2] = dq_uint >> 4;
        data_buf[3] = ((dq_uint & 0xf) << 4) | ((kp_uint >> 8) & 0xf);
        data_buf[4] = kp_uint & 0xff;
        data_buf[5] = kd_uint >> 4;
        data_buf[6] = ((kd_uint & 0xf) << 4) | ((tau_uint >> 8) & 0xf);
        data_buf[7] = tau_uint & 0xff;

        const bool sent = send_frame(id, data_buf);
        if(receive_feedback) this->receive();
        return sent;
    }

    void control_pos_vel(Motor& motor, float pos, float vel) {
        MotorId id = motor.get_slave_id();
        if(motors_by_slave_id_.find(id) == motors_by_slave_id_.end()) {
            throw std::runtime_error("POS_VEL ERROR : MotorControl id not found");
        }
        std::array<uint8_t, 8> data_buf{};
        memcpy(data_buf.data(), &pos, sizeof(float));
        memcpy(data_buf.data() + 4, &vel, sizeof(float));
        id += POS_MODE;
        (void)send_frame(id, data_buf);
        this->receive();
    }

    void control_vel(Motor& motor, float vel) {
        MotorId id = motor.get_slave_id();
        if(motors_by_slave_id_.find(id) == motors_by_slave_id_.end()) {
            throw std::runtime_error("VEL ERROR : id not found");
        }
        std::array<uint8_t, 8> data_buf = { 0 };
        memcpy(data_buf.data(), &vel, sizeof(float));
        id += SPEED_MODE;
        (void)send_frame(id, data_buf);
        this->receive();
    }

    void control_pos_force(Motor& motor, float pos, uint16_t vel, uint16_t i) {
        MotorId id = motor.get_slave_id();
        if(motors_by_slave_id_.find(id) == motors_by_slave_id_.end()) {
            throw std::runtime_error("pos_force ERROR : MotorControl id not found");
        }
        std::array<uint8_t, 8> data_buf{};
        memcpy(data_buf.data(), &pos, sizeof(float));
        memcpy(data_buf.data() + 4, &vel, sizeof(uint16_t));
        memcpy(data_buf.data() + 6, &i, sizeof(uint16_t));
        id += POSI_MODE;
        (void)send_frame(id, data_buf);
        this->receive();
    }

    void control_pos_vel_csp(Motor& motor, float pos, float vel) {
        MotorId id = motor.get_slave_id();
        if(motors_by_slave_id_.find(id) == motors_by_slave_id_.end()) {
            throw std::runtime_error("POS_VEL_CSP ERROR : MotorControl id not found");
        }
        std::array<uint8_t, 8> data_buf{};
        memcpy(data_buf.data(), &pos, sizeof(float));
        memcpy(data_buf.data() + 4, &vel, sizeof(float));
        id += POS_CSP_MODE;
        (void)send_frame(id, data_buf);
        this->receive();
    }

    void control_vel_csp(Motor& motor, float vel) {
        MotorId id = motor.get_slave_id();
        if(motors_by_slave_id_.find(id) == motors_by_slave_id_.end()) {
            throw std::runtime_error("VEL ERROR : id not found");
        }
        std::array<uint8_t, 8> data_buf = { 0 };
        memcpy(data_buf.data(), &vel, sizeof(float));
        id += SPEED_CSP_MODE;
        (void)send_frame(id, data_buf);
        this->receive();
    }

    void control_tor_csp(Motor& motor, float tor) {
        MotorId id = motor.get_slave_id();
        if(motors_by_slave_id_.find(id) == motors_by_slave_id_.end()) {
            throw std::runtime_error("VEL ERROR : id not found");
        }
        std::array<uint8_t, 8> data_buf = { 0 };
        memcpy(data_buf.data(), &tor, sizeof(float));
        id += TOR_CSP_MODE;
        (void)send_frame(id, data_buf);
        this->receive();
    }

    bool receive() {
        auto maybe_frame = channel_->receive(RECEIVE_SLICE);
        if(!maybe_frame) return false;
        ParameterResponse parameter_response;
        if(parse_parameter_frame(*maybe_frame, parameter_response)) return false;
        Motor* motor = nullptr;
        return decode_feedback_frame(*maybe_frame, motor);
    }

    bool receive_feedback_for(
        const Motor& motor,
        std::chrono::milliseconds timeout = DEFAULT_FEEDBACK_TIMEOUT) {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while(std::chrono::steady_clock::now() < deadline) {
            auto maybe_frame = channel_->receive(remaining_timeout(deadline, RECEIVE_SLICE));
            if(!maybe_frame) {
                if(maybe_frame.error() == serial_arm::transport::CanErr::TIMEOUT) continue;
                return false;
            }
            Motor* decoded_motor = nullptr;
            if(!decode_feedback_frame(*maybe_frame, decoded_motor)) continue;
            if(decoded_motor != nullptr && decoded_motor->get_slave_id() == motor.get_slave_id()) return true;
        }
        return false;
    }

    void receive_param() {
        const auto deadline = std::chrono::steady_clock::now() + RECEIVE_SLICE;
        while(std::chrono::steady_clock::now() < deadline) {
            auto maybe_frame = channel_->receive(remaining_timeout(deadline, RECEIVE_SLICE));
            if(!maybe_frame) return;
            ParameterResponse response;
            if(!parse_parameter_frame(*maybe_frame, response)) continue;
            apply_parameter_response(response);
            last_parameter_reply_can_id_ = maybe_frame->id;
            return;
        }
    }

    bool receive_param_for(
        Motor& motor,
        uint8_t reg_id,
        std::chrono::milliseconds timeout = DEFAULT_PARAMETER_TIMEOUT) {
        return receive_param_for_response(motor, reg_id, 0, timeout);
    }

    void add_motor(Motor* motor) {
        if(motor == nullptr) throw std::invalid_argument("Motor is null");
        const auto slave_id = motor->get_slave_id();
        const auto master_id = motor->get_master_id();

        const auto slave_it = motors_by_slave_id_.find(slave_id);
        if(slave_it != motors_by_slave_id_.end() && slave_it->second != motor) {
            throw std::invalid_argument("Damiao slave ID is not unique");
        }
        const auto slave_master_collision = motors_by_master_id_.find(slave_id);
        if(slave_master_collision != motors_by_master_id_.end() && slave_master_collision->second != motor) {
            throw std::invalid_argument("Damiao slave ID collides with another non-zero master ID");
        }

        if(master_id != 0) {
            const auto master_it = motors_by_master_id_.find(master_id);
            if(master_it != motors_by_master_id_.end() && master_it->second != motor) {
                throw std::invalid_argument("Damiao non-zero master ID is not unique");
            }
            const auto master_slave_collision = motors_by_slave_id_.find(master_id);
            if(master_slave_collision != motors_by_slave_id_.end() && master_slave_collision->second != motor) {
                throw std::invalid_argument("Damiao non-zero master ID collides with another slave ID");
            }
        }

        motors_by_slave_id_[slave_id] = motor;
        if(master_id != 0) motors_by_master_id_[master_id] = motor;
    }

    float read_motor_param(
        Motor& motor,
        uint8_t reg_id,
        std::chrono::milliseconds timeout = DEFAULT_PARAMETER_TIMEOUT) {
        auto target = motors_by_slave_id_.find(motor.get_slave_id());
        if(target == motors_by_slave_id_.end() || target->second != &motor) return 0;

        channel_->flush();
        motor.clear_param(reg_id);
        const uint32_t id = motor.get_slave_id();
        const uint8_t can_low = id & 0xff;
        const uint8_t can_high = (id >> 8) & 0xff;
        const std::array<uint8_t, 8> data_buf{ can_low, can_high, PARAM_READ_CMD, reg_id, 0x00, 0x00, 0x00, 0x00 };
        if(!send_frame(0x7FF, data_buf)) return 0;
        if(!receive_param_for_response(motor, reg_id, PARAM_READ_CMD, timeout)) return 0;
        if(is_in_ranges(reg_id)) return float(motor.get_param_as_uint32(reg_id));
        return motor.get_param_as_float(reg_id);
    }

    bool switch_control_mode(
        Motor& motor,
        DmControlMode mode,
        std::chrono::milliseconds timeout = DEFAULT_PARAMETER_TIMEOUT) {
        constexpr uint8_t reg_id = CTRL_MODE;
        auto target = motors_by_slave_id_.find(motor.get_slave_id());
        if(target == motors_by_slave_id_.end() || target->second != &motor) return false;

        channel_->flush();
        motor.clear_param(reg_id);
        uint8_t write_data[4] = { static_cast<uint8_t>(mode), 0x00, 0x00, 0x00 };
        if(!write_motor_param(motor, reg_id, write_data)) return false;
        if(!receive_param_for_response(motor, reg_id, PARAM_WRITE_CMD, timeout)) return false;
        return motor.get_param_as_uint32(reg_id) == static_cast<uint32_t>(mode);
    }

    bool change_motor_param(
        Motor& motor,
        uint8_t reg_id,
        float data,
        std::chrono::milliseconds timeout = DEFAULT_PARAMETER_TIMEOUT) {
        auto target = motors_by_slave_id_.find(motor.get_slave_id());
        if(target == motors_by_slave_id_.end() || target->second != &motor) return false;

        channel_->flush();
        motor.clear_param(reg_id);
        if(is_in_ranges(reg_id)) {
            uint32_t data_uint32 = float_to_uint32(data);
            auto* data_uint8 = reinterpret_cast<uint8_t*>(&data_uint32);
            if(!write_motor_param(motor, reg_id, data_uint8)) return false;
        }
        else {
            auto* data_uint8 = reinterpret_cast<uint8_t*>(&data);
            if(!write_motor_param(motor, reg_id, data_uint8)) return false;
        }

        if(!receive_param_for_response(motor, reg_id, PARAM_WRITE_CMD, timeout)) return false;
        if(is_in_ranges(reg_id)) return motor.get_param_as_uint32(reg_id) == float_to_uint32(data);
        return fabsf(motor.get_param_as_float(reg_id) - data) < 0.1f;
    }

    void save_motor_param(Motor& motor) {
        disable(motor);
        const uint32_t id = motor.get_slave_id();
        const uint8_t id_low = id & 0xff;
        const uint8_t id_high = (id >> 8) & 0xff;
        const std::array<uint8_t, 8> data_buf{ id_low, id_high, PARAM_SAVE_CMD, 0x01, 0x00, 0x00, 0x00, 0x00 };
        (void)send_frame(0x7FF, data_buf);
        usleep(100000);
    }

    static void change_motor_limit(Motor& motor, float p_max, float q_max, float t_max) {
        limit_param[motor.get_motor_type()] = { p_max, q_max, t_max };
    }

private:
    static std::chrono::milliseconds remaining_timeout(
        const std::chrono::steady_clock::time_point& deadline,
        std::chrono::milliseconds max_slice) {
        const auto now = std::chrono::steady_clock::now();
        if(now >= deadline) return std::chrono::milliseconds(0);
        const auto remaining = deadline - now;
        auto remaining_ms = std::chrono::duration_cast<std::chrono::milliseconds>(remaining);
        if(remaining_ms.count() == 0) remaining_ms = std::chrono::milliseconds(1);
        return remaining_ms < max_slice ? remaining_ms : max_slice;
    }

    struct ParameterResponse {
        Motor* motor{ nullptr };
        uint8_t reg_id{ 0 };
        uint8_t response_type{ 0 };
        std::array<uint8_t, 4> value{};
    };

    bool is_parameter_response(const serial_arm::transport::CanFrame& frame) const noexcept {
        if(frame.size < 8) return false;
        if(frame.data[2] != PARAM_READ_CMD && frame.data[2] != PARAM_WRITE_CMD) return false;
        const MotorId slave_id = static_cast<MotorId>(frame.data[0]) |
            (static_cast<MotorId>(frame.data[1]) << 8);
        return motors_by_slave_id_.find(slave_id) != motors_by_slave_id_.end();
    }

    Motor* feedback_motor_for(const serial_arm::transport::CanFrame& frame) const noexcept {
        if(frame.size < 6 || is_parameter_response(frame)) return nullptr;
        if(frame.id == 0) {
            const MotorId slave_id = frame.data[0] & 0x0f;
            auto it = motors_by_slave_id_.find(slave_id);
            return it == motors_by_slave_id_.end() ? nullptr : it->second;
        }
        auto master_it = motors_by_master_id_.find(frame.id);
        if(master_it != motors_by_master_id_.end()) return master_it->second;
        auto slave_it = motors_by_slave_id_.find(frame.id);
        return slave_it == motors_by_slave_id_.end() ? nullptr : slave_it->second;
    }

    bool decode_feedback_frame(const serial_arm::transport::CanFrame& frame, Motor*& decoded_motor) {
        decoded_motor = feedback_motor_for(frame);
        if(decoded_motor == nullptr) return false;
        static auto uint_to_float = [](uint16_t x, float xmin, float xmax, uint8_t bits) -> float {
            const float span = xmax - xmin;
            const float data_norm = float(x) / ((1 << bits) - 1);
            return data_norm * span + xmin;
        };
        const auto& data = frame.data;
        const uint16_t q_uint = (uint16_t(data[1]) << 8) | data[2];
        const uint16_t dq_uint = (uint16_t(data[3]) << 4) | (data[4] >> 4);
        const uint16_t tau_uint = (uint16_t(data[4] & 0xf) << 8) | data[5];
        const LimitParam limit = decoded_motor->get_limit_param();
        const float q = uint_to_float(q_uint, -limit.q_max, limit.q_max, 16);
        const float dq = uint_to_float(dq_uint, -limit.dq_max, limit.dq_max, 12);
        const float tau = uint_to_float(tau_uint, -limit.tau_max, limit.tau_max, 12);
        decoded_motor->receive_data(q, dq, tau);
        return true;
    }

    bool parse_parameter_frame(const serial_arm::transport::CanFrame& frame, ParameterResponse& response) const {
        response = ParameterResponse{};
        if(frame.size < 8) return false;
        if(frame.data[2] != PARAM_READ_CMD && frame.data[2] != PARAM_WRITE_CMD) return false;
        const auto& data = frame.data;
        const MotorId slave_id = static_cast<MotorId>(data[0]) |
            (static_cast<MotorId>(data[1]) << 8);
        auto motor_it = motors_by_slave_id_.find(slave_id);
        if(motor_it == motors_by_slave_id_.end()) return false;
        response.motor = motor_it->second;
        response.reg_id = data[3];
        response.response_type = data[2];
        std::copy(data.begin() + 4, data.begin() + 8, response.value.begin());
        return true;
    }

    static void apply_parameter_response(const ParameterResponse& response) {
        if(response.motor == nullptr) return;
        if(is_in_ranges(response.reg_id)) {
            const uint32_t value = static_cast<uint32_t>(response.value[0]) |
                (static_cast<uint32_t>(response.value[1]) << 8) |
                (static_cast<uint32_t>(response.value[2]) << 16) |
                (static_cast<uint32_t>(response.value[3]) << 24);
            response.motor->set_param(response.reg_id, value);
        }
        else {
            response.motor->set_param(response.reg_id, uint8_to_float(response.value.data()));
        }
    }

    bool receive_param_for_response(
        Motor& motor,
        uint8_t reg_id,
        uint8_t expected_response_type,
        std::chrono::milliseconds timeout) {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while(std::chrono::steady_clock::now() < deadline) {
            auto maybe_frame = channel_->receive(remaining_timeout(deadline, RECEIVE_SLICE));
            if(!maybe_frame) {
                if(maybe_frame.error() == serial_arm::transport::CanErr::TIMEOUT) continue;
                return false;
            }
            ParameterResponse response;
            if(!parse_parameter_frame(*maybe_frame, response)) {
                Motor* feedback_motor = nullptr;
                (void)decode_feedback_frame(*maybe_frame, feedback_motor);
                continue;
            }
            if(response.motor == nullptr || response.motor->get_slave_id() != motor.get_slave_id()) continue;
            if(response.reg_id != reg_id) continue;
            if(expected_response_type != 0 && response.response_type != expected_response_type) continue;
            apply_parameter_response(response);
            last_parameter_reply_can_id_ = maybe_frame->id;
            return true;
        }
        return false;
    }

    bool control_cmd(MotorId id, uint8_t cmd) {
        const std::array<uint8_t, 8> data_buf = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, cmd };
        return send_frame(id, data_buf);
    }

    bool write_motor_param(Motor& motor, uint8_t reg_id, const uint8_t data[4]) {
        const uint32_t id = motor.get_slave_id();
        const uint8_t can_low = id & 0xff;
        const uint8_t can_high = (id >> 8) & 0xff;
        std::array<uint8_t, 8> data_buf{ can_low, can_high, PARAM_WRITE_CMD, reg_id, 0x00, 0x00, 0x00, 0x00 };
        data_buf[4] = data[0];
        data_buf[5] = data[1];
        data_buf[6] = data[2];
        data_buf[7] = data[3];
        return send_frame(0x7FF, data_buf);
    }

    static bool is_in_ranges(int number) {
        return (7 <= number && number <= 10) ||
            (13 <= number && number <= 16) ||
            (35 <= number && number <= 36);
    }

    static uint32_t float_to_uint32(float value) {
        return static_cast<uint32_t>(value);
    }

    static float uint32_to_float(uint32_t value) {
        return static_cast<float>(value);
    }

    static float uint8_to_float(const uint8_t data[4]) {
        const uint32_t combined = (static_cast<uint32_t>(data[3]) << 24) |
            (static_cast<uint32_t>(data[2]) << 16) |
            (static_cast<uint32_t>(data[1]) << 8) |
            static_cast<uint32_t>(data[0]);
        float result;
        memcpy(&result, &combined, sizeof(result));
        return result;
    }

    bool send_frame(MotorId id, const std::array<uint8_t, 8>& data) {
        serial_arm::transport::CanFrame frame;
        frame.id = id;
        frame.size = 8;
        frame.data = data;
        return channel_->send(frame).has_value();
    }

    std::unordered_map<MotorId, Motor*> motors_by_slave_id_;
    std::unordered_map<MotorId, Motor*> motors_by_master_id_;
    std::shared_ptr<serial_arm::transport::CanChannel> channel_;
    MotorId last_parameter_reply_can_id_{ 0 };
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //

};

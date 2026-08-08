#pragma once

#include <array>
#include <cstdint>
#include <string>

#include "rm_bringup/rm65b.hpp"

class Hx10hm;
class SerialPort;

/**
 * @brief 遥操作运行时配置
 *
 * 所有参数都可以在程序运行期间通过菜单修改
 * 修改只对本次运行有效，不写回文件
 */
struct TeleopConfig {
    std::string serial_device{ "/dev/ttyACM0" };    ///< HX-10HM 调试板串口
    std::string rm_ip{ "192.168.1.18" };            ///< RM65-B 控制器 IPv4 地址
    int rm_port{ 8080 };                            ///< RM65-B 控制器 TCP 端口

    /**
     * 主臂到从臂的关节方向映射
     * +1 表示同向，-1 表示反向
     */
    std::array<float, 6> mapping_direction{
        1.0F,
        1.0F,
        -1.0F,
        1.0F,
        1.0F,
        1.0F,
    };

    /**
     * 遥操作持续时间，单位 s
     * -1 表示持续运行直到用户 Ctrl+C 中断
     */
    int teleop_duration_s{ -1 };

    /** 连续读取打印周期 */
    int read_print_period_ms{ 100 };

    /** 遥操作期望循环周期 */
    int teleop_period_ms{ 20 };

    /** 慢速遥操作单周期最大关节变化量 */
    float slow_max_step_degree{ 1.0F };

    /** 不归零启动时允许的最大主从初始角差 */
    float max_start_error_degree{ 30.0F };

    /** 主臂自动归零到位容差 */
    float master_home_tolerance_degree{ 2.0F };

    /** 主臂自动归零速度，单位 steps/s */
    std::uint16_t master_home_speed{ 100 };

    /** 主臂自动归零最长等待时间 */
    int master_home_timeout_s{ 30 };

    /** RM65-B 规划归零/全速模式启动前对齐的速度百分比 */
    int slave_home_speed_percent{ 10 };
};

/**
 * @brief Tea-Picking-Dual-Arm 菜单式主从遥操作程序
 *
 * 该程序不依赖 ROS 通信机制
 * ROS2/ament_cmake 仅用于当前工作区构建组织
 */
class TeaTeleop {
public:
    explicit TeaTeleop(TeleopConfig config = {});

    /**
     * @brief 进入交互式主菜单
     * @return 进程退出码
     */
    int run();

private:
    enum class ReadMode {
        Once,
        Continuous,
    };

    enum class TeleopMode {
        Slow,
        Full,
    };

    /** 主菜单与子菜单 */
    void print_main_menu() const;
    void print_config() const;
    void config_menu();
    void mapping_direction_menu();

    /** 主从臂读取 */
    void read_master_menu();
    void read_slave_menu();
    void read_compare_menu();
    void read_master(ReadMode mode);
    void read_slave(ReadMode mode);
    void read_compare(ReadMode mode);

    void home_master_menu();
    void home_slave_menu();
    void home_both_menu();
    void release_master_menu();
    bool home_master(Hx10hm& master, bool require_confirmation);
    bool home_slave(bool require_confirmation);
    bool release_master_torque(Hx10hm& master) noexcept;

    /** 遥操作 */
    void teleop(TeleopMode mode);

    /** 软件停止 */
    void software_stop();

    /** 通信与映射工具 */
    void ensure_rm_connected();
    [[nodiscard]] std::array<float, 6> master_raw_to_degree(
        const std::array<std::uint16_t, 6>& raw) const;
    [[nodiscard]] std::array<float, 6> limit_slow_command(
        const std::array<float, 6>& target,
        const std::array<float, 6>& previous) const;
    void validate_start_error(
        const std::array<float, 6>& master_degree,
        const std::array<float, 6>& slave_degree) const;

    /** 运行时配置 */
    TeleopConfig config_;

    /** RM65-B 连接在菜单运行期间复用 */
    Rm65bBringup rm_;
};

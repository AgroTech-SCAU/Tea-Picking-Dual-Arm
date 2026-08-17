#pragma once

#include <array>
#include <string>
#include <vector>

#include "rm_bringup/rm65b.hpp"

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

/**
 * @brief 单侧主从通道配置
 *
 * 主臂串口、舵机 ID 和零位仍由 SerialArm Profile 管理，本结构只描述
 * 双臂遥操作需要知道的 Profile、RM65-B 连接参数和关节方向映射
 */
struct DualArmConfig {
    std::string leader_profile;                    ///< SerialArm 主臂 Profile

    std::string follower_ip;                       ///< RM65-B 控制器地址
    int follower_port{ 8080 };                     ///< RM65-B Native SDK TCP 端口
    int follower_home_speed_percent{ 10 };         ///< 从臂归零与启动对齐速度

    std::array<float, 6> mapping_direction{
        1.0F,
        1.0F,
        -1.0F,
        1.0F,
        1.0F,
        1.0F,
    };
};

/**
 * @brief 双臂遥操作运行参数
 *
 * 左右两侧独立保存主从通道配置，遥操作周期、安全阈值和主臂归零参数
 * 在双臂运行中共享
 */
struct DualTeleopConfig {
    DualArmConfig left;
    DualArmConfig right;

    int teleop_duration_s{ -1 };                ///< -1 表示持续到 Ctrl+C
    int teleop_period_ms{ 10 };                 ///< 一轮左右从臂发送周期，默认 10 ms
    float slow_max_step_degree{ 1.0F };         ///< 慢速遥操作单周期最大变化量
    float max_start_error_degree{ 30.0F };      ///< 主从启动最大允许角差

    float leader_home_tolerance_degree{ 2.0F }; ///< 主臂归零容差
    float leader_home_speed_degree_s{ 10.0F };  ///< 主臂归零参考速度
    int leader_home_timeout_s{ 30 };            ///< 主臂归零超时
};

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

/**
 * @brief Tea-Picking-Dual-Arm 双臂主从遥操作程序
 *
 * 菜单进程存活期间保持两个 RM65-B 连接；每个动作临时创建主臂会话，
 * 进入遥操作时由两个 LeaderCycleWorker 分别维持 SerialArm 控制周期
 */
class DualTeaTeleop final {
public:
    explicit DualTeaTeleop(DualTeleopConfig config);
    int run();

private:
    enum class TeleopMode {
        Slow,
        Full,
    };

    void print_main_menu() const;
    void print_config() const;

    void read_all();
    void read_compare();

    bool home_leaders();
    bool home_followers();
    bool home_all();

    void release_leaders();
    void software_stop_all();

    void teleop(TeleopMode mode);

    void ensure_followers_connected();

    [[nodiscard]] std::array<float, 6> leader_joint_to_degree(
        const std::vector<double>& joint_position,
        const DualArmConfig& arm) const;

    [[nodiscard]] std::array<float, 6> limit_slow_command(
        const std::array<float, 6>& target,
        const std::array<float, 6>& previous) const;

    void validate_start_error(
        const char* side,
        const std::array<float, 6>& leader_degree,
        const std::array<float, 6>& follower_degree) const;

    DualTeleopConfig config_;

    Rm65bBringup left_rm_;
    Rm65bBringup right_rm_;
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //



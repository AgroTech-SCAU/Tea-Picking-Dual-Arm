#pragma once

#include <array>
#include <string>
#include <vector>

#include "rm_bringup/rm65b.hpp"

/**
 * @brief 遥操作运行参数
 *
 * 菜单修改仅作用于当前进程
 */
struct TeleopConfig {
    std::string rm_ip{ "192.168.1.18" };                   ///< RM65-B 控制器地址
    int rm_port{ 8080 };                                    ///< RM65-B 控制端口
    std::string leader_profile{ "tea_leader_right" };      ///< SerialArm 主臂 Profile

    std::array<float, 6> mapping_direction{
        1.0F,
        1.0F,
        -1.0F,
        1.0F,
        1.0F,
        1.0F,
    };

    int teleop_duration_s{ -1 };               ///< -1 表示持续到 Ctrl+C
    float slow_max_step_degree{ 1.0F };         ///< 慢速遥操作单周期最大变化量
    float max_start_error_degree{ 30.0F };      ///< 主从启动最大允许角差
    float leader_home_tolerance_degree{ 2.0F }; ///< 主臂归零容差
    float leader_home_speed_degree_s{ 10.0F };  ///< 主臂归零参考速度
    int leader_home_timeout_s{ 30 };            ///< 主臂归零超时
    int slave_home_speed_percent{ 10 };         ///< 从臂归零与启动对齐速度
};

/**
 * @brief Tea-Picking-Dual-Arm 主从遥操作程序
 */
class TeaTeleop {
public:
    explicit TeaTeleop(TeleopConfig config = {});
    int run();

private:
    enum class TeleopMode {
        Slow,
        Full,
    };

    void print_main_menu() const;
    void print_config() const;
    void config_menu();
    void mapping_direction_menu();

    void read_leader();
    void read_slave();
    void read_compare();

    void home_leader_menu();
    void home_slave_menu();
    void home_both_menu();
    void release_leader_menu();
    bool home_leader();
    bool home_slave();

    void teleop(TeleopMode mode);
    void software_stop();

    void ensure_rm_connected();
    [[nodiscard]] std::array<float, 6> leader_joint_to_degree(
        const std::vector<double>& joint_position) const;
    [[nodiscard]] std::array<float, 6> limit_slow_command(
        const std::array<float, 6>& target,
        const std::array<float, 6>& previous) const;
    void validate_start_error(
        const std::array<float, 6>& leader_degree,
        const std::array<float, 6>& slave_degree) const;

    TeleopConfig config_;
    Rm65bBringup rm_;
};

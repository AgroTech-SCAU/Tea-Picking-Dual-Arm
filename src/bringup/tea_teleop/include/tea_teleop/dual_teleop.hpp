#pragma once

#include <array>
#include <string>
#include <vector>

#include "rm_bringup/rm65b.hpp"

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

struct DualArmConfig {
    std::string leader_profile;

    std::string follower_ip;
    int follower_port{ 8080 };
    int follower_home_speed_percent{ 10 };

    std::array<float, 6> mapping_direction{
        1.0F,
        1.0F,
        -1.0F,
        1.0F,
        1.0F,
        1.0F,
    };
};

struct DualTeleopConfig {
    DualArmConfig left;
    DualArmConfig right;

    int teleop_duration_s{ -1 };
    int teleop_period_ms{ 10 };
    float slow_max_step_degree{ 1.0F };
    float max_start_error_degree{ 30.0F };

    float leader_home_tolerance_degree{ 2.0F };
    float leader_home_speed_degree_s{ 10.0F };
    int leader_home_timeout_s{ 30 };
};

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

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



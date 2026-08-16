#pragma once

#include <string>
#include <array>

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
    int teleop_period_ms{ 8 };
    float slow_max_step_degree{ 1.0F };
    float max_start_error_degree{ 30.0F };

    float leader_home_tolerance_degree{ 2.0F };
    float leader_home_speed_degree_s{ 10.0F };
    int leader_home_timeout_s{ 30 };
};

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //



// ! ========================= 模 版 方 法 实 现 ========================= ! //



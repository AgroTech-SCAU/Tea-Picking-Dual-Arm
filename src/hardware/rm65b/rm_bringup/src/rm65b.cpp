#include "rm_bringup/rm65b.hpp"
#include "rm_define.h"

#include <chrono>
#include <stdexcept>
#include <thread>

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //



// ! ========================= 私 有 量 / 工 具 函 数 实 现 ========================= ! //



// ! ========================= 接 口 类 方 法 / 函 数 实 现 ========================= ! //

Rm65bBringup::~Rm65bBringup() noexcept {
    disconnect();
}

void Rm65bBringup::connect(const std::string& ip, int port) {
    if(is_connected()) {
        throw std::logic_error("RM65-B is already connected");
    }

    acquire_sdk();

    try {
        handle_ = api_.rm_create_robot_arm(ip.c_str(), port);

        if(handle_ == nullptr || handle_->id < 0) {
            handle_ = nullptr;
            throw std::runtime_error("rm_create_robot_arm failed: " + ip + ":" + std::to_string(port));
        }

        rm_robot_info_t info{};
        const int info_ret = api_.rm_get_robot_info(handle_, &info);

        if(info_ret != 0) {
            throw std::runtime_error("rm_get_robot_info failed, error code: " + std::to_string(info_ret));
        }

        if(info.arm_dof != 6) {
            throw std::runtime_error("connected robot is not 6-DOF, arm_dof: " + std::to_string(info.arm_dof));
        }
    }
    catch(...) {
        if(handle_ != nullptr) {
            (void)api_.rm_delete_robot_arm(handle_);
            handle_ = nullptr;
        }

        release_sdk();
        throw;
    }
}

void Rm65bBringup::disconnect() noexcept {
    if(handle_ != nullptr) {
        (void)api_.rm_delete_robot_arm(handle_);
        handle_ = nullptr;
    }

    release_sdk();
}

std::array<float, 6> Rm65bBringup::read_all_degree() {
    if(!is_connected()) {
        throw std::logic_error("RM65-B is not connected");
    }

    std::array<float, 6> joint{};
    int ret = 0;

    for(int attempt = 0; attempt < 2; ++attempt) {
        ret = api_.rm_get_joint_degree(handle_, joint.data());
        if(ret == 0) return joint;
        if(attempt == 0) std::this_thread::sleep_for(std::chrono::milliseconds{ 5 });
    }

    throw std::runtime_error(
        "rm_get_joint_degree failed twice, error code: " + std::to_string(ret));
}

void Rm65bBringup::write_all_degree(
    const std::array<float, 6>& degree,
    bool follow,
    int trajectory_mode,
    int radio) {
    if(!is_connected()) {
        throw std::logic_error("RM65-B is not connected");
    }

    auto target = degree;
    int ret = 0;

    for(int attempt = 0; attempt < 2; ++attempt) {
        ret = api_.rm_movej_canfd(
            handle_,
            target.data(),
            follow,
            0,
            trajectory_mode,
            radio);
        if(ret == 0) return;
        if(attempt == 0) std::this_thread::sleep_for(std::chrono::milliseconds{ 5 });
    }

    throw std::runtime_error(
        "rm_movej_canfd failed twice, error code: " + std::to_string(ret));
}

void Rm65bBringup::movej_degree(
    const std::array<float, 6>& degree,
    int speed_percent,
    bool block) {
    if(!is_connected()) {
        throw std::logic_error("RM65-B is not connected");
    }
    if(speed_percent < 1 || speed_percent > 100) {
        throw std::out_of_range("RM65-B movej speed_percent must be in [1, 100]");
    }

    auto target = degree;
    int ret = 0;

    for(int attempt = 0; attempt < 2; ++attempt) {
        ret = api_.rm_movej(
            handle_,
            target.data(),
            speed_percent,
            0,
            0,
            block ? 1 : 0);
        if(ret == 0) return;
        if(attempt == 0) std::this_thread::sleep_for(std::chrono::milliseconds{ 20 });
    }

    throw std::runtime_error(
        "rm_movej failed twice, error code: " + std::to_string(ret));
}

void Rm65bBringup::stop() {
    if(!is_connected()) {
        return;
    }

    int ret = 0;
    for(int attempt = 0; attempt < 2; ++attempt) {
        ret = api_.rm_set_arm_stop(handle_);
        if(ret == 0) return;
        if(attempt == 0) std::this_thread::sleep_for(std::chrono::milliseconds{ 5 });
    }

    throw std::runtime_error(
        "rm_set_arm_stop failed twice, error code: " + std::to_string(ret));
}

// ! ========================= 私 有 类 方 法 实 现 ========================= ! //

void Rm65bBringup::acquire_sdk() {
    std::lock_guard<std::mutex> lock(sdk_mutex_);

    if(sdk_acquired_) return;
    if(sdk_users_ == 0) {
        const int ret = api_.rm_init(RM_TRIPLE_MODE_E);
        if(ret != 0) {
            throw std::runtime_error("rm_init failed, error code: " + std::to_string(ret));
        }
    }

    ++sdk_users_;
    sdk_acquired_ = true;
}

void Rm65bBringup::release_sdk() noexcept {
    std::lock_guard<std::mutex> lock(sdk_mutex_);

    if(!sdk_acquired_) return;

    sdk_acquired_ = false;

    if(sdk_users_ > 0) {
        --sdk_users_;
    }

    if(sdk_users_ == 0) {
        (void)api_.rm_destroy();
    }
}

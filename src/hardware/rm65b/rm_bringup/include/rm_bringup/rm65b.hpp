#pragma once

#include <array>
#include <mutex>
#include <string>

#include "rm_service.h"

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //



// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

/**
 * @brief RM65-B 原生 C++ SDK 封装
 */
class Rm65bBringup {
public:
    /**
     * @brief 默认构造函数
     */
    Rm65bBringup() = default;

    /**
     * @brief 析构函数
     * 自动调用 disconnect() 释放机械臂连接和 SDK 资源
     */
    ~Rm65bBringup() noexcept;

    /**
     * @brief 禁止复制
     */
    Rm65bBringup(const Rm65bBringup&) = delete;
    Rm65bBringup& operator=(const Rm65bBringup&) = delete;

    /**
     * @brief 初始化 SDK 并连接 RM65-B
     * @param ip 控制器 IPv4 地址，例如 192.168.1.18
     * @param port TCP 端口，默认 8080
     */
    void connect(const std::string& ip, int port = 8080);

    /**
     * @brief 断开 RM65-B 并销毁 SDK
     */
    void disconnect() noexcept;

    /**
     * @brief 查询当前是否已经建立有效机械臂连接
     * @return true handle_ 非空 false 当前没有机械臂连接
     */
    [[nodiscard]] bool is_connected() const noexcept {
        return handle_ != nullptr;
    }

    /**
     * @brief 读取 RM65-B 当前 J1~J6 角度
     * @return 角度数组，单位 degree
     */
    [[nodiscard]] std::array<float, 6> read_all_degree();

    /**
     * @brief 通过 CANFD 透传方式发送六关节目标角度
     * @param degree J1~J6 目标角度，单位 degree
     * @param follow false 为低跟随，true 为高跟随
     * @param trajectory_mode 高跟随轨迹处理模式
     * @param radio 曲线拟合/滤波平滑系数
     *
     * @warning rm_movej_canfd() 不执行常规关节轨迹规划
     * 上层必须自行保证命令连续性和变化率
     */
    void write_all_degree(const std::array<float, 6>& degree, bool follow = false, int trajectory_mode = 0, int radio = 0);

    /**
     * @brief 使用控制器关节空间规划移动到目标角度
     * @param degree J1~J6 目标角度，单位 degree
     * @param speed_percent 规划速度/加速度百分比 [1, 100]
     * @param block true 等待机械臂到位后返回
     *
     * @note 该接口用于归零和启动前慢速对齐，不用于连续遥操作
     */
    void movej_degree(const std::array<float, 6>& degree, int speed_percent = 10, bool block = true);

    /**
     * @brief 发送机械臂轨迹停止命令
     */
    void stop();

private:
    void acquire_sdk();
    void release_sdk() noexcept;

private:
    RM_Service api_;                        ///< SDK 接口对象
    rm_robot_handle* handle_{ nullptr };    ///< SDK 机械臂句柄，非空表示已连接
    bool sdk_acquired_{ false };

    static inline std::mutex sdk_mutex_;
    static inline std::size_t sdk_users_{ 0 };
};

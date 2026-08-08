#pragma once

#include <array>
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
     * @param ip RM65-B 控制器 IPv4 地址，例如 192.168.1.18
     * @param port RM65-B TCP 端口，默认 8080
     * @throws std::logic_error 当前对象已经连接机械臂
     * @throws std::runtime_error SDK 初始化失败、创建机械臂连接失败、机械臂信息读取失败或连接到的机械臂不是 6 自由度
     */
    void connect(const std::string& ip, int port = 8080);

    /**
     * @brief 断开 RM65-B 并销毁 SDK
     * 可重复调用
     * 未连接时不会执行任何机械臂操作
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
     * @brief 读取 RM65-B 当前 6 关节角度
     * @return std::array<float, 6> J1~J6 当前角度，单位 degree
     * @throws std::logic_error 当前没有连接机械臂
     * @throws std::runtime_error SDK 读取失败
     */
    [[nodiscard]] std::array<float, 6> read_all_degree();

    /**
     * @brief 通过 CANFD 透传方式发送 6 关节目标角度
     * @param degree J1~J6 目标角度，单位 degree
     * @param follow 跟随模式
     *        - false 低跟随，V1 默认使用，允许控制器对输入做一定处理
     *        - true 高跟随，对通信周期要求更严格，官方要求透传周期不超过 10 ms
     * @param trajectory_mode 高跟随下的轨迹处理模式
     *        - 0 完全透传
     *        - 1 曲线拟合
     *        - 2 滤波
     *        V1 使用低跟随时保持 0
     * @param radio 曲线拟合或滤波时的平滑系数
     *        V1 使用低跟随时保持 0
     * @throws std::logic_error 当前没有连接机械臂
     * @throws std::runtime_error rm_movej_canfd() 发送失败
     * @warning
     * rm_movej_canfd() 不进行常规关节轨迹规划
     * 上层必须自行保证目标角连续、限位正确、变化率合理
     */
    void write_all_degree(const std::array<float, 6>& degree, bool follow = false, int trajectory_mode = 0, int radio = 0);

    /**
     * @brief 发送机械臂轨迹急停命令
     * @throws std::runtime_error 急停命令发送失败
     */
    void stop();

private:
    /**
     * @brief RealMan SDK C++ 服务对象
     */
    RM_Service api_;

    /**
     * @brief RealMan 机械臂连接句柄
     */
    rm_robot_handle* handle_{ nullptr };

    /**
     * @brief SDK 是否已经通过 rm_init() 完成初始化
     */
    bool sdk_initialized_{ false };
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //



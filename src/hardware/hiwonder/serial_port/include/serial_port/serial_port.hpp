#pragma once

#include <chrono>
#include <system_error>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

/**
 * @brief Linux/POSIX 串口传输层封装
 *
 * SerialPort 只负责串口设备的打开、配置、字节收发、超时和缓冲区控制
 * 不负责协议帧头、长度字段、校验和、设备 ID 等上层协议解析
 *
 * read() 在 read_timeout 内读取当前可用数据
 * read_exact() 在一次 read_timeout 总预算内尽量读取指定长度
 * write() 在一次 write_timeout 总预算内尽量完整发送数据
 *
 * 线程安全：类内部不加锁
 * 多线程同时修改配置、关闭设备或发送数据时，应由调用方同步
 */
class SerialPort {
public:
    /**
     * @brief 单个串口字节类型
     */
    using Byte = std::uint8_t;

    /**
     * @brief 串口字节缓冲区类型
     */
    using Buffer = std::vector<Byte>;

    /**
     * @brief 奇偶校验方式
     */
    enum class Parity {
        /**
         * @brief 不使用奇偶校验
         */
        None,
        /**
         * @brief 使用偶校验
         */
        Even,
        /**
         * @brief 使用奇校验
         */
        Odd,
    };

    /**
     * @brief 停止位数量
     */
    enum class StopBits {
        /**
         * @brief 使用一个停止位
         */
        One,
        /**
         * @brief 使用两个停止位
         */
        Two,
    };

    /**
     * @brief 流控制方式
     */
    enum class FlowControl {
        /**
         * @brief 不使用流控制
         */
        None,
        /**
         * @brief 使用软件流控制
         */
        Software,
        /**
         * @brief 使用硬件流控制
         */
        Hardware,
    };

    /**
     * @brief 串口缓冲区清空方向
     */
    enum class FlushDirection {
        /**
         * @brief 清空输入缓冲区
         */
        Input,
        /**
         * @brief 清空输出缓冲区
         */
        Output,
        /**
         * @brief 同时清空输入和输出缓冲区
         */
        Both,
    };

    /**
     * @brief 串口运行配置
     */
    struct Config {
        /**
         * @brief 波特率
         */
        std::uint32_t baud_rate{ 115200 };
        /**
         * @brief 数据位数量
         */
        std::uint8_t data_bits{ 8 };
        /**
         * @brief 奇偶校验方式
         */
        Parity parity{ Parity::None };
        /**
         * @brief 停止位数量
         */
        StopBits stop_bits{ StopBits::One };
        /**
         * @brief 流控制方式
         */
        FlowControl flow_control{ FlowControl::None };
        /**
         * @brief 单次读取操作的超时时间
         */
        std::chrono::milliseconds read_timeout{ 2 };
        /**
         * @brief 单次写入操作的超时时间
         */
        std::chrono::milliseconds write_timeout{ 100 };
        /**
         * @brief 打开设备后是否清空输入和输出缓冲区
         */
        bool flush_on_open{ true };
    };

    /**
     * @brief 构造未打开的串口对象
     */
    SerialPort() = default;

    /**
     * @brief 使用默认配置打开指定串口
     * @param port 串口设备路径
     */
    explicit SerialPort(std::string port) {
        open(std::move(port));
    }

    /**
     * @brief 使用指定配置打开串口
     * @param port 串口设备路径
     * @param config 串口配置
     */
    SerialPort(std::string port, const Config& config) {
        open(std::move(port), config);
    }

    /**
     * @brief 析构并关闭串口
     */
    ~SerialPort() noexcept {
        close();
    }

    /**
     * @brief 禁止复制构造
     */
    SerialPort(const SerialPort&) = delete;

    /**
     * @brief 禁止复制赋值
     */
    SerialPort& operator=(const SerialPort&) = delete;

    /**
     * @brief 移动构造串口对象
     * @param other 被移动的串口对象
     */
    SerialPort(SerialPort&& other) noexcept
        : fd_(std::exchange(other.fd_, -1)), port_(std::move(other.port_)), config_(other.config_) {}

    /**
     * @brief 移动赋值串口对象
     * @param other 被移动的串口对象
     * @return 当前串口对象
     */
    SerialPort& operator=(SerialPort&& other) noexcept {
        if(this != &other) {
            close();
            fd_ = std::exchange(other.fd_, -1);
            port_ = std::move(other.port_);
            config_ = other.config_;
        }
        return *this;
    }

    /**
     * @brief 使用当前保存的配置打开串口
     */
    void open(std::string port) {
        open(std::move(port), config_);
    }

    /**
     * @brief 使用指定配置打开串口
     *
     * 如果当前对象已经打开串口，只有新设备成功打开并完成配置后
     * 才会关闭原设备并替换 fd
     */
    void open(std::string port, const Config& config) {
        validate_config(config);

        int new_fd = ::open(
            port.c_str(),
            O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);

        if(new_fd < 0) {
            throw_system_error("open(" + port + ")");
        }

        try {
            configure_fd(new_fd, config);
            if(config.flush_on_open) {
                flush_fd(new_fd, FlushDirection::Both);
            }
        }
        catch(...) {
            (void)::close(new_fd);
            throw;
        }

        close();
        fd_ = new_fd;
        port_ = std::move(port);
        config_ = config;
    }

    /**
     * @brief 关闭串口
     *
     * noexcept，允许重复调用
     */
    void close() noexcept {
        if(fd_ >= 0) {
            // Linux 上 close() 被 EINTR 中断时 fd 也可能已经释放
            // 因此不能盲目重试
            (void)::close(fd_);
            fd_ = -1;
        }
        port_.clear();
    }

    /**
     * @brief 查询串口是否已经打开
     * @return 串口已打开时返回 true
     */
    [[nodiscard]] bool is_open() const noexcept {
        return fd_ >= 0;
    }

    /**
     * @brief 获取底层文件描述符
     * @return 文件描述符或未打开时的负值
     */
    [[nodiscard]] int native_handle() const noexcept {
        return fd_;
    }

    /**
     * @brief 获取当前串口设备路径
     * @return 串口设备路径
     */
    [[nodiscard]] const std::string& port() const noexcept {
        return port_;
    }

    /**
     * @brief 获取当前串口配置
     * @return 当前配置的只读引用
     */
    [[nodiscard]] const Config& config() const noexcept {
        return config_;
    }

    /**
     * @brief 更新串口配置
     *
     * 已打开设备时立即应用配置
     * 未打开设备时仅保存配置
     */
    void set_config(const Config& config) {
        validate_config(config);
        if(is_open()) {
            configure_fd(fd_, config);
        }
        config_ = config;
    }

    /**
     * @brief 设置读取超时时间
     * @param timeout 新的读取超时时间
     */
    void set_read_timeout(std::chrono::milliseconds timeout) {
        validate_timeout(timeout, "read_timeout");
        config_.read_timeout = timeout;
    }

    /**
     * @brief 设置写入超时时间
     * @param timeout 新的写入超时时间
     */
    void set_write_timeout(std::chrono::milliseconds timeout) {
        validate_timeout(timeout, "write_timeout");
        config_.write_timeout = timeout;
    }

    /**
     * @brief 读取最多 len 字节到指定缓冲区
     * @param data 接收数据的缓冲区
     * @param len 缓冲区容量
     * @return 实际读取字节数
     */
    std::size_t read(Byte* data, std::size_t len) {
        ensure_open();
        validate_buffer(data, len);

        if(len == 0) {
            return 0;
        }

        if(!wait_ready(POLLIN, config_.read_timeout)) {
            return 0;
        }

        for(;;) {
            const ssize_t ret = ::read(fd_, data, len);
            if(ret >= 0) {
                return static_cast<std::size_t>(ret);
            }
            if(errno == EINTR) {
                continue;
            }
            if(errno == EAGAIN || errno == EWOULDBLOCK) {
                return 0;
            }
            throw_system_error("read(" + port_ + ")");
        }
    }

    /**
     * @brief 读取最多 max_bytes 字节并返回新缓冲区
     * @param max_bytes 最大读取字节数
     * @return 实际读取到的字节
     */
    [[nodiscard]] Buffer read(std::size_t max_bytes) {
        Buffer buffer(max_bytes);
        const std::size_t received = read(buffer.data(), buffer.size());
        buffer.resize(received);
        return buffer;
    }

    /**
     * @brief 读取最多 max_bytes 字节到指定缓冲区
     * @param buffer 接收数据的缓冲区
     * @param max_bytes 最大读取字节数
     * @return 实际读取字节数
     */
    std::size_t read(Buffer& buffer, std::size_t max_bytes) {
        buffer.resize(max_bytes);
        const std::size_t received = read(buffer.data(), buffer.size());
        buffer.resize(received);
        return received;
    }

    /**
     * @brief 在一次读取超时预算内尽量读取指定长度
     * @param data 接收数据的缓冲区
     * @param len 期望读取字节数
     * @return 实际读取字节数
     */
    std::size_t read_exact(Byte* data, std::size_t len) {
        ensure_open();
        validate_buffer(data, len);

        if(len == 0) {
            return 0;
        }

        const auto deadline =
            std::chrono::steady_clock::now() + config_.read_timeout;
        std::size_t total = 0;

        while(total < len) {
            const auto remaining = remaining_time(deadline);
            if(!wait_ready(POLLIN, remaining)) {
                break;
            }

            const ssize_t ret = ::read(fd_, data + total, len - total);
            if(ret > 0) {
                total += static_cast<std::size_t>(ret);
                continue;
            }
            if(ret == 0) {
                break;
            }
            if(errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
                continue;
            }
            throw_system_error("read(" + port_ + ")");
        }

        return total;
    }

    /**
     * @brief 在一次读取超时预算内尽量读取指定长度并返回新缓冲区
     * @param len 期望读取字节数
     * @return 实际读取到的字节
     */
    [[nodiscard]] Buffer read_exact(std::size_t len) {
        Buffer buffer(len);
        const std::size_t received = read_exact(buffer.data(), buffer.size());
        buffer.resize(received);
        return buffer;
    }

    /**
     * @brief 在一次读取超时预算内尽量读取指定长度到缓冲区
     * @param buffer 接收数据的缓冲区
     * @param len 期望读取字节数
     * @return 实际读取字节数
     */
    std::size_t read_exact(Buffer& buffer, std::size_t len) {
        buffer.resize(len);
        const std::size_t received = read_exact(buffer.data(), buffer.size());
        buffer.resize(received);
        return received;
    }

    /**
     * @brief 在一次写入超时预算内尽量发送指定长度
     * @param data 待发送数据
     * @param len 待发送字节数
     * @return 实际写入字节数
     */
    std::size_t write(const Byte* data, std::size_t len) {
        ensure_open();
        validate_buffer(data, len);

        if(len == 0) {
            return 0;
        }

        const auto deadline =
            std::chrono::steady_clock::now() + config_.write_timeout;
        std::size_t total = 0;

        while(total < len) {
            const auto remaining = remaining_time(deadline);
            if(!wait_ready(POLLOUT, remaining)) {
                break;
            }

            const ssize_t ret = ::write(fd_, data + total, len - total);
            if(ret > 0) {
                total += static_cast<std::size_t>(ret);
                continue;
            }
            if(ret == 0) {
                break;
            }
            if(errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
                continue;
            }
            throw_system_error("write(" + port_ + ")");
        }

        return total;
    }

    /**
     * @brief 完整发送 Buffer
     */
    /**
     * @brief 发送整个字节缓冲区
     * @param data 待发送缓冲区
     * @return 实际写入字节数
     */
    std::size_t write(const Buffer& data) {
        return write(data.data(), data.size());
    }

    /**
     * @brief 直接发送短字节序列
     */
    /**
     * @brief 发送初始化列表中的字节
     * @param data 待发送字节序列
     * @return 实际写入字节数
     */
    std::size_t write(std::initializer_list<Byte> data) {
        return write(data.begin(), data.size());
    }

    /**
     * @brief 等待驱动和 UART 发送缓冲中的数据发送完成
     */
    void drain() {
        ensure_open();

        int ret;
        do {
            ret = ::tcdrain(fd_);
        }
        while(ret < 0 && errno == EINTR);

        if(ret < 0) {
            throw_system_error("tcdrain(" + port_ + ")");
        }
    }

    /**
     * @brief 清空串口内核缓冲区
     */
    /**
     * @brief 清空指定方向的串口内核缓冲区
     * @param direction 要清空的缓冲区方向
     */
    void flush(FlushDirection direction = FlushDirection::Both) {
        ensure_open();
        flush_fd(fd_, direction);
    }

    /**
     * @brief 查询当前内核输入缓冲区中可立即读取的字节数
     */
    [[nodiscard]] std::size_t available() const {
        ensure_open();

        int bytes = 0;
        if(::ioctl(fd_, FIONREAD, &bytes) < 0) {
            throw_system_error("ioctl(FIONREAD, " + port_ + ")");
        }
        return bytes > 0 ? static_cast<std::size_t>(bytes) : 0U;
    }

private:
    /**
     * @brief 检查数据指针和长度是否匹配
     * @param data 数据缓冲区指针
     * @param len 数据长度
     */
    static void validate_buffer(const void* data, std::size_t len) {
        if(data == nullptr && len != 0) {
            throw std::invalid_argument("SerialPort buffer is null while len != 0");
        }
    }

    /**
     * @brief 检查超时时间是否合法
     * @param timeout 待检查的超时时间
     * @param name 超时配置名称
     */
    static void validate_timeout(
        std::chrono::milliseconds timeout,
        const char* name) {
        if(timeout.count() < 0) {
            throw std::invalid_argument(std::string("SerialPort ") + name + " must be >= 0 ms");
        }
    }

    /**
     * @brief 检查串口配置是否合法
     * @param config 待检查的串口配置
     */
    static void validate_config(const Config& config) {
        if(config.data_bits < 5 || config.data_bits > 8) {
            throw std::invalid_argument("SerialPort data_bits must be in [5, 8]");
        }
        if(config.baud_rate == 0) {
            throw std::invalid_argument("SerialPort baud_rate must be > 0");
        }

        validate_timeout(config.read_timeout, "read_timeout");
        validate_timeout(config.write_timeout, "write_timeout");

        // 提前验证，避免打开设备后才发现当前平台不支持该波特率
        (void)baud_to_speed(config.baud_rate);
    }

    /**
     * @brief 将数值波特率转换为 termios 波特率常量
     * @param baud_rate 数值波特率
     * @return termios 波特率常量
     */
    static speed_t baud_to_speed(std::uint32_t baud_rate) {
        switch(baud_rate) {
            case 50: return B50;
            case 75: return B75;
            case 110: return B110;
            case 134: return B134;
            case 150: return B150;
            case 200: return B200;
            case 300: return B300;
            case 600: return B600;
            case 1200: return B1200;
            case 1800: return B1800;
            case 2400: return B2400;
            case 4800: return B4800;
            case 9600: return B9600;
            case 19200: return B19200;
            case 38400: return B38400;
#ifdef B57600
            case 57600: return B57600;
#endif
#ifdef B115200
            case 115200: return B115200;
#endif
#ifdef B230400
            case 230400: return B230400;
#endif
#ifdef B460800
            case 460800: return B460800;
#endif
#ifdef B500000
            case 500000: return B500000;
#endif
#ifdef B576000
            case 576000: return B576000;
#endif
#ifdef B921600
            case 921600: return B921600;
#endif
#ifdef B1000000
            case 1000000: return B1000000;
#endif
#ifdef B1152000
            case 1152000: return B1152000;
#endif
#ifdef B1500000
            case 1500000: return B1500000;
#endif
#ifdef B2000000
            case 2000000: return B2000000;
#endif
#ifdef B2500000
            case 2500000: return B2500000;
#endif
#ifdef B3000000
            case 3000000: return B3000000;
#endif
#ifdef B3500000
            case 3500000: return B3500000;
#endif
#ifdef B4000000
            case 4000000: return B4000000;
#endif
            default:
                throw std::invalid_argument("SerialPort baud_rate is not supported by this termios platform: " + std::to_string(baud_rate));
        }
    }

    /**
     * @brief 使用配置初始化文件描述符
     * @param fd 串口文件描述符
     * @param config 串口配置
     */
    static void configure_fd(int fd, const Config& config) {
        struct termios tty {};
        if(::tcgetattr(fd, &tty) < 0) {
            throw_system_error("tcgetattr");
        }

        ::cfmakeraw(&tty);

        const speed_t speed = baud_to_speed(config.baud_rate);
        if(::cfsetispeed(&tty, speed) < 0 || ::cfsetospeed(&tty, speed) < 0) {
            throw_system_error("cfsetispeed/cfsetospeed");
        }

        tty.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
        tty.c_cflag &= static_cast<tcflag_t>(~CSIZE);

        switch(config.data_bits) {
            case 5: tty.c_cflag |= CS5; break;
            case 6: tty.c_cflag |= CS6; break;
            case 7: tty.c_cflag |= CS7; break;
            case 8: tty.c_cflag |= CS8; break;
            default: break;
        }

        tty.c_cflag &= static_cast<tcflag_t>(~(PARENB | PARODD));
        tty.c_iflag &= static_cast<tcflag_t>(~INPCK);

        if(config.parity == Parity::Even) {
            tty.c_cflag |= PARENB;
            tty.c_iflag |= INPCK;
        }
        else if(config.parity == Parity::Odd) {
            tty.c_cflag |= static_cast<tcflag_t>(PARENB | PARODD);
            tty.c_iflag |= INPCK;
        }

        if(config.stop_bits == StopBits::Two) {
            tty.c_cflag |= CSTOPB;
        }
        else {
            tty.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
        }

        tty.c_iflag &= static_cast<tcflag_t>(~(IXON | IXOFF | IXANY));
#ifdef CRTSCTS
        tty.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
#endif

        if(config.flow_control == FlowControl::Software) {
            tty.c_iflag |= static_cast<tcflag_t>(IXON | IXOFF);
        }
        else if(config.flow_control == FlowControl::Hardware) {
#ifdef CRTSCTS
            tty.c_cflag |= CRTSCTS;
#else
            throw std::invalid_argument("SerialPort hardware flow control is unsupported on this platform");
#endif
        }

        // 实际阻塞策略完全由 poll() 管理
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 0;

        if(::tcsetattr(fd, TCSANOW, &tty) < 0) {
            throw_system_error("tcsetattr");
        }
    }

    /**
     * @brief 清空文件描述符对应的串口缓冲区
     * @param fd 串口文件描述符
     * @param direction 要清空的缓冲区方向
     */
    static void flush_fd(int fd, FlushDirection direction) {
        int queue = TCIOFLUSH;
        switch(direction) {
            case FlushDirection::Input: queue = TCIFLUSH; break;
            case FlushDirection::Output: queue = TCOFLUSH; break;
            case FlushDirection::Both: queue = TCIOFLUSH; break;
        }

        int ret;
        do {
            ret = ::tcflush(fd, queue);
        }
        while(ret < 0 && errno == EINTR);

        if(ret < 0) {
            throw_system_error("tcflush");
        }
    }

    /**
     * @brief 等待文件描述符具备指定的读写事件
     * @param events 等待的 poll 事件
     * @param timeout 等待超时时间
     * @return 事件就绪时返回 true
     */
    [[nodiscard]] bool wait_ready(
        short events,
        std::chrono::milliseconds timeout) const {
        ensure_open();
        validate_timeout(timeout, "I/O timeout");

        const auto deadline = std::chrono::steady_clock::now() + timeout;

        for(;;) {
            struct pollfd pfd {};
            pfd.fd = fd_;
            pfd.events = events;

            const int timeout_ms = to_poll_timeout(remaining_time(deadline));
            const int ret = ::poll(&pfd, 1, timeout_ms);

            if(ret > 0) {
                if((pfd.revents & POLLNVAL) != 0) {
                    throw std::system_error(EBADF, std::generic_category(), "poll(" + port_ + ")");
                }
                if((pfd.revents & POLLERR) != 0) {
                    throw std::runtime_error("SerialPort poll error on " + port_);
                }
                if((pfd.revents & events) != 0) {
                    return true;
                }
                if((pfd.revents & POLLHUP) != 0) {
                    throw std::runtime_error("SerialPort disconnected: " + port_);
                }
                continue;
            }

            if(ret == 0) {
                return false;
            }

            if(errno == EINTR) {
                if(std::chrono::steady_clock::now() >= deadline) {
                    return false;
                }
                continue;
            }

            throw_system_error("poll(" + port_ + ")");
        }
    }

    /**
     * @brief 计算截止时间前的剩余毫秒数
     * @param deadline 操作截止时间
     * @return 剩余时间
     */
    static std::chrono::milliseconds remaining_time(
        const std::chrono::steady_clock::time_point& deadline) {
        const auto now = std::chrono::steady_clock::now();
        if(now >= deadline) {
            return std::chrono::milliseconds{ 0 };
        }

        const auto remaining = deadline - now;
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(remaining);
        if(ms.count() == 0 && remaining > std::chrono::steady_clock::duration::zero()) {
            ms = std::chrono::milliseconds{ 1 };
        }
        return ms;
    }

    /**
     * @brief 将毫秒超时时间转换为 poll 参数
     * @param timeout 毫秒超时时间
     * @return poll 使用的整数超时时间
     */
    static int to_poll_timeout(std::chrono::milliseconds timeout) {
        if(timeout.count() < 0) {
            return 0;
        }
        if(timeout.count() > std::numeric_limits<int>::max()) {
            return std::numeric_limits<int>::max();
        }
        return static_cast<int>(timeout.count());
    }

    /**
     * @brief 检查串口是否已打开
     */
    void ensure_open() const {
        if(fd_ < 0) {
            throw std::logic_error("SerialPort is not open");
        }
    }

    /**
     * @brief 根据当前 errno 抛出系统错误
     * @param operation 发生错误的操作名称
     */
    [[noreturn]] static void throw_system_error(
        const std::string& operation) {
        const int error = errno;
        throw std::system_error(error, std::generic_category(), operation);
    }

    int fd_{ -1 };
    std::string port_;
    Config config_{};
};

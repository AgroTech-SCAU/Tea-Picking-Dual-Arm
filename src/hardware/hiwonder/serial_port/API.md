# SerialPort API Reference

本文档面向需要在 Linux C++ 项目中复用串口通信能力的开发者

`SerialPort` 只负责串口传输层，不负责任何具体设备协议

每个主要公共接口按以下顺序说明

```text
Doxygen 语义
参数
返回值
具体使用示例
使用注意
```

---

## 1. API 总表

### 1.1. 公共接口索引

| API | 主要用途 |
| --- | --- |
| `SerialPort::Byte` | 单字节类型 |
| `SerialPort::Buffer` | 串口字节缓冲 |
| `SerialPort::Config` | 串口参数 |
| `open()` | 打开设备 |
| `close()` | 关闭设备 |
| `is_open()` | 查询状态 |
| `set_config()` | 更新串口配置 |
| `set_read_timeout()` | 更新读超时 |
| `set_write_timeout()` | 更新写超时 |
| `read(Byte*, len)` | 零分配读取最多 N 字节 |
| `read(max_bytes)` | 返回 Buffer 的普通读取 |
| `read(Buffer&, max_bytes)` | 复用 Buffer 读取 |
| `read_exact(...)` | 固定长度读取 |
| `write(Byte*, len)` | 完整发送原始缓冲 |
| `write(Buffer)` | 完整发送 Buffer |
| `write({...})` | 发送短初始化列表 |
| `drain()` | 等待 UART 实际发送结束 |
| `flush()` | 清空 tty 缓冲 |
| `available()` | 查询当前可读字节数 |
| `native_handle()` | 获取 Linux fd |

---

## 2. API 使用入口

**头文件**

```cpp
#include "serial_port.hpp"
```

**平台要求**

```text
Linux / POSIX
C++17 或更高版本
无第三方运行时依赖
```

**推荐分层**

```text
Application
    ↓
Device Driver
    ↓
Protocol Parser / Packet Builder
    ↓
SerialPort
    ↓
Linux tty / termios
    ↓
USB-UART / UART device
```

**职责划分**

| 层级 | 负责内容 |
| --- | --- |
| `SerialPort` | 打开设备、串口配置、字节收发、超时、flush、drain |
| Protocol | 帧头、长度、校验、命令格式、粘包拆包 |
| Device Driver | 设备 ID、寄存器、状态语义、设备命令 |
| Application | 控制周期、数据记录、业务逻辑 |

---

## 3. 最小使用示例

```cpp
#include "serial_port.hpp"

#include <chrono>
#include <iostream>

int main() {
    SerialPort::Config cfg;
    cfg.baud_rate = 1000000;
    cfg.data_bits = 8;
    cfg.parity = SerialPort::Parity::None;
    cfg.stop_bits = SerialPort::StopBits::One;
    cfg.flow_control = SerialPort::FlowControl::None;
    cfg.read_timeout = std::chrono::milliseconds{5};
    cfg.write_timeout = std::chrono::milliseconds{20};

    SerialPort serial("/dev/ttyUSB0", cfg);

    SerialPort::Buffer tx{
        0x55, 0x55, 0x01, 0x03, 0x1C, 0xDF
    };

    const std::size_t sent = serial.write(tx);
    if(sent != tx.size()) {
        std::cerr << "serial write timeout\n";
        return 1;
    }

    const auto rx = serial.read(64);

    std::cout << "received " << rx.size() << " bytes\n";
}
```

对于很短的固定测试命令，也可以直接写

```cpp
serial.write({
    0x55, 0x55, 0x01, 0x03, 0x1C, 0xDF
});
```

---

## 4. 基础类型

### 4.1. `SerialPort::Byte`

**定义**

```cpp
using Byte = std::uint8_t;
```

**用途**

表示串口传输层的单字节数据

**示例**

```cpp
SerialPort::Byte id = 0x01;
```

`Byte` 只是类型别名，不引入额外封装成本

---

### 4.2. `SerialPort::Buffer`

**定义**

```cpp
using Buffer = std::vector<Byte>;
```

**用途**

表示一段可变长度的串口字节数据

#### 4.2.1. 具体使用示例

```cpp
SerialPort::Buffer packet{
    0x55,
    0x55,
    0x01,
    0x03,
    0x1C,
    0xDF,
};

serial.write(packet);
```

**读取数据**

```cpp
SerialPort::Buffer rx = serial.read(64);
```

**复用内存**

```cpp
SerialPort::Buffer rx;
rx.reserve(256);

while(running) {
    const auto n = serial.read(rx, 256);
    if(n == 0) {
        continue;
    }

    parser.push(rx.data(), rx.size());
}
```

#### 4.2.2. 使用注意

`Buffer` 使用 `std::vector<std::uint8_t>` 而不是自定义容器

这样既能让 API 具有明确的串口语义，又可以直接使用标准库的 `size()`、`data()`、迭代器和算法

`Buffer` 不代表一个完整协议帧

协议层如果需要区分 `Packet`、`Frame` 或 `Command`，应在具体 Protocol 模块中定义自己的类型

---

## 5. 配置类型

### 5.1. `SerialPort::Config`

**定义**

```cpp
struct Config {
    std::uint32_t baud_rate{115200};
    std::uint8_t data_bits{8};
    Parity parity{Parity::None};
    StopBits stop_bits{StopBits::One};
    FlowControl flow_control{FlowControl::None};
    std::chrono::milliseconds read_timeout{2};
    std::chrono::milliseconds write_timeout{100};
    bool flush_on_open{true};
};
```

**字段**

| 字段 | 默认值 | 含义 |
| --- | ---: | --- |
| `baud_rate` | `115200` | 实际波特率数值 |
| `data_bits` | `8` | 数据位，允许 5 至 8 |
| `parity` | `None` | 奇偶校验 |
| `stop_bits` | `One` | 停止位 |
| `flow_control` | `None` | 流控 |
| `read_timeout` | `2 ms` | `read()` 等待超时或 `read_exact()` 总超时 |
| `write_timeout` | `100 ms` | `write()` 完整发送操作的总超时 |
| `flush_on_open` | `true` | 打开后是否清空输入和输出缓冲 |

#### 5.1.1. 具体使用示例

```cpp
SerialPort::Config cfg;
cfg.baud_rate = 1000000;
cfg.read_timeout = std::chrono::milliseconds{5};
cfg.write_timeout = std::chrono::milliseconds{20};

SerialPort serial("/dev/ttyUSB0", cfg);
```

#### 5.1.2. 可用波特率

当前实现通过 Linux `termios` 的 `Bxxxx` 波特率常量完成映射

**基础波特率**

以下波特率在当前实现中直接支持

| 波特率 | `termios` 常量 |
| ---: | --- |
| `50` | `B50` |
| `75` | `B75` |
| `110` | `B110` |
| `134` | `B134` |
| `150` | `B150` |
| `200` | `B200` |
| `300` | `B300` |
| `600` | `B600` |
| `1200` | `B1200` |
| `1800` | `B1800` |
| `2400` | `B2400` |
| `4800` | `B4800` |
| `9600` | `B9600` |
| `19200` | `B19200` |
| `38400` | `B38400` |

**条件支持的高速波特率**

以下波特率只有在当前 Linux/libc 的 `termios` 头文件定义了对应 `Bxxxx` 常量时才会被编译进 `SerialPort`

| 波特率 | `termios` 常量 |
| ---: | --- |
| `57600` | `B57600` |
| `115200` | `B115200` |
| `230400` | `B230400` |
| `460800` | `B460800` |
| `500000` | `B500000` |
| `576000` | `B576000` |
| `921600` | `B921600` |
| `1000000` | `B1000000` |
| `1152000` | `B1152000` |
| `1500000` | `B1500000` |
| `2000000` | `B2000000` |
| `2500000` | `B2500000` |
| `3000000` | `B3000000` |
| `3500000` | `B3500000` |
| `4000000` | `B4000000` |

在常见 Linux 开发环境中，上述高速档位通常可用，但最终仍以编译平台是否定义对应 `Bxxxx` 为准

**不可用波特率的错误行为**

`SerialPort` 不会把无法映射的数值静默降级到其他波特率

`open()` 和 `set_config()` 在修改设备配置之前都会执行配置校验，`validate_config()` 会调用内部 `baud_to_speed()` 检查波特率映射

如果当前平台不支持指定波特率，会抛出 `std::invalid_argument`

```cpp
SerialPort serial;
SerialPort::Config cfg;
cfg.baud_rate = 12345;

try {
    serial.set_config(cfg);
} catch(const std::invalid_argument& e) {
    std::cerr << e.what() << '\n';
}
```

典型错误信息为

```text
SerialPort baud_rate is not supported by this termios platform: 12345
```

因此，不支持的波特率可以被调用方正常捕获，不会继续以错误波特率配置设备

---

### 5.2. `SerialPort::Parity`

```cpp
enum class Parity {
    None,
    Even,
    Odd,
};
```

| 枚举 | 含义 |
| --- | --- |
| `None` | 无校验 |
| `Even` | 偶校验 |
| `Odd` | 奇校验 |

---

### 5.3. `SerialPort::StopBits`

```cpp
enum class StopBits {
    One,
    Two,
};
```

---

### 5.4. `SerialPort::FlowControl`

```cpp
enum class FlowControl {
    None,
    Software,
    Hardware,
};
```

| 枚举 | 含义 |
| --- | --- |
| `None` | 无流控 |
| `Software` | XON/XOFF |
| `Hardware` | RTS/CTS |

普通 USB-TTL、舵机总线和 MCU UART 通常使用 `None`

---

### 5.5. `SerialPort::FlushDirection`

```cpp
enum class FlushDirection {
    Input,
    Output,
    Both,
};
```

用于 `flush()` 指定清空方向

---

## 6. 生命周期与配置

### 6.1. `SerialPort()`

**定义**

```cpp
SerialPort() = default;
```

创建一个尚未打开设备的对象

#### 6.1.1. 示例

```cpp
SerialPort serial;

SerialPort::Config cfg;
cfg.baud_rate = 1000000;

serial.set_config(cfg);
serial.open("/dev/ttyUSB0");
```

---

### 6.2. `SerialPort(std::string port)`

**定义**

```cpp
explicit SerialPort(std::string port);
```

使用默认配置打开串口

#### 6.2.1. 示例

```cpp
SerialPort serial("/dev/ttyUSB0");
```

---

### 6.3. `SerialPort(std::string port, const Config& config)`

**定义**

```cpp
SerialPort(std::string port, const Config& config);
```

使用指定配置打开串口

#### 6.3.1. 异常

| 类型 | 条件 |
| --- | --- |
| `std::system_error` | Linux 系统调用失败 |
| `std::invalid_argument` | 参数非法或波特率不支持 |

---

### 6.4. `open()`

**定义**

```cpp
void open(std::string port);
void open(std::string port, const Config& config);
```

**用途**

打开指定串口设备

`open(port)` 使用当前 `SerialPort` 对象中保存的配置

`open(port, config)` 使用传入的 `config` 打开设备，并更新当前配置

如果对象已经打开了一个串口，只有在新串口成功打开并完成配置后，才会关闭原来的串口；如果新串口打开或配置失败，原来的串口仍保持可用状态

#### 6.4.1. 示例

```cpp
SerialPort serial;
serial.open("/dev/ttyUSB0");
```

指定配置打开

```cpp
SerialPort::Config cfg;
cfg.baud_rate = 1000000;
serial.open("/dev/ttyUSB0", cfg);
```

---

### 6.5. `close()`

**定义**

```cpp
void close() noexcept;
```

关闭当前串口

允许重复调用

析构函数会自动调用 `close()`

#### 6.5.1. 示例

```cpp
serial.close();
```

通常不需要手动关闭

RAII 生命周期结束时会自动释放 fd

---

### 6.6. `is_open()`

```cpp
[[nodiscard]] bool is_open() const noexcept;
```

#### 6.6.1. 示例

```cpp
if(!serial.is_open()) {
    return;
}
```

---

### 6.7. `native_handle()`

```cpp
[[nodiscard]] int native_handle() const noexcept;
```

返回 Linux fd

普通业务代码不应依赖这个接口

只在需要调用额外 ioctl 或平台特定功能时使用

---

### 6.8. `port()`

```cpp
[[nodiscard]] const std::string& port() const noexcept;
```

返回当前设备路径

---

### 6.9. `config()`

```cpp
[[nodiscard]] const Config& config() const noexcept;
```

返回当前配置

---

### 6.10. `set_config()`

```cpp
void set_config(const Config& config);
```

对象已经打开时立即重新配置设备

对象尚未打开时只保存配置

---

### 6.11. `set_read_timeout()`

```cpp
void set_read_timeout(std::chrono::milliseconds timeout);
```

#### 6.11.1. 示例

```cpp
serial.set_read_timeout(std::chrono::milliseconds{5});
```

---

### 6.12. `set_write_timeout()`

```cpp
void set_write_timeout(std::chrono::milliseconds timeout);
```

---

## 7. 数据读取

### 7.1. `read(Byte*, std::size_t)`

**Doxygen 语义**

```cpp
std::size_t read(Byte* data, std::size_t len);
```

在 `read_timeout` 内等待串口变为可读

一旦存在数据，最多读取 `len` 字节并返回

该接口不要求读取满 `len` 字节

#### 7.1.1. 参数

| 参数 | 含义 |
| --- | --- |
| `data` | 接收缓冲区 |
| `len` | 缓冲区最大长度 |

#### 7.1.2. 返回值

实际读取字节数

超时返回 `0`

#### 7.1.3. 示例

```cpp
std::uint8_t buffer[64]{};

const auto n = serial.read(buffer, sizeof(buffer));
```

#### 7.1.4. 使用注意

这是零额外内存分配的底层接口

适合实时循环或已有固定缓冲区的代码

---

### 7.2. `read(std::size_t)`

**Doxygen 语义**

```cpp
[[nodiscard]] Buffer read(std::size_t max_bytes);
```

读取最多 `max_bytes` 字节并直接返回 `SerialPort::Buffer`

#### 7.2.1. 返回值

- 收到数据时返回实际长度的 `Buffer`
- 超时时返回空 `Buffer`

#### 7.2.2. 示例

```cpp
const auto rx = serial.read(64);

if(rx.empty()) {
    // 本次读取超时或暂无数据
}
```

这是普通业务代码最推荐的读取方式

---

### 7.3. `read(Buffer&, std::size_t)`

**定义**

```cpp
std::size_t read(Buffer& buffer, std::size_t max_bytes);
```

把读取结果写入调用方提供的 `Buffer`

函数返回后，`buffer.size()` 等于实际读取字节数

#### 7.3.1. 示例

```cpp
SerialPort::Buffer rx;
rx.reserve(256);

while(running) {
    const auto n = serial.read(rx, 256);

    if(n != 0) {
        parser.push(rx.data(), rx.size());
    }
}
```

#### 7.3.2. 使用注意

该重载适合长期循环复用内存

调用方可以提前 `reserve()`，避免反复申请堆内存

---

### 7.4. `read_exact()`

**定义**

```cpp
std::size_t read_exact(Byte* data, std::size_t len);
[[nodiscard]] Buffer read_exact(std::size_t len);
std::size_t read_exact(Buffer& buffer, std::size_t len);
```

在一次 `read_timeout` 总时间预算内尽量读取指定长度

如果超时，返回或输出的数据长度可能小于 `len`

#### 7.4.1. 示例一：直接返回 Buffer

```cpp
const auto response = serial.read_exact(6);

if(response.size() != 6) {
    // 固定长度响应未完整到达
}
```

#### 7.4.2. 示例二：复用 Buffer

```cpp
SerialPort::Buffer response;
response.reserve(16);

const auto n = serial.read_exact(response, 6);
```

#### 7.4.3. 使用注意

不要把 `read_exact()` 当成协议解析器

对于存在帧头、长度字段、粘包、半包和校验的协议，推荐模式仍然是

```cpp
const auto chunk = serial.read(128);
parser.push(chunk);
```

由 Parser 决定何时形成完整帧

---

## 8. 数据发送

### 8.1. `write(const Byte*, std::size_t)`

**Doxygen 语义**

```cpp
std::size_t write(const Byte* data, std::size_t len);
```

在一次 `write_timeout` 总时间预算内尽量发送完整数据

内部自动处理 POSIX `write()` 的短写情况

#### 8.1.1. 返回值

实际成功交给 tty 驱动的字节数

如果

```cpp
written == len
```

表示整个缓冲区已成功写入 tty

如果小于 `len`，表示写超时或未完成

#### 8.1.2. 示例

```cpp
const std::uint8_t data[]{0x01, 0x02, 0x03};

const auto written = serial.write(data, sizeof(data));
```

#### 8.1.3. 使用注意

`write()` 已经具有原来 `write_all()` 的语义

因此公共 API 不再提供 `write_some()` / `write_all()` 两组名称

普通协议代码只需要一个明确的 `write()`

---

### 8.2. `write(const Buffer&)`

**定义**

```cpp
std::size_t write(const Buffer& data);
```

#### 8.2.1. 示例

```cpp
SerialPort::Buffer packet{
    0x55, 0x55, 0x01, 0x03, 0x1C, 0xDF
};

const auto written = serial.write(packet);

if(written != packet.size()) {
    // write timeout
}
```

这是协议 Driver 最推荐的发送接口

---

### 8.3. `write(std::initializer_list<Byte>)`

**定义**

```cpp
std::size_t write(std::initializer_list<Byte> data);
```

用于直接发送短字节序列

#### 8.3.1. 示例

```cpp
serial.write({
    0x55,
    0x55,
    0x01,
    0x03,
    0x1C,
    0xDF,
});
```

#### 8.3.2. 使用注意

更复杂的协议帧仍然建议先由 Packet Builder 生成 `Buffer`

不要在业务代码中大量手写魔法字节

---

## 9. 缓冲区控制

### 9.1. `drain()`

```cpp
void drain();
```

等待 tty 驱动和 UART 发送缓冲中的字节实际发送完成

#### 9.1.1. 示例

```cpp
serial.write(packet);
serial.drain();
```

#### 9.1.2. 适用场景

- 半双工 RS485 方向切换
- 发送完成后马上切换总线方向
- 某些严格要求发送时序的舵机总线

#### 9.1.3. 使用注意

`write() == packet.size()` 只表示数据已经交给 tty 驱动

不表示最后一个 UART bit 已经离开发送器

需要这个保证时再调用 `drain()`

---

### 9.2. `flush()`

```cpp
void flush(FlushDirection direction = FlushDirection::Both);
```

清空内核 tty 缓冲区

#### 9.2.1. 示例

```cpp
serial.flush(SerialPort::FlushDirection::Input);
```

不要把 `flush()` 当作常规协议同步手段

频繁清空输入缓冲可能丢失合法响应

---

### 9.3. `available()`

```cpp
[[nodiscard]] std::size_t available() const;
```

返回当前输入缓冲中可以立即读取的字节数

#### 9.3.1. 示例

```cpp
if(serial.available() != 0) {
    const auto rx = serial.read(128);
}
```

通常不需要先调用 `available()` 再调用 `read()`

`read()` 自己已经负责 `poll()` 和超时

`available()` 更适合调试、诊断或特殊事件循环

---

## 10. 协议层使用方式

### 10.1. 推荐的流式协议读取方式

```cpp
class ServoBus {
public:
    explicit ServoBus(SerialPort serial)
        : serial_(std::move(serial)) {}

    void poll() {
        const auto data = serial_.read(128);

        if(data.empty()) {
            return;
        }

        parser_.push(data.data(), data.size());

        while(auto frame = parser_.next_frame()) {
            handle_frame(*frame);
        }
    }

private:
    SerialPort serial_;
    Parser parser_;
};
```

`SerialPort` 不理解

```text
0x55 0x55
ID
Length
Command
Payload
Checksum
```

这些全部由 `Parser` 处理

---

### 10.2. 推荐的发送方式

```cpp
SerialPort::Buffer packet =
    Protocol::make_move_command(
        servo_id,
        position,
        duration_ms);

const auto written = serial.write(packet);

if(written != packet.size()) {
    throw std::runtime_error("servo packet write timeout");
}
```

如果总线方向切换依赖 UART 真正发送完成

```cpp
serial.write(packet);
serial.drain();
set_bus_rx_mode();
```

---

## 11. 错误处理

### 11.1. 参数错误

以下情况抛出 `std::invalid_argument`

- `baud_rate == 0`
- 数据位不在 5 至 8
- timeout 小于 0
- 当前平台不支持指定 termios 波特率
- `len != 0` 时传入空指针

其中波特率错误会在 `open()` 或 `set_config()` 的配置校验阶段抛出，不会静默替换为最接近的波特率

例如 `baud_rate = 12345` 会抛出类似以下异常信息

```text
SerialPort baud_rate is not supported by this termios platform: 12345
```

---

### 11.2. 系统调用错误

以下操作失败时主要抛出 `std::system_error`

```text
open
poll
read
write
tcgetattr
tcsetattr
tcflush
tcdrain
ioctl
```

**示例**

```cpp
try {
    SerialPort serial("/dev/ttyUSB0");
} catch(const std::system_error& e) {
    std::cerr << e.what() << '\n';
}
```

---

### 11.3. 超时不是异常

正常 I/O 超时不会抛异常

**读取超时**

```cpp
const auto rx = serial.read(64);

if(rx.empty()) {
    // timeout
}
```

**固定长度读取超时**

```cpp
const auto rx = serial.read_exact(6);

if(rx.size() != 6) {
    // timeout / incomplete
}
```

**发送超时**

```cpp
const auto written = serial.write(packet);

if(written != packet.size()) {
    // timeout / incomplete
}
```

这样 Driver 可以自行决定重试、丢帧或进入故障状态

---

## 12. 生命周期与线程安全

### 12.1. RAII

`SerialPort` 持有 Linux fd

析构时自动关闭

```cpp
{
    SerialPort serial("/dev/ttyUSB0");
    // use serial
}
// 自动 close
```

---

### 12.2. 禁止复制

```cpp
SerialPort(const SerialPort&) = delete;
SerialPort& operator=(const SerialPort&) = delete;
```

防止两个对象同时拥有同一个 fd

---

### 12.3. 支持移动

```cpp
SerialPort a("/dev/ttyUSB0");
SerialPort b = std::move(a);
```

**移动后**

```cpp
a.is_open() == false
b.is_open() == true
```

---

### 12.4. 线程安全

类内部不加锁

以下情况调用方需要自行同步

- 多线程同时修改配置
- 一边关闭串口一边进行 I/O
- 多个线程同时发送协议帧

对于控制器或设备 Driver，更推荐由单一通信线程持有 `SerialPort`

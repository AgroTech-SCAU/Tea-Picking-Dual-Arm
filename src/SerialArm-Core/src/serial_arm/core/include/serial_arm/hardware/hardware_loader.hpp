#pragma once

#include <tl/expected.hpp>

#include "serial_arm/hardware/motor_bus.hpp"

#include <memory>
#include <optional>
#include <string>

namespace serial_arm {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

enum class HardwareLoaderErr {
    OPEN_FAILED,
    SYMBOL_FAILED,
    CREATE_FAILED,
    CONFIGURE_FAILED,
    CONFIG_OPEN_FAILED,
    CONFIG_SYNTAX_ERROR,
    INVALID_OVERRIDE,
};

struct HardwareConfigOverrides {
    std::optional<std::string> serial_port;
    std::optional<int> baudrate;
    std::optional<std::string> bus;
};

class HardwareLoader {
public:
    HardwareLoader() = default;
    ~HardwareLoader();

    HardwareLoader(const HardwareLoader&) = delete;
    HardwareLoader& operator=(const HardwareLoader&) = delete;
    HardwareLoader(HardwareLoader&& other) noexcept;
    HardwareLoader& operator=(HardwareLoader&& other) noexcept;

    tl::expected<std::unique_ptr<MotorBus>, HardwareLoaderErr> load(const std::string& plugin, const std::string& config_path);
    tl::expected<std::unique_ptr<MotorBus>, HardwareLoaderErr> load(
        const std::string& plugin,
        const std::string& config_path,
        const HardwareConfigOverrides& overrides);
};

} // namespace serial_arm

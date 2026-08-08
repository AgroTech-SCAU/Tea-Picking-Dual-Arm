cmake_minimum_required(VERSION 3.8)
project(tea_teleop LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# tea_teleop 位于 src/bringup/tea_teleop
# ../../ 回到 src
set(TEA_SRC_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)

# 通用串口层
add_subdirectory(
    ${TEA_SRC_ROOT}/hardware/hiwonder/serial_port
    ${CMAKE_CURRENT_BINARY_DIR}/serial_port
)

# HX-10HM 驱动
add_subdirectory(
    ${TEA_SRC_ROOT}/hardware/hiwonder/hiwonder_driver
    ${CMAKE_CURRENT_BINARY_DIR}/hiwonder_driver
)

# RM65-B 原生 SDK 封装
add_subdirectory(
    ${TEA_SRC_ROOT}/hardware/rm65b/rm_bringup
    ${CMAKE_CURRENT_BINARY_DIR}/rm_bringup
)

add_executable(
    teleop_test
    src/teleop_test.cpp
)

target_include_directories(
    teleop_test
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(
    teleop_test
    PRIVATE
        hiwonder_driver
        rm_bringup
)

install(
    TARGETS teleop_test
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
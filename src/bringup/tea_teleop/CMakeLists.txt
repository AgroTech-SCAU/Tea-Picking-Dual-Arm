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
find_package(serial_arm_core CONFIG REQUIRED)
find_package(serial_arm_hardware_hiwonder CONFIG REQUIRED)
find_package(tea_leader_description CONFIG REQUIRED)
find_package(Threads REQUIRED)
include(CTest)

set(TEA_SRC_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)

add_subdirectory(
    ${TEA_SRC_ROOT}/hardware/rm65b/rm_bringup
    ${CMAKE_CURRENT_BINARY_DIR}/rm_bringup
)

add_executable(
    teleop
    src/teleop.cpp
    src/leader_runtime.cpp
)

target_include_directories(
    teleop
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(
    teleop
    PRIVATE
        rm_bringup
        serial_arm::robot
        serial_arm::config
        serial_arm::dynamics
        Threads::Threads
)

target_compile_features(teleop PRIVATE cxx_std_17)

set(REALMAN_SDK_LIBRARY
    ${TEA_SRC_ROOT}/hardware/rm65b/rm_driver/lib/libapi_cpp.so
)

set_target_properties(
    teleop
    PROPERTIES
        BUILD_RPATH ${TEA_SRC_ROOT}/hardware/rm65b/rm_driver/lib
        INSTALL_RPATH "$ORIGIN"
)

install(
    TARGETS teleop
    DESTINATION lib/${PROJECT_NAME}
)

install(
    FILES ${REALMAN_SDK_LIBRARY}
    DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
    add_test(
        NAME tea_teleop_leader_config
        COMMAND teleop --check-leader
    )
    set_tests_properties(
        tea_teleop_leader_config
        PROPERTIES
            ENVIRONMENT
                "SERIAL_ARM_RESOURCE_PATH=${TEA_SRC_ROOT}/SerialArm-Core/src/robot_supports/profiles/config/robot_profiles.yaml:${TEA_SRC_ROOT}/SerialArm-Core/src/robot_supports/robots/tea_leader/description:${CMAKE_BINARY_DIR}/../serial_arm_hardware_hiwonder"
    )
endif()

ament_package()

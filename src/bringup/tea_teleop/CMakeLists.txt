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
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(serial_arm_core CONFIG REQUIRED)
find_package(serial_arm_hardware_hiwonder CONFIG REQUIRED)
find_package(tea_leader_description CONFIG REQUIRED)
find_package(Threads REQUIRED)

if(TARGET yaml-cpp::yaml-cpp)
    set(TEA_YAML_CPP_TARGET yaml-cpp::yaml-cpp)
elseif(TARGET yaml-cpp)
    set(TEA_YAML_CPP_TARGET yaml-cpp)
else()
    message(FATAL_ERROR "yaml-cpp was found, but no usable target exists")
endif()
include(CTest)

set(TEA_SRC_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)

add_subdirectory(
    ${TEA_SRC_ROOT}/hardware/rm65b/rm_bringup
    ${CMAKE_CURRENT_BINARY_DIR}/rm_bringup
)

add_library(
    tea_teleop_runtime STATIC
    src/leader_runtime.cpp
)

target_include_directories(
    tea_teleop_runtime
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# ament_target_dependencies() 在 ROS 2 Humble 中会使用 plain signature 调用
# target_link_libraries()，因此这里也统一使用 plain signature，避免 CMake
# 报错“keyword signature 与 plain signature 混用”
target_link_libraries(
    tea_teleop_runtime
    PUBLIC
        serial_arm::robot
        serial_arm::config
        serial_arm::dynamics
        serial_arm_hardware_hiwonder
        Threads::Threads
)

target_compile_features(
    tea_teleop_runtime
    PUBLIC cxx_std_17
)

add_executable(
    teleop
    src/teleop.cpp
)

target_include_directories(
    teleop
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(
    teleop
    tea_teleop_runtime
    rm_bringup
    Threads::Threads
    ${TEA_YAML_CPP_TARGET}
)

ament_target_dependencies(teleop ament_index_cpp)

target_compile_features(teleop PRIVATE cxx_std_17)

add_executable(
    dual_teleop
    src/dual_teleop.cpp
)

target_include_directories(
    dual_teleop
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(
    dual_teleop
    tea_teleop_runtime
    rm_bringup
    Threads::Threads
    ${TEA_YAML_CPP_TARGET}
)

ament_target_dependencies(
    dual_teleop
    ament_index_cpp
)

target_compile_features(
    dual_teleop
    PRIVATE cxx_std_17
)

set(REALMAN_SDK_LIBRARY
    ${TEA_SRC_ROOT}/hardware/rm65b/rm_driver/lib/libapi_cpp.so
)

set_target_properties(
    teleop
    dual_teleop
    PROPERTIES
        BUILD_RPATH ${TEA_SRC_ROOT}/hardware/rm65b/rm_driver/lib
        INSTALL_RPATH "$ORIGIN"
)

install(
    TARGETS
        teleop
        dual_teleop
    DESTINATION lib/${PROJECT_NAME}
)

install(
    FILES ${REALMAN_SDK_LIBRARY}
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
    add_test(
        NAME tea_teleop_leader_config
        COMMAND teleop --check-leader --config ${CMAKE_CURRENT_SOURCE_DIR}/config/teleop.yaml
    )
    set_tests_properties(
        tea_teleop_leader_config
        PROPERTIES
            ENVIRONMENT
                "SERIAL_ARM_RESOURCE_PATH=${TEA_SRC_ROOT}/SerialArm-Core/src/robot_supports/profiles/config/robot_profiles.yaml:${TEA_SRC_ROOT}/SerialArm-Core/src/robot_supports/robots/tea_leader/description:${CMAKE_BINARY_DIR}/../serial_arm_hardware_hiwonder"
    )
endif()

ament_package()

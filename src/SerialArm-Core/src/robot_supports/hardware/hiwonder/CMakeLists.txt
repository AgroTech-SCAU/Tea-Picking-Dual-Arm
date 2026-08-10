cmake_minimum_required(VERSION 3.20)
project(serial_arm_hardware_hiwonder VERSION 0.2.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

find_package(ament_cmake QUIET)
find_package(serial_arm_core CONFIG REQUIRED)
find_package(serial_arm_protocol_hiwonder_bus_servo CONFIG REQUIRED)
find_package(yaml-cpp REQUIRED)

if(TARGET yaml-cpp::yaml-cpp)
    set(SERIAL_ARM_HIWONDER_YAML_CPP_TARGET yaml-cpp::yaml-cpp)
elseif(TARGET yaml-cpp)
    set(SERIAL_ARM_HIWONDER_YAML_CPP_TARGET yaml-cpp)
else()
    message(FATAL_ERROR "yaml-cpp was found, but no usable target exists")
endif()

add_library(serial_arm_hardware_hiwonder SHARED
    src/hx10hm_motor_bus.cpp
    src/plugin.cpp
)

target_include_directories(serial_arm_hardware_hiwonder
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(serial_arm_hardware_hiwonder
    PUBLIC
        serial_arm::core
        serial_arm_protocol_hiwonder_bus_servo
        ${SERIAL_ARM_HIWONDER_YAML_CPP_TARGET}
)

target_compile_options(serial_arm_hardware_hiwonder
    PRIVATE
        $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>
)

install(TARGETS serial_arm_hardware_hiwonder
    EXPORT serial_arm_hardware_hiwonder_targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(FILES package.xml
    DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}
)

configure_package_config_file(
    cmake/serial_arm_hardware_hiwonderConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/serial_arm_hardware_hiwonderConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/serial_arm_hardware_hiwonderConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(EXPORT serial_arm_hardware_hiwonder_targets
    FILE serial_arm_hardware_hiwonder_targets.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/serial_arm_hardware_hiwonderConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/serial_arm_hardware_hiwonderConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

if(ament_cmake_FOUND)
    ament_export_targets(serial_arm_hardware_hiwonder_targets HAS_LIBRARY_TARGET)
    ament_export_include_directories(include)
    ament_export_libraries(serial_arm_hardware_hiwonder)
    ament_export_dependencies(serial_arm_core serial_arm_protocol_hiwonder_bus_servo yaml-cpp)
    ament_package()
endif()

cmake_minimum_required(VERSION 3.16)
project(serial_port VERSION 0.0.1 LANGUAGES CXX)

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

add_library(serial_port INTERFACE)
add_library(serial_port::serial_port ALIAS serial_port)

target_include_directories(serial_port INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_compile_features(serial_port INTERFACE cxx_std_17)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS serial_port EXPORT serial_port_targets)

configure_package_config_file(
    cmake/serial_portConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/serial_portConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/serial_port
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/serial_portConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/serial_portConfig.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/serial_portConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/serial_port
)

install(
    EXPORT serial_port_targets
    NAMESPACE serial_port::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/serial_port
)

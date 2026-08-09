cmake_minimum_required(VERSION 3.20)
project(serial_arm_protocol_hiwonder_bus_servo VERSION 0.2.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

find_package(ament_cmake QUIET)
find_package(serial_arm_core CONFIG REQUIRED)
include(CTest)

add_library(serial_arm_protocol_hiwonder_bus_servo SHARED
    src/bus.cpp
)

target_include_directories(serial_arm_protocol_hiwonder_bus_servo
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(serial_arm_protocol_hiwonder_bus_servo
    PUBLIC
        serial_arm::core
)

target_compile_options(serial_arm_protocol_hiwonder_bus_servo
    PRIVATE
        $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>
)

install(TARGETS serial_arm_protocol_hiwonder_bus_servo
    EXPORT serial_arm_protocol_hiwonder_bus_servo_targets
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
    cmake/serial_arm_protocol_hiwonder_bus_servoConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/serial_arm_protocol_hiwonder_bus_servoConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/serial_arm_protocol_hiwonder_bus_servoConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(EXPORT serial_arm_protocol_hiwonder_bus_servo_targets
    FILE serial_arm_protocol_hiwonder_bus_servo_targets.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/serial_arm_protocol_hiwonder_bus_servoConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/serial_arm_protocol_hiwonder_bus_servoConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(GTest REQUIRED)

    add_executable(serial_arm_protocol_hiwonder_bus_servo_tests
        tests/bus_tests.cpp
    )
    target_link_libraries(serial_arm_protocol_hiwonder_bus_servo_tests
        PRIVATE
            serial_arm_protocol_hiwonder_bus_servo
            GTest::gtest_main
    )
    target_compile_features(serial_arm_protocol_hiwonder_bus_servo_tests PRIVATE cxx_std_17)
    target_compile_options(serial_arm_protocol_hiwonder_bus_servo_tests
        PRIVATE
            $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>
    )

    add_test(
        NAME serial_arm_protocol_hiwonder_bus_servo_tests
        COMMAND serial_arm_protocol_hiwonder_bus_servo_tests
    )
endif()

if(ament_cmake_FOUND)
    ament_export_targets(serial_arm_protocol_hiwonder_bus_servo_targets HAS_LIBRARY_TARGET)
    ament_export_include_directories(include)
    ament_export_libraries(serial_arm_protocol_hiwonder_bus_servo)
    ament_export_dependencies(serial_arm_core)
    ament_package()
endif()

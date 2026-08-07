cmake_minimum_required(VERSION 3.16)
project(hiwonder_driver VERSION 0.0.1 LANGUAGES CXX)

include(GNUInstallDirs)

find_package(serial_port CONFIG REQUIRED)

add_library(hiwonder_driver STATIC src/hx10hm.cpp)

target_include_directories(hiwonder_driver PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(hiwonder_driver PUBLIC serial_port::serial_port)
target_compile_features(hiwonder_driver PUBLIC cxx_std_17)
target_compile_options(hiwonder_driver PRIVATE -Wall -Wextra -Wpedantic)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS hiwonder_driver
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

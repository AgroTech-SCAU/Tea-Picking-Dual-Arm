cmake_minimum_required(VERSION 3.16)
project(rm_bringup LANGUAGES CXX)

set(REALMAN_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../rm_driver)

add_library(rm_bringup STATIC src/rm65b.cpp)
target_include_directories(rm_bringup PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${REALMAN_SDK_DIR}/include/rm_driver
)
target_link_libraries(rm_bringup PUBLIC ${REALMAN_SDK_DIR}/lib/libapi_cpp.so)
target_compile_features(rm_bringup PUBLIC cxx_std_17)
target_compile_options(rm_bringup PRIVATE -Wall -Wextra -Wpedantic)

cmake_minimum_required(VERSION 3.0.0)
project(own_proxy VERSION 0.1.0 LANGUAGES C CXX) 

find_package(MAVSDK REQUIRED)

add_subdirectory(modules/async-sockets-cpp)

add_executable(imu src/imu.cpp)

target_link_libraries( imu async-sockets MAVSDK::mavsdk )
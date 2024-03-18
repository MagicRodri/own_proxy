cmake_minimum_required(VERSION 3.0.0)
project(own_proxy VERSION 0.1.0 LANGUAGES C CXX) 

find_package(MAVSDK REQUIRED)

find_package( OpenCV REQUIRED )
find_package(SFML COMPONENTS system network)
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_subdirectory(modules/PX4-OpticalFlow)

add_subdirectory(modules/async-sockets-cpp)

add_executable(send_flow src/send_flow.cpp)
add_executable(imu src/imu.cpp)
add_executable(main src/main.cpp)
add_executable(client src/client.cpp)

target_link_libraries( main sfml-network)
target_link_libraries( client sfml-network)
target_link_libraries( send_flow OpticalFlow async-sockets MAVSDK::mavsdk ${OpenCV_LIBS} )
target_link_libraries( imu async-sockets MAVSDK::mavsdk)
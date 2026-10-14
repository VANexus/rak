cmake_minimum_required(VERSION 3.8)
project(ipc_bench)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O2 -Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(ipc_bench src/ipc_bench.cpp)
target_link_libraries(ipc_bench ${rclcpp_TARGETS} ${sensor_msgs_TARGETS})
install(TARGETS ipc_bench DESTINATION lib/${PROJECT_NAME})

ament_package()
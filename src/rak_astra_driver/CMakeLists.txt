cmake_minimum_required(VERSION 3.16)
project(rak_astra_driver CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(OrbbecSDK_DIR "/opt/OrbbecSDK_v1.10.18/SDK" CACHE PATH
  "Orbbec SDK 1.10.18 SDK directory (contains include/ and lib/)")
set(OrbbecSDK_LIB_ABS "${OrbbecSDK_DIR}/lib/libOrbbecSDK.so.1.10.18" CACHE FILEPATH
  "Orbbec SDK 1.10.18 shared library")
if(NOT EXISTS "${OrbbecSDK_LIB_ABS}")
  message(FATAL_ERROR "Orbbec SDK library not found: ${OrbbecSDK_LIB_ABS}. Set OrbbecSDK_DIR to the extracted SDK directory.")
endif()
find_path(ORBBecSDK_INC NAMES "libobsensor/ObSensor.hpp"
  PATHS "${OrbbecSDK_DIR}/include" NO_DEFAULT_PATH REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

message(STATUS "Using Orbbec SDK 1.10.18 include: ${ORBBecSDK_INC}")

include_directories(BEFORE ${ORBBecSDK_INC})

# 组件化 (L2, 2026-10-14): 同一份源码同时产出 (a) 可加载组件库 与 (b) 同名独立可执行
# (rclcpp_components_register_node 生成, 现有 launch 里的 astra_node 不变)。
# SDK 动态库挂在组件库上 (容器以 dlopen 加载组件, 必须靠组件自身的 RPATH 找到 libOrbbecSDK)。
add_library(astra_bridge_component SHARED src/astra_bridge_node.cpp)
target_link_libraries(astra_bridge_component ${OrbbecSDK_LIB_ABS})
ament_target_dependencies(astra_bridge_component rclcpp rclcpp_lifecycle rclcpp_components sensor_msgs)
set_target_properties(astra_bridge_component PROPERTIES
  BUILD_RPATH "${OrbbecSDK_DIR}/lib"
  INSTALL_RPATH "${OrbbecSDK_DIR}/lib")

rclcpp_components_register_node(astra_bridge_component
  PLUGIN "AstraBridgeNode"
  EXECUTABLE astra_node)

install(TARGETS astra_bridge_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS astra_node DESTINATION lib/${PROJECT_NAME})
ament_package()

cmake_minimum_required(VERSION 3.16)
project(rak_astra_driver CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(OrbbecSDK_DIR "/opt/OrbbecSDK_v1.10.18/SDK")
set(OrbbecSDK_LIB_ABS "/opt/OrbbecSDK_v1.10.18/SDK/lib/libOrbbecSDK.so.1.10.18")
find_path(ORBBecSDK_INC NAMES "libobsensor/ObSensor.hpp"
  PATHS "${OrbbecSDK_DIR}/include" NO_DEFAULT_PATH REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

message(STATUS "Using Orbbec SDK 1.10.18 include: ${ORBBecSDK_INC}")

include_directories(BEFORE ${ORBBecSDK_INC})

add_executable(astra_node src/astra_bridge_node.cpp)
target_link_libraries(astra_node ${OrbbecSDK_LIB_ABS})
ament_target_dependencies(astra_node rclcpp sensor_msgs)
# RPATH so libOrbbecSDK.so.1.10 is found at runtime without LD_LIBRARY_PATH
set_target_properties(astra_node PROPERTIES
  BUILD_RPATH "${OrbbecSDK_DIR}/lib"
  INSTALL_RPATH "${OrbbecSDK_DIR}/lib")

install(TARGETS astra_node DESTINATION lib/${PROJECT_NAME})
ament_package()

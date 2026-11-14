cmake_minimum_required(VERSION 3.22)
project(rak_chassis)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(canopen_interfaces REQUIRED)

pluginlib_export_plugin_description_file(hardware_interface rak_chassis_plugin.xml)

# [AI] 2026-11-14 薄 CANopen 桥：一个实现、两个 plugin 子类。协议/帧/主站全部
# 在 ros2_canopen 侧，本库只做 OD 对象 <-> ros2_control 接口、量化与安全门。
add_library(rak_chassis_plugin SHARED
  src/canopen_bridge.cpp
)
target_include_directories(rak_chassis_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(rak_chassis_plugin PUBLIC
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  canopen_interfaces::canopen_interfaces
)

install(TARGETS rak_chassis_plugin
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(FILES rak_chassis_plugin.xml DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(hardware_interface pluginlib rclcpp canopen_interfaces)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # [AI] 纯参数/量化单测：假 HardwareInfo，不连 ROS 运行时、不连 CAN。
  ament_add_gtest(canopen_bridge_test test/canopen_bridge_test.cpp)
  if(TARGET canopen_bridge_test)
    target_link_libraries(canopen_bridge_test rak_chassis_plugin)
  endif()

  # [AI] 整车 URDF 必须满足桥的参数契约（lead 维护该脚本）。
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME chassis_urdf_contract
    COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/test/validate_chassis_urdf.py)
endif()

ament_package()

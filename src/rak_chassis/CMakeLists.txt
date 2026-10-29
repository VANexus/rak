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
find_package(rclcpp_lifecycle REQUIRED)
find_package(Threads REQUIRED)

pluginlib_export_plugin_description_file(hardware_interface rak_chassis_plugin.xml)

add_library(rak_chassis_plugin SHARED
  src/can_decode.cpp
  src/can_session.cpp
  src/rak_system.cpp
)
target_include_directories(rak_chassis_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# [AI] lyrical 移除了 ament_target_dependencies，改用 target_link_libraries
target_link_libraries(rak_chassis_plugin PUBLIC
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)
target_link_libraries(rak_chassis_plugin PRIVATE Threads::Threads)

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
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(system_interface_test test/system_interface_test.cpp)
  if(TARGET system_interface_test)
    target_link_libraries(system_interface_test rak_chassis_plugin)
    target_link_libraries(system_interface_test hardware_interface::hardware_interface)
  endif()
  # [AI] CAN v4 帧解码的字节布局回归（纯函数，不需要 socket/ROS 运行时）
  ament_add_gtest(can_frame_decode_test test/can_frame_decode_test.cpp)
  if(TARGET can_frame_decode_test)
    target_link_libraries(can_frame_decode_test rak_chassis_plugin)
  endif()
  # [AI] ADR-0016: Session 接口的唯一实现是 CanSession

  # [AI] 2026-10-15: 整车 URDF 必须仍然满足 RakSystemHardware::on_init 的契约
  # （11 关节 / 固定顺序 / 接口种类 / min-max 非空 / gripper 无状态 / joint 存在于树）。
  # 起因是把机械臂真实几何并入整车 URDF —— 那步大改运动学树，而 on_init 失败是
  # **静默**的（只看到反复 "Waiting for data on 'robot_description'"），真机上极难查。
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME chassis_urdf_contract
    COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/test/validate_chassis_urdf.py)
endif()

ament_package()

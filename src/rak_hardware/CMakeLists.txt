cmake_minimum_required(VERSION 3.22)
project(rak_hardware)

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

pluginlib_export_plugin_description_file(hardware_interface rak_hardware_plugin.xml)

add_library(rak_hardware_plugin SHARED
  src/protocol.cpp
  src/serial_session.cpp
  src/can_session.cpp
  src/rak_system.cpp
)
target_include_directories(rak_hardware_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# [AI] lyrical 移除了 ament_target_dependencies，改用 target_link_libraries
target_link_libraries(rak_hardware_plugin PUBLIC
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)
target_link_libraries(rak_hardware_plugin PRIVATE Threads::Threads)

install(TARGETS rak_hardware_plugin
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(FILES rak_hardware_plugin.xml DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(system_interface_test test/system_interface_test.cpp)
  if(TARGET system_interface_test)
    target_link_libraries(system_interface_test rak_hardware_plugin)
    target_link_libraries(system_interface_test hardware_interface::hardware_interface)
  endif()

  add_executable(pty_peer_test
    test/pty_peer_test.cpp
    src/protocol.cpp
    src/serial_session.cpp
  )
  target_include_directories(pty_peer_test PRIVATE include)
  target_compile_options(pty_peer_test PRIVATE -UNDEBUG)
  target_link_libraries(pty_peer_test PRIVATE Threads::Threads util)
  add_test(NAME pty_peer_test COMMAND pty_peer_test)
  set_tests_properties(pty_peer_test PROPERTIES TIMEOUT 15)
endif()

ament_package()

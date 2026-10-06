cmake_minimum_required(VERSION 3.22)
project(rak_sensor_bringup)
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_depth_camera_launch
    test/test_depth_camera_launch.py
  )
endif()

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
ament_package()

cmake_minimum_required(VERSION 3.22)
project(rak_stereo_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find ROS dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)

# The driver depends on GPU utilities, not the decision message package.
find_package(rak_gpu REQUIRED)

# Find CUDA
find_package(CUDAToolkit REQUIRED)

# nvJPEG: GPU-side MJPEG decode (JetPack 7.2 / CUDA 13.2)
find_package(PkgConfig REQUIRED)
pkg_check_modules(NVJPEG REQUIRED IMPORTED_TARGET nvjpeg-13.2)
# nvjpeg.h lives under the CUDA targets include dir
set(NVJPEG_INCLUDE_DIR /usr/local/cuda-13.2/targets/sbsa-linux/include)

# Capture node (composable component)
add_library(capture_component SHARED
  src/capture_node.cpp
)

target_include_directories(capture_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${NVJPEG_INCLUDE_DIR}
)

ament_target_dependencies(capture_component
  rclcpp
  rclcpp_components
  sensor_msgs
  camera_info_manager
)

target_link_libraries(capture_component
  rak_gpu::gpu
  CUDA::cudart
  CUDA::cuda_driver
  PkgConfig::NVJPEG
)

rclcpp_components_register_node(capture_component
  PLUGIN "rak::stereo_driver::CaptureNode"
  EXECUTABLE stereo_node
)

install(TARGETS capture_component
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

# Calibration belongs to the sensor integration configuration.

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_components sensor_msgs camera_info_manager rak_gpu CUDAToolkit)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

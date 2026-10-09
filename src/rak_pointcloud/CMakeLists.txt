cmake_minimum_required(VERSION 3.22)
project(rak_pointcloud)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rak_gpu REQUIRED)
find_package(CUDAToolkit REQUIRED)

# Only the kernel needs nvcc; the ROS node stays plain C++ so it keeps the warning
# flags above and does not drag device code through the node's translation unit.
enable_language(CUDA)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

add_library(depth_to_xyz STATIC src/depth_to_xyz.cu)
target_include_directories(depth_to_xyz PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(depth_to_xyz PUBLIC CUDA::cudart)

add_executable(gpu_pointcloud_node src/gpu_pointcloud_node.cpp)

ament_target_dependencies(gpu_pointcloud_node
  rclcpp
  sensor_msgs
)

target_link_libraries(gpu_pointcloud_node
  depth_to_xyz
  rak_gpu::gpu
  CUDA::cudart
  CUDA::cuda_driver
)

# Orin is sm_87. Building the default arch list would compile the kernel a dozen
# times for hardware that is not on this board.
set_target_properties(depth_to_xyz PROPERTIES CUDA_ARCHITECTURES 87)

install(TARGETS gpu_pointcloud_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rclcpp sensor_msgs rak_gpu CUDAToolkit)
ament_package()

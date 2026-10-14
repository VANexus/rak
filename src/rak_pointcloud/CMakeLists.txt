cmake_minimum_required(VERSION 3.22)
project(rak_pointcloud)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
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

# [AI] 组件化: 同一份源码同时产出 (a) 可加载组件库 与 (b) 同名独立可执行
# (rclcpp_components_register_node 生成, 现有 launch 里的 gpu_pointcloud_node 不变)。
add_library(gpu_pointcloud_component SHARED src/gpu_pointcloud_node.cpp)
rclcpp_components_register_node(gpu_pointcloud_component
  PLUGIN "rak::pointcloud::GpuPointCloudNode"
  EXECUTABLE gpu_pointcloud_node)

ament_target_dependencies(gpu_pointcloud_component
  rclcpp
  rclcpp_components
  sensor_msgs
)

target_link_libraries(gpu_pointcloud_component
  depth_to_xyz
  rak_gpu::gpu
  CUDA::cudart
  CUDA::cuda_driver
)

# Orin is sm_87. Building the default arch list would compile the kernel a dozen
# times for hardware that is not on this board.
set_target_properties(depth_to_xyz PROPERTIES CUDA_ARCHITECTURES 87)

install(TARGETS gpu_pointcloud_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

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

ament_export_dependencies(rclcpp rclcpp_components sensor_msgs rak_gpu CUDAToolkit)
ament_package()

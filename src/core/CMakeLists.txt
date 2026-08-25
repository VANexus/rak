cmake_minimum_required(VERSION 3.22)
project(core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CanFrame.msg"
  "msg/TrackedObject.msg"
  "msg/TrackedObjects.msg"
  "msg/ModelConfig.msg"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs vision_msgs builtin_interfaces
)

# Find NVIDIA libraries
find_package(CUDAToolkit REQUIRED)

# TensorRT ships headers/libs in the default system paths on JetPack (no .pc files).
find_library(TENSORRT_NVINFER_LIB nvinfer REQUIRED)
find_library(TENSORRT_NVONNXPARSER_LIB nvonnxparser REQUIRED)
set(TENSORRT_LIBS ${TENSORRT_NVINFER_LIB} ${TENSORRT_NVONNXPARSER_LIB})

# VPI ships under /opt/nvidia/vpi4 on JetPack (no .pc file).
set(VPI_ROOT /opt/nvidia/vpi4)
find_library(VPI_LIB nvvpi REQUIRED HINTS ${VPI_ROOT}/lib/aarch64-linux-gnu)
set(VPI_INCLUDE_DIR ${VPI_ROOT}/include)

# Core shared library (headers + cuda/trt/vpi utils)
add_library(core_lib SHARED
  src/cuda.cpp
  src/trt.cpp
  src/vpi.cpp
)

target_include_directories(core_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(core_lib PUBLIC ${VPI_INCLUDE_DIR})

ament_target_dependencies(core_lib rclcpp)
target_link_libraries(core_lib
  CUDA::cudart
  CUDA::cuda_driver
  ${TENSORRT_LIBS}
  ${VPI_LIB}
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(core_lib "${cpp_typesupport_target}")

install(TARGETS core_lib
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp std_msgs sensor_msgs geometry_msgs vision_msgs builtin_interfaces CUDA)

ament_package()

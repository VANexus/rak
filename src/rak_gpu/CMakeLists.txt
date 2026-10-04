cmake_minimum_required(VERSION 3.22)
project(rak_gpu)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)

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

# GPU utilities have no dependency on ROS messages or node implementations.
add_library(gpu SHARED
  src/cuda.cpp
  src/trt.cpp
  src/vpi.cpp
)

target_include_directories(gpu PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(gpu PUBLIC ${VPI_INCLUDE_DIR})

target_link_libraries(gpu PUBLIC
  CUDA::cudart
  CUDA::cuda_driver
  ${TENSORRT_LIBS}
  ${VPI_LIB}
)

install(TARGETS gpu
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(CUDAToolkit)

ament_package()

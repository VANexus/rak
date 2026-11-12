cmake_minimum_required(VERSION 3.16)
project(rak_astra_driver CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- Orbbec SDK 发现 -------------------------------------------------------
# 为什么不再是硬编码的 1.10.18 路径：
#   历史上 SDK 解压在仓库的 tools/camera/（gitignored），CMake 缓存和 ELF 的
#   RUNPATH 都烙死了 /home/xrak/rak/tools/camera/.../SDK —— 换机器、删 tools/
#   或换 SDK 版本，configure 和运行时会**一起**断掉（libOrbbecSDK.so.1.10.18
#   not found）。
#   现在按优先级搜索，把"用哪个 SDK"变成机器事实而不是仓库事实：
#     1. -DOrbbecSDK_DIR=<dir>             显式指定（最高优先级，CI/交叉编译用）
#     2. $ENV{OrbbecSDK_DIR}
#     3. /opt/OrbbecSDK_*/SDK              厂商官方解压位置
#     4. <repo>/tools/camera/**/SDK        本仓历史上的位置（仍支持）
#     5. /opt/ros/<distro>                 apt 包 ros-jazzy-orbbec-camera 自带
#        的 libOrbbecSDK.so.2.x + 头文件（2.9.3 起官方随 apt 发布）
#   版本不匹配时给警告而不是静默：SDK 主版本变了，API 可能有差异。
set(_orbbec_hints "")
if(DEFINED OrbbecSDK_DIR AND OrbbecSDK_DIR)
  list(APPEND _orbbec_hints "${OrbbecSDK_DIR}")
elseif(DEFINED ENV{OrbbecSDK_DIR} AND NOT "$ENV{OrbbecSDK_DIR}" STREQUAL "")
  list(APPEND _orbbec_hints "$ENV{OrbbecSDK_DIR}")
endif()
file(GLOB _orbbec_opt_dirs "/opt/OrbbecSDK_*/SDK")
list(APPEND _orbbec_hints ${_orbbec_opt_dirs})
file(GLOB _orbbec_repo_dirs "${CMAKE_CURRENT_SOURCE_DIR}/../../tools/camera/*/SDK"
                            "${CMAKE_CURRENT_SOURCE_DIR}/../../tools/camera/SDK")
list(APPEND _orbbec_hints ${_orbbec_repo_dirs})
if(DEFINED ENV{ROS_DISTRO} AND NOT "$ENV{ROS_DISTRO}" STREQUAL "")
  list(APPEND _orbbec_hints "/opt/ros/$ENV{ROS_DISTRO}")
endif()

set(OrbbecSDK_LIB_ABS "" CACHE FILEPATH "Orbbec SDK shared library (auto-detected)")
set(ORBBecSDK_INC "" CACHE PATH "Orbbec SDK include dir (auto-detected)")

if(NOT OrbbecSDK_LIB_ABS OR NOT ORBBecSDK_INC)
  foreach(_hint ${_orbbec_hints})
    # 两种布局: <dir>/lib/libOrbbecSDK.so*（SDK 解压）与 <dir>/lib/libOrbbecSDK.so*
    file(GLOB _libs "${_hint}/lib/libOrbbecSDK.so*")
    find_path(_inc NAMES "libobsensor/ObSensor.hpp"
      PATHS "${_hint}/include" "${_hint}/include/OrbbecSDK"
      NO_DEFAULT_PATH)
    if(_libs AND _inc)
      list(GET _libs 0 _lib)
      set(OrbbecSDK_LIB_ABS "${_lib}" CACHE FILEPATH "Orbbec SDK shared library (auto-detected)" FORCE)
      set(ORBBecSDK_INC "${_inc}" CACHE PATH "Orbbec SDK include dir (auto-detected)" FORCE)
      break()
    endif()
  endforeach()
endif()

if(NOT EXISTS "${OrbbecSDK_LIB_ABS}")
  message(FATAL_ERROR
    "找不到 Orbbec SDK。已尝试: ${_orbbec_hints}\n"
    "解决方式（任选）:\n"
    "  * apt install ros-${ROS_DISTRO}-orbbec-camera   （随包提供 libOrbbecSDK.so.2.x）\n"
    "  * 下载 OrbbecSDK 解压后 -DOrbbecSDK_DIR=<dir>\n"
    "  * 本仓历史位置 tools/camera/.../SDK")
endif()
if(NOT ORBBecSDK_INC)
  message(FATAL_ERROR "找到 libOrbbecSDK 但没有头文件 libobsensor/ObSensor.hpp（OrbbecSDK_DIR=${OrbbecSDK_DIR}）")
endif()

# 把实际用到的 SDK 目录交给下面的 RUNPATH —— 运行时靠它找 .so，不能写死
get_filename_component(ORBBEC_SDK_LIBDIR "${OrbbecSDK_LIB_ABS}" DIRECTORY)
get_filename_component(OrbbecSDK_DIR "${ORBBEC_SDK_LIBDIR}" DIRECTORY)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

message(STATUS "Using Orbbec SDK library: ${OrbbecSDK_LIB_ABS}")
message(STATUS "Using Orbbec SDK include: ${ORBBecSDK_INC}")


include_directories(BEFORE ${ORBBecSDK_INC})

# 组件化 (L2, 2026-10-14): 同一份源码同时产出 (a) 可加载组件库 与 (b) 同名独立可执行
# (rclcpp_components_register_node 生成, 现有 launch 里的 astra_node 不变)。
# SDK 动态库挂在组件库上 (容器以 dlopen 加载组件, 必须靠组件自身的 RPATH 找到 libOrbbecSDK)。
add_library(astra_bridge_component SHARED src/astra_bridge_node.cpp)
target_link_libraries(astra_bridge_component ${OrbbecSDK_LIB_ABS})
ament_target_dependencies(astra_bridge_component rclcpp rclcpp_lifecycle rclcpp_components sensor_msgs)
set_target_properties(astra_bridge_component PROPERTIES
  BUILD_RPATH "${ORBBEC_SDK_LIBDIR}"
  INSTALL_RPATH "${ORBBEC_SDK_LIBDIR}")

rclcpp_components_register_node(astra_bridge_component
  PLUGIN "AstraBridgeNode"
  EXECUTABLE astra_node)

install(TARGETS astra_bridge_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS astra_node DESTINATION lib/${PROJECT_NAME})
ament_package()

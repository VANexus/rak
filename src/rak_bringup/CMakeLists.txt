cmake_minimum_required(VERSION 3.22)
project(rak_bringup)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_depth_camera_launch
    test/test_depth_camera_launch.py
  )
  # ADR-0020：清单与 launch 的一致性门禁已随清单迁到 rak_orchestration
  # （那是纯 x86 可建的契约层，桌面档也要能跑这条门禁）。
  # ADR-0002：EKF 配置自洽性（离线可跑，不依赖 ROS）
  ament_add_pytest_test(
    test_ekf_contract
    test/test_ekf_contract.py
  )

  # 2026-11-18: default `tts:=true` launch died on an undeclared
  # LaunchConfiguration after the 92b5a8b merge dropped nine declarations.
  ament_add_pytest_test(
    test_speech_launch_args
    test/test_speech_launch_args.py
  )

  # 2026-11-18: stage:=full included speech.launch.py twice (sensors owns it).
  ament_add_pytest_test(
    test_robot_launch_composition
    test/test_robot_launch_composition.py
  )

  # 2026-11-19: 纯视觉寻路的 RotationShim 覆盖层必须逐字镜像 nav2_params.yaml 的
  # MPPI 半边（旋转闸门按同名节点把参数交给副驾，没有独立命名空间）。
  ament_add_pytest_test(
    test_vision_follow_params
    test/test_vision_follow_params.py
  )
endif()

install(DIRECTORY launch config scripts udev DESTINATION share/${PROJECT_NAME})

# [AI] 2026-11-18: DDS 网卡是**机器相关**的。config/cyclonedds.xml 里写死
# NetworkInterface name（开发机 wlp0s20f3 / orin wlP1p1s0），名字在本机不存在时
# Cyclone 拒绝创建 participant，每个 ROS 进程都 "failed to create domain" 后 abort
# ——整栈起不来，看着像硬件故障。这里按**本机**检测到的局域网口替换后重装一份：
# 有全局 IPv4、且不是 docker/网桥/veth/netmaker/WireGuard/USB gadget 的第一个接口。
# 检测不到就原样安装（宁可退回 Cyclone autodetermine，也不要写坏）。
execute_process(
  COMMAND sh -c "ip -4 -o addr show scope global 2>/dev/null | awk '$2 !~ /^(docker|br-|veth|netmaker|l4tbr|usb)/ {print $2; exit}'"
  OUTPUT_VARIABLE RAK_DDS_IFACE OUTPUT_STRIP_TRAILING_WHITESPACE)
set(RAK_DDS_XML_SRC "${CMAKE_CURRENT_SOURCE_DIR}/config/cyclonedds.xml")
set(RAK_DDS_XML_OUT "${CMAKE_CURRENT_BINARY_DIR}/cyclonedds.xml")
file(READ "${RAK_DDS_XML_SRC}" RAK_DDS_XML_TEXT)
set(RAK_DDS_XML_TAG "<NetworkInterface name=\"")
string(FIND "${RAK_DDS_XML_TEXT}" "${RAK_DDS_XML_TAG}" _rak_dds_tag_at)
if(RAK_DDS_IFACE AND NOT _rak_dds_tag_at EQUAL -1)
  # 只替换 NetworkInterface 行的 name 值，注释与其它配置一字不动。
  string(REGEX REPLACE
    "<NetworkInterface name=\"[^\"]*\"" "<NetworkInterface name=\"${RAK_DDS_IFACE}\""
    RAK_DDS_XML_TEXT "${RAK_DDS_XML_TEXT}")
  message(STATUS "rak_bringup: DDS NetworkInterface -> ${RAK_DDS_IFACE}")
else()
  message(STATUS "rak_bringup: 未检测到局域网口，cyclonedds.xml 原样安装")
endif()
file(WRITE "${RAK_DDS_XML_OUT}" "${RAK_DDS_XML_TEXT}")
install(FILES "${RAK_DDS_XML_OUT}" DESTINATION share/${PROJECT_NAME}/config)

# [AI] 2026-10-14: CYCLONEDDS_URI 统一接线（docs/perf/ipc_baseline.md §9）。
# overlay source 时自动 set-if-unset 团队 CycloneDDS 配置——必须在 ament_package() 之前。
ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/rak_cyclonedds.sh"
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/rak_cyclonedds.dsv.in"
)

ament_package()

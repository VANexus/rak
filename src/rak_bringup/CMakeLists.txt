cmake_minimum_required(VERSION 3.22)
project(rak_bringup)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_depth_camera_launch
    test/test_depth_camera_launch.py
  )
  # ADR-0020：清单与 launch 的一致性门禁（需要 install 空间，故走 colcon test）
  ament_add_pytest_test(
    test_system_manifest
    test/test_system_manifest.py
  )
  # ADR-0002：EKF 配置自洽性（离线可跑，不依赖 ROS）
  ament_add_pytest_test(
    test_ekf_contract
    test/test_ekf_contract.py
  )
endif()

install(DIRECTORY launch config scripts udev DESTINATION share/${PROJECT_NAME})

# [AI] 2026-10-14: CYCLONEDDS_URI 统一接线（docs/perf/ipc_baseline.md §9）。
# overlay source 时自动 set-if-unset 团队 CycloneDDS 配置——必须在 ament_package() 之前。
ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/rak_cyclonedds.sh"
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/rak_cyclonedds.dsv.in"
)

ament_package()

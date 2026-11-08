cmake_minimum_required(VERSION 3.22)
project(rak_orchestration)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# 纯 Python 模块：清单加载、档位解析、launch 助手。不依赖任何硬件/GPU 包 ——
# 这是"桌面档一条命令就能建起来"的结构性前提（docs/adr/0026）。
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # 清单不变量 + 清单↔launch 一致性（ADR-0020 的可执行门禁）
  ament_add_pytest_test(test_system_manifest test/test_system_manifest.py)
  # 档位与 launch 契约：桌面档跳过哪些 launch 必须是**声明**，不能靠运行时嗅探
  ament_add_pytest_test(test_launch_contracts test/test_launch_contracts.py)
endif()

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()

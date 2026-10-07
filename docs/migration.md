# 统一包迁移说明

本次采用统一迁移，不提供旧包兼容别名。算法、话题名、节点默认名、
QoS、消息字段和 `brain/rak_ai/types.py` 的序列化格式保持不变。

## 包与 API 映射

| 旧入口 | 新入口 |
|---|---|
| `core/msg/*` / `from core.msg` | `rak_interfaces/msg/*` / `from rak_interfaces.msg` |
| `core::core_lib` | CMake 导出目标 `rak_gpu::gpu` |
| `<core/*.hpp>` / `rak::core` | `<rak_gpu/*.hpp>` / `rak::gpu` |
| `eye` / `eye_capture` | `rak_stereo_driver` / `stereo_node` |
| `<eye/capture_node.hpp>` / `rak::eye::CaptureNode` | `<rak_stereo_driver/capture_node.hpp>` / `rak::stereo_driver::CaptureNode` |
| `astra_bridge` / `astra_bridge_node` | `rak_astra_driver` / `astra_node` |
| `motor_stub` / Python `motor_stub` | `rak_motion_stub` / Python `rak_motion_stub` |
| `brain` / `brain_node` | `rak_agent` / `agent_node`；源码仍在 `brain/` |
| `boot lidar.launch.py` | `rak_sensor_bringup lidar.launch.py` |
| `boot slam.launch.py` / `nav2.launch.py` | `rak_navigation` 下同名 launch |
| `boot demo.launch.py` | `rak_moveit_config demo.launch.py` |
| `boot view_urdf.launch.py` / `display.launch.py` | `rak_description` 下同名 launch |
| `boot brain.launch.py` | `rak_bringup agent.launch.py` |
| `boot/config/cyclonedds.xml` | `rak_bringup/config/cyclonedds.xml` |
| `boot/urdf/rak.srdf` | `rak_moveit_config/config/rak.srdf` |

新增相机入口：`rak_sensor_bringup astra.launch.py` 和 `stereo.launch.py`。
`cmd_vel_stub` 可执行文件及 `/cmd_vel_stub/enable` 服务名不变；
旧文档中的 `/motor_stub/enable` 与实际私有服务解析不符，本次修正文档。

ROS 消息的包名前缀是类型身份的一部分。即使字段相同，
`core/msg/Decision` 与 `rak_interfaces/msg/Decision` 也不能直接互通。
外部节点必须更新依赖、重新构建；旧 rosbag 需要显式类型迁移或旧环境回放，
不能假设新节点能直接订阅旧类型。JSONL episode 数据没有改动。

## 2026-10-04 软件任务与硬件适配增量

统一重命名之后新增了以下边界；这些增量没有修改冻结的
`brain/rak_ai/types.py`、episode `state_text` 或第三方 vendor 包：

| 新包/接口 | 当前职责与边界 |
|---|---|
| `rak_task_executor` | `/task/execute` Action、`/task/control` Service、`/task/status`；唯一拥有导航任务与 Nav2 goal 生命周期 |
| `rak_base_driver` | Jetson 到底盘 F407 的 UART v1 host codec/节点；节点默认无端口、禁用硬件、速度限值为零、DISARMED |
| `rak_arm_simulation` | 六轴夹具使用的软件轨迹/夹爪 Action server；不含 physics 或 ros2_control 硬件接口 |
| `rak_interfaces` 新类型 | `ExecuteTask`、`TaskControl`、`TaskStatus`、`BaseTelemetry`；是加法接口，不取代旧决策消息 |

文字任务默认使用本地解析与配置的 waypoint allow-list；可选 RAI 慢规划只能选
白名单别名。暂停需要 Nav2 取消确认及新鲜静止遥测；恢复重新提交地图目标。
`task_simulation.launch.py` 使用 fake Nav2 与命令运动估算，不表示真实底盘反馈。

Nav2 新链为 Omni MPPI → velocity smoother → collision monitor → 最终 `/cmd_vel`
适配器。当前工作区的 `task_navigation.launch.py` 还增加了 `hardware` 末端选项，
在顶层 bringup 连接 UART 主机节点；launch 默认仍为 `sim`，硬件开关默认关闭。
该路径已通过安装态 launch 解析和模式映射检查，但没有真实设备运行验证。
UART v1 的主机格式见 [协议文档](protocol/jetson_f407_uart_v1.md)；仓库没有匹配的
F407 固件，因此规范、codec 与 PTY 验证不构成实机兼容声明。

`rak_description` 中新增的六轴模型、MoveIt SRDF 和限位仅为测试 fixture，独立于
原五轴 demo。不得将 fixture 尺寸、限位或动作服务器用于真实机械臂。

## 构建与环境

旧 `build/`、`install/` 不删除。新的输出使用：

```text
build/architecture/
install/architecture/
```

完整命令见 README。必须从只加载 `/opt/ros/jazzy` 的环境构建，
否则 colcon 生成的 setup 文件可能记录旧 overlay。既有 shell 已加载旧安装时，
可在仓库根目录显式清除继承的包搜索路径：

```bash
env -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH \
  -u PYTHONPATH -u LD_LIBRARY_PATH -u ROS_PACKAGE_PATH \
  CMAKE_BUILD_PARALLEL_LEVEL=2 bash --noprofile --norc -c \
  'source /opt/ros/jazzy/setup.bash && colcon build --base-paths src brain \
    --build-base build/architecture --install-base install/architecture \
    --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release'
```

运行时先加载 ROS，再加载 `install/architecture/local_setup.bash`。
不要继续加载旧 `install/setup.bash`；本次不自动修改个人 `.bashrc`。
当前 IDE 的包搜索路径同步到新安装目录，IDE 配置仍为本机未跟踪文件。

## Python 与操作脚本

- `brain/.venv`、`brain/data`、pip 项目名 `rak-brain` 和 `rak_ai` 导入不变。
- ROS 引导脚本更名为 `agent_node`，支持在仓库外启动及嵌套安装目录；
  `RAK_BRAIN_PYTHON` 仍可指定解释器。
- Laya/RAI 仍是可选依赖，stub 不需要 torch。
- `agent.launch.py` 现在显式透传 `s1_weights`，默认仍为 `auto`。
- 操作脚本迁移至 `src/rak_bringup/scripts/`，共享 `environment.sh`。
- `RAK_WORKSPACE`、`RAK_INSTALL_PREFIX`、DDS 配置与 ROS domain 可覆盖；
  默认安装目录是 `install/architecture`，不再手工补 Python 包前缀。
- 无 GUI 的雷达、SLAM、Nav2 脚本不再强制设置 DISPLAY；
  RViz/MoveIt demo 才检查可读的 Xauthority 文件。
- 不要单独复制一个 `run_*.sh` 到 `/tmp`，它现在需要同目录的 `environment.sh`。

## 检查与限制

`scripts/check_workspace.py` 校验包命名、构建类型、资源标记、依赖图和
launch 的显式运行依赖。`--installed` 还校验安装资源与源码一致，
逐个解析 launch、序列化消息并从仓库外验证智能体引导入口。
launch 检查使用 `--show-args`，不会打开设备或发出运动命令。

描述与 MoveIt 的旧结构测试接入 CTest，缺少资源时失败而不是静默跳过。
完整回归仍包括 `brain/tests/test_ai.py`。

不变项：第三方代码、固件协议、模型推理策略、真实运动行为、模型权重、episode。
现存安全/模型缺口见 [架构文档](architecture.md)，不能把迁移测试当作硬件验收。

## 统一迁移验证记录（2026-10-04）

- 本机 aarch64 全工作区构建：14 个包完成，命令退出码 0。
- 结构检查：11 个自有包，依赖图无环，显式 launch 依赖已声明。
- 安装态：9 个 launch 可解析，资源路径与源码一致，四种消息序列化往返通过。
- ROS 模拟链路：独立测试 domain 上验证至少三次决策输出，不调用真实运动接口。
- CTest：3 项测试通过，无失败、无跳过；现有 `brain/tests/test_ai.py` 通过。
- 新可执行入口和 `rak::stereo_driver::CaptureNode` 组件注册检查通过。
- 消息字段、冻结的 AI 类型/策略、Nav2 参数与第三方源码已核对未变。

受限环境中构建设置 `PYTHONDONTWRITEBYTECODE=1`，避免向系统 ROS 目录写缓存；
安装检查日志使用临时目录。该设置会产生 setuptools 的禁用字节码告警。
首次编译仍显示既有 C++ / 第三方告警，本次没有把它们当作迁移问题修改。
未运行真实传感器、底盘、机械臂运动测试，也未重跑 GPU 模型精度或性能测试。

## 任务与硬件适配增量验证（2026-10-04）

- 全工作区增量构建：17 个包完成；其中 14 个自有包，3 个 vendor 包。
- 安装态 checker：14 个自有包依赖无环，13 个 launch 可解析，新旧接口序列化通过，
  并从安装 overlay 导入 task intent、UART、task executor 和 arm simulation 模块。
- CTest 与 Python ROS 包测试：21 项通过，0 errors / failures / skipped。
- `brain/tests/test_ai.py` 和 `brain/tests/test_task_intent.py` 通过。
- UART driver 的 ROS 节点直接通过 PTY peer 验证 handshake、DISARMED、明确 ARM、
  速度缩放和过期输入撤销；独立 PTY 工具 100 Hz 发送 DISARMED 零速度。
- fake Nav2 测试覆盖导航成功、重复 request ID、暂停确认、恢复重规划、服务取消和
  Action 协议取消；fake arm action client 验证六关节轨迹与独立夹爪。
- 遗留 CRC-8 物理串口工具拒绝执行，避免发送与 UART v1 不兼容的运动帧。

这些都是软件/仿真结果；未连接 F407、雷达、相机或真实机械臂，未进行物理运动、
E-stop、碰撞停止距离或固件互操作验收。

本次收尾重新构建 17 个包，安装态 checker 检查 14 个自有包和 13 个 launch；
CTest 21 项、任务/机械臂 pytest 4 项、UART pytest 13 项以及两组 brain 测试通过。
硬件 backend 的安装态解析和模式映射通过，但未启动导航运行时或连接设备。
最新能力和未关闭的验收项见[项目总体状态](project_status.md)。

# 项目总体状态

更新日期：2026-10-14

## 2026-10-14 增量：删除 USB v2 链路（ADR-0016）

- **传输**：CAN v4（SocketCAN `can0`）成为唯一传输；`transport`/`device` 参数与
  `SerialSession`、`rak_base_driver`、`rak_serial_link`、直连硬件的示教/驾驶 TUI、
  6 个串口脚本与 `docs/protocol/` 全部删除（删除前快照：git tag `pre-usb-removal-20261013`）。
- **入口**：`task_navigation.launch.py` 的 `motion_backend` 只剩 `sim|stub`
  （`hardware` 选项删除）；真底盘经 `chassis_bringup.launch.py` →
  `mecanum_drive_controller` → `rak_chassis` → `can0`（`allow_motion` 默认 false）。
  基础层解除服务由 `/base_uart_driver/arm` 改名 `/base/arm`。
- **工具原则**：工具只经 topic/action/service 与驱动节点对话，禁止直连硬件。
- **链路验收（2026-10-13）**：H/L 对调后 `scripts/can_link_smoke.py can0` 连续
  10/10 通过（BAT≈12500 mV、FAULT=0、BUSOFF=0）。

## 2026-10-11 增量：全机坐标系分层（ADR-0015）

- **坐标系**：确立全机 TF 树 `world → map → odom → base_footprint → base_link → {4 轮, laser_link, mic_array_link, camera_link, imu_link, arm_base_link}`；采用 REP-103 + REP-120，`world` 作无限开放世界的无界全局根（现为静态恒等占位）。
- **几何**：车体实测轮距 27 cm / 轴距 25 cm 入 `rak_chassis.urdf.xacro`；雷达（前·中轴）、6 阵列麦克风（雷达上方）、深度相机（车头最前）、机械臂（后·中轴）挂载帧同处定义，高度/前后距离以 `TODO(measured)` 占位。
- **发布者收敛**：静态几何单一来源交给 `robot_state_publisher`；`slam_toolbox` 独占 `map→odom`，EKF 独占 `odom→base_footprint`；移除 launch 里 `base_link→laser_link`/`imu_link` 的临时静态 TF。
- **本机验证边界**：x86/IDE 侧已过 `xacro` 解析与 `check_workspace.py` 结构检查；`colcon build` 与上车 TF 复测在 Jetson 收口（见 TODO P1-6）。

## 2026-10-09 增量：CAN v4 主链路 + 工作空间重组（ADR-0012 / ADR-0013）

- **传输层**：合入上游 `Session` 抽象 + `CanSession`（SocketCAN CAN v4 主）
  后端；协议事实源升到下位仓 v4。取代 ADR-0007 的
  "USB 唯一链路"（**USB 备用链路部分已按 ADR-0016 于 2026-10-14 删除，见上**）。
- **工作空间**：按子系统归堆 23 → 18 构建包：`rak_hardware`→`rak_chassis`、新建
  `rak_commissioning`（双 TUI + chassis_key_test 转正 entry point）与 `rak_intent`
  （task_intent 自 brain 抽出）；`rak_bringup` 吸收 sensor_bringup/navigation/
  chassis_bringup 全部 launch/config；`brain/` 迁入 `src/rak_agent`（模块 `rak_ai`→
  `rak_agent`，语料迁仓库根 `data/episodes/`）；CAN v3 栈归档 `attic/`（2026-10-10 本地
  归档已删，仅存 git 历史）。
- **规范守护**：`check_workspace.py` 重写（EXPECTED 18 包、依赖倒置守卫
  `dependents(rak_agent) ⊆ {rak_bringup}`、launch 引用检查），本机结构检查全绿；
  并由此暴露并修复历史欠账 `nav2_lifecycle_manager` 未声明。
- **文档**：ADR-0012/0013、索引、README/architecture/AGENTS/CLAUDE/CONSTRUCTION/
  migration（新映射表）/TODO 同步完成。
- **本机验证边界**：x86 开发机（无 Jazzy/JetPack）——已跑通纯 Python 测试
  （`test_ai.py`、`test_task_intent.py`、模块导入与语料路径断言）；
  `colcon build/test` 与 `check_workspace --installed` 需在 Jetson 上首次收口
  （先 `rm -rf build/architecture install/architecture`）。

## 结论

项目已经从包重命名和依赖整理，推进到一个可运行的软件任务闭环：可以用文字提交已配置地点任务，在 fake Nav2 上验证导航成功、暂停、恢复和取消；CAN v4 链路已通过冒烟验收（2026-10-13），六轴机械臂动作夹具也有软件测试。

当前交付仍是**软件原型，不是可直接上车运行的整机系统**。真实传感器 Nav2 组合、collision monitor 停车效果、F407 固件互操作、物理急停和机械臂硬件执行都没有完成验收。Jev 与任务执行器之间的共享任务仲裁**已在 ADR-0010 中定义**（新包 `src/rak_harness` + 传递式提案缝），但 Harness 本体尚未实现，RAI 慢路径仍不是 Harness 持续任务系统。

## 能力状态

| 领域 | 当前状态 | 尚未证明 |
|---|---|---|
| ROS 包架构 | 统一 `rak_` 包迁移完成；依赖边界和安装态检查已覆盖 | 后续包边界变化仍需按架构文档评审 |
| 文字导航任务 | 本地解析、配置地点白名单、任务 Action/状态、fake Nav2 成功/暂停/恢复/取消已实现 | 到达与暂停竞态、各类超时、迟到慢规划结果、重启策略仍需补测 |
| RAI 慢规划 | 可选路径只能选配置的 waypoint 别名 | 不等同于 Laya/Jev System 1，也没有 Harness 的持续任务调度（ADR-0010 已定义缝，未实现） |
| Nav2 | Omni MPPI、速度平滑器、collision monitor 和末端适配器配置已接入 | 真实传感器/定位下的 lifecycle、碰撞停止行为、速度链单写者尚未端到端验证 |
| 底盘链路 | CAN v4（SocketCAN）冒烟验收通过（2026-10-13：`can_link_smoke.py can0` 10/10，BAT≈12500 mV / FAULT=0 / BUSOFF=0）；`rak_chassis` 导出 4 轮接口，底盘冒烟过（固件 R37） | 真实 encoder odom、E-stop、时序与整车验收未完成；CAN 侧 7 轴位置接口未实现（ADR-0012/0016 待闭合项） |
| 硬件启动路径 | 真底盘经 `chassis_bringup.launch.py` → `mecanum_drive_controller` → `rak_chassis` → CAN；`task_navigation.launch.py` 的 `motion_backend` 只有 `sim\|stub` | 尚未在真实传感器或 F407 上完成整车运行验收；`allow_motion` 默认 false 拒绝运动 |
| 机械臂 | 六关节加独立夹爪的软件动作夹具和 MoveIt 配置已实现 | 模型、限位是测试占位；F407 仅有六路 PWM、无关节反馈，夹爪通道和硬件控制器未完成 |
| 感知与语音 | 雷达/深度相机已有驱动或测试基础；语音链已通（2026-10-10：XFM-DP → `audio_capture` → `/audio` → Vosk 转写/意图；2026-10-11：板端唤醒/方位 `vtn_node` → `/speech/wake`、离线 TTS `tts_node` → `/audio_out` → `audio_play` → USB 喇叭，电气链路与真人唤醒均已实测）；当前仍以文字入口验证任务 | 传感器融合和完整导航输入链未验收；阵列声学段需真人说话验收；识别解码 0.84–1.01× 实时，余量不足；TTS 合成约 0.75× 实时（kokoro 24 kHz），播报音色/音量待真人验收；`/speech/say` 尚无决策层消费者 |
| Jev / Harness | `rak_agent` 保留现有 S1/S2 原型与冻结的数据契约；TensorRT decider 已接入；`src/rak_harness` + `harness_node` 已实现（2026-10-15 软件闭环）：`/harness/request|proposal|event` 契约、`slow_planner:=harness`、唤醒门控语音对话环（提交任务/pause-resume-cancel/播报结果）、人格卡+车况快照、云端 client stub（无 key 时本地保底），fake-Nav2 隔离域验证通过 | 云端通路未实测（无 `ANTHROPIC_API_KEY`）；S2 回复仍不驱动执行器；持续方位流、打断、流式 TTS 未做；表达层仍是门控 no-op stub |

## 已有验证证据

本次收尾验证结果：

- 全工作区构建 17 个包完成；安装态 checker 检查 14 个自有包、依赖图和 13 个 launch。
- CTest 21 项通过；直接 Python ROS 集成测试中，任务/机械臂 4 项通过。
- `src/rak_agent/test/test_ai.py` 与 `src/rak_intent/test/test_task_intent.py` 通过。
- fake Nav2 覆盖导航成功、重复 request ID、暂停确认、恢复重规划、Service 取消和 Action 取消。
- 六轴动作测试覆盖六关节轨迹与独立夹爪 Action。

以上都是软件构建、静态 launch 检查和仿真结果，不代表真实传感器、F407 固件、
底盘运动、物理急停或碰撞停止距离通过验收。

## 硬件门控现状

- `chassis_bringup.launch.py` 的 `allow_motion` 默认 **false**：硬件只握手+遥测、
  只发零目标（DISARMED）；`can_channel` 默认 `can0`，接口身份须由硬件负责人确认。
- 控制器命令限幅由 `controller_manager` 强制（`enforce_command_limits: true`）；
  轮几何与限速当前是占位值，实测替换前不得提速。
- `task_navigation.launch.py` 的 `motion_backend` 只有 `sim|stub`，不证明 F407 固件
  兼容或提供安全保证；真底盘只能经 `chassis_bringup.launch.py` 走 CAN（ADR-0016）。
- 物理底盘保持锁定，直到完成 [硬件验收门槛](hardware_acceptance.md)；六轴手臂继续只用软件夹具。

## 建议的下一步顺序

1. 在非硬件后端验证传感器/Nav2 组合：确认 lifecycle、定位输入、collision monitor 输出和
   `/cmd_vel` 单写者；不能用 fake 遥测替代真实反馈验收。
2. 补足任务状态机的暂停-到达竞态、goal response/任务超时、慢规划迟到回复和重启测试。
3. 完成 CAN 侧 7 轴位置接口 + arm controller + 夹爪轴（ADR-0012/0016 待闭合项），
   再按硬件验收文档逐阶段测试。
4. 获得机械臂真实几何、关节限位、反馈与 PWM 通道标定后，再解锁真实 arm driver；届时再连接任务层与
   Jev/Harness 仲裁，并规划语音接入。
5. 按 ADR-0010 实现 `src/rak_harness`（软件可仿真，不依赖硬件门）：先落 `harness_node` +
   契约 + `NoOpExpressionSink`，再实测 DeepSeek Anthropic 端点与 `anthropic` SDK 安装，
   最后补 `/harness/proposal` 的陈旧/过期/预算竞态测试。

在这些门槛完成前，推荐使用 task simulation 和 CAN 链路冒烟（`scripts/can_link_smoke.py`）做软件演示；不要执行真实底盘或机械臂动作。

# 项目总体状态

更新日期：2026-10-29

## 2026-10-29 增量：运行期编排与受管生命周期（ADR-0020）+ v5 事实核对（ADR-0021）

**先说影响最大的一条**：下位机已按 ADR-0021 切到 **v5 = CANopen**（在板 R73/A14），而本仓
`src/rak_chassis` 的 `CanSession`/`can_decode` 仍说 v4 帧 → **真机链路当前不可用**，
台架只能用伞仓 `tools/canopen_bringup.py` 验下位机；生产主站是待办 **P3（lely-core）**。
本轮**没有**做链路迁移，只做了运行期编排层。

新增 `src/rak_lifecycle`（基础设施包，不依赖任何自有包）+ `rak_bringup` 编排接线：

- 清单 = 故障域/组件/所有权/判据的唯一来源；`system_manager` 按依赖闭包拉起、稳态按新鲜度降级，
  **只降级自己拥有的组件**；`rak_system status|start|stop|recover` 是操作员入口。
- 真机清单 + **软件夹具清单**两套（`profile:=fixture` 不碰硬件，专用于验证编排本身）。
- 转换了两个自有节点：`rak_task_executor`（未激活没有 action server；配置失败从崩进程变成
  可读 FAULTED）与 `rak_wbc`（configure=复合 URDF+HQP，activate=100 Hz 环；停用不补发零速，
  停车靠 `mecanum_drive_controller.reference_timeout` 与臂 lease）。
- 阈值有出处：`/imu/data_raw` 903.6 Hz、`/odometry/filtered` 30.0 Hz、`/tf` 83.3 Hz
  是 2026-10-29 在本机 `ros2 topic hz` 实测（清单里标 `basis: measured`）；雷达 10 Hz 仍标
  `nominal`（现场那套栈 `lidar:=false`，没测到）。

**实测过的**（Orin 独立 worktree `~/rak-lifecycle`，`ROS_DOMAIN_ID=77`，全程未触碰硬件与
现场那棵有未提交在制品的工作树）：23 包 `colcon build` 成功；`rak_lifecycle` **27/27**、
`rak_task_executor` **4/4**；9 个新增/修改 launch `--show-args` 全绿；结构门禁
`check_workspace.py`（非 installed 模式）23 包通过。

**未跑/未验证**（本机→Orin 链路中断：`orin.xrak` 不解析、100.121.213.20 无路由）：
全量 `check_workspace.py --installed`、`colcon test --packages-select rak_bringup`
（清单↔launch 一致性门禁）、`profile:=fixture` 的实机编排演示（拉起/降级/单域停用）。
这三项跑绿之前，编排层**不得称"已验证"**。

**已知先存在性问题**（不是本轮引入，已在未改动的现场树上复现同样失败）：
`rak_wbc/test/test_hqp_numerical.py` 的 `test_support_cbf_locks_com_when_margin_consumed` 与
`test_solver_matches_true_optimum_across_objective_change` 2 项失败。

## 2026-10-29 续：CANopen v5 的 Jetson 侧契约层（ADR-0022，离线可验证部分已落地）

新包 `src/rak_canopen`（基础设施，不依赖任何自有包；24 包结构门禁已过）：

- **不重写协议**：生产主站是 lely-core（ADR-0021 已选）。本层把下位机 SSOT
  （`config/chassis.json` / `arm.json`，副本 + sha256 pin）派生成可执行事实：
  COB-ID、PDO 字节布局与 8 字节上限、偏移、速率（由 `tpdo_types × SYNC 2 ms` 算出：
  底盘 100 Hz，臂 50/100/10 Hz）、量化范围与越界行为、build_tag SDO 解码、命令活性三条件。
- **单位/因子单独一张表且每条带出处**（SSOT 里没有这些信息——这正是 v4 量错 10 倍的土壤）；
  语义名与 JSON 双向闭合，多/少/拼错都装载失败。
- **"禁止同总线混跑 v4/v5" 现在是算术**：v4 的 ID 从本仓 v4 后端源码正则读出，与派生 v5 ID 求交集，
  实测冲突 **0x201 / 0x202**（与规范件的说法对上）。
- **EKF 第一次有显式协方差**（此前 `grep -c covariance` = 0）+ `print_diagnostics: true`；
  协方差按 `*_config` 融合位生成并被测试钉住。过程中抓到两处真错：PyYAML 把 `1e6`/`1e+06`
  读成**字符串**、以及 `imu0_config` 开 vyaw 而方差却写在 yaw 位。
- 已知不自洽**未擅自改**、记录待裁定：`odom0` 位姿与速度同开 vs ADR-0002 约束 #3；
  `i2c_imu.py:240` 用发布时刻而非采样时刻（他人在制品，本轮不动）。
- 实测：`rak_canopen` 契约测试 **16/16**、`rak_bringup` EKF 契约 **7/7**（本机离线，
  无需 ROS/硬件/Orin）。**字节序 `little` 仍是推断**，明日首帧必须核对；
  实机项见 ADR-0022「明日实机门」。

下一步顺序（按用户定案）：先把本层收尾合入 → 再做 P3 CANopen 主站（届时 `link` 域整段重声明，
拥有者改为 v5 主站自带的网络管理，遵守 ADR-0020 §3 单一所有者）。

---

## 历史（2026-10-15 及更早）

## 2026-10-15 增量：机械臂几何并入整车 URDF + MoveIt 真机档

- **几何单点化**：臂的运动链（7 link + 6 joint）抽到
  `src/rak_description/urdf/macros/arm_chain.xacro` 的 `<xacro:rak_arm_chain/>` 宏，
  由 `rak_arm.urdf.xacro`（MoveIt 用）与 `rak_chassis.urdf.xacro`（真机 CM 用）
  **共用**。此前两边各写一份：整车那份是零长度控制桩，没有几何。
- **RSP 现在会发臂的 TF**：ADR-0015 待闭合项 #4 随之闭合。本机实测
  `tf2` 查 `arm_base_link→tool0` = (0.10, 0, 0.33)（Home 位）。
  → RViz / 任何外部消费者这才**看得到**机械臂（rviz 的 RobotModel 是逐 link 查 TF 的）。
- **契约不变**：`<ros2_control>` 仍是 11 关节（4 轮 + j1..j6 + gripper）、顺序与接口
  种类不变；臂的 `<limit>` 与 command min/max 改为引用 `properties_arm.xacro`
  的 `${jN_lower/upper}`（消掉 12 个魔法数字），有效限幅恒等于契约值。
  新增 `src/rak_chassis/test/validate_chassis_urdf.py` 把整套契约钉进 `colcon test`。
- **夹爪**：`tool0` 之后补了一个**无几何**的 `gripper_link` 帧（`gripper_joint`，
  revolute，限位与旧桩同值）——只为满足 component_parser 的"joint 必须在树里"，
  **不编造未实测的几何**（夹爪支架没量过）。
- **MoveIt 真机档**：`rak_moveit_config/launch/rak_arm.launch.py` 新增
  `use_hardware`（默认 **false**）。`true` 时**不再起**本地
  `controller_manager` / spawner / `robot_state_publisher`，只起 `move_group`(+RViz)，
  对接真机 `chassis_bringup` 起的同名 `arm_controller`。
  本机用 mock 替身验证：规划 69 点、Execute `error_code=0`、零
  "action client not connected"。仿真档（默认）行为不变。
- **仍未做**：真机的 J2–J6 运动/限位/急停验证（Arm Unlock Gates，未解除）；
  臂末端**没有急停**（契约 §2.5：急停只作用于轮驱动）；舵机无编码器，
  反馈是指令回声，MoveIt 无法知道真机是否真的到位。

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

## 2026-10-30 工程化阶段收口（第二轮）

本轮（2026-10-29/30）把四条结构问题收敛成正式档，全部有 ADR 与门禁背书：

- **组装层拆分**（ADR-0026）：`rak_orchestration`（契约，纯 x86 可建）+ `rak_bringup`（硬件全栈）。
  档位是数据（`build_profiles.yaml` + `launch_contracts.yaml`），
  `python3 scripts/build_workspace.py --profile desktop|jetson` 一条命令。
- **生命周期全覆盖**（ADR-0020 修订 d）：自有节点全部 managed（25/36 组件），
  三个驱动初始化顺序缺陷同批修掉；launch 统一走 `managed()` 助手，`supervised` 全链贯通。
- **时间基准**（ADR-0023）：IMU 采样时刻打戳（含自校验 `max_sample_age_ms`）；
  测量工具 `scripts/calibrate_imu.py`、占位值门禁 `scripts/timebase_gate.py`。
- **观测**（ADR-0020 §9 的观测半边）：新包 `rak_metrics`（p50/p95/p99/丢帧 + `/rak/metrics` + HTTP）
  与 rosbag 录制白名单 `scripts/record_session.py`。
- **CANopen v5 主站选型**（ADR-0024）：ros2_canopen（Jazzy），配置/launch/首帧核对工具已就位；
  **未编译未实机验证**——这是下一步的第一优先级。
- **行为层**（ADR-0025）：只出两级架构与迁移路径（BT.CPP 技能层 / SMACC2 任务层），代码未交付。
- **下位机能力规格**：`docs/firmware-capability-spec.md`——四路里程计的职责边界、
  EKF/Nav2 的八条隐性需求、固件补齐清单（F1–F10，含电池上报/EMCY 分类/参数持久化）。

**未验证清单（下次上机照 `docs/NEXT-2026-10-30.md` 逐条证伪）**：
C++ lifecycle 改造未编译；ros2_canopen 未安装；managed 全栈未真机激活；字节序待首帧；
EKF 协方差待 Allan/直行实测。

**环境事实**：桌面 Windows 机无 ROS/Tailscale，无法直达 Orin；本轮所有验证限于
纯 Python 门禁（清单↔launch 一致性、档位契约、家目录扫描——全部绿）。

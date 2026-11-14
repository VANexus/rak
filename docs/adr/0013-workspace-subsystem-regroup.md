# ADR-0013: 工作空间按子系统归堆 —— 23→18 包、brain 迁入 src、"冻结消息契约"定为分层规则

- **状态**: 已接受（2026-10-09，用户确认方向与三处权衡：brain 迁入 src / 按子系统归堆 / CAN v3 栈归档）
- **日期**: 2026-10-09
- **相关**: 全部 `src/rak_*` · `attic/`（本 ADR 创建）· `docs/explanation/architecture.md` · `docs/explanation/migration_2026-10.md` ·
  `scripts/check_workspace.py` · ADR-0012（传输缝）· ADR-0007 · ADR-0010 · ADR-0011
- **修订**: ADR-0007/0010 中"`src/rak_harness` 不存在"等表述不变；本 ADR 只改包拓扑与分层规则。

---

## 背景

- 重组前 **23 个可构建包**（22 个在 `src/` + `brain/` 在外）只有 ~17 个执行入口，其中 **6 个包零代码**
  （三个 bringup 变体、`rak_navigation` 只剩 launch、`rak_moveit_config`/`rak_description` 属建模资产）。
- 底盘一个变更半径横跨 **7 个包**（hardware、base_driver、drive_tui、chassis_bringup、navigation 内的
  chassis_test、can_driver、serial_link）——"太扁平"的实际伤害是变更半径，不是包数量本身。
- `brain/` 在 `src/` 之外：靠 `colcon --base-paths src brain` 特批、`PYTHONPATH=brain` 跑测试、
  `check_workspace.py` 为它开 `manifest.parent.name == "brain"` 特例。
- **依赖倒置**：`rak_task_executor`（node.py/console.py 顶层 import）与 `rak_speech`
  （asr_node 函数内懒 import）都 `from rak_ai.task_intent import ...`，package.xml `exec_depend rak_agent`。
  执行层反咬决策层。根因：`task_intent.py` 是两个消费者共享的**领域库**，却被放在决策层包里。
- `scripts/check_workspace.py` 的 EXPECTED 白名单已缺 7 个包（rak_hardware、rak_wbc、rak_speech、
  rak_pointcloud、rak_drive_tui、rak_arm_tui、rak_chassis_bringup、rak_serial_link），规范靠人肉补丁维持。
- CAN v4 主链路恢复（ADR-0012）后，CAN v3 host 栈（`rak_can_driver`/`rak_serial_link`）确认无回潮价值。

## 决策

### 1. 分层规则（本仓 workspace 的长期约束，写入 AGENTS.md/CLAUDE.md）

1. **消息契约是唯一的模块间缝。** 模块之间只通过 `rak_interfaces` 消息 + 标准 ROS 消息互用；
   消息与接口**冻结**——改消息 = 破坏性变更，必须同步全部消费方并在 migration 文档登记。
   上层永不感知传输后端（ADR-0012 的 Session）与传感器 SDK。
2. **同需求传感器发布同一消息集。** 不同 SDK 的深度相机（Astra/OrbbecSDK、Gemini 335L 等）各自成
   独立节点，发布相同的 topics/类型/坐标系约定；切换 = launch 参数，其他模块零改动。
3. **依赖方向单向向下**：`rak_bringup`（组装）→ 能力层（task_executor / speech / wbc）→
   领域库（`rak_intent`）与契约（`rak_interfaces`）→ 驱动层。环检测由 `check_workspace.py` 守护。
4. **rclpy 节点一律 `setup.py` console_scripts entry point**；禁止裸脚本 + `install(PROGRAMS)`
   （chassis_key_test 是最后一个违规者，本次转正）。
5. **brain（决策层）位于 `src/rak_agent`，被依赖收敛到组装层**：它只向上发布 `/brain/decision`；
   `src/` 下唯一允许 `exec_depend rak_agent` 的是 `rak_bringup`（launch `agent_node` 需要），
   任何其他包不得依赖或 import 它（ADR-0010 的 Harness 除外，尚未创建）。
   `check_workspace.py` 断言 `dependents ⊆ {rak_bringup}`。

### 2. 包归堆映射（23 → 18 构建包 + 2 归档）

| 目标包 | 来源 | 说明 |
| --- | --- | --- |
| `rak_chassis` | **更名自 `rak_hardware`** | 纯 C++：Session 抽象（CAN v4 主 / USB v2 备）、ros2_control 插件、协议编解码、测试。命名空间/插件名同步改 `rak_chassis` |
| `rak_commissioning` | **新**，吸收 `rak_drive_tui` + `rak_arm_tui` + `rak_navigation/scripts/chassis_key_test` + `launch/chassis_test.launch.py` | 全部标定/示教/驾驶 TUI 与键盘测试入口。依赖 `rak_base_driver`（USB v2 编解码共用） |
| `rak_bringup` | 吸收 `rak_sensor_bringup` + `rak_navigation` + `rak_chassis_bringup` 的 launch/config | 唯一组装层：lidar/astra/stereo/gemini335l、slam、nav2、task_navigation、agent、base_uart、chassis_bringup+controllers.yaml、run_*.sh、udev |
| `rak_intent` | **新**，`brain/rak_ai/task_intent.py` 抽出 | 版本化文本意图契约（stdlib-only + 懒加载 rai/pydantic）；executor 与 speech 共同依赖，方向向下 |
| `rak_agent` | **迁自 `brain/`** | 决策层；模块 `rak_ai` → `rak_agent`；episode 语料 → 仓库根 `data/episodes/`（数据是资产，不随代码走） |
| `attic/rak_can_driver`、`attic/rak_serial_link` | 归档 | CAN v3 host 栈，v4 退役（ADR-0012）；加 `COLCON_IGNORE`，不构建、不维护 |
| 其余 11 包不动 | interfaces、gpu、stereo_driver、astra_driver、pointcloud、base_driver、arm_simulation、description、moveit_config、task_executor（换依赖 rak_intent）、speech（同）、wbc、motion_stub | 重依赖隔离（--packages-skip）、建模资产、ADR-0011 边界等理由见"被否决" |

### 3. 记录为事实的偏差

- `rak_motion_stub` 保留独立包：其 2 个转发节点是 ament_python entry point，并入 cmake 型
  bringup 会退化成 `install(PROGRAMS)` 反模式（违反决策 4）。
- `rak_arm_simulation` 名称不改：名字本身携带"软件仿真、禁接硬件"的安全语义。

## 理由

- 按**子系统/变更耦合**归堆，而不是按产物类型（launch/工具/驱动各一包）：底盘收拢为
  `rak_chassis`（控制）+ `rak_commissioning`（调试输入）两处。
- `rak_intent` 让"文本→意图"的版本化契约只有一个事实源，执行层与语音层共同依赖向下，
  brain 回归零被依赖的纯决策层。
- brain 迁入 src 后：`--base-paths src brain`、`PYTHONPATH=brain`、check_workspace 特例全部消灭，
  测试走标准 install overlay。

## 后果

- **正面**：18 个包每个都有单一职责与真实内容；底盘/臂/传感器的变更半径各归 1–2 包；
  规范回归结构本身，check_workspace 回到"断言结构"而不是"白名单补丁"。
- **负面**：
  - `rak_bringup` 变成大组装包（launch/config 集中是 ROS 惯例，代价可接受）；
  - 一次性大规模路径变更：所有文档、launch 引用、`ros2 run/launch` 命令、CI 式检查脚本都要同步
    （本次随 ADR 一并落地，见 `docs/explanation/migration_2026-10.md` 2026-10-09 表）；
  - episode 语料默认路径变更（`brain/data/episodes` → `data/episodes`）；重组时语料目录为空，无迁移成本，
    但旧路径引用（文档/记忆）需以本 ADR 为准更新。

## 被否决的方案

- **`rak_description`/`rak_moveit_config` 并入 `rak_arm`**：URDF/SRDF 是 RViz/MoveIt/测试三方共用
  建模资产；six_axis fixture 有专门测试守护（禁止 ros2_control 标签），并入会增加误改面。
- **TUI 并入 `rak_chassis`（或 arm_simulation）**：ament_cmake 与 ament_python 不能混装，
  强并会引入 `install(PROGRAMS)` 裸脚本反模式。
- **`rak_navigation` 保留为"导航配置包"**：slam.launch.py include sensor bringup 的 lidar.launch，
  sensor_bringup 并入 rak_bringup 后会形成 `navigation ↔ bringup` 依赖环（check_workspace 环检测拒绝）。
  navigation 本无一行业务代码，整体并入。
- **brain 原地保留、只抽 task_intent**：反向依赖确实断掉，但 `--base-paths` 特批与 PYTHONPATH hack
  永久化，规范继续靠特例维持。
- **CAN v3 双包直接删除**：协议考古价值（v3→v4 演进、lease/编解码参考）+ "reserved" 语义
  （TODO P0-3）→ attic + COLCON_IGNORE，git 历史之外保留本地可读源码。

## 待闭合项

1. 本 ADR 落地同批次必须完成：全部文档路径同步（README/architecture/AGENTS/CLAUDE/TODO/project_status）+
   `docs/explanation/migration_2026-10.md` 追加映射表 + `check_workspace.py` 重写；任一未完成则本次重组不算收口。
2. `rak_speech/asr_node.py` 的函数内懒 import 改为顶层 import（依赖 rak_intent 后不再需要懒加载）。
3. 旧安装目录（`build/`、`install/`）与新结构混用时 `ros2 run` 会找不到旧可执行名——
   首次切换需 `rm -rf build/architecture install/architecture` 后全量重建。

## 修订记录

- 2026-10-10：**用户裁定删除 `attic/` 本地归档**（推翻本文"被否决的方案"中"双包直接删除"
  的否决理由——协议考古改由 git 历史承担，`git show 4f34795^:src/rak_can_driver/...` 可取回）。
  相关文档（README/architecture/AGENTS/CLAUDE/TODO/project_status）已同步。

# ADR-0020: 全栈受管生命周期 —— 故障域编排、单一所有者与存活性判据

- **状态**: 已接受（2026-10-29）
- **日期**: 2026-10-29
- **相关**: `src/rak_lifecycle/`（本决策的实现）· `src/rak_bringup/config/system_manifest.yaml` ·
  `src/rak_bringup/launch/supervisor.launch.py` · `src/rak_bringup/launch/robot.launch.py` ·
  `docs/architecture.md` §4/§6 · ADR-0011（运动单写者）、ADR-0013（分层与冻结契约）、
  ADR-0015（每帧唯一发布者）、ADR-0017（反馈存活性判据）、ADR-0019（按板故障域隔离）
- **背景来源**: 2026-10-29 现场核查（`~/rak` 工作树 + Orin 上 `ros2 pkg executables lifecycle` /
  `ros2 pkg list | grep -E 'lifecycle|diagnostic'` 实测输出）

---

## 背景

全栈现在同时存在**三套**生命周期机制，第三套根本不存在：

| 组件 | 现在的机制 | 出处 |
| --- | --- | --- |
| `slam_toolbox` | `LifecycleNode` + launch 里手写 `ChangeState`/`OnStateTransition` 事件 | `slam.launch.py:81,193-216` |
| Nav2 六个服务 | `nav2_lifecycle_manager`（`autostart=True`） | `nav2.launch.py:43-71` |
| `ros2_control` 控制器 | `spawner` 进程各自驱动 | `chassis_bringup.launch.py:79-138` |
| **其余全部自有节点** | 普通 `Node`，无状态、无启停语义 | `agent/speech/imu/task_executor/wbc/pointcloud/astra/…` |

由此产生四类**可复现**的后果，而不是风格问题：

1. **静默空转是常态**。`slam.launch.py` 自己的注释就写着：slam_toolbox 留在
   `unconfigured` 时"订阅不建、/map 不发、也不报错"。普通 `Node` 同构：起在依赖之前
   照样一声不响（`rf2o` 的 `init_pose_from_topic` 默认值是同一个失效模式的另一个实例）。
2. **所有权靠人肉布尔维持**。`robot.launch.py` 用 `publish_robot_tf:=false` /
   `chassis:=false` 这些参数避免两个 `robot_state_publisher` 同时发 `base_footprint`
   （ADR-0015 的规矩）；`AGENTS.md` 另外记着"绝不能同时跑两份 `chassis_bringup`"——
   因为第二份的 spawner 会打到第一份的 `/controller_manager` 上死掉。
   **这些约束只存在于注释和本文里，没有任何可执行的东西钉住它。**
3. **只有关进程这一档**。想停掉一个子系统只能 `pkill`（`docs/HANDOFF-AGENT.md` §1.1
   那套方括号脱敏写法就是这个操作的产物）。停不干净会留下半死的域，重启只能整栈推倒。
4. **没有"这台机器现在到底能不能用"的单一答案**。`docs/architecture.md` §6 阶段 C 与
   §7 至今挂着"lifecycle 与末端单写者尚未做运行验收"；操作员只能靠
   `ros2 node list` + `ros2 lifecycle get /controller_server` 手工拼一个判断。

同时有三条硬约束决定了不能简单"给每个节点套个状态机"：

- **4 GB 单板，资源获取很贵**：TTS CosyVoice3 推理 ~1.7 GB、历史 PyTorch 权重常驻 804 MiB
  （`docs/CONSTRUCTION.md` §2.2 与 `AGENTS.md` 记录）。反复重启一个域不能等于反复重载模型。
- **硬件不完美是定义性约束**：每一层必须能**独立降级**（`docs/architecture.md` §1 的
  2026-10-08 增补）。ADR-0019 刚把下位机按板拆成两个故障域，上位机侧却仍是一坨全有全无的进程。
- **F407 已有独立 deadman**（`cmd_stale`，ADR-0017）：Jetson 侧的生命周期**不得**变成第二条
  运动许可路径，否则安全语义出现两个权威源。

## 决策

1. **三层分工，各层只碰自己的事**：

   | 层 | 负责 | 载体 |
   | --- | --- | --- |
   | 机制 | 执行 `CONFIGURE`/`ACTIVATE`/`DEACTIVATE`/`CLEANUP` 转移 | 标准 `lifecycle_msgs` 服务（不自造状态机协议） |
   | 策略 | 谁属于哪个域、依赖谁、谁有权激活、故障时降级谁 | `rak_lifecycle` + `system_manifest.yaml` |
   | 观测 | 现在每一域什么状态、数据新不新 | 标准 `diagnostic_msgs`（`/diagnostics` + `/rak/system_state`） |

2. **八个故障域**（`system_manifest.yaml` 是唯一来源；域名对齐 `architecture.md` 的 L0–L5）：

   | 域 | 内容 | 上游依赖 | 本域故障时 |
   | --- | --- | --- | --- |
   | `link` | CAN 链路 + `controller_manager` + 两个硬件组件 + 控制器 | — | 下游**全部**降级 |
   | `body` | `robot_state_publisher`（机体 TF 唯一发布者） | — | `sensing`/`localization` 降级 |
   | `sensing` | 雷达 / 深度相机 / IMU / 麦克风 | `body` | `localization`/`task` 降级 |
   | `localization` | rf2o 或 EKF + slam_toolbox | `body`,`sensing` | `navigation` 降级 |
   | `navigation` | Nav2 六服务（仍由其自带的 manager 拥有） | `localization`,`link` | `task` 降级 |
   | `arbitration` | `rak_wbc`（底盘/臂单一运动写者，ADR-0011） | `link` | `task` 降级 |
   | `task` | `task_executor` / `tool_executor` / `harness` / `agent` | 见 manifest | `expression` 降级 |
   | `expression` | TTS / 唤醒 / 放音 | `task` | 仅记录（叶子域） |

3. **单一所有者规则**：一个组件恰好有一个生命周期拥有者，写进 manifest 并由测试校验。
   Nav2 域继续归 `nav2_lifecycle_manager`、`slam_toolbox` 继续归 `slam.launch.py` 的事件、
   控制器继续归 spawner/CM；`rak_lifecycle/system_manager` **只拥有自有节点**，
   对别人的组件只做 `GetState` 观测，绝不下发转移。

4. **组件分两档，逐域收敛**：
   - `managed`：`LifecycleNode`，由拥有者驱动，可原地激活/停用。
   - `observed`：仍是普通 `Node`，按**存活性判据**监控：节点在发现图里 + 其输出话题年龄
     在上限内 + 其硬依赖输入话题年龄在上限内。
     `observed → managed` 是路线，不是终态缺陷；混态期间两者的健康一起进 `/rak/system_state`。

5. **`ManagedNode` 契约**（`rak_lifecycle/managed_node.py`，自有节点的统一基类）：
   - `on_configure`：声明并读参数、**打开资源**（设备/模型/socket）、创建 pub/sub
     （用 `LifecyclePublisher`，保证未激活不发布）。
   - `on_activate`：启动 timer / 服务 / 动作，开始发布。**不重载资源**。
   - `on_deactivate`：取消 timer、停止发布，**保留资源**（原地重启不该重付模型加载）。
   - `on_cleanup`：释放资源。
   - 任一步失败返回 `FAILURE` 且**必须带可读 reason**（进 `/diagnostics`，manager 记录并停止下游）。
   - timer 一律由基类显式登记/取消，不依赖 `rclpy` 是否自动托管（该行为本设计不假设）。

6. **激活 ≠ 运动许可**。`allow_motion`（默认 false）是**唯一**运动许可；任何域被激活都不得
   产生 `DRIVE_CMD`/`ARM_CMD`。这条不变量优先于编排便利：lifecycle 描述"这一层活着且已配置"，
   不描述"允许推动执行器"。

7. **`autostart` 按依赖闭包顺序拉起已启用域**；某域超时或失败即置 `FAULTED`，其下游停在
   `inactive`，其上游**继续运行**（不做整栈推倒）。关停为反向顺序。

8. **所有"在线/可用"断言带时效窗**，延续 ADR-0019 第 6 条的做法：阈值来自各话题的标称频率
   （manifest 里逐条声明），**没有实测依据的阈值不得写死**，先按 `TODO: 实测替换` 标注。

9. **不新增消息契约**。本轮只用标准 `diagnostic_msgs` / `lifecycle_msgs`：`rak_interfaces`
   是冻结契约（ADR-0013），在语义尚未实机稳定前加一个 `SystemState.msg` 等于过早冻结。

10. **新增基础设施包 `src/rak_lifecycle`**：不依赖任何自有包（与 `rak_interfaces`/`rak_intent`
    同级"保持纯净"），`check_workspace.py` 按此校验；组装层仍只有 `rak_bringup`。

## 理由

- 生命周期是 ROS 2 原生机制，用它做"可停、可重启、可读"的子系统边界是**成本最低的正解**；
  `robot.launch.py` 那套布尔参数是在没有状态语义时的补丁。
- 上游没有通用 manager：实测 Orin 上 `ros2 pkg executables lifecycle` 只有
  `lifecycle_listener` / `lifecycle_service_client` / `lifecycle_talker`，**没有** `lifecycle_manager`；
  唯一在用的 manager 是 Nav2 那份。所以要自己有一层薄的**策略**代码——但机制仍走标准服务，
  不自造协议。
- manifest 化而非继续堆 include 链：把"谁发 TF / 谁有运动写权 / 谁依赖谁"从注释与本文
  变成**可被测试执行的数据**，这正是 ADR-0015/0011/0019 反复要求却一直没有可执行载体的东西。

## 后果

**正面**

- 子系统可原地停用/恢复，不必 `pkill` 一串；重启一个域不再等于重付模型加载。
- 单写者、单 TF 发布者、按板故障域这些"注释里的规矩"第一次有了可校验的数据源与测试。
- "整机现在能不能用"有单一答案（`/rak/system_state` + `rak_system status`），
  且每个 FAULTED 域带可读 reason。
- `architecture.md` §1 的"每层必须能独立降级"从口号变成编排约束。

**负面 / 代价**

- 多一个常驻进程与一层间接；调试时新增一个可混淆态：**进程活着但未激活**。
  排查路径必须同时给出（`ros2 lifecycle get <node>`、`/rak/system_state`）。
- 顺序门控让冷启动比并发拉起更慢；启动总时长与 `system_manager` 自身 RSS **待实测**。
- 混态期（managed + observed）判据有两套，observed 的话题年龄阈值**未经实测**，
  阈值写错会表现为"健康的域被报成 FAULTED"或反之。
- 被转换的每个节点都要动它的 launch（`Node` → `LifecycleNode` 且不自启）；漏改一处就是
  一个静默空转的节点 —— 这类回归由 manifest↔launch 一致性测试兜底，不靠人记。
- `diagnostic_aggregator` 本机未安装（实测 `ros2 pkg list`），聚合只能自出 DiagnosticArray；
  要么接受这份薄代码，要么装包（属环境变更，需用户定）。

## 被否决的方案

| 方案 | 否决理由 |
| --- | --- |
| 每组都用 `nav2_lifecycle_manager` | 它是一组**扁平** `node_names`，没有跨域门控与"上游故障→下游降级"；为非导航节点引入 nav2 依赖方向也违反 ADR-0013 |
| 引入 smacc2 / 用行为树做运行期状态机 | 为 20 来个节点的启停引入一个框架；本机的状态权威在任务层（`rak_task_executor` 唯一所有者）与 `guard()` 安全层，再叠一层只会多出一个权威源 |
| 纯 launch 事件串起全栈（沿用 `slam.launch.py` 的写法） | launch 在进程启动后就退出编排角色：**运行期无法重启单个域**，也拿不到整机状态；只适合两个节点的局部编排 |
| 每个节点自己等依赖（话题到了再干活） | 把时序耦合摊到每个节点，无法集中验收，也无法保证单写者与每帧一发布者 |
| 一次性把全部节点转成 managed | 现场语音/IMU/URDF 有未提交在制品，同时改必然冲突；C++ 驱动还要重建 GPU 包，代价与收益不匹配。分阶段收敛 |
| 用 lifecycle 取代 `allow_motion` 做运动许可 | 出现第二条许可路径与第二个安全权威源，直接违背 §5 约束与 F407 独立兜底的定位 |

## 待闭合项

1. `sensing` / `expression` 域的 managed 收敛（等现场 WIP 提交后进行），以及
   C++ 驱动（`rak_astra_driver` / `rak_pointcloud` / `rak_stereo_driver`）的 `rclcpp_lifecycle` 化。
2. observed 判据的阈值实测：各话题标称频率与允许年龄要由台架测量替换，不得长期占位。
3. `FAULTED` 域能否**原地** recover（不重启进程）—— 需真机验收，尤其是 CAN 掉线重连后
   `link` 域的 `deactivate → activate` 行为（与 ADR-0019 的组件级操作联动）。
4. `system_manager` 常驻 RSS、冷启动各域耗时、门控超时取值：待实测。
5. `diagnostic_aggregator` 缺口：装包 vs 保留薄聚合，由用户定路线。
6. 操作员入口（`run_*.sh` 系列）与新编排的关系：保留兼容还是收敛到 `robot.launch.py` + `rak_system`。

## 验收

- **桌面/板上可回归**（`ROS_DOMAIN_ID=77`，不碰硬件）：
  - manifest 不变量测试：无环、每组件恰好一个拥有者、运动写者唯一、TF 发布者唯一、
    被转换节点在 launch 里确实是 `LifecycleNode`；
  - `ManagedNode` 转移测试：未激活不发布、`on_activate` 不重载资源、失败带 reason、
    `on_cleanup` 释放；
  - 两域 fixture 测试：顺序激活；上游 FAULTED → 仅下游降级，上游其余部分不受影响。
- **真机门**（未过不得称"已验证"）：`allow_motion:=false` 下激活任意域，
  `scripts/can_monitor.py` / `can_link_smoke.py` 必须看不到 `DRIVE_CMD`/`ARM_CMD`；
  停掉 `expression` 域不影响 `navigation`；拔掉底盘板后 `link` 报 FAULTED 而 `sensing` 仍在。
- 现网那套跑了 5 小时的 `sensors.launch.py`（IMU+雷达+slam+ekf）属**未受管运行**，
  本轮不做改动、不据其状态声明验收结论。

# ADR-0024: Jetson 侧 v5 主站走 ros2_canopen（Jazzy 主线），lely-core 降为备选

- 状态: 已接受（**未编译、未实机验证**；见「待闭合项」第 1 条）
- 日期: 2026-10-29
- 相关: `src/rak_canopen/`（契约层）· `src/rak_bringup/launch/canopen_bringup.launch.py` ·
  `src/rak_canopen/config/bus.yaml` · `scripts/verify_canopen_frames.py` ·
  `docs/reference/canopen_bitfields.md` · ADR-0021（下位机换 CANopen）· ADR-0022（契约层）

## 背景

ADR-0021 决定两块 F407 的 CAN 层换 CANopen v5，并把 Jetson 侧主站选型留给后续 ADR
（当时写的是 lely-core）。现状是：

- 两板已是 v5（build_tag 底盘 R73 / 臂 A14），**Jetson 仍是 v4 `CanSession`**，两者不兼容；
- 台架只能靠 `tools/canopen_bringup.py`（原始 SocketCAN 脚本）打通；
- 用户明确要求 **ros2_canopen（Jazzy 最新版）**，而不是自己攒一套主站。

硬约束：

1. 主站必须能发 SYNC（2 ms）、跑 NMT、读写 PDO、做 SDO 客户端——这些是 CiA 301 的活，
   自研等于把"协议栈"重新实现一遍，违背"现成成熟方案优先"；
2. 生产路径必须能进 ros2_control（轮速/关节要走 `hardware_interface`）；
3. 明天要上实机，**必须留回滚**：v4 `rak_chassis` 不能因为迁移而消失。

## 决策

1. **主站 = `ros2_canopen`（canopen_core / canopen_ros2_control，Jazzy 分支最新版）**。
   Jetson 是 NMT master（node 127），底盘 node 1、机械臂 node 2，EDS 用
   `src/rak_canopen/config/{chassis,arm}.eds`（与 SSOT JSON 同批生成）。
2. **总线级配置集中在 `rak_canopen/config/bus.yaml`**：接口名、SYNC 周期、心跳、
   每个节点的 EDS 路径与驱动插件。任何"帧级"参数**不得**写进 launch。
3. **ros2_control 侧走 `canopen_ros2_control/RobotSystem`**，关节 → OD 对象的映射
   写在 `rak_canopen/config/canopen_controllers.yaml`，接口名以 `0x####`（对象索引）
   为准，**不新造语义名**（避免出现第四份帧表，ADR-0022 的原罪）。
4. **迁移是并联不是替换**：`chassis_bringup.launch.py` 增加 `transport:=canopen|can_v4`，
   默认仍是 `can_v4`（现场那套在跑）；切到 `canopen` 时不起 `CanSession`。
   两条路径互斥由 launch 参数保证，**不得同时起**（同一总线双主站会把节点踢出 Operational）。
5. **字节序与帧长以首帧实测为准**：`scripts/verify_canopen_frames.py` 吃 `candump -L`
   日志，按契约逐帧比对 COB-ID、DLC、字段偏移与量化后的物理量范围；
   契约里的 `byte_order: little` 标着 `TODO(实测)`，脚本跑绿之前不许改成"已确认"。
6. **文档措辞统一**：在 SRDO（CiA 304）未接之前，只说
   **"具备走向 SIL2 的通信基础"**，不写"可认证"。

## 理由

- `ros2_canopen` 已经把 NMT/SYNC/PDO/SDO/心跳消费与 `ros2_control` 插件做完了，
  我们只需要提供 EDS + 映射 + 生命周期接线——这正是"现成件优先"。
- 与 lely-core 相比：lely 更底层、许可证更宽松，但要自己写 `hardware_interface` 与
  ros2_control 胶水，工作量与出错面都更大；且它不在 ROS 生态里，升级/调试都靠自己。
- 保留 v4 回滚路径的成本很低（一个 launch 参数），而"明天上不了台架"的成本很高。

## 后果

**正面**

- 主站不再需要自研；NMT/SYNC/心跳这些最容易出错的时序由成熟实现承担。
- 与 `ros2_control` 天然对接，`mecanum_drive_controller` / `joint_trajectory_controller`
  的上层契约不变（ADR-0007 继续成立）。
- 字节序/帧长有了可执行核对工具，不再靠"按 STM32 小端推断"。

**负面 / 代价**

- 多一套外部依赖（canopen_core/canopen_ros2_control），**Orin 上是否装得上尚未验证**；
  装不上就必须回 v4 或走备选主站，这是明天之前必须确认的第一件事。
- `ros2_canopen` 的驱动插件模型要求 EDS 与它的对象命名约定对齐，
  我们的自定义对象（`0x2000` 起）需要逐条确认是否能被 proxy driver 暴露成接口。
- 两套传输并存期间，`link` 域的生命周期拥有者会随 `transport` 变化，
  清单必须按"当前 transport"声明（ADR-0020 修订记录已记这一条）。

## 被否决的方案

| 方案 | 否决理由 |
| --- | --- |
| lely-core（ADR-0021 原选型） | 需要自己写 ros2_control 胶水与 PDO 映射；今天无法完成，明天上不了台架 |
| 继续用 `tools/canopen_bringup.py` 当生产主站 | 它没有失败处理、没有 SDO 状态机、没有 ros2_control 接口；是台架脚本不是主站 |
| python-canopen（`canopen` 库）自建节点 | 比 lely 简单，但仍要自己写 `hardware_interface` 与生命周期；且多一个 Python 实时性包袱 |
| 保持 v4、让板子回滚到 `APP_USE_CANOPEN=OFF` | 与 P3 方向相反，且两块板已经在 v5 上验收过 |

## 待闭合项

1. **安装与编译验证**（最高优先）：在 Orin 上确认
   `ros-jazzy-canopen-core` / `canopen-interfaces` 版本与 Jazzy 主线一致，
   `canopen_ros2_control` 能否用我们的 EDS 起 `RobotSystem`。
   **在跑通之前，本 ADR 的实现状态是"未验证"，不得在 README/状态页写成"已接入"。**
2. **对象→接口映射**：确认 proxy driver 如何把 `0x2000..0x2003`（INT16 速令）暴露成
   `hardware_interface` 的 command interface；不确定就退回"自定义 SystemInterface +
   canopen_core 的 PDO 回调"，但那是新决策，需另开 ADR。
3. **SYNC 2 ms 的主站实现细节**：ros2_canopen 的 SYNC 由 `canopen_core` 的
   `sync_interval_us` 控制还是由 master 节点配置，需按实际版本核对。
4. **字节序首帧核对**：`scripts/verify_canopen_frames.py` 尚未在真实 candump 上跑过。
5. **心跳消费者 500 ms**：契约里已写 `consume_timeout_ms: 500`，主站侧是否真的在
   超时后把节点判死并触发降级，需实测。
6. **SRDO（CiA 304）**：未接。在此之前所有文档措辞为"具备走向 SIL2 的通信基础"。

> 本节的执行顺序、命令与判据见 [`docs/status/next_2026-10-30.md`](../status/next_2026-10-30.md) §5 CANopen 安装与首帧。

### 2026-10-30(a)：v4 整体下线，台架临时主站上线，三条实测事实入档

- **v4 没了**：R75/A16 已把 `app_can.c/h`、`PROTOCOL.md`、`APP_USE_CANOPEN` 开关全部删除
  （伞形仓 AGENTS.md 顶部通告）。本文与 ADR-0020 修订 (d) 里所有"v4 回滚分支"的表述
  **自此失效**：`rak_chassis` 的 CanSession 对真板无效，只剩软件夹具用途；
  `robot.launch.py transport` 默认改 `canopen`。
- **台架链路已通**：`tools/canopen_master.py`（伞形仓，零依赖、SSOT 派生、自标"到期即删"）
  完成 R74/A15 → R75/A16 两轮烧录后的三节点联测（SDO 身份、八条 TPDO 速率全达标、
  EMCY=0、ERROR-ACTIVE），记录见 `docs/CAN-BRINGUP-2026-10-30.md`。
  它**不是**生产主站，也不许被当成生产件（无 SDO 写、无心跳消费、SYNC 用户态抖动）。
- **两个未闭合的实测问题**（比本文任何选型讨论都优先）：
  1. **TPDO 全灭**（两次复现，根因未定，主嫌疑 `task_can_tx` 饿死）——自愈只治已实证形态；
  2. **底盘 ISR 占用率 41.3% > 35% 预算**（臂 18.8% 达标）——按 AGENTS §3 已"停下上报"，
     **在复测回落之前，底盘不得进入实车运动**。
- 本文"待闭合项 #1（安装与编译验证）"仍然成立：ros2_canopen 在 Orin 上仍没有装过。

### 2026-11-14：主机侧 v4 栈整体删除，本 ADR 的实现状态更新

- **v4 没了（主机侧）**：`CanSession`/`can_decode`/`can_session_pool`/`Session` 抽象、
  v4 帧表与 `scripts/can_link_smoke.py`、`canopen_bringup.launch.py` 全部删除。
  `chassis_bringup.launch.py` 是唯一底盘链入口，起 `canopen_core/device_container_node`
  （NMT master node 127，底盘 1 / 臂 2）+ `controller_manager`。
- **`canopen_ros2_control/CanopenSystem` 不满足运动环**：它的 read/write 是 raw
  RPDO/TPDO 直通（`rpdo/index|subindex|data`），不做量化、不做状态刷新，标准控制器
  （`mecanum_drive_controller` / JTC）无法直接绑定。故 `src/rak_chassis` 改为**薄桥**
  （`CanopenChassis` / `CanopenArm`）：订阅 `<driver>/rpdo`、发布 `<driver>/tpdo`，
  只做 OD 对象↔`ros2_control` 接口、量化、`allow_motion` 安全门与按板反馈超时；
  它不含任何 CAN 帧编解码，§3 的"不新造语义名"约束不变。
- **`master_config` 语义已实测**（待闭合项 #1 部分闭合，见
  `docs/status/canopen_vcan_2026-11-14.md`）：它必须是 **`dcfgen` 生成的主站 DCF**
  （含 1F80/1F81/1F85 网络表）。传从站 EDS 时主站不发 NMT reset → proxy driver 1 s
  Boot Timeout → 上游 retry 路径抛出未捕获 `std::system_error: Command: Operation
  not permitted` 并 abort。`device_container` **不是** lifecycle 节点。
  生成：`scripts/gen_canopen_master_dcf.py`；漂移门禁 `src/rak_canopen/test/test_dcf_sync.py`。
- **EDS 生成器修正**：两个下位仓的 `tools/canopen/template.eds` 把 COB-ID 默认值写成
  `0x80+$NODEID`，官方 `dcfgen`/`dcf-lint` 只接受 `$NODEID+0x80` 形式（否则直接
  `ValueError: invalid value`）。已改模板并重新生成两份 EDS。`CountLabel` 条目保留
  （CiA 306 合法、CANopenNode 导出器要），故 dcfgen 仍用 `-S`。
- **仍未闭合**（不得写成"已接入"）：真机 `can0` 未验收；vcan 只证明主站/NMT/SDO；
  PDO 实际收发、SYNC 2 ms 抖动、心跳消费（fake slave 的 `0x1017` 无默认值，不发周期心跳）、
  SDO 写、桥的端到端（proxy driver ↔ ros2_control）全部未验。

### 2026-11-14 追加：0.3.4 的 `device_container` 一个容器只挂了一个驱动

`chassis_bringup.launch.py` 在 vcan0 上实测（无下位机）：主站、`bus.yaml`、`dcf_path`、
两份 EDS 路径全部正确解析，但 `device_container_node` **只加载了 `bus.yaml` 里第一个非 master
设备**（`arm`；把两个设备名换成 `dev_a`/`dev_b` 后同样只有 `dev_a`）。三设备配置复现同一形态，
`load_drivers()` 之后没有任何"Loading driver failed / duplicate"日志 —— 即加载循环在第一台
成功后就不再继续（0.3.4 apt 构建的可观察行为，master 分支源码是遍历全部设备的）。

后果与处置（**未解决，勿当成双板可用**）：
- 一个容器 = 一块板，意味着第二条 `device_container` 会再起一个 NMT 主站 —— **同一总线禁止**。
  所以"双板 + 单主站"在当前 apt 版本上还没有可用配置。
- 已验证的是单板形态：`bus.yaml` 只留 `chassis`（或只留 `arm`）时驱动正常加载、
  `rak_chassis` 薄桥正常发布 tpdo、`controller_manager` 正常 configure/activate 该硬件。
- 下一条路（未选）：从源码编 ros2_canopen（master 分支已遍历全部设备）；或给两块板各一个
  `device_container` 并把 NMT 主站拆成独立进程（等于改上游模型）。任选其一都要另写 ADR。

另：无下位机时 proxy driver 三次 Boot Timeout 后会抛 `ros2_canopen::DriverException` 让
**整个容器进程 abort**（上游行为）。因此台架无板时 `chassis_bringup` 只能验证到"驱动加载 +
硬件 configure/activate"，不能长跑。

### 2026-11-14 二次修订：多从站、驱动激活、SYNC —— 台架实测结论

1. **"一个容器只加载第一个从站"是误判，已撤回。** 之前的现象是：第一台从站 boot 失败
   → 上游 proxy driver 三次超时后抛 `DriverException` 让**整个容器 abort**，第二台根本没轮到。
   用官方 `canopen_fake_slaves` 同时起 node 1 / node 2（各自喂我们的 `chassis.eds` /
   `arm.eds`）后，`device_container` 正常打印 `Found device arm` / `Found device chassis`
   并都进入 boot 成功。**多从站没有问题**，缺的是"无板时别让容器 abort"的台架纪律。
2. **驱动必须被官方 lifecycle 流程激活，否则不会发布 `~/rpdo`。**
   `ros2_canopen` README 的官方步骤是：容器加载后
   `ros2 lifecycle set /lifecycle_manager configure` → `activate`。
   因此 `chassis_bringup.launch.py` 现在：
   - 用 `LifecycleMasterDriver` / `LifecycleProxyDriver`（bus.yaml 已切换）；
   - 起容器后 6 s / 9 s 分别执行上面两条官方 CLI；
   - controller_manager 延后到 12 s 起动 —— 硬件若先激活，薄桥会在
     `startup_grace_ms`（新增参数，默认 5000 ms）后把硬件判死，控制器永远起不来。
   台架验证：两个 hardware 组件都 `Successful 'activate'`，
   `joint_state_broadcaster` / `mecanum_drive_controller` / `arm_controller` 全部
   `Configured and activated`。
3. **SYNC 缺失（真问题，未闭合）**：`ros2_canopen` 0.3.4 的主站**不发 SYNC**
   （源码里没有 SYNC producer；`bus.yaml` 的 `sync_period` 只被 dcfgen 写进
   `master.dcf` 的 `0x1006`，运行期无人消费）。我们的 TPDO transmission type = 5
   （SYNC 触发，[V5] §4），因此**没有 SYNC 就没有 TPDO**，薄桥收不到 `~/rpdo`，
   台架只能用心跳（0x701/0x702）证明节点活着。
   两条官方闭合路径（都需要先写 ADR，二选一）：
   - 主站侧补齐 SYNC producer（上游能力缺口，走 issue/PR 或换 ADR-0021 原本的 lely 主站）；
   - MCU 侧把 TPDO 改成事件定时器触发（transmission 254 + event timer），
     这会改 [V5] 规范件，属于下位机仓的协议决策。

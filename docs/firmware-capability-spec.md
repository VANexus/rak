# 下位机通讯能力规格 —— 让上位机"不用再拆机"的一次到位清单

- **日期**: 2026-10-30
- **状态**: 提案（待两块下位机仓签字后进固件，升 build_tag）
- **适用**: 底盘板 `vehical_rak_motion`（R73）、机械臂板 `vehical_rak_edge_ai`（A14）
- **关联**: `docs/CANOPEN-V5.md`（协议 SSOT）· `rak/docs/adr/0024`（主站）·
  `rak/docs/adr/0023`（时间基准）· `rak/docs/canopen-bitfields.md`（位语义）·
  ADR-0002（融合）/ ADR-0007（ros2_control）

> **为什么现在写**：目标链路是「四路里程计（IMU / 轮式 / 雷达 / SLAM）→ EKF →
> Nav2 避障导航」。这条链里**只有轮式里程计需要下位机参与**，其余三路都在 Jetson；
> 但 EKF 与 Nav2 对下位机还有一堆**隐性要求**（连续性、时戳、电池、故障语义）。
> 现在把它们一次写清楚并排进固件版本计划，就永远不用"拆下来重新写固件"。

---

## 1. 原则：库与职责边界（回答"工业级稳定？要不要自造轮子"）

| 层 | 用什么 | 为什么是它 |
| --- | --- | --- |
| MCU 协议栈 | **CANopenNode v4（第三方，含 CiA 301+304）+ 官方 STM32 移植** | 开源 CiA 301/304 的事实标准；EMCY/心跳/SRDO/SDO 都是现成实现。自研 = 把一致性测试重新踩一遍 |
| MCU ESD/固件升级流程 | EDSSharp 官方导出器（已在用）+ `gen_eds.py` | OD 由 EDS 生成，不允许手写（V5 §2 已定） |
| Jetson 主站 | **ros2_canopen（Jazzy）**（生产）；台架过渡 = `tools/canopen_master.py`（自标"到期即删"）；备选 lely-core | 生产用 ROS 生态成熟实现；台架不被选型堵住（ADR-0024 + 2026-10-30 修订） |
| 融合/导航 | robot_localization + Nav2（上游发行版，不自改源码） | 上游维护；我们的钱花在参数与判据上 |

**稳定性结论**：栈本身是工业级的。真正的风险从来不在库，而在五件事——
共地与终端电阻（V5 §1）、SYNC 时序、bus-off 自愈（R51 已修）、
EMCY 覆盖是否完整（**本轮缺口**，见 §5）、上电顺序（Pre-op → NMT Start）。
这五件在 §5 逐条有对策。

**职责红线**（谁都不许越）：

```
MCU：25 kHz 闭环 + 执行 + 本地安全兜底 + 原始反馈上报
Jetson：运动学（IK/正解）、全部里程计积分、融合、规划、任务
```

---

## 2. 四路里程计：逐个说清楚（含"下位机要做什么"）

| 里程计 | 在哪算 | 下位机职责 | 现状 |
| --- | --- | --- | --- |
| **轮式** | Jetson：`mecanum_drive_controller` 从四轮反馈正解积分（ADR-0007） | 上报**高质量轮速/轮位**（见 §4-1） | ✅ TPDO1/2/3 已有；⚠ 机械常数未钉死（§4-1） |
| **IMU** | Jetson：LSM6DSRTR 是航向权威（ADR-0002） | **不做**姿态解算；板载 imu_dyaw/pitch/roll 仅作稳定器与诊断 | ✅ TPDO4 已有；`imu_age` 必须真实（§4-3） |
| **雷达** | Jetson：rf2o/EKF | 无（`/scan` 由雷达驱动直发） | ✅ 与 MCU 无关 |
| **SLAM** | Jetson：slam_toolbox | 无 | ✅ 与 MCU 无关 |

结论：**不要在 MCU 里加任何"里程计"对象**。四路里 MCU 唯一的活是"把轮子数据报好"。

---

## 3. EKF / Nav2 对下位机的硬需求（这是"隐性要求"清单）

EKF 收敛与 Nav2 安全，各依赖下位机一条性质。缺任何一条，表现都是"导航莫名漂/莫名停"，
而且**查不到下位机头上**——所以必须预先钉死：

| # | 上游需求 | 下位机必须保证 | 现状 |
| --- | --- | --- | --- |
| N1 | 轮位反馈可积分 | counts 单调（同方向不回绕抖动）、符号约定"前进为正"、上电从 0 起 | ⚠ 符号已实测对齐；**单调性/回绕语义未写进规范** |
| N2 | 断链重连不跳变 | CAN 重连后**不重置**轮位计数；重置只能随 `reset_cause` 发生 | ❓ 未在规范中声明，需固件确认 |
| N3 | 反馈时戳可信 | `imu_age/cmd_age_ms` 真实反映采样年龄（不是恒 0） | ⚠ 有对象，语义未验证 |
| N4 | 速度上限有第二道闸 | MCU 侧对轮速指令有**硬饱和**（Nav2 参数错了也不许超物理） | ✅ 已有（轮级等比饱和） |
| N5 | 停车语义明确 | deadman 100 ms 超时 → 零目标 + 松链接（V5 §5）；急停锁存 → 拒绝 START | ✅ 已有 |
| N6 | 电量可见 | **电池电压上报**（Nav2 低电行为 / 任务取消需要） | ❌ **缺失**（`motor_vbat_mv` 在 MCU 内部，没进 OD） |
| N7 | 故障可分类 | EMCY 错误码 + `0x1001` error register 位语义 | ❌ **缺失**（现在故障只有"非零"，无法分类） |
| N8 | 重启可检测 | `reset_cause`（已有）+ 建议加**上电次数/运行秒数** | ⚠ 有 reset_cause，无维护量 |

---

## 4. 固件补齐清单（建议升 v5.1，按优先级）

> 每条 = 新增/修改 OD 对象或行为。全部走既有流程：改 `docs/canopen/<node>.json` →
> `gen_eds.py` → EDSSharp → `OD.c/h` → `canopen_gate.py` 绿 → 升 build_tag。
> **P1 = 本周就该做**（影响 EKF/Nav2 上车）；P2 = 下次拆机窗口；P3 = 可永远不做但留了对象。

### P1（直接卡 EKF/Nav2 上车）

| 项 | 节点 | 内容 | 验收 |
| --- | --- | --- | --- |
| F1 | 底盘 | `0x2060 vbat_mv`（uint16，mV）入 TPDO4 或 SDO 可读；来源 = 已有的 `motor_vbat_mv` | Jetson `ros2 topic echo` 可见电压；欠压时 `imu_evt` 或 `car_state` 给出标志 |
| F2 | 双板 | EMCY 错误码表 + `0x1001` error register 位定义，写进 JSON SSOT 与 `canopen-bitfields.md` | 人为制造一种故障（如轮堵转），EMCY 携带正确 code |
| F3 | 双板 | `imu_age`/`cmd_age_ms` 语义钉死："自最近一次采样/命令以来的 ms"，饱和到 65535 | 与 Jetson 时钟对齐验证（ADR-0023 §3 的偏移观测） |
| F4 | 底盘 | 轮位**单调性/回绕**语义：正转递增、不因断链重连重置；int32 回绕周期写入 JSON | 长跑 2 h 无跳变；拔插 CAN 后 counts 连续 |
| F5 | 机械臂 | `arm.json` 补 `identity` 段（device_name/hw_version/sw_version = A14） | `canopen_gate.py` 绿；SDO `0x2F00` 与 JSON 一致 |

### P2（工业健壮性，下次拆机窗口做）

| 项 | 节点 | 内容 | 说明 |
| --- | --- | --- | --- |
| F6 | 双板 | `0x2F02 build_date`（uint32，Unix 秒）、`0x2F03 git_hash32` | 可追溯：现场读两个数就知道跑的是哪次提交 |
| F7 | 双板 | `0x2F10 power_on_count`、`0x2F11 uptime_s`（uint32） | 维护与"何时该检修"的依据 |
| F8 | 双板 | `0x1010`/`0x1011` 参数存储/恢复（CANopenNode storage 回调） | 标定值（encoder_sign、零位）经 SDO 持久化，不再依赖刷写 |
| F9 | 底盘 | `0x2061 odom_epoch`（uint16）：每次非 CAN 复位的上电 +1 | Jetson 检测到 epoch 变化即拒绝旧地图/旧里程计（SLAM 连续性） |
| F10 | 机械臂 | `gripstate` 实装（当前恒 0）：位置 ±（若有）电流 | 夹爪状态进入 `joint_states` 才能进 MoveIt 规划 |

### P3（预留，只写进 JSON 注释，不实现）

- `0x2062 sample_tick`（uint32，1 kHz 自增）：MCU 采样时钟与 Jetson 漂移的可观测锚点
- `0x2063 wheel_load`（预留）：堵转/负载估计
- `0x2170 arm_payload_mass`（预留）：末端负载供 WBC/HQP 使用
- SRDO（CiA 304）：P2 计划中的双板安全协同，Jetson 只读

### 机械常数（不是固件，但同样"一次到位"）

轮径、`lx`/`ly` 轮距、减速比、counts/rev —— 这些是 `mecanum_drive_controller` 正解的输入，
**权威位置 = `rak_description` 的 URDF（xacro 参数）**，实测一次回填一次。
AGENTS 已把它们列为"待实测不得当事实"——排进 §6 的实测日，别让它们继续漂。

---

## 5. "图灵完备"检查表（每一项都要能回答"要不要、为什么"）

对下位机通讯逐项过一遍，能说"不需要"的也算闭合：

| 领域 | 结论 |
| --- | --- |
| 实时性 | 25 kHz 闭环在 MCU；Jetson 100 Hz 命令刷新 —— 已定，不变 |
| 时间同步 | 不做主站式同步；用"年龄"对齐（ADR-0023）。`TIME 0x100` 只作诊断 |
| 安全 | 急停 = 硬线双通道 + MCU 本地 ≤1 ms；CAN 只是非安全路径（V5 §5）——已定 |
| 故障语义 | F2 补齐后闭合；在此之前文档只用"SIL2 通信基础"措辞 |
| 参数管理 | F8 之后闭合；之前标定值只能随固件 |
| 可观测 | F6/F7 + 位表（canopen-bitfields.md）之后闭合 |
| 扩展性 | P3 预留区 0x2062..0x207F / 0x2170..0x217F 已声明，**不许散着加对象** |
| 固件更新 | ST-LINK 烧录（现状）；CAN OTA 不做（无安全隔离，收益低） |
| 多主站 | 不允许：Jetson 唯一 NMT master（node 127），台架脚本临时接管时必须先停 ros2_canopen |

---

## 6. 执行顺序

1. 两块下位机仓把 §4-P1 排进固件版本（F1–F5），`canopen_gate.py` 全绿后升 tag（建议 R74 / A15）。
2. Jetson 侧在同一次上机里做 `verify_canopen_frames.py` 首帧核对（字节序回填）。
3. P2 项挂在"下次拆机"窗口，不阻塞实车。
4. 本文件的每次变更同步三仓副本？——**不需要**：本文件是上位机视角的需求单，
   协议事实仍以 `docs/CANOPEN-V5.md`（三仓同哈希）与 `<node>.json` 为准。

## 7. 底盘精度与抗打滑设计（2026-10-30，基于首日台架实测）

实测底座（见 `CAN-BRINGUP-2026-10-30.md` §6）：四轮 Δpos 散布 0.5%、编码器标度
-61563±0.1% counts/rev（与标定 61600 吻合）、0.922 rad/s 对 1.0 指令稳态误差 -7.8%、
EMCY=0、slip 位路径通。

### 7.1 精度底座（轮式里程计的三条铁律）

1. **只用位置通道**：x2010..13 速度反馈实测虚高 ×100.2（待固件修，见 §4-P1 新增项 F11）。
   里程计积分只用 TPDO2 的 counts（已证正确、单调）；counts unwrapping 在 Jetson 侧做。
2. **几何常数实测回填**：cpr=61600（已实测确认）；轮径、lx+ly 进 URDF 后，
   mecanum_drive_controller 的 odom 才可信——这两个数不实测，后面全是空转。
3. **时间**：counts 自带 100 Hz 节拍（TPDO2），Jetson 用到达时刻+SYNC 对齐（ADR-0023）。

### 7.2 抗打滑（麦轮 × IMU，三级响应）

- **检测**（Jetson 侧，权威=Jetson LSM6DSR gyro-z）：
  `残差 = ω_kinematic(四轮速+几何) − ω_gyro_z`；|残差| > 3σ（σ 由 rak_metrics 首日基线出）
  持续 ≥5 周期 → 打滑嫌疑。固件 `imu_evt.SLIP`（0x02）作为旁证并入。
- **L1 观测**：残差进 `/rak/metrics`（p50/p95），先看分布再定阈值——阈值必须有 basis。
- **L2 融合保护**：打滑嫌疑期间轮式 odom 协方差膨胀（×(1+k·残差²)），
  航向权重向 gyro-z 倾斜；EKF 参数动态化若 robot_localization 不支持，
  退而求其次：切 `differential`/降 odom0 权重的静态保守值。
- **L3 执行保护**：打滑期间 `/cmd_vel` 限幅 ×0.5（arbitration 层做，工具层不碰安全语义）。
- **架空期限制**：真打滑不可复现，本轮只验证数据通路（CSV 已含 evt 位与 dyaw）；
  阈值标定排在落地期。

### 7.3 固件侧新增待办

- **F11（P1）**：修 x2010..13 速度反馈 ×100.2（对照 R49 注释"urad/s÷10⁴"核
  `counts_ms_q8` 窗口语义）；修好前 Jetson 契约标注"速度通道不可信，用位置通道"。
- **F12（P2）**：TIM6 丢拍计数器（UIF 残留检测）——ISR 门禁从"35% 魔法数"改为
  "不丢拍 + 稳态/瞬态双统计"（用户已质疑 35% 口径，见 CAN-BRINGUP §6.1）。

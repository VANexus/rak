# TODO — 工作队列

> 状态：现行 · 最后更新：2026-11-14

本文件是本仓**唯一**的工作队列。规则：

1. 只写"做什么 + 怎么判过 + 依据在哪"；细节、命令、验收脚本在依据文档里，不复制。
2. 做完打 `[x]` 并在括号里写实测结论（命令 + 关键输出），**不删条目**；
   一轮结束把已闭合条目移进 `docs/status/project_status.md` 对应日期的增量段。
3. 没跑过的验证一律写"未验证"，不写"应该没问题"。
4. 优先级看阻断关系：P0 挡住真机，P1 挡住定量结论，P2 是质量升级，P3 是远期。
5. 进度叙述只在这里和 `docs/status/project_status.md`，其他文档只链接。

## P0 — 挡住真机

- [ ] **R76 烧录 + 冷启动核验**（底盘板 `~/vehical_rak_motion`）
  构建双 preset 零警告、`tools/canopen_gate.py` 绿、tag `R76` 均已完成；
  剩烧录，冷启动后核 `build_tag`（`0x2F00`）与复位原因（`0x2F01`）。
  依据：`../vehical_rak_motion/docs/CANOPEN-V5.md`、`docs/status/project_status.md`。
  已就绪产物：Debug `abeea499f6a14924` / Release `58ee9eb151fe08a7`（SHA-256 前 16）。

- [ ] **ros2_canopen 主站装机 + `transport:=canopen` 首跑**
  先确认 apt 里有没有可用包；首帧核对按 ADR-0024 §5：每个 PDO 的 DLC 与契约
  `byte_size` 一致、没有未知 COB-ID、物理量在合理范围、字节序证据回填。
  依据：`docs/adr/0024-canopen-master-on-jetson.md` §5、
  `docs/adr/0022-jetson-canopen-contract-layer.md`、`docs/status/next_2026-10-30.md` §5。

- [ ] **机械臂真机 J2–J6 运动 + 限位 + 运动中急停**（J1 已验，其余未验）
  依据：`docs/reference/hardware_acceptance.md` 的臂侧门禁、
  `docs/reference/arm_moveit_contract.md` §1。

## P1 — 挡住定量结论

- [ ] **轮径 + lx/ly 实测回填 URDF**（麦轮正解的前提）
  依据：`docs/reference/firmware_capability_spec.md` §4。

- [ ] **EKF 协方差标定**：Allan 静态噪声 + 直行 2 m 回归，回填 `ekf.yaml`
  依据：`docs/adr/0023-time-base-and-alignment.md`、`docs/status/next_2026-10-30.md` §6。
  工具：`scripts/calibrate_imu.py`。直行横向偏移 > 5 cm 时**先查轮径/轮距与零偏**，
  不要直接调融合参数。

- [ ] **时间基准实测记录落盘**（静止 30–60 min，记 `imu0_covariance[vyaw]`）
  依据：ADR-0023；记录写在 `docs/reference/perf/`（新文件：`timebase.md`）。

- [ ] **pitch/roll 符号约定现场核对**（核对前 Jetson 不消费 `0x2031`/`0x2032`）
  依据：`docs/reference/canopen_bitfields.md` §7。

## P2 — 质量升级

- [ ] **底盘控制升级 C1–C5 按序推进**
  C1 静摩擦前馈机制已随 R76 落地，增益重整定等台架首测。
  依据：`docs/design/chassis_control.md` §6。

- [ ] **抗打滑三级接入的 σ 标定与 L3 gate 重接线**
  代码完成（L1 残差进 `/rak/metrics`、L3 gate 默认关、L2 静态回退）。
  架空期真打滑不可复现，落地期做。
  依据：`docs/reference/firmware_capability_spec.md` §7.2。

- [ ] **Nav2 `vx_min` 最低轮速钳位复核**（MPPI `vx_min` -0.10 → +0.02，basis 是推断）
  依据：`docs/design/chassis_control.md` §6。

- [ ] **zram/swap 取舍实测**（4 GB 板上是帮忙还是添乱，无实测不下结论）
  量 `si/so`、压缩比、真实负载下 p99 延迟。依据：`docs/reference/machine_baseline.md`。

- [ ] **`rak_metrics` 低速平坦样本占比**：Orin 上 `colcon` 编译后跑一次，留数据。
  依据：`docs/design/chassis_control.md` §6。

## P3 — 远期（前面全绿再动）

- [ ] **行为层 ADR-0025 阶段 1**：`behaviortree_cpp` 可用才开工；`rak_task_executor`
  加 `engine:=sm|bt`，**默认 `sm`**，验收要求两种引擎的 `/task/status` 序列逐字段相同。
  依据：`docs/adr/0025-behavior-layer-bt-and-smacc2.md`、
  `docs/status/next_2026-10-30.md` §9。

- [ ] **`docs/design/brain_arm_tools.md` 落 ADR 后再实现**（当前只是草案）。

## 文档与规范（持续）

- [x] 文档工程按 Diátaxis 重组 + 索引 + 状态行 + 门禁（2026-11-14）
  索引与写作规则：`docs/README.md`；门禁：`python3 scripts/check_docs.py`。
- [x] AGENTS.md 成为唯一代理规则文件，CLAUDE.md 删除（2026-11-14）。
- [x] 机械臂仓改名 `vehical_rak_edge_ai` → `vehical_rak_arm`（GitHub + 本地，2026-11-14）。
- [ ] 把 `python3 scripts/check_docs.py` 接进 `scripts/build_workspace.py` 的收尾检查，
  让文档门禁与构建门禁同批跑。
- [ ] 清理仍然悬空的文档引用（`docs/CAN-BRINGUP-2026-10-30.md` 从未存在过；
  含该引用的文件改成指向真实文档）。
- [ ] 每季度核对一次 `docs/README.md` 的"最后更新"列，过期的改成 `已过期` 并考虑移入
  `docs/archive/`。
- [ ] 项目 skills 更新：`cd ~/rak && npx skills@latest update -p -y`，改动进
  `skills-lock.json` 一起提交。

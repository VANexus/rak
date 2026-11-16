# docs/ 索引与写作规范

> 2026-11-14 重组：按 Diátaxis 四象限分目录，加本索引。
> 目的只有一个——**读者按"我要做什么"找到文档，并且一眼看出这份文档现在还准不准。**

## 1. 目录约定

| 目录 | 回答什么问题 | 读者状态 | 要求 |
|---|---|---|---|
| `tutorials/` | "我要从头把它跑起来，走一遍" | 学习 | 一条可照做的路径，命令可复制，不讲取舍 |
| `how-to/` | "我要完成某一件事，怎么做" | 有目标 | 单个任务、步骤化、前提与判定写清 |
| `reference/` | "这个字段/接口/门限到底是什么意思" | 查 | 可核对的事实，单一事实源，不写叙述历史 |
| `explanation/` | "为什么是这样设计的" | 想理解 | 背景、取舍、被否方案；不写操作步骤 |
| `design/` | "尚未落地的设计草案" | 评审 | 必须带状态行；接受后落 ADR 并回填状态 |
| `status/` | "现在到哪一步了、还差什么" | 跟踪 | **带日期**，过期即标过期，不追改历史 |
| `archive/` | 已被取代/已执行的旧快照 | 考古 | 只读，不再更新，正文顶部说明被谁取代 |
| `adr/` | 不可变决策记录 | 追溯 | **冻结**：只允许修正链接与状态行，正文不改 |

决策类内容先写 ADR（`adr/`，编号续排），不要塞进 `explanation/`。
`suggestion → accepted` 的流程见 `adr/README.md`。

## 2. 索引

**入口文档**

| 文档 | 象限 | 状态 |
|---|---|---|
| [`../README.md`](../README.md) | 工程入口 | 现行 |
| [`../AGENTS.md`](../AGENTS.md) | 代理/协作规则（唯一份） | 现行 |
| [`../TODO.md`](../TODO.md) | 工作队列 | 现行 |

**Tutorials**

| 文档 | 状态 | 最后更新 |
|---|---|---|
| [`tutorials/onboarding.md`](tutorials/onboarding.md) | 现行（新队友：模型网关 / 工具配置 / 环境 / 基础设施地图） | 2026-11-16 |
| [`tutorials/construction.md`](tutorials/construction.md) | 现行（装配顺序 + 部署清单） | 2026-10-14 |

**How-to**

| 文档 | 状态 | 最后更新 |
|---|---|---|
| [`how-to/laya_tensorrt_runbook.md`](how-to/laya_tensorrt_runbook.md) | 现行（TensorRT 导出/验收 runbook） | 2026-10-17 |

下位机侧的操作步骤在各自仓，不在这里复制：

- CANopen 主站联调 / EDS 重生成：`../vehical_rak_arm/docs/QUICKSTART.md`、`../vehical_rak_motion/docs/QUICKSTART.md`
- CubeMX 改工程：`../vehical_rak_arm/docs/CUBEMX-SOP.md`

**Reference**

| 文档 | 状态 | 最后更新 |
|---|---|---|
| [`hardware_ownership.md`](hardware_ownership.md) | 现行（机器分工，本机有没有硬件的唯一权威） | 2026-11-16 |
| [`reference/arm_moveit_contract.md`](reference/arm_moveit_contract.md) | **冻结**（2026-10-09，MoveIt 侧唯一接口） | 2026-10-28 |
| [`reference/canopen_bitfields.md`](reference/canopen_bitfields.md) | 现行（v5 位域与身份表） | 2026-11-09 |
| [`reference/firmware_capability_spec.md`](reference/firmware_capability_spec.md) | **提案**（待两块下位机仓签字） | 2026-11-09 |
| [`reference/hardware_acceptance.md`](reference/hardware_acceptance.md) | 现行（顶部有过期横幅，按横幅读） | 2026-10-14 |
| [`reference/machine_baseline.md`](reference/machine_baseline.md) | 现行（Orin 基线，可一键回滚） | 2026-11-12 |
| [`reference/imu.md`](reference/imu.md) | 现行（板载 LSM6DSRTR / I2C 驱动与话题） | 2026-11-16 |
| [`reference/cameras.md`](reference/cameras.md) | 现行（Astra S + Gemini 335L 双 backend，Orbbec udev / IMU 话题已记录） | 2026-11-16 |
| [`reference/microphone_array.md`](reference/microphone_array.md) | 现行（2026-10-10 实测记录） | 2026-10-30 |
| [`reference/perf/ipc_baseline.md`](reference/perf/ipc_baseline.md) | 实测记录 | 2026-10-14 |
| [`reference/perf/cross_host_dds.md`](reference/perf/cross_host_dds.md) | 实测记录 | 2026-11-09 |

**Explanation**

| 文档 | 状态 | 最后更新 |
|---|---|---|
| [`explanation/architecture.md`](explanation/architecture.md) | 现行（含历史章节，读时看日期） | 2026-11-09 |
| [`explanation/migration_2026-10.md`](explanation/migration_2026-10.md) | 历史（旧名 → 新名对照表仍有效） | 2026-10-17 |

**Design（未落地）**

| 文档 | 状态 | 最后更新 |
|---|---|---|
| [`design/vision_nav_v1.md`](design/vision_nav_v1.md) | **草案**（未接受，落地前需 ADR） | 2026-11-15 |
| [`design/vision_line_follow_v1.md`](design/vision_line_follow_v1.md) | **草案**（纯视觉寻线；含 2024 论文遗留删除清单，待评审） | 2026-11-16 |
| [`design/vision_nav_v2.md`](design/vision_nav_v2.md) | 已落 ADR-0027（4D 融合全栈；真机未验证） | 2026-11-15 |
| [`design/brain_arm_tools.md`](design/brain_arm_tools.md) | **草案**（未接受，落地前需 ADR） | 2026-10-17 |
| [`design/chassis_control.md`](design/chassis_control.md) | **提案**（等 F11/F1 前置闭合） | 2026-10-30 |
| [`design/grasp_perception.md`](design/grasp_perception.md) | 草案 / 分支实现 | 2026-10-14 |
| [`design/2026-10-03-rak-moveit2-integration-design.md`](design/2026-10-03-rak-moveit2-integration-design.md) | 设计记录 | 2026-10-03 |
| [`design/2026-10-07-rak-arm-moveit2-design.md`](design/2026-10-07-rak-arm-moveit2-design.md) | 设计记录 | 2026-10-07 |
| [`design/2026-10-08-rak-wbc-hqp-design.md`](design/2026-10-08-rak-wbc-hqp-design.md) | 设计记录（ADR-0011 输入） | 2026-10-08 |

**Status**

| 文档 | 状态 | 最后更新 |
|---|---|---|
| [`status/project_status.md`](status/project_status.md) | 现行（活文档，增量追加不追改） | 2026-11-12 |
| [`status/next_2026-10-30.md`](status/next_2026-10-30.md) | **已过期**（2026-10-30 施工单，留档） | 2026-10-30 |
| [`status/grasp_validation.md`](status/grasp_validation.md) | 实测记录（2026-10） | 2026-10-14 |
| [`status/canopen_vcan_2026-11-14.md`](status/canopen_vcan_2026-11-14.md) | 实测记录：无下位机时 ros2_canopen 主站的 vcan 验证 | 2026-11-14 |

**Archive**

| 文档 | 说明 |
|---|---|
| [`archive/handoff_agent_2026-10-09.md`](archive/handoff_agent_2026-10-09.md) | 2026-10-09 交接快照，已被 `machine_baseline.md` 与 `README.md` 取代 |
| [`archive/2026-10-03-rak-moveit2-integration.md`](archive/2026-10-03-rak-moveit2-integration.md) | 已执行的实施计划 |

**ADR**：索引在 [`adr/README.md`](adr/README.md)，编号 0001–0026。

## 3. 写作规范

适用于本目录所有新文档与改稿。

1. **一份文档只有一个象限。** 想同时当教材和参考，就拆成两份并互相链接。
2. **状态行必须存在。** 每份 `docs/**` 文档顶部（H1 之后）写一行
   `> 状态：现行 | 冻结 | 提案 | 已过期 | 存档 · 最后更新：YYYY-MM-DD`。
   改内容必须同时改日期；过期不删，标过期。
3. **单一事实源。** 同一事实只写一处，别处给链接。协议/对象字典的事实源在两块下位机仓，
   本仓只写"怎么用"，不写"是什么"。
4. **区分事实与推断。** 实测写"实测 + 日期 + 命令"，推断写"待核"，不要写成结论。
   没跑过的验收一律写"未验证"。
5. **相对链接。** 仓内互链用相对路径（`../reference/xxx.md`）；ADRs 用 `adr/` 相对路径。
6. **标题与文件名。** 文件名 `snake_case.md`；标题说结论，不写"关于…的说明"。
7. **代码与命令**给完整可复制的形式，注明在哪个目录、依赖哪个 `source`。
8. **不写重复进度表。** 进度只在 `status/project_status.md` 与 `TODO.md`，其他文档只链接。
9. **语言**：中文叙述、英文标识符、SI 单位。

## 4. 门禁

```bash
python3 scripts/check_docs.py        # 链接 + 索引覆盖 + 状态行
```

检查三件事，任一失败即非零退出：仓内 Markdown 相对链接全部可达；`docs/**`
（`adr/` 除外）每份文档都在本索引里出现；每份文档有状态行。
改文档与改代码同一批提交。

## 5. 本次重组对照（2026-11-14）

| 旧路径 | 新路径 |
|---|---|
| `docs/architecture.md` | `docs/explanation/architecture.md` |
| `docs/migration.md` | `docs/explanation/migration_2026-10.md` |
| `docs/CONSTRUCTION.md` | `docs/tutorials/construction.md` |
| `docs/laya_tensorrt.md` | `docs/how-to/laya_tensorrt_runbook.md` |
| `docs/arm_moveit_contract.md` | `docs/reference/arm_moveit_contract.md` |
| `docs/canopen-bitfields.md` | `docs/reference/canopen_bitfields.md` |
| `docs/cameras.md` | `docs/reference/cameras.md` |
| `docs/firmware-capability-spec.md` | `docs/reference/firmware_capability_spec.md` |
| `docs/hardware_acceptance.md` | `docs/reference/hardware_acceptance.md` |
| `docs/machine-baseline.md` | `docs/reference/machine_baseline.md` |
| `docs/microphone_array.md` | `docs/reference/microphone_array.md` |
| `docs/perf/*` | `docs/reference/perf/*` |
| `docs/brain_arm_tools_design.md` | `docs/design/brain_arm_tools.md` |
| `docs/chassis-control-design.md` | `docs/design/chassis_control.md` |
| `docs/grasp_perception.md` | `docs/design/grasp_perception.md` |
| `docs/superpowers/specs/*` | `docs/design/*` |
| `docs/superpowers/plans/*` | `docs/archive/*` |
| `docs/project_status.md` | `docs/status/project_status.md` |
| `docs/NEXT-2026-10-30.md` | `docs/status/next_2026-10-30.md` |
| `docs/grasp_validation.md` | `docs/status/grasp_validation.md` |
| `docs/HANDOFF-AGENT.md` | `docs/archive/handoff_agent_2026-10-09.md` |

`docs/adr/` 位置不变（ADR 编号是全仓链接锚点）。本次只机械修正了 ADR 里的链接路径，
决策正文未改。

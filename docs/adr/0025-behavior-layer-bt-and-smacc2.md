# ADR-0025: 行为层分两级 —— BT.CPP 做技能，SMACC2 做任务（分阶段落地）

- 状态: 已接受（**阶段 1 未实现**；本文只定结构与边界，不声称已交付代码）
- 日期: 2026-10-29
- 相关: `src/rak_task_executor/`（现任务状态机）· `src/rak_harness`（仲裁缝）·
  `docs/brain_arm_tools_design.md` · ADR-0010（Harness）· ADR-0011（WBC 单写者）·
  ADR-0020（生命周期与单一所有者）

## 背景

用户要求把行为层升级为 **SMACC2 + BehaviorTree.CPP**，并要"并行控制 + 行为全面升级"。
现状是：`rak_task_executor/node.py` 786 行手写状态机（`IDLE→ACCEPTED→PLANNING→
NAVIGATING→…→SAFETY_HOLD`），所有转移写在一个 `_execute()` 里，恢复逻辑靠 `if`。

三条硬约束决定了不能"直接引入框架"：

1. **SMACC2 不在 apt**：它是源码仓（含代码生成器与 `cl_lely`/`smacc2_msgs` 等依赖），
   4 GB Orin 上编译一次的成本与风险都不可忽略；今天装不上就上不了台架。
2. **单一权威源**：ADR-0020 §3 已经裁定"一个组件恰好一个生命周期拥有者"，
   ADR-0013 裁定"任务生命周期唯一所有者 = `rak_task_executor`"。
   再叠一层状态机而不撤销旧权威源，就会同时有两个"任务现在到哪一步"的答案。
3. **安全语义不能下放**：`allow_motion` 与 `guard()` 是唯一运动许可（ADR-0020 §6），
   行为树/状态机**不得**获得直接发 `DRIVE_CMD` 的能力。

## 决策

1. **两级分工，各自只做一件事**：

   | 层 | 负责 | 载体 | 不负责 |
   | --- | --- | --- | --- |
   | 技能层 | 单个可复用动作（导航到点、抓取、说一句、等待、并行/重试/超时） | **BehaviorTree.CPP** 节点库 + XML 树 | 任务级状态、恢复策略、权限 |
   | 任务层 | 任务的生命周期、并行分支、失败恢复、对外 Action 契约 | **SMACC2** 状态机（阶段 2） | 具体动作实现、运动许可 |

2. **对外契约不变**：`/task/execute`（`rak_interfaces/action/ExecuteTask`）与
   `/task/status` 保持不变。`rak_task_executor` 降级为**门面**：收 goal → 驱动任务引擎 →
   回报 result。这样 `harness`/`tool_executor`/Nav2 全部零改动。

3. **阶段 1（可今天落地，不依赖 SMACC2）**：
   - 新增 `src/rak_behaviors`：BT.CPP 节点库（`NavigateToPose` / `Say` / `ToolCall` /
     `Wait` / `SetBlackboard`）+ XML 树；`find_package(behaviortree_cpp QUIET)` 守卫，
     装不上就**不构建**（而不是构建失败）。
   - `rak_task_executor` 增加 `engine:=sm|bt` 参数：`sm` = 现有状态机（默认，行为不变），
     `bt` = 用 XML 树跑同一批技能。两条路径共用同一个 Action 门面与同一套安全检查。
   - 先在一个技能子集上跑通（导航 + 说话 + 工具调用），验证"树能表达并行/超时/重试"。

4. **阶段 2（SMACC2，需先解决可安装性）**：
   - 新增 `src/rak_mission`（`find_package(smacc2 QUIET)` 守卫），状态：
     `Idle / Navigating / Manipulating / Recovering / Halted`，正交组件
     `OrSafety`（订阅 `/diagnostics` 与 `allow_motion`）。
   - 落地前必须先撤销 `rak_task_executor` 里的状态机（**同一个转移只能有一个实现**），
     否则违反 ADR-0020 §3。撤销的方式是"门面转发事件给 SMACC2"，不是两套并存。

5. **并行控制的语义写在树里，不写在 C++ 里**：`Parallel` / `RoundRobin` /
   `RetryUntilSuccessful` / `Timeout` 全部用 BT.CPP 内建节点表达；自定义节点只做
   "与 ROS 交互"这一件事（发 Action、读话题、调服务）。

## 理由

- BT 擅长"一个任务内部怎么组合动作"，SMACC2 擅长"任务之间的状态与事件"。
  把两件事塞进一个框架，两边都会别扭；分层之后各自都能用现成件。
- 阶段 1 不引入不可安装的依赖，明天就能在真机上验证"树能不能表达我们要的并行与恢复"，
  这个结论会反过来决定阶段 2 的状态机该长什么样。
- 保持 Action 契约不变，使得任何阶段的回滚都只是换一个 `engine` 参数。

## 后果

**正面**

- 技能可复用、可在 RViz/Groot 里可视化（BT.CPP 的 XML + Groot2）；
- 恢复策略从"手写 if"变成"树的形状"，评审时看得见；
- 与 Nav2 的 BT 生态同源（它也吃 BehaviorTree.CPP），不会出现两套树语义。

**负面 / 代价**

- 阶段 1 期间存在两套任务执行路径（`sm` / `bt`），必须靠参数与测试保证只有一套生效；
  这也是为什么默认值保持 `sm`。
- BT.CPP 的 XML 与节点注册需要编译期插件登记，桌面档若没装 `behaviortree_cpp`
  就完全测不到——只能靠"守卫构建 + XML 静态校验"兜底。
- SMACC2 的代码生成器会引入一堆生成文件，git 里要明确哪些是生成物
  （否则 review 会被生成代码淹没）。

## 被否决的方案

| 方案 | 否决理由 |
| --- | --- |
| 直接用 SMACC2 替换 `task_executor`（一步到位） | 装不上就什么都没有；且会同时存在两个任务权威源，违反 ADR-0020 §3 |
| 只用 BT.CPP，不要任务层 | "任务之间的状态/事件/恢复"用树表达会退化成巨型树，Groot 里读不懂 |
| 只用 SMACC2，不要 BT | 每个技能都要写一个 state class，复用性差；Nav2 已经是 BT，等于两套语义 |
| 自己写一个轻量状态机框架 | 就是现在这 786 行的来路；换个名字不会变好 |

## 待闭合项

1. **阶段 1 代码未交付**：`src/rak_behaviors` 与 `engine:=bt` 尚未实现。
   本轮只完成边界与迁移路径的裁定（以及生命周期/观测/时间基准这些前置项）。
2. **SMACC2 可安装性**：需要在 Orin 上确认 Jazzy 分支能否编译（依赖 `smacc2_msgs`、
   `cl_ros2_toolbox` 等），以及编译产物对 4 GB 内存的影响。
3. **`engine` 参数的验收**：两条路径必须产出**逐字段相同**的 `/task/status` 序列，
   否则"换引擎"会变成"换行为"。这条要有回归测试才允许切默认值。
4. **Groot2 的 XML 校验**进 CI：XML 拼错在运行期才报，代价太高。

> 本节的执行顺序、命令与判据见 [`docs/NEXT-2026-10-30.md`](../NEXT-2026-10-30.md) §9 行为层。

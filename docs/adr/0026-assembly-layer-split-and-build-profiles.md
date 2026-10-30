# ADR-0026: 组装层拆成「硬件全栈」与「契约层」，构建档位变成数据

- 状态: 已接受
- 日期: 2026-10-29
- 相关: `src/rak_orchestration/`（新建）· `src/rak_bringup/` ·
  `src/rak_orchestration/config/build_profiles.yaml` ·
  `src/rak_orchestration/config/launch_contracts.yaml` ·
  `scripts/build_workspace.py` · `scripts/check_workspace.py` ·
  `src/rak_orchestration/test/test_launch_contracts.py`

## 背景

`rak_bringup` 同时承担了两件性质相反的事：

1. **硬件全栈组合**——拉起 Astra/雷达/双目/底盘控制链，这些包只在 Orin 上存在；
2. **组装契约**——系统清单、故障域、launch 与谁归属哪一档。

它把 GPU/Jetson/厂商包全部写成 `exec_depend`，于是"桌面机能建"这件事**没有载体**。
实际表现（用户 2026-10-29 现场）：桌面机必须手打
`colcon build --packages-ignore rak_astra_driver … rf2o_laser_odometry` 才能通过，
而且 `check_workspace --installed` 永远红——它无条件遍历所有包。

那份命令行是一条**不可复现的隐性依赖**：换人、换机、换一天就漏一两个包，而漏掉的
表现是"launch 起不来"，不是"依赖没装"。同一类问题还有一个变体：
`check_workspace --installed --skip-absent` 会在失败后从报错字符串里**猜**哪个包缺了，
猜对是运气，猜错就把真正的语法错误当成"缺包"跳过。

硬约束：

- 桌面机（x86，无 GPU/JetPack/相机）必须能**一条命令**建起来并跑结构门禁；
- Orin 上仍然要有"全量"这一档，且两档的口径必须来自同一份事实；
- 不许把 `rak_bringup` 的硬件组合拆散——ADR-0013 定的"唯一组装层"原则不变，
  变的只是**组装层内部的分工**。

## 决策

1. **新增 `src/rak_orchestration`（契约层，纯 x86 可建）**，内容只有：
   - `config/system_manifest.yaml` + `system_manifest_fixture.yaml`（故障域/所有权/判据）；
   - `config/build_profiles.yaml`（档位：跳过哪些包、容忍哪些包缺失）；
   - `config/launch_contracts.yaml`（每个 launch 归哪一档、缺什么、为什么）；
   - `launch/supervisor.launch.py`（编排入口，不 import 任何硬件包）；
   - `rak_orchestration.profiles` / `launch_helpers`（档位解析、受管节点 launch 助手）。
   它**不得**依赖任何 `rak_gpu` / 厂商 / GPU 包——由 `check_workspace` 断言。

2. **`rak_bringup` 只留硬件全栈组合**：传感器/底盘/导航/任务/语音的 launch 与参数。
   它 `exec_depend` 契约层，并在 `robot.launch.py` 里**只 include 一次**
   `supervisor.launch.py`（避免第二个生命周期权威源）。

3. **档位是数据，不是命令行**。`build_profiles.yaml` 定义两类缺口：

   | 键 | 语义 | 门禁行为 |
   | --- | --- | --- |
   | `skip_packages` | 本档**不构建也不需要**（GPU/厂商/仅 Jetson） | 引用它的 launch 必须声明非本档，否则红 |
   | `tolerate_absent` | 可能装也可能没装（apt 可选件、源码构建件） | 允许运行时缺失，但**逐项打印** |

4. **launch 归属必须声明**。`launch_contracts.yaml` 里每条记录
   `package/name/profiles/requires/reason`；门禁钉三条不变量：
   引用跳过包的 launch 必须声明且不含 `desktop`；声明的 `requires` 必须与本档跳过表
   有交集（防过期豁免）；声明的 launch 必须真实存在。

5. **一条命令**：`python3 scripts/build_workspace.py --profile desktop`（或 `jetson`）。
   它把档位翻译成 `colcon build --packages-skip …`，构建完立刻跑
   `check_workspace.py --installed --profile <name>`。

6. **`--skip-absent` 降级为兼容开关**（等价 `--profile desktop`），不再靠报错字符串
   猜包名：缺口先查表、再执行。

## 理由

- 拆分点是**依赖方向**而不是文件名：清单/档位/编排入口不 import 硬件，天然可 x86 建；
  硬件 launch 必须 import 厂商包，天然只能在 Orin 建。沿这条缝拆，两边都不需要特判。
- 把豁免写成数据之后，"为什么这台机器跑不了这个 launch"有了唯一答案，
  而不是每个人在自己机器上维护一份 `--packages-ignore`。
- 门禁与构建共用同一份 YAML，避免"文档说能建、命令建不起来"的漂移。

## 后果

**正面**

- 桌面档一条命令成立；`check_workspace` 从"永远红"变成可用门禁。
- 新增硬件包时，忘了声明档位会被门禁抓住（而不是等某台机器构建失败）。
- 编排入口可在桌面机跑 `profile:=fixture`，端到端验证编排本身。

**负面 / 代价**

- 多一个包与一层间接；`rak_bringup` 与 `rak_orchestration` 的边界要靠门禁维持，
  否则会重新长回去（`check_assembly_layers` 断言清单/编排入口不得留在 bringup）。
- 档位表是人工维护的：新增厂商包必须同时改 `skip_packages` 与 `launch_contracts`，
  漏改的表现是桌面档门禁红（可接受，至少是显式的）。
- `tolerate_absent` 会让"本机没装"与"真的坏了"在输出上长得像，
  所以它**必须**逐项打印——这条靠代码约束，不靠自觉。

## 被否决的方案

| 方案 | 否决理由 |
| --- | --- |
| 只加 `--skip-absent`，不拆包 | 那就是今天的补丁：跳过逻辑靠猜、口径不可复现；且编排入口仍被硬件包绑在 Orin 上 |
| 用 `package.xml` 的 `condition` 属性做条件依赖 | 没有标准环境变量可判"桌面/机器人"，等于把档位藏进 shell 环境 |
| 拆成 `rak_bringup_hardware` / `rak_bringup_logic` 两个硬件包 | 命名会让人以为有两个组装层，违反 ADR-0013 的"唯一组装层"表述 |
| 用 `COLCON_IGNORE` 文件切换档位 | 会污染工作树（`git status` 变脏），且切回来容易忘 |

## 待闭合项

1. `colcon` 的 `defaults.yaml` 尚未引入；目前档位只在 `build_workspace.py` 里生效，
   直接 `colcon build` 仍是全量。是否引入由用户定（会让 `colcon build` 行为依赖 cwd）。
2. 桌面档的 `ros2 launch --show-args` 只验证"能解析"，不验证"节点能起来"；
   后者需要夹具档（`supervisor.launch.py profile:=fixture`）在 CI 里跑。
3. `tolerate_absent` 的清单是经验值，缺一次实测：哪些 apt 包在 Jetson 镜像里是默认有的。

> 本节的执行顺序、命令与判据见 [`docs/NEXT-2026-10-30.md`](../NEXT-2026-10-30.md) §2 结构门禁（含 desktop 档验收）。

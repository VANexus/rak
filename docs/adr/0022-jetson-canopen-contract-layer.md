# ADR-0022: Jetson 侧 CANopen v5 架构 —— 契约层与协议栈分离，SSOT 单向，混跑可计算禁止

- **状态**: 已接受（设计 + 契约层实现）；**生产主站实施 = P3 未开始**
- **日期**: 2026-10-29
- **相关**: ADR-0021（v5 决策本体）、ADR-0020（生命周期与故障域）、伞形仓
  `docs/CANOPEN-V5.md`（规范件）、`docs/superpowers/specs/2026-10-29-can-v5-canopen-design.md`
  （对象表与端口表）、`src/rak_canopen/`（本决策的实现）、`tools/canopen_gate.py`（MCU 侧门禁）
- **不取代**: ADR-0021。本 ADR 只管**Jetson 侧怎么接**，协议内容仍由 ADR-0021 与规范件决定。

---

## 背景

ADR-0021 把下位机换成 CANopen，但没有回答上位机怎么接。若按 v4 的老路子中文化重写一遍
（`can_session.cpp` + `can_decode.cpp` 那种手写编解码），就正好复发 ADR-0021 记录的原罪：
**帧表在三处复制**，历史上出过量化因子错 10 倍（R62/R63）与双板同 ID 双发。

今天还有两个硬约束：**没有实机**（明天才测），**Orin 不可达**（本机隧道断了）。
所以能推进的部分只有一类：把"总线上应当发生什么"变成**可以从 SSOT 派生、并且能离线证伪**的东西。

## 决策

1. **契约层 ≠ 协议栈**。生产链路的 NMT/SYNC/PDO/SDO 一律由 **lely-core** 承担（ADR-0021 已选，
   本 ADR 不重开选型）。`src/rak_canopen` 只做派生与校验：COB-ID、PDO 字节布局、速率、
   量化范围、活性判据、混跑冲突。它不许成为第二套协议实现。
2. **SSOT 单向**。对象清单由 MCU 仓维护（`docs/canopen/<node>.json`），rak 侧存副本并记录
   `sha256`（本仓副本实测 `chassis d63beea8251da5c1…` / `arm 749bd88db2ac22e6…`）。
   待闭合：把这两个哈希纳入 `canopen_gate.py` 的检查项，让"三处复制"变成"一处 + 两个被校验的副本"。
3. **禁止第四份帧表**。JSON 只有 `index/name/type/access/pdo`，**不含单位与量化因子**；
   单位写在 `config/canopen_contract.yaml` 里，**每条带出处**（[V5]/[SPEC]/[A21]）。
   `contract.py` 双向校验：语义表里多一个名字、少一个名字、名字打错，装载即失败。
4. **"禁止同总线混跑 v4/v5" 变成可计算的**。v4 的 ID 从本仓 v4 后端源码里正则出来
   （`constexpr uint32_t kId…`，不抄第三份表），与派生的 v5 COB-ID 求交集，非空即失败。
   今天实测算出的冲突：**0x201**（v4 `WHEEL_POS_B` vs v5 `chassis.rpdo1`）、
   **0x202**（v4 `WHEEL_SPD` vs v5 `arm.rpdo1`）——规范件的说法由此得到算术证明。
5. **NMT 与 ROS 生命周期的映射**（`link` 域）：

   | ROS 2 生命周期 | CANopen 动作 | 就绪判据 |
   | --- | --- | --- |
   | `configure` | NMT Reset node → 停在 `Pre-operational` | 心跳 `0x7F`，且 SDO `0x2F00` 读回的 build_tag == 期望固件（R73/A14） |
   | `activate` | NMT Enter Operational | 心跳 `0x05` + 该节点 TPDO 按派生速率到达 |
   | `deactivate` | NMT Enter Pre-operational（停止收 PDO） | 板子回到"安全停止"，`cmd_en=0` |
   | `shutdown` | NMT Reset communication / node | 心跳消失 |

   **注意这条不能反过来理解**：`Operational` 只是"允许说话"，**不是运动许可**；
   运动许可仍是 `allow_motion` + `cmd_flags.bit0`（ADR-0020 §6）。
6. **`link` 域的生命周期拥有者是 lely 主站的网络管理，不是 `rak_lifecycle`**。
   本 manager 对它只 `GetState` 观测（ADR-0020 §3 单一所有者规则的直接推论）。
7. **活性与安全语义按 [V5] §5 建模为纯函数**：`cmd_seq` 必须在 **100 ms** 窗内变化 **且**
   `cmd_flags.bit0=1` **且** 节点 `Operational`，否则 MCU 侧 `enable=0`；心跳消费者超时
   **500 ms**（[SPEC]）；SYNC 由 Jetson 以 **2 ms** 产生，因此**主站自己**也是一个有周期的写入者，
   它的存活必须进健康聚合，否则"Jetson 侧 SYNC 停发"表现为所有同步 PDO 同时消失而无归因。
8. **数据面纪律**：TPDO 解码进 `ros2_control` 反馈；RPDO 由控制器输出编码，
   **越界抛错、绝不静默饱和**（v4 的"负角被截 0"必须是显式失败，`test_contract.py` 已钉）。

## 理由

- 协议交给现成栈（lely），**语义交给单一来源 + 派生**，是同时满足"现成库优先"和
  "别再抄三份"的唯一解。
- 派生而非抄表还有个副作用红利：速率是从 `tpdo_types × SYNC` 算出来的，
  所以 JSON 一旦改了发送类型，`rak` 侧的判据阈值会自动跟着变，不会出现"板子 50 Hz、
  上位机按 100 Hz 判超时"这类漂移。

## 后果

**正面**
- 明日台架第一次上总线之前，以下事实已被**离线证明**（16 项契约测试 + 7 项 EKF 契约测试全绿）：
  COB-ID 表、PDO 布局与 8 字节上限、偏移连续性、每路速率、量化范围与越界行为、
  build_tag 解码、活性三条件、v4/v5 冲突集合。
- "混跑禁止"从一句话变成 CI 可跑的断言；单位与因子第一次带出处。
- EKF 第一次有**显式协方差**（此前 `grep -c covariance` = 0）与 `print_diagnostics: true`。

**负面 / 代价**
- 语义表是**新增的一份人工维护物**：MCU 侧改了单位而这里没改，测试无法自动发现
  （除非把 `canopen_gate.py` 延伸到哈希校验 —— 待闭合项）。
- **字节序是推断，不是实测**：`byte_order: little` 依据"STM32 小端 + CANopenNode 按 OD
  内存序打包"，明日首帧必须核对；若相反，所有解码同时错，且不会报错。
- lely 的接线工作量没有因本 ADR 减少；本层只是把它需要的事实先算清楚。
- 契约副本进本仓意味着 JSON 变更要两边同步（比引用路径多一次拷贝，换来可单独 clone 构建）。
- EKF 协方差填的是**占位值**（`TODO(measured)`），明日必须以实测替换；
  占位值让 EKF "保守"，不保证正确。

## 被否决的方案

| 方案 | 否决理由 |
| --- | --- |
| 在 `rak_chassis` 里手写 CANopen 编解码（像 v4 那样） | 就是 v4 原罪；协议正确性由手写代码承担，且无法与下位机 SSOT 校验 |
| 把 JSON 直接编译进 C++（生成一份 `.h` 提交进 rak） | 生成物一旦提交就是第四份表；且改 JSON 要重编才报错，派生层的价值就没了 |
| 同总线灰度混跑（一块板先切） | 算术上 `0x201/0x202` 直接撞 ID（本 ADR 的碰撞测试），不是"注意一下就行" |
| 用 NMT `Operational` 当运动许可 | 出现第二个安全权威源，违背 ADR-0020 §6 与"Jetson 在安全链外"（ADR-0021） |
| 现在就把 lely 接进 `ros2_control` 再一起测 | 今天既无实机也无 Orin 可达，写出来的接线只能靠猜，且违反"做不到就写未验证" |

## 待闭合项

1. **字节序实测**（首帧核对 `little`/`big`）。
2. `imu_evt` / `state` / `fault` 的位定义：规范件只给了字段名，没有枚举语义（`semantics` 里已标 `todo`）。
3. `j8cmd`/`j8state`：清单叫 j8，规范件写"保留" —— 明日核对语义。
4. `arm.json` 没有 `identity` 段（chassis 有），臂的 build_tag 只有 SDO 一条路，需补进清单。
5. 把本仓两个 JSON 副本的 `sha256` 纳入 `tools/canopen_gate.py` 的检查项（与两仓 third_party 同哈希同性质）。
6. lely-core 接入的具体 API 面（NMT 主站、SYNC 产生、PDO 配置、SDO 客户端）与
   `Session` 抽象的贴合点；`link` 域在 `system_manifest.yaml` 里的整段重声明。
7. **单位/因子应当进 SSOT**（JSON 里加 `unit`/`factor`），本层随之退化为纯派生 —— 这条要
   与下位机团队一起定，不擅自动别人的清单。
8. EKF 的 F1 路（轮式里程计）仍缺席；`i2c_imu.py:240` 的时间戳仍是发布时刻（他人在制品，未动）。

## 验收

- **今日已过（离线，无需硬件/ROS）**：`rak_canopen` 契约测试 **16/16**、
  `rak_bringup` EKF 契约测试 **7/7**、`check_workspace.py` 24 包结构门禁。
  命令与输出记在提交信息里。
- **明日实机门**（未过不得称"已验证"）：首帧字节序核对；`0x2F00` 读回 R73/A14；
  TPDO 速率对齐派生值（100/50/10 Hz）；`cmd_seq` 断供 100 ms 后板子 `enable=0`；
  拔线 500 ms 内心跳判失败且只降级 `link` 域及其下游。
- 台架验收一律先用 `tools/canopen_bringup.py`（不依赖本仓），本仓链路验证只在 P3 之后。

# ADR-0003: 协议分层归属、单一来源与跨仓变更流程

- **状态**: 已接受（归属与流程）；**分歧清单待双方签字**。
  ⚠️ **2026-10-05 更新：CAN 层已失效** —— 臂板烧毁，CAN 对端不存在，Jetson 只走 USB（见 ADR-0007）。
  下表的 CAN v3 部分保留为**历史记录与未来参考**（若恢复双板则复用），当前实施只需 UART/USB 层。
- **日期**: 2026-10-05
- **相关**: `docs/protocol/*`（本仓）· `vehical_rak_chassis/docs/PROTOCOL.md` · `vehical_rak_chassis/docs/protocol-signals.yaml` · `vehical_rak_chassis/docs/gen/can_protocol.h` · `vehical_rak_chassis/docs/UART-v1-ALIGN.md` · `src/rak_base_driver` · `src/rak_can_driver` · `src/rak_serial_link`

---

## 背景

系统有**两层独立协议**，分属不同仓库，且文档存在多份副本：

| 层 | 端点 | 规范源 | 机器可读源 | 门禁 | 归属仓 |
|---|---|---|---|---|---|
| **CAN v3** | F407 ↔ F407（底盘↔臂） | `vehical_rak_chassis/docs/PROTOCOL.md` | `protocol-signals.yaml` → `docs/gen/can_protocol.h` | `tools/can_check.py` | 下位机（chassis + arm **各存一份，须同步改**） |
| **UART v1 / v1.1** | Jetson ↔ 底盘 F407 | `rak/docs/protocol/jetson_f407_uart_v1*.md` | 无（只有 golden vectors） | `scripts/serial_100hz.py`（PTY only） | 上位机（rak） |

下位机 `AGENTS.md` 已规定：改 `PROTOCOL.md` / `protocol-signals.yaml` 必须**同批**改姊妹仓
`vehical_rak_arm` 的那份，并在提交说明写明。CAN v3 已有可执行门禁
（`can_check.py`：结构校验 + 向量比对 + CRC 篡改负例 + `--c-header` 生成）。

**UART 层目前没有等价门禁**，也没有机器可读定义 —— 只有 md 里的 golden vectors。

---

## 决策

1. **归属按边界划分，不按仓库划分。**
   - **CAN v3 归下位机**：`protocol-signals.yaml` 是唯一机器可读源，`can_check.py` 是唯一门禁。
     `rak` 侧只做**消费者**，不得自行发明 CAN 帧字段。
   - **UART v1/v1.1 归上位机（`rak`）**：`docs/protocol/jetson_f407_uart_v1*.md` 是规范。
     下位机实现，但不得单方面改语义。
2. **机器可读 > 散文。** 冲突时以 `protocol-signals.yaml` / `gen/can_protocol.h` 为准，
   `PROTOCOL.md` 的散文表格是它的渲染结果，不是独立来源。
3. **版本协商必须是"宿主可降级"。** 下位机当前**只实现 v1**、`REQ_FEATURES = bit0|bit1`，
   不报 v1.1 的 `bit3`。宿主必须能自动回退 v1 并**在没有协商到 bit3 时不发 v1.1 载荷**。
   （`rak_base_driver/node.py` 已实现该分支：`pack_velocity_v1_1` vs `pack_velocity`。）
4. **跨仓协议变更走"同一批提交"。** 改 UART 层要同时动 `rak`（规范 + host 实现）与
   `vehical_rak_chassis`（固件实现），改 CAN 层要同时动 chassis + arm。**不允许单侧顺手改。**
5. **状态声明必须带日期与验证方式。** 两仓都有"另一侧还没做 X"的声明，实测已过期（见下）。
   任何"未实现/未验证"的断言要么给出 commit + 日期，要么标注为**未核实**。

---

## 理由

1. **边界即所有权**：谁实现端点，谁定义该端点的线上格式，避免"两边都在改"。
2. **CAN 已有门禁，UART 没有** —— 这是当前最大的不对称风险：CAN 改错会在 CI 抖出来，
   UART 改错要等实车。
3. **下位机固件已存在并台架实测**（`UART-v1-ALIGN.md`：build_tag R16 已烧 A 板，
   实测 RX 带宽 91514 B/s = 理论 99.3%，echo 100 Hz 零丢帧）。上位机侧的"未验证"声明已不成立。

---

## 后果

### 正面
- 变更影响面可判定：改哪层、动哪几个仓、跑哪个门禁，一目了然。
- 下位机 CAN 的门禁模式（yaml → 生成头文件 → 校验器）**可以复制到 UART 层**，
  这是 UART 层补齐门禁的现成模板。

### 负面（代价）
- **跨仓同批提交成本高**：需要两侧同时在位，且下位机文档有两份副本（chassis + arm）。
- UART 层缺机器可读源，**golden vectors 只能人工比对**，容易漂移。
- 归属划分意味着改一个字段可能要动 3 个仓（rak + chassis + arm）。

---

## 被否决的方案

| 方案 | 否决理由 |
|---|---|
| 把两层协议都收进 `rak` 仓统一管理 | 下位机固件必须能独立编译/自检；把 CAN 定义搬走上位机会破坏 `can_check.py` 的离线门禁 |
| 用单一"总协议"文档合并 CAN 与 UART | 两层端点、速率、语义完全不同；合并会让"改一处影响另一处"变得不可见 |
| 只靠散文 md，不做机器可读源 | CAN 层已经证明机器可读 + 门禁才拦得住"顺手改"；UART 层的漂移正是这么来的 |

---

## 待闭合项

### A. 已核实的分歧（2026-10-05）

| # | 分歧 | 上位机侧 | 下位机侧 | 处置 |
|---|---|---|---|---|
| A1 | **lease 时长** | 规范 `[10, 50] ms` | 实发 **200 ms**（`UART-v1-ALIGN.md` 请求签字；50 ms 必触发超时锁存） | **需团队批准 200 ms 为过渡值**；`node.py` 的 lease 处理要兼容。**先查 `latency_timer`（默认 16 ms，见 `CONSTRUCTION.md` §6.1）再签字** —— 外部研究认为 200–500 ms 看门狗本属正常范围，但 16 ms 的 USB 延迟会放大抖动，可能是 50 ms 不达标的真因 |
| A2 | **v1.1 支持** | `jetson_f407_uart_v1.1.md` 详述 | **暂不实现**，只报 bit0\|bit1 | host 必须回退 v1（已实现）；文档要标注"v1.1 未上线" |
| A3 | ~~Jetson 是否在 CAN 总线上~~ | `architecture.md`：**不接 CAN** | `PROTOCOL.md` §10.3 曾要求新增 CAN 节点 | ✅ **已裁定（2026-10-05 用户确认）：Jetson 只通过 USB 与下位机通信，不接 CAN。** CAN 层随臂板烧毁而失效 |
| A4 | **Nav2 控制器选型** | 已用 `nav2_mppi_controller` + `Omni` | §10.2-2 仍写"必须换 `nav2_mecanum_controller`" | 下位机文档过期，需更新 |
| A5 | **上位机"未验证"声明** | `project_status.md`：无匹配固件、硬件未验收 | 固件已实现并台架实测（R16/R17/R18） | 上位机文档过期，需更新 |
| A6 | **下位机 §10.3 Jetson 待办** | — | 说 rak 有 3 处 "CAN FD"、叶子节点型号错 | **实测已不存在**，该待办本身过期 |
| A7 | **TJA1050 电平** | 未处理 | §10.3-5 警告：TJA1050 是 5 V 器件，RXD 直连 Orin 3.3 V 域有风险 | **上电前必须量 RXD 对地 ≤3.6 V**，或换 SN65HVD230 |
| A8 | **UART 层无机器可读源** | golden vectors 只在 md | 有 `uart_v1_test.py` 镜像 | 建议仿 CAN：抽 `uart-signals.yaml` + 生成头 + 校验器 |

### B. 结构性待办

1. **仓库状态冲突**（下位机 §10.3 提出）：两仓 HEAD 曾分叉且都未 push。
   当前 `rak` 已在 `b965c9c` 与 `origin/main` 同步、工作区仅 4 个未跟踪文件（2026-10-05 核实）。
   **动协议前先确认两侧 HEAD 与 push 状态**，否则会互相覆盖。
2. **UART 层门禁缺失**：需要一个不依赖实车的校验器（复用 CAN 的 yaml→header→check 模式）。
3. **`max_vy_m_s` 默认 0.0**（`node.py:71`）使 vy 被限成 0 且无法 ARM。
   按 ADR-0007 迁移到 `ros2_control` 后，此项由 `mecanum_drive_controller` 的 `linear.y` 限幅取代。
4. **CAN 引脚不在 `jetson-io` 可配范围**（下位机实测 grep 无命中），具体针脚需查载板手册；
   且 `can0` 存在但 DOWN（mttcan `c310000`，clock 50 MHz，**硬件支持 FD 但本项目禁用**）。
5. **下位机已知未解现象**：开机起持续 ~2000 B/s 线级噪声（来源未明）；
   DMA 收据改造（R19）已回滚。

---

## 参考

- `vehical_rak_chassis/docs/PROTOCOL.md` §9（一致性测试向量与门禁）、§10（已定决定与待办）
- `vehical_rak_chassis/docs/UART-v1-ALIGN.md`（R16 七项确认 + R17/R18 实测 + lease 偏离）
- `vehical_rak_chassis/AGENTS.md`（硬性约束、跨仓同批提交规定）
- `rak/docs/protocol/*`、`rak/docs/architecture.md` §4/§7、`rak/src/rak_base_driver`

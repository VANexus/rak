# ADR-0021: 下位机 CAN 换成 **v5 = CANopen**（CiA 301 + CiA 304）；`rak` 侧 v4 主站暂时失效，P3 换 lely-core

- **状态**: 已接受（2026-10-29）—— **`rak` 侧实施状态 = 未开始（真机链路当前不可用）**
- **日期**: 2026-10-29
- **相关**: 伞形仓 `docs/CANOPEN-V5.md`（规范件）、
  `docs/superpowers/specs/2026-10-29-can-v5-canopen-design.md`（设计与实测进度）、
  `vehical_rak_motion` R73 / `vehical_rak_arm` A14（已切 v5）、`src/rak_chassis`（仍是 v4）
- **取代**: **ADR-0012（CAN v4 为主链路）与 ADR-0016（v4 唯一传输）的"协议内容"部分**。
  仍然有效的部分：`Session` 抽象、上层禁分支传输、协议事实源在下位仓、CAN 是唯一主链路。
- **部分作废**: `docs/PROTOCOL.md`(v4) 与 `docs/protocol-signals.yaml` 降级为**回滚对照**，不再演进。

---

## 背景与决策

v4 是自研帧表 + 手写编解码：帧表在三处复制（YAML / 两端 C++ 与 C），历史上出过量化因子错 10 倍
（R62/R63）与双板 `0x210/0x211` 同 ID 双发这类问题；且**没有网络管理、没有安全通信、没有一致性工具**，
无法作为功能安全认证的对象。

决策：下位机整体换成工业标准 **CANopen**（现成栈，不手写协议）：

| 项 | v5 |
| --- | --- |
| 应用层 | CANopenNode v4（Apache-2.0，CiA 301：NMT/Heartbeat/EMCY/SDO/SYNC/LSS）+ 官方 STM32 移植层 |
| 安全层 | CiA 304 SRDO（EN 50325-5，IEC 61784-3 黑通道）—— P2 待接 |
| 节点 | Jetson = NMT master（node 127）、底盘 = node 1、机械臂 = node 2 |
| 周期 | SYNC 2 ms；PDO 100/50/10 Hz（实测 3 s：300/150/30 帧） |
| 单一来源 | `docs/canopen/<node>.json` → EDS → **CANopenEditor 官方导出器**生成 `OD.c/h`（不手写） |
| 门禁 | `tools/canopen_gate.py`（清单↔EDS↔OD 逐字一致、ODList 升序、PDO 禁用位、两仓 third_party 同哈希）+ `tools/canopen_bringup.py`（台架速率/SDO 断言） |

## 对本仓（`rak`）的直接影响 —— 必须知道

1. **`rak_chassis` 的 `CanSession`/`can_decode` 说的是 v4 帧（`0x010/0x100/0x200…`），
   对现在的 R73/A14 板子完全无效**：板子上电后停在 Pre-operational，除非 `rak` 变成 CANopen
   NMT 主站并改说 PDO，否则 ros2_control 拿不到任何反馈、也发不出命令。
   **禁止**用 `cansend` 手搓 v4 帧去"验证下位机坏了"。
2. 台架验证下位机 v5 用伞形仓/MCU 仓里的 `tools/canopen_bringup.py`（原始 SocketCAN，不依赖本仓）。
   当前用它实测：心跳 `05`、TPDO 速率全中、SDO `0x2F00` 读回 `"R73"`、`cmd_en` 语义正确。
3. **禁止同一总线混跑 v4 与 v5**：`0x100`（v4 DRIVE_CMD / v5 TIME）、`0x200/0x201`
   （v4 WHEEL_POS / v5 RPDO1/RPDO2）等直接撞 ID。要么全 v5，要么全 v4 回滚
   （下位机 `cmake -DAPP_USE_CANOPEN=OFF`）。
4. 认证边界不变：**Jetson 在安全链外**。急停/安全停止由两块 MCU + 硬线回路承担，
   `rak` 的指令按非安全处理；这条是 v5 能谈 SIL 2 通信能力的前提。

## 后果与待办（P3）

- 新增 `rak_canopen`（或把 `rak_chassis` 的后端换成 CANopen）：用 **lely-core**（Apache-2.0，
  现成 CANopen 栈）实现 NMT 主站 + SYNC 产生者 + PDO 映射 + SDO 客户端；对上层保持
  `Session` API 不变（`set_wheel_command`/`set_arm_command`/`SessionSnapshot`）。
- 参数标定改走 SDO（`0x2007` 轮控增益 / `0x2107` 关节配置等），不再自造串口配置协议。
- `rak/scripts/can_*.py`（v4 冒烟/回归脚本）在 P3 前标注"仅 v4 回滚分支可用"。
- 建 `docs/reference/hardware_acceptance.md` 的 v5 版验收清单（速率、`cmd_en`、SRDO 超时、故障注入）。

# ADR-0012: 传输层 Session 抽象 —— CAN v4 唯一主链路，USB v2 冻结备用

- **状态**: 已接受（2026-10-09）
- **日期**: 2026-10-09
- **相关**: `src/rak_hardware`（`session.hpp` / `can_session` / `serial_session`，本 ADR 落地后更名 `rak_chassis`，见 ADR-0013）·
  `../vehical_rak_chassis/docs/PROTOCOL.md`（v4，协议事实源）· `../vehical_rak_chassis/docs/ROS2-CONTROL-ARCH.md` ·
  ADR-0007（部分取代）· ADR-0011
- **取代**: ADR-0007 的"USB 是唯一 Jetson↔MCU 链路、CAN 总线无对端已死"决策；其余内容（单 F407、
  麦轮 IK 在上位机、Jetson IMU 喂 EKF）**继续有效**。

---

## 背景

- ADR-0007 时点（2026-10-05）：机械臂板烧毁退役，CAN 总线失去对端，USB 是 Jetson↔F407 的唯一链路。
- 2026-10-09 联调实测（下位仓提交 `d6179bc`→`67d75a2`）：SN65HVD230 外置收发器恢复 CAN 物理层。
  联调根因有二——**接线 H/L 极性对调**、**两节点 GND 未互联**（差分共模参考缺失，表现为
  "偶发收得到、大量 bus-off"）。协议文档 §1 已把两者列为上电检查项。
- 协议 v4 定稿（`vehical_rak_chassis/docs/PROTOCOL.md`，2026-10-09，"不再开洞"）：
  1. **CAN 是唯一主链路**：安全门、4 轮速指令、7 轴舵机（6 关节 + 1 夹爪）、全量遥测全部走 CAN；
  2. **USB v2 串口降级为备用链路**，帧格式冻结不变；
  3. v3 失效退役：单节点拓扑，底盘↔机械臂互锁层（`XFER_*`）整体作废；
  4. 控制契约沿用 ros2-control 架构：4 轮速 + 7 轴位置，麦轮 IK 在 Jetson 侧。
- 上位机已随协议落地（本仓 `45383e6`/`89fe7fe`）：`rak_hardware` 内新增 `Session` 抽象基类 +
  `SerialSession`（USB v2）/ `CanSession`（SocketCAN）双实现，`transport:=usb|can` 后端工厂；
  `RakSystemHardware` 与上层 ros2_control 完全无感切换。`SessionSnapshot` 两后端单位约定一致
  （position/velocity = rad，FL/FR/RL/RR），线程安全契约（控制线程 set_*、IO 线程收发）已写入头文件。

## 决策

1. **CAN v4 是唯一主链路；USB v2 是冻结的备用链路。任一时刻只有一条活跃。**
2. **传输缝收敛在 `rak_chassis`（原 `rak_hardware`）包内的 `Session` 抽象之后**：
   上层（ros2_control 控制器、`rak_wbc`、Nav2、任务层）只依赖 Session API 与 ros2_control 的
   state/command 接口契约，**禁止在任何上层代码里判断或分支传输类型**。
3. **协议事实源在下位仓**：`vehical_rak_chassis/docs/PROTOCOL.md` v4 + `protocol-signals.yaml` +
   `tools/can_check.py`。本仓不复制协议正文，代码注释只引用路径（ADR-0003 的归属不变，v3→v4）。
4. **切换传输 = 改一个参数**：`hardware_parameters` 的 `transport` 字段（`usb|can`），零上层改动、
   零控制器改动。`SessionSnapshot` 的单位与语义保证两后端可互换。
5. CAN 链路配置（mttcan `bitrate 1000000 sample-point 0.810 restart-ms 100`、`fd off`、120 Ω ×2、
   共地）是部署项，写入 `docs/CONSTRUCTION.md`，不进代码。

## 理由

- ros2_control 的 `hardware_interface` 天然就是传输缝：控制器只认接口名与单位，不认线。
  把双后端收敛在插件内部的工厂处，比两条独立节点链路（旧 `rak_can_driver` 路径）少一层
  语义漂移，也让 ADR-0007 的"统一走 ros2_control"继续成立。
- "消息协议冻结，接口冻结"（工作空间分层原则，见 ADR-0013 决策 1）要求：换传输、换 SDK、
  换器件不得波及上层。Session 抽象是该原则在传输层的具体化。
- USB 帧格式冻结不删：它是无 CAN 收发器时的救急链路和协议考古样本，维护成本 ≈ 0。

## 后果

- **正面**：主链路回到 1 Mbps 经典 CAN（100 Hz 控制周期，与 controller_manager 对齐）；
  遥测全量上行；传输可冷切换；固件与上位机有 `can_check.py` 测试向量对账。
- **负面**：
  - 双后端长期并行存在（USB 冻结可接受，CAN v4 仍在演进）；
  - SocketCAN 把内核链路配置变成部署前置（`ip link` 配置错误表现为 bus-off，与代码 bug 难区分）；
  - `SessionSnapshot` 成为新的兼容性承诺：改字段 = 破坏两后端对齐，须同步下位仓文档。

## 被否决的方案

- **以 `rak_can_driver`（python-can，CAN v3 host 栈）作为 CAN 主链路**：v3 协议已退役
  （`XFER_*`/帧布局全部作废），且独立节点路径绕开 ros2_control 控制器契约，与 ADR-0007
  冲突。包归档进 `attic/`（ADR-0013），仅留协议考古价值。
- **主备双链路同时活跃（热切换）**：F407 单会话语义 + 命令 lease 机制不支持双写方；
  故障切换的复杂度远超单机调试收益 → 冷备，切换靠重启 controller_manager。
- **USB 退役删除**：无收发器/总线故障时的唯一救急路径，且 SerialSession 是 CanSession 的
  行为参照实现（测试对拍用），删除得不偿失。

## 待闭合项

1. `rak_chassis` 仍只导出 4 轮接口；CAN v4 的 7 轴 `ARM_COMMAND` 位置接口、arm controller、
   夹爪轴未实现（ADR-0007/0011 的臂前置继续有效）。
2. 固件 R55+ 与 Jetson 的 CAN 全链路验收（`docs/hardware_acceptance.md` 底盘门 + 臂门）未跑。
3. bringup 的 `transport` 默认值仍是 `usb`；CAN 验收通过后翻转为 `can` 并更新 launch。

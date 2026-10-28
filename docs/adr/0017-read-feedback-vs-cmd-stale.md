# ADR-0017: read() 不把 cmd_stale（STATE.bit3）当反馈失效 —— 反馈存活性与命令路径状态解耦

- **状态**: 已接受（2026-10-14）
- **日期**: 2026-10-14
- **相关**: `src/rak_chassis/src/rak_system.cpp`（`read()`）· `src/rak_chassis/src/can_session.cpp`
  （`kStateFaultMask` / `kIdDiag` STATE 解析）· `~/vehical_rak_chassis/docs/PROTOCOL.md`
  （v4，协议事实源，§5 STATE 位图 + §6.2 断流看门狗）· ADR-0012（Session 抽象）、ADR-0016（CAN 唯一链路）
- **背景来源**: `AGENTS.md` Gotchas（2026-10-14 实测记录）· `docs/arm_moveit_contract.md` §1

---

## 背景

- F407 的 `STATE` 位图（PROTOCOL.md §5）: `bit0 estop_latched` / `bit2 fault_active` /
  **`bit3 cmd_stale`** / `bit4 bat_uv` / `bit5 can_recovering`（其余位为放行链自诊断，1=正常）。
  §6.2 看门狗语义：**`DRIVE_CMD` 流断 100 ms → 轮速目标清零、受控减速停 → IDLE → `bit3=1`**。
- 主机侧有两套故障判定，职责本不同：
  - `CanSession` 的 `faulted` = `(STATE & 0x0005) != 0 || MODE∈{2,3}` —— 只含 estop/fault_active，
    **刻意排除 bit3/4/5**（`can_session.cpp` `kIdDiag` 分支）；
  - `CanSession` 的 `fault_bitmap = STATE & kStateFaultMask(0x003D)` —— 纯**诊断**字段，含 bit3。
- 但 `rak_system.cpp` 的 `read()` 在 `state.faulted` 之外**又加了一条** `state.fault_bitmap != 0U`
  —— 把 bit3（连同 bit4/5）重新拉回了硬否决，绕过了上面那次刻意的排除。
- 2026-10-14 实测（AGENTS.md Gotchas）：出现过 `read() lost feedback` 日志，逐字段打印为
  `connected=1 handshaken=1 feedback_valid=1 faulted=0 feedback_age=2ms fault_bitmap=0x0008`
  —— **链路完全健康，read() 仅因 bit3 返回 ERROR**。且该链是自锁的：

  ```
  控制器未加载/间隙期(无人动 DRIVE_CMD 节拍) → 固件看门狗置 bit3(0x0008)
      → read() 返回 ERROR → controller_manager 停用硬件
      → on_deactivate → set_motion_enabled(false)（运动使能关闭）
      → bit3 的成因被本层动作保持 → read() 永远过不去 → 只有整体 relaunch 才能逃出
  ```

  即：**反馈读取路径被一个"命令没在发"的状态位反向锁死**，而"命令没在发"恰恰是本层停用
  之后的必然状态。

## 决策

1. **`read()` 只用反馈存活性 + 固件硬故障做否决，不看 cmd_stale。** 判据收敛为：
   - 链路/反馈：`connected`、`handshaken`、`feedback_valid`、`feedback_age ≤ 120 ms`；
   - 固件硬故障：`state.faulted`（estop_latched / fault_active / MODE=ESTOP|FAULT）；
   - 固件非自锁坏消息位：`fault_bitmap & ~(bit3 cmd_stale)` —— **保留** `bat_uv`、`can_recovering`
     （两者不会被本层的停用动作引发或保持，不构成自锁）；
   - **排除** `bit3 cmd_stale`：它描述的是命令路径/固件门控状态，由协议 §6.2 看门狗解释，
     不是反馈是否可信的证据——反馈断了 `feedback_age` 必然先超限。
2. **`kStateFaultMask`(0x003D) 与 `fault_bitmap` 内容不变。** bit3 仍是诊断信息，照常发布
   供日志/上层观测；被否决的从来只是"把诊断位直接当失效判据"这一用法。
3. **`write()` 维持现状**（只看 `state.faulted`），本就与新判据一致。
4. **停用后不自动重新使能的行为保持不变**（`read()` 失败 → `set_motion_enabled(false)` 后需
   重新激活才恢复运动）——那是有意的安全设计，本 ADR 只修"恢复不了"的自锁部分。

## 理由

- **单一职责**：`read()` 回答"我现在读到的轮/臂反馈可信吗"，不回答"固件此刻允许动吗"。
  后者的权威在固件（本地否决先于任何主机指令生效），上位机至多观察与上报。
- **自锁有代码级证明**：否决条件之一（bit3）会被否决动作本身（停用→关使能）维持，
  形成 `ERROR → 停用 → 置位 → ERROR` 的环；协议 §6.2 使"断流→IDLE→置位"成为固件的正确行为，
  环的起点在主机侧解读而非固件错误。
- **排除 bit3 不损失任何保护**：真正的链路死亡由 `feedback_age/feedback_valid` 捕获（≤120 ms），
  真正的固件急停由 `faulted` 捕获，`bat_uv/can_recovering` 继续保留。

## 后果

- **正面**：
  - 消除"控制器未就绪期 → bit3 → read() ERROR → 硬件被停用 → 永远起不来，只能 relaunch"的
    自锁死（AGENTS.md 实测链的根因）；
  - 启动宽限期不再被固件看门狗的历史状态污染；控制器加载失败后可在不重启进程的前提下重试。
- **负面（代价）**：
  - `bit3` 期间如果**同时**主机确实停发 `DRIVE_CMD`（异常场景），read() 不再替固件报出来——
    该场景由固件本地清零/停机兜底，主机侧从"反馈错误"降级为"诊断位可见"；依赖 read() ERROR
    来察觉"命令流断了"的用法（若有）从此失效，须改为观测 `fault_bitmap`。
  - `fault_bitmap != 0` 这个粗判据没有了，未来新增的坏消息位必须显式决定进/不进 read() 否决集，
    不能再靠"非零即坏"顺带生效——判据维护成本变高（换来的是每一位都有明确理由）。

## 被否决的方案

- **改 `kStateFaultMask` 把 bit3 从掩码里去掉**：掩码的职责是"哪些位属于坏消息"（诊断完整性），
  bit3 客观存在且有用；把实现 bug 藏进掩码会让诊断丢信息，且固件位义一变又要再改一次。
  （AGENTS.md 明令"not by editing the mask ad hoc"。）
- **read() 彻底不看 `fault_bitmap`（只留 faulted）**：会连 `bat_uv`、`can_recovering` 一起去掉。
  二者目前没有别的主机侧消费点，去掉等于静默丢告警；等它们有独立上报通道再议（见待闭合项 2）。
- **read() ERROR 后允许自动重新使能**：把"安全恢复"做成自动行为，违背"故障后需显式重新激活"
  的既有安全设计；本 ADR 只拆自锁，不改恢复语义。
- **在固件侧清 bit3 再起控制器**（主机要求固件强行清位）：固件位是看门狗的真实结论，
  主机覆写属于越权（System 2 不得放宽 guard 的同构问题），否决。

## 待闭合项

1. **固件位义实机确认**：在"主机持续以 100 Hz 发 `DRIVE_CMD`、远程门关闭（DISARMED）"的稳态下，
   `bit3` 是否仍置位？（协议 §6.2 只定义"断流→置位"，AGENTS.md 的实测链提示门控态也可能置位。）
   需一次 `can_link_smoke.py` + 本仓 chassis bringup 的定点观测，结果回填本文背景段。
   若确认"门关也算 stale"，固件域（`~/vehical_rak_chassis`，只读）需另行提出语义修订，
   本 ADR 的主机侧解耦依然成立。
2. **`bat_uv` / `can_recovering` 的主机侧上报通道**：目前只存在于 `fault_bitmap` 与日志；
   应有 diagnostics/事件出口，落点待与诊断体系（`/diagnostics` 现由谁拥有）一起定。
3. **停用后无 relaunch 的恢复路径验证**：修复后应实测
   `ros2 control set_hardware_component_state`（或 spawner 重试）能否在同进程内重新激活硬件，
   验证结果写入 `docs/hardware_acceptance.md`；在验证前，"整体 relaunch"仍是最短恢复路径。

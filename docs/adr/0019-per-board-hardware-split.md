# ADR-0019: 双下位机按板拆成两个 ros2_control hardware 组件，共享一个 CanSession

- **状态**: 已接受（2026-10-29）
- **日期**: 2026-10-29
- **相关**: `src/rak_chassis/src/rak_system.cpp` · `src/rak_chassis/src/can_session_pool.cpp` ·
  `src/rak_description/urdf/rak_chassis.urdf.xacro` · `docs/explanation/architecture.md` §3 ·
  ADR-0012（Session 抽象）、ADR-0016（CAN 唯一链路）、ADR-0017（反馈存活性判据）
- **背景来源**: 2026-10-28 恢复双板架构后的现场核查；`docs/CAN-BRINGUP` 系列记录

---

## 背景

2026-10-28 起舵机从底盘板迁到机械臂板，三节点（Jetson + 两块 C30D）同挂一条 CAN 总线。
**协议层已经按节点分开了**：

| 帧 | 归属 |
| --- | --- |
| `0x030 NODE_INFO` / `0x200-202` / `0x220 DIAG` / `0x032 CHASSIS_IMU` | 底盘板 |
| `0x031 ARM_NODE_INFO` / `0x210-211 SERVO_STATE` / `0x221 ARM_DIAG` / `0x222 ARM_IMU` | 机械臂板 |

但 **Jetson 的硬件接口层没跟着分**：一个 `<ros2_control name="rak_chassis_system">`、
一个 `RakSystemHardware`、一个 `can_channel` 参数，同时管理 4 轮（底盘板）和 7 轴（机械臂板）。

后果是可复现的（`rak_system.cpp` 的 `read()`）：

```text
底盘板掉线（feedback_age > 120 ms）→ read() 返回 ERROR
  → controller_manager 停用整个 hardware
  → arm_controller 一起被 deactivate（哪怕机械臂板还在 1 Hz / 10 Hz 正常上行）
```

反方向是好的（臂掉线只让 `arm_ready_` 保持 false，四轮不受影响），也就是**隔离是单向的**。
此外：`can_channel` 之类的总线参数只能配一份；包描述写的是 "Wheel-only"；
`docs/explanation/architecture.md` 还写着"臂接口待加"——三者都与实现脱节。

## 决策

1. **拆成两个 `<ros2_control>` 段 = 两个 plugin = 两个故障域：**
   - `rak_chassis/RakChassisHardware` → 只声明 4 轮（`0x030/0x200-202/0x220/0x032`）
   - `rak_chassis/RakArmHardware` → 只声明 6 定位轴 + 夹爪（`0x031/0x210/0x211/0x221/0x222`）
   - 实现仍是一份 `RakSystemHardware` + `Role` 分派，两个薄子类只负责设置角色。
2. **两个组件共享同一个 `CanSession`**（`can_session_pool`，进程级注册表 + 引用计数）：
   同进程、同 socket、引用归零才 `stop()`。**不允许各开一个 socket** ——
   100 Hz × 13 帧会被重复解码，而 Jetson 的 `mttcan` `qlen` 只有 10。
3. **共享参数必须逐字段一致**（`can_channel` / `host_build_id` / `allow_motion`）：
   不一致时 `on_configure` 直接失败并打印差异（宁可起不来，也不要两个组件对同一条总线
   持有不同假设）。URDF 契约测试同样钉住这一条。
4. **失败判据按板隔离**：底盘 `read()` 只看底盘自己的帧；臂组件只看 `arm_online`。
   底盘掉线只让轮子控制器下电，臂保持原位（固件 deadman 只清轮速，臂保持当前目标）。
5. **停用语义不对称**：底盘 deactivate 撤整机运动使能（`set_motion_enabled(false)`）；
   臂 deactivate 只置 `arm_ready_=false`，不动底盘导航。
6. **所有"在线/握手"断言都带时效窗**（本次一并收紧）：
   `handshaken` 1.5 s（NODE_INFO @1 Hz）、`arm_online` 1.5 s、`imu_valid` 100 ms、
   `arm_imu_valid` 200 ms。以前 `handshaken` 与 `imu_valid` 只置 true 从不复位。

## 后果

**正面**

- 一块板掉线不再拖垮另一块板的控制器（拆分的主要目的）。
- 可以单独 `deactivate`/重新激活某一块板；参数可按板配置。
- 命名与文档回到与实现一致：`rak_chassis` 管"底盘 + 臂两个硬件组件"，不再是 "Wheel-only"。
- 契约测试从 6 个（11 关节混合）变成 12 个（两块板各自的接受/拒绝/导出），
  "一段里混进另一块板的关节"成为**显式失败用例**。

**负面 / 代价**

- 共享会话需要引用计数与参数一致性校验；`<ros2_control>` 两段的参数必须手写两遍。
- `ros2_control` 的 hardware 生命周期仍由 CM 统一管理，两段通常一起 activate；
  真正的"单独重启一块板"要走 `controller_manager` 的组件级操作，而不是重连总线。
- 若将来两块板分挂两条 CAN（例如臂板改用 `can1`），pool 会自然变成两个会话，
  但 URDF 参数需要各写自己的 `can_channel`。

## 被否决的替代方案

| 方案 | 否决理由 |
| --- | --- |
| 保持单接口，`read()` 按板返回不同严重度 | `ros2_control` 的 `return_type::ERROR` 是**组件级**语义，没有"部分错误"，做不到隔离 |
| 拆成两个进程，各开一个 SocketCAN fd | CAN 是广播，双份解码白烧 CPU；`qlen=10` 下更容易丢帧；写侧还要额外保证单写者 |
| 只改名/改文档，不动结构 | 故障域耦合是行为问题，不是命名问题 |
| 让臂组件也管轮子接口的"只读副本" | 会造出两个状态源，违反 ADR-0015 的单一发布者原则 |

## 验收

- `colcon build --packages-select rak_chassis` 通过；
- `system_interface_test` 12/12 通过（两块板各自的接受/拒绝/导出）；
- `validate_chassis_urdf.py` 全过（两段、两个 plugin、共享参数一致、合并关节顺序不变）；
- 真实 `controller_manager` 载入日志同时出现 `hardware 'rak_chassis'` 与 `hardware 'rak_arm'`；
- **未上真机**：故障域隔离的实际行为（拔掉底盘板、臂仍可动）待上板验收。

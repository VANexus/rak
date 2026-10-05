# Jetson ↔ Base F407 UART Protocol v1.1 — 原子动作 + 底盘里程计扩展

**状态**:v0.2 提案（2026-10-04）—— 待 F407 固件负责人 + Jetson host 联合签字

**适用范围**: `docs/protocol/jetson_f407_uart_v1.md` 的修订补丁。v1 行为向后兼容；MCU 报告
`FEATURE_V1_1_ACTIONS` 后才发送 v1.1 字节。

**关键变化**: `SET_BASE_VELOCITY` 加 2 个字节——`action_type`（0..8）和 `max_exec_ms`。
Nav2 路径发 raw twist (`action_type=0`, `max_exec_ms=0`)；task_executor / 直控路径发
6 个原子动作 + 限时，MCU 按动作类型加不同安全约束。

## v1.1 在 v1 上的差异

| 字段 | 变化 |
|---|---|
| Feature bit 3 | 新增 `FEATURE_V1_1_ACTIONS = 1 << 3` |
| `SET_BASE_VELOCITY` payload | 15 → 19 字节 |
| `HELLO.requested_features` / `HELLO_ACK.accepted_features` | bit 0/1 仍必需；bit 3 协商 |
| Message type `0x13 BASE_ODOMETRY` | 新增，56 字节 payload（保留在 `nav2_base_odometry.md`，本文件只引用）|

## 修订后的 Message types 表

| ID | Name | Payload | 版本 |
|---:|---|---|---|
| `0x01` | `HELLO` | `host_build_id:u32`, `requested_features:u32` | v1/v1.1 |
| `0x02` | `HELLO_ACK` | `board_id:u16`, `firmware_version:u32`, `boot_id:u32`, `accepted_features:u32` | v1/v1.1 |
| `0x10` | `SET_BASE_VELOCITY` | **v1.1 扩展**：见下表 | v1.1 |
| `0x11` | `BASE_STATE` | 23 字节（同 v1）| v1/v1.1 |
| `0x12` | `FAULT_EVENT` | 6 字节（同 v1）| v1/v1.1 |
| `0x13` | `BASE_ODOMETRY` | 56 字节（详见 `nav2_base_odometry.md` v0.1 § UART 里程计扩展）| v1.1 |

## SET_BASE_VELOCITY v1.1 payload（19 字节）

| Offset | 字段 | 类型 | 含义 |
|---:|---|---|---|
| 0 | `mode` | `u8` | `0=DISARMED, 1=ARMED` |
| 1 | `lease_ms` | `u16 LE` | `[10, 50]`；超时未收到新帧 → DISARM + fault bit 1 |
| 3 | `action_type` | `u8` | 见下表 |
| 4 | `reserved` | `u8` | `0x00`，alignment |
| 5 | `vx_mm_s` | `i32 LE` | 底盘前进速度 |
| 9 | `vy_mm_s` | `i32 LE` | 底盘横移速度 |
| 13 | `wz_mrad_s` | `i32 LE` | 底盘偏航角速度 |
| 17 | `max_exec_ms` | `u16 LE` | `0`=无上限（Nav2 路径）；`>0`=MCU 限时到达后自动刹车 |

## action_type 编码

| 取值 | 名字 | twist 字段约束 | max_exec_ms 推荐 | MCU 安全策略 |
|---:|---|---|---:|---|
| 0 | CONTINUOUS | `vx, vy, wz` 由 Nav2 自由给 | `0`（不超时）| 等同 v1 行为；不应用任何动作约束 |
| 1 | FORWARD | `vx>0, vy=0, wz=0` | `1500` | 端点对齐检查；超时硬停 |
| 2 | BACKWARD | `vx<0, vy=0, wz=0` | `1500` | 同上 |
| 4 | STRAFE_LEFT | `vx=0, vy>0, wz=0` | `1500` | 同上 |
| 5 | STRAFE_RIGHT | `vx=0, vy<0, wz=0` | `1500` | 同上 |
| 6 | ROTATE_CW | `vx=0, vy=0, wz>0` | `1000` | yaw 漂移限幅；超时硬停 |
| 7 | ROTATE_CCW | `vx=0, vy=0, wz<0` | `1000` | 同上 |
| 8 | STOP | `vx=0, vy=0, wz=0` | `100` | 立即 brake，不用等 lease 过期 |

> 取值非 power-of-2（用 1, 2, 4, 5, 6, 7, 8）避免与 feature bit 混淆；`action_type=3` 保留为非法。

## MCU 必须行为（v1.1 新增部分）

1. **action_type ↔ twist 一致性校验**：违反上表 twist 字段约束时
   - **建议**: 写 `fault bit 7 (invalid feedback)`，进入 FAULT 状态，**不立即 DISARM**（让 host 看到错误后调整）
   - **替代**: 直接丢弃该帧并 DISARM（host 必须靠日志发现）
   - **请固件负责人确认**: 用哪个？我倾向第一种（带 fault bit 不锁机），但 MCU 可改

2. **`max_exec_ms` 计时**：
   - `max_exec_ms=0`：不计时，行为等同 v1
   - `max_exec_ms>0`：从收到该帧起计时，到达 `max_exec_ms` 后 MCU **必须**进入 brake（无需依赖 lease）
   - 注意：v1 lease 是 50ms（防掉线），`max_exec_ms` 是动作限时（防原子动作无限运行）。两个独立计时器

3. **STOP 动作语义**：`action_type=8, max_exec_ms=100`：
   - 收到时立即输出 brake 电流（不等 lease）
   - 100ms 后回到 DISARMED
   - 期间忽略同帧的 vx/vy/wz（即使非零）

4. **action_type=0 CONTINUOUS 的边界**：
   - MCU **不应**应用原子动作的安全约束（端点对齐、限时）
   - 这是 Nav2 的高频跟踪路径；任何额外开销都会引起速度抖动

## v1 → v1.1 feature 协商流程

```
Jetson -> F407  HELLO  requested_features = 0x00000003 (bit 0|1)
Jetson -> F407  HELLO  requested_features = 0x0000000B (bit 0|1|3) -- v1.1
```

| MCU 在 HELLO_ACK.accepted_features 中报 bit 3 ? | Jetson 行为 |
|---|---|
| 否 (v1-only MCU) | 继续走 v1 路径（payload 15 字节），不报错——纯 backward compat |
| 是 (v1.1 MCU) | 发送 v1.1 SET_BASE_VELOCITY（payload 19 字节）|

**MCU 端注意**: 如果 MCU 报 bit 3 但 Jetson host 端用的还是 v1 编码（15 字节），MCU 应**按 v1 解析**——前 15 字节字段布局相同，新增字节当作 0/保留。

## GTA 编码示例（Nav2 路径）

```text
Nav2 MPPI 算出 (vx=0.20 m/s, vy=0, wz=0)
Jetson 编码:
  mode=1, lease_ms=50,
  action_type=0, reserved=0,
  vx_mm_s=200, vy_mm_s=0, wz_mrad_s=0,
  max_exec_ms=0

Frame:
  01 01 34 12 12 00
    01 [mode] [lease_ms LE] [action_type] [reserved]
    [vx_mm_s LE] [vy_mm_s LE] [wz_mrad_s LE]
    [max_exec_ms LE]
    [CRC 2 字节 LE]
  00 (COBS delimiter)
```

## GTA 编码示例（FORWARD 原子动作）

```text
task_executor 发 "前进 1 米 @ 0.5 m/s"
Jetson 编码:
  mode=1, lease_ms=50,
  action_type=1, reserved=0,
  vx_mm_s=500, vy_mm_s=0, wz_mrad_s=0,
  max_exec_ms=1500  (1.5 m / 1 m/s ≈ 1.5s, 含安全余量)

Frame: 同上布局，但 action_type=1, max_exec_ms=1500
```

MCU 应在 1500ms 后硬停，即使 Jetson 后续还在发同一条 FRAME（不依赖 lease）。

## Golden vectors (PRIMARY — 双方 PTY 测试共享)

```text
Test 1: Nav2 路径 (action_type=0)
  payload: 01 32 00 00 00 c8 00 00 00 00 00 00 00 00 00 00 00 00
  decoded: mode=1, lease_ms=50, action_type=0, reserved=0,
           vx=200 mm/s, vy=0, wz=0, max_exec_ms=0
  CRC: 待补

Test 2: FORWARD 1.5s
  payload: 01 32 00 01 00 00 01 00 00 00 00 00 00 dc 05
  decoded: mode=1, lease_ms=50, action_type=1, reserved=0,
           vx=256 mm/s (=0x100), vy=0, wz=0, max_exec_ms=1500 (=0x05dc)
  CRC: 待补

Test 3: STOP
  payload: 01 32 00 08 00 00 00 00 00 00 00 00 64 00
  decoded: mode=1, lease_ms=50, action_type=8, reserved=0,
           vx=0, vy=0, wz=0, max_exec_ms=100
```

## Jetson host 端要做的事（实现清单）

1. `protocol.py`:
   - 加 `FEATURE_V1_1_ACTIONS = 1 << 3`
   - 加 `SET_BASE_VELOCITY_V1_1 = struct.Struct("BBBi i i H")` (19 字节)
   - 加 `encode_set_base_velocity_v1_1(...)` helper
   - `node.py`: 根据 `HELLO_ACK.accepted_features & 0x8` 决定用 v1 还是 v1.1 编码

2. `node.py`:
   - `max_exec_ms` 计时器（独立于 v1 旋转 lease）：到点自动 DISARM + fault bit 1
   - action_type ↔ twist 一致性校验：违则**写 fault bit 7（当前实现里已定义）** + 撤销 ARM
   - 新增 service `/base_uart_driver/send_action` (action_type, magnitude)，task_executor 调用
   - **默认不启用 v1.1**（HELLO 不请求 bit 3，除非明确置 `requested_features:=v1_1`）

3. `rak_task_executor/`:
   - `ExecuteTask.action` 解析后分解成原子动作序列
   - 通过 `/base_uart_driver/send_action` service 逐个发出
   - 每个原子动作有 max_exec_ms 默认值

4. `base_uart.launch.py`:
   - 新增 `protocol_version` arg (`v1` or `v1_1`)，默认 `v1`
   - 新增 `action_max_distance_m` 等原子动作默认限时参数

5. PTY 测试 (`scripts/pty_f407_sim.py`):
   - 用 pty 模拟 F407
   - 接受 HELLO，回 HELLO_ACK（v1 或 v1.1，按 host 请求）
   - 回 `BASE_STATE` (echoed seq, measured twist, fault_bitmap)
   - 模拟 max_exec_ms 超时（到点写 FAULT_EVENT）
   - 模拟 action_type 校验失败（写 fault bit 7）
   - 模拟 lease 超时

## 验收（继承 v1 + v1.1 新增）

### Stage 1: PTY 联调
- 双方跑同一份 golden vector，比对 CRC + payload 字节
- Host 发 v1.1 FORWARD，simulator 在 max_exec_ms 后 DISARM
- Host 触发违反一致性，simulator 写 fault bit 7

### Stage 2: 符号台架（手推车）
- 4 轮离地，手推车，验证 vx>0 时 RF2o odom 的 x 单调增
- 验证 strafe_right (vy<0) 时 y 单调增

### Stage 3: 物理 bench
- 架起车轮，F407 报告 ARMED
- 单轴 FORWARD 0.5m/s × 1.5s → 期望约 0.7m 距离后停止
- ROTATE_CW 0.5 rad/s × 1s → 期望约 30° 偏航
- STOP → 立即停

### Stage 4: Nav2 闭环
- chain: nav2.launch.py motion_backend:=none + task_navigation.launch.py motion_backend:=hardware
- 闭环跑到 rf2o 验证 path 跟踪

---

## 给固件负责人的**对接议题包**

```
1. feature bit 3 = FEATURE_V1_1_ACTIONS 是否同意?
2. SET_BASE_VELOCITY payload 加 2 字节（action_type + max_exec_ms + reserved 共 3 字节，
   total 19）布局是否同意？ 还是想用其他 layout（比如把 action_type 放最后）?
3. action_type 编码 0=CONTINUOUS, 1=FORWARD, 2=BACKWARD, 4=STRAFE_LEFT, 5=STRAFE_RIGHT,
   6=ROTATE_CW, 7=ROTATE_CCW, 8=STOP（避开 power-of-2）
   是否同意? 如果你更习惯 power-of-2, 我能调，但 feature bit 就要重新分配
4. max_exec_ms 默认推荐: FORWARD/BACKWARD/STRAFE=1500, ROTATE=1000, STOP=100
   是否同意? 还是由固件自定?
5. action_type ↔ twist 一致性校验: 违反时写 fault bit 7 (invalid feedback) + FAULT 状态（不锁机）
   还是直接丢弃 DISARM?
6. STOP 语义: 立即 brake (不等 lease), 100ms 后 DISARM 是否同意?
7. max_exec_ms 计时是独立于 v1 lease 的独立计时器?
8. 同时 HOST 端需要你方确认:
   - REP-103 轴向 +X/+Y/+Z yaw 与你 F407 内部坐标一致?
   - 故障 bit 0..7 物理实现: E-stop / lease timeout / CAN / motor / 欠压 / 过温 / 反馈
   - 底盘 IMU 是否进 F407 内部里程计, 而不仅仅是电机稳定
```

---

**文档版本**: v0.2 提案 — 2026-10-04
**作者**: Claude (host 端起草, 等固件负责人签字)
**配套文档**:
- `jetson_f407_uart_v1.md` (v1 baseline)
- `nav2_base_odometry.md` v0.1 (`BASE_ODOMETRY` 详细字段)

**变更点**: 把 v0.1 中散落的 BASE_ODOMETRY 提案从本文件**剥离**到 nav2_base_odometry.md，本文件只引用——避免 0x13 在两个文件里被独立描述。
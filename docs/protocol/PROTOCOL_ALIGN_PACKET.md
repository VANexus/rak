# Jetson ↔ Base F407 UART 协议对齐包

**状态**：2026-10-04 待签字（host + 固件双方）
**对接方式**：固件负责人读完本文档，确认/修改下面 §3 的 7 项，然后我们把固件 + host 一起 update
**当前基线**：`docs/protocol/jetson_f407_uart_v1.md`（v1）已经在仓库里

---

## 1. 物理链路（已确认 2026-10-04）

```
Jetson  ──USB──  F407 base  ──4× 麦轮 + IMU + encoders──  底盘
921600 baud 8N1  100 Hz 双向
port: /dev/wheel_motor (udev alias → /dev/ttyACM0，WCH CDC-ACM, hub 1-2.3.3)
```

F407 在 bus 启动后周期性广播 `ID:A\n`（ASCII @ 921600 8N1）用于设备识别——**这是 Jetson 端的设备身份确认信号，不参与协议握手**。

## 2. Frame 公共格式（v1 + v1.1 通用）

```
wire = COBS(decoded_frame) || 0x00

decoded_frame = {
    protocol_version : u8     = 1
    message_type     : u8     (下表)
    sequence         : u16 LE  (上 1..32767 单调递增)
    payload_length  : u16 LE  (≤ 128)
    payload         : bytes   = payload_length
    CRC             : u16 LE  (CRC16/CCITT-FALSE over 前 N-2 bytes)
}
```

CRC check value: `b1 29` for ASCII `123456789`

## 3. v1 baseline（必须实现，已在 Jetson 仓库）

| Type | Name | Size | 用途 |
|---:|---|---:|---|
| 0x01 | HELLO | 8 | host → f407：能力协商 |
| 0x02 | HELLO_ACK | 14 | f407 → host：回 capability |
| 0x10 | SET_BASE_VELOCITY | 15 | host → f407：底盘中心 body twist setpoint |
| 0x11 | BASE_STATE | 23 | f407 → host：状态回显、实测 twist、故障位 |
| 0x12 | FAULT_EVENT | 6 | f407 → host：事件故障位 + source seq |

### 3.1 HELLO (8 bytes)
```
host_build_id      : u32   (host 唯一标识)
requested_features : u32   bit 0 = BASE_VELOCITY，bit 1 = BASE_TELEMETRY 都必须
```

### 3.2 HELLO_ACK (14 bytes)
```
board_id           : u16   (建议 0x0407 = STM32F407)
firmware_version   : u32   (major<<16 | minor<<8 | patch)
boot_id            : u32   (每次 MCU 重启必须变化)
accepted_features  : u32   bit 0/1 都必须 set
```

### 3.3 SET_BASE_VELOCITY v1 (15 bytes) — 必须能跑
```
mode        : u8     0=DISARMED, 1=ARMED
lease_ms    : u16 LE [10, 50]
vx_mm_s     : i32 LE  底盘前进速度
vy_mm_s     : i32 LE  底盘横移速度
wz_mrad_s   : i32 LE  底盘偏航角速度（REP-103 +X 前/+Y 左/+Z yaw）
```

### 3.4 BASE_STATE (23 bytes) — 必须能发
```
echoed_command_sequence : u16 LE  回显的 SET_BASE_VELOCITY seq
state                   : u8      0=BOOTING, 1=DISARMED, 2=ARMED, 3=FAULT
fault_bitmap            : u32 LE  (bit 0:E-stop, 1:lease timeout, 2:CAN peer off,
                                     3:peer fault, 4:motor driver, 5:undervoltage,
                                     6:overtemp, 7:invalid feedback)
vx_mm_s                 : i32 LE  实测底盘前进速度
vy_mm_s                 : i32 LE
wz_mrad_s               : i32 LE
device_uptime_ms        : u32 LE
```

### 3.5 FAULT_EVENT (6 bytes)
```
fault_bitmap     : u32 LE
source_sequence  : u16 LE  触发该故障的 SET_BASE_VELOCITY seq
```

## 4. v1.1 提案（待你确认是否启用）

**目的**：host 端多一个 `action_type` + `max_exec_ms` 字段，让 Jetson task_executor 能发"原子动作"（"前进 1 米"、"左转 30°"），MCU 按动作类型加不同安全约束。Nav2 路径继续走 raw twist，不受影响。

如果你的固件不实现 v1.1 → 跳过本节，host 默认走 v1（自动 fallback）。

### 4.1 feature bit 3 = `FEATURE_V1_1_ACTIONS`

HELLO 时 host 在 `requested_features` 里 **追加** bit 3；F407_ACK 里 bit 3 set 表示支持。

### 4.2 SET_BASE_VELOCITY v1.1（19 bytes）— 在 v1 基础上加 3 字节

```
mode          : u8
lease_ms       : u16 LE   [10, 50]
action_type   : u8      0=CONTINUOUS（v1 兼容）, 1=FORWARD, 2=BACKWARD,
                       4=STRAFE_LEFT, 5=STRAFE_RIGHT, 6=ROTATE_CW,
                       7=ROTATE_CCW, 8=STOP
reserved      : u8      0x00
vx_mm_s       : i32 LE
vy_mm_s       : i32 LE
wz_mrad_s     : i32 LE
max_exec_ms   : u16 LE   0=不限时（CONTINUOUS），>0=MCU 限时到达后自动刹车
```

### 4.3 action_type ↔ twist 一致性校验

| action_type | 必满足 |
|---|---|
| 0 CONTINUOUS | 无约束（Nav2 raw twist） |
| 1 FORWARD | vx>0, vy=0, wz=0 |
| 2 BACKWARD | vx<0, vy=0, wz=0 |
| 4 STRAFE_LEFT | vx=0, vy>0, wz=0 |
| 5 STRAFE_RIGHT | vx=0, vy<0, wz=0 |
| 6 ROTATE_CW | vx=0, vy=0, wz>0 |
| 7 ROTATE_CCW | vx=0, vy=0, wz<0 |
| 8 STOP | vx=0, vy=0, wz=0 |

违则：**写 fault_bitmap bit 7 (invalid feedback) + state=FAULT**，不立即 DISARM（让 host 看到错误后调整）。

### 4.4 max_exec_ms 行为

```
action_type=0 (CONTINUOUS): max_exec_ms 必须 = 0（不限时）
action_type∈{1,2,4,5,6,7,8}: max_exec_ms 必须 > 0（设时限）
收到时：MCU 启动 max_exec_ms 计时器
到达后：MCU 立即 brake（不等 lease 过期）+ DISARMED
动作完成后：MCU 仍在该帧 lease_ms（50ms）窗内继续接收 host 新帧
```

## 5. 安全状态机（F407 必做）

```
启动 → DISARMED (vx/vy/wz 必为 0，不记得任何 ARM)
握手：host HELLO → F407 必回 HELLO_ACK（v1 features 都 + v1.1 可选）
CRC/长度错、序号倒退/重复 → 丢帧，不刷新 lease
100Hz SET_BASE_VELOCITY 中断 → lease 50ms 到期 → DISARMED + fault bit 1 锁存
fault_bitmap != 0 / state 非法 → DISARMED
MCU 重启 / boot_id 变化 → host 必清 ARM、重新握手
恢复通信不自动 ARM —— 必须 host 显式 service 调用 /base_uart_driver/arm
```

## 6. Jetson host 已实现的代码（今晚 ready 的部分）

| 文件 | 功能 |
|---|---|
| `src/rak_base_driver/protocol.py` | v1 baseline + v1.1 helpers（pack/unpack/validate）|
| `src/rak_base_driver/node.py` | HELLO 协商、SET_BASE_VELOCITY 自动 v1/v1.1 切换、fault 闭锁、`~/arm` + `~/send_atomic_action` service |
| `src/rak_interfaces/srv/SendAtomicAction.srv` | 类型化 service：action_type / magnitude / max_exec_ms → 返回 wire bytes |
| `src/rak_bringup/launch/task_navigation.launch.py` | hardware conditional bug 修复 |
| `docs/protocol/jetson_f407_uart_v1.1.md` | v1.1 完整版 spec（这次对接用本文档即可，文档内含更多细节）|

**Host 端默认 `request_v1_1_actions:=false`** ——固件只支持 v1 也直接跑通，不需要任何改动。

## 7. 验收路径（按 5 阶段文档）

1. **PTY bench**（host ↔ Python sim）：我已经写了 `scripts/pty_f407_sim.py` 跑通 HELLO/HELLO_ACK + v1.1 19字节 SET_BASE_VELOCITY 解码 + fault bit 7 一致性校验
2. **固件 ↔ PTY bench**（固件 ↔ host）：固件负责人烧 v1，host 默认请求 v1 features 即可跑通
3. **固件 ↔ Jetson PTY bench**：固件烧进真板子，host `motion_backend:=hardware port:=/dev/wheel_motor` 直跑
4. **物理 bench**（4 轮离地，单轴 FORWARD/ROTATE_CW，E-stop 在手）
5. **Nav2 闭环**：链入成本地图 + refcount/odom + collision_monitor

## 8. 你（固件负责人）需要确认 7 项

```
□ 1. 物理链路 921600 8N1 USB-UART 不变（已确认）
□ 2. v1 frame 公共格式 + HELLO/HELLO_ACK/SET_BASE_VELOCITY/BASE_STATE/FAULT_EVENT 实现
     （REQ_FEATURES = bit0|1 必须）
□ 3. feature bit 3 = FEATURE_V1_1_ACTIONS（v1.1 新增，可选）
□ 5. action_type 编码 0/1/2/4/5/6/7/8 + twist 一致性校验（可选）
□ 6. max_exec_ms 独立计时器 + 到达自动 brake（可选）
□ 7. 故障位 bit 0..7 物理实现：E-stop / lease timeout / CAN peer / 电机 / 欠压 / 过温 / 反馈无效
```

v1 必须实现（□ 1-3, 7）。v1.1 可选（□ 4-6）。

---

**最简复路径**：固件实现 §3 的 v1（必）+ §5 安全机 + §7 物理故障位 → Jetson host 默认配置可直接驱动麦轮。v1.1 是增量项，固件时间紧可以分两步。
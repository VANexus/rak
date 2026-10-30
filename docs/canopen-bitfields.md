# CANopen v5 位域与身份表（R73 / A14）

> 本文件回答一个此前没有答案的问题：`imu_evt`、`state`、`fault` 这些整数字段
> **每一位到底是什么意思**。没有它，上层只能把状态字当"非零即故障"看，
> 既无法定位，也无法体面降级。
>
> **事实来源**：逐条抄自固件头/实现，路径见每行"出处"列。
> **未在总线上逐位验证** —— 首帧核对只验证了帧长与量化范围
> （`scripts/verify_canopen_frames.py`），位语义的验证要靠下面"如何验证"一列。
> 引用时按 AGENTS §4 的规矩写"转述自固件源码 R73/A14"，不要写成"实测"。

## 1. 底盘 node 1

### 1.1 `0x2033 imu_evt`（uint8，TPDO4，100 Hz）

| 位 | 掩码 | 语义 | 触发条件 | 出处 | 如何验证 |
| --- | --- | --- | --- | --- | --- |
| 0 | `0x01` | `IMPACT` 冲击 | 加速度二阶差分超阈，持续 `IMU_IMPACT_HOLD_MS` | `Core/Inc/app_imu.h:35` | 轻敲车身，看该位短暂置 1 |
| 1 | `0x02` | `SLIP` 打滑 | 轮速与 IMU 推算不一致（`dbg_slip`） | `app_imu.h:36`、`app_canopen.c:174` | 架空单轮转，看是否置位 |
| 2 | `0x04` | `STILL` 静止 | 角速度/加速度都在静止带内 | `app_imu.h:37` | 静置 3 s 应稳定置 1 |
| 3 | `0x08` | `SATURATED` 饱和 | `d2 > IMU_SAT_MG²` | `app_imu.h:38`、`app_imu.c:286` | 快速晃动到量程边界 |
| 4 | `0x10` | `STALE` 陈旧 | IMU 采样超时 | `app_imu.h:39` | 拔 IMU（若可）或压测 I2C |

未定义位：5–7，**必须忽略**（不得作为故障判据）。

### 1.2 `0x2052 car_state`（uint16，SDO 读，10 Hz 语义）

位语义来自 `app_car.c:1301` 的 `car_flags()`：

| 位 | 掩码 | 语义 |
| --- | --- | --- |
| 0 | `0x0001` | 遥控中（`can_remote`） |
| 1 | `0x0002` | 急停锁存 |
| 2 | `0x0004` | 自检中 |

### 1.3 `0x2053 car_mode`（uint8，SDO 读）

`enum run_mode { IDLE, RUNNING, DONE, STOPPED, FAULT, CALIBRATING, CALIB_DONE }`
（`app_car.c:80`）→ 数值 0..6，**顺序即语义**。

### 1.4 `0x2050 cmd_en` / `0x2051 cmd_age_ms`

- `cmd_en`：命令使能（deadman 与 `cmd_flags.bit0` 同时成立才为 1）。
- `cmd_age_ms`：`now - 最近一帧 cmd_seq 变化`，单位 ms，饱和到 `uint16`。
  用于把"命令年纪"与 Jetson 时钟对齐（ADR-0023 §2）。

## 2. 机械臂 node 2

### 2.1 `0x2130 state`（uint16，TPDO4，10 Hz）

`arm_state_bitmap()`（`vehical_rak_edge_ai/Core/Src/app_arm.c:145`）：

| 位 | 掩码 | 语义 | 备注 |
| --- | --- | --- | --- |
| 0 | `0x0001` | `estop_latched` | 锁存，需显式清除 |
| 1 | `0x0002` | `remote_running` | 远程使能中 |
| 2 | `0x0004` | `fault` | **J1/J2 位置环故障**（跟随误差 >30° 或软限位越界） |
| 3 | `0x0008` | `cmd_stale` | 命令断流 |
| 4 | `0x0010` | `bat_uv` | 本板未接电池 ADC，**恒 0** |
| 5 | `0x0020` | `can_recovering` | bus-off 恢复中 |

> 注意：**"未归零"不算 fault**（源码注释明确），它是"未就绪"，OLED 单独显示 `NOTZRO`。
> 上层不要把它并进故障位图，否则每次上电都会误报。

### 2.2 `0x2131 fault` / `0x2154 joint_fault_bm`

- `0x2131 fault`：当前固件恒写 0（`app_canopen.c:159`），**不要依赖**。
- `0x2154 joint_fault_bm`：**每轴一位**，`bit a` = 第 a 轴（J1=bit0 … 第 7 轴=bit6），
  由 `joint_fault()` 逐轴或起来（`app_joint.c:365`）。清除走 SDO 写 `joint_fault_clear()` 的
  对应命令对象（见待闭合项）。

### 2.3 `0x2132 mode`（uint8）

`arm_mode()`（`app_arm.c:159`）：`0 = IDLE`、`1 = REMOTE`、`2 = ESTOP`。

### 2.4 `0x2122 imu_evt`（uint8）

与底盘同表（`vehical_rak_edge_ai/Core/Inc/app_imu.h:48-52`），其中
**`SLIP` 恒 0**（打滑判定在底盘板）。

### 2.5 `0x2155 joint_zeroed`（uint8）

1 = 六轴已归零；与 `state.bit2` 无关（见上面的注意）。

## 3. 身份段（build_tag）

| 节点 | 对象 | 语义 | 现状 |
| --- | --- | --- | --- |
| 底盘 | `0x2F00 tag_packed` | build_tag 前 4 字符小端打包（`52 37 33` = "R73"） | 有 |
| 底盘 | `0x2F01 reset_cause` | `'S'` 软复位 / `'I'` 上电 / `'P'` 看门狗 / `'B'` 掉电 | 有 |
| 机械臂 | `0x2F00 / 0x2F01` | 同上（A14） | **有对象，但 `arm.json` 缺 `identity` 段** |

**缺口说明（不在本仓修复）**：`vehical_rak_edge_ai/docs/canopen/arm.json` 没有
`identity` 段（底盘那份有 `device_name/hw_version/sw_version`）。补它要改 SSOT JSON
并重生成 EDS 与 `OD.c/h`，那条链在 MCU 仓的 `tools/canopen/`，本仓只能标出来。
**在此之前，Judge 运行版本一律读 SDO `0x2F00`，不读 `arm.json` 的元数据。**

## 4. 与 SIL2 措辞的关系

- SRDO（CiA 304）**未接**。
- 心跳消费者 500 ms 在 Jetson 侧**未实现**（契约里声明了，主站还没做）。
- 因此本仓所有文档统一表述为：**"具备走向 SIL2 的通信基础"**，
  禁止写"可认证"、"满足 SIL2"。这条由 `docs/` 术语检查 + code review 保证，
  本轮已把 `docs/CANOPEN-V5.md` 与 ADR-0021/0022 的措辞对齐。

## 5. 待闭合

1. `0x2154` 的清除路径（是否有对应的 SDO 写对象）未在 JSON 里找到，需固件确认。
2. `0x2131 fault` 恒 0 是否为临时占位，需确认后要么删掉对象、要么实现它。
3. 位语义尚未在总线上逐位验证（本表的"如何验证"列就是明天的动作清单）。

## 6. 2026-10-30 首帧实测回填（43395 帧，candump -L）

**字节序：`little` 已实测确认**（此前 TODO 项闭合）。证据三条：
1. `0x581` SDO 应答 `43 00 2F 00 52 37 35 00` = 索引 0x2F00 小端 + "R75" 字符串序；
2. `0x182 j1state` 小端解出 19.3°（合理的臂静止角），字节反序解出 0.0（无意义）；
3. 主站写 `cmd_seq=1,2,3…`（小端），总线上原样可见。

**同场核对发现三处契约↔固件出入**（rak 契约 vs `app_canopen.c` R75 实装）：

| 字段 | rak 契约写的 | 固件实装（源码 R75） | 首帧实测 | 待办 |
| --- | --- | --- | --- | --- |
| `imu_dyaw` | deg/s ×0.01，"[SPEC] 0x2004" | **`imu_take_yaw_delta_mrad()` 毫弧度增量**，对象 0x2030 | ≈0（静止，无法判别单位） | 契约单位改为 mrad 并写清"每 SYNC 的增量"；融合侧改用 |
| `imu_pitch`/`imu_roll` | deg ×0.01（对，源码 ×100.0f） | 源码 ×100.0f ✓ | **28701 / -25981（≈±163°）**——数值稳定但在漂，物理上不该这么大 | **需要现场回答：车在台架上怎么摆的？** 若车正放，则 pitch/roll 的轴向映射/约定有错 |
| `imu_gz`（臂 0x382） | °/s ×0.01 | （臂板待查 `app_canopen.c` A16） | **61.06 °/s**——静止不可能 | 同上，对照臂板源码 |

修正来源标注：对象是 **0x2030..0x2034**（不是契约注释里的 0x2004 / 0x2104）。
`verify_canopen_frames.py` 的 ±90°/±20° 合理范围是**启发式**，抓的正是这类出入。

## 7. pitch/roll ±163° 根因定位（2026-10-30 13:15，源码诊断）

`app_imu.c:277-281`：`pitch = -(acc_y/acc_z)·57.3`、`roll = (acc_x/acc_z)·57.3`——
**芯片系小角近似，未做竖装轴旋转**。模块竖装（+Y 朝上）时重力沿芯片 Y 轴，acc_z≈0，
比值爆炸 → 就是实测的 ±163°（且随轮振漂移）。与实测现象完全吻合。
**修复方案（R76 固件）**：先按已知安装（+Y 朝上）把加速度旋到水平系再算 tilt，
或直接按竖装解析式重推；影响面=遥测/OLED（`imu_tilt_deg` 消费者），不碰稳定器闭环。
修好前 Jetson **不消费** 0x2031/0x2032（EKF 航向权威=Jetson LSM6DSR，本就不依赖）。

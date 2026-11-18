# 底盘 CANopen v5 驱动层验收 runbook（G0–G3）

> 状态：现行 · 最后更新：2026-11-16

目标：在 Jetson 上按固定顺序验证底盘驱动层，直到"车轮真转 10 s"这一步。
每一步都有明确判定与失败含义；跳过前一步做后一步只会得到误判。

## 0. 前提

- 底盘板固件 = 仓库 `../vehical_rak_motion` 的 `build/Debug`（有符号表，取证用）。
  在板版本以 SDO `0x2F00` 实读为准，不以 git HEAD 为准。
- 同一总线**只能有一个 NMT 主站**（Jetson）。调试期间不要同时起第二个 `device_container`。
- `device_container_node` 需要 `CAP_NET_ADMIN`，用 root 起；**不要**给二进制加 file capability。

## 1. G0 —— 链路与板子在场

```bash
python3 scripts/can0_preflight.py
```

只读，不发帧。判据：

| 输出 | 含义 | 处理 |
| --- | --- | --- |
| `can0 不存在` | 接口没建 | 装并启用 `can0-up.service`（含 Orin AON pinmux） |
| `bitrate配置=缺` | 没设 1 Mbps | `ip link set can0 type can bitrate 1000000 sample-point 0.800` |
| `berr tx=N rx=0` | 我发过帧，**从未被 ACK** | 板不在总线上：供电 → CAN_H/CAN_L → 终端电阻 |
| `被动收帧 > 0` | 板在说话 | 进 G1 |

`berr tx>0 且 rx=0` 是"我发了没人应答"，不是"板子坏了"。Jetson 侧
`state ERROR-PASSIVE` 与这个计数同源，清计数器要 `ip link set can0 down/up`。

**loopback 自检**（区分 Jetson 侧故障与总线故障，不发到线上）：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 loopback on && sudo ip link set can0 up
(timeout 3 candump -n 2 can0 &) ; sleep 1; cansend can0 123#DEADBEEF
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 sample-point 0.800 loopback off restart-ms 100
sudo ip link set can0 up
```

抓到 `123 [4] DE AD BE EF` = mttcan 与 socketcan 正常，问题在物理层或对端。

## 2. G1 —— 主站与驱动起来

```bash
sudo -n bash -c 'source /opt/ros/jazzy/setup.bash \
  && source install/architecture/local_setup.bash \
  && export ROS_DOMAIN_ID=47 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  && ros2 launch rak_bringup chassis_bringup.launch.py slaves:=chassis allow_motion:=false'
```

判定（约 20 s 内）：

- `[chassis]: Driver booted and ready.` —— 从站完成 boot-up
- `canopen bridge ready: ... allow_motion=false` —— 薄桥拿到 proxy driver
- `ros2 topic hz /chassis/rpdo` 有稳定频率（实测 648 Hz）
- `ros2 topic echo --once /joint_states` 四轮 `velocity` 是**数字**，不是 `nan`

`slaves:=chassis` 只裁剪单板；**不要**只改 bus.yaml 而留着另一台的 URDF 段，
否则那台的 hardware 仍会被激活、永远收不到反馈 → `read()` ERROR → 连
`joint_state_broadcaster` 一起被停用。

## 3. G2 —— 在板版本与复位原因

```bash
ros2 service call /chassis/sdo_read canopen_interfaces/srv/CORead "{index: 0x2F00, subindex: 0}"
ros2 service call /chassis/sdo_read canopen_interfaces/srv/CORead "{index: 0x2F01, subindex: 0}"
```

`CORead.srv` 只有 `index`/`subindex` 两个请求字段（没有 `type`）。

- `0x2F00` 返回 4 字节小端打包的 build_tag。实测 `3553106 = 0x363352` → `R76`。
- `0x2F01` 返回复位原因字符：`S` 软复位 / `I` 看门狗 / `P` 上电 / `B` 掉电。

`0x1008/0x1009/0x100A` 在本移植会触发 HardFault，**不要**读它们来"确认版本"。

## 4. G3 —— 前移 10 s

```bash
# 车轮架空或场地已清空、有人守在总电源旁
ros2 launch rak_bringup chassis_bringup.launch.py slaves:=chassis allow_motion:=true
ros2 run rak_commissioning chassis_forward_test            # 预演，不运动
ros2 run rak_commissioning chassis_forward_test -- --yes   # 真实前移 10 s
```

脚本契约与 `chassis_tui` 一致：`/mecanum_drive_controller/reference`
(`TwistStamped`) @20 Hz；1 s 加速 → 保持 `duration` → 1 s 减速 → 2 s 零目标。
速度默认 0.10 m/s（10 s ≈ 1 m），上限 0.15 m/s 未标定前不放开。

三道门，任一不过即拒绝执行：

1. 无 `--yes` 只预演，不发运动指令；
2. `/joint_states` 没有四轮速度反馈 → 拒绝开步（驱动层未就绪）；
3. 保持段开跑 1.5 s 后轮反馈仍全零 → 立即归零退出。**最可能是
   `allow_motion:=false`**（薄桥只发零目标）或 MCU enable 位没开。

退出码 0 = 四轮反馈同向为正；1 = 卡住或方向不一致（查轮序/极性）。

## 5. 调试 SWD 时的两个陷阱（2026-11-16 实测）

排查物理层时很容易把板子"弄坏"成看起来像固件故障的样子：

- **`st-flash read` 会 halt 核且不释放**。读一次内存，核就停在原地，CAN 与
  OLED 全停、屏幕留半帧残影（"花屏"）。判断是不是这个原因：读 DHCSR
  (`0xE000EDF0`)，`S_HALT=1` 且 `C_DEBUGEN=1` 就是被调试器按住了。
- **`C_DEBUGEN` 会跨复位保持**。复位后核停在 `Reset_Handler` 而不是跑
  `main`，于是 `MCR` 保持复位默认 `SLEEP=1`、CAN 不发不 ACK —— 现象与
  "CAN 收发器坏"完全一致。
- 恢复：清 `DEMCR` 的 vector-catch、清 FPB、`SYSRESETREQ`，然后
  `set *(unsigned int*)0xE000EDF0 = 0xA05F0000` 释放，**之后不要再碰 SWD**。
  验证是否真在跑：隔 10 s 用 `st-flash read` 读 `uwTick`（`0x2000074C`），
  数值增长即已运行（实测 10 s 内 0 → 12435）。
- gdb 的 `detach` 在 st-util 1.8.0 上返回 `Remote doesn't know how to detach`；
  `continue` 期间也无法 `detach`。别指望它，用上面的寄存器写法。

## 6. 已确认的事实（2026-11-16）

| 项 | 值 | 依据 |
| --- | --- | --- |
| Jetson mttcan | 正常 | loopback 抓到自发帧 |
| can0 位时序 | 1 Mbps, sample-point 0.800 | `ip -details link show can0` |
| Orin pinmux | PAA0=0xC400, PAA1=0xC458 已写 | `devmem` 实读 |
| 在板固件 | R76（= repo HEAD） | SDO `0x2F00` = 3553106 |
| 固件 CAN 位时序 | 1 Mbps（BRP=2, TS1=16, TS2=4, 21 Tq @47.6 ns） | SWD 实读 `BTR=0x003F0001` |
| 固件主循环 | 在跑 | `uwTick` 10 s 内 0 → 12435 |
| 固件 CAN 外设 | 已初始化（`MCR=0x00010060`，SLEEP=0，CAN1EN=1） | SWD 实读 |
| **总线** | **双向零帧**：Jetson `berr tx=139 rx=0`，F407 `ESR LEC=3 TEC=128 REC=0` | 两端寄存器 |

结论：**驱动层软件链路与两端位时序都已排除**，剩余故障在物理层——CAN_H/CAN_L
接线、收发器供电或终端电阻。`LEC=3`（ACK error）而 `REC=0` 表示对端一帧都没收到。

## 7. G3 真机通过记录（2026-11-17）

物理层修复（重压 XH2.54 端子）后，按 G0→G3 全链实测：

```
[INFO] chassis_forward_test: 运动开始：前移 0.1 m/s，保持 10.0s。
[INFO] chassis_forward_test: 结束。停止瞬间反馈 FL=3.39 FR=3.43 RL=3.48 RR=3.41；
       带符号峰值 [5.11, 4.11, 4.4, 4.59]；方向一致性(四轮峰值均为正): OK。
```

| 项 | 实测值 | 判读 |
| --- | --- | --- |
| `/chassis/rpdo` | 603–703 Hz | 反馈轮询 10 ms + TPDO1 事件 |
| SDO `0x2F00` | 3553106 = `R76` | 在板固件 = repo HEAD |
| SDO `0x2052` car_state | 1（bit0 enable=1） | 门控全开（SWD 对照 gate_dbg=0x1F） |
| 电池（SWD `motor_vbat_mv`） | 12.36–12.37 V | > UV 阈值 10.5 V |
| 线级命令 0x201 | 364 counts = 3.64 rad/s | = 0.1 m/s ÷ 0.0275 m |
| 稳态反馈 | 3.4 rad/s 四轮同向 | 滑移损失 ~7%，正常 |
| 轮序/极性（v5 首次） | [FL,FR,RL,RR]=W1..W4，全 +1 | `wheel_sign` 默认 1 1 1 1 成立，v5 实测闭合 |

取证方法（下次单轮不动时直接抄）：

1. 线级证据：`candump can0,201:7FF`（命令）+ `can0,181:7FF`（反馈）同一时刻比对。
   注意薄桥轮命令是**事件驱动**（值变化才发一帧），抓帧必须从命令起始就开始，
   事后挂 candump 会错过全部写沿，误判成"桥没发命令"。
2. SDO 旁路：`ros2 service call /chassis/sdo_write canopen_interfaces/srv/COWrite
   "{index: 0x2000, subindex: 0, data: 364}"` 直接写轮目标，绕开 ROS 桥——
   轮动 = 桥/控制器侧问题，轮不动 = 门控/功率级问题。
3. SWD 直读：`v_ramp`/`wheel_speed`/`duty_q8`/`encoder_sign`/`car_drive_gate_dbg`
   （符号见 Debug ELF），读毕按 §5 释放，勿留 halt。

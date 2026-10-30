# TODO — 当前工作队列（极简版）

> 只记"做什么"，细节/命令/验收全在指向的文档里。做完一项勾一项。
> 更新日期：2026-10-30

## R76 固件（下次烧录一批做完）

- [x] F12：TIM6 丢拍计数器（UIF 残留检测，SWD 可读）—— `docs/CAN-BRINGUP-2026-10-30.md` §6.1
  （代码完成：`dbg_isr_overrun_cnt` @0x20009c40；烧录后冷启动核验才算闭合）
- [x] 低速 1/T 混合测速 + 积分亚 LSB + 静摩擦前馈 —— `docs/chassis-control-design.md` §6
  （代码完成：|counts/ms|<8 切 1/T 迟滞 12；duty_q8 亚 LSB；FF 75% 为 car2 转述值——**台架首测项**）
- [x] pitch/roll 竖装轴旋转修复 —— `docs/canopen-bitfields.md` §7
  （代码完成：水平系旋转 + atan2；**符号约定待现场核对**，核对前 Jetson 不消费 0x2031/0x2032）
- [ ] 构建双 preset 零警告 + `tools/canopen_gate.py` 绿 + 升 tag + 烧录（冷启动核 build_tag）
  （前三步已完成：Debug/Release 零警告、门禁全绿、tag=R76；**剩烧录 + 冷启动核验**。
  bin SHA-256 前 16：Debug `abeea499f6a14924` / Release `58ee9eb151fe08a7`）

## Jetson / 标定

- [ ] 轮径 + lx/ly 实测回填 URDF（mecanum 正解前提）—— `docs/firmware-capability-spec.md` §4
- [ ] ros2_canopen 装机 + `transport:=canopen` 首跑 —— ADR-0024 待闭合 #1、`docs/NEXT-2026-10-30.md` §5
- [x] rak_metrics 跟踪"平坦样本占比"（低速质量指标）—— `docs/chassis-control-design.md` §6
  （代码完成：joint_states 探针 flatness 统计 + 离线单测过；**待 Orin colcon 编译**）

## 融合与导航（EKF + Nav2）

- [ ] EKF 协方差：Allan 静态 + 直行 2 m 实测回填 —— `scripts/calibrate_imu.py`、ADR-0023
- [x] 抗打滑三级接入（检测→协方差膨胀→限幅）—— `docs/firmware-capability-spec.md` §7.2
  （代码完成：slip_watch 节点 L1 残差进 /rak/metrics + L3 gate 通路（默认关）+ L2 静态回退写入
  ekf.yaml 注释；离线单测过。**σ 标定与 gate 重接线排落地期**——架空期真打滑不可复现）
- [x] Nav2 `vx_min` 最低轮速钳位 —— `docs/chassis-control-design.md` §6
  （MPPI vx_min -0.10→+0.02，basis=推断待复核；倒车仍走 behavior_server BackUp）

## 规则 / 签字

- [x] AGENTS §3 ISR 口径改稿（35% → 不丢拍+双统计）—— 已定稿入 AGENTS.md（2026-10-30 签字）
- [ ] 控制升级 C1–C5 按序推进 —— `docs/chassis-control-design.md`
  （C1 的静摩擦前馈机制已随 R76 落地，增益重整定待台架）

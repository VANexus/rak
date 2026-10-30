# TODO — 当前工作队列（极简版）

> 只记"做什么"，细节/命令/验收全在指向的文档里。做完一项勾一项。
> 更新日期：2026-10-30

## R76 固件（下次烧录一批做完）

- [ ] F12：TIM6 丢拍计数器（UIF 残留检测，SWD 可读）—— `docs/CAN-BRINGUP-2026-10-30.md` §6.1
- [ ] 低速 1/T 混合测速 + 积分亚 LSB + 静摩擦前馈 —— `docs/chassis-control-design.md` §6
- [ ] pitch/roll 竖装轴旋转修复 —— `docs/canopen-bitfields.md` §7
- [ ] 构建双 preset 零警告 + `tools/canopen_gate.py` 绿 + 升 tag + 烧录（冷启动核 build_tag）

## Jetson / 标定

- [ ] 轮径 + lx/ly 实测回填 URDF（mecanum 正解前提）—— `docs/firmware-capability-spec.md` §4
- [ ] ros2_canopen 装机 + `transport:=canopen` 首跑 —— ADR-0024 待闭合 #1、`docs/NEXT-2026-10-30.md` §5
- [ ] rak_metrics 跟踪"平坦样本占比"（低速质量指标）—— `docs/chassis-control-design.md` §6

## 融合与导航（EKF + Nav2）

- [ ] EKF 协方差：Allan 静态 + 直行 2 m 实测回填 —— `scripts/calibrate_imu.py`、ADR-0023
- [ ] 抗打滑三级接入（检测→协方差膨胀→限幅）—— `docs/firmware-capability-spec.md` §7.2
- [ ] Nav2 `vx_min` 最低轮速钳位 —— `docs/chassis-control-design.md` §6

## 规则 / 签字

- [x] AGENTS §3 ISR 口径改稿（35% → 不丢拍+双统计）—— 已定稿入 AGENTS.md（2026-10-30 签字）
- [ ] 控制升级 C1–C5 按序推进 —— `docs/chassis-control-design.md`

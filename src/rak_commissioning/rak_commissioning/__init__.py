"""Commissioning / teach tools (ADR-0013).

标定与调试专用，不是生产运动接口。三个入口共用同一批底层知识：
- drive_tui        底盘驾驶 TUI（麦轮 IK + 状态面板，USB v2 DRIVE_COMMAND）
- arm_servo_tui    舵机示教 TUI（6 轴 µrad 位置，USB v2 ARM_COMMAND）
- chassis_key_test 键盘速度源（/cmd_vel* Twist，经 Nav2/控制器链路）

电机安全由 rak_chassis（hardware_interface）把关，默认 DISARMED。
"""

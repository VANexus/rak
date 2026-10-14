"""Commissioning / teach tools (ADR-0013, ADR-0016).

标定与调试专用，不是生产运动接口。工具只说标准 ROS 接口，绝不直连传输层：
- chassis_key_test 键盘速度源（发布 Twist → mecanum_drive_controller → CAN）

电机安全由 rak_chassis（hardware_interface）把关，默认 DISARMED。
"""

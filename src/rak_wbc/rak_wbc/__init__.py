"""rak_wbc — 速度级全身运动仲裁层（WBC/HQP）。

[AI] 模块边界（ADR-0011 D1）：
- model_builder / hqp 不 import rclpy：数值层可桌面回归（`test/`）。
- node 只做 ROS I/O 与状态机，不承担数学。
"""
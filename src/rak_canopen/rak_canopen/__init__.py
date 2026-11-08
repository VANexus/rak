"""rak_canopen —— CANopen v5 的 Jetson 侧契约层（ADR-0022）。

生产链路的协议栈是 lely-core（ADR-0021 已选），本包**不是**第二套协议实现：
它只把下位机 SSOT（`config/<node>.json`）+ 通信/单位契约派生成可执行事实，
用于离线单测、混跑门禁与明日台架的预期值核对。
"""

"""fake_arm_stream — 臂位置流的软件夹具（ADR-0011 D7/③ 路线）。

[AI] 为什么存在：`rak_chassis` 尚未导出臂位置接口（ADR-0011 待闭合 #1），但 HQP 的
P1/伺服模式需要在 ROS 链路上端到端可验。本节点照 `rak_arm_simulation` 的"软件夹具"
模式实现臂位置流的**消费端**：订阅 `/wbc/arm_setpoints`，回读为 `JointState`。

语义诚实性（重要）：回读的是**命令回显**（optimistic echo），不是测量反馈——与真机
`ARM_STATE` 的 `measured=unavailable` 语义一致（真机无关节位置传感器）。夹具不得被
当作测量源；抓取闭环仍要靠视觉（ADR-0008/ADR-0011 D4）。

[AI] 2026-10-28：对齐真机接口——名字用**硬件侧** `j1_base..j6_wrist_roll`、值用
**指令角域**（rad，0..270°，Home 0/225/270/0/0/0），即与 /joint_states 完全同构，
这样 WBC 的 joint_map 换算（指令角 <-> q_urdf）在影子链路上也被端到端走到。
（J1/J2 在真机 A2 起是编码器真测量，这里仍是回声——夹具不冒充测量。）

安全：本节点不接触任何硬件，只发 ROS 话题。缺 setpoint 超过 timeout 时保持最后位置
（对齐固件 lease 超时的"主动保持"语义）。
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from rak_wbc import joint_map

DEFAULT_JOINT_NAMES = list(joint_map.ARM_CTRL_JOINTS)
_HOME_CTRL = joint_map.q_to_ctrl(joint_map.HOME_Q)
PARAM_DEFAULTS = {
    "arm_setpoints_topic": "/wbc/arm_setpoints",
    "joint_states_topic": "/joint_states",
    "joint_names": DEFAULT_JOINT_NAMES,
    "init_positions": _HOME_CTRL,
    "publish_rate_hz": 50.0,
    "setpoint_timeout": 0.5,
}


class FakeArmStream(Node):
    def __init__(self):
        super().__init__("fake_arm_stream")
        for name, value in PARAM_DEFAULTS.items():
            self.declare_parameter(name, value)

        names = [str(item) for item in self.get_parameter("joint_names").value]
        init = [float(item) for item in self.get_parameter("init_positions").value]
        if len(names) != len(init):
            raise ValueError("joint_names 与 init_positions 长度不一致")
        self._names = names
        self._positions = list(init)
        self._last_setpoint_time = None
        self._timeout = float(self.get_parameter("setpoint_timeout").value)
        rate = float(self.get_parameter("publish_rate_hz").value)

        self.create_subscription(
            Float64MultiArray,
            str(self.get_parameter("arm_setpoints_topic").value),
            self._on_setpoints,
            10,
        )
        self._pub = self.create_publisher(
            JointState, str(self.get_parameter("joint_states_topic").value), 10
        )
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"fake_arm_stream（软件夹具，命令回显非测量）：{len(names)} 轴 @ {rate:.0f} Hz"
        )

    def _on_setpoints(self, msg: Float64MultiArray):
        if len(msg.data) != len(self._names):
            self.get_logger().warn(
                f"setpoint 长度 {len(msg.data)} != {len(self._names)}；丢弃",
                throttle_duration_sec=2.0,
            )
            return
        self._positions = [float(value) for value in msg.data]
        self._last_setpoint_time = self.get_clock().now()

    def _tick(self):
        if self._last_setpoint_time is not None and self._timeout > 0.0:
            age = (self.get_clock().now() - self._last_setpoint_time).nanoseconds * 1e-9
            if age > self._timeout:
                return  # 超时保持：停止回显新位置（等价"主动保持最后位置"）
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self._names)
        msg.position = list(self._positions)
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeArmStream()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

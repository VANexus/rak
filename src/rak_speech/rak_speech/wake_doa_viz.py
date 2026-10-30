"""Draw the mic array's sound-direction-of-arrival as an RViz arrow.

The R818 board only reports an angle together with a wake event (no continuous
bearing stream), so this is a decaying marker, not a live vector: an event fires,
the arrow shows where the sound came from, and it fades.

Audio has no RViz display. The DOA angle is the mic array's picture.
"""

import math

import rclpy
from geometry_msgs.msg import Point
from rclpy.lifecycle import TransitionCallbackReturn
from rak_interfaces.msg import WakeEvent
from rak_lifecycle.managed_node import ManagedNode
from visualization_msgs.msg import Marker


class WakeDoaViz(ManagedNode):
    """Marker fan-out for the array's bearing, split along ADR-0020 §5.

    Nothing here is expensive, but the contract is the same as every other managed
    node: parameters and entities in `on_configure`, no publishing before
    `on_activate`.
    """

    def __init__(self):
        super().__init__("wake_doa_viz")
        # 只做字段初始化：参数、发布器、订阅都归 on_configure（ADR-0020 §5）。
        self.frame = "mic_array_link"
        self.length = 0.35
        self.lifetime = 8.0
        self._seq = 0
        self.pub = None
        self.sub = None

    def on_configure(self, state) -> TransitionCallbackReturn:
        super().on_configure(state)
        wake_topic = str(self.declare("wake_topic", "/speech/wake"))
        marker_topic = str(self.declare("marker_topic", "/wake_doa"))
        self.frame = str(self.declare("frame_id", "mic_array_link"))
        self.length = float(self.declare("length", 0.35))
        self.lifetime = float(self.declare("lifetime_sec", 8.0))
        self._seq = 0

        # 未激活不发布：发布器一律走受管包装。
        self.pub = self.lifecycle_publisher(Marker, marker_topic, 10)
        try:
            self.sub = self.create_subscription(WakeEvent, wake_topic, self._on_wake, 10)
        except Exception as exc:  # noqa: BLE001 -- configure 失败必须带可读原因
            return self.fail(f"订阅 {wake_topic} 失败: {exc}")
        self.get_logger().info(
            "wake DOA -> marker ready; audio has no RViz display, the arrow is it"
        )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state) -> TransitionCallbackReturn:
        ret = super().on_activate(state)
        if ret != TransitionCallbackReturn.SUCCESS:
            return ret
        self._seq = 0
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state) -> TransitionCallbackReturn:
        return super().on_deactivate(state)

    def on_cleanup(self, state) -> TransitionCallbackReturn:
        if self.sub is not None:
            try:
                self.destroy_subscription(self.sub)
            except Exception:  # noqa: BLE001 -- 退出路径不打新错
                pass
            self.sub = None
        return super().on_cleanup(state)

    def _on_wake(self, msg: WakeEvent) -> None:
        # WakeEvent.angle is the circular array's own bearing in degrees (0-365),
        # NOT robot heading -- docs/microphone_array.md. Display it as-is in the
        # array frame; rotating to the robot frame needs a calibration we lack.
        theta = math.radians(float(msg.angle) % 360.0)
        end_x = self.length * math.cos(theta)
        end_y = self.length * math.sin(theta)

        m = Marker()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.frame
        self._seq += 1
        m.ns = "wake_doa"
        m.id = 0
        m.type = Marker.ARROW
        m.action = Marker.ADD

        # Two points define the arrow: origin -> bearing. Simpler than hand-rolling
        # an orientation quaternion, and RViz widths the shaft/head for us.
        for x, y in ((0.0, 0.0), (end_x, end_y)):
            p = Point()
            p.x, p.y, p.z = x, y, 0.0
            m.points.append(p)

        m.scale.x = 0.04   # shaft diameter
        m.scale.y = 0.09   # head diameter
        m.scale.z = 0.12   # head length
        m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.9, 0.9, 1.0
        m.lifetime.sec = int(self.lifetime)
        m.lifetime.nanosec = int((self.lifetime % 1.0) * 1e9)
        m.text = f"{msg.keyword} {msg.angle:.0f} deg"
        m.pose.orientation.w = 1.0

        self.publish(m, publisher=self.pub)
        self.get_logger().info(
            f"wake '{msg.keyword}' score={msg.score:.3f} angle={msg.angle:.1f} deg "
            f"-> marker drawn"
        )


def main(args=None):
    rclpy.init(args=args)
    node = WakeDoaViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

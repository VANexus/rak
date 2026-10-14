"""Draw the mic array's sound-direction-of-arrival as an RViz arrow.

The R818 board only reports an angle together with a wake event (no continuous
bearing stream), so this is a decaying marker, not a live vector: an event fires,
the arrow shows where the sound came from, and it fades.

Audio has no RViz display. The DOA angle is the mic array's picture.
"""

import math

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rak_interfaces.msg import WakeEvent
from visualization_msgs.msg import Marker


class WakeDoaViz(Node):
    def __init__(self):
        super().__init__("wake_doa_viz")
        self.declare_parameter("wake_topic", "/speech/wake")
        self.declare_parameter("marker_topic", "/wake_doa")
        self.declare_parameter("frame_id", "mic_array_link")
        self.declare_parameter("length", 0.35)
        self.declare_parameter("lifetime_sec", 8.0)

        self.frame = str(self.get_parameter("frame_id").value)
        self.length = float(self.get_parameter("length").value)
        self.lifetime = float(self.get_parameter("lifetime_sec").value)
        self._seq = 0

        self.pub = self.create_publisher(
            Marker, str(self.get_parameter("marker_topic").value), 10
        )
        self.sub = self.create_subscription(
            WakeEvent, str(self.get_parameter("wake_topic").value), self._on_wake, 10
        )
        self.get_logger().info(
            "wake DOA -> marker ready; audio has no RViz display, the arrow is it"
        )

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

        self.pub.publish(m)
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

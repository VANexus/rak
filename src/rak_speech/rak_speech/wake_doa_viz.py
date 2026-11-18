"""Draw the mic array's sound-bearing as a live sector fan in RViz.

The R818 board reports a bearing only together with a wake event (no
continuous bearing stream yet, docs/microphone_array.md), so this node holds
the latest bearing and repaints at a fixed rate. Each arriving event kicks a
short pulse (the fan opens from zero width to full width over ~0.12 s), which
reads as "heard you over there"; a bounded trail of recent sectors fades
oldest-to-newest, so circling while talking paints a fan of light around the
robot.

Sources (one marker pipeline, two subscriptions):
    /speech/wake (WakeEvent) -- wake-word confirmed bearing, bright cyan.
    /speech/doa  (WakeEvent) -- optional plain-speech bearing stream; the
        board firmware is still being probed for it, so this topic may simply
        stay silent (docs/microphone_array.md). Purple when it flows.

Markers carry a short lifetime (a few publish periods): as long as the node
runs the view is refreshed; if it dies the fans disappear instead of lying
about a bearing nobody measures anymore.

Audio has no RViz display. The DOA bearing is the mic array's picture.
"""

import json
import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import Point
from rclpy.lifecycle import TransitionCallbackReturn
from rak_interfaces.msg import WakeEvent
from rak_lifecycle.managed_node import ManagedNode
from std_msgs.msg import String
from visualization_msgs.msg import Marker

#: Marker namespace kept for the existing sensors_rviz.rviz / doa_rviz.rviz
#: namespace toggle (`wake_doa: true`).
NS = "wake_doa"
#: id 0 = current sector, 1..trail_size = trail slots, next id = status text.
CURRENT_ID = 0
TEXT_ID_OFFSET = 1
#: Keep every marker alive only while the republish timer is healthy.
MARKER_LIFETIME_SEC = 1.5

#: Sector colours: wake-confirmed vs plain-speech (provisional source).
_WAKE_RGB = (0.10, 0.90, 0.90)
_SPEECH_RGB = (0.40, 0.30, 1.00)
#: Trail colour ramp: oldest -> dim blue, newest -> bright cyan.
_TRAIL_LO = (0.05, 0.15, 0.30)
_TRAIL_HI = (0.10, 0.90, 0.90)


def fan_points(
    angle_deg: float,
    radius: float,
    half_width_deg: float,
    segments: int,
) -> list[tuple[float, float]]:
    """Triangle-list fan around `angle_deg` (array frame, 0-365 deg).

    RViz TRIANGLE_LIST wants every triangle as three consecutive vertices;
    each slice is (centre, arc_i, arc_i+1), so the fan is a solid pie wedge.
    """
    centre = (0.0, 0.0)
    half = math.radians(half_width_deg)
    theta = math.radians(angle_deg % 360.0)
    seg = max(1, int(segments))
    pts: list[tuple[float, float]] = []
    for i in range(seg):
        a0 = theta - half + (2.0 * half) * i / seg
        a1 = theta - half + (2.0 * half) * (i + 1) / seg
        pts.append(centre)
        pts.append((radius * math.cos(a0), radius * math.sin(a0)))
        pts.append((radius * math.cos(a1), radius * math.sin(a1)))
    return pts


def trail_color(frac: float) -> tuple[float, float, float, float]:
    """Colour for a trail slot; frac 0 = oldest, 1 = newest. Pure, testable."""
    frac = min(1.0, max(0.0, frac))
    rgb = [_TRAIL_LO[i] + (_TRAIL_HI[i] - _TRAIL_LO[i]) * frac for i in range(3)]
    return (*rgb, 0.20 + 0.55 * frac)


class TrailBuffer:
    """Bounded FIFO of recent bearings, oldest first, angle wrapped to 0-360."""

    def __init__(self, maxlen: int) -> None:
        self._maxlen = max(1, int(maxlen))
        self._items: list[float] = []

    def push(self, angle_deg: float) -> None:
        self._items.append(angle_deg % 360.0)
        if len(self._items) > self._maxlen:
            del self._items[: len(self._items) - self._maxlen]

    @property
    def capacity(self) -> int:
        return self._maxlen

    def __len__(self) -> int:
        return len(self._items)

    def __iter__(self):
        return iter(self._items)


class WakeDoaViz(ManagedNode):
    """受管版本（ADR-0020 §5）：参数/发布器/订阅在 configure，重绘 timer 在 activate。"""

    def __init__(self):
        super().__init__("wake_doa_viz")
        self.pub = None
        self.frame = "mic_array_link"
        self.radius = 0.5
        self.half_width = 15.0
        self.segments = 12
        self.stale_sec = 30.0
        self.pulse_sec = 0.4
        self.trail = TrailBuffer(16)
        self.rate = 10.0

        # Last-known bearing: None until the first event ever arrives.
        self._angle: float | None = None
        self._keyword = ""
        self._source = "wake"  # "wake" | "speech"
        self._last_event = 0.0  # monotonic; drives staleness dimming
        self._pulse_start = 0.0  # monotonic; 0 = no pulse ran yet
        self._speaker = ""  # latest identity from speaker_id, if any
        self._speaker_at = 0.0

    def on_configure(self, state):
        super().on_configure(state)
        wake_topic = str(self.declare("wake_topic", "/speech/wake"))
        doa_topic = str(self.declare("doa_topic", "/speech/doa"))
        speaker_topic = str(self.declare("speaker_topic", "/speech/speaker"))
        marker_topic = str(self.declare("marker_topic", "/wake_doa"))
        self.frame = str(self.declare("frame_id", "mic_array_link"))
        self.radius = float(self.declare("radius", 0.5))
        self.half_width = float(self.declare("half_width_deg", 15.0))
        self.segments = int(self.declare("segments", 12))
        self.stale_sec = float(self.declare("stale_sec", 30.0))
        self.pulse_sec = float(self.declare("pulse_sec", 0.4))
        self.trail = TrailBuffer(int(self.declare("trail_size", 16)))
        self.rate = float(self.declare("publish_rate", 10.0))

        # 未激活不发布：LifecyclePublisher 在 activate 之前丢弃 publish()
        self.pub = self.lifecycle_publisher(Marker, marker_topic, 10)
        self.create_subscription(WakeEvent, wake_topic, self._on_wake, 10)
        # Optional plain-speech bearing source; silent until the firmware probe
        # proves the board emits one (docs/microphone_array.md).
        self.create_subscription(WakeEvent, doa_topic, self._on_doa, 10)
        # Speaker identity rides along in the status text (speaker_node).
        self.create_subscription(String, speaker_topic, self._on_speaker, 10)

        self._delete_all()
        self.report("viz", DiagnosticStatus.OK,
                    f"fan ±{self.half_width:.0f}° trail={self.trail.capacity}",
                    {"frame": self.frame, "rate_hz": f"{self.rate:.0f}"})
        self.get_logger().info(
            "DOA fan configured; repaint at "
            f"{self.rate:.0f} Hz, fan ±{self.half_width:.0f}°, trail holds up "
            f"to {self.trail.capacity} bearings (array frame, no heading calibration)"
        )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        ret = super().on_activate(state)
        if ret != TransitionCallbackReturn.SUCCESS:
            return ret
        period = 1.0 / self.rate if self.rate > 0.0 else 0.1
        self.add_timer(period, self._tick)
        return TransitionCallbackReturn.SUCCESS

    def _delete_all(self) -> None:
        m = Marker()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.frame
        m.action = Marker.DELETEALL
        self.publish(m, publisher=self.pub)

    def _update(self, msg: WakeEvent, source: str) -> None:
        # WakeEvent.angle is the circular array's own bearing in degrees (0-365),
        # NOT robot heading -- docs/microphone_array.md. Display it as-is in the
        # array frame; rotating to the robot frame needs a calibration we lack.
        self._angle = float(msg.angle) % 360.0
        self._keyword = str(msg.keyword)
        self._source = source
        now = time.monotonic()
        self._last_event = now
        self._pulse_start = now
        self.trail.push(self._angle)
        if source == "wake":
            self.get_logger().info(
                f"wake '{msg.keyword}' score={msg.score:.3f} angle={msg.angle:.1f} deg "
                "-> fan held, trail updated"
            )

    def _on_wake(self, msg: WakeEvent) -> None:
        self._update(msg, "wake")

    def _on_doa(self, msg: WakeEvent) -> None:
        self._update(msg, "speech")

    def _on_speaker(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except ValueError:
            return
        name = str(payload.get("speaker", ""))
        if name:
            self._speaker = name
            self._speaker_at = time.monotonic()

    def _tick(self) -> None:
        if self._angle is None:
            return  # nothing ever detected: publish nothing rather than a guess
        self.mark_seen("marker_topic")

        now = time.monotonic()
        stale = (now - self._last_event) > self.stale_sec
        stamp = self.get_clock().now().to_msg()
        lifetime = self._lifetime_msg()

        # -- pulse: fan opens from zero width to full width after an event ----
        elapsed = now - self._pulse_start
        if elapsed < self.pulse_sec:
            frac = min(1.0, elapsed / 0.12)
        else:
            frac = 1.0
        width = self.half_width * frac if frac > 0.0 else self.half_width

        # -- current bearing: id 0, always present while the node runs --------
        rgb = _WAKE_RGB if self._source == "wake" else _SPEECH_RGB
        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = self.frame
        m.ns = NS
        m.id = CURRENT_ID
        m.type = Marker.TRIANGLE_LIST
        m.action = Marker.ADD
        for x, y in fan_points(self._angle, self.radius, width, self.segments):
            p = Point()
            p.x, p.y, p.z = x, y, 0.0
            m.points.append(p)
        m.scale.x = m.scale.y = m.scale.z = 1.0
        m.color.r, m.color.g, m.color.b = rgb
        m.color.a = 0.35 if stale else 0.85
        m.lifetime = lifetime
        m.pose.orientation.w = 1.0
        self.publish(m, publisher=self.pub)

        # -- status text: id right after the trail slots ---------------------
        t = Marker()
        t.header.stamp = stamp
        t.header.frame_id = self.frame
        t.ns = NS
        t.id = TEXT_ID_OFFSET + self.trail.capacity
        t.type = Marker.TEXT_VIEW_FACING
        t.action = Marker.ADD
        t.pose.position.x = 0.0
        t.pose.position.y = 0.0
        t.pose.position.z = 0.18
        t.pose.orientation.w = 1.0
        t.scale.z = 0.08
        t.color.r = t.color.g = t.color.b = 0.9
        t.color.a = 0.8 if not stale else 0.4
        kind = "wake" if self._source == "wake" else "voice"
        suffix = " (stale)" if stale else ""
        who = ""
        if self._speaker and self._speaker != "unknown" and (
            now - self._speaker_at
        ) < 20.0:
            who = f"{self._speaker} "
        t.text = f"{who}{self._angle:.0f} deg {kind}{suffix}"
        t.lifetime = lifetime
        self.publish(t, publisher=self.pub)

        # -- trail: ids 1..N, oldest dim .. newest bright ---------------------
        n = len(self.trail)
        for i, angle in enumerate(self.trail):
            frac_i = (i / (n - 1)) if n > 1 else 1.0
            tr = Marker()
            tr.header.stamp = stamp
            tr.header.frame_id = self.frame
            tr.ns = NS
            tr.id = TEXT_ID_OFFSET + i
            tr.type = Marker.TRIANGLE_LIST
            tr.action = Marker.ADD
            for x, y in fan_points(angle, self.radius * 0.8, self.half_width, self.segments):
                p = Point()
                p.x, p.y, p.z = x, y, 0.0
                tr.points.append(p)
            tr.scale.x = tr.scale.y = tr.scale.z = 1.0
            r, g, b, a = trail_color(frac_i)
            tr.color.r, tr.color.g, tr.color.b, tr.color.a = r, g, b, a
            tr.lifetime = lifetime
            tr.pose.orientation.w = 1.0
            self.publish(tr, publisher=self.pub)

    def _lifetime_msg(self):
        # Marker.lifetime is a default-constructed Duration; fill it in place
        # rather than importing builtin_interfaces just for the constructor.
        d = Marker().lifetime
        d.sec = int(MARKER_LIFETIME_SEC)
        d.nanosec = int((MARKER_LIFETIME_SEC % 1.0) * 1e9)
        return d


def main(args=None):
    rclpy.init(args=args)
    node = WakeDoaViz()
    # Lifecycle is driven by the composition layer: `managed()` in
    # sensors.launch.py, or system_manager when supervised:=true. Do not
    # configure/activate here -- doing so races those events and creates a
    # second lifecycle authority (ADR-0020 §3). For a bare `ros2 run`,
    # drive it explicitly with `ros2 lifecycle set /wake_doa_viz configure`
    # and `... activate`.
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

"""rak_metrics —— 把"这栈现在快不快、数据新不新、丢没丢"变成一条可读的答案。

存在理由（`docs/architecture.md` 自己写着"158 ms 不是服务等级"）：在此之前，
`/diagnostics` 只告诉每个域活没活，没人回答下面四个问题——

1. **命令年龄**：`/cmd_vel` 现在这帧是多久以前的？
2. **反馈年龄**：`/joint_states` 的 header 落后多少？
3. **链路延迟**：每个传感流 `now - header.stamp` 的 p50/p95/p99 是多少？
4. **丢帧**：相邻帧间隔超过标称周期 2.5 倍的次数。

设计约束：

* **不新增消息契约**（ADR-0020 §9）：全部以 `diagnostic_msgs/DiagnosticArray`
  发到 `/rak/metrics`，键名即指标名，rqt 与 `ros2 topic echo` 都能读。
* **不增加 GPU/模型依赖**：纯 rclpy + stdlib。轻 HTTP 用 `http.server`，
  默认绑 127.0.0.1:8088（板上经 SSH 转发访问）。
* **常量级内存**：每个探针一个 200 样本滑窗，没有累积历史。

事实分级（AGENTS §4）：阈值来自探针配置里的 `nominal_hz`，逐条带 `basis`
（measured / nominal / placeholder）。**没有实测依据的阈值不当作事实**。
"""

from __future__ import annotations

import json
import threading
from typing import Any

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from rak_lifecycle.managed_node import ManagedNode

from .stats import FlatnessTracker, RateMeter, Window

#: 探针类型 → 消息类（模块名, 类名）。缺包时跳过该探针，不整体失败。
_PROBE_TYPES = {
    "imu": ("sensor_msgs.msg", "Imu"),
    "scan": ("sensor_msgs.msg", "LaserScan"),
    "odom": ("nav_msgs.msg", "Odometry"),
    "joint_state": ("sensor_msgs.msg", "JointState"),
    "image": ("sensor_msgs.msg", "Image"),
    "pointcloud": ("sensor_msgs.msg", "PointCloud2"),
    "twist": ("geometry_msgs.msg", "Twist"),
}

#: 默认探针。频率与依据逐条给出处，不写"看起来差不多"的数。
DEFAULT_PROBES = [
    {"name": "imu", "topic": "/imu/data_raw", "type": "imu", "nominal_hz": 1000.0,
     "basis": "measured", "note": "2026-10-29 现场 903.6 Hz"},
    {"name": "scan", "topic": "/scan", "type": "scan", "nominal_hz": 10.0,
     "basis": "nominal", "note": "N10 标称 10 Hz，本机未复测"},
    {"name": "odom", "topic": "/odometry/filtered", "type": "odom", "nominal_hz": 30.0,
     "basis": "measured", "note": "2026-10-29 现场 30.0 Hz"},
    {"name": "joint_states", "topic": "/joint_states", "type": "joint_state",
     "nominal_hz": 100.0, "basis": "nominal", "note": "控制器标称",
     # [AI] 低速质量指标（chassis-control-design §6）：四轮速度反馈的台阶统计。
     # watch=关节名子串匹配；moving_eps(rad/s) 区分"在动"与"停着"——basis
     # placeholder，待落地期用首日基线标定。
     "flatness": True, "watch": ["wheel"], "moving_eps": 0.05},
    {"name": "cmd_vel", "topic": "/cmd_vel", "type": "twist", "nominal_hz": 20.0,
     "basis": "nominal", "note": "Nav2 controller 默认 20 Hz"},
]

WINDOW = 200


def _load_class(module_name: str, class_name: str):
    module = __import__(module_name, fromlist=[class_name])
    return getattr(module, class_name)


class MetricProbe:
    """一个话题的滑窗、频率计与到达时刻。"""

    def __init__(self, spec: dict[str, Any]) -> None:
        self.name = str(spec["name"])
        self.topic = str(spec["topic"])
        self.type_name = str(spec["type"])
        nominal_hz = float(spec.get("nominal_hz", 0.0) or 0.0)
        self.nominal_hz = nominal_hz
        self.basis = str(spec.get("basis", "placeholder"))
        self.note = str(spec.get("note", ""))
        period = None if nominal_hz <= 0 else 1.0 / nominal_hz
        self.age = Window(WINDOW, period)
        self.interval = Window(WINDOW, period)
        self.rate = RateMeter()
        self.received = 0
        self.rate_hz = 0.0
        self._last_receive_s: float | None = None
        # [AI] 低速质量指标（chassis-control-design §6）：joint_state 探针可选
        # 开启平坦样本统计（flatness/watch/moving_eps，见 DEFAULT_PROBES）。
        self.flatness: FlatnessTracker | None = None
        if spec.get("flatness"):
            self.flatness = FlatnessTracker(
                WINDOW, moving_eps=float(spec.get("moving_eps", 0.05)))
            self._watch = tuple(str(w) for w in spec.get("watch", ()))

    def observe(self, now_s: float, stamp_s: float | None) -> None:
        self.received += 1
        self.rate_hz = self.rate.tick(now_s)
        if self._last_receive_s is not None:
            self.interval.add((now_s - self._last_receive_s) * 1e3)
            self.interval.observe_gap(now_s)
        self._last_receive_s = now_s
        if stamp_s is not None:
            self.age.add((now_s - stamp_s) * 1e3)
            self.age.observe_gap(stamp_s)

    def observe_joint_state(self, name: str, velocity: float | None) -> None:
        """喂平坦样本统计（joint_state 探针专用；watch 子串匹配关节名）。"""
        if self.flatness is None or velocity is None:
            return
        if self._watch and not any(w in name for w in self._watch):
            return
        self.flatness.observe(float(velocity))

    def summary(self) -> dict[str, Any]:
        age = self.age.summary()
        interval = self.interval.summary()
        out = {
            "topic": self.topic,
            "rate_hz": round(self.rate_hz, 2),
            "received": self.received,
            "age_p50_ms": age.get("p50"),
            "age_p95_ms": age.get("p95"),
            "age_p99_ms": age.get("p99"),
            "interval_p95_ms": interval.get("p95"),
            "gap_events": int(age.get("gap_events", 0) or 0),
            "basis": self.basis,
            "note": self.note,
        }
        if self.flatness is not None:
            out.update(self.flatness.summary())
        return out

    def age_budget_ms(self) -> float | None:
        """p95 超过它就告警：2 个标称周期 + 20 ms 调度余量。"""
        if self.nominal_hz <= 0:
            return None
        return 2.0 * 1e3 / self.nominal_hz + 20.0


class MetricsNode(ManagedNode):
    """受管节点（ADR-0020）：探针在 configure 建，窗口在 activate 才滚。"""

    def __init__(self) -> None:
        super().__init__("rak_metrics", health_rate_hz=0.2)
        self._probes: dict[str, MetricProbe] = {}
        self._subs: list[Any] = []
        self._metrics_pub = None
        self._snapshot: dict[str, Any] = {}
        self._http: Any = None
        self._lock = threading.Lock()

    # ---------------------------------------------------------------- lifecycle
    def on_configure(self, state):
        super().on_configure(state)
        specs = self._probe_specs()
        qos = QoSProfile(
            depth=50,
            history=HistoryPolicy.KEEP_LAST,
            # 传感流是 best_effort；用 reliable 订阅 best_effort 发布者会建不起连接。
            # 指标节点宁可少收，也不要因为自己的 QoS 影响别人。
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        skipped: list[str] = []
        for spec in specs:
            entry = _PROBE_TYPES.get(str(spec.get("type")))
            if entry is None:
                skipped.append(f"{spec.get('name')}(未知类型 {spec.get('type')})")
                continue
            try:
                message_type = _load_class(*entry)
            except Exception as exc:  # noqa: BLE001 - 缺包是预期情况（桌面档）
                skipped.append(f"{spec.get('name')}({exc})")
                continue
            probe = MetricProbe(spec)
            self._probes[probe.name] = probe
            self._subs.append(self.create_subscription(
                message_type, probe.topic,
                lambda msg, p=probe: self._on_message(p, msg), qos,
            ))
        if skipped:
            self.get_logger().warning(f"跳过的探针: {', '.join(skipped)}")
        if not self._probes:
            return self.fail("没有任何可用探针：检查话题与消息包是否安装")

        self._metrics_pub = self.lifecycle_publisher(DiagnosticArray, "/rak/metrics", 10)
        self.declare("http_enabled", True)
        self.declare("http_port", 8088)
        self.declare("http_host", "127.0.0.1")
        self.get_logger().info(f"探针 {len(self._probes)} 个: {', '.join(sorted(self._probes))}")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        ret = super().on_activate(state)
        if ret != TransitionCallbackReturn.SUCCESS:
            return ret
        for probe in self._probes.values():
            probe.received = 0
            probe.rate_hz = 0.0
        self.add_timer(2.0, self._publish_snapshot)
        if bool(self.get_parameter("http_enabled").value):
            self._start_http()
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        self._stop_http()
        return super().on_deactivate(state)

    def on_cleanup(self, state):
        self._stop_http()
        self._subs.clear()
        self._probes.clear()
        self._metrics_pub = None
        return super().on_cleanup(state)

    # ------------------------------------------------------------------ probes
    def _probe_specs(self) -> list[dict[str, Any]]:
        """参数优先：`probes` 是 YAML 字符串，给了就完全替换默认表。

        为什么是字符串而不是列表参数：rclpy 不能声明"无元素列表"（推不出类型），
        而 launch 传 YAML 列表又会被当成字符串 —— 直接把边界定成"YAML 文本"最省事。
        """
        import yaml

        raw = str(self.declare("probes", "") or "").strip()
        if not raw:
            return list(DEFAULT_PROBES)
        parsed = yaml.safe_load(raw)
        if not isinstance(parsed, list) or not parsed:
            raise ValueError("probes 必须是 YAML 列表")
        return parsed

    def _on_message(self, probe: MetricProbe, message: Any) -> None:
        if not self.active:
            return
        now_s = self.get_clock().now().nanoseconds / 1e9
        stamp_s = None
        header = getattr(message, "header", None)
        if header is not None and hasattr(header, "stamp"):
            stamp_s = header.stamp.sec + header.stamp.nanosec * 1e-9
        probe.observe(now_s, stamp_s)
        # [AI] joint_state 探针：喂平坦样本统计（低速质量指标，§6）
        if probe.flatness is not None and probe.type_name == "joint_state":
            names = getattr(message, "name", None) or []
            velocities = getattr(message, "velocity", None) or []
            for name, vel in zip(names, velocities):
                probe.observe_joint_state(name, vel)
        self.mark_seen(probe.topic)

    # ----------------------------------------------------------------- publish
    def _publish_snapshot(self) -> None:
        statuses: list[DiagnosticStatus] = []
        snapshot: dict[str, Any] = {}
        worst = 0.0
        for name in sorted(self._probes):
            probe = self._probes[name]
            summary = probe.summary()
            snapshot[name] = summary
            level = DiagnosticStatus.OK
            budget = probe.age_budget_ms()
            age_p95 = summary.get("age_p95_ms")
            if summary["received"] == 0:
                level = DiagnosticStatus.WARN
            elif budget is not None and isinstance(age_p95, float) and age_p95 > budget:
                level = DiagnosticStatus.WARN
            p99 = summary.get("age_p99_ms")
            if isinstance(p99, float) and p99 == p99:
                worst = max(worst, p99)
            statuses.append(DiagnosticStatus(
                level=level,
                name=f"rak_metrics: {name}",
                message=f"{summary['topic']} @{summary['rate_hz']:.1f}Hz",
                hardware_id="rak_metrics",
                values=[KeyValue(key=str(k), value=_fmt(v)) for k, v in summary.items()],
            ))
        with self._lock:
            self._snapshot = snapshot
        self.report("probes", DiagnosticStatus.OK, f"{len(statuses)} 个探针", {
            "count": len(statuses),
            "worst_age_p99_ms": f"{worst:.2f}",
        })
        if self._metrics_pub is not None:
            self.publish(DiagnosticArray(status=statuses), publisher=self._metrics_pub)

    # -------------------------------------------------------------------- http
    def _start_http(self) -> None:
        port = int(self.get_parameter("http_port").value)
        host = str(self.get_parameter("http_host").value)
        try:
            self._http = _JsonServer(host, port, self._json_payload)
            self._http.start()
            self.get_logger().info(f"metrics HTTP: http://{host}:{port}/metrics")
        except OSError as exc:
            # 端口占用不该让域 FAULTED —— 指标是观测，不是功能。
            self.report("http", DiagnosticStatus.WARN, f"HTTP 启动失败: {exc}", {"port": port})

    def _stop_http(self) -> None:
        if self._http is not None:
            self._http.stop()
            self._http = None

    def _json_payload(self) -> dict[str, Any]:
        with self._lock:
            return {"probes": self._snapshot}


class _JsonServer:
    """stdlib 的极小 HTTP 服务：GET /metrics → JSON。"""

    def __init__(self, host: str, port: int, payload) -> None:
        from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self) -> None:  # noqa: N802 - stdlib 命名
                data = json.dumps(payload(), ensure_ascii=False).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def log_message(self, *args) -> None:  # 别把访问日志塞进 ROS 日志
                return

        self._server = ThreadingHTTPServer((host, port), Handler)
        self._thread = threading.Thread(
            target=self._server.serve_forever, name="rak-metrics-http", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        try:
            self._server.shutdown()
            self._server.server_close()
        except Exception:  # noqa: BLE001 - 关停路径不抛
            pass


def _fmt(value: Any) -> str:
    if isinstance(value, float):
        return "nan" if value != value else f"{value:.2f}"
    return str(value)


def main(args=None) -> None:
    import rclpy

    rclpy.init(args=args)
    node = MetricsNode()
    # 不要在 main() 里自己 trigger_configure/activate：受管节点由 launch 的
    # managed()（或 system_manager）驱动转移，自己再来一次会和它抢同一个状态机，
    # 结果是 "Transition is not registered" 直接退出（本轮实测）。
    # 单独 `ros2 run` 调试时才手工驱动：
    #   ros2 lifecycle set /rak_metrics configure && ros2 lifecycle set /rak_metrics activate
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

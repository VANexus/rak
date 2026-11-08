"""ROS 2 edge: the brain's tick loop as a node.

Ticks are event-driven -- a perception message drives one, so `guard()` runs on
every frame the sensor actually produced instead of on a wall clock that can
drift away from the data. A watchdog covers the other case: perception stopping
entirely.

What crosses the wire is perception IN and decisions OUT, which are the two
edges the design allows. System 1 does not cross: its decider is constructed
and called in this process. Putting it
behind a topic or a service would put a cross-process boundary on the fast
path, which the architecture forbids.

ROS must be sourced before this module is imported -- `rclpy` comes from
/opt/ros/jazzy and `rak_interfaces.msg` from the colcon install space.

The node is `ManagedNode` (ADR-0020 §5): the TensorRT engine is opened in
`on_configure` and released in `on_cleanup`, so `deactivate -> activate` restarts
the tick loop without re-paying an 800 MiB weight load on a 4 GB board.

    ros2 launch rak_bringup agent.launch.py                        # autostarts
    ros2 run rak_agent agent_node --ros-args -p decider:=stub -p sim:=true
    ros2 lifecycle set /brain configure && ros2 lifecycle set /brain activate
"""

from __future__ import annotations

import threading
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from rclpy.executors import ExternalShutdownException
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from rak_lifecycle.managed_node import ManagedNode
from rak_interfaces.msg import Action as ActionMsg
from rak_interfaces.msg import Decision as DecisionMsg
from rak_interfaces.msg import TrackedObject
from rak_interfaces.msg import TrackedObjects

from .body import MockBody
from .loop import Logger, _drain  # noqa: PLC2701 -- same package, one tick pipeline
from .system1 import Decider, StubDecider, guard, make_decider, resolve
from .system2 import Consultant, StubConsultant
from .types import Action, Decision, Entity, SceneState, Tier

PERCEPTION_TOPIC = "/perception/tracked_objects"
DECISION_TOPIC = "/brain/decision"


def _action_to_msg(a: Action) -> ActionMsg:
    """types.Action -> rak_interfaces/msg/Action. The pydantic model stays authoritative."""
    m = ActionMsg()
    m.kind = a.kind
    m.has_target = a.target_id is not None
    m.target_id = a.target_id if a.target_id is not None else 0
    m.reason = a.reason
    return m


def _state_from_msg(msg: TrackedObjects, scene: str, flags: list[str]) -> SceneState:
    """rak_interfaces/msg/TrackedObjects -> SceneState.

    Raises on anything that would silently corrupt a decision: a non-finite
    coordinate reads as False in every comparison `guard()` makes, so a NaN from
    a failed tracker would disarm the veto rather than trip it.
    """
    stamp = msg.header.stamp
    t = stamp.sec + stamp.nanosec * 1e-9
    if t == 0.0:  # publisher did not stamp; fall back to the node clock
        t = time.time()

    entities = []
    for o in msg.objects:
        coords = (o.position.x, o.position.y, o.position.z, o.velocity.linear.x, o.velocity.linear.z)
        if not all(c == c and abs(c) != float("inf") for c in coords):  # NaN/inf
            raise ValueError(f"non-finite geometry in track {o.track_id}")
        entities.append(
            Entity(
                id=int(o.track_id),
                label=str(o.label).strip().lower(),
                score=float(o.score),
                x=float(o.position.x),
                y=float(o.position.y),
                z=float(o.position.z),
                vx=float(o.velocity.linear.x),
                vy=float(o.velocity.linear.y),
                vz=float(o.velocity.linear.z),
            )
        )
    return SceneState(t=t, scene=scene, flags=flags, entities=entities)


def _state_to_msg(state: SceneState, clock, frame_id: str) -> TrackedObjects:
    """SceneState -> rak_interfaces/msg/TrackedObjects. Used by the built-in simulator."""
    msg = TrackedObjects()
    msg.header.stamp = clock.now().to_msg()
    msg.header.frame_id = frame_id
    objects = []
    for e in sorted(state.entities, key=lambda e: e.id):
        o = TrackedObject()
        o.track_id = e.id
        o.label = e.label
        o.score = e.score
        o.position.x, o.position.y, o.position.z = e.x, e.y, e.z
        # `size` is not modelled by the scripted body; zeros mean "unknown".
        o.velocity.linear.x, o.velocity.linear.y, o.velocity.linear.z = e.vx, e.vy, e.vz
        objects.append(o)
    msg.objects = objects
    return msg


def _make_decider(name: str, model_dir: str = "") -> Decider:
    """Backwards-compatible alias; the factory itself lives in `system1`."""
    return make_decider(name, model_dir)


#: `report()` 里的引擎后端名（ADR-0020 §5 的健康字段）。
_ENGINE_BACKENDS = {"tensorrt": "trt", "stub": "text"}


class BrainNode(ManagedNode):
    """Runs guard/gate/resolve over live perception and publishes the decision.

    Lifecycle（ADR-0020 §5）：引擎与图上接口在 `on_configure` 打开、`on_cleanup` 释放，
    `deactivate -> activate` 只重开 tick 循环。TensorRT 权重是几百 MiB 到 ~800 MiB 的
    代价，重启一个域不该重付一次 —— 这就是把加载搬出构造函数的全部理由。
    """

    def __init__(self) -> None:
        super().__init__("brain")

        # -- 参数值：on_configure 才读（launch 的 -p override 到那时已经可见）---
        self._scene = "open_floor"
        self._flags: list[str] = []
        self._frame_id = "stereo_camera"
        self._rate = 10.0
        self._force_s2 = -1
        self._max_state_age = 1.0
        self._verbose = True
        self._max_s2 = 1
        self._episode_ticks = 10000
        self._decider = "stub"
        self._consultant = "stub"
        self._backend = "none"

        # -- 资源：on_configure 打开、on_cleanup 释放 --------------------------
        self._s1: Decider | None = None
        self._s2: Consultant | None = None
        self._s1_device = ""
        self._s1_weights = ""
        self._s1_place_error = ""
        self._s1_weights_mib = 0
        self._out_dir = None
        self._log: Logger | None = None
        self._pub = None
        self._sim_pub = None
        self._sub = None
        self._body: MockBody | None = None

        # -- 运行态：on_activate 起、on_deactivate 停 --------------------------
        self._last_rx = 0.0
        self._last_state: SceneState | None = None
        self._stale_reported = False
        self._tick_count = 0
        self._last_veto = ""
        # `SceneState.t` is scenario time -- `body.py` steps it 0, 0.033, ...
        # and `state_text` opens with `t=12.44`. Perception stamps wall clock,
        # so the first stamp anchors a clock the corpus can actually share
        # between the CLI loop and this node.
        self._started = 0.0
        self._epoch0: float | None = None

        self._pending: dict[int, tuple[SceneState, Decision, str]] = {}
        self._replies: list[tuple[int, Decision | None]] = []
        self._applied: dict[int, bool] = {}
        self._threads: list[threading.Thread] = []
        self._landed: dict[int, Decision | None] = {}  # S2 threads write, tick thread drains
        self._s2_lock = threading.Lock()
        self._s2_in_flight = 0

    # ---------------------------------------------------------------- configure
    def on_configure(self, state) -> TransitionCallbackReturn:
        """参数、引擎、两半大脑与图上接口在这一装配（ADR-0020 §5）。

        引擎加载失败（plan 缺失、反序列化失败、cudaMalloc 失败）在这里变成
        FAULTED 的可读 reason；旧实现里它是构造函数异常 = 进程消失。
        """
        ret = super().on_configure(state)
        if ret != TransitionCallbackReturn.SUCCESS:
            return ret

        # -- parameters -------------------------------------------------
        decider = str(self.declare("decider", "stub"))
        # Empty -> the decider searches data/models/ under the module dir and
        # CWD (see tensorrt_decider._default_model_dir); set it when the assets
        # live somewhere else.
        model_dir = str(self.declare("decider_model_dir", ""))
        consultant = str(self.declare("consultant", "stub"))
        scenario = str(self.declare("scenario", "approach"))
        sim = bool(self.declare("sim", False))
        self._rate = max(0.1, float(self.declare("rate", 10.0)))  # sim + watchdog Hz
        self._force_s2 = int(self.declare("force_s2", -1))  # escalate from this tick on
        self._max_state_age = float(self.declare("max_state_age", 1.0))  # seconds before stale
        # scene/flags are parameters because SceneState.to_text() feeds the
        # frozen episode corpus: perception does not carry them yet, and
        # guessing them at runtime would silently change every state_text.
        self._scene = str(self.declare("scene", "open_floor"))
        self._flags = [f for f in str(self.declare("flags", "")).split(",") if f]
        self._frame_id = str(self.declare("frame_id", "stereo_camera"))  # matches eye
        out = str(self.declare("out", ""))  # "" -> <repo>/data/episodes (ADR-0013)
        self._verbose = bool(self.declare("verbose", True))
        # Out-of-domain System 1 answers rarely clear the gate, so escalation would
        # otherwise fire on every tick while System 2 takes 12-30 s to answer:
        # hundreds of live threads, and answers that were stale on arrival.
        self._max_s2 = max(0, int(self.declare("max_s2_in_flight", 1)))
        # `loop.py` is bounded by --ticks and so gets one file for free. This
        # node never stops, so without a tick budget per file an always-on run
        # produces a single ever-growing file no tool wants to open. 0 = never.
        self._episode_ticks = max(0, int(self.declare("episode_ticks", 10000)))

        # -- 引擎：本节点最贵的资源，只能在这里付一次 ---------------------
        try:
            self._s1 = _make_decider(decider, model_dir)
        except Exception as exc:  # noqa: BLE001 - 加载失败是状态，不是崩溃
            self._s1 = None
            return self.fail(f"decider={decider} 加载失败: {exc}")
        self._decider = decider
        self._backend = _ENGINE_BACKENDS.get(decider, decider)
        # Preserve device/weight provenance in episode metadata across decider backends.
        self._s1_device = str(getattr(self._s1, "device", ""))
        self._s1_weights = str(getattr(self._s1, "weights", ""))
        self._s1_place_error = str(getattr(self._s1, "place_error", ""))
        self._s1_weights_mib = round(int(getattr(self._s1, "weights_bytes", 0) or 0) / 1048576)

        try:
            self._s2 = StubConsultant() if consultant == "stub" else _rai(consultant)
        except Exception as exc:  # noqa: BLE001 - 同上：让编排层读到原因
            self._release_engine()
            return self.fail(f"consultant={consultant} 初始化失败: {exc}")
        self._consultant = consultant

        try:
            self._out_dir = _episode_dir(out)
            self._log = self._new_episode()
        except OSError as exc:  # 语料目录不可写：配置失败，而不是跑到一半才发现
            self._release_engine()
            return self.fail(f"episode corpus 不可写 ({self._out_dir}): {exc}")

        try:
            self._body = MockBody(scenario) if sim else None
        except Exception as exc:  # noqa: BLE001 - scenario 名非法
            self._release_engine()
            return self.fail(f"scenario={scenario} 无法模拟: {exc}")

        # -- wire -------------------------------------------------------
        # Perception is keep-last-1 best-effort: only the newest frame matters
        # and a backlog would be worse than a gap. Decision is reliable -- the
        # consumer is an actuator.
        perception_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT
        )
        self._pub = self.lifecycle_publisher(
            DecisionMsg, DECISION_TOPIC, QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10)
        )
        self._sim_pub = (
            self.lifecycle_publisher(TrackedObjects, PERCEPTION_TOPIC, perception_qos)
            if self._body is not None
            else None
        )
        self._sub = self.create_subscription(
            TrackedObjects, PERCEPTION_TOPIC, self._on_perception, perception_qos
        )

        # 新配置 = 新一集语料：tick 计数与场景时钟都从头锚定。
        self._tick_count = 0
        self._last_veto = ""
        self._last_state = None
        self._stale_reported = False
        self._epoch0 = None
        self._pending.clear()
        self._replies.clear()
        self._applied.clear()
        self._landed.clear()
        self._threads[:] = [t for t in self._threads if t.is_alive()]

        self.report(
            "engine",
            DiagnosticStatus.OK,
            f"{self._backend} 已加载 {self._s1_weights_mib} MiB",
            {
                "backend": self._backend,
                "decider": self._decider,
                "device": self._s1_device or "-",
                "weights_mib": self._s1_weights_mib,
            },
        )
        self.get_logger().info(
            f"brain configured: decider={decider} backend={self._backend} "
            f"s1_device={self._s1_device or '-'} sim={self._body is not None}"
        )
        # An installed node's default corpus is not beside its source, so say
        # where it goes rather than making someone discover that after a run.
        self.get_logger().info(f"corpus: {self._log.path.parent}")
        if self._s1_device == "cpu":
            self.get_logger().warn(
                "System 1 reports CPU execution; verify that this backend meets "
                "the sensor-rate and latency requirements."
                + (f" Placement said: {self._s1_place_error[:200]}" if self._s1_place_error else "")
            )
        return TransitionCallbackReturn.SUCCESS

    # ----------------------------------------------------------------- activate
    def on_activate(self, state) -> TransitionCallbackReturn:
        """激活 = 开始 tick。**不重载引擎**：这是整个转换的收益所在。"""
        ret = super().on_activate(state)
        if ret != TransitionCallbackReturn.SUCCESS:
            return ret
        if self._s1 is None or self._log is None:
            # activate 只该从 inactive 来；没有 configure 的激活等于一个静默空转的 tick 环。
            return self.fail("activate 之前没有 configure：引擎与语料都不在")
        # 停用期间可能过了很久，看门狗必须从激活这一刻重新计时。
        self._last_rx = time.monotonic()
        self._started = time.monotonic()
        self._stale_reported = False
        if self._body is not None:
            self.add_timer(1.0 / self._rate, self._sim_step)
        self.add_timer(1.0 / self._rate, self._watchdog)
        self.add_timer(1.0, self._report_health)
        self._report_health()
        self.get_logger().info(
            f"brain up: decider={self._decider} s1_device={self._s1_device or '-'} "
            f"sim={self._body is not None} "
            f"perception={PERCEPTION_TOPIC} decision={DECISION_TOPIC}"
        )
        return TransitionCallbackReturn.SUCCESS

    # --------------------------------------------------------------- deactivate
    def on_deactivate(self, state) -> TransitionCallbackReturn:
        """停 tick 循环（timer 由基类登记与取消）；引擎、语料、图上接口原样保留。"""
        self.cancel_timers()
        return super().on_deactivate(state)

    # ------------------------------------------------------------------ cleanup
    def on_cleanup(self, state) -> TransitionCallbackReturn:
        """释放 TensorRT 引擎（唯一真正昂贵的资源），再交回基类。

        发布器不在这里手动销毁：`on_cleanup` 的受管实体钩子会走这一步，提前 destroy
        会让那次钩子打到失效句柄上，cleanup 转移失败。订阅器不归生命周期管，手动销毁。
        """
        self._release_engine()
        if self._sub is not None:
            self.destroy_subscription(self._sub)
            self._sub = None
        self._pub = None
        self._sim_pub = None
        self._body = None
        self._log = None
        self.report("engine", DiagnosticStatus.WARN, "已释放（unconfigured）", {"backend": "released"})
        return super().on_cleanup(state)

    # ------------------------------------------------------------------- health
    def _report_health(self) -> None:
        age = self.age_ms(DECISION_TOPIC)
        values = {
            "backend": self._backend,
            "decider": self._decider,
            "decisions": self._tick_count,
            "last_decision_age_ms": "never" if age is None else f"{age:.0f}",
            "s1_device": self._s1_device or "-",
            "weights_mib": self._s1_weights_mib,
        }
        level = DiagnosticStatus.OK
        message = f"{self._backend} decisions={self._tick_count}"
        if age is None:
            level = DiagnosticStatus.WARN
            message += ": 还没有决策"
        elif age > 5000.0:
            level = DiagnosticStatus.WARN
            message += f": 上次决策 {age:.0f} ms 前"
        self.report("decision", level, message, values)

    def _release_engine(self) -> None:
        """释放 decider 的引擎（`StubDecider` 没有释放路径，跳过）。"""
        s1, self._s1 = self._s1, None
        close = getattr(s1, "close", None)
        if callable(close):
            close()

    # -- inputs --------------------------------------------------------

    def _sim_step(self) -> None:
        assert self._body is not None and self._sim_pub is not None
        self.publish(
            _state_to_msg(self._body.step(), self.get_clock(), self._frame_id),
            publisher=self._sim_pub,
        )

    def _on_perception(self, msg: TrackedObjects) -> None:
        # 订阅器不随生命周期启停，所以"未激活不干活"必须由这里判（ADR-0020 §5）。
        if not self.active:
            return
        try:
            state = _state_from_msg(msg, self._scene, self._flags)
        except ValueError as exc:
            self.get_logger().error(f"dropping perception: {exc}")
            self._emit_stale("unparsable perception", None)
            return
        if self._epoch0 is None:  # first frame anchors scenario time at 0.0
            self._epoch0 = state.t
        state.t -= self._epoch0
        self._stale_reported = False
        self._tick(state)
        # Anchored when the frame is *consumed*, not when it arrived: System 1
        # runs inside this callback, so a decide() slower than `max_state_age`
        # would otherwise make the watchdog blame the sensor for our own latency.
        self._last_rx = time.monotonic()

    def _watchdog(self) -> None:
        """Perception silence is a state of its own, and not a safe one."""
        if self._stale_reported or time.monotonic() - self._last_rx <= self._max_state_age:
            return
        self._emit_stale("perception stale", self._last_state)

    # -- one tick ------------------------------------------------------

    def _tick(self, state: SceneState) -> None:
        self._last_state = state
        self._tick_count += 1

        decision = self._s1.decide(state)
        final, tier, veto = resolve(state, decision, force_s2=0 <= self._force_s2 <= self._tick_count - 1)
        self._publish(final, tier)
        self._record(state, decision, tier, veto, final)
        self._escalate(state, final, tier, veto)
        self._roll_episode()  # after the drain, so one file holds one arc

    def _emit_stale(self, reason: str, last: SceneState | None) -> None:
        """No trustworthy input: hold the last known veto, otherwise look.

        Deliberately not `continue` -- "path clear" is exactly the claim a
        missing sensor can no longer support. `guard()` is called directly
        rather than through `resolve()`, because there is no System 1 decision
        to resolve: the guard is the authority and needs no model to run, while
        the rest of that policy assumes a model output that does not exist here.

        Logged as `kind="stale"` rather than `tick` so it never enters the
        finetune corpus pretending to be a decision the model actually made.
        """
        self._stale_reported = True
        if last is not None:
            state = last
        else:
            # No frame has ever arrived, so there is no sensor clock to anchor
            # to; elapsed wall time starts at 0 the same way `epoch0` would.
            state = SceneState(t=time.monotonic() - self._started, scene=self._scene, flags=self._flags)
        veto = guard(state)
        if veto is not None:
            final = Decision(action=veto, source="guard", t=state.t)
        else:
            final = Decision(action=Action(kind="inspect", reason=reason), source="fallback", t=state.t)
        tier = Tier.LOW  # with no input, every world is a low-confidence world
        self._publish(final, tier)
        self._log.log(
            kind="stale",
            tick=self._tick_count,
            t=state.t,
            state=state.model_dump(),
            state_text=state.to_text(),
            reason=reason,
            veto=veto.model_dump() if veto is not None else None,
            final=final.model_dump(),
            tier=tier.value,
        )
        # Deliberately no `_escalate` here. Escalation exists to resolve a
        # System 1 decision, and there is none on this path -- and escalating
        # under `self._tick_count` would file a second reply against a tick that
        # already has one, so `_drain` would apply and log both.
        self.get_logger().warn(f"{reason}: publishing {final.action.kind} ({tier.value})")

    def _publish(self, final: Decision, tier: Tier) -> None:
        msg = DecisionMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.action = _action_to_msg(final.action)
        msg.source = final.source
        msg.p, msg.margin = float(final.p), float(final.margin)
        msg.latency_ms, msg.t = float(final.latency_ms), float(final.t)
        msg.tier = tier.value
        if self.publish(msg, publisher=self._pub):
            self.mark_seen(DECISION_TOPIC)

    def _record(self, state: SceneState, decision: Decision, tier: Tier, veto: Action | None, final: Decision) -> None:
        """Same episode shape as the CLI loop -- one corpus, two entry points."""
        self._log.log(
            kind="tick",
            tick=self._tick_count,
            t=state.t,
            state=state.model_dump(),
            state_text=state.to_text(),  # verbatim: never regenerate at train time
            s1=decision.model_dump(),
            tier=tier.value,
            veto=veto.model_dump() if veto is not None else None,
            final=final.model_dump(),
            label=None,
        )
        # A veto that holds across ticks is one fact, not ten per second: log
        # the transition (both edges) and otherwise rely on the cadence.
        kind = veto.kind if veto is not None else ""
        changed, self._last_veto = kind != self._last_veto, kind
        if self._verbose and (self._tick_count % 20 == 1 or changed):
            tag = "VETO" if veto else final.source
            self.get_logger().info(f"[{self._tick_count:4d}] {state.to_text()[:60]} -> {final.action.kind} ({tag})")

    def _escalate(self, state: SceneState, final: Decision, tier: Tier, veto: Action | None) -> None:
        """Fire-and-forget, applied on a later tick -- never on this one."""
        # Land whatever the slow path finished, then drop what has served its
        # purpose. Both happen here, on the tick thread, because everything in
        # `_pending`/`_applied`/`_replies` is single-writer by construction:
        # System 2 threads only ever write to `_landed` under `_s2_lock`.
        with self._s2_lock:
            for k, reply in self._landed.items():
                self._replies.append((k, reply))
            self._landed.clear()
        _drain(self._replies, self._applied, self._pending, state, self._log, self._verbose)
        self._prune()

        if tier is Tier.HIGH:
            return
        with self._s2_lock:
            if self._s2_in_flight >= self._max_s2:
                # The slow path is busy, so skip this tick. Skipping is the
                # point: System 2 takes 12-30 s, an answer landing that late is
                # about a scene that no longer exists, and unbounded spawning
                # is how a node that runs for days becomes a thread leak.
                return
            self._s2_in_flight += 1

        reason = "guard" if veto is not None else f"tier={tier.value}"
        tick = self._tick_count
        self._pending[tick] = (state, final, reason)

        def ask() -> None:
            reply: Decision | None = None
            try:
                reply = self._s2.consult(state, final, reason)
            except Exception as exc:  # a slow path that throws must not kill the node
                self.get_logger().error(f"s2 failed: {exc}")
            finally:
                with self._s2_lock:
                    self._s2_in_flight -= 1
                    self._landed[tick] = reply

        th = threading.Thread(target=ask, daemon=True)
        th.start()
        self._threads.append(th)

    def _prune(self) -> None:
        """Bound the bookkeeping. `loop.py`'s copies are bounded by `--ticks`.

        A reply that has been applied has served its purpose: drop it, the
        scene it was asked about, and the marker that stopped it being applied
        twice. A `None` reply (the slow path failed) is dropped as well --
        `_drain` never marks those applied, so left alone they accumulate.
        """
        done = {k for k, reply in self._replies if reply is None or k in self._applied}
        self._replies[:] = [r for r in self._replies if r[1] is not None and r[0] not in self._applied]
        for k in done:
            self._pending.pop(k, None)
            self._applied.pop(k, None)
        self._threads[:] = [t for t in self._threads if t.is_alive()]

    # -- episode bookkeeping ------------------------------------------

    def _new_episode(self) -> Logger:
        log = Logger(out_dir=self._out_dir)
        log.log(
            kind="meta",
            decider=self._decider,
            consultant=self._consultant,
            node="ros",
            rate=self._rate,
            s1_device=self._s1_device,  # a CPU row was produced by an 8x slower run
            s1_weights=self._s1_weights or None,  # fp16 rows are not fp32 comparable
        )
        return log

    def _roll_episode(self) -> None:
        """Close the file and start another once the tick budget is spent.

        A late System 2 reply can still land in the next file -- it is asked
        about between ticks by definition. It carries its own `tick`, so the
        record stays unambiguous even across the boundary.
        """
        if self._episode_ticks <= 0 or self._tick_count % self._episode_ticks != 0:
            return
        self._log.log(kind="end", ticks=self._tick_count)
        self._log = self._new_episode()
        self.get_logger().info(f"episode rotated at tick {self._tick_count}: {self._log.path}")

    # -- shutdown ------------------------------------------------------

    def shutdown(self) -> None:
        """Flush in-flight System 2 replies, then close the episode."""
        for th in self._threads:
            th.join(timeout=30)
        with self._s2_lock:
            for k, reply in self._landed.items():
                self._replies.append((k, reply))
            self._landed.clear()
        if self._log is None:  # 从未 configure：没有语料文件可收尾
            return
        if self._replies and self._pending:
            _drain(self._replies, self._applied, self._pending, self._last_state, self._log, self._verbose)
        self._log.log(kind="end", ticks=self._tick_count)
        self.get_logger().info(f"episode: {self._log.path}")


def _rai(consultant: str) -> Consultant:
    from .system2 import RaiConsultant  # noqa: PLC0415 -- optional dep, import late

    return RaiConsultant(vendor=None if consultant == "rai" else consultant)


def _episode_dir(out: str):
    """Where the corpus is appended. `out` wins; "" means "the one corpus".

    The package-relative default is the source tree for local tests.
    `ros2 run rak_agent agent_node` runs the *installed*
    copy, where `__file__` sits under `<ws>/install/`, a disposable tree that
    `rm -rf install` would take the corpus with. From there walk back out to
    the workspace and use `<ws>/data/episodes` (ADR-0013): the same directory
    the CLI appends to, so ROS runs and CLI runs build one corpus instead of
    two.
    """
    from pathlib import Path  # noqa: PLC0415

    if out:
        return Path(out)
    pkg = Path(__file__).resolve().parent.parent  # .../site-packages
    if (pkg / "package.xml").exists():  # source tree: pkg = <repo>/src/rak_agent
        return pkg.parent.parent / "data" / "episodes"
    # <ws>/install/<pkg>/lib/pythonX.Y/site-packages
    for parent in pkg.parents:
        if parent.name == "install" and (parent.parent / "src" / "rak_agent" / "package.xml").exists():
            return parent.parent / "data" / "episodes"
    return pkg / "data" / "episodes"  # unknown layout: keep it beside the package


def main(args=None) -> int:
    rclpy.init(args=args)
    node = None
    try:
        node = BrainNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # SIGINT/SIGTERM: the finally below still flushes the episode
    except RuntimeError:
        # A callback can be dispatched while the signal handler is tearing the
        # context down, so its publish() fails with RCLError instead of spin()
        # raising -- RCLError is a RuntimeError, and `rclpy.error` is gone as an
        # import in Jazzy. Suppress only that race: a healthy context means this
        # is a real failure and has to surface.
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""ROS 2 edge: the brain's tick loop as a node.

Ticks are event-driven -- a perception message drives one, so `guard()` runs on
every frame the sensor actually produced instead of on a wall clock that can
drift away from the data. A watchdog covers the other case: perception stopping
entirely.

What crosses the wire is perception IN and decisions OUT, which are the two
edges the design allows. System 1 does not cross: Laya is constructed and
called in this process, exactly as in `python -m rak_ai.loop`. Putting it
behind a topic or a service would put a cross-process boundary on the fast
path, which the architecture forbids.

ROS must be sourced before this module is imported -- `rclpy` comes from
/opt/ros/jazzy and `core.msg` from the colcon install space.

    ros2 run brain brain_node --ros-args -p decider:=laya -p sim:=true
"""

from __future__ import annotations

import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from core.msg import Action as ActionMsg
from core.msg import Decision as DecisionMsg
from core.msg import TrackedObject
from core.msg import TrackedObjects

from .body import MockBody
from .loop import Logger, _drain  # noqa: PLC2701 -- same package, one tick pipeline
from .system1 import Decider, LayaDecider, StubDecider, guard, resolve
from .system2 import Consultant, StubConsultant
from .types import Action, Decision, Entity, SceneState, Tier

PERCEPTION_TOPIC = "/perception/tracked_objects"
DECISION_TOPIC = "/brain/decision"


def _action_to_msg(a: Action) -> ActionMsg:
    """types.Action -> core/msg/Action. The pydantic model stays authoritative."""
    m = ActionMsg()
    m.kind = a.kind
    m.has_target = a.target_id is not None
    m.target_id = a.target_id if a.target_id is not None else 0
    m.reason = a.reason
    return m


def _state_from_msg(msg: TrackedObjects, scene: str, flags: list[str]) -> SceneState:
    """core/msg/TrackedObjects -> SceneState.

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
    """SceneState -> core/msg/TrackedObjects. Used by the built-in simulator."""
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


class BrainNode(Node):
    """Runs guard/gate/resolve over live perception and publishes the decision."""

    def __init__(self) -> None:
        super().__init__("brain")

        # -- parameters -------------------------------------------------
        self.declare_parameter("decider", "stub")
        self.declare_parameter("consultant", "stub")
        self.declare_parameter("scenario", "approach")
        self.declare_parameter("sim", False)
        self.declare_parameter("rate", 10.0)  # sim + watchdog Hz
        self.declare_parameter("force_s2", -1)  # escalate from this tick on
        self.declare_parameter("max_state_age", 1.0)  # seconds before stale
        # scene/flags are parameters because SceneState.to_text() feeds the
        # frozen episode corpus: perception does not carry them yet, and
        # guessing them at runtime would silently change every state_text.
        self.declare_parameter("scene", "open_floor")
        self.declare_parameter("flags", "")
        self.declare_parameter("frame_id", "stereo_camera")  # matches eye
        self.declare_parameter("out", "")  # "" -> brain/data/episodes
        self.declare_parameter("verbose", True)
        # Out-of-domain Laya rarely clears the gate, so escalation would
        # otherwise fire on every tick while System 2 takes 12-30 s to answer:
        # hundreds of live threads, and answers that were stale on arrival.
        self.declare_parameter("max_s2_in_flight", 1)
        # `loop.py` is bounded by --ticks and so gets one file for free. This
        # node never stops, so without a tick budget per file an always-on run
        # produces a single ever-growing file no tool wants to open. 0 = never.
        self.declare_parameter("episode_ticks", 10000)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self._scene = str(g("scene"))
        self._flags = [f for f in str(g("flags")).split(",") if f]
        self._frame_id = str(g("frame_id"))
        self._rate = max(0.1, float(g("rate")))
        self._force_s2 = int(g("force_s2"))
        self._max_state_age = float(g("max_state_age"))
        self._verbose = bool(g("verbose"))
        self._max_s2 = max(0, int(g("max_s2_in_flight")))
        self._episode_ticks = max(0, int(g("episode_ticks")))

        # -- the two halves of the brain, both in this process ----------
        decider = str(g("decider"))
        self._s1: Decider = StubDecider() if decider == "stub" else LayaDecider()
        # "cuda"/"cpu": laya swallows CUDA OOM and degrades to CPU, which on this
        # box is 8x slower -- slower than the sensor rate, so it must not be quiet.
        self._s1_device = str(getattr(self._s1, "device", ""))
        consultant = str(g("consultant"))
        self._s2: Consultant = StubConsultant() if consultant == "stub" else _rai(consultant)

        self._decider, self._consultant = decider, consultant
        self._out_dir = _episode_dir(str(g("out")))
        self._log = self._new_episode()

        # -- wire -------------------------------------------------------
        # Perception is keep-last-1 best-effort: only the newest frame matters
        # and a backlog would be worse than a gap. Decision is reliable -- the
        # consumer is an actuator.
        self._pub = self.create_publisher(
            DecisionMsg, DECISION_TOPIC, QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10)
        )
        self._sim_pub = (
            self.create_publisher(
                TrackedObjects,
                PERCEPTION_TOPIC,
                QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
            )
            if bool(g("sim"))
            else None
        )
        self._sub = self.create_subscription(
            TrackedObjects,
            PERCEPTION_TOPIC,
            self._on_perception,
            QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )

        self._body = MockBody(str(g("scenario"))) if self._sim_pub is not None else None
        self._last_rx = time.monotonic()
        self._last_state: SceneState | None = None
        self._stale_reported = False
        self._tick_count = 0
        self._last_veto = ""
        # `SceneState.t` is scenario time -- `body.py` steps it 0, 0.033, ...
        # and `state_text` opens with `t=12.44`. Perception stamps wall clock,
        # so the first stamp anchors a clock the corpus can actually share
        # between the CLI loop and this node.
        self._started = time.monotonic()
        self._epoch0: float | None = None

        self._pending: dict[int, tuple[SceneState, Decision, str]] = {}
        self._replies: list[tuple[int, Decision | None]] = []
        self._applied: dict[int, bool] = {}
        self._threads: list[threading.Thread] = []
        self._landed: dict[int, Decision | None] = {}  # S2 threads write, tick thread drains
        self._s2_lock = threading.Lock()
        self._s2_in_flight = 0

        if self._body is not None:
            self.create_timer(1.0 / self._rate, self._sim_step)
        self.create_timer(1.0 / self._rate, self._watchdog)
        self.get_logger().info(
            f"brain up: decider={decider} s1_device={self._s1_device or '-'} "
            f"sim={self._body is not None} "
            f"perception={PERCEPTION_TOPIC} decision={DECISION_TOPIC}"
        )
        # An installed node's default corpus is not beside its source, so say
        # where it goes rather than making someone discover that after a run.
        self.get_logger().info(f"corpus: {self._log.path.parent}")
        if self._s1_device == "cpu":
            self.get_logger().warn(
                "System 1 fell back to CPU (~1.6 s/predict, vs ~0.19 s on GPU): "
                "slower than the sensor rate, so frames are dropped. This box has "
                "3.4 GB of unified memory and laya needs ~1.9 GB of it -- free "
                "some, or run with -p decider:=stub."
            )

    # -- inputs --------------------------------------------------------

    def _sim_step(self) -> None:
        assert self._body is not None and self._sim_pub is not None
        self._sim_pub.publish(_state_to_msg(self._body.step(), self.get_clock(), self._frame_id))

    def _on_perception(self, msg: TrackedObjects) -> None:
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
        self._pub.publish(msg)

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
        if self._replies and self._pending:
            _drain(self._replies, self._applied, self._pending, self._last_state, self._log, self._verbose)
        self._log.log(kind="end", ticks=self._tick_count)
        self.get_logger().info(f"episode: {self._log.path}")


def _rai(consultant: str) -> Consultant:
    from .system2 import RaiConsultant  # noqa: PLC0415 -- optional dep, import late

    return RaiConsultant(vendor=None if consultant == "rai" else consultant)


def _episode_dir(out: str):
    """Where the corpus is appended. `out` wins; "" means "the one corpus".

    The package-relative default is the source tree when the brain runs
    editable -- the CLI loop. `ros2 run brain brain_node` runs the *installed*
    copy, where `__file__` sits under `<ws>/install/`, a disposable tree that
    `rm -rf install` would take the corpus with. From there walk back out to
    the workspace and use `brain/data/episodes`: the same directory the CLI
    appends to, so ROS runs and CLI runs build one corpus instead of two.
    """
    from pathlib import Path  # noqa: PLC0415

    if out:
        return Path(out)
    pkg = Path(__file__).resolve().parent.parent  # .../site-packages
    if (pkg / "pyproject.toml").exists():  # source or editable install
        return pkg / "data" / "episodes"
    # <ws>/install/<pkg>/lib/pythonX.Y/site-packages
    for parent in pkg.parents:
        if parent.name == "install" and (parent.parent / "brain" / "pyproject.toml").exists():
            return parent.parent / "brain" / "data" / "episodes"
    return pkg / "data" / "episodes"  # unknown layout: keep it beside the package


def main(args=None) -> int:
    rclpy.init(args=args)
    node = BrainNode()
    try:
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
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

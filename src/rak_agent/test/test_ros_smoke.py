"""Exercise the migrated ROS edge with simulated observations, never hardware."""

import os
import tempfile
import time

# Keep the simulator off the operator's ROS domain.
os.environ["ROS_DOMAIN_ID"] = "187"
os.environ["ROS_LOCALHOST_ONLY"] = "1"

import rclpy
from rak_interfaces.msg import Decision

from rak_agent.node import BrainNode, DECISION_TOPIC, _make_decider
from rak_agent.tensorrt_decider import DEFAULT_MODEL_DIR


def main():
    assert type(_make_decider("stub")).__name__ == "StubDecider"
    # The legacy PyTorch backend stays rejected, whatever else is true.
    try:
        _make_decider("laya")
    except RuntimeError as exc:
        assert "disabled" in str(exc)
    else:
        raise AssertionError("legacy laya backend was accepted")

    # The TensorRT backend is integrated: it must construct when the engine is
    # present and skip (not fail) when this checkout has not built it.
    if (DEFAULT_MODEL_DIR / "laya-typed-decisions-bf16.plan").is_file():
        s1 = _make_decider("tensorrt")
        assert type(s1).__name__ == "TensorRtDecider"
        s1.close()
    else:
        print("  skip: TensorRT engine not built")

    with tempfile.TemporaryDirectory(prefix="rak-agent-smoke-") as output:
        rclpy.init(args=[
            "--ros-args", "-p", "sim:=true", "-p", "decider:=stub",
            "-p", "consultant:=stub", "-p", "verbose:=false",
            "-p", f"out:={output}",
        ])
        node = BrainNode()
        replies = []
        node.create_subscription(Decision, DECISION_TOPIC, replies.append, 10)
        try:
            deadline = time.monotonic() + 10
            while len(replies) < 3 and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            assert len(replies) >= 3, "simulated perception did not produce decisions"
            assert all(reply.action.kind for reply in replies)
        finally:
            node.shutdown()
            node.destroy_node()
            rclpy.shutdown()
    print("OK: ROS simulated perception -> migrated interfaces -> agent decisions")


if __name__ == "__main__":
    main()

"""Exercise the migrated ROS edge with simulated observations, never hardware."""

import os
import tempfile
import threading
import time

# Keep the simulator off the operator's ROS domain.
os.environ["ROS_DOMAIN_ID"] = "187"
os.environ["ROS_LOCALHOST_ONLY"] = "1"

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rak_interfaces.msg import Decision
from rak_lifecycle.testing import bring_up, new_driver, transition

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
        driver = new_driver()
        replies = []
        node.create_subscription(Decision, DECISION_TOPIC, replies.append, 10)
        # 转移只能由另一个节点经 change_state 发起（ADR-0020 机制层），所以两个节点
        # 必须同时在拍：目标节点在后台 executor 上，驱动方在本线程里同步调用。
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.add_node(driver)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        try:
            assert bring_up(driver, "/brain"), "managed node did not reach active"
            engine = node._s1
            assert engine is not None, "configure did not load System 1"
            deadline = time.monotonic() + 10
            while len(replies) < 3 and time.monotonic() < deadline:
                time.sleep(0.05)
            assert len(replies) >= 3, "simulated perception did not produce decisions"
            assert all(reply.action.kind for reply in replies)

            # ADR-0020 §5：deactivate 释放"运行"，不释放引擎 —— 原地恢复不重付加载。
            assert transition(driver, "/brain", "deactivate"), "deactivate failed"
            assert node._s1 is engine, "deactivate released the engine; activate would reload it"
            assert transition(driver, "/brain", "activate"), "re-activate failed"
            assert node._s1 is engine, "activate reloaded the engine"
            assert transition(driver, "/brain", "deactivate"), "deactivate failed"
            assert transition(driver, "/brain", "cleanup"), "cleanup failed"
            assert node._s1 is None, "cleanup did not release the engine"
        finally:
            executor.shutdown()
            node.shutdown()
            node.destroy_node()
            driver.destroy_node()
            rclpy.shutdown()
    print("OK: ROS simulated perception -> migrated interfaces -> agent decisions")


if __name__ == "__main__":
    main()

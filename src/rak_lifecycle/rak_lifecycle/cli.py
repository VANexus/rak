"""操作员入口 `rak_system`：看整机状态、按域启停、要求重试（ADR-0020）。

存在的理由：编排收敛到 manager 之后，`ros2 lifecycle set` 逐个节点点已经不该是人操作的方式
（点错顺序会造出半死的域）。这里把"期望状态"表达成 manager 的参数，把"现在怎么样"
表达成 `/rak/system_state` —— 两个都是标准机制，没有自定义服务。
"""

from __future__ import annotations

import argparse
import sys
import time

import diagnostic_msgs.msg
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

MANAGER = "/rak_system_manager"
STATE_TOPIC = "/rak/system_state"

_LEVEL_NAME = {0: "OK", 1: "WARN", 2: "ERROR", 3: "STOP"}


class _Reader(Node):
    def __init__(self) -> None:
        super().__init__("rak_system_cli")
        self.latest: diagnostic_msgs.msg.DiagnosticArray | None = None
        self.create_subscription(
            diagnostic_msgs.msg.DiagnosticArray, STATE_TOPIC, self._on_state, 10
        )

    def _on_state(self, message: diagnostic_msgs.msg.DiagnosticArray) -> None:
        self.latest = message


def _read_state(timeout_s: float) -> diagnostic_msgs.msg.DiagnosticArray | None:
    node = _Reader()
    deadline = time.monotonic() + timeout_s
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.latest is not None:
                return node.latest
    finally:
        node.destroy_node()
    return None


def _values(status: diagnostic_msgs.msg.DiagnosticStatus) -> dict[str, str]:
    return {pair.key: pair.value for pair in status.values}


def command_status(argv: list[str]) -> int:
    rclpy.init()
    try:
        array = _read_state(2.5)
        if array is None:
            print(
                f"没拿到 {STATE_TOPIC} —— manager 在跑吗？"
                "（ros2 launch rak_bringup supervisor.launch.py）",
                file=sys.stderr,
            )
            return 2
        worst = "OK"
        print(f"{'DOMAIN':<13} {'STATE':<12} MESSAGE")
        for status in sorted(array.statuses, key=lambda s: s.name):
            domain = status.name.rsplit("/", 1)[-1]
            state = _values(status).get("state", "?")
            print(f"{domain:<13} {state:<12} {status.message}")
            for key, value in sorted(_values(status).items()):
                if key.startswith("/") or key.startswith("age_ms"):
                    print(f"{'':<13} · {key} = {value}")
            if state in ("faulted",):
                worst = "FAULTED"
            elif state in ("degraded", "bringing_up", "idle") and worst == "OK":
                worst = "DEGRADED"
        print(f"\noverall: {worst}")
        return 0 if worst == "OK" else 1
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def _set_wanted_domains(keep: set[str]) -> int:
    """把"该活的域"整体写成 manager 的 `enabled_domains`。"""
    from rcl_interfaces.srv import SetParameters

    node = Node("rak_system_cli")
    client = node.create_client(SetParameters, f"{MANAGER}/set_parameters")
    if not client.wait_for_service(timeout_sec=5.0):
        print(f"{MANAGER} 不在（manager 没起？）", file=sys.stderr)
        node.destroy_node()
        return 2
    request = SetParameters.Request()
    request.values.append(
        Parameter("enabled_domains", value=str(",".join(sorted(keep)))).to_parameter_msg()
    )
    response = client.call(request)
    node.destroy_node()
    if response is None or not response.results or not response.results[0].successful:
        reason = (
            response.results[0].reason
            if response and response.results
            else "服务无回应"
        )
        print(f"设置失败：{reason}", file=sys.stderr)
        return 1
    return 0


def _current_wanted() -> list[str] | None:
    from rcl_interfaces.srv import GetParameters

    node = Node("rak_system_cli")
    client = node.create_client(GetParameters, f"{MANAGER}/get_parameters")
    if not client.wait_for_service(timeout_sec=5.0):
        node.destroy_node()
        return None
    response = client.call(GetParameters.Request(names=["enabled_domains", "autostart"]))
    node.destroy_node()
    if response is None or not response.values:
        return None
    raw = response.values[0].string_value
    return [name for name in raw.split(",") if name.strip()] or None


def command_domains(argv: list[str]) -> int:
    rclpy.init()
    try:
        wanted = _current_wanted()
        print("desired domains:", ",".join(sorted(wanted)) if wanted else "<全部>")
        return command_status([])
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def command_stop(argv: list[str]) -> int:
    rclpy.init()
    try:
        all_domains = _domain_names()
        current = _current_wanted()
        keep = set(current if current is not None else all_domains)
        keep.difference_update(argv.domain)
        return _set_wanted_domains(keep)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def command_start(argv: list[str]) -> int:
    rclpy.init()
    try:
        all_domains = _domain_names()
        current = _current_wanted()
        keep = set(current if current is not None else all_domains)
        unknown = sorted(set(argv.domain) - set(all_domains))
        if unknown:
            print(f"未知域名: {unknown}", file=sys.stderr)
            return 1
        keep.update(argv.domain)
        return _set_wanted_domains(keep)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def command_recover(argv: list[str]) -> int:  # noqa: ARG001
    from rcl_interfaces.srv import SetParameters

    rclpy.init()
    node = Node("rak_system_cli")
    try:
        client = node.create_client(SetParameters, f"{MANAGER}/set_parameters")
        if not client.wait_for_service(timeout_sec=5.0):
            print(f"{MANAGER} 不在", file=sys.stderr)
            return 2
        # 读当前值再 +1：避免两次 recover 撞同一个值而被当成"没变化"。
        from rcl_interfaces.srv import GetParameters

        getter = node.create_client(GetParameters, f"{MANAGER}/get_parameters")
        getter.wait_for_service(timeout_sec=5.0)
        current = getter.call(GetParameters.Request(names=["retry_epoch"]))
        epoch = int(current.values[0].integer_value) + 1 if current and current.values else 1
        request = SetParameters.Request()
        request.values.append(
            Parameter("retry_epoch", value=epoch).to_parameter_msg()
        )
        response = client.call(request)
        if response is None or not response.results or not response.results[0].successful:
            print("recover 被拒绝", file=sys.stderr)
            return 1
        print(f"retry_epoch -> {epoch}（FAULTED 域会重新走一遍 bring-up）")
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _domain_names() -> list[str]:
    from .manifest import load

    return list(load(_manifest_path()).order)


def _manifest_path() -> str:
    from pathlib import Path

    from ament_index_python.packages import get_package_share_directory

    return str(Path(get_package_share_directory("rak_bringup")) / "config/system_manifest.yaml")


def main() -> int:
    parser = argparse.ArgumentParser(prog="rak_system", description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    status = sub.add_parser("status", help="打印每个故障域的状态与判据")
    status.set_defaults(func=lambda _argv: command_status([]))

    domains = sub.add_parser("domains", help="列出期望启用的域 + 当前状态")
    domains.set_defaults(func=command_domains)

    stop = sub.add_parser("stop", help="停用某些域（连带降级其下游）")
    stop.add_argument("domain", nargs="+")
    stop.set_defaults(func=command_stop)

    start = sub.add_parser("start", help="启用某些域")
    start.add_argument("domain", nargs="+")
    start.set_defaults(func=command_start)

    recover = sub.add_parser("recover", help="要求 manager 重试所有 FAULTED 域")
    recover.set_defaults(func=command_recover)

    argv = parser.parse_args()
    return argv.func(argv)


if __name__ == "__main__":
    raise SystemExit(main())

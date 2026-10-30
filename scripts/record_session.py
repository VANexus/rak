#!/usr/bin/env python3
"""按白名单录制 rosbag（配置在 `src/rak_metrics/config/record_profile.yaml`）。

为什么是脚本而不是一行 `ros2 bag record -a`：

* `-a` 会把 1080p 图像与点云一起录，几分钟写满 eMMC，回放时还分不清哪条是噪声；
* 传感流是 best_effort，rosbag 默认 reliable 订阅**录不到帧**，必须给 QoS 覆盖；
* "大小上限"要能真的停，不是靠人盯着 `du`。

用法::

    python3 scripts/record_session.py --profile logic --duration 300
    python3 scripts/record_session.py --profile perception --out bags/grasp-01
    python3 scripts/record_session.py --profile bench --dry-run

回放：`ros2 bag play bags/<name> --clock`（配合 `use_sim_time:=true`）。
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT / "src" / "rak_metrics" / "config" / "record_profile.yaml"

#: rosbag2 的 QoS 覆盖文件只认这几个键；多写会被忽略或报错。
_QOS_KEYS = {"reliability", "durability", "history", "depth"}


def _config_path() -> Path:
    override = os.environ.get("RAK_RECORD_PROFILE", "")
    if override:
        return Path(override).expanduser()
    try:
        from ament_index_python.packages import get_package_share_directory

        installed = Path(get_package_share_directory("rak_metrics")) / "config" / "record_profile.yaml"
        if installed.is_file():
            return installed
    except Exception:  # noqa: BLE001 - 未 source 环境时退源码树
        pass
    return DEFAULT_CONFIG


def load_profile(name: str) -> tuple[dict, dict, dict]:
    import yaml

    doc = yaml.safe_load(_config_path().read_text(encoding="utf-8"))
    profiles = doc.get("profiles", {})
    if name not in profiles:
        raise SystemExit(f"未知录制档位 {name!r}；可用: {sorted(profiles)}")
    spec = dict(profiles[name])
    parent = spec.get("extends")
    topics: list[str] = []
    if parent:
        if parent not in profiles:
            raise SystemExit(f"{name} extends 了不存在的档位 {parent!r}")
        topics.extend(profiles[parent].get("topics", []))
    topics.extend(spec.get("topics", []))
    # 去重但保序：同一个话题在两个档位里都写不算错。
    seen: set[str] = set()
    ordered = [t for t in topics if not (t in seen or seen.add(t))]
    spec["topics"] = ordered
    return spec, doc.get("qos_overrides", {}), doc.get("defaults", {})


def write_qos_overrides(overrides: dict, topics: list[str]) -> str | None:
    """生成 rosbag2 的 QoS 覆盖文件；没有需要覆盖的话题时返回 None。"""
    if not overrides:
        return None
    payload = {}
    for topic in topics:
        entry = overrides.get(topic)
        if not entry:
            continue
        payload[topic] = {k: v for k, v in entry.items() if k in _QOS_KEYS}
    if not payload:
        return None
    handle = tempfile.NamedTemporaryFile(
        "w", suffix=".yaml", prefix="rak-record-qos-", delete=False, encoding="utf-8")
    import yaml

    yaml.safe_dump(payload, handle, allow_unicode=True, sort_keys=True)
    handle.close()
    return handle.name


def _dir_size_mb(path: Path) -> float:
    total = 0
    for root, _dirs, files in os.walk(path):
        for name in files:
            try:
                total += (Path(root) / name).stat().st_size
            except OSError:
                pass
    return total / 1e6


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--profile", default="logic", help="录制档位（见 record_profile.yaml）")
    parser.add_argument("--duration", type=float, default=0.0, help="录制时长 [s]；0 = 一直录到 Ctrl-C")
    parser.add_argument("--out", default="", help="输出目录；留空按档位与时间戳生成")
    parser.add_argument("--dry-run", action="store_true", help="只打印命令行")
    args = parser.parse_args()

    spec, overrides, defaults = load_profile(args.profile)
    topics = spec["topics"]
    if not topics:
        raise SystemExit(f"档位 {args.profile} 没有任何话题")

    stamp = time.strftime("%Y%m%d-%H%M%S")
    output = Path(args.out or Path(defaults.get("output_dir", "bags")) / f"{args.profile}-{stamp}")

    command = [
        "ros2", "bag", "record",
        "-o", str(output),
        "--storage", str(defaults.get("storage", "mcap")),
        "--max-bag-size", str(int(spec.get("split_mb", 512)) * 1024 * 1024),
    ]
    qos_file = write_qos_overrides(overrides, topics)
    if qos_file:
        command += ["--qos-profile-overrides-path", qos_file]
    command += topics

    print(f"[{args.profile}] {spec.get('description', '')}")
    print(f"[{args.profile}] {len(topics)} 个话题，单卷 {spec.get('split_mb', 512)} MB，"
          f"总量上限 {spec.get('max_size_mb', 0)} MB")
    print("+ " + " ".join(command))
    if args.dry_run:
        return 0

    output.parent.mkdir(parents=True, exist_ok=True)
    process = subprocess.Popen(command)

    limit_mb = float(spec.get("max_size_mb", 0) or 0)
    deadline = time.monotonic() + args.duration if args.duration > 0 else None
    stop_reason = ""
    try:
        while process.poll() is None:
            time.sleep(1.0)
            if deadline is not None and time.monotonic() >= deadline:
                stop_reason = "时长到"
                break
            if limit_mb > 0 and _dir_size_mb(output) > limit_mb:
                stop_reason = f"超过 {limit_mb:.0f} MB 上限"
                break
    except KeyboardInterrupt:
        stop_reason = "操作员 Ctrl-C"
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                process.kill()
    if qos_file:
        try:
            os.unlink(qos_file)
        except OSError:
            pass
    print(f"录制结束（{stop_reason or '进程自行退出'}）：{output}")
    return process.returncode or 0


if __name__ == "__main__":
    sys.exit(main())

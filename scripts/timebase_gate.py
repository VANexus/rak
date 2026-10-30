#!/usr/bin/env python3
"""列出所有"还没实测"的时间基准相关数值（ADR-0023 §4）。

存在理由：`TODO(measured)` 这类占位值一旦能过构建，就会永久留在配置里，
然后某天被人当成事实引用。本脚本把它变成**可查询的清单**：

    python3 scripts/timebase_gate.py             # 报告（总是 exit 0）
    python3 scripts/timebase_gate.py --strict    # 发布档门禁：有占位值即 exit 1

同时钉住两件容易坏的事：

1. `ekf.yaml` 的每个 `*_covariance` 必须是 225 个数字（15×15），
   且不能出现 PyYAML 会读成字符串的科学计数法（`1e6` 不带小数点会被读成 str）；
2. `canopen_contract.yaml` 的 `byte_order` 若仍标着 `TODO`，说明首帧还没核对
   （对应 `scripts/verify_canopen_frames.py`）。
"""

from __future__ import annotations

import argparse
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[1]
MARKER = re.compile(r"TODO\((measured|实测)\)|TODO:\s*实测|占位")
NUMERIC = re.compile(r"(?<![\w.])[-+]?\d+(?:\.\d+)?[eE][-+]?\d+")


def _yaml_files() -> list[Path]:
    roots = [ROOT / "src" / "rak_bringup" / "config", ROOT / "src" / "rak_canopen" / "config",
             ROOT / "src" / "rak_orchestration" / "config", ROOT / "src" / "rak_metrics" / "config"]
    files: list[Path] = []
    for root in roots:
        if root.is_dir():
            files.extend(sorted(root.glob("*.yaml")))
    return files


def scan_markers() -> list[tuple[str, int, str]]:
    hits: list[tuple[str, int, str]] = []
    for path in _yaml_files():
        for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
            if MARKER.search(line):
                hits.append((str(path.relative_to(ROOT)), number, line.strip()[:110]))
    return hits


def check_ekf() -> list[str]:
    try:
        import yaml
    except ImportError:
        return ["PyYAML 不可用，跳过 ekf 检查"]
    path = ROOT / "src" / "rak_bringup" / "config" / "ekf.yaml"
    if not path.is_file():
        return [f"缺少 {path}"]
    params = yaml.safe_load(path.read_text(encoding="utf-8"))["ekf_filter_node"]["ros__parameters"]
    problems = []
    for key, value in params.items():
        if not key.endswith("_covariance"):
            continue
        if not isinstance(value, list) or len(value) != 225:
            problems.append(f"{key}: 需要 225 个数字（15×15），实得 "
                            f"{len(value) if isinstance(value, list) else type(value).__name__}")
            continue
        bad = [i for i, item in enumerate(value) if not isinstance(item, (int, float)) or isinstance(item, bool)]
        if bad:
            problems.append(f"{key}: 第 {bad[:3]} 项不是数字（科学计数法会被读成字符串）")
    for key in ("odom0", "imu0"):
        if f"{key}_config" not in params:
            problems.append(f"{key}_config 缺失")
    return problems


def check_numeric_notation() -> list[str]:
    problems = []
    for path in _yaml_files():
        for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
            if line.lstrip().startswith("#"):
                continue
            if NUMERIC.search(line):
                problems.append(f"{path.relative_to(ROOT)}:{number}: 科学计数法 {NUMERIC.search(line).group(0)}"
                                "（PyYAML 1.1 会读成字符串）")
    return problems


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--strict", action="store_true", help="有未实测项就失败（发布档门禁）")
    args = parser.parse_args()

    markers = scan_markers()
    ekf_problems = check_ekf()
    notation = check_numeric_notation()

    print(f"未实测/占位标记: {len(markers)} 处")
    for rel, line, text in markers:
        print(f"  {rel}:{line}  {text}")
    if ekf_problems:
        print("\nEKF 配置问题:")
        for item in ekf_problems:
            print("  -", item)
    if notation:
        print("\n数值写法问题:")
        for item in notation:
            print("  -", item)

    if args.strict and (markers or ekf_problems or notation):
        print("\nFAIL: 发布档不允许未实测的占位值（ADR-0023 §4）", file=sys.stderr)
        return 1
    print("\nOK（报告模式：占位值已列出，不阻断构建）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

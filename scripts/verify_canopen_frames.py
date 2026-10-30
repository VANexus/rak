#!/usr/bin/env python3
"""核对 CANopen v5 的**首帧**：帧长、COB-ID、字节序、量化后是否落在物理范围。

背景（ADR-0024 / AGENTS §5）：`canopen_contract.yaml` 里的 `byte_order: little` 是
**按 STM32 小端推断的**，不是实测。推断当事实用过一次就出过 10 倍量化错（v4 的教训）。
本脚本把"看一眼 candump"变成可复现的判据：

    candump -L can0 > frames.log          # 抓 3 秒
    python3 scripts/verify_canopen_frames.py frames.log

判据（任一不过即 exit 1）：

1. 每帧 DLC == 该 PDO 的 `byte_size`（少一字节就是映射对不上，不做"尽力解析"）；
2. 每个 COB-ID 都能在契约里找到归属（未知 ID = 有第四份帧表在发）；
3. 解码出的物理量落在类型量程内**且**在合理物理范围内（轮速 |v| < 40 rad/s 等）；
4. 若同一对象在小端/大端两种解释下都能解出，打印两者差值——
   差值大而物理量合理的那一侧，才说明字节序。脚本不替你拍板，但把证据摆出来。

输入格式 = `candump -L`（`(1698... ) can0 201#AABB...`），也接受 `-ta` 的裸帧。
"""

from __future__ import annotations

import argparse
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "rak_canopen"))

from rak_canopen import contract as contract_mod  # noqa: E402

#: 物理合理范围（不是量程）。给的是"这台车不可能超过"的界，用来抓字节序错。
PLAUSIBLE = {
    "w1spd": 40.0, "w2spd": 40.0, "w3spd": 40.0, "w4spd": 40.0,
    "w1spd_fb": 40.0, "w2spd_fb": 40.0, "w3spd_fb": 40.0, "w4spd_fb": 40.0,
    "imu_dyaw": 20.0, "imu_pitch": 90.0, "imu_roll": 90.0,
    "imu_gz": 20.0, "imu_amag": 4.0,
}

_LINE = re.compile(r"\((?P<ts>[\d.]+)\)\s+(?P<bus>\S+)\s+(?P<cob>[0-9A-Fa-f]{3,8})#(?P<data>[0-9A-Fa-f]*)")


def parse(path: Path) -> list[tuple[float, int, bytes]]:
    frames = []
    for number, line in enumerate(path.read_text(encoding="utf-8", errors="replace").splitlines(), 1):
        match = _LINE.search(line)
        if not match:
            continue
        data_hex = match.group("data")
        if len(data_hex) % 2:
            print(f"WARN 第 {number} 行数据长度为奇数，跳过: {line.strip()[:80]}")
            continue
        frames.append((float(match.group("ts")), int(match.group("cob"), 16), bytes.fromhex(data_hex)))
    return frames


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("log", type=Path, help="candump -L 输出")
    parser.add_argument("--config", default="", help="契约目录（默认 src/rak_canopen/config）")
    parser.add_argument("--min-frames", type=int, default=1, help="每个 PDO 至少要看几帧")
    args = parser.parse_args()

    contract = contract_mod.load(args.config or None)
    frames = parse(args.log)
    if not frames:
        print("没有解析到任何帧：确认 candump 用了 -L（带 (timestamp)）", file=sys.stderr)
        return 2

    # COB-ID → (node, pdo)；外加协议标准帧（NMT/SYNC，CiA 301 固定 COB-ID）
    owners = {0x000: ("", "nmt"), 0x080: ("", "sync")}
    for name, node in contract.nodes.items():
        for cob_id, label in node.cob_ids().items():
            owners[cob_id] = (name, label)

    problems: list[str] = []
    seen: dict[int, int] = {}
    first_payload: dict[int, bytes] = {}

    for _ts, cob_id, payload in frames:
        owner = owners.get(cob_id)
        if owner is None:
            problems.append(f"未知 COB-ID 0x{cob_id:03X}（契约里没有：可能有第四份帧表在发）")
            continue
        node_name, label = owner
        seen[cob_id] = seen.get(cob_id, 0) + 1
        first_payload.setdefault(cob_id, payload)
        node = contract.nodes.get(node_name)
        if node is None:
            continue  # NMT/SYNC：协议标准帧，无契约对象可查
        pdo = node.find_cob(cob_id)
        if pdo is None:
            continue
        if len(payload) != pdo.byte_size:
            problems.append(
                f"0x{cob_id:03X} {label}: DLC {len(payload)} != 契约 {pdo.byte_size}")
            continue
        try:
            decoded = contract.decode(node_name, cob_id, payload)
        except Exception as exc:  # noqa: BLE001 - ContractError 及解码异常
            problems.append(f"0x{cob_id:03X} {label}: 解码失败 {exc}")
            continue
        for key, value in decoded.items():
            limit = PLAUSIBLE.get(key)
            if limit is not None and abs(value) > limit:
                problems.append(
                    f"0x{cob_id:03X} {label}.{key} = {value:.3f} 超出物理合理范围 ±{limit}"
                    "（最可能是字节序/量化因子错，不是传感器坏了）")

    print(f"帧总数 {len(frames)}，COB-ID {len(seen)} 个")
    for cob_id in sorted(seen):
        node_name, label = owners[cob_id]
        payload = first_payload[cob_id]
        print(f"  0x{cob_id:03X} {label:24s} x{seen[cob_id]:5d}  first={payload.hex(' ')}")

    missing = [f"0x{c:03X} {owners[c][1]}" for c in owners
               if c in seen and seen[c] < args.min_frames]
    if missing:
        problems.extend(f"帧数不足: {item}" for item in missing)

    # 字节序证据：把小端与"翻转后按小端解"都算一遍，差得多说明顺序敏感。
    print("\n字节序证据（同一帧两种解释的物理量，差值大者才具备判别力）：")
    for cob_id in sorted(seen):
        node_name = owners[cob_id][0]
        if node_name not in contract.nodes:
            continue  # NMT/SYNC
        pdo = contract.nodes[node_name].find_cob(cob_id)
        if pdo is None or pdo.byte_size < 2:
            continue
        payload = first_payload[cob_id]
        try:
            little = contract.decode(owners[cob_id][0], cob_id, payload)
            swapped = contract.decode(owners[cob_id][0], cob_id, payload[::-1])
        except Exception:  # noqa: BLE001
            continue
        key = pdo.object_names[0]
        print(f"  0x{cob_id:03X} {key}: little={little.get(key)}  byte-swapped={swapped.get(key)}")

    if problems:
        print("\nFAIL:")
        for item in dict.fromkeys(problems):
            print("  -", item)
        return 1
    print("\nOK: 帧长、COB-ID、物理范围全部与契约一致")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

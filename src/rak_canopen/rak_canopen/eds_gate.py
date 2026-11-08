"""EDS ↔ 契约一致性门禁（ADR-0022 待闭合 #5 的实现）。

`canopen_gate.py` 在下位机侧保证 JSON→EDS→OD 三件套一致；本模块把这条链延伸到 Jetson：
校验"我们派生出来的 COB-ID / 发送类型 / 映射对象数与位宽"是否与**同一份 EDS** 说的相符。

为什么值得单独做：EDS 是 CANopenEditor 生成固件 OD 的中间物，也是 ros2_canopen 装载从站的
输入（`slave_config`/`eds_file` 都吃 EDS）。也就是说 EDS 是"三端共同的事实"，
只校验下位机侧 = Jetson 仍然是靠推断对齐。

用法：
    python3 -m rak_canopen.eds_gate              # 全部节点
    python3 -m rak_canopen.eds_gate --node chassis
"""

from __future__ import annotations

import argparse
import configparser
import re
import sys
from pathlib import Path

from . import contract

TPDO_COM_BASE, TPDO_MAP_BASE = 0x1800, 0x1A00
RPDO_COM_BASE, RPDO_MAP_BASE = 0x1400, 0x1600


class EdsError(ValueError):
    pass


def read_eds(path: Path) -> dict[tuple[str, str], str]:
    """EDS 是 INI 方言（小节名大小写敏感、含 `sub1` 后缀），用 configparser 原样读。"""
    parser = configparser.ConfigParser(interpolation=None, strict=False)
    # EDS 里允许重复键（不同小节），且没有默认值段
    text = path.read_text(encoding="utf-8", errors="replace")
    parser.read_string(text)
    out: dict[tuple[str, str], str] = {}
    for section in parser.sections():
        for key, value in parser.items(section):
            out[(section.lower(), key.lower())] = value.strip()
    return out


def _get(eds, section: str, key: str, default: str | None = None) -> str | None:
    return eds.get((section.lower(), key.lower()), default)


def check_node(node_name: str, contract_obj, *, eds_dir: str | Path | None = None,
               tolerance_frames: int = 0) -> list[str]:
    """返回不一致清单；空列表 = 一致。"""
    root = Path(eds_dir) if eds_dir else (contract_obj.root or contract.default_config_dir())
    path = root / f"{node_name}.eds"
    if not path.is_file():
        raise EdsError(f"缺少 EDS 副本：{path}")
    eds = read_eds(path)
    problems: list[str] = []
    node = contract_obj.node(node_name)

    for index in range(1, 5):
        for direction, com_base, map_base, cia_base in (
            ("tpdo", TPDO_COM_BASE, TPDO_MAP_BASE, 0x180),
            ("rpdo", RPDO_COM_BASE, RPDO_MAP_BASE, 0x200),
        ):
            group = f"{direction}{index}"
            pdo = next((p for p in node.pdos if p.name == group), None)
            # 槽位号从 1 起，但 EDS 段号从 0 起：TPDO1 的通信参数是 0x1800、映射是 0x1A00。
            # 第一版写成 +index，于是整条链都读成了下一个 PDO 的参数（0x280、48 位…）。
            com_section, map_section = (
                f"{com_base + index - 1:04X}", f"{map_base + index - 1:04X}",
            )
            # EDS 的通信参数在子索引里：sub1 = COB-ID，sub2 = 发送类型；
            # 段本身只有 ParameterName/ObjectType/SubNumber。第一版把它当段属性读，
            # 于是所有 PDO 都被误报成"EDS 里没有"。
            cob_default = _get(eds, f"{com_section}sub1", "DefaultValue")
            if cob_default is None:
                if pdo is not None and _get(eds, com_section, "ParameterName") is None:
                    problems.append(f"{node_name}.{group}: EDS 里没有该 PDO 通信参数段")
                elif pdo is not None:
                    problems.append(f"{node_name}.{group}: EDS 缺 {com_section}sub1 DefaultValue")
                continue
            if pdo is None:
                # 未使用的槽位必须置禁用位（bit31），否则 CANopenNode 会报 PDO_WRONG_MAPPING。
                if int(cob_default, 0) & 0x80000000 == 0:
                    problems.append(
                        f"{node_name}.{group}: EDS 未使用却未置禁用位 bit31（{cob_default}）"
                    )
                continue
            # EDS/OD 存的是**基址**（0x180/0x200…），节点号由栈在运行时加：
            # 实测 OD.c `x1800...COB_IDUsedByTPDO = 0x00000180`，而总线上是 0x181（node 1）。
            # 所以比较式是 EDS 基址 + node_id == 派生 COB-ID，不是 EDS 值直接相等。
            expected = cia_base + (index - 1) * 0x100 + node.node_id
            cob_value = (int(cob_default, 0) & 0x1FFFFFFF) + node.node_id
            if cob_value != expected:
                problems.append(
                    f"{node_name}.{group}: EDS COB-ID 默认 0x{cob_value:03X} "
                    f"!= 派生 0x{expected:03X}（CiA 301 基址+node_id）"
                )
            if direction == "tpdo":
                ttype = _get(eds, f"{com_section}sub2", "DefaultValue")
                # 发送类型：JSON 的 tpdo_types 与 EDS 必须同值，否则上位机速率判据全错。
                if ttype is not None and pdo.transmission_type is not None:
                    if int(ttype, 0) != pdo.transmission_type:
                        problems.append(
                            f"{node_name}.{group}: EDS 发送类型 {ttype} "
                            f"!= 清单 {pdo.transmission_type}（⇒ 速率判据不一致）"
                        )
            # 实际映射数 = sub0 的 DefaultValue；SubNumber 只是"支持几个子索引"（模板里恒为 9）。
            count = _get(eds, f"{map_section}sub0", "DefaultValue")
            if count is not None and int(count, 0) != len(pdo.objects):
                problems.append(
                    f"{node_name}.{group}: EDS 映射对象数 {int(count, 0)} "
                    f"!= 清单 {len(pdo.objects)}"
                )
            # 映射位宽之和必须等于 PDO 字节数（EDS 里 1600sub* 是 0xIIII0003 形式）
            bits = 0
            for sub in range(1, len(pdo.objects) + 1):
                entry = _get(eds, f"{map_section}sub{sub}", "DefaultValue")
                if entry is None:
                    break
                bits += int(entry, 0) & 0xFF
            if bits and bits != sum(o.bits for o in pdo.objects):
                problems.append(
                    f"{node_name}.{group}: EDS 映射位宽合计 {bits} "
                    f"!= 清单 {sum(o.bits for o in pdo.objects)}"
                )
    return problems


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="rak_canopen.eds_gate")
    parser.add_argument("--node", default=None, choices=sorted(["chassis", "arm"]))
    args = parser.parse_args(argv)
    c = contract.load()
    names = [args.node] if args.node else sorted(c.nodes)
    failed = False
    for name in names:
        problems = check_node(name, c)
        if problems:
            failed = True
            print(f"FAIL {name}:")
            for item in problems:
                print(f"  - {item}")
        else:
            print(f"OK   {name}: EDS 与契约派生一致（COB-ID / 发送类型 / 映射数与位宽）")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())

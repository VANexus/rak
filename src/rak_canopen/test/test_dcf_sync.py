#!/usr/bin/env python3
"""master.dcf 漂移门禁：用官方 `dcfgen` 重新生成，与仓内副本逐字节比对。

为什么要这条：`master.dcf` 是**派生件**（bus.yaml + 两份 EDS 的产物），不是手工文件。
上游 ros2_canopen 的官方生成器就是 `dcfgen`（apt: ros-jazzy-canopen-dcf-*）。
不钉住它，改 EDS/改节点 id 后主站会拿一份旧网络表启动，症状是"从站不 boot"。

重生成（`@BUS_CONFIG_PATH@` 必须先替换成 EDS 所在目录，否则从站 dcf 拼不出路径）：
    scripts/gen_canopen_master_dcf.py

`-S` 的原因：EDS 由固件仓 gen_eds.py 生成，里面保留 CANopenNode 导出器要求的
`CountLabel` 条目 —— CiA 306 合法，但 dcf-lint 不认，只能 no-strict。其余条目
（含 `$NODEID+0x...` 形式的 COB-ID 默认值）都按 dcf-lint 的严格形式写。
"""

import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

CONFIG = Path(__file__).resolve().parents[1] / "config"


def _dcfgen(tmp_path: Path) -> Path:
    exe = shutil.which("dcfgen")
    if exe is None:
        pytest.skip("dcfgen 未安装（apt: ros-jazzy-canopen-dcf-tools）")
    for name in ("chassis.eds", "arm.eds"):
        shutil.copy(CONFIG / name, tmp_path / name)
    # 与 chassis_bringup 相同的替换：@BUS_CONFIG_PATH@ → 本次 EDS 所在目录
    bus = (CONFIG / "bus.yaml").read_text(encoding="utf-8")
    (tmp_path / "bus.yaml").write_text(
        bus.replace("@BUS_CONFIG_PATH@", str(tmp_path)), encoding="utf-8")
    proc = subprocess.run(
        [exe, "-S", "-r", "-d", str(tmp_path), str(tmp_path / "bus.yaml")],
        cwd=tmp_path,
        capture_output=True,
        text=True,
    )
    assert proc.returncode == 0, proc.stderr[-800:]
    out = tmp_path / "master.dcf"
    assert out.is_file(), proc.stderr[-800:]
    return out


def test_master_dcf_matches_official_generator(tmp_path):
    generated = _dcfgen(tmp_path)
    committed = (CONFIG / "master.dcf").read_text(encoding="utf-8")
    fresh = generated.read_text(encoding="utf-8")
    if generated.read_bytes() != (CONFIG / "master.dcf").read_bytes():
        sys.stderr.write(
            "master.dcf 与 dcfgen 重新生成的结果不一致：\n"
            "    cd src/rak_canopen/config && dcfgen -S -d . bus.yaml\n"
        )
    assert fresh == committed

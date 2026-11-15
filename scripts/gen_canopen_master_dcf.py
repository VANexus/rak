#!/usr/bin/env python3
"""用官方 dcfgen 重新生成 `src/rak_canopen/config/master.dcf`（ADR-0024）。

为什么需要这一步：`device_container` 的 `master_config` 必须是**主站自己的 DCF**，
里面要带 CiA 302 网络表（1F80/1F81/1F85）。拿从站 EDS 冒充它，主站能加载但
不发 NMT reset，代理侧 1 s Boot Timeout 后 abort（2026-11-14 实测，见
docs/status/canopen_vcan_2026-11-14.md）。

`bus.yaml` 里的 `dcf_path` 是 `@BUS_CONFIG_PATH@` 占位符（运行时由 chassis_bringup
替换成安装态 config 目录）；dcfgen 需要真实目录，所以这里先展开再生成。
"""

import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

CONFIG = Path(__file__).resolve().parents[1] / "src" / "rak_canopen" / "config"


def main() -> int:
    exe = shutil.which("dcfgen")
    if exe is None:
        sys.exit("dcfgen 未安装（apt: ros-jazzy-canopen-dcf-tools）")
    with tempfile.TemporaryDirectory() as tmp:
        tmp_path = Path(tmp)
        for name in ("chassis.eds", "arm.eds"):
            shutil.copy(CONFIG / name, tmp_path / name)
        bus = (CONFIG / "bus.yaml").read_text(encoding="utf-8")
        (tmp_path / "bus.yaml").write_text(
            bus.replace("@BUS_CONFIG_PATH@", str(tmp_path)), encoding="utf-8")
        # `-r`（remote PDO）不是可选项：没有它主站 OD 里就没有 0x5800/0x5A00 这组
        # 远端 PDO 对象，lely 不会把从站 TPDO 当 RPDO 收下，OnRpdoWrite 永不触发，
        # `~/rpdo` 一帧都没有（2026-11-15 真机实测：0x182 在线上 50 Hz，话题空）。
        proc = subprocess.run([exe, "-S", "-r", "-d", str(tmp_path), str(tmp_path / "bus.yaml")],
                              cwd=tmp_path, capture_output=True, text=True)
        if proc.returncode != 0:
            sys.exit("dcfgen 失败：\n" + (proc.stderr or proc.stdout)[-1200:])
        shutil.copy(tmp_path / "master.dcf", CONFIG / "master.dcf")
    print("master.dcf 已重新生成：", CONFIG / "master.dcf")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

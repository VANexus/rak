#!/usr/bin/env python3
"""按**档位**构建工作区（ADR-0026），让"桌面档一条命令成立"从口头变成命令。

背景：在此之前，桌面机能不能建起来取决于操作员记不记得那一串
``--packages-ignore rak_astra_driver … rf2o_laser_odometry``。那是一份**隐性依赖**：
换人、换机器、换一天就漏一两个包，而漏掉的表现是"launch 起不来"。

现在档位是 `src/rak_orchestration/config/build_profiles.yaml` 里的数据，
本脚本只做两件事：把 skip 表翻译成 colcon 参数，构建完立刻跑结构门禁。

用法::

    python3 scripts/build_workspace.py --profile desktop
    python3 scripts/build_workspace.py --profile jetson --no-check
    python3 scripts/build_workspace.py --profile desktop -- --cmake-args -DCMAKE_BUILD_TYPE=Release

`--` 之后的参数原样透传给 colcon（不要在档位里塞构建选项）。
"""

from __future__ import annotations

import argparse
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "rak_orchestration"))

from rak_orchestration import profiles  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--profile", default="desktop", help="档位名（见 build_profiles.yaml）")
    parser.add_argument("--no-check", action="store_true", help="只构建，不跑 check_workspace")
    parser.add_argument("--dry-run", action="store_true", help="只打印将要执行的命令")
    parser.add_argument("colcon_args", nargs=argparse.REMAINDER, help="`--` 之后透传给 colcon")
    args = parser.parse_args()

    profile = profiles.load(args.profile)
    extra = [item for item in args.colcon_args if item != "--"]

    # 只发现 src/（自有包 + src/vendor 厂商栈）。仓库根下还有 tools/（orbbec SDK
    # 示例等非 ROS 内容），默认全仓扫描会把它们当成包——OrbbecSDK-Samples 就是
    # 这样混进构建图并整条链 aborted 的。
    command = ["colcon", "build", "--symlink-install", "--base-paths", "src"]
    # skip 只影响"要不要构建"，不影响依赖声明；colcon 允许过滤不存在的包名。
    if profile.skip_packages:
        command += ["--packages-skip", *profile.skip_packages]
    command += extra

    # CUDA：JetPack 的 nvcc 不在非登录 shell 的 PATH 里，colcon 的 enable_language(CUDA)
    # 会报 "No CMAKE_CUDA_COMPILER"。有 /usr/local/cuda 就补上（用户显式设置的优先）。
    cuda_bin = Path("/usr/local/cuda/bin")
    if cuda_bin.is_file() or (cuda_bin.with_name("nvcc")).is_file():
        os.environ.setdefault("CUDACXX", "/usr/local/cuda/bin/nvcc")
        os.environ["PATH"] = f"/usr/local/cuda/bin{os.pathsep}{os.environ.get('PATH', '')}"

    print(f"[{profile.name}] {profile.description.strip()}")
    if profile.skip_packages:
        print(f"[{profile.name}] 跳过 {len(profile.skip_packages)} 个包: "
              f"{', '.join(profile.skip_packages)}")
    if profile.tolerate_absent:
        print(f"[{profile.name}] 容忍缺失: {', '.join(profile.tolerate_absent)}")
    print("+ " + " ".join(command))

    if args.dry_run:
        return 0
    result = subprocess.run(command, cwd=ROOT)
    if result.returncode != 0:
        return result.returncode
    if args.no_check:
        return 0
    return subprocess.run(
        [sys.executable, str(ROOT / "scripts" / "check_workspace.py"),
         "--installed", "--profile", profile.name],
        cwd=ROOT,
    ).returncode


if __name__ == "__main__":
    raise SystemExit(main())

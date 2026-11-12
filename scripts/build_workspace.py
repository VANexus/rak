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
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "rak_orchestration"))

from rak_orchestration import profiles  # noqa: E402


def _install_base(colcon_args: list[str]):
    """从透传给 colcon 的参数里取 --install-base；没写就用默认的 install/。"""
    for index, item in enumerate(colcon_args):
        if item == "--install-base" and index + 1 < len(colcon_args):
            return ROOT / colcon_args[index + 1]
        if item.startswith("--install-base="):
            return ROOT / item.split("=", 1)[1]
    return ROOT / "install"


def _augment_env(env: dict[str, str], install_base) -> None:
    """把安装空间叠加到环境里，等价于 source 它的 local_setup.bash。

    只做三件事，不解析 setup 脚本本身：
      * AMENT_PREFIX_PATH —— ament_index_python 找 share/<pkg>/ 靠它；
      * PYTHONPATH        —— ament_python 包的模块与 console entry point；
      * LD_LIBRARY_PATH   —— ament_cmake 包的 .so（含 rak_chassis 的插件）。
    setup.bash 还会设 PYTHONPATH 里的 site-packages 等等，但门禁只用到上述三类；
    多设反而会在重复构建时把旧路径越叠越长。
    """
    if not install_base.is_dir():
        return
    prefixes = sorted({str(path) for path in install_base.iterdir() if path.is_dir()})
    prefixes.insert(0, str(install_base))
    def _prepend(key: str, paths: list[str]) -> None:
        old = env.get(key, "")
        env[key] = os.pathsep.join(paths + ([old] if old else []))
    _prepend("AMENT_PREFIX_PATH", prefixes)
    _prepend("LD_LIBRARY_PATH", [str(install_base / name / "lib")
                                 for name in prefixes[1:] if (install_base / name / "lib").is_dir()])
    # ament_python 的模块位置有两处，都要给：
    #   * <install>/<pkg>/lib/python3*/site-packages —— 安装态的正式位置；
    #   * <build>/<pkg>          —— --symlink-install 的实际落点（egg-link
    #     指回这里，但 egg-link 只在 pkg_resources 先被 import 时才生效，
    #     门禁进程不满足这个前提，所以把路径直接给够，不赌 import 顺序）。
    python_paths = [str(path) for path in install_base.glob("*/lib/python3*/site-packages")]
    build_base = ROOT / "build"
    if build_base.is_dir():
        python_paths += [str(path) for path in sorted(build_base.glob("*/"))]
    _prepend("PYTHONPATH", python_paths)


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
    # 2026-11-13 修正：原判断是 cuda_bin.is_file() or cuda_bin.with_name("nvcc").is_file()，
    # 而 Path("/usr/local/cuda/bin").with_name("nvcc") 得到的是 /usr/local/cuda/nvcc
    # —— with_name 替换的是最后一段（bin），不是追加。所以两个条件永远为假，
    # PATH/CUDACXX 从来没被设置过，CMake 的 find_package(CUDAToolkit) 也就找不到
    # include 与 libcudart（它靠 nvcc 的路径反推 SDK 根，而 /usr/local/bin/nvcc
    # 是符号链接，反推出来的根是 /usr/local，那里没有 targets/sbsa-linux）。
    cuda_bin = Path("/usr/local/cuda/bin")
    if (cuda_bin / "nvcc").is_file():
        os.environ.setdefault("CUDACXX", "/usr/local/cuda/bin/nvcc")
        os.environ["PATH"] = f"/usr/local/cuda/bin{os.pathsep}{os.environ.get('PATH', '')}"
        # CUDAToolkit_ROOT 是给 CMake 的确定性提示：不依赖 nvcc 符号链接能否被
        # realpath 解开（JetPack 上 /usr/local/bin/nvcc -> /usr/local/cuda/bin/nvcc
        # 是两级链接，FindCUDAToolkit 只解到 /usr/local 就停了）。
        os.environ.setdefault("CUDAToolkit_ROOT", "/usr/local/cuda")

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
    # 门禁必须看见**刚构建出来的**安装空间。
    # 2026-11-13：这里以前直接 subprocess.run(...)，继承了调用者的环境 —— 于是
    # 在不 source 任何东西的终端里跑本脚本，门禁会以
    #   package not found, searching: [/opt/ros/jazzy]
    # 失败。构建明明是成功的，门禁却红；而 --help 从没说要先 source ——
    # 这是脚本的锅，不是使用者的锅。
    # 修法：把安装空间拼进 AMENT_PREFIX_PATH / PYTHONPATH / LD_LIBRARY_PATH，
    # 而不是逼人先 source（source 在 subprocess 里传不下去）。
    env = os.environ.copy()
    install_base = _install_base(extra)
    if install_base is not None:
        _augment_env(env, install_base)
    return subprocess.run(
        [sys.executable, str(ROOT / "scripts" / "check_workspace.py"),
         "--installed", "--profile", profile.name],
        cwd=ROOT,
        env=env,
    ).returncode


if __name__ == "__main__":
    raise SystemExit(main())

"""构建档位与 launch 契约（ADR-0026）。

存在理由：这台工作区同时要在**两种机器**上成立——

* 桌面机（x86，无 GPU、无 Astra/雷达/双目、无 JetPack）：写上层逻辑、跑 RViz 与夹具；
* Orin（Jetson，全套硬件）：跑真栈。

在拆分之前，`rak_bringup` 把所有 GPU/传感器包写成 `exec_depend`，于是"桌面档能建"
只能靠人肉 `--packages-ignore rak_astra_driver ... rf2o_laser_odometry` 拼出来。
那种命令行是**不可复现的隐性依赖**：换个人、换台机器就失效，而且没人知道漏了哪个。

所以档位是数据，不是命令行：

* `config/build_profiles.yaml` —— 每档跳过哪些包、容忍哪些包缺失；
* `config/launch_contracts.yaml` —— 每个 launch 属于哪一档、为什么（桌面档不可用的
  必须写明缺什么），跳过是**声明**而不是"启动失败后猜原因"。

本模块只做解析与不变量校验，不 import ROS，因此 `check_workspace.py`、`build_workspace.py`
和 pytest 三处都能用同一份事实。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
import os

import yaml

#: 允许通过环境变量覆盖配置目录（测试与 CI 用；正常工作走 ament 安装空间）。
CONFIG_ENV = "RAK_ORCHESTRATION_CONFIG_DIR"

#: 兜底：源码树内相对路径。仅在未安装（直接跑脚本）时使用。
_SOURCE_CONFIG = Path(__file__).resolve().parents[1] / "config"


class ProfileError(RuntimeError):
    """档位或契约不自洽。消息里必须指出是哪个键、哪一条。"""


def config_dir() -> Path:
    """配置目录：环境变量 → ament 安装空间 → 源码树。"""
    override = os.environ.get(CONFIG_ENV)
    if override:
        return Path(override)
    try:  # 安装后走 ament index（ADR 原则：禁止依赖源码绝对路径）
        from ament_index_python.packages import get_package_share_directory

        installed = Path(get_package_share_directory("rak_orchestration")) / "config"
        if installed.is_dir():
            return installed
    except Exception:  # noqa: BLE001 - 未 source 环境时退源码树
        pass
    return _SOURCE_CONFIG


def _load(name: str) -> dict:
    path = config_dir() / name
    if not path.is_file():
        raise ProfileError(f"缺少配置: {path}")
    with path.open(encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


@dataclass(frozen=True)
class LaunchContract:
    """一个 launch 的归属声明。"""

    name: str
    package: str
    profiles: tuple[str, ...]
    requires: tuple[str, ...] = ()
    reason: str = ""

    @property
    def desktop(self) -> bool:
        return "desktop" in self.profiles


@dataclass(frozen=True)
class Profile:
    name: str
    description: str
    skip_packages: tuple[str, ...] = ()
    tolerate_absent: tuple[str, ...] = ()
    launch_contracts: dict[str, LaunchContract] = field(default_factory=dict)

    # ---------------------------------------------------------------- 查询
    def absent_candidates(self) -> set[str]:
        """本档**允许**缺失的包。注意是允许，不是保证。"""
        return set(self.skip_packages) | set(self.tolerate_absent)

    def skips(self, package: str) -> bool:
        return package in self.skip_packages

    def launch_allowed(self, launch: str) -> bool:
        """未知 launch 默认允许（外部/上游包），已知的按声明的档位判。"""
        contract = self.launch_contracts.get(launch)
        return True if contract is None else self.name in contract.profiles

    def launch_reason(self, launch: str) -> str:
        contract = self.launch_contracts.get(launch)
        if contract is None:
            return "未声明"
        return contract.reason or ",".join(contract.requires) or "未说明理由"


def _launch_key(package: str, name: str) -> str:
    return f"{package}/{name}"


def load_launch_contracts() -> dict[str, LaunchContract]:
    raw = _load("launch_contracts.yaml")
    contracts: dict[str, LaunchContract] = {}
    for item in raw.get("launches", []):
        missing = {"package", "name", "profiles"} - set(item)
        if missing:
            raise ProfileError(f"launch 契约缺键 {sorted(missing)}: {item}")
        key = _launch_key(item["package"], item["name"])
        if key in contracts:
            raise ProfileError(f"launch 契约重复声明: {key}")
        profiles = tuple(item["profiles"])
        unknown = set(profiles) - set(profile_names())
        if unknown:
            raise ProfileError(f"{key} 引用了未知档位 {sorted(unknown)}")
        contracts[key] = LaunchContract(
            name=item["name"],
            package=item["package"],
            profiles=profiles,
            requires=tuple(item.get("requires", ())),
            reason=item.get("reason", ""),
        )
    return contracts


def profile_names() -> tuple[str, ...]:
    raw = _load("build_profiles.yaml")
    return tuple(raw.get("profiles", {}))


def load(name: str) -> Profile:
    raw = _load("build_profiles.yaml")
    profiles = raw.get("profiles", {})
    if name not in profiles:
        raise ProfileError(f"未知档位 {name!r}；可用: {sorted(profiles)}")
    body = profiles[name] or {}
    contracts = load_launch_contracts()
    return Profile(
        name=name,
        description=body.get("description", ""),
        skip_packages=tuple(body.get("skip_packages", ())),
        tolerate_absent=tuple(body.get("tolerate_absent", ())),
        launch_contracts=contracts,
    )


def validate(profile: Profile, packages: set[str] | None = None) -> list[str]:
    """返回问题列表（空 = 通过）。供 check_workspace 与 pytest 共用。

    钉住三件事，都是今天真出过问题的：

    1. 声明跳过某个自有包 → 必须同时有 launch 契约说明它归哪档，否则这个包就是
       "谁也起不来但也没人说"的孤儿；
    2. 桌面档跳过的包，必须真的被某个 launch 用到；否则是一条过期豁免，
       会让人以为"桌面档需要它"而反复装依赖；
    3. 档位里出现的包名必须是工作区真实存在的自有包或已知外部包（拼错即失败）。
    """
    problems: list[str] = []
    own = packages or set()
    for package in profile.skip_packages:
        if own and package not in own:
            # 外部包（lslidar_driver 等）不在 EXPECTED 里，这里只校验自有包的拼写
            if package.startswith("rak_") and package not in own:
                problems.append(f"档位 {profile.name} 跳过的自有包不存在: {package}")
    for key, contract in profile.launch_contracts.items():
        excluded = profile.name not in contract.profiles
        if excluded and not (set(contract.requires) & profile.absent_candidates()):
            # 契约说"本档不可用"，但 requires 与本档的缺口表没有交集 → 理由不成立，
            # 这是一条过期豁免（会让人以为"本档需要它"而反复装依赖）。
            problems.append(
                f"{key} 声明 {profile.name} 不可用，但 requires={list(contract.requires)} "
                f"与本档跳过表无交集"
            )
        if not excluded and set(contract.requires) & set(profile.skip_packages):
            problems.append(
                f"{key} 自称 {profile.name} 可用却依赖被跳过的 "
                f"{sorted(set(contract.requires) & set(profile.skip_packages))}"
            )
    return problems

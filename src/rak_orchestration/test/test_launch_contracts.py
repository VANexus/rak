"""档位与 launch 契约门禁（ADR-0026）。

钉住的是"桌面档为什么起不来"这件事的**可执行版本**：

1. 引用了本档 `skip_packages` 的 launch，必须在 `launch_contracts.yaml` 里声明，
   且声明里不含 `desktop` —— 漏声明就是红，不是"启动失败后猜原因"；
2. 反过来，声明"桌面不可用"的 launch 必须真的引用了一个被跳过的包，
   否则那条豁免是过期借口，会让人以为"桌面档需要它"；
3. 声明里提到的包必须出现在档位的 skip/tolerate 里（拼错即失败）。

这三条正好对应今天的现状：`rak_bringup` 把 GPU/厂商包写成 `exec_depend`，
于是"桌面档能建"只能靠 `--packages-ignore rak_astra_driver …` 拼出来。
把豁免变成数据之后，同一条命令在任何机器上都成立。
"""

from __future__ import annotations

from pathlib import Path
import re

import pytest

from rak_orchestration import profiles as profile_mod

#: rak/src —— 测试文件在 rak/src/rak_orchestration/test/ 下。
SRC = Path(__file__).resolve().parents[2]

#: launch 里引用包名的两种写法（与本仓库现有 launch 的写法一致）。
_REF = re.compile(r'(?:package\s*=\s*"([a-z0-9_]+)"|get_package_share_directory\(\s*"([a-z0-9_]+)"\s*\))')


def _launch_files() -> dict[str, Path]:
    found: dict[str, Path] = {}
    for path in sorted(SRC.glob("*/launch/*.launch.py")):
        if "__pycache__" in path.parts:
            continue
        found[f"{path.parent.parent.name}/{path.name}"] = path
    return found


def _referenced_packages(path: Path) -> set[str]:
    text = path.read_text(encoding="utf-8")
    return {a or b for a, b in _REF.findall(text)}


def _contracts() -> dict[str, profile_mod.LaunchContract]:
    return profile_mod.load_launch_contracts()


def test_profile_files_load_and_are_self_consistent():
    names = profile_mod.profile_names()
    assert "desktop" in names and "jetson" in names
    for name in names:
        problems = profile_mod.validate(profile_mod.load(name))
        assert not problems, f"{name}: {problems}"


def test_declared_contracts_point_at_real_launch_files():
    files = _launch_files()
    for key, contract in _contracts().items():
        assert key in files, f"契约声明了不存在的 launch: {key}"


def test_every_launch_touching_a_skipped_package_is_declared_non_desktop():
    """这是本门禁的主判据：桌面档跑不了的 launch 必须**说出来**。"""
    desktop = profile_mod.load("desktop")
    contracts = _contracts()
    undeclared = []
    for key, path in _launch_files().items():
        touched = _referenced_packages(path) & set(desktop.skip_packages)
        if not touched:
            continue
        contract = contracts.get(key)
        if contract is None:
            undeclared.append(f"{key} 引用 {sorted(touched)} 但没有契约")
        elif contract.desktop:
            undeclared.append(f"{key} 引用 {sorted(touched)} 却声明 desktop 可用")
    assert not undeclared, "桌面档无法解析的 launch 必须显式豁免:\n" + "\n".join(undeclared)


def test_contract_exemptions_are_not_stale():
    """不桌面可用的契约必须给出**本档真的跳过**的包，避免过期豁免。"""
    desktop = profile_mod.load("desktop")
    stale = []
    for key, contract in _contracts().items():
        if contract.desktop:
            continue
        if not set(contract.requires) & desktop.absent_candidates():
            stale.append(f"{key}: requires={list(contract.requires)} 与档位跳过表无交集")
    assert not stale, "\n".join(stale)


def test_hardware_packages_are_not_pulled_in_by_contract_layers():
    """契约层不许依赖硬件包 —— 否则拆分只是把同一个问题挪了个位置。"""
    desktop = profile_mod.load("desktop")
    orchestration_manifest = (SRC / "rak_orchestration/package.xml").read_text(encoding="utf-8")
    for package in desktop.skip_packages:
        assert f"<exec_depend>{package}</exec_depend>" not in orchestration_manifest, package
        assert f"<depend>{package}</depend>" not in orchestration_manifest, package


@pytest.mark.parametrize("profile_name", ["desktop", "jetson"])
def test_profile_skip_list_only_names_known_packages(profile_name):
    """档位里拼错一个包名，表现是"豁免没生效"而不是报错 —— 所以在这里钉死。"""
    profile = profile_mod.load(profile_name)
    own = {path.parent.name for path in (SRC).glob("rak_*/package.xml")}
    unknown_own = {p for p in profile.skip_packages if p.startswith("rak_") and p not in own}
    assert not unknown_own, f"{profile_name} 跳过了不存在的自有包: {sorted(unknown_own)}"

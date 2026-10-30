"""定位 vendored Claude Agent SDK 覆盖层（ADR-0010 / AGENTS §2 第 3 条）。

**为什么需要这个模块**：原来四个文件里各写死一次
``$HOME/rak/tools/harness-sdk``。换人、换机、换用户名即坏，而且坏的方式是
"语音路径静默退回 dialogue"——看起来像模型没配好，实际是路径不存在。

解析顺序（先具体后通用，每一步都能被操作员覆盖）：

1. **参数** ``agent_sdk_path``（launch 传 ``-p agent_sdk_path:=...``，最高优先）；
2. **环境变量** ``RAK_HARNESS_SDK_DIR``（CI 与临时覆盖用，换机只改一处）；
3. **ament 索引** ``share/rak_harness/harness-sdk``（把 SDK 装进包里的部署形态，
   换机器不用改任何脚本）；
4. **源码树** ``<repo>/tools/harness-sdk``（开发机 ``--symlink-install`` 形态）。

任何一步都不假定用户名或盘符。全部失败返回 ``None``：上层据此**降级**到
dialogue 语音路径并打印"为什么"，而不是崩或装作正常。
"""

from __future__ import annotations

import os
from pathlib import Path

#: 覆盖层里必须存在的目录 —— 用它判断"这是一个真的 SDK 覆盖层"而不是空目录。
_SENTINEL = "claude_agent_sdk"

ENV_VAR = "RAK_HARNESS_SDK_DIR"


def _looks_like_sdk(path: Path) -> bool:
    return path.is_dir() and (path / _SENTINEL).is_dir()


def _ament_candidate() -> Path | None:
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("rak_harness")) / "harness-sdk"
    except Exception:  # noqa: BLE001 - 未 source 环境时正常走到源码树
        return None


def _source_tree_candidate() -> Path | None:
    # .../src/rak_harness/rak_harness/sdk.py → .../<repo>/tools/harness-sdk
    here = Path(__file__).resolve()
    for parent in here.parents:
        candidate = parent / "tools" / "harness-sdk"
        if candidate.is_dir():
            return candidate
    return None


def candidates(param: str = "") -> list[Path]:
    """按优先级返回所有候选路径（含不存在的），供日志与排障使用。"""
    items: list[Path] = []
    if param:
        items.append(Path(param).expanduser())
    env = os.environ.get(ENV_VAR, "")
    if env:
        items.append(Path(env).expanduser())
    for candidate in (_ament_candidate(), _source_tree_candidate()):
        if candidate is not None:
            items.append(candidate)
    # 老默认值保留为**最后**的兜底，仅为让现场那台机器在迁移期不掉线；
    # 它不再是任何代码的声明默认值，也不再出现在 launch 里。
    items.append(Path("/home/xrak/rak/tools/harness-sdk"))  # RAK_ALLOW_HOME_PATH: 迁移期兜底
    return items


def resolve(param: str = "") -> Path | None:
    """返回第一个存在的 SDK 覆盖层；都不存在返回 ``None``。"""
    for candidate in candidates(param):
        if _looks_like_sdk(candidate):
            return candidate
    return None


def explain(param: str = "") -> str:
    """一行可读的排障串：每个候选 + 是否存在，进 /diagnostics 与日志。"""
    parts = []
    for candidate in candidates(param):
        mark = "ok" if _looks_like_sdk(candidate) else "missing"
        parts.append(f"{candidate} [{mark}]")
    return "; ".join(parts)

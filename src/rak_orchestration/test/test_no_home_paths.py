"""禁止源码里出现绝对家目录/盘符（AGENTS §2 第 3 条）。

这条规矩以前只写在文档里，于是四个文件各写死一次
``$HOME/rak/tools/harness-sdk``，换人换机即坏。文档拦不住复制粘贴，
门禁可以。

判据是**路径语义**而不是字符串：`/home/<user>/…`、`/Users/<user>/…`、
`C:\\Users\\…` 都算违规。真要保留（迁移期兜底），必须在同一行写
``RAK_ALLOW_HOME_PATH`` 与理由，这样它至少是**被看见的**例外。

为什么放在 rak_orchestration：它是纯 x86 契约层，桌面档也会跑这条测试，
而违规最常见的来源正是"本机路径先跑通再说"。
"""

from __future__ import annotations

import re
from pathlib import Path

SRC = Path(__file__).resolve().parents[2]

#: 源码与配置（不含 install/build/git 历史；`vendor/` 是上游原样代码，不改）。
_SUFFIXES = {".py", ".cpp", ".hpp", ".h", ".c", ".yaml", ".yml", ".sh", ".xml", ".md"}
_SKIP_DIRS = {"__pycache__", "install", "build", "log", ".git", "vendor"}

_HOME_PATH = re.compile(
    r"(/home/[A-Za-z0-9._-]+/|/Users/[A-Za-z0-9._-]+/|[A-Za-z]:\\\\?Users\\\\[A-Za-z0-9._-]+)"
)
_ALLOW = "RAK_ALLOW_HOME_PATH"


def _iter_sources():
    for path in SRC.rglob("*"):
        if not path.is_file() or path.suffix not in _SUFFIXES:
            continue
        if _SKIP_DIRS & set(path.parts):
            continue
        yield path


def test_no_absolute_home_paths_in_sources():
    offenders: list[str] = []
    for path in _iter_sources():
        try:
            text = path.read_text(encoding="utf-8")
        except UnicodeDecodeError:
            continue
        for number, line in enumerate(text.splitlines(), start=1):
            if not _HOME_PATH.search(line):
                continue
            if _ALLOW in line:
                continue
            offenders.append(f"{path.relative_to(SRC)}:{number}: {line.strip()[:120]}")
    assert not offenders, (
        "源码里出现绝对家目录（AGENTS §2 第 3 条）。改为环境变量/参数/ament 索引推导；\n"
        "确需保留的写在同一行并注明 RAK_ALLOW_HOME_PATH：\n" + "\n".join(offenders)
    )

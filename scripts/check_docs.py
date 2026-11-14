#!/usr/bin/env python3
"""docs/ 门禁：链接可达、索引覆盖、状态行存在。

用法（仓根目录）：
    python3 scripts/check_docs.py

失败即退出码 1，并逐条打印问题。规则见 docs/README.md §3。
"""
from __future__ import annotations

import os
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DOCS = ROOT / "docs"
INDEX = DOCS / "README.md"
ADR_INDEX = DOCS / "adr" / "README.md"
SCAN_FILES = [ROOT / "README.md", ROOT / "AGENTS.md", ROOT / "TODO.md"]
STATUS_RE = re.compile(r"^> 状态：", re.M)
LINK_RE = re.compile(r"\[[^\]]*\]\(([^)\s]+)")
STATUS_WINDOW = 12


def markdown_files() -> list[Path]:
    return sorted(DOCS.rglob("*.md"))


def check_links(paths: list[Path]) -> list[str]:
    bad = []
    for f in paths:
        text = f.read_text(encoding="utf-8")
        for m in LINK_RE.finditer(text):
            target = m.group(1).split("#")[0]
            if not target or target.startswith(("http", "mailto", "<")):
                continue
            resolved = (f.parent / target).resolve()
            if not resolved.exists():
                bad.append(f"{f.relative_to(ROOT)}: 链接目标不存在 -> {target}")
    return bad


def check_index(docs: list[Path]) -> list[str]:
    bad = []
    if not INDEX.exists():
        return [f"{INDEX.relative_to(ROOT)}: 索引文件缺失"]
    index_text = INDEX.read_text(encoding="utf-8")
    for f in docs:
        rel = f.relative_to(DOCS).as_posix()
        if rel.startswith("adr/") or f == INDEX:
            continue
        if rel not in index_text:
            bad.append(f"{f.relative_to(ROOT)}: 未收录进 docs/README.md 索引")
    return bad


def check_status_lines(docs: list[Path]) -> list[str]:
    bad = []
    for f in docs:
        if f.relative_to(DOCS).as_posix().startswith("adr/") or f == INDEX:
            continue
        head = f.read_text(encoding="utf-8").split("\n")[:STATUS_WINDOW]
        if not any(line.startswith("> 状态：") for line in head):
            bad.append(f"{f.relative_to(ROOT)}: 前 {STATUS_WINDOW} 行缺少 '> 状态：' 行")
    return bad


def check_adr_index(docs: list[Path]) -> list[str]:
    bad = []
    if not ADR_INDEX.exists():
        return [f"{ADR_INDEX.relative_to(ROOT)}: ADR 索引缺失"]
    index_text = ADR_INDEX.read_text(encoding="utf-8")
    for f in docs:
        rel = f.relative_to(DOCS).as_posix()
        if not rel.startswith("adr/") or f == ADR_INDEX:
            continue
        if f.name not in index_text:
            bad.append(f"{f.relative_to(ROOT)}: 未收录进 docs/adr/README.md")
    return bad


def main() -> int:
    docs = markdown_files()
    problems = []
    problems += check_links(docs + [p for p in SCAN_FILES if p.exists()])
    problems += check_index(docs)
    problems += check_status_lines(docs)
    problems += check_adr_index(docs)
    if problems:
        print(f"docs 门禁失败（{len(problems)} 项）：")
        for p in problems:
            print("  -", p)
        return 1
    print(f"docs 门禁通过：{len(docs)} 份文档，链接/索引/状态行/ADR 索引全部合规")
    return 0


if __name__ == "__main__":
    sys.exit(main())

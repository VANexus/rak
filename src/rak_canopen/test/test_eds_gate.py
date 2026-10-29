"""EDS ↔ 契约一致性门禁的自测。

这条链的价值在于它**曾经报出 20 条假问题**（我把 TPDO1 的段号算成 1801，于是整条链
读成了下一个 PDO；又把 SubNumber 当成映射数）。那两类错误如果留在门禁里，下一轮没人
会再看它输出的任何一行 —— 所以"门禁自己会错"必须被测试钉住，而不是靠人小心。
"""
from pathlib import Path

import pytest

from rak_canopen import contract, eds_gate

CONFIG = Path(__file__).resolve().parents[1] / "config"


@pytest.mark.parametrize("node", ["chassis", "arm"])
def test_eds_matches_derived_contract(node):
    c = contract.load(CONFIG)
    assert eds_gate.check_node(node, c) == [], f"{node} 存在未解释的分歧"


def test_missing_eds_is_loud(tmp_path, monkeypatch):
    monkeypatch.setattr(eds_gate, "BASE", tmp_path)
    with pytest.raises(eds_gate.EdsError, match="缺少 EDS 副本"):
        eds_gate.check_node("chassis", contract.load(CONFIG))

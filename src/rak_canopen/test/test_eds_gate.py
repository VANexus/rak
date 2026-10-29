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
    #: `Contract.root` 同时是 EDS 的落点：这条断言钉住"目录由装载决定，不由 `__file__` 猜"。
    #: 原先门禁自己用 `Path(__file__).parents[1]/"config"` 推，装进前缀后指到
    #: `site-packages/config/`，本机实测直接红（`eds_dir=None` 这条分支就是那条路）。
    assert c.root == CONFIG
    assert eds_gate.check_node(node, c, eds_dir=None) == [], f"{node} 存在未解释的分歧"


def test_default_config_dir_follows_the_ament_index():
    """安装态（板子档）必须从 share/ 找配置，而不是从源码树猜。"""
    from ament_index_python.packages import get_package_share_directory

    assert contract.default_config_dir() == (
        Path(get_package_share_directory("rak_canopen")) / "config"
    )


def test_missing_eds_is_loud(tmp_path):
    with pytest.raises(eds_gate.EdsError, match="缺少 EDS 副本"):
        eds_gate.check_node("chassis", contract.load(CONFIG), eds_dir=tmp_path)

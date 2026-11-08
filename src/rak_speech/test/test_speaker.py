# -*- coding: utf-8 -*-
"""声纹纯函数测试:归一化/余弦/在线聚类语义。

不依赖 ROS、不加载 onnxruntime——模型在运行时冒烟,这里只锁数学与聚类行为。
运行:python3 -m pytest test_speaker.py(colcon test 同样收集)。
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from rak_speech.speaker_core import (  # noqa: E402
    SpeakerClusterer,
    cosine,
    l2norm,
)


def test_l2norm_unit_length_and_zero_safe():
    v = l2norm(np.array([3.0, 4.0], dtype=np.float32))
    assert np.isclose(np.linalg.norm(v), 1.0, atol=1e-6)
    zero = l2norm(np.zeros(4, dtype=np.float32))
    assert not np.any(np.isnan(zero))  # 零向量不能产出 NaN


def test_cosine_identical_and_orthogonal():
    a = np.array([1.0, 2.0, 3.0], dtype=np.float32)
    assert np.isclose(cosine(a, a), 1.0, atol=1e-6)
    assert np.isclose(cosine(np.array([1.0, 0.0]), np.array([0.0, 1.0])), 0.0, atol=1e-6)
    assert cosine(np.zeros(2), a) == 0.0  # 零向量不炸


# -- 在线聚类(无档案) --------------------------------------------------------
def _voice(direction: int, jitter: float = 0.0, seed: int = 0) -> np.ndarray:
    rng = np.random.default_rng(seed)
    base = np.zeros(16, dtype=np.float32)
    base[direction] = 1.0
    return base + jitter * rng.standard_normal(16).astype(np.float32)


def test_first_voice_opens_s1():
    clusterer = SpeakerClusterer(threshold=0.5)
    label, score, is_new = clusterer.assign(_voice(0))
    assert (label, score, is_new) == ("S1", 1.0, True)
    assert clusterer.labels == ["S1"]


def test_same_voice_keeps_label_and_updates_centroid():
    clusterer = SpeakerClusterer(threshold=0.5)
    clusterer.assign(_voice(0, seed=1))
    for seed in (2, 3, 4):
        label, score, is_new = clusterer.assign(_voice(0, jitter=0.05, seed=seed))
        assert label == "S1" and not is_new and score > 0.9
    assert clusterer.labels == ["S1"]  # 反复出现不会长出新簇


def test_new_voice_opens_s2_and_labels_stay_stable():
    clusterer = SpeakerClusterer(threshold=0.5)
    clusterer.assign(_voice(0))
    label, _, is_new = clusterer.assign(_voice(5))
    assert label == "S2" and is_new
    assert clusterer.assign(_voice(0, jitter=0.03, seed=7))[0] == "S1"  # 回到第一个人
    assert clusterer.labels == ["S1", "S2"]


def test_threshold_gates_same_or_new():
    """同一条边界样本,阈值高则开新簇、阈值低则归入旧簇。"""
    a = _voice(0)
    b = _voice(0, jitter=0.6, seed=9)  # 与 a 相似但有限
    sim = cosine(a, b)
    strict = SpeakerClusterer(threshold=min(0.99, sim + 0.05))
    strict.assign(a)
    assert strict.assign(b)[2] is True
    loose = SpeakerClusterer(threshold=max(0.0, sim - 0.05))
    loose.assign(a)
    assert loose.assign(b)[2] is False


def test_max_speakers_caps_labels():
    """人再多也不让标签爆炸:弱匹配折进最近簇,并如实给出低分。"""
    clusterer = SpeakerClusterer(threshold=0.5, max_speakers=2)
    clusterer.assign(_voice(0))
    clusterer.assign(_voice(5))
    label, score, is_new = clusterer.assign(_voice(10))  # 第三个声音
    assert label in ("S1", "S2") and not is_new and score < 0.5
    assert len(clusterer) == 2


def test_reset_clears_session():
    clusterer = SpeakerClusterer(threshold=0.5)
    clusterer.assign(_voice(0))
    clusterer.assign(_voice(5))
    clusterer.reset()
    assert len(clusterer) == 0
    assert clusterer.assign(_voice(5))[0] == "S1"  # 清零后从头编号

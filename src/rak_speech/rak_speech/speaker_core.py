"""Speaker-identification pure helpers: normalisation, cosine, online clustering.

No ROS, no model imports — the node builds on these and the tests assert the
math without loading onnxruntime.

**No enrolment, no profile files (工程约定)**: speakers are discovered online.
Every speech segment's embedding is matched against in-memory cluster centroids;
a confident match votes for that cluster (centroid updates), a poor match opens
a new one. Labels are session-scoped ("S1", "S2", ...) and reset with the node —
there is deliberately nothing on disk to maintain.
"""

from __future__ import annotations

import numpy as np

#: Cosine-similarity threshold for "same voice". Calibrate on the robot:
#: 2-5 people, far-field array audio. Below it, a new cluster opens.
DEFAULT_THRESHOLD = 0.5
#: Hard cap on clusters per session — beyond this, weak matches just go to the
#: closest cluster (keeps labels bounded; a crowded room is out of scope).
DEFAULT_MAX_SPEAKERS = 8


def l2norm(vec: np.ndarray) -> np.ndarray:
    v = np.asarray(vec, dtype=np.float32)
    n = float(np.linalg.norm(v))
    return v / n if n > 0.0 else v


def cosine(a: np.ndarray, b: np.ndarray) -> float:
    aa = np.asarray(a, dtype=np.float32)
    bb = np.asarray(b, dtype=np.float32)
    na = float(np.linalg.norm(aa))
    nb = float(np.linalg.norm(bb))
    if na == 0.0 or nb == 0.0:
        return 0.0
    return float(np.dot(aa, bb) / (na * nb))


class SpeakerClusterer:
    """Online single-speaker clustering with running-mean centroids.

    One embedding per evaluation tick; the same person keeps landing on the same
    label while their centroid gets a little more evidence each time. No files,
    no roster — cluster state lives exactly as long as this object.
    """

    def __init__(
        self,
        threshold: float = DEFAULT_THRESHOLD,
        max_speakers: int = DEFAULT_MAX_SPEAKERS,
    ) -> None:
        self.threshold = float(threshold)
        self.max_speakers = max(1, int(max_speakers))
        self._centroids: list[np.ndarray] = []
        self._counts: list[int] = []

    def __len__(self) -> int:
        return len(self._centroids)

    @property
    def labels(self) -> list[str]:
        return [f"S{i + 1}" for i in range(len(self._centroids))]

    def assign(self, embedding: np.ndarray) -> tuple[str, float, bool]:
        """Assign one embedding; returns (label, best_score, opened_new_cluster).

        Threshold decides "same voice"; when the cluster cap is reached a new
        voice is folded into its closest cluster (score reports the uncertainty)
        instead of growing labels without bound.
        """
        emb = l2norm(embedding)
        if not self._centroids:
            self._open(emb)
            return "S1", 1.0, True

        scores = [cosine(emb, c) for c in self._centroids]
        best = int(np.argmax(scores))
        best_score = scores[best]

        if best_score >= self.threshold or len(self._centroids) >= self.max_speakers:
            self._vote(best, emb)
            return f"S{best + 1}", best_score, False

        self._open(emb)
        return f"S{len(self._centroids)}", 1.0, True

    def reset(self) -> None:
        self._centroids.clear()
        self._counts.clear()

    # -- internals ----------------------------------------------------------
    def _open(self, emb: np.ndarray) -> None:
        self._centroids.append(emb.copy())
        self._counts.append(1)

    def _vote(self, index: int, emb: np.ndarray) -> None:
        n = self._counts[index]
        self._centroids[index] = l2norm(
            (self._centroids[index] * n + emb) / (n + 1)
        )
        self._counts[index] = n + 1

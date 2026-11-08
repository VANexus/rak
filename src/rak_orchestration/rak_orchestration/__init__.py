"""rak_orchestration —— 组装层契约（ADR-0026）。

这里只有"谁属于哪一档、谁依赖谁、谁在什么条件下算活着"这类**契约**，
不放任何硬件驱动、launch 组合或算法。硬件全栈组合仍在 `rak_bringup`。
"""

from __future__ import annotations

__all__ = ["profiles", "launch_helpers"]

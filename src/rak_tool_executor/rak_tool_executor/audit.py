"""Append-only audit trail for tool calls, in the episode corpus shape.

Same JSONL family as the agent episodes (kind/call_id fields) so a later
merge step can fold tool events into an episode. Never blocks a proposal:
write failures are logged and counted, never raised into the call path.
"""

from __future__ import annotations

import json
import threading
import time
from pathlib import Path
from typing import Any


class Audit:
    def __init__(self, directory: str | Path):
        self._dir = Path(directory)
        self._dir.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()

    def _path(self) -> Path:
        return self._dir / f"tool_events-{time.strftime('%Y%m%d')}.jsonl"

    def event(self, kind: str, **fields: Any) -> None:
        record = {"t": time.time(), "kind": kind, **fields}
        try:
            line = json.dumps(record, ensure_ascii=False)
            with self._lock:
                with self._path().open("a", encoding="utf-8") as fh:
                    fh.write(line + "\n")
        except OSError:
            # audit must never break the tool path; the result still carries detail
            pass


def call_record(tool: str, source: str, requested_by: str, args: Any,
                safety: str, confirmed: bool) -> dict[str, Any]:
    return {
        "tool": tool,
        "source": source,
        "requested_by": requested_by,
        "args": args,
        "safety": safety,
        "operator_confirmed": confirmed,
    }

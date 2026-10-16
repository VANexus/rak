"""Cloud client: independent anthropic path (ADR-0010 D3, stdlib-only here).

- Key only from `ANTHROPIC_API_KEY`; endpoint passed explicitly
  (https://api.deepseek.com/anthropic), never from env-implied drift.
- `anthropic` SDK is lazily imported: absent SDK or key degrades to
  CloudUnavailable and the node falls back to local replies (stub).
- Local heuristic never invents locations: unknown destination -> NO_OP.
"""
from __future__ import annotations

import os
import time
from dataclasses import dataclass

from .contract import (
    KIND_NO_OP,
    KIND_REQUEST_CLARIFICATION,
    KIND_SELECT_WAYPOINT,
)

DEFAULT_BASE_URL = "https://api.deepseek.com/anthropic"
DEFAULT_MODEL = "deepseek-chat"  # TODO(measured): verify against endpoint docs


class CloudUnavailable(RuntimeError):
    pass


@dataclass
class ProposalData:
    kind: int
    waypoint: str
    rationale: str


class HarnessCloud:
    def __init__(
        self,
        *,
        model: str = DEFAULT_MODEL,
        base_url: str = DEFAULT_BASE_URL,
        timeout_sec: float = 20.0,
        max_calls_per_task: int = 3,
    ) -> None:
        self._model = model
        self._base_url = base_url
        self._timeout = timeout_sec
        self._budget = max_calls_per_task
        self._calls: dict[str, int] = {}
        self._client = None

    def _ensure_client(self):
        if self._client is not None:
            return self._client
        api_key = os.environ.get("ANTHROPIC_API_KEY")
        if not api_key:
            raise CloudUnavailable("ANTHROPIC_API_KEY not set")
        try:
            from anthropic import Anthropic
        except ImportError as exc:
            raise CloudUnavailable("anthropic SDK not installed") from exc
        self._client = Anthropic(
            api_key=api_key, base_url=self._base_url,
            timeout=self._timeout, max_retries=1,
        )
        return self._client

    def available(self) -> bool:
        try:
            self._ensure_client()
        except CloudUnavailable:
            return False
        return True

    def propose_local(
        self, *, text: str, destination: str, allowed_waypoints: object,
        task_id: str,
    ) -> ProposalData:
        """Deterministic fallback: only echo a whitelisted alias, never invent."""
        if destination in allowed_waypoints:
            return ProposalData(
                KIND_SELECT_WAYPOINT, destination, f"本地直认白名单别名:{destination}")
        return ProposalData(
            KIND_NO_OP, "", f"未知地点:{destination}，本地不猜（原文:{text})")

    def propose(
        self, *, text: str, destination: str, allowed_waypoints: object,
        context_text: str, task_id: str, expires_epoch_sec: float,
    ) -> ProposalData:
        """Cloud tool-use path with per-task budget; expired requests never retry."""
        if time.time() > expires_epoch_sec:
            raise CloudUnavailable("request already expired, no retry")
        used = self._calls.get(task_id, 0)
        if used >= self._budget:
            return ProposalData(KIND_NO_OP, "", "本任务云调用预算耗尽")
        client = self._ensure_client()
        self._calls[task_id] = used + 1
        tool_schema = {
            "name": "propose_plan",
            "description": "Choose a whitelisted waypoint or ask for clarification.",
            "input_schema": {
                "type": "object",
                "properties": {
                    "kind": {"type": "string",
                             "enum": ["SELECT_WAYPOINT", "REQUEST_CLARIFICATION"]},
                    "waypoint": {"type": "string",
                                 "enum": sorted(set(allowed_waypoints))},
                    "rationale": {"type": "string"},
                },
                "required": ["kind", "rationale"],
            },
        }
        message = client.messages.create(
            model=self._model,
            max_tokens=256,
            tools=[tool_schema],
            tool_choice={"type": "tool", "name": "propose_plan"},
            system=("你是具身机器人任务仲裁助手，只输出结构化提案，"
                    f"绝不输出速度/关节命令。车况：{context_text}"),
            messages=[{"role": "user",
                       "content": f"任务文本：{text}；候选地点：{sorted(set(allowed_waypoints))}"}],
        )
        for block in message.content:
            if getattr(block, "type", "") == "tool_use" and block.name == "propose_plan":
                args = block.input or {}
                if args.get("kind") == "SELECT_WAYPOINT" and args.get("waypoint") in allowed_waypoints:
                    return ProposalData(KIND_SELECT_WAYPOINT, args["waypoint"],
                                        str(args.get("rationale", ""))[:200])
                return ProposalData(KIND_REQUEST_CLARIFICATION, "",
                                    str(args.get("rationale", "请澄清目标地点"))[:200])
        return ProposalData(KIND_NO_OP, "", "云端未返回可用提案")

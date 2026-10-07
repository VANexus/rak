"""Versioned task-intent contract kept separate from frozen episode types."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
import re
from typing import Mapping, Protocol, Sequence


class ControlCommand(str, Enum):
    PAUSE = "pause"
    RESUME = "resume"
    CANCEL = "cancel"


@dataclass(frozen=True)
class PoseProposal:
    frame_id: str
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


@dataclass(frozen=True)
class NavigationIntent:
    destination: str
    source_text: str
    schema_version: int = 1


@dataclass(frozen=True)
class WaypointSelection:
    waypoint: str
    rationale: str = ""
    schema_version: int = 1


class SlowTaskPlanner(Protocol):
    """Optional slow-path adapter; it returns an allow-listed waypoint only."""

    def propose(
        self,
        text: str,
        *,
        task_id: str,
        revision: int,
        allowed_waypoints: Sequence[str],
    ) -> WaypointSelection:
        ...


_CONTROL_PHRASES = {
    "暂停": ControlCommand.PAUSE,
    "停一下": ControlCommand.PAUSE,
    "pause": ControlCommand.PAUSE,
    "暂停任务": ControlCommand.PAUSE,
    "继续": ControlCommand.RESUME,
    "恢复": ControlCommand.RESUME,
    "resume": ControlCommand.RESUME,
    "取消": ControlCommand.CANCEL,
    "取消任务": ControlCommand.CANCEL,
    "cancel": ControlCommand.CANCEL,
}
_NAV_PREFIX = re.compile(
    r"^(?:请)?(?:带我)?(?:去|前往|到)\s*(.+?)\s*[。.!！]?$|"
    r"^(?:please\s+)?(?:go|navigate|move)\s+to\s+(.+?)\s*[.!?]?$",
    re.IGNORECASE,
)


def parse_control_command(text: str) -> ControlCommand | None:
    return _CONTROL_PHRASES.get(text.strip().lower())


def parse_navigation_intent(text: str) -> NavigationIntent | None:
    match = _NAV_PREFIX.match(text.strip())
    if not match:
        return None
    destination = next((part for part in match.groups() if part), "").strip()
    if not destination or len(destination) > 128:
        return None
    return NavigationIntent(destination=destination, source_text=text)


def validate_pose_proposal(proposal: PoseProposal) -> None:
    if proposal.frame_id != "map":
        raise ValueError("navigation proposals must use the map frame")
    values = (proposal.x, proposal.y, proposal.z, proposal.qx, proposal.qy, proposal.qz, proposal.qw)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("pose proposal contains a non-finite value")
    q_norm = math.sqrt(proposal.qx**2 + proposal.qy**2 + proposal.qz**2 + proposal.qw**2)
    if not 0.99 <= q_norm <= 1.01:
        raise ValueError("pose proposal quaternion must be normalized")


def resolve_waypoint(name: str, waypoints: Mapping[str, PoseProposal]) -> PoseProposal:
    proposal = waypoints.get(name)
    if proposal is None:
        raise ValueError(f"unknown waypoint {name!r}; configure it before use")
    validate_pose_proposal(proposal)
    return proposal


class RaiWaypointPlanner:
    """Optional structured RAI/LLM path restricted to configured waypoint names."""

    def __init__(
        self,
        *,
        config_path: str,
        vendor: str | None = None,
        model_type: str = "complex_model",
    ) -> None:
        from rai import get_llm_model
        from pydantic import BaseModel, Field

        class StructuredWaypointSelection(BaseModel):
            schema_version: int = 1
            waypoint: str = Field(min_length=1, max_length=128)
            rationale: str = Field(default="", max_length=512)

        self._chain = get_llm_model(
            model_type, vendor=vendor, config_path=config_path
        ).with_structured_output(StructuredWaypointSelection)

    def propose(
        self,
        text: str,
        *,
        task_id: str,
        revision: int,
        allowed_waypoints: Sequence[str],
    ) -> WaypointSelection:
        allowed = sorted(set(allowed_waypoints))
        if not allowed:
            raise ValueError("no configured waypoints are available to the slow planner")
        structured = self._chain.invoke(
            "Select exactly one destination alias from the allowed list. "
            "Return no coordinates and do not request tools or issue motion commands. "
            f"Task {task_id}, revision {revision}. "
            f"Allowed waypoint aliases: {allowed}. User request: {text}"
        )
        proposal = WaypointSelection(
            waypoint=structured.waypoint,
            rationale=structured.rationale,
            schema_version=structured.schema_version,
        )
        if proposal.schema_version != 1 or proposal.waypoint not in allowed:
            raise ValueError("slow planner returned an unknown waypoint or schema")
        return proposal

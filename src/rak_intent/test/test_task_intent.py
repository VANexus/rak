#!/usr/bin/env python3
"""Small standalone tests for the versioned task-intent contract."""

from rak_intent.task_intent import (
    ControlCommand,
    PoseProposal,
    parse_control_command,
    parse_navigation_intent,
    resolve_waypoint,
    validate_pose_proposal,
)


def main():
    assert parse_control_command("暂停") is ControlCommand.PAUSE
    assert parse_control_command("resume") is ControlCommand.RESUME
    assert parse_control_command("不暂停") is None
    assert parse_navigation_intent("请带我去实验室").destination == "实验室"
    assert parse_navigation_intent("Navigate to charging area").destination == "charging area"
    assert parse_navigation_intent("帮我找东西") is None

    pose = PoseProposal("map", 1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    assert resolve_waypoint("lab", {"lab": pose}) == pose
    try:
        resolve_waypoint("unknown", {})
    except ValueError:
        pass
    else:
        raise AssertionError("unknown waypoint was accepted")

    invalid = PoseProposal("odom", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    try:
        validate_pose_proposal(invalid)
    except ValueError:
        pass
    else:
        raise AssertionError("non-map proposal was accepted")
    print("task intent tests passed")


if __name__ == "__main__":
    main()

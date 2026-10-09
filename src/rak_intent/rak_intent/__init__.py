"""Shared text-to-intent contract (ADR-0013).

One source of truth for `rak_task_executor` and `rak_speech`: a spoken
command and a typed one cannot diverge. Import time is stdlib-only; the
optional slow-path planner lazy-imports rai/pydantic.
"""

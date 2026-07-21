"""Deterministic state machine for semantic trajectory holds."""

from __future__ import annotations


class SemanticTrajectoryGuard:
    """Track active path intrusions without coupling policy to ROS callbacks."""

    def __init__(self, clear_dwell_sec: float = 1.5, hold_timeout_sec: float = 30.0):
        self.clear_dwell_sec = max(0.0, float(clear_dwell_sec))
        self.hold_timeout_sec = max(0.1, float(hold_timeout_sec))
        self.active_object_ids: set[str] = set()
        self.hold_started_at: float | None = None
        self.clear_started_at: float | None = None

    def update_event(
        self,
        event_type: str,
        active: bool,
        object_ids: list[str],
        now: float,
    ) -> None:
        ids = {str(value) for value in object_ids if str(value)}
        if event_type == "person_entered_path" and active:
            self.active_object_ids.update(ids)
            if self.hold_started_at is None:
                self.hold_started_at = now
            self.clear_started_at = None
        elif event_type == "person_cleared_path" and not active:
            self.active_object_ids.difference_update(ids)
            if not self.active_object_ids and self.hold_started_at is not None:
                self.clear_started_at = now

    def decision(self, now: float) -> str:
        if self.hold_started_at is None:
            return "tracking"
        if now - self.hold_started_at >= self.hold_timeout_sec:
            return "timeout"
        if self.active_object_ids:
            return "hold"
        if self.clear_started_at is None:
            self.clear_started_at = now
        if now - self.clear_started_at < self.clear_dwell_sec:
            return "clearing"
        return "resume"

    def begin_mission(self, now: float) -> None:
        self.hold_started_at = now if self.active_object_ids else None
        self.clear_started_at = None

    def consume_resume(self, now: float) -> float:
        duration = 0.0
        if self.hold_started_at is not None:
            duration = max(0.0, now - self.hold_started_at)
        self.hold_started_at = None
        self.clear_started_at = None
        return duration

    def end_mission(self) -> None:
        self.hold_started_at = None
        self.clear_started_at = None

"""Monotonic mission lifecycle shared by simulation and real execution."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
from enum import Enum
from uuid import UUID


class MissionState(str, Enum):
    CREATED = "created"
    VALIDATED = "validated"
    ACCEPTED = "accepted"
    RUNNING = "running"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"
    EXPIRED = "expired"


TERMINAL_STATES = {
    MissionState.COMPLETED,
    MissionState.FAILED,
    MissionState.CANCELLED,
    MissionState.EXPIRED,
}

_ALLOWED_TRANSITIONS = {
    MissionState.CREATED: {
        MissionState.VALIDATED,
        MissionState.FAILED,
        MissionState.CANCELLED,
        MissionState.EXPIRED,
    },
    MissionState.VALIDATED: {
        MissionState.ACCEPTED,
        MissionState.FAILED,
        MissionState.CANCELLED,
        MissionState.EXPIRED,
    },
    MissionState.ACCEPTED: {
        MissionState.RUNNING,
        MissionState.FAILED,
        MissionState.CANCELLED,
        MissionState.EXPIRED,
    },
    MissionState.RUNNING: {
        MissionState.COMPLETED,
        MissionState.FAILED,
        MissionState.CANCELLED,
        MissionState.EXPIRED,
    },
}


@dataclass(frozen=True)
class MissionTransition:
    mission_id: UUID
    previous: MissionState
    current: MissionState
    occurred_at_utc: datetime
    reason: str


class MissionTransitionError(ValueError):
    pass


class MissionStateMachine:
    def __init__(self, mission_id: UUID) -> None:
        self.mission_id = mission_id
        self._state = MissionState.CREATED
        self._history: list[MissionTransition] = []

    @property
    def state(self) -> MissionState:
        return self._state

    @property
    def terminal(self) -> bool:
        return self._state in TERMINAL_STATES

    @property
    def history(self) -> tuple[MissionTransition, ...]:
        return tuple(self._history)

    def transition(
        self,
        target: MissionState,
        *,
        reason: str,
        occurred_at_utc: datetime | None = None,
    ) -> MissionTransition | None:
        if target == self._state:
            return None
        allowed = _ALLOWED_TRANSITIONS.get(self._state, set())
        if target not in allowed:
            raise MissionTransitionError(
                f"invalid mission transition {self._state.value} -> {target.value}"
            )

        occurred_at = occurred_at_utc or datetime.now(timezone.utc)
        if occurred_at.tzinfo is None or occurred_at.utcoffset() is None:
            raise MissionTransitionError("transition time must include a timezone")
        occurred_at = occurred_at.astimezone(timezone.utc)

        transition = MissionTransition(
            mission_id=self.mission_id,
            previous=self._state,
            current=target,
            occurred_at_utc=occurred_at,
            reason=reason,
        )
        self._state = target
        self._history.append(transition)
        return transition

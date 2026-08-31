"""Shared monotonic, UTC wall-clock and simulator time stamps."""

from __future__ import annotations

import time
from collections.abc import Callable
from datetime import datetime, timezone

from pydantic import ConfigDict, Field, field_validator, model_validator

from .contracts.models import StrictModel


def utc_now() -> datetime:
    return datetime.now(timezone.utc)


def _as_utc(value: datetime) -> datetime:
    if value.tzinfo is None or value.utcoffset() is None:
        raise ValueError("wall_time must include a timezone")
    return value.astimezone(timezone.utc)


class ClockStamp(StrictModel):
    """One observation of the process, wall and simulator clocks."""

    model_config = ConfigDict(extra="forbid", frozen=True, allow_inf_nan=False)

    monotonic_s: float = Field(ge=0.0)
    wall_time: datetime
    sim_time_s: float | None = Field(default=None, ge=0.0)

    _wall_time_is_utc = field_validator("wall_time")(_as_utc)


class DualClockStamp(StrictModel):
    """Receive-side and mirror-side stamps for one state or evidence item."""

    model_config = ConfigDict(extra="forbid", frozen=True, allow_inf_nan=False)

    received: ClockStamp
    mirrored: ClockStamp

    @model_validator(mode="after")
    def mirror_follows_receive(self) -> "DualClockStamp":
        if self.mirrored.monotonic_s < self.received.monotonic_s:
            raise ValueError("mirrored monotonic time must not precede receive time")
        if self.mirrored.wall_time < self.received.wall_time:
            raise ValueError("mirrored wall time must not precede receive time")
        if (
            self.received.sim_time_s is not None
            and self.mirrored.sim_time_s is not None
            and self.mirrored.sim_time_s < self.received.sim_time_s
        ):
            raise ValueError("mirrored simulator time must not move backwards")
        return self


class MonotonicClock:
    """Injectable clock sampler that rejects a backwards process clock."""

    def __init__(
        self,
        *,
        monotonic: Callable[[], float] = time.monotonic,
        wall_clock: Callable[[], datetime] = utc_now,
    ) -> None:
        self._monotonic = monotonic
        self._wall_clock = wall_clock
        self._last_monotonic_s: float | None = None

    def stamp(self, *, sim_time_s: float | None = None) -> ClockStamp:
        monotonic_s = float(self._monotonic())
        if (
            self._last_monotonic_s is not None
            and monotonic_s < self._last_monotonic_s
        ):
            raise RuntimeError("monotonic clock moved backwards")
        stamp = ClockStamp(
            monotonic_s=monotonic_s,
            wall_time=self._wall_clock(),
            sim_time_s=sim_time_s,
        )
        self._last_monotonic_s = stamp.monotonic_s
        return stamp


def capture_clock(*, sim_time_s: float | None = None) -> ClockStamp:
    """Capture all three clocks for stateless call sites."""

    return ClockStamp(
        monotonic_s=time.monotonic(),
        wall_time=utc_now(),
        sim_time_s=sim_time_s,
    )


__all__ = [
    "ClockStamp",
    "DualClockStamp",
    "MonotonicClock",
    "capture_clock",
    "utc_now",
]

"""High-level ArduCopter command service built on the single-owner link."""

from __future__ import annotations

import asyncio
from dataclasses import dataclass

from .apm_link import ApmLink, CommandHandle
from .mavlink.models import CommandResult, CommandStatus, FcuAction, FcuCommand


@dataclass(frozen=True, slots=True)
class CommandSequenceResult:
    name: str
    completed: bool
    steps: tuple[CommandResult, ...]
    detail: str


class FlightCommandService:
    def __init__(self, link: ApmLink) -> None:
        self._link = link
        self._last_compensation_handle: CommandHandle | None = None

    @property
    def last_compensation_handle(self) -> CommandHandle | None:
        return self._last_compensation_handle

    def arm(self, *, expires_monotonic_s: float | None = None) -> CommandHandle:
        return self._link.submit(
            FcuCommand(FcuAction.ARM),
            expires_monotonic_s=expires_monotonic_s,
        )

    def disarm(self, *, expires_monotonic_s: float | None = None) -> CommandHandle:
        return self._link.submit(
            FcuCommand(FcuAction.DISARM),
            expires_monotonic_s=expires_monotonic_s,
        )

    def set_mode(
        self,
        mode: str,
        *,
        expires_monotonic_s: float | None = None,
    ) -> CommandHandle:
        return self._link.submit(
            FcuCommand(FcuAction.SET_MODE, mode=mode.upper()),
            expires_monotonic_s=expires_monotonic_s,
        )

    def takeoff(
        self,
        altitude_m: float,
        *,
        expires_monotonic_s: float | None = None,
    ) -> CommandHandle:
        return self._link.submit(
            FcuCommand(FcuAction.TAKEOFF, target_altitude_m=altitude_m),
            expires_monotonic_s=expires_monotonic_s,
        )

    def goto_local_ned(
        self,
        north_m: float,
        east_m: float,
        down_m: float,
        *,
        yaw_rad: float | None = None,
        expires_monotonic_s: float | None = None,
    ) -> CommandHandle:
        return self._link.submit(
            FcuCommand(
                FcuAction.GOTO_LOCAL_NED,
                target_position_ned_m=(north_m, east_m, down_m),
                yaw_rad=yaw_rad,
            ),
            expires_monotonic_s=expires_monotonic_s,
        )

    def hold(self, *, expires_monotonic_s: float | None = None) -> CommandHandle:
        return self._link.submit(
            FcuCommand(FcuAction.HOLD),
            expires_monotonic_s=expires_monotonic_s,
        )

    def land(self, *, expires_monotonic_s: float | None = None) -> CommandHandle:
        return self._link.submit(
            FcuCommand(FcuAction.LAND),
            expires_monotonic_s=expires_monotonic_s,
        )

    def rtl(self, *, expires_monotonic_s: float | None = None) -> CommandHandle:
        return self._link.submit(
            FcuCommand(FcuAction.RTL),
            expires_monotonic_s=expires_monotonic_s,
        )

    async def arm_and_takeoff(
        self,
        altitude_m: float,
        *,
        expires_monotonic_s: float | None = None,
    ) -> CommandSequenceResult:
        return await self._run_sequence(
            "arm_and_takeoff",
            (
                lambda: self.set_mode(
                    self._link.guided_mode,
                    expires_monotonic_s=expires_monotonic_s,
                ),
                lambda: self.arm(expires_monotonic_s=expires_monotonic_s),
                lambda: self.takeoff(
                    altitude_m,
                    expires_monotonic_s=expires_monotonic_s,
                ),
            ),
            on_cancel=self.land,
        )

    async def guided_goto_local_ned(
        self,
        north_m: float,
        east_m: float,
        down_m: float,
        *,
        yaw_rad: float | None = None,
        expires_monotonic_s: float | None = None,
    ) -> CommandSequenceResult:
        return await self._run_sequence(
            "guided_goto_local_ned",
            (
                lambda: self.set_mode(
                    self._link.guided_mode,
                    expires_monotonic_s=expires_monotonic_s,
                ),
                lambda: self.goto_local_ned(
                    north_m,
                    east_m,
                    down_m,
                    yaw_rad=yaw_rad,
                    expires_monotonic_s=expires_monotonic_s,
                ),
            ),
            on_cancel=self.hold,
        )

    async def _run_sequence(self, name: str, steps, *, on_cancel) -> CommandSequenceResult:
        results: list[CommandResult] = []
        try:
            for create_handle in steps:
                result = await create_handle().result
                results.append(result)
                if result.status != CommandStatus.COMPLETED:
                    return CommandSequenceResult(
                        name=name,
                        completed=False,
                        steps=tuple(results),
                        detail=f"stopped after {result.action.value}: {result.status.value}",
                    )
        except asyncio.CancelledError:
            self._last_compensation_handle = on_cancel()
            raise
        return CommandSequenceResult(
            name=name,
            completed=True,
            steps=tuple(results),
            detail="all command steps completed",
        )

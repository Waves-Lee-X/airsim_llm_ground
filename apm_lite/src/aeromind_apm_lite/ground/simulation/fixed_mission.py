"""Evidence-preserving M1 fixed mission runner."""

from __future__ import annotations

import asyncio
import json
import os
import time
from collections.abc import Awaitable, Callable
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any
from uuid import UUID, uuid4

from aeromind_apm_lite.onboard.apm_link import ApmLink, CommandHandle
from aeromind_apm_lite.onboard.flight_commands import FlightCommandService
from aeromind_apm_lite.onboard.mavlink.models import CommandResult, CommandStatus


class MissionTerminalState(str, Enum):
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"


class MissionBatchState(str, Enum):
    COMPLETED = "completed"
    PARTIAL = "partial"
    FAILED = "failed"


class MissionEvidenceJournalError(RuntimeError):
    """Raised after a terminal result exists but cannot be persisted."""


@dataclass(frozen=True, slots=True)
class MissionStepEvidence:
    name: str
    result: CommandResult


@dataclass(frozen=True, slots=True)
class FixedMissionResult:
    run_id: UUID
    attempt: int
    terminal_state: MissionTerminalState
    started_monotonic_s: float
    finished_monotonic_s: float
    steps: tuple[MissionStepEvidence, ...]
    recovery_steps: tuple[MissionStepEvidence, ...]
    detail: str

    @property
    def completed(self) -> bool:
        return self.terminal_state == MissionTerminalState.COMPLETED


@dataclass(frozen=True, slots=True)
class FixedMissionBatchResult:
    batch_id: UUID
    terminal_state: MissionBatchState
    requested_runs: int
    runs: tuple[FixedMissionResult, ...]

    @property
    def completed_runs(self) -> int:
        return sum(result.completed for result in self.runs)

    @property
    def success_rate(self) -> float:
        return self.completed_runs / len(self.runs)


def _ack_dict(result: CommandResult) -> dict[str, Any]:
    ack = result.mavlink_ack
    return {
        "applicable": ack.applicable,
        "received": ack.received,
        "command_id": ack.command_id,
        "result": ack.result,
        "observed_monotonic_s": ack.observed_monotonic_s,
        "detail": ack.detail,
    }


def command_result_dict(result: CommandResult) -> dict[str, Any]:
    """Return all three FCU evidence layers as JSON-compatible data."""

    return {
        "request_id": str(result.request_id),
        "action": result.action.value,
        "status": result.status.value,
        "application": {
            "accepted": result.application.accepted,
            "observed_monotonic_s": result.application.observed_monotonic_s,
            "detail": result.application.detail,
        },
        "sent_monotonic_s": result.sent_monotonic_s,
        "mavlink_ack": _ack_dict(result),
        "physical_completion": {
            "confirmed": result.physical_completion.confirmed,
            "observed_monotonic_s": (
                result.physical_completion.observed_monotonic_s
            ),
            "detail": result.physical_completion.detail,
        },
        "detail": result.detail,
    }


def mission_result_dict(result: FixedMissionResult) -> dict[str, Any]:
    return {
        "event": "fixed_mission_result",
        "run_id": str(result.run_id),
        "attempt": result.attempt,
        "terminal_state": result.terminal_state.value,
        "started_monotonic_s": result.started_monotonic_s,
        "finished_monotonic_s": result.finished_monotonic_s,
        "steps": [
            {"name": step.name, "result": command_result_dict(step.result)}
            for step in result.steps
        ],
        "recovery_steps": [
            {"name": step.name, "result": command_result_dict(step.result)}
            for step in result.recovery_steps
        ],
        "detail": result.detail,
    }


class MissionEvidenceJournal:
    """Durable JSONL evidence output, one complete terminal record per run."""

    def __init__(self, path: Path) -> None:
        path = Path(path)
        if not path.is_absolute():
            raise ValueError("mission evidence journal path must be absolute")
        self._path = path

    @property
    def path(self) -> Path:
        return self._path

    def append(self, result: FixedMissionResult) -> None:
        self._path.parent.mkdir(parents=True, exist_ok=True)
        encoded = (
            json.dumps(
                mission_result_dict(result),
                ensure_ascii=True,
                allow_nan=False,
                separators=(",", ":"),
            )
            + "\n"
        )
        with self._path.open("a", encoding="utf-8", newline="\n") as stream:
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())


class FixedMissionRunner:
    """Run a repeatable out-and-back LOCAL_NED mission with physical evidence."""

    def __init__(
        self,
        link: ApmLink,
        *,
        altitude_m: float = 2.0,
        forward_north_m: float = 3.0,
        journal: MissionEvidenceJournal | None = None,
        before_arm: Callable[[], Awaitable[None]] | None = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if not 0.5 <= altitude_m <= 10.0:
            raise ValueError("fixed mission altitude must be in [0.5, 10] metres")
        if not 0.5 <= forward_north_m <= 20.0:
            raise ValueError("fixed mission forward distance must be in [0.5, 20] metres")
        self._link = link
        self._service = FlightCommandService(link)
        self._altitude_m = float(altitude_m)
        self._forward_north_m = float(forward_north_m)
        self._journal = journal
        self._before_arm = before_arm
        self._clock = clock
        self._lock = asyncio.Lock()
        self._last_result: FixedMissionResult | None = None
        self._last_journal_error: MissionEvidenceJournalError | None = None

    @property
    def last_result(self) -> FixedMissionResult | None:
        return self._last_result

    @property
    def last_journal_error(self) -> MissionEvidenceJournalError | None:
        return self._last_journal_error

    async def run_once(self, *, attempt: int = 1) -> FixedMissionResult:
        if (
            isinstance(attempt, bool)
            or not isinstance(attempt, int)
            or attempt < 1
        ):
            raise ValueError("mission attempt must be a positive integer")
        async with self._lock:
            return await self._run_locked(attempt)

    async def run_repeated(
        self,
        repetitions: int,
        *,
        stop_on_failure: bool = False,
        inter_run_delay_s: float = 0.0,
    ) -> FixedMissionBatchResult:
        if isinstance(repetitions, bool) or not 1 <= repetitions <= 100:
            raise ValueError("repetitions must be an integer in [1, 100]")
        if not 0.0 <= inter_run_delay_s <= 60.0:
            raise ValueError("inter_run_delay_s must be in [0, 60]")

        batch_id = uuid4()
        runs: list[FixedMissionResult] = []
        async with self._lock:
            for attempt in range(1, repetitions + 1):
                result = await self._run_locked(attempt)
                runs.append(result)
                if stop_on_failure and not result.completed:
                    break
                if attempt < repetitions and inter_run_delay_s:
                    await asyncio.sleep(inter_run_delay_s)

        completed = sum(result.completed for result in runs)
        if completed == repetitions:
            terminal_state = MissionBatchState.COMPLETED
        elif completed:
            terminal_state = MissionBatchState.PARTIAL
        else:
            terminal_state = MissionBatchState.FAILED
        return FixedMissionBatchResult(
            batch_id=batch_id,
            terminal_state=terminal_state,
            requested_runs=repetitions,
            runs=tuple(runs),
        )

    async def _run_locked(self, attempt: int) -> FixedMissionResult:
        run_id = uuid4()
        started = self._clock()
        steps: list[MissionStepEvidence] = []
        arm_attempted = False
        start_position = self._link.telemetry_snapshot().local_position_ned_m
        if start_position is None:
            return self._finish(
                run_id,
                attempt,
                started,
                MissionTerminalState.FAILED,
                steps,
                (),
                "fresh LOCAL_POSITION_NED is required before the mission",
            )

        def goto_forward_from_start() -> CommandHandle:
            return self._service.goto_local_ned(
                start_position[0] + self._forward_north_m,
                start_position[1],
                -self._altitude_m,
            )

        def goto_start() -> CommandHandle:
            return self._service.goto_local_ned(
                start_position[0],
                start_position[1],
                -self._altitude_m,
            )

        sequence: tuple[tuple[str, Callable[[], CommandHandle]], ...] = (
            ("guided", lambda: self._service.set_mode(self._link.guided_mode)),
            ("arm", self._service.arm),
            ("takeoff_2m", lambda: self._service.takeoff(self._altitude_m)),
            (
                "forward_3m_local_ned",
                goto_forward_from_start,
            ),
            ("hold", self._service.hold),
            ("guided_return", lambda: self._service.set_mode(self._link.guided_mode)),
            ("return_to_start_local_ned", goto_start),
            ("land", self._service.land),
        )

        try:
            for name, create_handle in sequence:
                if name == "arm":
                    if self._before_arm is not None:
                        await self._before_arm()
                    arm_attempted = True
                result = await create_handle().result
                steps.append(MissionStepEvidence(name, result))
                if result.status != CommandStatus.COMPLETED:
                    recovery, recovery_detail = await self._recover_land(
                        enabled=arm_attempted and name != "land"
                    )
                    return self._finish(
                        run_id,
                        attempt,
                        started,
                        MissionTerminalState.FAILED,
                        steps,
                        recovery,
                        self._with_recovery_detail(
                            f"stopped after {name}: {result.status.value}",
                            recovery_detail,
                        ),
                    )
        except MissionEvidenceJournalError:
            raise
        except asyncio.CancelledError as cancellation:
            recovery, recovery_detail = await self._recover_land(
                enabled=arm_attempted
            )
            try:
                self._finish(
                    run_id,
                    attempt,
                    started,
                    MissionTerminalState.CANCELLED,
                    steps,
                    recovery,
                    self._with_recovery_detail(
                        "mission waiter cancelled; LAND recovery submitted",
                        recovery_detail,
                    ),
                )
            except MissionEvidenceJournalError:
                pass
            raise cancellation
        except Exception as exc:
            recovery, recovery_detail = await self._recover_land(
                enabled=arm_attempted
            )
            return self._finish(
                run_id,
                attempt,
                started,
                MissionTerminalState.FAILED,
                steps,
                recovery,
                self._with_recovery_detail(
                    f"runner exception: {type(exc).__name__}: {exc}",
                    recovery_detail,
                ),
            )

        return self._finish(
            run_id,
            attempt,
            started,
            MissionTerminalState.COMPLETED,
            steps,
            (),
            "all eight FCU commands completed with physical evidence",
        )

    async def _recover_land(
        self,
        *,
        enabled: bool,
    ) -> tuple[tuple[MissionStepEvidence, ...], str | None]:
        if not enabled:
            return (), None
        try:
            result = await self._service.land().result
        except Exception as exc:
            return (), f"LAND recovery raised {type(exc).__name__}: {exc}"
        evidence = (MissionStepEvidence("recovery_land", result),)
        if result.status != CommandStatus.COMPLETED:
            return evidence, f"LAND recovery ended {result.status.value}"
        return evidence, None

    @staticmethod
    def _with_recovery_detail(detail: str, recovery_detail: str | None) -> str:
        if recovery_detail is None:
            return detail
        return f"{detail}; {recovery_detail}"

    def _finish(
        self,
        run_id: UUID,
        attempt: int,
        started: float,
        terminal_state: MissionTerminalState,
        steps: list[MissionStepEvidence],
        recovery: tuple[MissionStepEvidence, ...],
        detail: str,
    ) -> FixedMissionResult:
        result = FixedMissionResult(
            run_id=run_id,
            attempt=attempt,
            terminal_state=terminal_state,
            started_monotonic_s=started,
            finished_monotonic_s=self._clock(),
            steps=tuple(steps),
            recovery_steps=recovery,
            detail=detail,
        )
        self._last_result = result
        if self._journal is not None:
            try:
                self._journal.append(result)
            except OSError as exc:
                journal_error = MissionEvidenceJournalError(
                    f"terminal mission result could not be persisted: {exc}"
                )
                self._last_journal_error = journal_error
                raise journal_error from exc
            else:
                self._last_journal_error = None
        return result

import asyncio
import json
from collections import defaultdict
from pathlib import Path
from uuid import uuid4

import pytest

from aeromind_apm_lite.ground.simulation import (
    FixedMissionRunner,
    MissionBatchState,
    MissionEvidenceJournal,
    MissionEvidenceJournalError,
    MissionTerminalState,
    command_result_dict,
)
from aeromind_apm_lite.onboard.mavlink import (
    ApplicationAcceptance,
    CommandResult,
    CommandStatus,
    FcuAction,
    MavlinkAckEvidence,
    PhysicalCompletionEvidence,
)


def completed_result(action, status=CommandStatus.COMPLETED):
    confirmed = status == CommandStatus.COMPLETED
    return CommandResult(
        request_id=uuid4(),
        action=action,
        status=status,
        application=ApplicationAcceptance(True, 1.0, "accepted"),
        sent_monotonic_s=1.1,
        mavlink_ack=MavlinkAckEvidence(
            applicable=action != FcuAction.GOTO_LOCAL_NED,
            received=action != FcuAction.GOTO_LOCAL_NED,
            command_id=400,
            result=0,
            observed_monotonic_s=1.2,
            detail="ACK accepted",
        ),
        physical_completion=PhysicalCompletionEvidence(
            confirmed=confirmed,
            observed_monotonic_s=1.3 if confirmed else None,
            detail="physical result",
        ),
        detail=status.value,
    )


class ImmediateHandle:
    def __init__(self, result):
        loop = asyncio.get_running_loop()
        self._result = loop.create_future()
        self._result.set_result(result)

    @property
    def result(self):
        return asyncio.shield(self._result)


class ScriptedLink:
    guided_mode = "GUIDED"

    def __init__(self, policy=None):
        self.commands = []
        self.counts = defaultdict(int)
        self.local_position_ned_m = (0.0, 0.0, -2.0)
        self.policy = policy or (lambda _command, _count: CommandStatus.COMPLETED)

    def telemetry_snapshot(self):
        class Snapshot:
            pass

        snapshot = Snapshot()
        snapshot.local_position_ned_m = self.local_position_ned_m
        return snapshot

    def submit(self, command, **_kwargs):
        self.commands.append(command)
        self.counts[command.action] += 1
        status = self.policy(command, self.counts[command.action])
        if isinstance(status, Exception):
            raise status
        if (
            command.action == FcuAction.GOTO_LOCAL_NED
            and status == CommandStatus.COMPLETED
        ):
            self.local_position_ned_m = command.target_position_ned_m
        return ImmediateHandle(completed_result(command.action, status))


def test_fixed_mission_runs_exact_out_and_back_commands_in_order():
    async def scenario():
        link = ScriptedLink()
        result = await FixedMissionRunner(link).run_once()

        assert result.terminal_state == MissionTerminalState.COMPLETED
        assert [command.action for command in link.commands] == [
            FcuAction.SET_MODE,
            FcuAction.ARM,
            FcuAction.TAKEOFF,
            FcuAction.GOTO_LOCAL_NED,
            FcuAction.HOLD,
            FcuAction.SET_MODE,
            FcuAction.GOTO_LOCAL_NED,
            FcuAction.LAND,
        ]
        assert link.commands[0].mode == "GUIDED"
        assert link.commands[2].target_altitude_m == 2.0
        assert link.commands[3].target_position_ned_m == (3.0, 0.0, -2.0)
        assert link.commands[6].target_position_ned_m == (0.0, 0.0, -2.0)
        assert len(result.steps) == 8
        assert all(step.result.application.accepted for step in result.steps)
        assert all(step.result.physical_completion.confirmed for step in result.steps)
        assert result.recovery_steps == ()

    asyncio.run(scenario())


def test_before_arm_gate_runs_after_guided_and_before_arm():
    async def scenario():
        link = ScriptedLink()
        observed_actions = []

        async def before_arm():
            observed_actions.extend(command.action for command in link.commands)

        result = await FixedMissionRunner(
            link,
            before_arm=before_arm,
        ).run_once()

        assert result.terminal_state == MissionTerminalState.COMPLETED
        assert observed_actions == [FcuAction.SET_MODE]

    asyncio.run(scenario())


def test_before_arm_gate_failure_never_sends_arm_or_recovery_land():
    async def scenario():
        link = ScriptedLink()

        async def before_arm():
            raise RuntimeError("GPS health did not remain stable")

        result = await FixedMissionRunner(
            link,
            before_arm=before_arm,
        ).run_once()

        assert result.terminal_state == MissionTerminalState.FAILED
        assert "GPS health did not remain stable" in result.detail
        assert [command.action for command in link.commands] == [FcuAction.SET_MODE]
        assert result.recovery_steps == ()

    asyncio.run(scenario())


def test_failure_after_arm_has_one_failed_terminal_and_land_recovery(tmp_path):
    async def scenario():
        def policy(command, _count):
            if command.action == FcuAction.TAKEOFF:
                return CommandStatus.PHYSICAL_TIMEOUT
            return CommandStatus.COMPLETED

        link = ScriptedLink(policy)
        journal_path = tmp_path / "mission.jsonl"
        runner = FixedMissionRunner(
            link,
            journal=MissionEvidenceJournal(journal_path),
        )

        result = await runner.run_once(attempt=4)

        assert result.terminal_state == MissionTerminalState.FAILED
        assert [step.result.action for step in result.steps] == [
            FcuAction.SET_MODE,
            FcuAction.ARM,
            FcuAction.TAKEOFF,
        ]
        assert [step.result.action for step in result.recovery_steps] == [
            FcuAction.LAND
        ]
        assert [command.action for command in link.commands][-1] == FcuAction.LAND
        record = json.loads(journal_path.read_text(encoding="utf-8"))
        assert record["terminal_state"] == "failed"
        assert record["attempt"] == 4
        assert record["steps"][2]["result"]["application"]["accepted"]
        assert record["steps"][2]["result"]["mavlink_ack"]["received"]
        assert not record["steps"][2]["result"]["physical_completion"][
            "confirmed"
        ]

    asyncio.run(scenario())


def test_guided_failure_does_not_send_land_while_known_unarmed():
    async def scenario():
        link = ScriptedLink(
            lambda command, _count: (
                CommandStatus.MAVLINK_REJECTED
                if command.action == FcuAction.SET_MODE
                else CommandStatus.COMPLETED
            )
        )

        result = await FixedMissionRunner(link).run_once()

        assert result.terminal_state == MissionTerminalState.FAILED
        assert [command.action for command in link.commands] == [FcuAction.SET_MODE]
        assert result.recovery_steps == ()

    asyncio.run(scenario())


def test_runner_exception_after_arm_still_attempts_land():
    async def scenario():
        def policy(command, _count):
            if command.action == FcuAction.TAKEOFF:
                return RuntimeError("injected dispatch exception")
            return CommandStatus.COMPLETED

        link = ScriptedLink(policy)
        result = await FixedMissionRunner(link).run_once()

        assert result.terminal_state == MissionTerminalState.FAILED
        assert "injected dispatch exception" in result.detail
        assert link.commands[-1].action == FcuAction.LAND
        assert result.recovery_steps[0].result.action == FcuAction.LAND

    asyncio.run(scenario())


def test_failed_recovery_still_produces_one_terminal_result():
    async def scenario():
        def policy(command, _count):
            if command.action == FcuAction.TAKEOFF:
                return CommandStatus.PHYSICAL_TIMEOUT
            if command.action == FcuAction.LAND:
                return RuntimeError("injected recovery failure")
            return CommandStatus.COMPLETED

        link = ScriptedLink(policy)
        runner = FixedMissionRunner(link)

        result = await runner.run_once()

        assert result.terminal_state == MissionTerminalState.FAILED
        assert result.recovery_steps == ()
        assert "LAND recovery raised RuntimeError" in result.detail
        assert runner.last_result is result
        assert sum(command.action == FcuAction.LAND for command in link.commands) == 1

    asyncio.run(scenario())


def test_repeated_batch_reports_nine_of_ten_without_losing_failed_evidence():
    async def scenario():
        def policy(command, count):
            if command.action == FcuAction.GOTO_LOCAL_NED and count == 9:
                return CommandStatus.PHYSICAL_TIMEOUT
            return CommandStatus.COMPLETED

        link = ScriptedLink(policy)
        batch = await FixedMissionRunner(link).run_repeated(10)

        assert len(batch.runs) == 10
        assert batch.completed_runs == 9
        assert batch.success_rate == pytest.approx(0.9)
        assert batch.terminal_state == MissionBatchState.PARTIAL
        assert batch.runs[4].terminal_state == MissionTerminalState.FAILED
        assert batch.runs[4].recovery_steps[0].result.action == FcuAction.LAND
        assert sum(command.action == FcuAction.LAND for command in link.commands) == 10
        north_targets = [
            command.target_position_ned_m[0]
            for command in link.commands
            if command.action == FcuAction.GOTO_LOCAL_NED
        ]
        assert north_targets == (
            [value for _ in range(4) for value in (3.0, 0.0)]
            + [3.0]
            + [value for _ in range(5) for value in (3.0, 0.0)]
        )

    asyncio.run(scenario())


def test_stop_on_failure_records_requested_target_and_completed_prefix():
    async def scenario():
        def policy(command, count):
            if command.action == FcuAction.ARM and count == 2:
                return CommandStatus.MAVLINK_REJECTED
            return CommandStatus.COMPLETED

        batch = await FixedMissionRunner(ScriptedLink(policy)).run_repeated(
            10,
            stop_on_failure=True,
        )

        assert batch.requested_runs == 10
        assert len(batch.runs) == 2
        assert batch.completed_runs == 1
        assert batch.success_rate == pytest.approx(0.5)
        assert batch.terminal_state == MissionBatchState.PARTIAL

    asyncio.run(scenario())


def test_cancelled_arm_waiter_records_cancelled_terminal_and_land_recovery(tmp_path):
    class PendingArmLink(ScriptedLink):
        def __init__(self):
            super().__init__()
            self.arm_started = asyncio.Event()

        def submit(self, command, **kwargs):
            if command.action == FcuAction.ARM:
                self.commands.append(command)
                self.arm_started.set()
                loop = asyncio.get_running_loop()
                future = loop.create_future()

                class PendingHandle:
                    @property
                    def result(self):
                        return asyncio.shield(future)

                return PendingHandle()
            return super().submit(command, **kwargs)

    async def scenario():
        link = PendingArmLink()
        runner = FixedMissionRunner(
            link,
            journal=MissionEvidenceJournal(tmp_path / "cancelled.jsonl"),
        )
        task = asyncio.create_task(runner.run_once())
        await link.arm_started.wait()

        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task

        assert runner.last_result is not None
        assert runner.last_result.terminal_state == MissionTerminalState.CANCELLED
        assert runner.last_result.recovery_steps[0].result.action == FcuAction.LAND
        record = json.loads(
            (tmp_path / "cancelled.jsonl").read_text(encoding="utf-8")
        )
        assert record["terminal_state"] == "cancelled"

    asyncio.run(scenario())


def test_command_result_json_keeps_all_three_evidence_layers():
    encoded = command_result_dict(completed_result(FcuAction.ARM))

    assert encoded["application"]["accepted"] is True
    assert encoded["mavlink_ack"]["received"] is True
    assert encoded["physical_completion"]["confirmed"] is True


def test_journal_failure_does_not_trigger_a_second_flight_or_recovery_action():
    class FailingJournal:
        def append(self, _result):
            raise OSError("injected full disk")

    async def scenario():
        link = ScriptedLink()
        runner = FixedMissionRunner(link, journal=FailingJournal())

        with pytest.raises(MissionEvidenceJournalError, match="full disk"):
            await runner.run_once()

        assert runner.last_result is not None
        assert runner.last_result.terminal_state == MissionTerminalState.COMPLETED
        assert runner.last_journal_error is not None
        assert len(link.commands) == 8

    asyncio.run(scenario())


def test_journal_and_repeat_arguments_are_strict(tmp_path):
    with pytest.raises(ValueError, match="absolute"):
        MissionEvidenceJournal(Path("relative.jsonl"))

    async def scenario():
        runner = FixedMissionRunner(ScriptedLink())
        with pytest.raises(ValueError, match="repetitions"):
            await runner.run_repeated(0)
        with pytest.raises(ValueError, match="inter_run_delay"):
            await runner.run_repeated(1, inter_run_delay_s=-1)

    asyncio.run(scenario())

import asyncio
from collections import defaultdict
from pathlib import Path
from types import SimpleNamespace
from uuid import uuid4

from aeromind_apm_lite.ground.simulation.m1_acceptance import (
    AcceptancePaths,
    build_launch_config,
    run_rtl_check,
    select_m1_vehicle,
)
from aeromind_apm_lite.onboard.apm_link import CommandHandle
from aeromind_apm_lite.onboard.mavlink import (
    ApplicationAcceptance,
    CommandResult,
    CommandStatus,
    FcuAction,
    MavlinkAckEvidence,
    PhysicalCompletionEvidence,
)


class RtlLink:
    guided_mode = "GUIDED"

    def __init__(self, failed_action=None):
        self.failed_action = failed_action
        self.commands = []
        self.counts = defaultdict(int)
        self.position = (0.0, 0.0, -2.0)

    def telemetry_snapshot(self):
        return SimpleNamespace(local_position_ned_m=self.position)

    def submit(self, command, **_kwargs):
        self.commands.append(command)
        self.counts[command.action] += 1
        if command.action == FcuAction.GOTO_LOCAL_NED:
            self.position = command.target_position_ned_m
        status = (
            CommandStatus.PHYSICAL_TIMEOUT
            if command.action == self.failed_action
            else CommandStatus.COMPLETED
        )
        confirmed = status == CommandStatus.COMPLETED
        application = ApplicationAcceptance(True, 1.0, "accepted")
        ack = MavlinkAckEvidence(
            applicable=True,
            received=True,
            command_id=20,
            result=0,
            observed_monotonic_s=1.1,
            detail="accepted",
        )
        physical = PhysicalCompletionEvidence(
            confirmed=confirmed,
            observed_monotonic_s=1.2 if confirmed else None,
            detail="physical",
        )
        result = CommandResult(
            request_id=uuid4(),
            action=command.action,
            status=status,
            application=application,
            sent_monotonic_s=1.0,
            mavlink_ack=ack,
            physical_completion=physical,
            detail=status.value,
        )
        loop = asyncio.get_running_loop()
        ack_future = loop.create_future()
        ack_future.set_result(ack)
        physical_future = loop.create_future()
        physical_future.set_result(physical)
        result_future = loop.create_future()
        result_future.set_result(result)
        return CommandHandle(
            request_id=result.request_id,
            action=command.action,
            application=application,
            _mavlink_ack=ack_future,
            _physical_completion=physical_future,
            _result=result_future,
        )


def test_rtl_check_flies_away_then_requires_rtl_completion():
    async def scenario():
        link = RtlLink()
        result = await run_rtl_check(link)

        assert result["completed"]
        assert [command.action for command in link.commands] == [
            FcuAction.SET_MODE,
            FcuAction.ARM,
            FcuAction.TAKEOFF,
            FcuAction.GOTO_LOCAL_NED,
            FcuAction.RTL,
        ]
        assert link.commands[3].target_position_ned_m == (3.0, 0.0, -2.0)
        assert result["recovery_steps"] == []

    asyncio.run(scenario())


def test_rtl_failure_records_land_recovery():
    async def scenario():
        link = RtlLink(failed_action=FcuAction.RTL)
        result = await run_rtl_check(link)

        assert not result["completed"]
        assert link.commands[-1].action == FcuAction.LAND
        assert result["recovery_steps"][0]["name"] == "recovery_land"

    asyncio.run(scenario())


def test_acceptance_paths_are_unique_and_non_destructive(tmp_path):
    first = AcceptancePaths.create(tmp_path)
    second = AcceptancePaths.create(tmp_path)

    assert first.run_directory.parent == tmp_path
    assert first.summary_path.parent == first.run_directory
    assert first.run_directory != second.run_directory


def test_build_launch_uses_frozen_single_vehicle_ports(tmp_path):
    project = tmp_path / "Blocks.uproject"
    executable = tmp_path / "UE4Editor.exe"
    parameter_file = tmp_path / "arducopter-m1.parm"
    project.write_text("{}", encoding="ascii")
    executable.write_bytes(b"exe")
    parameter_file.write_text("MAV_SYSID 1\n", encoding="ascii")
    vehicle = select_m1_vehicle(
        Path(__file__).parents[1] / "configs" / "sim" / "fleet.m1.yaml"
    )
    paths = AcceptancePaths.create(tmp_path / "state")

    launch = build_launch_config(
        vehicle=vehicle,
        parameter_file=parameter_file,
        paths=paths,
        ardupilot_root=tmp_path / "ardupilot",
        airsim_executable=executable,
        airsim_project=project,
        distro_name="Ubuntu-Test",
    )

    assert launch.instances[0].system_id == 1
    assert launch.instances[0].gcs_system_id == 201
    assert launch.instances[0].mavlink.port == 14550
    assert launch.airsim_process.arguments[0].startswith("\\\\wsl.localhost\\")

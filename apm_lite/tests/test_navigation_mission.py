import asyncio
import time
from dataclasses import replace
from types import MappingProxyType
from uuid import uuid4

import pytest

from aeromind_apm_lite.common.mission import MissionState
from aeromind_apm_lite.ground.simulation import (
    NavigationFaultInjection,
    NavigationFaultInjector,
    NavigationFaultType,
)
from aeromind_apm_lite.ground.simulation.m1_acceptance import (
    _flight_ready_reason,
)
from aeromind_apm_lite.onboard import (
    NavigationFailureCode,
    NavigationMissionPlan,
    NavigationMissionRunner,
    NavigationSafetyLimits,
    navigation_mission_result_dict,
)
from aeromind_apm_lite.onboard.mavlink import (
    ApplicationAcceptance,
    CommandResult,
    CommandStatus,
    FcuAction,
    MavlinkAckEvidence,
    PhysicalCompletionEvidence,
    TelemetrySnapshot,
)


def healthy_snapshot(*, armed=False, mode="STABILIZE"):
    now = time.monotonic()
    return TelemetrySnapshot(
        observed_monotonic_s=now,
        fcu_link_ok=True,
        armed=armed,
        mode=mode,
        relative_altitude_m=2.0 if armed else 0.0,
        local_position_ned_m=(0.0, 0.0, -2.0 if armed else 0.0),
        velocity_ned_m_s=(0.0, 0.0, 0.0),
        global_position_deg_m=(34.7472, 113.6253, 112.4),
        attitude_rpy_rad=(0.0, 0.0, 0.0),
        battery_remaining=0.8,
        battery_voltage_v=15.4,
        gps_fix_type=3,
        satellites_visible=16,
        gps_hdop=0.8,
        gps_healthy=True,
        prearm_ok=True,
        ekf_flags=831,
        landed_state=2 if armed else 1,
        home_position_deg_m=(34.7472, 113.6253, 112.4),
        home_position_ned_m=(0.0, 0.0, 0.0),
        last_status_text=None,
        last_heartbeat_monotonic_s=now,
        field_ages_s=MappingProxyType(
            {
                "heartbeat": 0.0,
                "gps": 0.0,
                "gps_health": 0.0,
                "ekf": 0.0,
                "local_position": 0.0,
                "global_position": 0.0,
                "velocity": 0.0,
                "landed_state": 0.0,
                "home": 0.0,
                "prearm": 0.0,
            }
        ),
    )


def command_result(action, status=CommandStatus.COMPLETED):
    completed = status == CommandStatus.COMPLETED
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
            detail="accepted",
        ),
        physical_completion=PhysicalCompletionEvidence(
            completed,
            1.3 if completed else None,
            "physical result",
        ),
        detail=status.value,
    )


class ImmediateHandle:
    def __init__(self, result):
        self._result = result

    @property
    def result(self):
        async def wait():
            return self._result

        return wait()


class DelayedHandle:
    def __init__(self, result):
        self._result = result

    @property
    def result(self):
        async def wait():
            await asyncio.sleep(10.0)
            return self._result

        return wait()


class NavigationLink:
    guided_mode = "GUIDED"

    def __init__(self, *, delay_goto=False, failed_action=None):
        self.commands = []
        self.armed = False
        self.mode = "STABILIZE"
        self.delay_goto = delay_goto
        self.failed_action = failed_action

    def telemetry_snapshot(self):
        return healthy_snapshot(armed=self.armed, mode=self.mode)

    def submit(self, command, **_kwargs):
        self.commands.append(command)
        status = (
            CommandStatus.MAVLINK_REJECTED
            if command.action == self.failed_action
            else CommandStatus.COMPLETED
        )
        if command.action == FcuAction.SET_MODE and status == CommandStatus.COMPLETED:
            self.mode = command.mode
        elif command.action == FcuAction.ARM and status == CommandStatus.COMPLETED:
            self.armed = True
        elif command.action == FcuAction.LAND and status == CommandStatus.COMPLETED:
            self.armed = False
        result = command_result(command.action, status)
        if self.delay_goto and command.action == FcuAction.GOTO_LOCAL_NED:
            return DelayedHandle(result)
        return ImmediateHandle(result)


def runner(link, **kwargs):
    return NavigationMissionRunner(
        link,
        safety_limits=NavigationSafetyLimits(check_interval_s=0.01),
        preflight_timeout_s=0.2,
        preflight_hold_s=0.0,
        **kwargs,
    )


def test_navigation_mission_runs_unified_six_step_state_machine():
    async def scenario():
        link = NavigationLink()
        plan = NavigationMissionPlan(target_position_ned_m=(5.0, 2.0, -2.0))

        result = await runner(link).run(plan)

        assert result.completed
        assert result.terminal_state == MissionState.COMPLETED
        assert result.failure_code is None
        assert [command.action for command in link.commands] == [
            FcuAction.SET_MODE,
            FcuAction.ARM,
            FcuAction.TAKEOFF,
            FcuAction.GOTO_LOCAL_NED,
            FcuAction.HOLD,
            FcuAction.LAND,
        ]
        assert link.commands[0].mode == "GUIDED"
        assert link.commands[3].target_position_ned_m == (5.0, 2.0, -2.0)
        assert [transition.current for transition in result.transitions] == [
            MissionState.VALIDATED,
            MissionState.ACCEPTED,
            MissionState.RUNNING,
            MissionState.COMPLETED,
        ]
        payload = navigation_mission_result_dict(result)
        assert payload["terminal_state"] == "completed"
        assert len(payload["steps"]) == 6

    asyncio.run(scenario())


@pytest.mark.parametrize(
    "fault,expected_code",
    [
        (NavigationFaultType.GPS_LOSS, NavigationFailureCode.GPS_UNHEALTHY),
        (NavigationFaultType.HDOP_EXCEEDED, NavigationFailureCode.HDOP_EXCEEDED),
        (NavigationFaultType.EKF_FAILURE, NavigationFailureCode.EKF_UNHEALTHY),
        (NavigationFaultType.MISSION_EXPIRED, NavigationFailureCode.MISSION_EXPIRED),
        (
            NavigationFaultType.GROUND_LINK_LOSS,
            NavigationFailureCode.GROUND_LINK_LOST,
        ),
    ],
)
def test_injected_goto_fault_stops_trajectory_then_holds_and_lands(
    fault,
    expected_code,
):
    async def scenario():
        link = NavigationLink(delay_goto=True)
        injector = NavigationFaultInjector(
            NavigationFaultInjection(fault=fault)
        )
        result = await runner(link, observation_filter=injector).run(
            NavigationMissionPlan(target_position_ned_m=(4.0, 0.0, -2.0))
        )

        expected_terminal = (
            MissionState.EXPIRED
            if fault == NavigationFaultType.MISSION_EXPIRED
            else MissionState.FAILED
        )
        assert injector.active
        assert result.terminal_state == expected_terminal
        assert result.failure_code == expected_code
        assert result.safety_events[0].phase == "goto"
        assert [command.action for command in link.commands][-2:] == [
            FcuAction.HOLD,
            FcuAction.LAND,
        ]
        assert [step.name for step in result.recovery_steps] == [
            "safety_hold",
            "recovery_land",
        ]
        assert all(
            step.result.physical_completion.confirmed
            for step in result.recovery_steps
        )

    asyncio.run(scenario())


def test_bad_preflight_never_arms_or_issues_recovery_commands():
    class BadHdopLink(NavigationLink):
        def telemetry_snapshot(self):
            return replace(super().telemetry_snapshot(), gps_hdop=9.0)

    async def scenario():
        link = BadHdopLink()
        result = await runner(link).run(
            NavigationMissionPlan(target_position_ned_m=(3.0, 0.0, -2.0))
        )

        assert result.terminal_state == MissionState.FAILED
        assert result.failure_code == NavigationFailureCode.PREFLIGHT_TIMEOUT
        assert "HDOP" in result.detail
        assert link.commands == []
        assert result.recovery_steps == ()

    asyncio.run(scenario())


def test_command_failure_after_arm_records_hold_and_land_recovery():
    async def scenario():
        link = NavigationLink(failed_action=FcuAction.TAKEOFF)
        result = await runner(link).run(
            NavigationMissionPlan(target_position_ned_m=(3.0, 0.0, -2.0))
        )

        assert result.terminal_state == MissionState.FAILED
        assert result.failure_code == NavigationFailureCode.COMMAND_FAILED
        assert [step.name for step in result.recovery_steps] == [
            "safety_hold",
            "recovery_land",
        ]

    asyncio.run(scenario())


def test_navigation_plan_rejects_ground_targets_and_invalid_deadlines():
    with pytest.raises(ValueError, match="above Home"):
        NavigationMissionPlan(target_position_ned_m=(1.0, 2.0, 0.0))
    with pytest.raises(ValueError, match="shorter"):
        NavigationMissionPlan(
            target_position_ned_m=(1.0, 2.0, -2.0),
            mission_timeout_s=10.0,
            goto_timeout_s=10.0,
        )


def test_original_sitl_readiness_gate_now_requires_hdop_and_ekf():
    snapshot = healthy_snapshot()
    assert _flight_ready_reason(snapshot, freshness_s=1.0) is None
    assert "HDOP" in _flight_ready_reason(
        replace(snapshot, gps_hdop=3.0),
        freshness_s=1.0,
    )
    assert "EKF" in _flight_ready_reason(
        replace(snapshot, ekf_flags=0),
        freshness_s=1.0,
    )

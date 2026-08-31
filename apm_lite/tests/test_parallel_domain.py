from __future__ import annotations

import asyncio
import time
from datetime import datetime, timedelta, timezone

import pytest
from pydantic import ValidationError

from aeromind_apm_lite.common.coordinates import (
    CalibrationStatus,
    GeoReference,
    VehicleHome,
    Wgs84Position,
)
from aeromind_apm_lite.ground.parallel_domain.adapters import FakeFlightControllerAdapter
from aeromind_apm_lite.ground.parallel_domain.config import (
    MavlinkEndpointConfig,
    ParallelLineConfig,
)
from aeromind_apm_lite.ground.parallel_domain.core import (
    BridgeStateError,
    GateRejected,
    ParallelDomainCore,
)
from aeromind_apm_lite.ground.parallel_domain.models import (
    AdapterKind,
    EndpointRole,
    MissionAction,
    MissionPlan,
    MissionStep,
    SafetyPolicy,
)


def georeference() -> GeoReference:
    origin = Wgs84Position(
        latitude_deg=47.641468,
        longitude_deg=-122.140165,
        altitude_m=122.0,
    )
    return GeoReference(
        calibration_id="parallel-test-v1",
        status=CalibrationStatus.DRAFT,
        map_origin_wgs84=origin,
        map_x_heading_from_true_north_deg=0.0,
        airsim_origin_wgs84=origin,
        vehicle_homes=(
            VehicleHome(vehicle_id=1, map_position_m=(0.0, 0.0, 0.0)),
            VehicleHome(vehicle_id=2, map_position_m=(0.0, 0.0, 0.0)),
        ),
    )


def line_config(policy: SafetyPolicy) -> ParallelLineConfig:
    return ParallelLineConfig(
        line_id="px4",
        adapter=AdapterKind.PX4,
        vehicle_id=1,
        platform_type="px4_multirotor",
        real=MavlinkEndpointConfig(
            endpoint="udpin:0.0.0.0:14540",
            target_system=1,
        ),
        virtual=MavlinkEndpointConfig(
            endpoint="udpin:0.0.0.0:14541",
            target_system=1,
        ),
        policy=policy,
    )


def plan(*, altitude_m: float = 5.0) -> MissionPlan:
    return MissionPlan(
        line_id="px4",
        vehicle_id=1,
        frame_calibration_id="parallel-test-v1",
        cruise_speed_m_s=4.0,
        steps=(
            MissionStep(action=MissionAction.TAKEOFF, altitude_m=altitude_m),
            MissionStep(action=MissionAction.WAYPOINT, position_map_m=(5.0, 0.0, altitude_m)),
            MissionStep(action=MissionAction.LAND),
        ),
    )


def test_mission_contract_has_no_low_level_control_surface():
    with pytest.raises(ValidationError, match="extra"):
        MissionStep.model_validate(
            {"action": "takeoff", "altitude_m": 5.0, "motor_outputs": [0.1, 0.1, 0.1, 0.1]}
        )


def test_px4_adapter_maps_only_task_level_mavlink_commands():
    from aeromind_apm_lite.ground.parallel_domain.adapters.px4 import Px4Adapter

    async def callback(_telemetry):
        return None

    adapter = Px4Adapter(
        endpoint=MavlinkEndpointConfig(endpoint="udpin:0.0.0.0:14541", target_system=1),
        georeference=georeference(),
        line_id="px4",
        vehicle_id=1,
        platform_type="px4_multirotor",
        role=EndpointRole.VIRTUAL,
        telemetry_callback=callback,
    )
    items = adapter._build_mission_items(plan())
    assert [item.command for item in items] == [178, 22, 16, 21]
    assert items[2].x != 0 and items[2].y != 0
    assert items[2].z == 5.0


class RecordingTaskIO:
    def __init__(self):
        self.modes = []
        self.commands = []
        self.local_targets = []
        self.global_targets = []
        self.mission_calls = []

    async def open(self):
        return None

    async def close(self):
        return None

    async def recv(self, _timeout_s):
        return None

    async def send_heartbeat(self):
        return None

    async def send_command_long(self, command_id, params):
        self.commands.append((command_id, tuple(params)))

    async def set_mode(self, mode):
        self.modes.append(mode)

    async def send_position_target_local_ned(self, north_m, east_m, down_m):
        self.local_targets.append((north_m, east_m, down_m))

    async def send_position_target_global_int(
        self,
        latitude_deg,
        longitude_deg,
        relative_altitude_m,
    ):
        self.global_targets.append(
            (latitude_deg, longitude_deg, relative_altitude_m)
        )

    async def mission_clear_all(self):
        self.mission_calls.append("clear")

    async def mission_count(self, count):
        self.mission_calls.append(("count", count))

    async def mission_item_int(self, item):
        self.mission_calls.append(("item", item))

    async def mission_set_current(self, sequence):
        self.mission_calls.append(("current", sequence))


def test_live_adapters_keep_offboard_and_guided_differences_local(monkeypatch):
    from aeromind_apm_lite.ground.parallel_domain.adapters.ardupilot import (
        ArduPilotAdapter,
    )
    from aeromind_apm_lite.ground.parallel_domain.adapters.px4 import Px4Adapter
    from aeromind_apm_lite.ground.parallel_domain.adapters import px4 as px4_module

    async def callback(_telemetry):
        return None

    async def no_delay(_seconds):
        return None

    monkeypatch.setattr(px4_module.asyncio, "sleep", no_delay)

    async def scenario():
        px4_io = RecordingTaskIO()
        px4_adapter = Px4Adapter(
            endpoint=MavlinkEndpointConfig(
                endpoint="udpin:0.0.0.0:14541", target_system=1
            ),
            georeference=georeference(),
            line_id="px4",
            vehicle_id=1,
            platform_type="px4_multirotor",
            role=EndpointRole.VIRTUAL,
            telemetry_callback=callback,
            io=px4_io,
        )
        px4_record = await px4_adapter._dispatch(plan())
        assert px4_record.execution_semantic.value == "px4_offboard_task"
        assert px4_io.modes == ["OFFBOARD"]
        assert len(px4_io.local_targets) == 10
        assert px4_io.mission_calls == []
        assert await px4_adapter.safety_hold() == "SET_MODE:AUTO.LOITER"
        assert await px4_adapter.recovery_land() == "SET_MODE:AUTO.LAND"
        assert px4_io.modes[-2:] == ["AUTO.LOITER", "AUTO.LAND"]

        apm_plan = plan().model_copy(update={"line_id": "apm", "vehicle_id": 2})
        apm_io = RecordingTaskIO()
        apm_adapter = ArduPilotAdapter(
            endpoint=MavlinkEndpointConfig(
                endpoint="udpin:0.0.0.0:14551",
                target_system=2,
                dialect="ardupilotmega",
            ),
            georeference=georeference(),
            line_id="apm",
            vehicle_id=2,
            platform_type="ardupilot_multirotor",
            role=EndpointRole.VIRTUAL,
            telemetry_callback=callback,
            io=apm_io,
        )
        apm_record = await apm_adapter._dispatch(apm_plan)
        assert apm_record.execution_semantic.value == "ardupilot_guided_task"
        assert apm_io.modes == ["GUIDED"]
        assert [command for command, _params in apm_io.commands] == [400, 178, 22]
        assert apm_io.mission_calls == []
        assert await apm_adapter.safety_hold() == "SET_MODE:LOITER"
        assert await apm_adapter.recovery_land() == "SET_MODE:LAND+MAV_CMD_NAV_LAND"
        assert apm_io.modes[-2:] == ["LOITER", "LAND"]
        assert [command for command, _params in apm_io.commands[-1:]] == [21]

        class AliasOnlyPX4IO(RecordingTaskIO):
            async def set_mode(self, mode):
                raise ValueError(f"mode alias not advertised: {mode}")

        alias_io = AliasOnlyPX4IO()
        alias_adapter = Px4Adapter(
            endpoint=MavlinkEndpointConfig(
                endpoint="udpin:0.0.0.0:14541", target_system=1
            ),
            georeference=georeference(),
            line_id="px4",
            vehicle_id=1,
            platform_type="px4_multirotor",
            role=EndpointRole.REAL,
            telemetry_callback=callback,
            io=alias_io,
        )
        fallback_record = await alias_adapter._dispatch(plan())
        assert fallback_record.execution_semantic.value == "px4_offboard_task"
        assert await alias_adapter.safety_hold() == "SET_MODE:LOITER"
        assert await alias_adapter.recovery_land() == "SET_MODE:LAND"
        assert [command for command, _params in alias_io.commands] == [
            178,
            176,
            400,
            176,
            176,
        ]

    asyncio.run(scenario())


def test_core_rejects_parameter_boundary_before_virtual_dispatch(tmp_path):
    async def scenario():
        policy = SafetyPolicy(
            allowed_actions=tuple(MissionAction),
            max_altitude_m=10.0,
            max_distance_from_home_m=100.0,
            max_speed_m_s=8.0,
        )
        core = ParallelDomainCore(
            georeference=georeference(),
            state_directory=str(tmp_path),
        )
        async def callback(_telemetry):
            return None

        virtual = FakeFlightControllerAdapter(
            kind=AdapterKind.PX4,
            line_id="px4",
            vehicle_id=1,
            role=EndpointRole.VIRTUAL,
            telemetry_callback=callback,
        )
        real = FakeFlightControllerAdapter(
            kind=AdapterKind.PX4,
            line_id="px4",
            vehicle_id=1,
            role=EndpointRole.REAL,
            telemetry_callback=callback,
        )
        core.add_line(line_config(policy), real=real, virtual=virtual)
        await core.start()
        with pytest.raises(GateRejected):
            await core.submit_rehearsal(plan(altitude_m=20.0))
        assert not virtual.executions
        await core.stop()

    asyncio.run(scenario())


def test_core_rejects_non_whitelisted_action_before_virtual_dispatch(tmp_path):
    async def scenario():
        policy = SafetyPolicy(
            allowed_actions=(
                MissionAction.TAKEOFF,
                MissionAction.WAYPOINT,
                MissionAction.LAND,
            )
        )
        core = ParallelDomainCore(
            georeference=georeference(),
            state_directory=str(tmp_path),
        )

        async def callback(_telemetry):
            return None

        virtual = FakeFlightControllerAdapter(
            kind=AdapterKind.PX4,
            line_id="px4",
            vehicle_id=1,
            role=EndpointRole.VIRTUAL,
            telemetry_callback=callback,
        )
        real = FakeFlightControllerAdapter(
            kind=AdapterKind.PX4,
            line_id="px4",
            vehicle_id=1,
            role=EndpointRole.REAL,
            telemetry_callback=callback,
        )
        core.add_line(line_config(policy), real=real, virtual=virtual)
        denied = MissionPlan(
            line_id="px4",
            vehicle_id=1,
            frame_calibration_id="parallel-test-v1",
            cruise_speed_m_s=4.0,
            steps=(
                MissionStep(action=MissionAction.TAKEOFF, altitude_m=5.0),
                MissionStep(
                    action=MissionAction.WAYPOINT,
                    position_map_m=(5.0, 0.0, 5.0),
                ),
                MissionStep(action=MissionAction.RTL),
            ),
        )
        await core.start()
        with pytest.raises(GateRejected, match="capability_whitelist"):
            await core.submit_rehearsal(denied)
        assert not virtual.executions
        audit = (tmp_path / "audit" / "events.jsonl").read_text(encoding="utf-8")
        assert '"event_type":"rehearsal_rejected"' in audit
        assert '"name":"capability_whitelist","passed":false' in audit
        await core.stop()

    asyncio.run(scenario())


def test_live_health_fault_runs_hold_then_recovery_land_and_finishes_observed(tmp_path):
    async def scenario():
        policy = SafetyPolicy(allowed_actions=tuple(MissionAction))
        core = ParallelDomainCore(
            georeference=georeference(),
            state_directory=str(tmp_path),
            recovery_samples=1,
            safety_hold_duration_s=0.0,
        )
        adapters = {}
        for role in (EndpointRole.REAL, EndpointRole.VIRTUAL):
            adapters[role] = FakeFlightControllerAdapter(
                kind=AdapterKind.PX4,
                line_id="px4",
                vehicle_id=1,
                role=role,
                telemetry_callback=lambda telemetry, current_role=role: core.on_telemetry(
                    "px4", current_role, telemetry
                ),
            )
        core.add_line(
            line_config(policy),
            real=adapters[EndpointRole.REAL],
            virtual=adapters[EndpointRole.VIRTUAL],
        )
        await core.start()
        mission = plan()
        monotonic_base = time.monotonic()
        await core.submit_rehearsal(mission)
        await adapters[EndpointRole.VIRTUAL].emit(
            latitude_deg=47.641468,
            longitude_deg=-122.140165,
            altitude_m=122.0,
            local_position_ned_m=(0.0, 0.0, 0.0),
            mission_phase="rehearsal",
            received_monotonic_s=monotonic_base,
            mission_sequence=0,
        )
        await adapters[EndpointRole.VIRTUAL].emit(
            latitude_deg=47.641468,
            longitude_deg=-122.140165,
            altitude_m=122.0,
            local_position_ned_m=(1.0, 0.0, -2.0),
            mission_phase="completed",
            received_monotonic_s=monotonic_base + 0.1,
            mission_sequence=2,
            mission_complete=True,
        )
        digest = core.approval_digest(mission.mission_id)
        await core.approve(
            mission.mission_id,
            mission_hash=mission.mission_hash,
            operator_id="fault-test",
            confirmation_digest=digest,
        )
        await adapters[EndpointRole.REAL].emit(
            latitude_deg=47.641468,
            longitude_deg=-122.140165,
            altitude_m=122.0,
            local_position_ned_m=(0.0, 0.0, -1.0),
            mission_phase="ready",
            received_monotonic_s=monotonic_base + 0.2,
        )
        await core.dispatch_real(mission.mission_id)
        await adapters[EndpointRole.REAL].emit(
            latitude_deg=47.641468,
            longitude_deg=-122.140165,
            altitude_m=122.0,
            local_position_ned_m=(1.0, 0.0, -2.0),
            mission_phase="executing",
            received_monotonic_s=monotonic_base + 0.3,
        )
        before = core._lines["px4"].state_set.observed
        await adapters[EndpointRole.REAL].emit(
            latitude_deg=47.641468,
            longitude_deg=-122.140165,
            altitude_m=122.0,
            local_position_ned_m=(2.0, 0.0, -2.0),
            mission_phase="fault",
            received_monotonic_s=monotonic_base + 0.4,
            gps_fix_type=1,
            heartbeat_age_s=4.0,
            gps_age_s=4.0,
            link_ok=False,
        )
        runtime = core._lines["px4"]
        assert runtime.mirror_status.state.value == "stale"
        assert runtime.state_set.observed == before
        assert runtime.sessions[mission.mission_id].status == "recovery_land"
        assert adapters[EndpointRole.REAL].safety_commands == [
            "SET_MODE:AUTO.LOITER",
            "SET_MODE:AUTO.LAND",
        ]
        await adapters[EndpointRole.REAL].emit(
            latitude_deg=47.641468,
            longitude_deg=-122.140165,
            altitude_m=122.0,
            local_position_ned_m=(2.0, 0.0, -2.0),
            mission_phase="completed",
            received_monotonic_s=monotonic_base + 0.5,
            mission_sequence=2,
            mission_complete=True,
        )
        assert runtime.sessions[mission.mission_id].status == "completed"
        audit = (tmp_path / "audit" / "events.jsonl").read_text(encoding="utf-8")
        assert '"event_type":"safety_hold"' in audit
        assert '"event_type":"recovery_land"' in audit
        assert '"line_id":"px4"' in audit
        assert '"vehicle_id":1' in audit
        await core.stop()

    asyncio.run(scenario())


def test_real_dispatch_requires_confirmation_and_fresh_health(tmp_path):
    async def scenario():
        policy = SafetyPolicy(
            allowed_actions=tuple(MissionAction),
            max_altitude_m=20.0,
            max_distance_from_home_m=100.0,
            max_speed_m_s=8.0,
        )
        core = ParallelDomainCore(
            georeference=georeference(),
            state_directory=str(tmp_path),
        )
        adapters = {}
        for role in (EndpointRole.REAL, EndpointRole.VIRTUAL):
            adapters[role] = FakeFlightControllerAdapter(
                kind=AdapterKind.PX4,
                line_id="px4",
                vehicle_id=1,
                role=role,
                telemetry_callback=lambda telemetry, current_role=role: core.on_telemetry(
                    "px4", current_role, telemetry
                ),
            )
        core.add_line(line_config(policy), real=adapters[EndpointRole.REAL], virtual=adapters[EndpointRole.VIRTUAL])
        await core.start()
        current = time.monotonic()
        await core.submit_rehearsal(plan())
        for complete, received in ((False, current), (True, current + 0.1)):
            await adapters[EndpointRole.VIRTUAL].emit(
                latitude_deg=47.641468,
                longitude_deg=-122.140165,
                altitude_m=122.0,
                local_position_ned_m=(0.0, 0.0, 0.0),
                mission_phase="completed" if complete else "mission",
                mission_sequence=2,
                mission_complete=complete,
                received_monotonic_s=received,
            )
        mission_id = next(iter(core._lines["px4"].sessions))
        with pytest.raises(Exception):
            await core.dispatch_real(mission_id)
        digest = core.approval_digest(mission_id)
        session = core._lines["px4"].sessions[mission_id]
        receipt = await core.approve(
            mission_id,
            mission_hash=session.plan.mission_hash,
            operator_id="test-operator",
            confirmation_digest=digest,
        )
        with pytest.raises(BridgeStateError, match="already been confirmed"):
            await core.approve(
                mission_id,
                mission_hash=session.plan.mission_hash,
                operator_id="test-operator",
                confirmation_digest=digest,
            )
        await adapters[EndpointRole.REAL].emit(
            latitude_deg=47.641468,
            longitude_deg=-122.140165,
            altitude_m=122.0,
            local_position_ned_m=(0.0, 0.0, 0.0),
            mission_phase="ready",
            gps_fix_type=3,
            received_monotonic_s=time.monotonic(),
        )
        await core.dispatch_real(mission_id, receipt)
        with pytest.raises(BridgeStateError, match="already been consumed"):
            await core.dispatch_real(mission_id, receipt)
        await adapters[EndpointRole.REAL].emit(
            latitude_deg=47.641468,
            longitude_deg=-122.140165,
            altitude_m=122.0,
            local_position_ned_m=(0.0, 0.0, 0.0),
            mission_phase="mission",
            mission_sequence=1,
            received_monotonic_s=time.monotonic(),
        )
        await adapters[EndpointRole.REAL].emit(
            latitude_deg=47.641468,
            longitude_deg=-122.140165,
            altitude_m=122.0,
            local_position_ned_m=(1.0, 0.0, 0.0),
            mission_phase="completed",
            mission_sequence=2,
            mission_complete=True,
            received_monotonic_s=time.monotonic() + 0.1,
        )
        assert adapters[EndpointRole.REAL].executions
        assert core.status()["lines"]["px4"]["missions"][str(mission_id)] == "completed"
        latest = (tmp_path / "twin" / "px4" / "real.json").read_text(encoding="utf-8")
        assert '"source": "observed"' in latest
        evidence_roles = {
            item.model_dump(mode="json")["role"]
            for item in (
                core.evidence_store.load(
                    core._lines["px4"].sessions[mission_id].predicted_evidence.evidence_id
                ),
                core._lines["px4"].sessions[mission_id].observed_evidence,
            )
        }
        assert evidence_roles == {"predicted", "observed"}
        comparison = core._lines["px4"].sessions[mission_id].comparison
        assert comparison is not None
        assert comparison.alignment_clock == "wall_time"
        assert comparison.report.comparisons[0].reference_role.value == "predicted"
        assert comparison.report.comparisons[0].compared_role.value == "observed"
        audit = (tmp_path / "audit" / "events.jsonl").read_text(encoding="utf-8")
        assert '"event_type":"approval_replay_rejected"' in audit
        assert '"event_type":"real_dispatch_replay_rejected"' in audit
        await core.stop()

    asyncio.run(scenario())


def test_expired_operator_confirmation_is_rejected_and_audited(tmp_path):
    async def scenario():
        now = [datetime(2026, 8, 16, 12, 0, tzinfo=timezone.utc)]
        policy = SafetyPolicy(
            allowed_actions=tuple(MissionAction),
            max_altitude_m=20.0,
            max_distance_from_home_m=100.0,
            max_speed_m_s=8.0,
            approval_ttl_s=30,
        )
        core = ParallelDomainCore(
            georeference=georeference(),
            state_directory=str(tmp_path),
            wall_clock=lambda: now[0],
        )
        adapters = {}
        for role in (EndpointRole.REAL, EndpointRole.VIRTUAL):
            adapters[role] = FakeFlightControllerAdapter(
                kind=AdapterKind.PX4,
                line_id="px4",
                vehicle_id=1,
                role=role,
                telemetry_callback=lambda telemetry, current_role=role: core.on_telemetry(
                    "px4", current_role, telemetry
                ),
            )
        core.add_line(
            line_config(policy),
            real=adapters[EndpointRole.REAL],
            virtual=adapters[EndpointRole.VIRTUAL],
        )
        await core.start()
        mission = plan()
        await core.submit_rehearsal(mission)
        for offset, complete in ((0.0, False), (0.1, True)):
            await adapters[EndpointRole.VIRTUAL].emit(
                latitude_deg=47.641468,
                longitude_deg=-122.140165,
                altitude_m=122.0,
                local_position_ned_m=(offset, 0.0, 0.0),
                mission_phase="completed" if complete else "rehearsal",
                mission_complete=complete,
                received_monotonic_s=1.0 + offset,
                sampled_at_utc=now[0] + timedelta(seconds=offset),
            )
        digest = core.approval_digest(mission.mission_id)
        now[0] += timedelta(seconds=31)
        with pytest.raises(BridgeStateError, match="expired"):
            await core.approve(
                mission.mission_id,
                mission_hash=mission.mission_hash,
                operator_id="ttl-test",
                confirmation_digest=digest,
            )
        audit = (tmp_path / "audit" / "events.jsonl").read_text(encoding="utf-8")
        assert '"reason":"approval_ttl_expired"' in audit
        assert not (tmp_path / "approvals" / f"{mission.mission_id}.json").exists()
        await core.stop()

    asyncio.run(scenario())

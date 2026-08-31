from __future__ import annotations

import asyncio
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from pathlib import Path

from aeromind_apm_lite.common.contracts import TwinSource
from aeromind_apm_lite.common.coordinates import (
    CalibrationStatus,
    GeoReference,
    Vec3,
    VehicleHome,
    Wgs84Position,
    map_to_geodetic,
)
from aeromind_apm_lite.ground.parallel_domain.adapters import (
    ArduPilotAdapter,
    FakeFlightControllerAdapter,
    Px4Adapter,
)
from aeromind_apm_lite.ground.parallel_domain.config import (
    MavlinkEndpointConfig,
    ParallelLineConfig,
)
from aeromind_apm_lite.ground.parallel_domain.core import ParallelDomainCore
from aeromind_apm_lite.ground.parallel_domain.models import (
    AdapterKind,
    EndpointRole,
    MirrorState,
    MissionAction,
    SafetyPolicy,
)
from aeromind_apm_lite.ground.parallel_domain.service import ParallelDomainService
from aeromind_apm_lite.onboard.mavlink.models import MavlinkEnvelope


@dataclass
class ManualClock:
    monotonic_s: float = 100.0
    wall: datetime = datetime(2026, 8, 16, 1, 0, tzinfo=timezone.utc)

    def monotonic(self) -> float:
        return self.monotonic_s

    def wall_time(self) -> datetime:
        return self.wall

    def advance(self, seconds: float = 1.0) -> None:
        self.monotonic_s += seconds
        self.wall += timedelta(seconds=seconds)


def _georeference() -> GeoReference:
    origin = Wgs84Position(
        latitude_deg=47.641468,
        longitude_deg=-122.140165,
        altitude_m=122.0,
    )
    return GeoReference(
        calibration_id="parallel-live-test-v1",
        status=CalibrationStatus.SURVEYED,
        map_origin_wgs84=origin,
        map_x_heading_from_true_north_deg=0.0,
        airsim_origin_wgs84=origin,
        vehicle_homes=(
            VehicleHome(vehicle_id=1, map_position_m=(0.0, 0.0, 0.0)),
            VehicleHome(vehicle_id=2, map_position_m=(0.0, 3.0, 0.0)),
        ),
    )


def _line(line_id: str, vehicle_id: int, kind: AdapterKind) -> ParallelLineConfig:
    dialect = "common" if kind == AdapterKind.PX4 else "ardupilotmega"
    base_port = 14540 if kind == AdapterKind.PX4 else 14550
    return ParallelLineConfig(
        line_id=line_id,
        adapter=kind,
        vehicle_id=vehicle_id,
        platform_type=f"{line_id}_multirotor",
        real=MavlinkEndpointConfig(
            endpoint=f"udpin:0.0.0.0:{base_port}",
            target_system=vehicle_id,
            dialect=dialect,
        ),
        virtual=MavlinkEndpointConfig(
            endpoint=f"udpin:0.0.0.0:{base_port + 1}",
            target_system=vehicle_id,
            dialect=dialect,
        ),
        policy=SafetyPolicy(allowed_actions=tuple(MissionAction)),
    )


async def _emit_real(
    adapter: FakeFlightControllerAdapter,
    clock: ManualClock,
    georeference: GeoReference,
    *,
    x_m: float,
    sim_time_s: float,
    advance_s: float = 1.0,
    received_monotonic_s: float | None = None,
    **overrides: object,
):
    clock.advance(advance_s)
    assert georeference.map_origin_wgs84 is not None
    position = map_to_geodetic(
        Vec3(x_m, 0.0, 2.0),
        georeference.venue_calibration(),
        georeference.map_origin_wgs84.coordinate(),
    )
    return await adapter.emit(
        latitude_deg=position.latitude_deg,
        longitude_deg=position.longitude_deg,
        altitude_m=position.altitude_m,
        local_position_ned_m=(x_m, 0.0, -2.0),
        mission_phase="manual_flight",
        received_monotonic_s=(
            clock.monotonic_s
            if received_monotonic_s is None
            else received_monotonic_s
        ),
        sampled_at_utc=clock.wall,
        sim_time_s=sim_time_s,
        **overrides,
    )


def test_live_real_to_virtual_freshness_replay_and_state_slots(tmp_path: Path):
    async def scenario() -> None:
        clock = ManualClock()
        georeference = _georeference()
        core = ParallelDomainCore(
            georeference=georeference,
            state_directory=str(tmp_path),
            telemetry_stale_after_s=3.0,
            recovery_samples=2,
            monotonic_clock=clock.monotonic,
            wall_clock=clock.wall_time,
        )
        adapters: dict[str, tuple[FakeFlightControllerAdapter, FakeFlightControllerAdapter]] = {}
        for line_id, vehicle_id, kind in (
            ("px4", 1, AdapterKind.PX4),
            ("apm", 2, AdapterKind.ARDUPILOT),
        ):
            real = FakeFlightControllerAdapter(
                kind=kind,
                line_id=line_id,
                vehicle_id=vehicle_id,
                role=EndpointRole.REAL,
                telemetry_callback=lambda telemetry, line=line_id: core.on_telemetry(
                    line, EndpointRole.REAL, telemetry
                ),
            )
            virtual = FakeFlightControllerAdapter(
                kind=kind,
                line_id=line_id,
                vehicle_id=vehicle_id,
                role=EndpointRole.VIRTUAL,
                telemetry_callback=lambda telemetry, line=line_id: core.on_telemetry(
                    line, EndpointRole.VIRTUAL, telemetry
                ),
            )
            core.add_line(_line(line_id, vehicle_id, kind), real=real, virtual=virtual)
            adapters[line_id] = (real, virtual)

        await core.start()
        for line_id, scale in (("px4", 1.0), ("apm", 10.0)):
            real, virtual = adapters[line_id]
            await _emit_real(real, clock, georeference, x_m=1.0, sim_time_s=scale)
            assert core.status()["lines"][line_id]["mirror"]["state"] == "fresh"
            initial = core._lines[line_id].state_set.observed
            assert initial is not None

            await _emit_real(
                real,
                clock,
                georeference,
                x_m=20.0,
                sim_time_s=scale * 2,
                heartbeat_age_s=4.0,
            )
            assert core._lines[line_id].mirror_status.state == MirrorState.STALE
            assert core._lines[line_id].state_set.observed == initial
            await _emit_real(real, clock, georeference, x_m=21.0, sim_time_s=scale * 3)
            assert core._lines[line_id].mirror_status.state == MirrorState.RECOVERING
            assert core._lines[line_id].state_set.observed == initial
            await _emit_real(real, clock, georeference, x_m=2.0, sim_time_s=scale * 4)

            before_gps_fault = core._lines[line_id].state_set.observed
            await _emit_real(
                real,
                clock,
                georeference,
                x_m=30.0,
                sim_time_s=scale * 5,
                gps_fix_type=1,
            )
            assert core._lines[line_id].mirror_status.reason == "gps_fix_lost"
            assert core._lines[line_id].state_set.observed == before_gps_fault
            await _emit_real(real, clock, georeference, x_m=31.0, sim_time_s=scale * 6)
            await _emit_real(real, clock, georeference, x_m=3.0, sim_time_s=scale * 7)

            before_jitter = core._lines[line_id].state_set.observed
            await _emit_real(
                real,
                clock,
                georeference,
                x_m=40.0,
                sim_time_s=scale * 8,
                link_ok=False,
            )
            assert core._lines[line_id].mirror_status.reason == "heartbeat_lost"
            assert core._lines[line_id].state_set.observed == before_jitter
            await _emit_real(real, clock, georeference, x_m=41.0, sim_time_s=scale * 9)
            await _emit_real(real, clock, georeference, x_m=4.0, sim_time_s=scale * 10)

            before_silence = core._lines[line_id].state_set.observed
            count_before_silence = len(core.store.read_observed(line_id))
            clock.advance(10.0)
            await core.refresh_freshness()
            assert core._lines[line_id].mirror_status.reason == "telemetry_timeout"
            assert core._lines[line_id].state_set.observed == before_silence
            assert len(core.store.read_observed(line_id)) == count_before_silence

            await _emit_real(
                real,
                clock,
                georeference,
                x_m=50.0,
                sim_time_s=0.1,
                connection_generation=2,
            )
            assert core._lines[line_id].mirror_status.state == MirrorState.RECOVERING
            assert core._lines[line_id].state_set.observed == before_silence
            recovered = await _emit_real(
                real,
                clock,
                georeference,
                x_m=5.0,
                sim_time_s=1.1,
                connection_generation=2,
            )
            assert core._lines[line_id].mirror_status.state == MirrorState.FRESH

            count_before_duplicate = len(core.store.read_observed(line_id))
            await _emit_real(
                real,
                clock,
                georeference,
                x_m=60.0,
                sim_time_s=2.1,
                source_sequence=recovered.source_sequence,
                connection_generation=2,
            )
            assert len(core.store.read_observed(line_id)) == count_before_duplicate
            await _emit_real(
                real,
                clock,
                georeference,
                x_m=61.0,
                sim_time_s=3.1,
                received_monotonic_s=clock.monotonic_s - 2.0,
                source_sequence=recovered.source_sequence + 1,
                connection_generation=2,
                advance_s=0.0,
            )
            assert len(core.store.read_observed(line_id)) == count_before_duplicate

            observed_before_prediction = core._lines[line_id].state_set.observed
            clock.advance()
            await virtual.emit(
                latitude_deg=47.641468,
                longitude_deg=-122.140165,
                altitude_m=122.0,
                local_position_ned_m=(7.0, 0.0, -2.0),
                mission_phase="rehearsal",
                received_monotonic_s=clock.monotonic_s,
                sampled_at_utc=clock.wall,
                sim_time_s=scale * 20,
            )
            state_set = core._lines[line_id].state_set
            assert state_set.observed == observed_before_prediction
            assert state_set.predicted is not None
            assert state_set.predicted.source == TwinSource.PREDICTED
            assert not real.executions and not virtual.executions

        px4_records = core.store.read_observed("px4")
        apm_records = core.store.read_observed("apm")
        for records, line_id, vehicle_id in (
            (px4_records, "px4", 1),
            (apm_records, "apm", 2),
        ):
            assert [item.sequence for item in records] == list(range(1, len(records) + 1))
            assert all(item.line.value == line_id for item in records)
            assert all(item.vehicle_id == vehicle_id for item in records)
            assert all(item.source == TwinSource.OBSERVED for item in records)
            assert all(item.frame.value == "map" for item in records)
            assert all(item.frame_calibration_sha256 == georeference.config_hash for item in records)
            assert all(not item.preview_only for item in records)
            assert all(
                left.monotonic_time_s < right.monotonic_time_s
                and left.wall_time <= right.wall_time
                and (left.sim_time or 0.0) <= (right.sim_time or 0.0)
                for left, right in zip(records, records[1:])
            )
        assert abs(px4_records[0].position.x - apm_records[0].position.x) < 1e-6
        assert apm_records[1].sim_time - apm_records[0].sim_time == 30.0
        assert px4_records[1].sim_time - px4_records[0].sim_time == 3.0
        merged = core.store.merged_observed(["px4", "apm"])
        assert [item.wall_time for item in merged] == sorted(item.wall_time for item in merged)
        await core.stop()

    asyncio.run(scenario())


class FlakyMavlinkIO:
    def __init__(self) -> None:
        self.open_count = 0
        self.close_count = 0
        self.heartbeat_count = 0
        self.command_count = 0
        self._items: list[MavlinkEnvelope | Exception] = []
        self._generations = [
            [
                _message("HEARTBEAT", {"base_mode": 128, "mode_name": "POSCTL"}),
                _message("GPS_RAW_INT", {"fix_type": 3, "eph": 80, "satellites_visible": 12}),
                _position_message(1.0, 1_000),
                OSError("injected receive failure"),
            ],
            [
                _position_message(2.0, 100),
                _message("HEARTBEAT", {"base_mode": 128, "mode_name": "POSCTL"}),
                _message("GPS_RAW_INT", {"fix_type": 3, "eph": 80, "satellites_visible": 12}),
                _position_message(3.0, 200),
            ],
        ]

    async def open(self) -> None:
        self.open_count += 1
        self._items = list(self._generations[min(self.open_count - 1, 1)])

    async def close(self) -> None:
        self.close_count += 1

    async def recv(self, _timeout_s: float) -> MavlinkEnvelope | None:
        await asyncio.sleep(0)
        if not self._items:
            return None
        item = self._items.pop(0)
        if isinstance(item, Exception):
            raise item
        return item

    async def send_heartbeat(self) -> None:
        self.heartbeat_count += 1

    async def send_command_long(self, _command_id, _params) -> None:
        self.command_count += 1

    async def set_mode(self, _mode: str) -> None:
        self.command_count += 1

    async def mission_clear_all(self) -> None:
        self.command_count += 1

    async def mission_count(self, _count: int) -> None:
        self.command_count += 1

    async def mission_item_int(self, _item) -> None:
        self.command_count += 1

    async def mission_set_current(self, _sequence: int) -> None:
        self.command_count += 1


class BurstyMavlinkIO(FlakyMavlinkIO):
    """Fake socket whose buffered reads complete without yielding."""

    def __init__(self, items: list[MavlinkEnvelope]) -> None:
        self.open_count = 0
        self.close_count = 0
        self.heartbeat_count = 0
        self.command_count = 0
        self._items = items

    async def open(self) -> None:
        self.open_count += 1

    async def recv(self, _timeout_s: float) -> MavlinkEnvelope | None:
        if self._items:
            return self._items.pop(0)
        await asyncio.sleep(0)
        return None


def _message(name: str, fields: dict[str, object]) -> MavlinkEnvelope:
    return MavlinkEnvelope(
        name=name,
        fields=fields,
        source_system=1,
        source_component=1,
    )


def _position_message(north_m: float, time_boot_ms: int) -> MavlinkEnvelope:
    return _message(
        "GLOBAL_POSITION_INT",
        {
            "time_boot_ms": time_boot_ms,
            "lat": round((47.641468 + north_m / 111_111.0) * 10_000_000),
            "lon": round(-122.140165 * 10_000_000),
            "alt": 122_000,
            "vx": 0,
            "vy": 0,
            "vz": 0,
        },
    )


def test_mavlink_adapter_reconnects_and_does_not_reuse_cached_health():
    async def scenario() -> None:
        samples = []
        three_samples = asyncio.Event()

        async def receive(telemetry) -> None:
            samples.append(telemetry)
            if len(samples) >= 3:
                three_samples.set()

        io = FlakyMavlinkIO()
        adapter = Px4Adapter(
            endpoint=MavlinkEndpointConfig(
                endpoint="udpin:0.0.0.0:14540",
                target_system=1,
            ),
            georeference=_georeference(),
            line_id="px4",
            vehicle_id=1,
            platform_type="px4_multirotor",
            role=EndpointRole.REAL,
            telemetry_callback=receive,
            io=io,
            reconnect_delay_s=0.01,
            silence_timeout_s=0.1,
        )
        await adapter.start()
        await asyncio.wait_for(three_samples.wait(), timeout=1.0)
        await adapter.stop()

        assert io.open_count >= 2
        assert adapter.link_status()["reconnect_count"] >= 1
        assert [item.source_sequence for item in samples[:3]] == [1, 2, 3]
        assert [item.connection_generation for item in samples[:3]] == [1, 2, 2]
        first_after_reconnect = samples[1]
        assert first_after_reconnect.gps_fix_type is None
        assert first_after_reconnect.heartbeat_age_s is None
        assert first_after_reconnect.mode == "UNKNOWN"
        assert first_after_reconnect.mission_phase == "idle"
        assert not first_after_reconnect.link_ok
        assert samples[2].gps_fix_type == 3 and samples[2].link_ok
        assert io.command_count == 0

    asyncio.run(scenario())


def test_busy_mavlink_endpoint_does_not_starve_another_adapter():
    async def scenario() -> None:
        busy_io = BurstyMavlinkIO(
            [_message("SYS_STATUS", {"battery_remaining": 90}) for _ in range(5_000)]
        )
        probe_io = BurstyMavlinkIO(
            [
                _message("HEARTBEAT", {"base_mode": 0, "mode_name": "POSCTL"}),
                _message(
                    "GPS_RAW_INT",
                    {"fix_type": 3, "eph": 80, "satellites_visible": 12},
                ),
                _position_message(1.0, 1_000),
            ]
        )
        probe_received = asyncio.Event()

        async def receive(_telemetry) -> None:
            probe_received.set()

        common = {
            "georeference": _georeference(),
            "vehicle_id": 1,
            "platform_type": "px4_multirotor",
            "role": EndpointRole.REAL,
            "reconnect_delay_s": 0.01,
            "silence_timeout_s": 0.5,
        }
        busy = Px4Adapter(
            endpoint=MavlinkEndpointConfig(
                endpoint="udpin:0.0.0.0:14540", target_system=1
            ),
            line_id="busy",
            telemetry_callback=receive,
            io=busy_io,
            **common,
        )
        probe = Px4Adapter(
            endpoint=MavlinkEndpointConfig(
                endpoint="udpin:0.0.0.0:14541", target_system=1
            ),
            line_id="probe",
            telemetry_callback=receive,
            io=probe_io,
            **common,
        )
        await busy.start()
        await probe.start()
        await asyncio.wait_for(probe_received.wait(), timeout=0.5)
        remaining_when_probe_ran = len(busy_io._items)
        await busy.stop()
        await probe.stop()

        assert remaining_when_probe_ran > 0
        assert busy_io.command_count == 0 and probe_io.command_count == 0

    asyncio.run(scenario())


def test_adapter_sample_limit_still_consumes_latest_mavlink_state():
    async def scenario() -> None:
        positions = [_position_message(float(index), 1_000 + index) for index in range(100)]
        io = BurstyMavlinkIO(
            [
                _message("HEARTBEAT", {"base_mode": 0, "mode_name": "POSCTL"}),
                _message(
                    "GPS_RAW_INT",
                    {"fix_type": 3, "eph": 80, "satellites_visible": 12},
                ),
                *positions,
            ]
        )
        samples = []

        async def receive(telemetry) -> None:
            samples.append(telemetry)

        adapter = Px4Adapter(
            endpoint=MavlinkEndpointConfig(
                endpoint="udpin:0.0.0.0:14540", target_system=1
            ),
            georeference=_georeference(),
            line_id="px4",
            vehicle_id=1,
            platform_type="px4_multirotor",
            role=EndpointRole.REAL,
            telemetry_callback=receive,
            io=io,
            reconnect_delay_s=0.01,
            silence_timeout_s=0.5,
            sample_interval_s=0.2,
        )
        await adapter.start()
        deadline = asyncio.get_running_loop().time() + 0.5
        while io._items and asyncio.get_running_loop().time() < deadline:
            await asyncio.sleep(0)
        await adapter.stop()

        assert not io._items
        assert len(samples) == 1
        assert adapter._sim_time_s == 1.099
        assert adapter._global_position is not None
        assert adapter._global_position[0] > samples[0].global_position.latitude_deg
        assert io.command_count == 0

    asyncio.run(scenario())


def test_ardupilot_attitude_is_mapped_inside_its_adapter():
    async def scenario() -> None:
        samples = []

        async def receive(telemetry) -> None:
            samples.append(telemetry)

        adapter = ArduPilotAdapter(
            endpoint=MavlinkEndpointConfig(
                endpoint="udpin:0.0.0.0:14550",
                target_system=2,
                dialect="ardupilotmega",
            ),
            georeference=_georeference(),
            line_id="apm",
            vehicle_id=2,
            platform_type="apm_multirotor",
            role=EndpointRole.REAL,
            telemetry_callback=receive,
        )
        adapter._connection_generation = 1
        await adapter._handle_envelope(
            MavlinkEnvelope(
                name="ATTITUDE",
                fields={"time_boot_ms": 100, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
                source_system=2,
                source_component=1,
            )
        )
        await adapter._handle_envelope(
            MavlinkEnvelope(
                name="HEARTBEAT",
                fields={"base_mode": 0, "mode_name": "LOITER"},
                source_system=2,
                source_component=1,
            )
        )
        await adapter._handle_envelope(
            MavlinkEnvelope(
                name="GPS_RAW_INT",
                fields={"fix_type": 3, "eph": 80, "satellites_visible": 12},
                source_system=2,
                source_component=1,
            )
        )
        await adapter._handle_envelope(
            MavlinkEnvelope(
                name="GLOBAL_POSITION_INT",
                fields={
                    "time_boot_ms": 200,
                    "lat": 476_414_680,
                    "lon": -1_221_401_650,
                    "alt": 122_000,
                    "vx": 0,
                    "vy": 0,
                    "vz": 0,
                },
                source_system=2,
                source_component=1,
            )
        )
        assert len(samples) == 1
        assert samples[0].attitude_quaternion_wxyz == (1.0, 0.0, 0.0, 0.0)
        assert samples[0].gps_fix_type == 3

    asyncio.run(scenario())


def test_ardupilot_virtual_global_position_forms_predicted_sample():
    async def scenario() -> None:
        samples = []

        async def receive(telemetry) -> None:
            samples.append(telemetry)

        adapter = ArduPilotAdapter(
            endpoint=MavlinkEndpointConfig(
                endpoint="udpin:0.0.0.0:14551",
                target_system=2,
                dialect="ardupilotmega",
            ),
            georeference=_georeference(),
            line_id="apm",
            vehicle_id=2,
            platform_type="apm_multirotor",
            role=EndpointRole.VIRTUAL,
            telemetry_callback=receive,
        )
        adapter._connection_generation = 1
        await adapter._handle_envelope(
            MavlinkEnvelope(
                name="HEARTBEAT",
                fields={"base_mode": 0, "mode_name": "STABILIZE"},
                source_system=2,
                source_component=1,
            )
        )
        await adapter._handle_envelope(
            MavlinkEnvelope(
                name="GPS_RAW_INT",
                fields={"fix_type": 3, "eph": 80, "satellites_visible": 12},
                source_system=2,
                source_component=1,
            )
        )
        await adapter._handle_envelope(
            MavlinkEnvelope(
                name="GLOBAL_POSITION_INT",
                fields={
                    "time_boot_ms": 200,
                    "lat": 476_414_680,
                    "lon": -1_221_401_650,
                    "alt": 122_000,
                    "vx": 0,
                    "vy": 0,
                    "vz": 0,
                },
                source_system=2,
                source_component=1,
            )
        )
        assert len(samples) == 1
        assert samples[0].sample_kind == "global_position"
        assert samples[0].global_position is not None

    asyncio.run(scenario())


def test_ardupilot_virtual_rejects_local_and_unfixed_global_position():
    async def scenario() -> None:
        samples = []

        async def receive(telemetry) -> None:
            samples.append(telemetry)

        adapter = ArduPilotAdapter(
            endpoint=MavlinkEndpointConfig(
                endpoint="udpin:0.0.0.0:14551",
                target_system=2,
                dialect="ardupilotmega",
            ),
            georeference=_georeference(),
            line_id="apm",
            vehicle_id=2,
            platform_type="apm_multirotor",
            role=EndpointRole.VIRTUAL,
            telemetry_callback=receive,
        )
        adapter._connection_generation = 1
        await adapter._handle_envelope(
            MavlinkEnvelope(
                name="HEARTBEAT",
                fields={"base_mode": 0, "mode_name": "STABILIZE"},
                source_system=2,
                source_component=1,
            )
        )
        await adapter._handle_envelope(
            MavlinkEnvelope(
                name="LOCAL_POSITION_NED",
                fields={"time_boot_ms": 100, "x": 1.0, "y": 2.0, "z": -3.0},
                source_system=2,
                source_component=1,
            )
        )
        await adapter._handle_envelope(
            MavlinkEnvelope(
                name="GLOBAL_POSITION_INT",
                fields={"time_boot_ms": 200, "lat": 0, "lon": 0, "alt": 0},
                source_system=2,
                source_component=1,
            )
        )
        assert samples == []

    asyncio.run(scenario())


def test_fast_connection_generation_change_still_requires_recovery(tmp_path: Path):
    async def scenario() -> None:
        clock = ManualClock()
        georeference = _georeference()
        core = ParallelDomainCore(
            georeference=georeference,
            state_directory=str(tmp_path),
            telemetry_stale_after_s=3.0,
            recovery_samples=2,
            monotonic_clock=clock.monotonic,
            wall_clock=clock.wall_time,
        )
        real = FakeFlightControllerAdapter(
            kind=AdapterKind.PX4,
            line_id="px4",
            vehicle_id=1,
            role=EndpointRole.REAL,
            telemetry_callback=lambda telemetry: core.on_telemetry(
                "px4", EndpointRole.REAL, telemetry
            ),
        )
        virtual = FakeFlightControllerAdapter(
            kind=AdapterKind.PX4,
            line_id="px4",
            vehicle_id=1,
            role=EndpointRole.VIRTUAL,
            telemetry_callback=lambda telemetry: core.on_telemetry(
                "px4", EndpointRole.VIRTUAL, telemetry
            ),
        )
        core.add_line(
            _line("px4", 1, AdapterKind.PX4),
            real=real,
            virtual=virtual,
        )
        await core.start()
        await _emit_real(
            real,
            clock,
            georeference,
            x_m=1.0,
            sim_time_s=1.0,
            connection_generation=1,
        )
        before = core._lines["px4"].state_set.observed
        await _emit_real(
            real,
            clock,
            georeference,
            x_m=20.0,
            sim_time_s=0.1,
            connection_generation=2,
        )
        assert core._lines["px4"].mirror_status.state == MirrorState.RECOVERING
        assert core._lines["px4"].state_set.observed == before
        assert len(core.store.read_observed("px4")) == 1
        await _emit_real(
            real,
            clock,
            georeference,
            x_m=2.0,
            sim_time_s=1.1,
            connection_generation=2,
        )
        assert core._lines["px4"].mirror_status.state == MirrorState.FRESH
        assert len(core.store.read_observed("px4")) == 2
        await core.stop()

    asyncio.run(scenario())


def test_observation_only_service_never_processes_control_spool():
    class CoreProbe:
        def __init__(self) -> None:
            self.service: ParallelDomainService | None = None
            self.started = False
            self.stopped = False
            self.refresh_count = 0

        async def start(self) -> None:
            self.started = True

        async def refresh_freshness(self) -> None:
            self.refresh_count += 1
            assert self.service is not None
            self.service.stop()

        async def stop(self) -> None:
            self.stopped = True

    class ObservationService(ParallelDomainService):
        def __init__(self) -> None:
            self.config = type(
                "Config",
                (),
                {"observation_only": True, "poll_interval_s": 0.05},
            )()
            self.core = CoreProbe()
            self.core.service = self
            self._stop_event = asyncio.Event()
            self._seen_plans = set()
            self._seen_approvals = set()
            self.spool_calls = 0

        async def _process_spool(self) -> None:
            self.spool_calls += 1

    async def scenario() -> None:
        service = ObservationService()
        await service.run()
        assert service.core.started and service.core.stopped
        assert service.core.refresh_count == 1
        assert service.spool_calls == 0

    asyncio.run(scenario())

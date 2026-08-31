from datetime import datetime, timedelta, timezone

import pytest
from pydantic import ValidationError

from aeromind_apm_lite.common.contracts import (
    CoordinateFrame,
    FidelityLevel,
    TwinLine,
    TwinSource,
    TwinState,
    TwinStateSet,
    Vector3,
)
from aeromind_apm_lite.common.coordinates import (
    CalibrationStatus,
    GeoReference,
    Vec3,
    VehicleHome,
    Wgs84Position,
    ecef_to_enu,
    ecef_to_wgs84,
    enu_to_ecef,
    wgs84_to_ecef,
)
from aeromind_apm_lite.common.timing import (
    ClockStamp,
    DualClockStamp,
    MonotonicClock,
)


NOW = datetime(2026, 8, 16, 8, 0, tzinfo=timezone.utc)


def reference(*, status=CalibrationStatus.SURVEYED):
    origin = Wgs84Position(
        latitude_deg=47.641468,
        longitude_deg=-122.140165,
        altitude_m=122.0,
    )
    return GeoReference(
        calibration_id="parallel-map-20260816-v1",
        status=status,
        map_origin_wgs84=origin,
        map_x_heading_from_true_north_deg=0.0,
        airsim_origin_wgs84=origin,
        vehicle_homes=(
            VehicleHome(vehicle_id=1, map_position_m=(0.0, 0.0, 0.0)),
            VehicleHome(vehicle_id=2, map_position_m=(0.0, 3.0, 0.0)),
        ),
    )


def state(source: TwinSource, *, line=TwinLine.PX4, offset=0.0):
    wall_time = NOW + timedelta(seconds=offset)
    return TwinState(
        twin_id=f"{line.value}-1",
        vehicle_id=1,
        line=line,
        platform_type="multirotor",
        frame=CoordinateFrame.MAP,
        frame_calibration_id="parallel-map-20260816-v1",
        position=Vector3(x=offset, y=2.0, z=3.0),
        attitude=None,
        mode="AUTO",
        task_phase="survey",
        source=source,
        confidence=0.95,
        fidelity=FidelityLevel.L2,
        sim_time=offset if source in {TwinSource.PREDICTED, TwinSource.PLANNED} else None,
        wall_time=wall_time,
        received_monotonic_s=100.0 + offset,
        received_wall_time=wall_time,
        mirrored_monotonic_s=100.01 + offset,
        mirrored_wall_time=wall_time + timedelta(milliseconds=10),
    )


def test_new_twin_contract_fields_and_json_schema_are_public():
    value = state(TwinSource.PREDICTED)

    assert value.position == value.position_m
    assert value.task_phase == value.mission_phase
    assert value.line == TwinLine.PX4
    schema = TwinState.model_json_schema()
    for field_name in (
        "twin_id",
        "vehicle_id",
        "line",
        "source",
        "frame",
        "position",
        "attitude",
        "mode",
        "task_phase",
        "sim_time",
        "wall_time",
        "confidence",
        "fidelity",
    ):
        assert field_name in schema["properties"]
    assert {"twin_id", "line", "position", "task_phase", "wall_time"} <= set(
        schema["required"]
    )


def test_planned_predicted_and_observed_slots_are_independent():
    observed = state(TwinSource.OBSERVED)
    collection = TwinStateSet(
        twin_id=observed.twin_id,
        vehicle_id=observed.vehicle_id,
        line=observed.line,
        observed=observed,
    )

    updated = collection.updated(state(TwinSource.PREDICTED, offset=1.0))

    assert updated.observed == observed
    assert updated.predicted is not None
    with pytest.raises(ValidationError, match="predicted slot"):
        TwinStateSet(
            twin_id=observed.twin_id,
            vehicle_id=observed.vehicle_id,
            line=observed.line,
            predicted=observed,
        )


def test_ecef_enu_and_map_wgs84_round_trips_are_below_one_micrometre():
    georeference = reference()
    point_map = Vec3(42.5, -17.25, 8.75)
    gps = georeference.map_to_wgs84(point_map)
    decoded_map = georeference.wgs84_to_map(gps)
    origin = georeference.map_origin_wgs84.coordinate()
    point_ecef = wgs84_to_ecef(gps.coordinate())
    point_enu = ecef_to_enu(point_ecef, origin)
    decoded_ecef = enu_to_ecef(point_enu, origin)
    decoded_gps = ecef_to_wgs84(decoded_ecef)

    assert _distance(point_map, decoded_map) < 1e-6
    assert _distance(point_ecef, decoded_ecef) < 1e-6
    gps_error = _distance(
        point_enu,
        ecef_to_enu(wgs84_to_ecef(decoded_gps), origin),
    )
    assert gps_error < 1e-6


def test_same_gps_is_identical_for_px4_and_apm_map_normalization():
    georeference = reference()
    gps = georeference.map_to_wgs84(Vec3(12.0, 9.0, 4.0))

    px4_map = georeference.telemetry_wgs84_to_map(gps, line="px4")
    apm_map = georeference.telemetry_wgs84_to_map(gps, line="apm")

    assert _distance(px4_map, apm_map) == 0.0


def test_clock_stamps_are_utc_and_monotonic():
    values = iter((10.0, 10.1, 10.2))
    clock = MonotonicClock(monotonic=lambda: next(values), wall_clock=lambda: NOW)
    stamps = [clock.stamp(sim_time_s=index * 0.1) for index in range(3)]

    assert [item.monotonic_s for item in stamps] == sorted(
        item.monotonic_s for item in stamps
    )
    assert all(item.wall_time.tzinfo == timezone.utc for item in stamps)
    DualClockStamp(received=stamps[0], mirrored=stamps[1])

    with pytest.raises(ValidationError, match="must not precede"):
        DualClockStamp(received=stamps[1], mirrored=stamps[0])
    with pytest.raises(ValidationError, match="timezone"):
        ClockStamp(monotonic_s=1.0, wall_time=datetime.now(), sim_time_s=0.0)


def test_draft_calibration_is_preview_only_even_when_math_round_trip_passes():
    georeference = reference(status=CalibrationStatus.DRAFT)
    report = georeference.coordinate_round_trip_report()

    assert georeference.preview_only
    assert report["passed"]
    assert report["preview_only"]
    assert report["acceptance_passed"] is None
    assert georeference.sha256 == georeference.calibration_sha256
    assert len(georeference.sha256) == 64


def _distance(left: Vec3, right: Vec3) -> float:
    return (
        (left.x - right.x) ** 2
        + (left.y - right.y) ** 2
        + (left.z - right.z) ** 2
    ) ** 0.5

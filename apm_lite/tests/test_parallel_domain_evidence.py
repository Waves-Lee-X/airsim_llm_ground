from datetime import datetime, timedelta, timezone
from pathlib import Path
from uuid import uuid4

import pytest

from aeromind_apm_lite.common.contracts import (
    CoordinateFrame,
    FidelityLevel,
    HealthStatus,
    TwinLine,
    TwinSource,
    TwinState,
)
from aeromind_apm_lite.common.contracts.models import Quaternion, Vector3
from aeromind_apm_lite.common.coordinates import (
    CalibrationStatus,
    GeoReference,
    VehicleHome,
    Wgs84Position,
)
from aeromind_apm_lite.common.trajectory import ArtifactVersions
from aeromind_apm_lite.ground.parallel_domain.calibration import (
    CalibrationDirection,
    CalibrationError,
    CalibrationTruthSource,
    LiveCalibrationDataset,
    calibration_measurement_from_gazebo,
    freeze_georeference,
    measure_calibration,
    parse_gazebo_pose_tuple,
)
from aeromind_apm_lite.ground.parallel_domain.models import ObservedReplayRecord


ROOT = Path(__file__).resolve().parents[1]


def _reference() -> GeoReference:
    origin = Wgs84Position(
        latitude_deg=47.641468,
        longitude_deg=-122.140165,
        altitude_m=122.0,
    )
    return GeoReference(
        calibration_id="test-live-calibration-v1",
        status=CalibrationStatus.SURVEYED,
        map_origin_wgs84=origin,
        map_x_heading_from_true_north_deg=0.0,
        airsim_origin_wgs84=origin,
        vehicle_homes=(VehicleHome(vehicle_id=1, map_position_m=(0.0, 0.0, 0.0)),),
    )


def _dataset(reference: GeoReference) -> LiveCalibrationDataset:
    payload = {
        "calibration_id": reference.calibration_id,
        "line": "px4",
        "vehicle_id": 1,
        "truth_source": CalibrationTruthSource.SIMULATION_GROUND_TRUTH,
        "truth_reference": "independent Gazebo model pose fixture",
        "created_at_utc": datetime(2026, 8, 17, tzinfo=timezone.utc),
        "measurements": [
            {
                "control_point_id": "CP0",
                "pass_id": "outbound-1",
                "direction": "outbound",
                "captured_at_utc": datetime(2026, 8, 17, 0, 0, 1, tzinfo=timezone.utc),
                "measured_wgs84": reference.map_to_wgs84(Vector3(x=0.02, y=0.0, z=0.01)),
                "ground_truth_map_m": (0.0, 0.0, 0.0),
            },
            {
                "control_point_id": "CP1",
                "pass_id": "outbound-1",
                "direction": "outbound",
                "captured_at_utc": datetime(2026, 8, 17, 0, 0, 2, tzinfo=timezone.utc),
                "measured_wgs84": reference.map_to_wgs84(Vector3(x=10.03, y=0.0, z=2.01)),
                "ground_truth_map_m": (10.0, 0.0, 2.0),
            },
            {
                "control_point_id": "CP1",
                "pass_id": "return-1",
                "direction": "return",
                "captured_at_utc": datetime(2026, 8, 17, 0, 0, 3, tzinfo=timezone.utc),
                "measured_wgs84": reference.map_to_wgs84(Vector3(x=9.98, y=0.01, z=1.99)),
                "ground_truth_map_m": (10.0, 0.0, 2.0),
            },
            {
                "control_point_id": "CP0",
                "pass_id": "return-1",
                "direction": "return",
                "captured_at_utc": datetime(2026, 8, 17, 0, 0, 4, tzinfo=timezone.utc),
                "measured_wgs84": reference.map_to_wgs84(Vector3(x=0.01, y=0.0, z=0.0)),
                "ground_truth_map_m": (0.0, 0.0, 0.0),
            },
            {
                "control_point_id": "CP2",
                "pass_id": "outbound-1",
                "direction": "outbound",
                "captured_at_utc": datetime(2026, 8, 17, 0, 0, 5, tzinfo=timezone.utc),
                "measured_wgs84": reference.map_to_wgs84(Vector3(x=10.0, y=10.02, z=3.0)),
                "ground_truth_map_m": (10.0, 10.0, 3.0),
            },
            {
                "control_point_id": "CP2",
                "pass_id": "return-1",
                "direction": "return",
                "captured_at_utc": datetime(2026, 8, 17, 0, 0, 6, tzinfo=timezone.utc),
                "measured_wgs84": reference.map_to_wgs84(Vector3(x=10.01, y=9.98, z=3.02)),
                "ground_truth_map_m": (10.0, 10.0, 3.0),
            },
        ],
    }
    return LiveCalibrationDataset.model_validate(payload)


def _replay(reference: GeoReference, wall_time: datetime) -> ObservedReplayRecord:
    position = Vector3(x=1.0, y=2.0, z=3.0)
    state = TwinState(
        twin_id="px4-1",
        vehicle_id=1,
        line=TwinLine.PX4,
        platform_type="px4_multirotor",
        frame=CoordinateFrame.MAP,
        frame_calibration_id=reference.calibration_id,
        position=position,
        position_m=position,
        attitude=Quaternion(w=1.0, x=0.0, y=0.0, z=0.0),
        energy_remaining=1.0,
        communication_quality=1.0,
        mode="LOITER",
        task_phase="manual",
        mission_phase="manual",
        health_status=HealthStatus.NOMINAL,
        source=TwinSource.OBSERVED,
        confidence=1.0,
        fidelity=FidelityLevel.L2,
        sim_time=4.0,
        wall_time=wall_time,
        received_monotonic_s=10.0,
        received_wall_time=wall_time,
        mirrored_monotonic_s=10.1,
        mirrored_wall_time=wall_time + timedelta(milliseconds=1),
        sampled_at_utc=wall_time,
    )
    return ObservedReplayRecord(
        sequence=1,
        source_sequence=1,
        twin_id="px4-1",
        vehicle_id=1,
        line=TwinLine.PX4,
        frame=CoordinateFrame.MAP,
        frame_calibration_id=reference.calibration_id,
        frame_calibration_sha256=reference.config_hash,
        calibration_status=CalibrationStatus.SURVEYED,
        preview_only=False,
        position=position,
        attitude=state.attitude,
        mode="LOITER",
        task_phase="manual",
        wall_time=wall_time,
        sim_time=4.0,
        monotonic_time_s=10.0,
        mirrored_monotonic_s=10.1,
        mirrored_wall_time=wall_time + timedelta(milliseconds=1),
        state=state,
    )


def test_roundtrip_fixture_reports_live_error_not_zero_and_freezes(tmp_path):
    draft = GeoReference.model_validate(
        __import__("yaml").safe_load(
            (ROOT / "configs/parallel_domain/sitl-georeference-draft.yaml").read_text()
        )
    )
    dataset = __import__(
        "aeromind_apm_lite.ground.parallel_domain.calibration",
        fromlist=["load_calibration_dataset"],
    ).load_calibration_dataset(
        ROOT / "configs/parallel_domain/calibration/sitl-ground-truth-roundtrip.json"
    )
    report = measure_calibration(draft, dataset)
    assert report.preview_only is True
    assert report.acceptance_passed is None
    assert report.distribution.three_dimensional_rmse_m > 0.0
    assert "live" in report.result_statement_zh

    frozen, frozen_report, receipt = freeze_georeference(draft, dataset)
    assert frozen.status == CalibrationStatus.SURVEYED
    assert frozen.preview_only is False
    assert frozen_report.acceptance_passed is True
    assert receipt.frozen_calibration_sha256 == frozen.config_hash


def test_freeze_rejects_missing_return_pass():
    reference = _reference().model_copy(update={"status": CalibrationStatus.DRAFT})
    dataset = _dataset(reference).model_copy(
        update={
            "measurements": tuple(
                item
                for item in _dataset(reference).measurements
                if item.direction == CalibrationDirection.OUTBOUND
            )
        }
    )
    with pytest.raises(CalibrationError, match="round_trip_passes"):
        freeze_georeference(reference, dataset)


def test_gazebo_measurement_requires_independent_wall_time_and_hash():
    reference = _reference()
    wall = datetime(2026, 8, 17, tzinfo=timezone.utc)
    replay = _replay(reference, wall)
    measurement = calibration_measurement_from_gazebo(
        reference=reference,
        replay=replay,
        gazebo_position_enu_m=parse_gazebo_pose_tuple("-2 1 3 0 0 0"),
        ground_truth_captured_at_utc=wall + timedelta(milliseconds=100),
        ground_truth_reference="independent Gazebo pose",
        control_point_id="CP-live",
        pass_id="outbound-1",
        direction=CalibrationDirection.OUTBOUND,
    )
    assert measurement.pairing_skew_s == pytest.approx(0.1)
    assert measurement.ground_truth_map_m != tuple(replay.position.model_dump().values())
    with pytest.raises(CalibrationError, match="skew"):
        calibration_measurement_from_gazebo(
            reference=reference,
            replay=replay,
            gazebo_position_enu_m=Vector3(x=0.0, y=0.0, z=0.0),
            ground_truth_captured_at_utc=wall + timedelta(seconds=1),
            ground_truth_reference="independent Gazebo pose",
            control_point_id="CP-live",
            pass_id="outbound-1",
            direction=CalibrationDirection.OUTBOUND,
        )

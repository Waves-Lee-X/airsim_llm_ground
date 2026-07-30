import json
from datetime import datetime, timedelta, timezone
from types import MappingProxyType
from uuid import uuid4

import pytest
from pydantic import ValidationError

from aeromind_apm_lite.common.coordinates import (
    CalibrationStatus,
    GeoReference,
    VehicleHome,
    Wgs84Position,
)
from aeromind_apm_lite.common.trajectory import (
    ArtifactVersions,
    GpsReplayError,
    TrajectoryEvidence,
    TrajectoryEvidenceError,
    TrajectoryEvidenceStore,
    TrajectoryRole,
    TrajectorySample,
    TrajectorySource,
    TrajectoryThresholds,
    compare_trajectories,
    load_evidence_file,
    replay_gps_csv,
)
from aeromind_apm_lite.ground.trajectory.cli import main as trajectory_cli_main
from aeromind_apm_lite.ground.simulation.trajectory_recording import (
    NavigationTrajectoryRecorder,
)
from aeromind_apm_lite.onboard import NavigationMissionPlan
from aeromind_apm_lite.onboard.mavlink import TelemetrySnapshot


MISSION_ID = uuid4()
HASH_A = "a" * 64
HASH_B = "b" * 64
HASH_C = "c" * 64
HASH_D = "d" * 64


def surveyed_reference():
    return GeoReference(
        calibration_id="test-field-v1",
        status=CalibrationStatus.SURVEYED,
        map_origin_wgs84=Wgs84Position(
            latitude_deg=34.7472,
            longitude_deg=113.6253,
            altitude_m=112.4,
        ),
        map_x_heading_from_true_north_deg=0.0,
        airsim_origin_wgs84=Wgs84Position(
            latitude_deg=34.7472,
            longitude_deg=113.6253,
            altitude_m=112.4,
        ),
        vehicle_homes=(VehicleHome(vehicle_id=1, map_position_m=(0.0, 0.0, 0.0)),),
    )


def complete_versions():
    return ArtifactVersions(
        mission_plan_hash=HASH_A,
        software_commit="8605422",
        arducopter_version="ArduCopter-4.5-test",
        parameter_hash=HASH_B,
        scene_hash=HASH_C,
    )


def evidence(role, *, offset=(0.0, 0.0, 0.0), reference=None, versions=None):
    reference = reference or surveyed_reference()
    return TrajectoryEvidence(
        mission_id=MISSION_ID,
        vehicle_id=1,
        role=role,
        source={
            TrajectoryRole.PLANNED: TrajectorySource.MISSION_PLAN,
            TrajectoryRole.PREDICTED: TrajectorySource.SITL,
            TrajectoryRole.OBSERVED: TrajectorySource.REAL_TELEMETRY,
        }[role],
        frame_calibration_id=reference.calibration_id,
        frame_calibration_hash=reference.config_hash,
        calibration_status=reference.status,
        preview_only=False,
        producer_version="trajectory-test-v1",
        versions=versions or complete_versions(),
        samples=tuple(
            TrajectorySample(
                time_from_start_ms=index * 1_000,
                position_map_m=(index + offset[0], offset[1], offset[2]),
            )
            for index in range(3)
        ),
    )


def test_trajectory_evidence_hash_is_stable_and_store_is_immutable(tmp_path):
    original = evidence(TrajectoryRole.PLANNED)
    equivalent = TrajectoryEvidence.model_validate(original.model_dump(mode="json"))
    store = TrajectoryEvidenceStore(tmp_path)

    assert original.evidence_hash == equivalent.evidence_hash
    assert store.save(original) == original
    assert store.load(original.evidence_id) == original
    assert store.list_summaries()[0]["evidence_hash"] == original.evidence_hash
    assert not list(tmp_path.glob("*.tmp"))

    changed = original.model_copy(update={"producer_version": "changed"})
    with pytest.raises(TrajectoryEvidenceError, match="immutable"):
        store.save(changed)

    path = tmp_path / f"{original.evidence_id}.json"
    payload = json.loads(path.read_text(encoding="utf-8"))
    payload["evidence_hash"] = "0" * 64
    path.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(TrajectoryEvidenceError, match="hash mismatch"):
        store.load(original.evidence_id)


def test_trajectory_evidence_rejects_non_monotonic_samples_and_draft_final_use():
    payload = evidence(TrajectoryRole.PLANNED).model_dump(mode="json")
    payload["samples"][1]["time_from_start_ms"] = 0
    with pytest.raises(ValidationError, match="strictly increasing"):
        TrajectoryEvidence.model_validate(payload)

    payload = evidence(TrajectoryRole.PLANNED).model_dump(mode="json")
    payload["calibration_status"] = "draft"
    with pytest.raises(ValidationError, match="preview_only"):
        TrajectoryEvidence.model_validate(payload)


def test_comparison_reports_all_three_pairs_and_known_errors():
    planned = evidence(TrajectoryRole.PLANNED)
    predicted = evidence(TrajectoryRole.PREDICTED, offset=(1.0, 0.0, 0.0))
    observed = evidence(TrajectoryRole.OBSERVED, offset=(0.0, 0.0, 2.0))
    report = compare_trajectories(
        [planned, predicted, observed],
        TrajectoryThresholds(validated_for_venue=True),
    )

    assert report.ready
    assert not report.preview_only
    assert report.metrics_within_thresholds
    assert report.acceptance_passed is True
    assert len(report.comparisons) == 3
    by_pair = {
        (item.reference_role, item.compared_role): item for item in report.comparisons
    }
    planned_predicted = by_pair[(TrajectoryRole.PLANNED, TrajectoryRole.PREDICTED)]
    assert planned_predicted.horizontal_rmse_m == pytest.approx(1.0)
    assert planned_predicted.vertical_rmse_m == pytest.approx(0.0)
    planned_observed = by_pair[(TrajectoryRole.PLANNED, TrajectoryRole.OBSERVED)]
    assert planned_observed.vertical_rmse_m == pytest.approx(2.0)
    assert len(report.report_hash) == 64


def test_comparison_stays_preview_without_all_roles_versions_and_validated_thresholds():
    planned = evidence(
        TrajectoryRole.PLANNED,
        versions=ArtifactVersions(),
    )
    predicted = evidence(TrajectoryRole.PREDICTED)
    report = compare_trajectories([planned, predicted])

    assert not report.ready
    assert report.preview_only
    assert report.acceptance_passed is None
    assert any("missing trajectory roles" in item for item in report.limitations)
    assert any("missing reproduction metadata" in item for item in report.limitations)


def test_comparison_rejects_calibration_hash_mismatch():
    planned = evidence(TrajectoryRole.PLANNED)
    predicted = evidence(TrajectoryRole.PREDICTED).model_copy(
        update={"frame_calibration_hash": HASH_D}
    )

    with pytest.raises(TrajectoryEvidenceError, match="calibration hashes"):
        compare_trajectories([planned, predicted])


def test_gps_csv_replay_preserves_raw_gps_and_normalizes_to_map(tmp_path):
    source = tmp_path / "uav1-gps.csv"
    source.write_text(
        "observed_at_utc,latitude_deg,longitude_deg,altitude_m,gps_fix_type,gps_hdop,"
        "satellites_visible\n"
        "2026-07-30T10:00:00Z,34.7472,113.6253,112.4,3,0.9,14\n"
        "2026-07-30T10:00:00Z,34.7472,113.6253,112.4,3,0.9,14\n"
        "2026-07-30T10:00:01Z,34.74721,113.6253,113.4,3,1.0,13\n",
        encoding="utf-8",
    )

    result = replay_gps_csv(
        source,
        surveyed_reference(),
        mission_id=MISSION_ID,
        vehicle_id=1,
        producer_version="test",
        versions=complete_versions(),
    )

    assert result.role == TrajectoryRole.OBSERVED
    assert not result.preview_only
    assert result.samples[0].position_map_m == pytest.approx((0.0, 0.0, 0.0), abs=1e-6)
    assert result.samples[1].position_map_m[0] == pytest.approx(1.109, abs=0.02)
    assert result.samples[1].position_map_m[2] == pytest.approx(1.0, abs=0.01)
    assert result.samples[0].source_wgs84.latitude_deg == pytest.approx(34.7472)
    assert result.samples[0].gps_quality.fix_type == 3
    assert result.source_metadata["duplicate_timestamp_count"] == 1
    assert len(result.source_metadata["source_file_sha256"]) == 64


def test_gps_replay_rejects_draft_unless_explicitly_previewed(tmp_path):
    source = tmp_path / "gps.csv"
    start = datetime(2026, 7, 30, tzinfo=timezone.utc)
    source.write_text(
        "timestamp,lat,lon,alt\n"
        f"{start.isoformat()},34.7472,113.6253,112.4\n"
        f"{(start + timedelta(seconds=1)).isoformat()},34.74721,113.6253,112.4\n",
        encoding="utf-8",
    )
    reference = surveyed_reference().model_copy(update={"status": CalibrationStatus.DRAFT})

    with pytest.raises(GpsReplayError, match="surveyed calibration"):
        replay_gps_csv(
            source,
            reference,
            mission_id=MISSION_ID,
            vehicle_id=1,
            producer_version="test",
        )
    preview = replay_gps_csv(
        source,
        reference,
        mission_id=MISSION_ID,
        vehicle_id=1,
        producer_version="test",
        allow_draft=True,
    )
    assert preview.preview_only


def test_trajectory_cli_replays_gps_to_hashed_json(tmp_path):
    source = tmp_path / "gps.csv"
    source.write_text(
        "timestamp_utc,latitude_deg,longitude_deg,altitude_m\n"
        "2026-07-30T10:00:00+00:00,34.7472,113.6253,112.4\n"
        "2026-07-30T10:00:01+00:00,34.74721,113.6253,112.4\n",
        encoding="utf-8",
    )
    config = tmp_path / "venue.json"
    config.write_text(
        json.dumps(surveyed_reference().model_dump(mode="json")),
        encoding="utf-8",
    )
    output = tmp_path / "observed.json"

    result = trajectory_cli_main(
        [
            "replay-gps",
            "--input",
            str(source),
            "--georeference",
            str(config),
            "--mission-id",
            str(MISSION_ID),
            "--vehicle-id",
            "1",
            "--producer-version",
            "test",
            "--output",
            str(output),
        ]
    )

    assert result == 0
    assert load_evidence_file(output).role == TrajectoryRole.OBSERVED


def test_navigation_recorder_builds_aligned_planned_and_predicted_evidence():
    plan = NavigationMissionPlan(target_position_ned_m=(5.0, 2.0, -2.0))
    recorder = NavigationTrajectoryRecorder(
        surveyed_reference(),
        plan,
        vehicle_id=1,
        producer_version="test",
        versions=complete_versions(),
        started_at_utc=datetime(2026, 7, 30, tzinfo=timezone.utc),
    )
    recorder.record_snapshot("preflight", telemetry_snapshot((0.0, 0.0, 0.0)), 10.0)
    recorder.record_snapshot("goto", telemetry_snapshot((5.0, 2.0, -2.0)), 11.0)
    recorder.record_snapshot("landed", telemetry_snapshot((0.0, 0.0, 0.0)), 12.0)

    planned, predicted = recorder.finalize()

    assert planned.role == TrajectoryRole.PLANNED
    assert predicted.role == TrajectoryRole.PREDICTED
    assert [sample.phase for sample in predicted.samples] == [
        "preflight",
        "goto",
        "landed",
    ]
    assert predicted.samples[1].position_map_m == pytest.approx((5.0, -2.0, 2.0))
    assert planned.samples[1].position_map_m == pytest.approx((5.0, -2.0, 2.0))
    assert planned.samples[2].position_map_m == pytest.approx((5.0, -2.0, 2.0))
    assert planned.versions.mission_plan_hash is not None
    assert predicted.samples[1].observed_at_utc == datetime(
        2026, 7, 30, 0, 0, 1, tzinfo=timezone.utc
    )


def telemetry_snapshot(local_position):
    return TelemetrySnapshot(
        observed_monotonic_s=1.0,
        fcu_link_ok=True,
        armed=False,
        mode="GUIDED",
        relative_altitude_m=-local_position[2],
        local_position_ned_m=local_position,
        velocity_ned_m_s=(0.0, 0.0, 0.0),
        global_position_deg_m=(34.7472, 113.6253, 112.4 - local_position[2]),
        attitude_rpy_rad=(0.0, 0.0, 0.0),
        battery_remaining=0.8,
        battery_voltage_v=15.0,
        gps_fix_type=3,
        satellites_visible=14,
        gps_hdop=0.8,
        gps_healthy=True,
        prearm_ok=True,
        ekf_flags=831,
        landed_state=1,
        home_position_deg_m=(34.7472, 113.6253, 112.4),
        home_position_ned_m=(0.0, 0.0, 0.0),
        last_status_text=None,
        last_heartbeat_monotonic_s=1.0,
        field_ages_s=MappingProxyType({}),
    )

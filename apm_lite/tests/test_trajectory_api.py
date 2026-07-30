from uuid import uuid4

from fastapi.testclient import TestClient

from aeromind_apm_lite.common.coordinates import (
    CalibrationStatus,
    GeoReference,
    GeoReferenceStore,
    VehicleHome,
    Wgs84Position,
)
from aeromind_apm_lite.common.trajectory import (
    ArtifactVersions,
    TrajectoryEvidence,
    TrajectoryEvidenceStore,
    TrajectoryRole,
    TrajectorySample,
    TrajectorySource,
)
from aeromind_apm_lite.ground.browser.app import create_app
from aeromind_apm_lite.ground.browser.runtime import (
    ManualRuntime,
    ManualRuntimeConfig,
    ManualRuntimeMode,
)


def surveyed_reference():
    return GeoReference(
        calibration_id="api-field-v1",
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


def trajectory_evidence(reference, mission_id, role):
    return TrajectoryEvidence(
        mission_id=mission_id,
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
        producer_version="api-test-v1",
        versions=ArtifactVersions(
            mission_plan_hash="a" * 64,
            software_commit="test-commit",
            arducopter_version="test-copter",
            parameter_hash="b" * 64,
            scene_hash="c" * 64,
        ),
        samples=(
            TrajectorySample(time_from_start_ms=0, position_map_m=(0.0, 0.0, 0.0)),
            TrajectorySample(time_from_start_ms=1_000, position_map_m=(1.0, 0.0, 0.0)),
        ),
    )


def trajectory_app(tmp_path):
    reference = surveyed_reference()
    georeference = GeoReferenceStore(tmp_path / "venue.yaml")
    georeference.save(reference)
    runtime = ManualRuntime(
        ManualRuntimeConfig(
            mode=ManualRuntimeMode.DEMO,
            startup_timeout_s=2.0,
        )
    )
    return (
        create_app(
            runtime,
            georeference=georeference,
            trajectory_evidence=TrajectoryEvidenceStore(tmp_path / "evidence"),
        ),
        reference,
    )


def test_trajectory_api_saves_lists_loads_and_compares_three_roles(tmp_path):
    app, reference = trajectory_app(tmp_path)
    mission_id = uuid4()
    evidence = [
        trajectory_evidence(reference, mission_id, role) for role in TrajectoryRole
    ]

    with TestClient(app) as client:
        for item in evidence:
            response = client.post(
                "/api/trajectory/evidence",
                json=item.model_dump(mode="json"),
            )
            assert response.status_code == 201
            assert response.json()["evidence_hash"] == item.evidence_hash

        listed = client.get("/api/trajectory/evidence").json()["items"]
        assert len(listed) == 3
        loaded = client.get(
            f"/api/trajectory/evidence/{evidence[0].evidence_id}"
        )
        assert loaded.status_code == 200
        assert loaded.json()["evidence"]["role"] == "planned"

        report = client.post(
            "/api/trajectory/reports",
            json={
                "evidence_ids": [str(item.evidence_id) for item in evidence],
                "thresholds": {"validated_for_venue": True},
            },
        )
        assert report.status_code == 200
        result = report.json()["report"]
        assert result["ready"]
        assert result["acceptance_passed"] is True
        assert len(result["comparisons"]) == 3


def test_trajectory_api_rejects_inactive_calibration_and_duplicate_report_ids(tmp_path):
    app, reference = trajectory_app(tmp_path)
    item = trajectory_evidence(reference, uuid4(), TrajectoryRole.PLANNED)
    mismatch = item.model_copy(update={"frame_calibration_hash": "d" * 64})

    with TestClient(app) as client:
        rejected = client.post(
            "/api/trajectory/evidence",
            json=mismatch.model_dump(mode="json"),
        )
        assert rejected.status_code == 409
        assert "active GeoReference" in rejected.json()["detail"]

        assert client.post(
            "/api/trajectory/evidence",
            json=item.model_dump(mode="json"),
        ).status_code == 201
        duplicate = client.post(
            "/api/trajectory/reports",
            json={"evidence_ids": [str(item.evidence_id), str(item.evidence_id)]},
        )
        assert duplicate.status_code == 422
        assert "unique" in duplicate.json()["detail"]

from __future__ import annotations

import json
import zipfile
from datetime import datetime, timezone
from pathlib import Path
from types import SimpleNamespace
from uuid import uuid4

from aeromind_apm_lite.common.trajectory import (
    TrajectoryComparisonReport,
    TrajectoryPairMetrics,
    TrajectoryRole,
    TrajectoryThresholds,
)
from aeromind_apm_lite.ground.parallel_domain.models import MissionAction
from tools.end_to_end_demo import (
    comparison_report_pairs,
    make_mission,
    selected_lines,
    trajectory_error_curve,
    validate_airsim_manifest,
    write_same_screen_report,
)


def test_line_selection_and_each_run_gets_fresh_mission_ids():
    assert selected_lines("px4") == ("px4",)
    assert selected_lines("apm") == ("apm",)
    assert selected_lines("all") == ("px4", "apm")

    legal = make_mission("px4")
    repeated = make_mission("px4")
    denied = make_mission("apm", unauthorized=True)
    assert legal.mission_id != repeated.mission_id
    assert legal.steps[-1].action == MissionAction.LAND
    assert denied.steps[-1].action == MissionAction.RTL


def test_airsim_manifest_requires_live_isolated_profiles():
    payload = {
        "standard_settings_sha256_before": "abc",
        "standard_settings_sha256_after": "abc",
        "instances": [
            {
                "line": "px4",
                "pid": 101,
                "rpc_port": 41451,
                "settings_path": r"C:\profiles\px4\settings.json",
                "settings_sha256": "1" * 64,
                "arguments": [r"-settings=C:\profiles\px4\settings.json"],
            },
            {
                "line": "apm",
                "pid": 102,
                "rpc_port": 41452,
                "settings_path": r"C:\profiles\apm\settings.json",
                "settings_sha256": "2" * 64,
                "arguments": [r"-settings=C:\profiles\apm\settings.json"],
            },
        ],
    }
    live = {
        "px4": {
            "process_alive": True,
            "rpc_listening": True,
            "rpc_process_id": 201,
            "rpc_settings_match": True,
        },
        "apm": {
            "process_alive": True,
            "rpc_listening": True,
            "rpc_process_id": 202,
            "rpc_settings_match": True,
        },
    }
    result = validate_airsim_manifest(payload, ("px4", "apm"), live)
    assert result["passed"] is True
    assert result["settings_profiles_isolated"] is True
    assert result["standard_settings_untouched"] is True

    payload["instances"][1]["settings_path"] = payload["instances"][0]["settings_path"]
    result = validate_airsim_manifest(payload, ("px4", "apm"), live)
    assert result["passed"] is False
    assert result["settings_profiles_isolated"] is False


def test_error_curve_uses_three_dimensional_distance():
    reference = SimpleNamespace(
        samples=(
            SimpleNamespace(time_from_start_ms=0, position_map_m=(0.0, 0.0, 0.0)),
            SimpleNamespace(time_from_start_ms=1000, position_map_m=(1.0, 0.0, 0.0)),
        )
    )
    compared = SimpleNamespace(
        samples=(
            SimpleNamespace(time_from_start_ms=0, position_map_m=(0.0, 0.0, 0.0)),
            SimpleNamespace(time_from_start_ms=1000, position_map_m=(1.0, 4.0, 3.0)),
        )
    )
    curve = trajectory_error_curve(reference, compared)
    assert curve == [
        {"t_s": 0.0, "horizontal_m": 0.0, "vertical_m": 0.0, "three_d_m": 0.0},
        {"t_s": 1.0, "horizontal_m": 4.0, "vertical_m": 3.0, "three_d_m": 5.0},
    ]


def _pair(reference: TrajectoryRole, compared: TrajectoryRole) -> TrajectoryPairMetrics:
    return TrajectoryPairMetrics(
        reference_role=reference,
        compared_role=compared,
        sample_count=2,
        overlap_start_ms=0,
        overlap_end_ms=200,
        horizontal_rmse_m=0.0,
        vertical_rmse_m=0.0,
        three_dimensional_rmse_m=0.0,
        maximum_3d_error_m=0.0,
        p95_3d_error_m=0.0,
        endpoint_3d_error_m=0.0,
        within_thresholds=True,
    )


def test_packaged_comparison_report_is_read_directly(tmp_path: Path):
    report = TrajectoryComparisonReport(
        mission_id=uuid4(),
        vehicle_id=1,
        frame_calibration_id="parallel-sitl-locked-v1",
        frame_calibration_hash="a" * 64,
        evidence_hashes={role: str(index) * 64 for index, role in enumerate(TrajectoryRole, 1)},
        thresholds=TrajectoryThresholds(validated_for_venue=True),
        comparisons=(
            _pair(TrajectoryRole.PLANNED, TrajectoryRole.PREDICTED),
            _pair(TrajectoryRole.PLANNED, TrajectoryRole.OBSERVED),
            _pair(TrajectoryRole.PREDICTED, TrajectoryRole.OBSERVED),
        ),
        ready=True,
        preview_only=False,
        metrics_within_thresholds=True,
        acceptance_passed=True,
    )
    package = tmp_path / "evidence.zip"
    manifest = {
        "artifacts": [
            {
                "name": "comparison-report",
                "relative_path": "artifacts/comparison-report.json",
            }
        ]
    }
    with zipfile.ZipFile(package, "w") as archive:
        archive.writestr("manifest.json", json.dumps(manifest))
        archive.writestr(
            "artifacts/comparison-report.json",
            json.dumps(report.public_payload(), default=str),
        )
    assert comparison_report_pairs(package) == (
        ("planned", "predicted"),
        ("planned", "observed"),
        ("predicted", "observed"),
    )


def test_same_screen_report_labels_simulation_boundaries(tmp_path: Path):
    output = tmp_path / "same-screen-report.html"
    write_same_screen_report(
        output,
        {
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
            "scope": "SITL 实机替身 + AirSim 孪生；不是外场真机",
            "lines": {},
        },
    )
    html = output.read_text(encoding="utf-8")
    assert "Gazebo / ArduPilot 原生 SITL（仿真/HIL）" in html
    assert "AirSim 是仿真/孪生" in html
    assert "不代表外场真机已接入" in html
    assert "<canvas" in html

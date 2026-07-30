from pathlib import Path

import pytest

from aeromind_apm_lite.common.mission import MissionState
from aeromind_apm_lite.ground.simulation.fault_injection import (
    NavigationFaultType,
)
from aeromind_apm_lite.ground.simulation.navigation_acceptance import (
    NavigationAcceptanceError,
    _attach_trajectory_evidence,
    acceptance_passed,
    build_argument_parser,
    select_navigation_vehicle,
)
from aeromind_apm_lite.onboard import NavigationFailureCode


ROOT = Path(__file__).resolve().parents[1]


def test_navigation_acceptance_selects_only_the_requested_sitl_vehicle():
    vehicle = select_navigation_vehicle(
        ROOT / "configs/sim/fleet.m1.yaml",
        1,
    )

    assert vehicle.vehicle_id == 1
    assert vehicle.fcu.endpoint == "udpin:0.0.0.0:14550"

    with pytest.raises(NavigationAcceptanceError, match="sim mode"):
        select_navigation_vehicle(
            ROOT / "configs/real/fleet.example.yaml",
            1,
        )


def test_acceptance_requires_success_or_the_exact_injected_failure():
    assert acceptance_passed(MissionState.COMPLETED, None, None)
    assert not acceptance_passed(
        MissionState.FAILED,
        NavigationFailureCode.HDOP_EXCEEDED,
        None,
    )
    assert acceptance_passed(
        MissionState.FAILED,
        NavigationFailureCode.HDOP_EXCEEDED,
        NavigationFaultType.HDOP_EXCEEDED,
    )
    assert not acceptance_passed(
        MissionState.FAILED,
        NavigationFailureCode.EKF_UNHEALTHY,
        NavigationFaultType.HDOP_EXCEEDED,
    )


def test_navigation_acceptance_cli_exposes_all_fault_scenarios():
    parser = build_argument_parser()
    for fault in NavigationFaultType:
        args = parser.parse_args(["--fault", fault.value])
        assert args.fault == fault.value


def test_navigation_acceptance_cli_exposes_optional_trajectory_evidence():
    args = build_argument_parser().parse_args(
        [
            "--georeference-config",
            "venue.yaml",
            "--trajectory-evidence-dir",
            "/tmp/evidence",
            "--software-commit",
            "8605422",
            "--arducopter-version",
            "4.5-test",
        ]
    )

    assert args.georeference_config == Path("venue.yaml")
    assert args.trajectory_evidence_dir == Path("/tmp/evidence")
    assert args.software_commit == "8605422"


def test_navigation_result_contract_keeps_evidence_error_separate():
    class FailedRecorder:
        def finalize(self):
            raise ValueError(
                "trajectory recording requires at least two position observations"
            )

    payload = {
        "passed": True,
        "result": {
            "terminal_state": "failed",
            "failure_code": "preflight_timeout",
        },
    }
    result = _attach_trajectory_evidence(
        payload,
        FailedRecorder(),
        Path("/tmp/evidence"),
        type("Plan", (), {"mission_id": "mission-id"})(),
        None,
    )

    assert not result["passed"]
    assert result["result"]["failure_code"] == "preflight_timeout"
    assert "two position observations" in result["trajectory_evidence_error"]

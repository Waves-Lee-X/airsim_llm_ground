import json
from datetime import datetime, timezone

import pytest

from aeromind_apm_lite.common.evidence import (
    EvidenceError,
    EvidencePackageBuilder,
    ExperimentLock,
    verify_evidence_package,
)
from tools.export_evidence import main as evidence_cli_main


NOW = datetime(2026, 8, 10, 8, 0, tzinfo=timezone.utc)


def experiment_lock():
    return ExperimentLock.from_inputs(
        code_commit="8346b5c",
        software_versions={"aeromind-apm-lite": "0.1.0"},
        model_versions={"l0": "l0-event-v1"},
        algorithm_versions={"pareto": "pareto-v1"},
        scenario={"name": "m1-demo", "vehicles": 4},
        configuration={"duration_s": 120, "branches": 4},
        random_seeds=(42,),
    )


def builder():
    package = EvidencePackageBuilder(
        experiment_id="m1-demo-20260810",
        created_at_utc=NOW,
        lock=experiment_lock(),
        requirements=("M1-WP2-PARETO", "M1-WP6-HASH"),
        applicability=("L0 deterministic simulation",),
        limitations=("No L2 SITL ranking claim",),
        conclusion="The constrained L0 branches were evaluated reproducibly.",
    )
    package.add_json("deduction-result", {"selected": ["branch-1"]}, role="result")
    package.add_bytes("run-log", b"branch-1 completed\n", role="log", media_type="text/plain")
    return package


def test_evidence_directory_hash_verifies_and_tampering_is_detected(tmp_path):
    output = tmp_path / "package"
    exported = builder().export(output)

    verified = verify_evidence_package(output)
    assert verified.package_hash == exported.package_hash
    assert len(verified.manifest.artifacts) == 2

    artifact = output / verified.manifest.artifacts[0].relative_path
    artifact.write_text("tampered", encoding="utf-8")
    with pytest.raises(EvidenceError, match="hash mismatch"):
        verify_evidence_package(output)


def test_evidence_zip_is_deterministic_for_identical_inputs(tmp_path):
    first = tmp_path / "first.zip"
    second = tmp_path / "second.zip"

    first_package = builder().export(first)
    second_package = builder().export(second)

    assert first.read_bytes() == second.read_bytes()
    assert first_package.package_hash == second_package.package_hash
    assert verify_evidence_package(first).package_hash == first_package.package_hash


def test_export_tool_builds_and_verifies_package_from_spec(tmp_path):
    artifact = tmp_path / "metrics.json"
    artifact.write_text('{"completion": 1.0}\n', encoding="utf-8")
    spec = tmp_path / "spec.json"
    spec.write_text(
        json.dumps(
            {
                "experiment_id": "tool-test",
                "created_at_utc": NOW.isoformat(),
                "requirements": ["WP6"],
                "applicability": ["test"],
                "limitations": [],
                "conclusion": "verified",
                "lock": experiment_lock().model_dump(mode="json"),
                "artifacts": [
                    {
                        "name": "metrics",
                        "path": "metrics.json",
                        "role": "metrics",
                        "media_type": "application/json",
                    }
                ],
            },
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )
    output = tmp_path / "tool-package.zip"

    assert evidence_cli_main(["export", "--spec", str(spec), "--output", str(output)]) == 0
    assert evidence_cli_main(["verify", str(output)]) == 0
    assert verify_evidence_package(output).manifest.experiment_id == "tool-test"

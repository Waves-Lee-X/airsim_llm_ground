#!/usr/bin/env python3
"""Export deterministic JSON Schemas for protocol review and non-Python clients."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from aeromind_apm_lite.common.config import FleetConfig  # noqa: E402
from aeromind_apm_lite.common.coordinates import GeoReference  # noqa: E402
from aeromind_apm_lite.common.contracts.codec import wire_message_json_schema  # noqa: E402
from aeromind_apm_lite.common.trajectory import (  # noqa: E402
    TrajectoryComparisonReport,
    TrajectoryEvidence,
)
from aeromind_apm_lite.common.contracts.twin import (  # noqa: E402
    MetricRecord,
    MissionGraph,
    StrategySpec,
    TwinState,
    VehicleProfile,
    WorldEvent,
)
from aeromind_apm_lite.common.evidence import (  # noqa: E402
    EvidenceManifest,
    EvidencePackage,
    ExperimentLock,
)
from aeromind_apm_lite.ground.deduction import (  # noqa: E402
    BaselineSnapshot,
    BranchResult,
    BranchSpec,
    DeductionRun,
    ParetoReport,
)
from aeromind_apm_lite.ground.browser.vision_evidence import (  # noqa: E402
    ColorDetection,
    VisionAnalysisEvidence,
)


def rendered_schema() -> str:
    bundle = {
        "$schema": "https://json-schema.org/draft/2020-12/schema",
        "schema_version": "1.0",
        "schemas": {
            "fleet_config": FleetConfig.model_json_schema(),
            "georeference": GeoReference.model_json_schema(),
            "trajectory_comparison_report": (
                TrajectoryComparisonReport.model_json_schema()
            ),
            "trajectory_evidence": TrajectoryEvidence.model_json_schema(),
            "vision_analysis_evidence": VisionAnalysisEvidence.model_json_schema(),
            "color_detection": ColorDetection.model_json_schema(),
            "wire_message": wire_message_json_schema(),
        },
    }
    return json.dumps(bundle, ensure_ascii=False, indent=2, sort_keys=True) + "\n"


def rendered_m1_schemas() -> dict[str, str]:
    bundles = {
        "twin-v1.schema.json": {
            "twin_state": TwinState,
            "vehicle_profile": VehicleProfile,
            "mission_graph": MissionGraph,
            "world_event": WorldEvent,
            "strategy_spec": StrategySpec,
            "metric_record": MetricRecord,
        },
        "deduction-v1.schema.json": {
            "baseline_snapshot": BaselineSnapshot,
            "branch_spec": BranchSpec,
            "branch_result": BranchResult,
            "deduction_run": DeductionRun,
            "pareto_report": ParetoReport,
        },
        "evidence-v1.schema.json": {
            "experiment_lock": ExperimentLock,
            "evidence_manifest": EvidenceManifest,
            "evidence_package": EvidencePackage,
        },
    }
    return {
        filename: json.dumps(
            {
                "$schema": "https://json-schema.org/draft/2020-12/schema",
                "schema_version": "1.0",
                "schemas": {
                    name: model.model_json_schema()
                    for name, model in schemas.items()
                },
            },
            ensure_ascii=False,
            indent=2,
            sort_keys=True,
        )
        + "\n"
        for filename, schemas in bundles.items()
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true", help="fail if the committed schema is stale")
    args = parser.parse_args()
    destination = ROOT / "schemas" / "protocol-v1.schema.json"
    expected = rendered_schema()
    m1_schemas = rendered_m1_schemas()

    if args.check:
        expected_values = {destination: expected, **{
            ROOT / "schemas" / name: value for name, value in m1_schemas.items()
        }}
        for path, expected_value in expected_values.items():
            if not path.exists() or path.read_text(encoding="utf-8") != expected_value:
                print(f"stale schema: {path.name}; run {Path(__file__).name}", file=sys.stderr)
                return 1
        return 0

    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(expected, encoding="utf-8")
    for filename, value in m1_schemas.items():
        (destination.parent / filename).write_text(value, encoding="utf-8")
    print(destination)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

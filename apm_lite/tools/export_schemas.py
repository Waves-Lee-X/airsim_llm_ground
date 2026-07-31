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


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true", help="fail if the committed schema is stale")
    args = parser.parse_args()
    destination = ROOT / "schemas" / "protocol-v1.schema.json"
    expected = rendered_schema()

    if args.check:
        if not destination.exists() or destination.read_text(encoding="utf-8") != expected:
            print(f"stale schema: run {Path(__file__).name}", file=sys.stderr)
            return 1
        return 0

    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(expected, encoding="utf-8")
    print(destination)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

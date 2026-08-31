#!/usr/bin/env python3
"""Offline acceptance for the shared twin/coordinate/time representation."""

from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from aeromind_apm_lite.common.contracts import TwinState  # noqa: E402
from aeromind_apm_lite.common.coordinates import (  # noqa: E402
    GeoReference,
    Vec3,
    load_georeference,
)
from aeromind_apm_lite.common.timing import MonotonicClock  # noqa: E402


def distance(left: Vec3, right: Vec3) -> float:
    return sum((a - b) ** 2 for a, b in zip(
        (left.x, left.y, left.z), (right.x, right.y, right.z)
    )) ** 0.5


def accept(reference: GeoReference) -> dict[str, object]:
    known_map = Vec3(12.5, -8.0, 6.25)
    gps = reference.map_to_wgs84(known_map)
    px4_map = reference.telemetry_wgs84_to_map(gps, line="px4")
    apm_map = reference.telemetry_wgs84_to_map(gps, line="apm")
    map_round_trip_error_m = distance(known_map, px4_map)
    cross_line_error_m = distance(px4_map, apm_map)
    clock = MonotonicClock()
    stamps = [clock.stamp(sim_time_s=float(index)) for index in range(3)]
    schema = TwinState.model_json_schema()
    required_contract_fields = {
        "twin_id", "vehicle_id", "line", "source", "frame", "position",
        "attitude", "mode", "task_phase", "sim_time", "wall_time",
        "confidence", "fidelity",
    }
    schema_fields = set(schema["properties"])
    numerical_passed = (
        map_round_trip_error_m < 1e-6
        and cross_line_error_m < 1e-6
        and required_contract_fields <= schema_fields
        and all(
            current.monotonic_s >= previous.monotonic_s
            for previous, current in zip(stamps, stamps[1:])
        )
    )
    preview_only = reference.preview_only
    return {
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "calibration_id": reference.calibration_id,
        "calibration_sha256": reference.calibration_sha256,
        "calibration_status": reference.status.value,
        "map_round_trip_error_m": map_round_trip_error_m,
        "cross_line_error_m": cross_line_error_m,
        "tolerance_m": 1e-6,
        "schema_fields_present": sorted(required_contract_fields),
        "monotonic_samples_s": [item.monotonic_s for item in stamps],
        "numerical_passed": numerical_passed,
        "preview_only": preview_only,
        "acceptance_passed": numerical_passed if not preview_only else None,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--georeference",
        type=Path,
        default=ROOT / "configs/parallel_domain/demo-georeference.yaml",
    )
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = accept(load_georeference(args.georeference))
    rendered = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
    print(rendered, end="")
    return 0 if report["numerical_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

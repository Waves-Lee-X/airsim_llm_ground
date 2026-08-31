#!/usr/bin/env python3
"""Measure, freeze and verify a parallel-domain GeoReference."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Sequence

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from aeromind_apm_lite.common.coordinates import (  # noqa: E402
    GeoReferenceStore,
    load_georeference,
)
from aeromind_apm_lite.ground.parallel_domain.calibration import (  # noqa: E402
    CalibrationDirection,
    CalibrationError,
    CalibrationThresholds,
    CalibrationTruthSource,
    LiveCalibrationDataset,
    calibration_measurement_from_gazebo,
    freeze_georeference,
    load_calibration_dataset,
    load_calibration_report,
    load_freeze_receipt,
    measure_calibration,
    parse_gazebo_pose_tuple,
    read_latest_observed_replay,
    verify_calibration_freeze,
    write_calibration_dataset,
    write_calibration_report,
    write_freeze_receipt,
)


def _add_thresholds(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--minimum-samples", type=int, default=6)
    parser.add_argument("--minimum-control-points", type=int, default=3)
    parser.add_argument("--horizontal-rmse-m", type=float, default=1.0)
    parser.add_argument("--vertical-rmse-m", type=float, default=1.0)
    parser.add_argument("--maximum-3d-error-m", type=float, default=2.0)
    parser.add_argument("--p95-3d-error-m", type=float, default=1.5)


def _thresholds(args: argparse.Namespace) -> CalibrationThresholds:
    return CalibrationThresholds(
        minimum_samples=args.minimum_samples,
        minimum_control_points=args.minimum_control_points,
        horizontal_rmse_m=args.horizontal_rmse_m,
        vertical_rmse_m=args.vertical_rmse_m,
        maximum_3d_error_m=args.maximum_3d_error_m,
        p95_3d_error_m=args.p95_3d_error_m,
    )


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    measure = subparsers.add_parser(
        "measure",
        help="compare measured GPS with independent map ground truth",
    )
    measure.add_argument("--georeference", type=Path, required=True)
    measure.add_argument("--measurements", type=Path, required=True)
    measure.add_argument("--output", type=Path, required=True)
    _add_thresholds(measure)

    capture = subparsers.add_parser(
        "capture-gazebo",
        help="pair live observed GPS with independent Gazebo model ground truth",
    )
    capture.add_argument("--georeference", type=Path, required=True)
    capture.add_argument("--replay", type=Path, required=True)
    capture.add_argument("--line", choices=("px4", "apm"), required=True)
    capture.add_argument("--vehicle-id", type=int, required=True)
    capture.add_argument("--model", required=True)
    capture.add_argument("--world", default="default")
    capture.add_argument("--control-point", required=True)
    capture.add_argument("--pass-id", required=True)
    capture.add_argument(
        "--direction",
        choices=tuple(item.value for item in CalibrationDirection),
        required=True,
    )
    capture.add_argument("--sample-count", type=int, default=10)
    capture.add_argument("--sample-interval-s", type=float, default=0.2)
    capture.add_argument("--sample-timeout-s", type=float, default=30.0)
    capture.add_argument("--maximum-pairing-skew-s", type=float, default=0.5)
    capture.add_argument("--output", type=Path, required=True)
    capture.add_argument("--append", action="store_true")
    capture.add_argument("--report", type=Path)
    _add_thresholds(capture)

    freeze = subparsers.add_parser(
        "freeze",
        help="promote a draft only after the measured calibration gate passes",
    )
    freeze.add_argument("--draft", type=Path, required=True)
    freeze.add_argument("--measurements", type=Path, required=True)
    freeze.add_argument("--output-georeference", type=Path, required=True)
    freeze.add_argument("--output-report", type=Path, required=True)
    freeze.add_argument("--output-receipt", type=Path, required=True)
    _add_thresholds(freeze)

    verify = subparsers.add_parser(
        "verify",
        help="verify the frozen SHA-256, measurement report and receipt",
    )
    verify.add_argument("--georeference", type=Path, required=True)
    verify.add_argument("--measurements", type=Path, required=True)
    verify.add_argument("--report", type=Path, required=True)
    verify.add_argument("--receipt", type=Path, required=True)

    args = parser.parse_args(argv)
    if args.command == "capture-gazebo":
        minimum_capture_samples = 1 if args.append else 2
        if not minimum_capture_samples <= args.sample_count <= 10_000:
            lower = "1 when --append is used, otherwise 2"
            raise SystemExit(f"--sample-count must be at least {lower} and at most 10000")
        if args.sample_interval_s < 0.05:
            raise SystemExit("--sample-interval-s must be at least 0.05")
        if args.sample_timeout_s <= 0.0:
            raise SystemExit("--sample-timeout-s must be positive")
        if args.maximum_pairing_skew_s <= 0.0:
            raise SystemExit("--maximum-pairing-skew-s must be positive")
        if args.output.exists() and not args.append:
            raise SystemExit(f"capture output already exists: {args.output}")
        if args.append and not args.output.is_file():
            raise SystemExit(f"--append requires an existing dataset: {args.output}")
        reference = load_georeference(args.georeference)
        existing = (
            load_calibration_dataset(args.output)
            if args.append
            else None
        )
        truth_reference = (
            f"Gazebo Classic world={args.world} model={args.model} root pose from "
            "`gz model -p`, wall-time paired with GPS-derived observed replay"
        )
        if existing is not None:
            if (
                existing.calibration_id != reference.calibration_id
                or existing.line != args.line
                or existing.vehicle_id != args.vehicle_id
                or existing.truth_source
                != CalibrationTruthSource.SIMULATION_GROUND_TRUTH
            ):
                raise SystemExit("append dataset identity does not match capture arguments")
            truth_reference = existing.truth_reference
        measurements = []
        last_sequence = -1
        for _ in range(args.sample_count):
            deadline = time.monotonic() + args.sample_timeout_s
            last_pairing_error: CalibrationError | None = None
            while True:
                before = datetime.now(timezone.utc)
                try:
                    completed = subprocess.run(
                        ["gz", "model", "-w", args.world, "-m", args.model, "-p"],
                        check=True,
                        capture_output=True,
                        text=True,
                        timeout=5.0,
                    )
                except (OSError, subprocess.CalledProcessError, subprocess.TimeoutExpired) as exc:
                    raise SystemExit(f"failed to read Gazebo model pose: {exc}") from exc
                after = datetime.now(timezone.utc)
                replay = read_latest_observed_replay(args.replay)
                if replay.line.value != args.line or replay.vehicle_id != args.vehicle_id:
                    raise SystemExit("observed replay line/vehicle does not match capture")
                if replay.sequence > last_sequence:
                    try:
                        measurement = calibration_measurement_from_gazebo(
                            reference=reference,
                            replay=replay,
                            gazebo_position_enu_m=parse_gazebo_pose_tuple(completed.stdout),
                            ground_truth_captured_at_utc=before + (after - before) / 2,
                            ground_truth_reference=truth_reference,
                            control_point_id=args.control_point,
                            pass_id=args.pass_id,
                            direction=CalibrationDirection(args.direction),
                            maximum_pairing_skew_s=args.maximum_pairing_skew_s,
                        )
                    except CalibrationError as exc:
                        last_pairing_error = exc
                        if time.monotonic() >= deadline:
                            raise SystemExit(str(last_pairing_error))
                    else:
                        measurements.append(measurement)
                        last_sequence = replay.sequence
                        break
                if time.monotonic() >= deadline:
                    detail = (
                        f": {last_pairing_error}"
                        if last_pairing_error is not None
                        else ""
                    )
                    raise SystemExit(
                        "timed out waiting for a fresh observed replay sample" + detail
                    )
                time.sleep(args.sample_interval_s)
            time.sleep(args.sample_interval_s)
        combined = tuple(existing.measurements if existing is not None else ()) + tuple(
            measurements
        )
        dataset_values = {
            "calibration_id": reference.calibration_id,
            "line": args.line,
            "vehicle_id": args.vehicle_id,
            "truth_source": CalibrationTruthSource.SIMULATION_GROUND_TRUTH,
            "truth_reference": truth_reference,
            "created_at_utc": (
                existing.created_at_utc if existing is not None else datetime.now(timezone.utc)
            ),
            "measurements": combined,
        }
        if existing is not None:
            dataset_values["dataset_id"] = existing.dataset_id
        dataset = LiveCalibrationDataset(**dataset_values)
        write_calibration_dataset(args.output, dataset)
        report = measure_calibration(reference, dataset, _thresholds(args))
        if args.report is not None:
            write_calibration_report(args.report, report)
        print(
            json.dumps(
                {
                    "dataset": str(args.output),
                    "dataset_hash": dataset.dataset_hash,
                    "report": str(args.report) if args.report is not None else None,
                    "report_hash": report.report_hash,
                    "sample_count": len(dataset.measurements),
                    "preview_only": report.preview_only,
                    "acceptance_passed": report.acceptance_passed,
                    "distribution": report.distribution.model_dump(mode="json"),
                    "statement": report.result_statement_zh,
                },
                ensure_ascii=False,
                indent=2,
                sort_keys=True,
            )
        )
        return 0

    if args.command == "measure":
        report = measure_calibration(
            load_georeference(args.georeference),
            load_calibration_dataset(args.measurements),
            _thresholds(args),
        )
        write_calibration_report(args.output, report)
        print(
            json.dumps(
                {
                    "output": str(args.output),
                    "report_hash": report.report_hash,
                    "preview_only": report.preview_only,
                    "acceptance_passed": report.acceptance_passed,
                    "distribution": report.distribution.model_dump(mode="json"),
                    "statement": report.result_statement_zh,
                },
                ensure_ascii=False,
                indent=2,
                sort_keys=True,
            )
        )
        return 0 if report.metrics_within_thresholds else 1

    if args.command == "freeze":
        outputs = (
            args.output_georeference,
            args.output_report,
            args.output_receipt,
        )
        existing = [str(path) for path in outputs if path.exists()]
        if existing:
            raise SystemExit("freeze outputs already exist: " + ", ".join(existing))
        dataset = load_calibration_dataset(args.measurements)
        frozen, report, receipt = freeze_georeference(
            load_georeference(args.draft),
            dataset,
            _thresholds(args),
        )
        GeoReferenceStore(args.output_georeference).save(frozen)
        write_calibration_report(args.output_report, report)
        write_freeze_receipt(args.output_receipt, receipt)
        print(
            json.dumps(
                {
                    "georeference": str(args.output_georeference),
                    "calibration_id": frozen.calibration_id,
                    "calibration_sha256": frozen.config_hash,
                    "report": str(args.output_report),
                    "report_hash": report.report_hash,
                    "receipt": str(args.output_receipt),
                    "receipt_hash": receipt.receipt_hash,
                    "preview_only": frozen.preview_only,
                    "acceptance_passed": report.acceptance_passed,
                },
                ensure_ascii=False,
                indent=2,
                sort_keys=True,
            )
        )
        return 0

    reference = load_georeference(args.georeference)
    dataset = load_calibration_dataset(args.measurements)
    report = load_calibration_report(args.report)
    receipt = load_freeze_receipt(args.receipt)
    verify_calibration_freeze(reference, dataset, report, receipt)
    print(
        json.dumps(
            {
                "verified": True,
                "calibration_id": reference.calibration_id,
                "calibration_sha256": reference.config_hash,
                "receipt_hash": receipt.receipt_hash,
                "preview_only": reference.preview_only,
                "acceptance_passed": report.acceptance_passed,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

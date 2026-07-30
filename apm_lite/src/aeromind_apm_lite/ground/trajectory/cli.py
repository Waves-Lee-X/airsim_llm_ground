"""CLI for GPS replay and planned/predicted/observed trajectory reports."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from uuid import UUID

from aeromind_apm_lite.common.coordinates import load_georeference
from aeromind_apm_lite.common.trajectory import (
    ArtifactVersions,
    TrajectoryEvidenceError,
    TrajectoryThresholds,
    compare_trajectories,
    load_evidence_file,
    replay_gps_csv,
    write_evidence_file,
    write_report_file,
)


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Build replayable trajectory evidence and sim-to-real reports"
    )
    commands = parser.add_subparsers(dest="command", required=True)

    replay = commands.add_parser("replay-gps", help="convert a GPS CSV into observed evidence")
    replay.add_argument("--input", type=Path, required=True)
    replay.add_argument("--georeference", type=Path, required=True)
    replay.add_argument("--mission-id", type=UUID, required=True)
    replay.add_argument("--vehicle-id", type=int, required=True)
    replay.add_argument("--producer-version", required=True)
    replay.add_argument("--output", type=Path, required=True)
    replay.add_argument("--allow-draft", action="store_true")
    _add_version_arguments(replay)

    report = commands.add_parser("report", help="compare two or three evidence files")
    report.add_argument("--evidence", type=Path, action="append", required=True)
    report.add_argument("--output", type=Path, required=True)
    report.add_argument("--horizontal-rmse-m", type=float, default=3.0)
    report.add_argument("--vertical-rmse-m", type=float, default=2.0)
    report.add_argument("--maximum-3d-error-m", type=float, default=8.0)
    report.add_argument("--endpoint-3d-error-m", type=float, default=5.0)
    report.add_argument("--sample-period-ms", type=int, default=200)
    report.add_argument("--thresholds-validated", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_argument_parser().parse_args(argv)
    try:
        if args.command == "replay-gps":
            evidence = replay_gps_csv(
                args.input,
                load_georeference(args.georeference),
                mission_id=args.mission_id,
                vehicle_id=args.vehicle_id,
                producer_version=args.producer_version,
                versions=_versions_from_args(args),
                allow_draft=args.allow_draft,
            )
            write_evidence_file(args.output, evidence)
            payload = {
                "written": True,
                "output": str(args.output),
                "evidence_id": str(evidence.evidence_id),
                "evidence_hash": evidence.evidence_hash,
                "role": evidence.role.value,
                "sample_count": len(evidence.samples),
                "preview_only": evidence.preview_only,
            }
        else:
            if not 2 <= len(args.evidence) <= 3:
                raise TrajectoryEvidenceError("report requires two or three --evidence files")
            thresholds = TrajectoryThresholds(
                horizontal_rmse_m=args.horizontal_rmse_m,
                vertical_rmse_m=args.vertical_rmse_m,
                maximum_3d_error_m=args.maximum_3d_error_m,
                endpoint_3d_error_m=args.endpoint_3d_error_m,
                sample_period_ms=args.sample_period_ms,
                validated_for_venue=args.thresholds_validated,
            )
            report = compare_trajectories(
                [load_evidence_file(path) for path in args.evidence],
                thresholds,
            )
            write_report_file(args.output, report)
            payload = report.public_payload()
    except Exception as exc:
        print(
            json.dumps(
                {"passed": False, "error": f"{type(exc).__name__}: {exc}"},
                ensure_ascii=False,
                indent=2,
            )
        )
        return 2
    print(json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True))
    return 0


def _add_version_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--mission-plan-hash")
    parser.add_argument("--software-commit")
    parser.add_argument("--arducopter-version")
    parser.add_argument("--parameter-hash")
    parser.add_argument("--scene-hash")


def _versions_from_args(args: argparse.Namespace) -> ArtifactVersions:
    return ArtifactVersions(
        mission_plan_hash=args.mission_plan_hash,
        software_commit=args.software_commit,
        arducopter_version=args.arducopter_version,
        parameter_hash=args.parameter_hash,
        scene_hash=args.scene_hash,
    )


if __name__ == "__main__":
    raise SystemExit(main())

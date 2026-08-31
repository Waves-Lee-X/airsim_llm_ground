#!/usr/bin/env python3
"""Build or verify a live parallel-domain trajectory evidence package."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path
from typing import Sequence
from uuid import UUID

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from aeromind_apm_lite.common.coordinates import load_georeference  # noqa: E402
from aeromind_apm_lite.ground.parallel_domain.calibration import (  # noqa: E402
    load_calibration_dataset,
    load_calibration_report,
    load_freeze_receipt,
)
from aeromind_apm_lite.ground.parallel_domain.live_evidence import (  # noqa: E402
    build_live_evidence_package,
    software_tree_sha256,
    verify_live_evidence_package,
)


def _version_values(values: list[str]) -> dict[str, str]:
    result: dict[str, str] = {}
    for value in values:
        name, separator, version = value.partition("=")
        if not separator or not name.strip() or not version.strip():
            raise SystemExit(f"invalid --version {value!r}; expected NAME=VALUE")
        result[name.strip()] = version.strip()
    return result


def _git_commit() -> str | None:
    try:
        return subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=ROOT,
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    build = subparsers.add_parser("build")
    build.add_argument("--state-directory", type=Path, required=True)
    build.add_argument("--mission-id", type=UUID, required=True)
    build.add_argument("--georeference", type=Path, required=True)
    build.add_argument("--calibration-measurements", type=Path)
    build.add_argument("--calibration-report", type=Path)
    build.add_argument("--calibration-receipt", type=Path)
    build.add_argument("--output", type=Path, required=True)
    build.add_argument("--random-seed", type=int, required=True)
    build.add_argument(
        "--version",
        action="append",
        default=[],
        metavar="NAME=VALUE",
    )
    build.add_argument("--software-root", type=Path, default=ROOT)
    build.add_argument("--software-commit")
    build.add_argument("--allow-preview", action="store_true")
    verify = subparsers.add_parser("verify")
    verify.add_argument("package", type=Path)
    args = parser.parse_args(argv)

    if args.command == "verify":
        manifest = verify_live_evidence_package(args.package)
        print(
            json.dumps(
                {
                    "verified": True,
                    "package": str(args.package),
                    "content_sha256": manifest.content_sha256,
                    "mission_id": str(manifest.mission_id),
                    "line": manifest.line,
                    "preview_only": manifest.preview_only,
                    "acceptance_passed": manifest.acceptance_passed,
                },
                indent=2,
                sort_keys=True,
            )
        )
        return 0

    calibration_paths = (
        args.calibration_measurements,
        args.calibration_report,
        args.calibration_receipt,
    )
    if any(path is not None for path in calibration_paths) and any(
        path is None for path in calibration_paths
    ):
        raise SystemExit(
            "all three calibration measurement/report/receipt paths are required"
        )
    versions = _version_values(args.version)
    software_root = args.software_root.resolve()
    manifest = build_live_evidence_package(
        state_directory=args.state_directory,
        mission_id=args.mission_id,
        output=args.output,
        georeference=load_georeference(args.georeference),
        versions=versions,
        software_sha256=software_tree_sha256(software_root),
        software_hash_scope=str(software_root),
        random_seed=args.random_seed,
        software_commit=args.software_commit or _git_commit(),
        calibration_dataset=(
            load_calibration_dataset(args.calibration_measurements)
            if args.calibration_measurements is not None
            else None
        ),
        calibration_report=(
            load_calibration_report(args.calibration_report)
            if args.calibration_report is not None
            else None
        ),
        calibration_receipt=(
            load_freeze_receipt(args.calibration_receipt)
            if args.calibration_receipt is not None
            else None
        ),
        allow_preview=args.allow_preview,
    )
    print(
        json.dumps(
            {
                "output": str(args.output),
                "content_sha256": manifest.content_sha256,
                "mission_id": str(manifest.mission_id),
                "line": manifest.line,
                "time_range_utc": manifest.time_range_utc.model_dump(mode="json"),
                "preview_only": manifest.preview_only,
                "acceptance_passed": manifest.acceptance_passed,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

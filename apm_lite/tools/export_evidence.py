#!/usr/bin/env python3
"""Export or verify a self-contained AeroMind evidence package."""

from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime
from pathlib import Path
from typing import Sequence

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from aeromind_apm_lite.common.evidence import (  # noqa: E402
    EvidencePackageBuilder,
    ExperimentLock,
    verify_evidence_package,
)


def _export(spec_path: Path, output: Path) -> int:
    payload = json.loads(spec_path.read_text(encoding="utf-8"))
    created_at = datetime.fromisoformat(
        str(payload["created_at_utc"]).replace("Z", "+00:00")
    )
    builder = EvidencePackageBuilder(
        experiment_id=payload["experiment_id"],
        created_at_utc=created_at,
        lock=ExperimentLock.model_validate(payload["lock"]),
        requirements=payload["requirements"],
        applicability=payload["applicability"],
        limitations=payload.get("limitations", []),
        conclusion=payload["conclusion"],
    )
    for artifact in payload["artifacts"]:
        source = Path(artifact["path"])
        if not source.is_absolute():
            source = spec_path.parent / source
        builder.add_file(
            artifact["name"],
            source,
            role=artifact["role"],
            media_type=artifact.get("media_type", "application/octet-stream"),
        )
    package = builder.export(output)
    print(json.dumps({"output": str(output), "package_hash": package.package_hash}))
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    export_parser = subparsers.add_parser("export")
    export_parser.add_argument("--spec", type=Path, required=True)
    export_parser.add_argument("--output", type=Path, required=True)
    verify_parser = subparsers.add_parser("verify")
    verify_parser.add_argument("package", type=Path)
    args = parser.parse_args(argv)

    if args.command == "export":
        return _export(args.spec, args.output)
    package = verify_evidence_package(args.package)
    print(json.dumps({"package_hash": package.package_hash, "verified": True}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""Command line entry points for the reproducible parallel-domain bridge."""

from __future__ import annotations

import argparse
import asyncio
import json
import sys
from datetime import timedelta
from pathlib import Path
from uuid import UUID

from .config import load_bridge_config
from .models import (
    ApprovalCommand,
    LiveFaultCommand,
    LiveFaultType,
    MissionPlan,
    RealDispatchRequest,
    utc_now,
)
from .service import (
    ParallelDomainService,
    write_approval_to_spool,
    write_dispatch_to_spool,
    write_fault_to_spool,
    write_plan_to_spool,
)
from .storage import BridgeStore
from .core import confirmation_digest_for
from .observer import serve_observer


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="PX4/APM parallel-domain bridge")
    subparsers = parser.add_subparsers(dest="command", required=True)
    serve = subparsers.add_parser("serve", help="run the resident telemetry bridge")
    serve.add_argument("--config", required=True, type=Path)
    observe = subparsers.add_parser(
        "observe", help="serve the read-only live observed trajectory monitor"
    )
    observe.add_argument("--config", required=True, type=Path)
    observe.add_argument("--host", default="127.0.0.1")
    observe.add_argument("--port", type=int, default=8091)
    observe.add_argument("--max-points", type=int, default=1_000)
    submit = subparsers.add_parser("submit", help="place a virtual rehearsal in the spool")
    submit.add_argument("--config", required=True, type=Path)
    submit.add_argument("--mission", required=True, type=Path)
    approve = subparsers.add_parser("approve", help="place an explicit operator approval in the spool")
    approve.add_argument("--config", required=True, type=Path)
    approve.add_argument("--mission-id", required=True, type=UUID)
    approve.add_argument("--mission-hash", required=True)
    approve.add_argument("--operator", required=True)
    approve.add_argument("--digest", required=True)
    status = subparsers.add_parser("status", help="print persisted bridge status")
    status.add_argument("--config", required=True, type=Path)
    dispatch = subparsers.add_parser(
        "request-dispatch",
        help="request real dispatch; the core still requires operator confirmation",
    )
    dispatch.add_argument("--config", required=True, type=Path)
    dispatch.add_argument("--mission-id", required=True, type=UUID)
    dispatch.add_argument("--line", required=True)
    dispatch.add_argument("--vehicle-id", required=True, type=int)
    fault = subparsers.add_parser(
        "inject-fault",
        help="arm one short-lived SITL-only live telemetry fault",
    )
    fault.add_argument("--config", required=True, type=Path)
    fault.add_argument("--mission-id", required=True, type=UUID)
    fault.add_argument("--line", required=True)
    fault.add_argument("--vehicle-id", required=True, type=int)
    fault.add_argument(
        "--fault",
        choices=tuple(item.value for item in LiveFaultType),
        default=LiveFaultType.HEARTBEAT_GPS_LOSS.value,
    )
    fault.add_argument("--confirm-sitl", action="store_true")
    args = parser.parse_args(argv)
    if args.command == "serve":
        return _serve(args.config)
    config = load_bridge_config(args.config)
    if args.command == "observe":
        return serve_observer(
            config.state_directory,
            host=args.host,
            port=args.port,
            max_points=args.max_points,
        )
    if args.command == "submit":
        plan = MissionPlan.model_validate(json.loads(args.mission.read_text(encoding="utf-8")))
        if plan.line_id not in {line.line_id for line in config.lines}:
            raise SystemExit(f"mission line_id {plan.line_id} is not in the bridge config")
        path = write_plan_to_spool(config, plan)
        print(json.dumps({"mission_id": str(plan.mission_id), "mission_hash": plan.mission_hash, "spool": str(path)}, indent=2))
        return 0
    if args.command == "approve":
        request = BridgeStore(config.state_directory).load_approval_request(args.mission_id)
        expected = confirmation_digest_for(request)
        if args.mission_hash != request.mission_hash:
            raise SystemExit("mission hash does not match pending approval request")
        if args.digest != expected:
            raise SystemExit(f"confirmation digest mismatch; expected {expected}")
        command = ApprovalCommand(
            mission_id=args.mission_id,
            mission_hash=args.mission_hash,
            operator_id=args.operator,
            confirmation_digest=args.digest,
        )
        path = write_approval_to_spool(config, command)
        print(json.dumps({"mission_id": str(args.mission_id), "spool": str(path), "operator": args.operator}, indent=2))
        return 0
    if args.command == "status":
        service = ParallelDomainService(config)
        print(json.dumps(service.core.status(), ensure_ascii=True, indent=2))
        return 0
    if args.command == "request-dispatch":
        command = RealDispatchRequest(
            mission_id=args.mission_id,
            line_id=args.line,
            vehicle_id=args.vehicle_id,
        )
        path = write_dispatch_to_spool(config, command)
        print(
            json.dumps(
                {
                    "request_id": str(command.request_id),
                    "mission_id": str(command.mission_id),
                    "spool": str(path),
                },
                indent=2,
            )
        )
        return 0
    if args.command == "inject-fault":
        if not args.confirm_sitl:
            raise SystemExit("fault injection requires --confirm-sitl")
        if not config.live_fault_injection_enabled:
            raise SystemExit("bridge config does not enable live fault injection")
        line = next((item for item in config.lines if item.line_id == args.line), None)
        if line is None or line.vehicle_id != args.vehicle_id:
            raise SystemExit("fault target does not match bridge config")
        if line.real.simulator not in {"gazebo", "native-sitl"}:
            raise SystemExit("fault injection target is not an explicit SITL endpoint")
        requested_at = utc_now()
        command = LiveFaultCommand(
            mission_id=args.mission_id,
            line_id=args.line,
            vehicle_id=args.vehicle_id,
            fault=LiveFaultType(args.fault),
            requested_at_utc=requested_at,
            expires_at_utc=requested_at + timedelta(seconds=30),
        )
        path = write_fault_to_spool(config, command)
        print(
            json.dumps(
                {
                    "fault_id": str(command.fault_id),
                    "mission_id": str(command.mission_id),
                    "spool": str(path),
                },
                indent=2,
            )
        )
        return 0
    return 2


def _serve(config_path: Path) -> int:
    config = load_bridge_config(config_path)
    service = ParallelDomainService(config)
    try:
        asyncio.run(service.run())
    except KeyboardInterrupt:
        return 130
    return 0


if __name__ == "__main__":
    sys.exit(main())


__all__ = ["main"]

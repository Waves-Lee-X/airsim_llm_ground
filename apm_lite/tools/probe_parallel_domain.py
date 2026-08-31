#!/usr/bin/env python3
"""Read-only preflight probe for selected parallel-domain MAVLink outputs."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from pymavlink import mavutil

from aeromind_apm_lite.ground.parallel_domain.config import load_bridge_config


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--timeout-s", type=float, default=15.0)
    parser.add_argument(
        "--role",
        choices=("all", "real", "virtual"),
        default="all",
        help="probe all endpoints or only one side before it is bound by the bridge",
    )
    parser.add_argument(
        "--require-observed-telemetry",
        action="store_true",
        help=(
            "for selected real endpoints, require the GPS, global position, and "
            "attitude messages consumed by the live observed pipeline"
        ),
    )
    args = parser.parse_args()
    if not 1.0 <= args.timeout_s <= 120.0:
        raise SystemExit("--timeout-s must be in [1, 120]")
    config = load_bridge_config(args.config)
    connections = []
    pending = {}
    try:
        for line in config.lines:
            for role, endpoint in (("real", line.real), ("virtual", line.virtual)):
                if args.role != "all" and role != args.role:
                    continue
                key = f"{line.line_id}/{role}"
                connection = mavutil.mavlink_connection(
                    endpoint.endpoint,
                    source_system=252,
                    source_component=190,
                    dialect="common" if line.adapter.value == "px4" else "ardupilotmega",
                    autoreconnect=False,
                )
                connections.append(connection)
                required_messages = {"HEARTBEAT"}
                if args.require_observed_telemetry and role == "real":
                    required_messages.update(
                        {
                            "GPS_RAW_INT",
                            "GLOBAL_POSITION_INT",
                            (
                                "ATTITUDE_QUATERNION"
                                if line.adapter.value == "px4"
                                else "ATTITUDE"
                            ),
                        }
                    )
                pending[key] = {
                    "endpoint": endpoint.endpoint,
                    "simulator": endpoint.simulator,
                    "expected_system_id": endpoint.target_system,
                    "connection": connection,
                    "required_messages": required_messages,
                    "seen_messages": set(),
                }
        deadline = time.monotonic() + args.timeout_s
        while pending and time.monotonic() < deadline:
            for key, item in list(pending.items()):
                message = item["connection"].recv_match(blocking=False)
                if message is None:
                    continue
                system_id = int(message.get_srcSystem())
                if system_id != item["expected_system_id"]:
                    item["unexpected_system_id"] = system_id
                    continue
                message_type = message.get_type()
                item["seen_messages"].add(message_type)
                if message_type == "HEARTBEAT":
                    item["autopilot"] = int(message.autopilot)
                    item["vehicle_type"] = int(message.type)
            if all(
                item["required_messages"] <= item["seen_messages"]
                for item in pending.values()
            ):
                break
            time.sleep(0.02)
        results = {}
        for key, item in pending.items():
            required = item["required_messages"]
            seen = item["seen_messages"]
            results[key] = {
                name: value
                for name, value in item.items()
                if name not in {"connection", "required_messages", "seen_messages"}
            }
            results[key]["required_messages"] = sorted(required)
            results[key]["seen_required_messages"] = sorted(required & seen)
            results[key]["missing_messages"] = sorted(required - seen)
        expected_count = len(config.lines) * (2 if args.role == "all" else 1)
        passed = len(results) == expected_count and all(
            not item["missing_messages"] for item in results.values()
        )
        output = {
            "gate": "PARALLEL_DOMAIN_MAVLINK_PREFLIGHT",
            "passed": passed,
            "role": args.role,
            "require_observed_telemetry": args.require_observed_telemetry,
            "timeout_s": args.timeout_s,
            "endpoints": results,
            "note": "read-only probe; run before the bridge binds the same UDP ports",
        }
        print(json.dumps(output, ensure_ascii=True, indent=2))
        return 0 if passed else 1
    finally:
        for connection in connections:
            connection.close()


if __name__ == "__main__":
    raise SystemExit(main())

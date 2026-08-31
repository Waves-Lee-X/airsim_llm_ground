#!/usr/bin/env python3
"""Independent heartbeat and opt-in ARM/TAKEOFF acceptance for the four SITLs."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from pymavlink import mavutil

from aeromind_apm_lite.ground.parallel_domain.config import load_bridge_config


def _selected(config, line_name: str, world: str):
    for line in config.lines:
        if line_name not in {"all", line.line_id}:
            continue
        for role, endpoint in (("real", line.real), ("virtual", line.virtual)):
            if world not in {"all", role}:
                continue
            yield line, role, endpoint


def _ack(connection, command: int, timeout_s: float) -> dict[str, int] | None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        message = connection.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.25)
        if message is None or int(message.command) != command:
            continue
        return {"command": int(message.command), "result": int(message.result)}
    return None


def _exercise(
    connection,
    line_id: str,
    system_id: int,
    altitude_m: float,
    timeout_s: float,
    force_arm: bool,
):
    mode_requested = None
    if line_id == "apm":
        # ArduCopter accepts NAV_TAKEOFF only from a guided-capable mode.
        connection.set_mode_apm("GUIDED")
        mode_requested = "GUIDED"
    connection.mav.command_long_send(
        system_id,
        1,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,
        21196 if force_arm else 0,
        0,
        0,
        0,
        0,
        0,
    )
    arm = _ack(connection, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, timeout_s)
    connection.mav.command_long_send(
        system_id,
        1,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        altitude_m,
    )
    takeoff = _ack(connection, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, timeout_s)
    accepted = {mavutil.mavlink.MAV_RESULT_ACCEPTED, mavutil.mavlink.MAV_RESULT_IN_PROGRESS}
    return {
        "mode_requested": mode_requested,
        "arm_ack": arm,
        "takeoff_ack": takeoff,
        "accepted": bool(arm and takeoff and arm["result"] in accepted and takeoff["result"] in accepted),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--line", choices=("all", "px4", "apm"), default="all")
    parser.add_argument("--world", choices=("all", "real", "virtual"), default="all")
    parser.add_argument("--timeout-s", type=float, default=15.0)
    parser.add_argument("--exercise", action="store_true", help="send ARM then TAKEOFF")
    parser.add_argument("--confirm-sitl", action="store_true", help="required with --exercise")
    parser.add_argument("--force-arm", action="store_true", help="SITL-only force-arm magic; requires --exercise")
    parser.add_argument("--takeoff-altitude-m", type=float, default=5.0)
    args = parser.parse_args()
    if args.exercise and not args.confirm_sitl:
        parser.error("--exercise requires --confirm-sitl; this tool is for SITL only")
    if args.force_arm and not args.exercise:
        parser.error("--force-arm requires --exercise")
    if not 1.0 <= args.timeout_s <= 120.0:
        parser.error("--timeout-s must be in [1, 120]")
    if not 1.0 <= args.takeoff_altitude_m <= 30.0:
        parser.error("--takeoff-altitude-m must be in [1, 30]")

    config = load_bridge_config(args.config)
    results: dict[str, object] = {}
    connections = []
    passed = True
    try:
        for line, role, endpoint in _selected(config, args.line, args.world):
            key = f"{line.line_id}/{role}"
            connection = mavutil.mavlink_connection(
                endpoint.endpoint,
                source_system=252,
                source_component=190,
                dialect=endpoint.dialect,
                autoreconnect=False,
            )
            connections.append(connection)
            heartbeat = connection.wait_heartbeat(timeout=args.timeout_s)
            item: dict[str, object] = {
                "endpoint": endpoint.endpoint,
                "simulator": endpoint.simulator,
                "dialect": "common" if line.line_id == "px4" else "ardupilotmega",
                "expected_system_id": line.vehicle_id,
            }
            if heartbeat is None:
                item["heartbeat"] = False
                passed = False
            else:
                actual_system_id = int(heartbeat.get_srcSystem())
                item["heartbeat"] = True
                item["system_id"] = actual_system_id
                item["autopilot"] = int(heartbeat.autopilot)
                item["system_id_match"] = actual_system_id == line.vehicle_id
                if actual_system_id != line.vehicle_id:
                    passed = False
                if args.exercise:
                    exercise = _exercise(
                        connection,
                        line.line_id,
                        line.vehicle_id,
                        args.takeoff_altitude_m,
                        args.timeout_s,
                        args.force_arm,
                    )
                    item["exercise"] = exercise
                    passed = passed and bool(exercise["accepted"])
            results[key] = item
    finally:
        for connection in connections:
            connection.close()

    payload = {
        "gate": "PARALLEL_DOMAIN_LIVE_SITL",
        "passed": passed and bool(results),
        "exercise": args.exercise,
        "force_arm": args.force_arm,
        "endpoints": results,
        "note": "Run before the ground bridge binds these UDP listener ports; exercise is SITL-only.",
    }
    print(json.dumps(payload, ensure_ascii=True, indent=2))
    return 0 if payload["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

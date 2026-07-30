"""Run a bounded, read-only UAV3 P9 acceptance check on Windows."""

from __future__ import annotations

import argparse
import asyncio
import json
import time

from aeromind_apm_lite.common.contracts import Heartbeat, VehicleTelemetry
from aeromind_apm_lite.common.credentials import load_shared_secret
from aeromind_apm_lite.ground.serial_server import SerialGroundServer


async def accept(args: argparse.Namespace) -> dict[str, object]:
    server = SerialGroundServer(
        serial_port=args.serial_port,
        serial_baudrate=args.serial_baud,
        vehicle_secrets={3: load_shared_secret("AEROMIND_UAV3_TOKEN")},
        calibration_ids={3: "uav3-local-ned-bench-v1"},
    )
    await server.start()
    try:
        session_id = await server.wait_for_vehicle(3, timeout_s=args.timeout)
        heartbeat = None
        telemetry = None
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline and (
            heartbeat is None or telemetry is None
        ):
            message = await server.next_message(
                3,
                timeout_s=max(0.1, deadline - time.monotonic()),
            )
            if isinstance(message, Heartbeat):
                heartbeat = message
            elif isinstance(message, VehicleTelemetry):
                telemetry = message

        if heartbeat is None:
            raise TimeoutError("authenticated UAV3 session produced no heartbeat")
        if telemetry is None:
            raise TimeoutError("authenticated UAV3 session produced no telemetry")
        if heartbeat.command_output_enabled:
            raise RuntimeError("read-only safety gate is unexpectedly enabled")

        return {
            "accepted": True,
            "vehicle_id": 3,
            "session_id": str(session_id),
            "command_output_enabled": heartbeat.command_output_enabled,
            "software_version": heartbeat.software_version,
            "configuration_hash": heartbeat.configuration_hash,
            "telemetry": telemetry.model_dump(mode="json"),
            "serial_stats": server.serial_stats,
        }
    finally:
        await server.stop()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--serial-port", default="COM3")
    parser.add_argument("--serial-baud", type=int, default=57_600)
    parser.add_argument("--timeout", type=float, default=20.0)
    result = asyncio.run(accept(parser.parse_args()))
    print(json.dumps(result, ensure_ascii=True, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

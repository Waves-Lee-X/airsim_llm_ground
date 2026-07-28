"""Read-only FCU identity and telemetry probe for SITL and bench use."""

from __future__ import annotations

import argparse
import asyncio
import json
from dataclasses import fields
from pathlib import Path
from types import MappingProxyType
from typing import Any

from aeromind_apm_lite.common.config import VehicleRuntimeConfig, load_fleet_config

from .apm_link import ApmLink
from .mavlink.pymavlink_transport import PymavlinkTransport


def select_vehicle(config_path: Path, vehicle_id: int) -> VehicleRuntimeConfig:
    fleet = load_fleet_config(config_path)
    for vehicle in fleet.vehicles:
        if vehicle.vehicle_id == vehicle_id:
            return vehicle
    raise ValueError(f"vehicle_id {vehicle_id} is not present in {config_path}")


def _dataclass_payload(value: Any) -> dict[str, Any]:
    payload: dict[str, Any] = {}
    for item in fields(value):
        field_value = getattr(value, item.name)
        if isinstance(field_value, MappingProxyType):
            field_value = dict(field_value)
        payload[item.name] = field_value
    return payload


async def probe(
    vehicle: VehicleRuntimeConfig,
    *,
    observe_seconds: float,
) -> dict[str, Any]:
    transport = PymavlinkTransport(vehicle.fcu)
    link = ApmLink.from_vehicle_config(transport, vehicle)
    try:
        identity = await link.start()
        await asyncio.sleep(observe_seconds)
        telemetry = link.telemetry_snapshot()
        return {
            "vehicle_id": vehicle.vehicle_id,
            "runtime_host": vehicle.runtime_host,
            "identity": _dataclass_payload(identity),
            "telemetry": _dataclass_payload(telemetry),
        }
    finally:
        await link.stop()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Read FCU HEARTBEAT/AUTOPILOT_VERSION without sending flight commands."
    )
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--vehicle-id", type=int, default=1)
    parser.add_argument("--observe-seconds", type=float, default=1.0)
    args = parser.parse_args(argv)
    if not 0.0 <= args.observe_seconds <= 60.0:
        parser.error("--observe-seconds must be in [0, 60]")

    vehicle = select_vehicle(args.config, args.vehicle_id)
    result = asyncio.run(probe(vehicle, observe_seconds=args.observe_seconds))
    print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

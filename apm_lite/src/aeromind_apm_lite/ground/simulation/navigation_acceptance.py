"""Run one unified mission against an operator-managed AirSim/SITL session."""

from __future__ import annotations

import argparse
import asyncio
import json
import os
from pathlib import Path
from typing import Any

from aeromind_apm_lite.common.config import (
    DeploymentMode,
    FcuTransport,
    VehicleRuntimeConfig,
    load_fleet_config,
)
from aeromind_apm_lite.common.mission import MissionState
from aeromind_apm_lite.onboard import (
    ApmLink,
    NavigationFailureCode,
    NavigationMissionPlan,
    NavigationMissionRunner,
    navigation_mission_result_dict,
)
from aeromind_apm_lite.onboard.mavlink.pymavlink_transport import (
    PymavlinkTransport,
)

from .fault_injection import (
    NavigationFaultInjection,
    NavigationFaultInjector,
    NavigationFaultType,
)


EXPECTED_FAILURES = {
    NavigationFaultType.GPS_LOSS: NavigationFailureCode.GPS_UNHEALTHY,
    NavigationFaultType.HDOP_EXCEEDED: NavigationFailureCode.HDOP_EXCEEDED,
    NavigationFaultType.EKF_FAILURE: NavigationFailureCode.EKF_UNHEALTHY,
    NavigationFaultType.MISSION_EXPIRED: NavigationFailureCode.MISSION_EXPIRED,
    NavigationFaultType.GROUND_LINK_LOSS: NavigationFailureCode.GROUND_LINK_LOST,
}


class NavigationAcceptanceError(RuntimeError):
    pass


def select_navigation_vehicle(
    config_path: str | Path,
    vehicle_id: int,
) -> VehicleRuntimeConfig:
    fleet = load_fleet_config(config_path)
    if fleet.mode != DeploymentMode.SIM:
        raise NavigationAcceptanceError("navigation acceptance requires sim mode")
    matches = [vehicle for vehicle in fleet.vehicles if vehicle.vehicle_id == vehicle_id]
    if len(matches) != 1:
        raise NavigationAcceptanceError(f"vehicle_id {vehicle_id} is not unique")
    vehicle = matches[0]
    if vehicle.fcu.transport != FcuTransport.UDP:
        raise NavigationAcceptanceError("navigation acceptance requires UDP SITL")
    return vehicle


def acceptance_passed(
    terminal_state: MissionState,
    failure_code: NavigationFailureCode | None,
    fault: NavigationFaultType | None,
) -> bool:
    if fault is None:
        return terminal_state == MissionState.COMPLETED and failure_code is None
    return failure_code == EXPECTED_FAILURES[fault]


def _write_report(path: Path, payload: dict[str, Any]) -> None:
    path = Path(path)
    if not path.is_absolute():
        raise ValueError("report path must be absolute")
    path.parent.mkdir(parents=True, exist_ok=True)
    encoded = json.dumps(
        payload,
        ensure_ascii=False,
        allow_nan=False,
        indent=2,
        sort_keys=True,
    ) + "\n"
    temporary = path.with_name(f".{path.name}.tmp-{os.getpid()}")
    temporary.write_text(encoded, encoding="utf-8", newline="\n")
    temporary.replace(path)


async def run_navigation_acceptance(
    vehicle: VehicleRuntimeConfig,
    plan: NavigationMissionPlan,
    *,
    fault: NavigationFaultType | None = None,
) -> dict[str, Any]:
    transport = PymavlinkTransport(vehicle.fcu)
    link = ApmLink.from_vehicle_config(transport, vehicle)
    injector = (
        NavigationFaultInjector(NavigationFaultInjection(fault=fault))
        if fault is not None
        else None
    )
    await link.start()
    try:
        runner = NavigationMissionRunner(
            link,
            safety_limits=None,
            observation_filter=injector,
            preflight_timeout_s=30.0,
            preflight_hold_s=1.0,
        )
        result = await runner.run(plan)
    finally:
        await link.stop()

    passed = acceptance_passed(
        result.terminal_state,
        result.failure_code,
        fault,
    )
    return {
        "gate": "M3_UNIFIED_NAVIGATION_SITL",
        "passed": passed,
        "vehicle_id": vehicle.vehicle_id,
        "fcu_endpoint": vehicle.fcu.endpoint,
        "external_processes_managed": False,
        "fault_injection": fault.value if fault is not None else None,
        "fault_injection_layer": (
            "navigation_health_observation" if fault is not None else None
        ),
        "ardupilot_sensor_state_modified": False,
        "plan": {
            "mission_id": str(plan.mission_id),
            "target_position_ned_m": list(plan.target_position_ned_m),
            "takeoff_altitude_m": plan.takeoff_altitude_m,
            "mission_timeout_s": plan.mission_timeout_s,
            "goto_timeout_s": plan.goto_timeout_s,
        },
        "result": navigation_mission_result_dict(result),
    }


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Run GUIDED/ARM/TAKEOFF/GOTO/HOLD/LAND against an already "
            "running AirSim/ArduCopter SITL session."
        )
    )
    parser.add_argument(
        "--fleet-config",
        type=Path,
        default=Path("configs/sim/fleet.m1.yaml"),
    )
    parser.add_argument("--vehicle-id", type=int, default=1)
    parser.add_argument("--north-m", type=float, default=3.0)
    parser.add_argument("--east-m", type=float, default=0.0)
    parser.add_argument("--altitude-m", type=float, default=2.0)
    parser.add_argument("--mission-timeout-s", type=float, default=90.0)
    parser.add_argument("--goto-timeout-s", type=float, default=15.0)
    parser.add_argument(
        "--fault",
        choices=tuple(fault.value for fault in NavigationFaultType),
    )
    parser.add_argument("--report", type=Path)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_argument_parser().parse_args(argv)
    try:
        vehicle = select_navigation_vehicle(args.fleet_config, args.vehicle_id)
        plan = NavigationMissionPlan(
            target_position_ned_m=(
                args.north_m,
                args.east_m,
                -args.altitude_m,
            ),
            takeoff_altitude_m=args.altitude_m,
            mission_timeout_s=args.mission_timeout_s,
            goto_timeout_s=args.goto_timeout_s,
        )
        fault = NavigationFaultType(args.fault) if args.fault else None
        payload = asyncio.run(
            run_navigation_acceptance(vehicle, plan, fault=fault)
        )
        if args.report is not None:
            _write_report(args.report, payload)
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        payload = {
            "gate": "M3_UNIFIED_NAVIGATION_SITL",
            "passed": False,
            "error": f"{type(exc).__name__}: {exc}",
        }
        print(json.dumps(payload, ensure_ascii=False, indent=2))
        return 2
    print(json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True))
    return 0 if payload["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

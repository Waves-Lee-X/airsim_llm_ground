"""M5 formation acceptance: drive four SITL vehicles through formations.

The script connects to four ArduCopter SITL instances (operator-managed or
launched by itself), flies them through a formation sequence and records the
whole-flight trajectories. A strict spatiotemporal separation check decides
whether the run passed. No real machine command is ever issued.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping

from pymavlink import mavutil

from aeromind_apm_lite.common.config import FcuConnection, FcuTransport
from aeromind_apm_lite.common.formation import (
    FormationType,
    formation_offsets,
    validate_trajectory_separation,
)
from aeromind_apm_lite.ground.simulation.airsim_settings import AirSimSettings
from aeromind_apm_lite.ground.simulation.config import (
    MavlinkUdpEndpoint,
    SimulationLaunchConfig,
    fleet_launch_config,
)
from aeromind_apm_lite.ground.simulation.lifecycle import (
    SimulationRuntime,
    parse_parameter_file,
)
from aeromind_apm_lite.onboard.mavlink.pymavlink_transport import (
    PymavlinkTransport,
)

DEFAULT_AIRSIM_SETTINGS_OUTPUT = (
    Path("/mnt/c/Users/16401/Documents/AirSim/settings.json")
)

_GUIDED = "GUIDED"
_SAMPLE_INTERVAL_S = 0.2


class FormationAcceptanceError(RuntimeError):
    pass


@dataclass(frozen=True)
class VehicleLinkSpec:
    """Minimal link description used when SITL is operator-managed."""

    port: int
    system_id: int
    source_system: int


@dataclass
class _CommandRequest:
    kind: str
    args: tuple[Any, ...]
    future: asyncio.Future[Any]


@dataclass
class VehicleDriver:
    vehicle_id: int
    transport: PymavlinkTransport
    altitude_m: float
    position: tuple[float, float, float] | None = None
    armed: bool | None = None
    mode: str | None = None
    trajectory: list[tuple[float, float, float, float]] = field(
        default_factory=list
    )
    stop: bool = False
    pump_error: str | None = None
    status_texts: list[str] = field(default_factory=list)
    _last_sample_s: float = 0.0
    _last_heartbeat_s: float = 0.0
    _commands: asyncio.Queue[_CommandRequest] = field(
        default_factory=asyncio.Queue
    )
    _goto_target: tuple[float, float, float] | None = None

    async def pump(self) -> None:
        try:
            await self.transport.open()
            await self.transport.request_message_interval(
                mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
                5.0,
            )
        except Exception as exc:
            self.pump_error = f"{type(exc).__name__}: {exc}"
            return
        last_position_seen = time.monotonic()
        interval_retries = 0
        while not self.stop:
            while True:
                try:
                    request = self._commands.get_nowait()
                except asyncio.QueueEmpty:
                    break
                try:
                    result = await self._execute_command(request)
                except Exception as exc:
                    if not request.future.done():
                        request.future.set_exception(exc)
                else:
                    if not request.future.done():
                        request.future.set_result(result)
            now = time.monotonic()
            if now - self._last_heartbeat_s >= 0.5:
                await self.transport.send_heartbeat()
                self._last_heartbeat_s = now
            if self._goto_target is not None:
                north, east, down = self._goto_target
                await self.transport.send_local_position_target(
                    north,
                    east,
                    down,
                    yaw_rad=None,
                )
            envelope = await self.transport.receive(timeout_s=0.05)
            if envelope is None:
                if (
                    interval_retries < 3
                    and time.monotonic() - last_position_seen >= 2.0
                ):
                    interval_retries += 1
                    try:
                        await self.transport.request_message_interval(
                            mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
                            5.0,
                        )
                    except Exception as exc:
                        self.pump_error = f"{type(exc).__name__}: {exc}"
                        return
                continue
            if envelope.name == "LOCAL_POSITION_NED":
                last_position_seen = time.monotonic()
                fields = envelope.fields
                position = (
                    float(fields["x"]),
                    float(fields["y"]),
                    float(fields["z"]),
                )
                self.position = position
                now = time.monotonic()
                if now - self._last_sample_s >= _SAMPLE_INTERVAL_S:
                    self._last_sample_s = now
                    self.trajectory.append((now, *position))
            elif envelope.name == "HEARTBEAT":
                self.armed = bool(envelope.fields.get("armed"))
                self.mode = envelope.fields.get("mode_name")
            elif envelope.name == "STATUSTEXT":
                text = str(envelope.fields.get("text", "")).strip()
                if text and text not in self.status_texts:
                    self.status_texts.append(text)

    async def _execute_command(self, request: _CommandRequest) -> Any:
        kind = request.kind
        args = request.args
        if kind == "set_mode":
            mode_id = await self.transport.mode_id(str(args[0]))
            await self.transport.send_command_long(
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                [1.0, float(mode_id), 0.0, 0.0, 0.0, 0.0, 0.0],
            )
        elif kind == "arm":
            await self.transport.send_command_long(
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            )
        elif kind == "takeoff":
            await self.transport.send_command_long(
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, self.altitude_m],
            )
        elif kind == "land":
            await self.transport.send_command_long(
                mavutil.mavlink.MAV_CMD_NAV_LAND,
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            )
        elif kind == "goto_start":
            self._goto_target = tuple(float(value) for value in args[0])
        elif kind == "goto_stop":
            self._goto_target = None
        else:
            raise FormationAcceptanceError(f"unknown command kind {kind}")
        return None

    async def _send_command(self, kind: str, *args: Any) -> Any:
        future: asyncio.Future[Any] = asyncio.get_running_loop().create_future()
        await self._commands.put(
            _CommandRequest(kind=kind, args=args, future=future)
        )
        return await future

    async def set_mode(self, mode: str) -> None:
        await self._send_command("set_mode", mode)

    async def arm(self) -> None:
        await self._send_command("arm")

    async def takeoff(self) -> None:
        await self._send_command("takeoff")

    async def land(self) -> None:
        await self._send_command("land")

    async def goto(self, target: tuple[float, float, float]) -> None:
        await self._send_command("goto_start", target)
        try:
            await self.wait_until(
                lambda vehicle: (
                    vehicle.position is not None
                    and math.dist(vehicle.position, target) <= 1.2
                ),
                timeout_s=60.0,
                label=f"goto {target}",
            )
        finally:
            await self._send_command("goto_stop")

    async def wait_until(self, predicate, timeout_s: float, label: str) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if predicate(self):
                return
            if self.pump_error is not None:
                raise FormationAcceptanceError(
                    f"vehicle {self.vehicle_id} pump failed: {self.pump_error}"
                )
            await asyncio.sleep(0.1)
        detail = f" (pump error: {self.pump_error})" if self.pump_error else ""
        texts = "; ".join(self.status_texts[-6:])
        raise FormationAcceptanceError(
            f"vehicle {self.vehicle_id} timed out waiting for {label}{detail} "
            f"[mode={self.mode} armed={self.armed} pos={self.position} "
            f"status={texts}]"
        )


def formation_run_targets(
    formation: FormationType,
    leader_target: tuple[float, float, float],
    spacing_m: float,
    vehicle_count: int,
) -> dict[int, tuple[float, float, float]]:
    offsets = formation_offsets(formation, vehicle_count, spacing_m)
    return {
        vehicle_id: (
            round(leader_target[0] + offset_x, 6),
            round(leader_target[1] + offset_y, 6),
            round(leader_target[2], 6),
        )
        for vehicle_id, (offset_x, offset_y) in enumerate(offsets, start=1)
    }


async def run_formation_cycle(
    drivers: Mapping[int, VehicleDriver],
    formation_sequence: list[FormationType],
    leader_target: tuple[float, float, float],
    spacing_m: float,
    hold_s: float,
) -> dict[str, Any]:
    vehicles = list(drivers.values())
    try:
        for vehicle in vehicles:
            await vehicle.set_mode(_GUIDED)
        await asyncio.gather(
            *(
                vehicle.wait_until(
                    lambda current: current.mode == _GUIDED,
                    timeout_s=20.0,
                    label="GUIDED",
                )
                for vehicle in vehicles
            )
        )

        async def arm_with_retry(vehicle: VehicleDriver) -> None:
            for attempt in range(4):
                await vehicle.arm()
                try:
                    await vehicle.wait_until(
                        lambda current: current.armed is True,
                        timeout_s=12.0,
                        label=f"arm attempt {attempt + 1}",
                    )
                    return
                except FormationAcceptanceError:
                    if attempt == 3:
                        raise

        await asyncio.gather(*(arm_with_retry(vehicle) for vehicle in vehicles))
        await asyncio.gather(*(vehicle.takeoff() for vehicle in vehicles))
        await asyncio.gather(
            *(
                vehicle.wait_until(
                    lambda current, altitude=vehicle.altitude_m: (
                        current.position is not None
                        and current.position[2] <= -0.7 * altitude
                    ),
                    timeout_s=45.0,
                    label="takeoff",
                )
                for vehicle in vehicles
            )
        )
        for formation in formation_sequence:
            targets = formation_run_targets(
                formation,
                leader_target,
                spacing_m,
                len(vehicles),
            )
            await asyncio.gather(
                *(vehicles[index - 1].goto(targets[index]) for index in sorted(targets))
            )
            await asyncio.sleep(hold_s)
        await asyncio.gather(*(vehicle.land() for vehicle in vehicles))
        await asyncio.gather(
            *(
                vehicle.wait_until(
                    lambda current: (
                        current.position is not None
                        and current.position[2] >= -0.2
                    ),
                    timeout_s=40.0,
                    label="land",
                )
                for vehicle in vehicles
            )
        )
    finally:
        for vehicle in vehicles:
            vehicle.stop = True
    return {
        "formation_sequence": [formation.value for formation in formation_sequence],
        "leader_target_map_m": list(leader_target),
        "spacing_m": spacing_m,
    }


async def run_fleet_formation_acceptance(
    launch_config: SimulationLaunchConfig,
    *,
    formations: list[FormationType],
    runs: int,
    leader_north_m: float,
    leader_east_m: float,
    altitude_m: float,
    spacing_m: float,
    hold_s: float,
    run_tag: str,
) -> list[dict[str, Any]]:
    link_specs = [
        VehicleLinkSpec(
            port=instance.mavlink.port,
            system_id=instance.system_id,
            source_system=201 + index,
        )
        for index, instance in enumerate(launch_config.instances)
    ]
    return await run_fleet_formation_acceptance_specs(
        link_specs,
        formations=formations,
        runs=runs,
        leader_north_m=leader_north_m,
        leader_east_m=leader_east_m,
        altitude_m=altitude_m,
        spacing_m=spacing_m,
        hold_s=hold_s,
        run_tag=run_tag,
    )


async def run_fleet_formation_acceptance_specs(
    link_specs: list[VehicleLinkSpec],
    *,
    formations: list[FormationType],
    runs: int,
    leader_north_m: float,
    leader_east_m: float,
    altitude_m: float,
    spacing_m: float,
    hold_s: float,
    run_tag: str,
) -> list[dict[str, Any]]:
    reports: list[dict[str, Any]] = []
    for run_index in range(1, runs + 1):
        drivers: dict[int, VehicleDriver] = {}
        for spec in link_specs:
            transport = PymavlinkTransport(
                FcuConnection(
                    transport=FcuTransport.UDP,
                    endpoint=f"udpin:0.0.0.0:{spec.port}",
                    source_system=spec.source_system,
                    source_component=191,
                    target_system=spec.system_id,
                    target_component=1,
                )
            )
            drivers[spec.system_id] = VehicleDriver(
                vehicle_id=spec.system_id,
                transport=transport,
                altitude_m=altitude_m,
            )
        pumps: list[asyncio.Task[None]] = []
        try:
            pumps = [
                asyncio.create_task(driver.pump()) for driver in drivers.values()
            ]
            await asyncio.gather(
                *(
                    driver.wait_until(
                        lambda current: current.position is not None,
                        timeout_s=60.0,
                        label="position",
                    )
                    for driver in drivers.values()
                )
            )
            leader_target = (leader_north_m, leader_east_m, -altitude_m)
            details = await run_formation_cycle(
                drivers,
                formations,
                leader_target,
                spacing_m,
                hold_s,
            )
            trajectories: dict[int, list[tuple[float, float, float, float]]] = {}
            for vehicle_id, driver in drivers.items():
                if not driver.trajectory:
                    continue
                start_time = driver.trajectory[0][0]
                trajectories[vehicle_id] = [
                    (round(sample[0] - start_time, 3), *sample[1:])
                    for sample in driver.trajectory
                ]
            report = dict(details)
            try:
                separation = validate_trajectory_separation(
                    trajectories,
                    min_distance_m=spacing_m,
                )
                passed = True
                separation_error = None
            except ValueError as exc:
                separation = {}
                passed = False
                separation_error = str(exc)
            report.update(
                {
                    "gate": "M5_FORMATION_SITL",
                    "run": run_index,
                    "run_tag": run_tag,
                    "passed": passed,
                    "min_distance_m": separation.get("min_distance_m"),
                    "closest_pair": separation.get("closest_pair"),
                    "separation_error": separation_error,
                    "sample_counts": {
                        str(vehicle_id): len(driver.trajectory)
                        for vehicle_id, driver in drivers.items()
                    },
                }
            )
            reports.append(report)
        finally:
            for driver in drivers.values():
                driver.stop = True
            for task in pumps:
                task.cancel()
            await asyncio.gather(*pumps, return_exceptions=True)
            for driver in drivers.values():
                await asyncio.gather(
                    driver.transport.close(),
                    return_exceptions=True,
                )
    return reports


def _instance_parameter_files(
    run_dir: Path,
    vehicle_count: int,
) -> list[Path]:
    """Generate per-vehicle parameter files with the M1 safety freeze."""
    from aeromind_apm_lite.ground.simulation.config import (
        DEFAULT_M1_PARAMETER_FILE,
    )

    base = parse_parameter_file(DEFAULT_M1_PARAMETER_FILE)
    outputs = []
    for index in range(vehicle_count):
        parameters = dict(base)
        parameters["MAV_SYSID"] = str(index + 1)
        parameters["MAV_GCS_SYSID"] = str(201 + index)
        output = run_dir / "params" / f"arducopter-m1-{index + 1}.parm"
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(
            "".join(
                f"{name} {value}\n"
                for name, value in sorted(parameters.items())
            ),
            encoding="ascii",
            newline="\n",
        )
        outputs.append(output)
    return outputs


def _disable_gcs_failsafe(port: int, system_id: int, source_system: int) -> None:
    """Temporarily disable the GCS failsafe so SITL arming is allowed."""
    conn = mavutil.mavlink_connection(
        f"udpin:0.0.0.0:{port}",
        source_system=source_system,
        source_component=191,
    )
    try:
        conn.wait_heartbeat(timeout=10.0)
        conn.mav.param_set_send(
            conn.target_system,
            conn.target_component,
            b"FS_GCS_ENABLE",
            0.0,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        )
        deadline = time.monotonic() + 6.0
        confirmed = False
        while time.monotonic() < deadline:
            message = conn.recv_match(
                type="PARAM_VALUE",
                blocking=True,
                timeout=1.0,
            )
            if message is None:
                continue
            raw_id = message.param_id
            param_id = (
                raw_id.decode(errors="ignore").rstrip("\x00")
                if isinstance(raw_id, bytes)
                else str(raw_id).rstrip("\x00")
            )
            if param_id == "FS_GCS_ENABLE":
                confirmed = True
                print(
                    f"vehicle {system_id}: FS_GCS_ENABLE={message.param_value}",
                    flush=True,
                )
                break
        if not confirmed:
            print(
                f"vehicle {system_id}: FS_GCS_ENABLE change not confirmed",
                flush=True,
            )
    finally:
        conn.close()


def _write_report(path: Path, payload: dict[str, Any]) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
        newline="\n",
    )


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Fly four ArduCopter SITL vehicles through formation sequences "
            "and verify spatiotemporal separation."
        )
    )
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument(
        "--formations",
        default="line,v,diamond",
        help="Formation sequence per run (comma separated).",
    )
    parser.add_argument("--runs", type=int, default=1)
    parser.add_argument("--leader-north-m", type=float, default=8.0)
    parser.add_argument("--leader-east-m", type=float, default=0.0)
    parser.add_argument("--altitude-m", type=float, default=2.0)
    parser.add_argument("--spacing-m", type=float, default=3.0)
    parser.add_argument("--hold-s", type=float, default=3.0)
    parser.add_argument("--base-port", type=int, default=14550)
    parser.add_argument("--port-stride", type=int, default=10)
    parser.add_argument("--vehicle-count", type=int, default=4)
    parser.add_argument(
        "--launch-sitl",
        action="store_true",
        help="Launch and stop the four SITL instances itself.",
    )
    parser.add_argument(
        "--sitl-only",
        action="store_true",
        help="Start the four SITL instances and keep them running without a mission.",
    )
    parser.add_argument(
        "--airsim-settings-output",
        type=Path,
        default=DEFAULT_AIRSIM_SETTINGS_OUTPUT,
        help="Absolute settings.json path used by the UE/AirSim session.",
    )
    parser.add_argument(
        "--wsl-ip",
        help="Override the WSL IP used by the launch config (useful when running on Windows).",
    )
    parser.add_argument(
        "--mavlink-host",
        default="127.0.0.1",
        help="Host the SITL instances send MAVLink to (default 127.0.0.1).",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_argument_parser().parse_args(argv)
    run_dir = Path(args.run_dir)
    formations = [FormationType(value) for value in args.formations.split(",")]

    launch_config: SimulationLaunchConfig | None = None
    if args.launch_sitl or args.sitl_only:
        from dataclasses import replace

        launch_config = fleet_launch_config(
            4,
            spacing_m=args.spacing_m,
            state_directory=run_dir / "sitl",
            airsim_settings_path=args.airsim_settings_output,
            wsl_ip=args.wsl_ip,
        )
        parameter_files = _instance_parameter_files(
            run_dir, len(launch_config.instances)
        )
        launch_config = replace(
            launch_config,
            instances=tuple(
                replace(
                    instance,
                    parameter_file=parameter_file,
                    mavlink=MavlinkUdpEndpoint.for_instance(
                        instance.instance,
                        host=args.mavlink_host,
                    ),
                )
                for instance, parameter_file in zip(
                    launch_config.instances, parameter_files
                )
            ),
        )

    async def run_all() -> list[dict[str, Any]]:
        runtime: SimulationRuntime | None = None
        try:
            if args.launch_sitl:
                AirSimSettings(launch_config).write(args.airsim_settings_output)
                runtime = SimulationRuntime(launch_config)
                await runtime.start()
                print("SITL instances launched")
            return await run_fleet_formation_acceptance(
                launch_config,
                formations=formations,
                runs=args.runs,
                leader_north_m=args.leader_north_m,
                leader_east_m=args.leader_east_m,
                altitude_m=args.altitude_m,
                spacing_m=args.spacing_m,
                hold_s=args.hold_s,
                run_tag=run_dir.name,
            )
        finally:
            if runtime is not None:
                try:
                    await runtime.stop()
                    print("SITL instances stopped")
                except Exception as exc:  # pragma: no cover - cleanup path
                    print(f"SITL stop warning: {exc}")

    if args.sitl_only:
        if launch_config is None:
            raise FormationAcceptanceError("internal error: launch config missing")

        async def start_only() -> None:
            runtime = SimulationRuntime(launch_config)
            await runtime.start()
            print("SITL_READY")

        asyncio.run(start_only())
        return 0
    if not args.launch_sitl:
        link_specs = [
            VehicleLinkSpec(
                port=args.base_port + args.port_stride * index,
                system_id=index + 1,
                source_system=201 + index,
            )
            for index in range(args.vehicle_count)
        ]
        for spec in link_specs:
            _disable_gcs_failsafe(spec.port, spec.system_id, spec.source_system)
        reports = asyncio.run(
            run_fleet_formation_acceptance_specs(
                link_specs,
                formations=formations,
                runs=args.runs,
                leader_north_m=args.leader_north_m,
                leader_east_m=args.leader_east_m,
                altitude_m=args.altitude_m,
                spacing_m=args.spacing_m,
                hold_s=args.hold_s,
                run_tag=run_dir.name,
            )
        )
    else:
        reports = asyncio.run(run_all())
    summary = {
        "gate": "M5_FORMATION_SITL",
        "passed": bool(reports) and all(report["passed"] for report in reports),
        "reports": reports,
    }
    _write_report(run_dir / "summary.json", summary)
    print(json.dumps(summary, ensure_ascii=False, indent=2, sort_keys=True))
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

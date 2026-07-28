"""Executable M1 AirSim/ArduPilot acceptance gate."""

from __future__ import annotations

import argparse
import asyncio
import hashlib
import json
import os
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Awaitable, Callable
from uuid import uuid4

from aeromind_apm_lite.common.config import (
    DeploymentMode,
    VehicleRuntimeConfig,
    load_fleet_config,
)
from aeromind_apm_lite.onboard.apm_link import ApmLink, CommandHandle
from aeromind_apm_lite.onboard.flight_commands import FlightCommandService
from aeromind_apm_lite.onboard.mavlink.models import (
    CommandStatus,
    MavLandedState,
    TelemetrySnapshot,
)
from aeromind_apm_lite.onboard.mavlink.pymavlink_transport import PymavlinkTransport
from aeromind_apm_lite.onboard.probe import probe

from .config import (
    ARDUPILOT_COMMIT,
    AirSimProcessConfig,
    ArduPilotBuild,
    SimulationLaunchConfig,
    SimulationPaths,
    SitlInstanceConfig,
)
from .fixed_mission import (
    FixedMissionBatchResult,
    FixedMissionRunner,
    MissionEvidenceJournal,
    command_result_dict,
    mission_result_dict,
)
from .lifecycle import SimulationRuntime, windows_path_from_wsl

DEFAULT_AIRSIM_EXECUTABLE = Path(
    "/mnt/d/data/epic/UE_4.27/Engine/Binaries/Win64/UE4Editor.exe"
)
DEFAULT_AIRSIM_PROJECT = Path(
    "/mnt/d/AirSim/AirSim-1.8.1-windows/Unreal/Environments/Blocks/Blocks.uproject"
)
DEFAULT_STATE_ROOT = Path("/mnt/d/AirSim/aeromind-apm-lite-m1")


class AcceptanceError(RuntimeError):
    """The M1 environment or acceptance sequence could not be completed."""


@dataclass(frozen=True, slots=True)
class AcceptancePaths:
    run_directory: Path
    probe_path: Path
    readiness_path: Path
    mission_journal_path: Path
    rtl_path: Path
    summary_path: Path

    @classmethod
    def create(cls, state_root: Path) -> "AcceptancePaths":
        root = Path(state_root)
        if not root.is_absolute() or root == Path(root.anchor):
            raise ValueError("acceptance state root must be an absolute non-root path")
        timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        run_directory = root / f"run-{timestamp}-{os.getpid()}-{uuid4().hex[:8]}"
        return cls(
            run_directory=run_directory,
            probe_path=run_directory / "probe.json",
            readiness_path=run_directory / "flight-ready.json",
            mission_journal_path=run_directory / "fixed-missions.jsonl",
            rtl_path=run_directory / "rtl.json",
            summary_path=run_directory / "summary.json",
        )


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    encoded = json.dumps(
        payload,
        ensure_ascii=True,
        allow_nan=False,
        indent=2,
        sort_keys=True,
    ) + "\n"
    temporary = path.with_name(f".{path.name}.tmp-{os.getpid()}")
    temporary.write_text(encoded, encoding="utf-8", newline="\n")
    temporary.replace(path)


def select_m1_vehicle(config_path: Path) -> VehicleRuntimeConfig:
    fleet = load_fleet_config(config_path)
    if fleet.mode != DeploymentMode.SIM:
        raise AcceptanceError("M1 acceptance requires a simulation fleet config")
    if len(fleet.vehicles) != 1 or fleet.vehicles[0].vehicle_id != 1:
        raise AcceptanceError("M1 acceptance requires exactly vehicle_id 1")
    vehicle = fleet.vehicles[0]
    if vehicle.fcu.endpoint != "udpin:0.0.0.0:14550":
        raise AcceptanceError("M1 vehicle must listen for SITL on udpin:0.0.0.0:14550")
    return vehicle


def build_launch_config(
    *,
    vehicle: VehicleRuntimeConfig,
    parameter_file: Path,
    paths: AcceptancePaths,
    ardupilot_root: Path,
    airsim_executable: Path,
    airsim_project: Path,
    distro_name: str | None = None,
) -> SimulationLaunchConfig:
    for label, path in (
        ("M1 parameter file", parameter_file),
        ("AirSim executable", airsim_executable),
        ("AirSim Blocks project", airsim_project),
    ):
        if not path.is_file():
            raise AcceptanceError(f"{label} does not exist: {path}")

    build = ArduPilotBuild(
        source_root=ardupilot_root,
        executable=ardupilot_root / "build" / "sitl" / "bin" / "arducopter",
    )
    instance = SitlInstanceConfig(
        instance=0,
        build=build,
        parameter_file=parameter_file,
        gcs_system_id=vehicle.fcu.source_system,
    )
    project_argument = windows_path_from_wsl(
        airsim_project,
        distro_name=distro_name,
    )
    process = AirSimProcessConfig(
        executable=airsim_executable,
        arguments=(
            project_argument,
            "-game",
            "-windowed",
            "-ResX=1280",
            "-ResY=720",
            "-NoSound",
            "-stdout",
            "-FullStdOutLogOutput",
        ),
        working_directory=airsim_project.parent,
    )
    simulation_paths = SimulationPaths(
        state_directory=paths.run_directory,
        airsim_settings_path=paths.run_directory / "settings.json",
    )
    return SimulationLaunchConfig(
        instances=(instance,),
        paths=simulation_paths,
        airsim_process=process,
        startup_timeout_s=30.0,
        startup_grace_s=1.0,
        terminate_timeout_s=10.0,
        kill_timeout_s=5.0,
    )


async def wait_for_probe(
    runtime: SimulationRuntime,
    vehicle: VehicleRuntimeConfig,
    *,
    ready_timeout_s: float,
) -> dict[str, Any]:
    deadline = asyncio.get_running_loop().time() + ready_timeout_s
    errors: list[str] = []
    while True:
        runtime.assert_healthy()
        try:
            return await probe(vehicle, observe_seconds=1.0)
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            errors.append(f"{type(exc).__name__}: {exc}")
        remaining = deadline - asyncio.get_running_loop().time()
        if remaining <= 0.0:
            tail = "; ".join(errors[-3:])
            raise AcceptanceError(
                f"SITL probe did not become ready within {ready_timeout_s:g}s: {tail}"
            )
        await asyncio.sleep(min(2.0, remaining))


def _flight_ready_reason(
    snapshot: TelemetrySnapshot,
    *,
    freshness_s: float,
    required_mode: str | None = None,
) -> str | None:
    if not snapshot.fcu_link_ok:
        return "FCU heartbeat is not fresh"
    if snapshot.armed is not False:
        return "vehicle must be disarmed before acceptance"
    if snapshot.prearm_ok is not True:
        return "ArduPilot pre-arm health is not ready"
    if snapshot.gps_healthy is not True:
        return "ArduPilot GPS health bit is not ready"
    if snapshot.gps_fix_type is None or snapshot.gps_fix_type < 3:
        return "GPS 3D fix is not ready"
    if required_mode is not None and snapshot.mode != required_mode:
        return f"vehicle mode is not {required_mode}"
    if snapshot.local_position_ned_m is None:
        return "LOCAL_POSITION_NED is unavailable"
    if snapshot.global_position_deg_m is None:
        return "GLOBAL_POSITION_INT is unavailable"
    if snapshot.velocity_ned_m_s is None:
        return "velocity telemetry is unavailable"
    if snapshot.home_position_deg_m is None:
        return "HOME_POSITION is unavailable"
    if snapshot.landed_state != int(MavLandedState.ON_GROUND):
        return "EXTENDED_SYS_STATE does not report ON_GROUND"

    maximum_ages = {
        "heartbeat": freshness_s,
        "prearm": freshness_s,
        "gps_health": freshness_s,
        "gps": freshness_s,
        "local_position": freshness_s,
        "global_position": freshness_s,
        "velocity": freshness_s,
        "landed_state": freshness_s,
        # HOME_POSITION is intentionally requested at 0.5 Hz.
        "home": max(3.0, freshness_s),
    }
    for field_name, maximum_age in maximum_ages.items():
        age = snapshot.field_ages_s.get(field_name)
        if age is None or age > maximum_age:
            return f"{field_name} telemetry age exceeds {maximum_age:g}s"
    return None


async def wait_for_flight_ready(
    runtime: SimulationRuntime,
    link: ApmLink,
    *,
    timeout_s: float,
    freshness_s: float,
    hold_s: float = 1.0,
    required_mode: str | None = None,
) -> TelemetrySnapshot:
    """Require stable ArduPilot navigation and pre-arm health before commands."""

    loop = asyncio.get_running_loop()
    deadline = loop.time() + timeout_s
    candidate_since: float | None = None
    last_reason = "no telemetry received"
    last_snapshot = link.telemetry_snapshot()
    while loop.time() < deadline:
        runtime.assert_healthy()
        snapshot = link.telemetry_snapshot()
        reason = _flight_ready_reason(
            snapshot,
            freshness_s=freshness_s,
            required_mode=required_mode,
        )
        if reason is None:
            if candidate_since is None:
                candidate_since = loop.time()
            elif loop.time() - candidate_since >= hold_s:
                return snapshot
        else:
            candidate_since = None
            last_reason = reason
        last_snapshot = snapshot
        await asyncio.sleep(0.1)
    status = last_snapshot.last_status_text or "<no STATUSTEXT>"
    raise AcceptanceError(
        f"flight readiness timed out after {timeout_s:g}s: {last_reason}; "
        f"last STATUSTEXT={status!r}"
    )


def _readiness_payload(snapshot: TelemetrySnapshot) -> dict[str, Any]:
    return {
        "observed_monotonic_s": snapshot.observed_monotonic_s,
        "fcu_link_ok": snapshot.fcu_link_ok,
        "armed": snapshot.armed,
        "mode": snapshot.mode,
        "prearm_ok": snapshot.prearm_ok,
        "gps_healthy": snapshot.gps_healthy,
        "gps_fix_type": snapshot.gps_fix_type,
        "satellites_visible": snapshot.satellites_visible,
        "local_position_ned_m": snapshot.local_position_ned_m,
        "global_position_deg_m": snapshot.global_position_deg_m,
        "home_position_deg_m": snapshot.home_position_deg_m,
        "landed_state": snapshot.landed_state,
        "last_status_text": snapshot.last_status_text,
        "field_ages_s": dict(snapshot.field_ages_s),
    }


async def _await_step(
    name: str,
    create_handle: Callable[[], CommandHandle],
    steps: list[dict[str, Any]],
) -> bool:
    result = await create_handle().result
    steps.append({"name": name, "result": command_result_dict(result)})
    return result.status == CommandStatus.COMPLETED


async def run_rtl_check(
    link: ApmLink,
    *,
    altitude_m: float = 2.0,
    before_arm: Callable[[], Awaitable[None]] | None = None,
) -> dict[str, Any]:
    """Fly away from Home and require RTL physical landing evidence."""

    service = FlightCommandService(link)
    steps: list[dict[str, Any]] = []
    recovery: list[dict[str, Any]] = []
    arm_attempted = False
    completed = False
    detail = ""
    try:
        if not await _await_step(
            "guided", lambda: service.set_mode(link.guided_mode), steps
        ):
            detail = "GUIDED did not complete"
            return _rtl_result(completed, steps, recovery, detail)
        if before_arm is not None:
            await before_arm()
        arm_attempted = True
        if not await _await_step("arm", service.arm, steps):
            detail = "ARM did not complete"
            return _rtl_result(completed, steps, recovery, detail)
        if not await _await_step(
            "takeoff_2m", lambda: service.takeoff(altitude_m), steps
        ):
            detail = "TAKEOFF did not complete"
            return _rtl_result(completed, steps, recovery, detail)

        position = link.telemetry_snapshot().local_position_ned_m
        if position is None:
            raise AcceptanceError("RTL check requires LOCAL_POSITION_NED after takeoff")
        if not await _await_step(
            "move_3m_from_home",
            lambda: service.goto_local_ned(
                position[0] + 3.0,
                position[1],
                -altitude_m,
            ),
            steps,
        ):
            detail = "pre-RTL movement did not complete"
            return _rtl_result(completed, steps, recovery, detail)
        if not await _await_step("rtl", service.rtl, steps):
            detail = "RTL did not complete with Home, landed and disarmed evidence"
            return _rtl_result(completed, steps, recovery, detail)
        completed = True
        detail = "RTL returned to Home, landed and disarmed with fresh telemetry"
        return _rtl_result(completed, steps, recovery, detail)
    except asyncio.CancelledError:
        raise
    except Exception as exc:
        detail = f"RTL runner exception: {type(exc).__name__}: {exc}"
        return _rtl_result(completed, steps, recovery, detail)
    finally:
        if arm_attempted and not completed:
            try:
                result = await service.land().result
                recovery.append(
                    {"name": "recovery_land", "result": command_result_dict(result)}
                )
            except Exception as exc:
                recovery.append(
                    {
                        "name": "recovery_land",
                        "error": f"{type(exc).__name__}: {exc}",
                    }
                )


def _rtl_result(
    completed: bool,
    steps: list[dict[str, Any]],
    recovery: list[dict[str, Any]],
    detail: str,
) -> dict[str, Any]:
    return {
        "completed": completed,
        "steps": steps,
        "recovery_steps": recovery,
        "detail": detail,
    }


def _batch_payload(batch: FixedMissionBatchResult) -> dict[str, Any]:
    return {
        "batch_id": str(batch.batch_id),
        "terminal_state": batch.terminal_state.value,
        "requested_runs": batch.requested_runs,
        "completed_runs": batch.completed_runs,
        "success_rate": batch.success_rate,
        "runs": [mission_result_dict(result) for result in batch.runs],
    }


async def run_acceptance(
    *,
    launch: SimulationLaunchConfig,
    vehicle: VehicleRuntimeConfig,
    paths: AcceptancePaths,
    repetitions: int,
    minimum_successes: int,
    ready_timeout_s: float,
) -> dict[str, Any]:
    if not 1 <= minimum_successes <= repetitions:
        raise ValueError("minimum_successes must be in [1, repetitions]")

    runtime = SimulationRuntime(launch)
    probe_payload: dict[str, Any] | None = None
    batch: FixedMissionBatchResult | None = None
    rtl_payload: dict[str, Any] | None = None
    try:
        await runtime.start()
        probe_payload = await wait_for_probe(
            runtime,
            vehicle,
            ready_timeout_s=ready_timeout_s,
        )
        _write_json(paths.probe_path, probe_payload)

        transport = PymavlinkTransport(vehicle.fcu)
        link = ApmLink.from_vehicle_config(transport, vehicle)
        await link.start()
        try:
            ready_snapshot = await wait_for_flight_ready(
                runtime,
                link,
                timeout_s=ready_timeout_s,
                freshness_s=vehicle.telemetry_freshness_s,
            )
            _write_json(paths.readiness_path, _readiness_payload(ready_snapshot))

            async def wait_for_guided_arm_ready() -> None:
                await wait_for_flight_ready(
                    runtime,
                    link,
                    timeout_s=min(15.0, ready_timeout_s),
                    freshness_s=vehicle.telemetry_freshness_s,
                    hold_s=1.0,
                    required_mode=link.guided_mode,
                )

            runner = FixedMissionRunner(
                link,
                journal=MissionEvidenceJournal(paths.mission_journal_path),
                before_arm=wait_for_guided_arm_ready,
            )
            batch = await runner.run_repeated(repetitions, inter_run_delay_s=1.0)
            rtl_payload = await run_rtl_check(
                link,
                before_arm=wait_for_guided_arm_ready,
            )
            _write_json(paths.rtl_path, rtl_payload)
        finally:
            await link.stop()
    finally:
        await runtime.stop()

    if probe_payload is None or batch is None or rtl_payload is None:
        raise AcceptanceError("acceptance ended without all evidence records")
    gate_passed = (
        batch.completed_runs >= minimum_successes and rtl_payload["completed"]
    )
    settings_path = launch.paths.airsim_settings_path
    parameter_path = launch.instances[0].parameter_file
    summary = {
        "gate": "M1_SINGLE_VEHICLE_AIRSIM_SITL",
        "passed": gate_passed,
        "required_mission_successes": minimum_successes,
        "completed_mission_runs": batch.completed_runs,
        "requested_mission_runs": batch.requested_runs,
        "rtl_completed": rtl_payload["completed"],
        "ardupilot_commit": ARDUPILOT_COMMIT,
        "ardupilot_binary_sha256": _sha256(
            launch.instances[0].build.executable
        ),
        "parameter_file_sha256": _sha256(parameter_path),
        "airsim_settings_sha256": _sha256(settings_path),
        "wsl_ip": launch.wsl_ip,
        "windows_host_ip": launch.instances[0].simulator_address,
        "evidence": {
            "probe": str(paths.probe_path),
            "flight_readiness": str(paths.readiness_path),
            "fixed_missions": str(paths.mission_journal_path),
            "rtl": str(paths.rtl_path),
        },
        "batch": _batch_payload(batch),
    }
    _write_json(paths.summary_path, summary)
    return summary


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run the AeroMind APM Lite M1 AirSim/ArduPilot acceptance gate."
    )
    parser.add_argument("--fleet-config", type=Path, required=True)
    parser.add_argument("--parameter-file", type=Path, required=True)
    parser.add_argument("--ardupilot-root", type=Path, default=Path("/home/waves/ardupilot"))
    parser.add_argument(
        "--airsim-executable", type=Path, default=DEFAULT_AIRSIM_EXECUTABLE
    )
    parser.add_argument("--airsim-project", type=Path, default=DEFAULT_AIRSIM_PROJECT)
    parser.add_argument("--state-root", type=Path, default=DEFAULT_STATE_ROOT)
    parser.add_argument("--repetitions", type=int, default=10)
    parser.add_argument("--minimum-successes", type=int, default=9)
    parser.add_argument("--ready-timeout-s", type=float, default=120.0)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if not 1 <= args.repetitions <= 100:
        raise SystemExit("--repetitions must be in [1, 100]")
    if not 1 <= args.minimum_successes <= args.repetitions:
        raise SystemExit("--minimum-successes must be in [1, repetitions]")
    if not 10.0 <= args.ready_timeout_s <= 600.0:
        raise SystemExit("--ready-timeout-s must be in [10, 600]")

    paths = AcceptancePaths.create(args.state_root)
    try:
        vehicle = select_m1_vehicle(args.fleet_config)
        launch = build_launch_config(
            vehicle=vehicle,
            parameter_file=args.parameter_file,
            paths=paths,
            ardupilot_root=args.ardupilot_root,
            airsim_executable=args.airsim_executable,
            airsim_project=args.airsim_project,
        )
        summary = asyncio.run(
            run_acceptance(
                launch=launch,
                vehicle=vehicle,
                paths=paths,
                repetitions=args.repetitions,
                minimum_successes=args.minimum_successes,
                ready_timeout_s=args.ready_timeout_s,
            )
        )
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        error = {
            "gate": "M1_SINGLE_VEHICLE_AIRSIM_SITL",
            "passed": False,
            "error": f"{type(exc).__name__}: {exc}",
            "run_directory": str(paths.run_directory),
        }
        _write_json(paths.summary_path, error)
        print(json.dumps(error, ensure_ascii=False, indent=2))
        return 2

    print(json.dumps(summary, ensure_ascii=False, indent=2, sort_keys=True))
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
# flake8: noqa: E501
"""Run the live PX4/APM parallel-domain loop with explicit SITL approval."""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import os
import shlex
import subprocess
import sys
import time
import zipfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Sequence
from uuid import UUID, uuid4

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from aeromind_apm_lite.common.coordinates import (  # noqa: E402
    GeoReferenceStore,
    load_georeference,
)
from aeromind_apm_lite.common.trajectory import (  # noqa: E402
    TrajectoryComparisonReport,
    TrajectoryEvidence,
    TrajectoryRole,
    load_evidence_file,
)
from aeromind_apm_lite.ground.parallel_domain.calibration import (  # noqa: E402
    freeze_georeference,
    load_calibration_dataset,
    write_calibration_report,
    write_freeze_receipt,
)
from aeromind_apm_lite.ground.parallel_domain.config import (  # noqa: E402
    BridgeConfig,
    load_bridge_config,
)
from aeromind_apm_lite.ground.parallel_domain.core import (  # noqa: E402
    confirmation_digest_for,
)
from aeromind_apm_lite.ground.parallel_domain.live_evidence import (  # noqa: E402
    build_live_evidence_package,
    software_tree_sha256,
    verify_live_evidence_package,
)
from aeromind_apm_lite.ground.parallel_domain.models import (  # noqa: E402
    ApprovalCommand,
    MissionAction,
    MissionPlan,
    MissionStep,
)
from aeromind_apm_lite.ground.parallel_domain.service import (  # noqa: E402
    ParallelDomainService,
    write_approval_to_spool,
    write_plan_to_spool,
)
from aeromind_apm_lite.ground.parallel_domain.storage import BridgeStore  # noqa: E402


LINE_SPECS: dict[str, dict[str, Any]] = {
    "px4": {
        "vehicle_id": 1,
        "home_y_m": 0.0,
        "sessions": ("pd-px4-real", "pd-px4-virtual"),
        "rpc_port": 41451,
        "real_label": "实机替身（仿真/HIL：Gazebo Classic + PX4 SITL）",
        "twin_label": "孪生（仿真/孪生：AirSim UE + PX4 SITL）",
        "flight_controller": "PX4-SITL",
        "real_simulator": "Gazebo-Classic",
        "calibration_fixture": "sitl-ground-truth-roundtrip.json",
    },
    "apm": {
        "vehicle_id": 2,
        "home_y_m": 3.0,
        "sessions": ("pd-apm-real", "pd-apm-virtual"),
        "rpc_port": 41452,
        "real_label": "实机替身（仿真/HIL：ArduPilot 原生 SITL）",
        "twin_label": "孪生（仿真/孪生：AirSim UE + ArduCopter SITL）",
        "flight_controller": "ArduCopter-SITL",
        "real_simulator": "ArduPilot-native-SITL",
        "calibration_fixture": "apm-sitl-ground-truth-roundtrip.json",
    },
}


def selected_lines(value: str) -> tuple[str, ...]:
    if value == "all":
        return ("px4", "apm")
    if value not in LINE_SPECS:
        raise ValueError("line must be px4, apm or all")
    return (value,)


def make_mission(line: str, *, unauthorized: bool = False) -> MissionPlan:
    spec = LINE_SPECS[line]
    return MissionPlan(
        line_id=line,
        vehicle_id=spec["vehicle_id"],
        frame_calibration_id="parallel-sitl-locked-v1",
        cruise_speed_m_s=2.0,
        rehearsal_timeout_s=240.0,
        steps=(
            MissionStep(action=MissionAction.TAKEOFF, altitude_m=3.0),
            MissionStep(
                action=MissionAction.WAYPOINT,
                position_map_m=(4.0, spec["home_y_m"], 3.0),
                acceptance_radius_m=1.5,
            ),
            MissionStep(
                action=MissionAction.RTL if unauthorized else MissionAction.LAND
            ),
        ),
    )


def _tmux_session_exists(name: str) -> bool:
    return subprocess.run(
        ["tmux", "has-session", "-t", name],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    ).returncode == 0


def _run_checked(command: list[str], *, env: dict[str, str] | None = None) -> None:
    completed = subprocess.run(command, cwd=ROOT, env=env, check=False)
    if completed.returncode != 0:
        raise RuntimeError(
            f"command failed with exit code {completed.returncode}: "
            + shlex.join(command)
        )


def _wsl_windows_path(path: Path) -> str:
    completed = subprocess.run(
        ["wslpath", "-w", str(path.resolve())],
        check=True,
        capture_output=True,
        text=True,
    )
    return completed.stdout.strip()


def _start_missing_sitl(lines: tuple[str, ...], runtime: Path) -> None:
    launcher = ROOT / "deploy/parallel_domain/start_all.sh"
    environment = os.environ.copy()
    environment["PD_RUNTIME_DIR"] = str(runtime.resolve())
    for line in lines:
        real_session, virtual_session = LINE_SPECS[line]["sessions"]
        real_missing = not _tmux_session_exists(real_session)
        virtual_missing = not _tmux_session_exists(virtual_session)
        if not real_missing and not virtual_missing:
            continue
        world = "all" if real_missing and virtual_missing else (
            "real" if real_missing else "virtual"
        )
        _run_checked(
            ["bash", str(launcher), "--line", line, "--world", world],
            env=environment,
        )


def _powershell_instance_state(
    pid: int,
    rpc_port: int,
    settings_path: str,
) -> dict[str, Any]:
    script = (
        f"$p=Get-Process -Id {pid} -ErrorAction SilentlyContinue;"
        f"$l=Get-NetTCPConnection -LocalPort {rpc_port} -State Listen "
        "-ErrorAction SilentlyContinue;"
        "$owner=if($l){@($l)[0].OwningProcess}else{$null};"
        "$rpc=if($owner){Get-CimInstance Win32_Process -Filter "
        "\"ProcessId=$owner\" -ErrorAction SilentlyContinue}else{$null};"
        "[pscustomobject]@{process_alive=[bool]$p;rpc_listening=[bool]$l;"
        "rpc_process_id=$owner;rpc_command_line=$rpc.CommandLine}|"
        "ConvertTo-Json -Compress"
    )
    try:
        completed = subprocess.run(
            ["powershell.exe", "-NoProfile", "-Command", script],
            check=True,
            capture_output=True,
            text=True,
            timeout=15.0,
        )
        payload = json.loads(completed.stdout.strip())
        command_line = str(payload.get("rpc_command_line") or "")
        return {
            "process_alive": bool(payload.get("process_alive")),
            "rpc_listening": bool(payload.get("rpc_listening")),
            "rpc_process_id": payload.get("rpc_process_id"),
            "rpc_settings_match": settings_path.lower() in command_line.lower(),
        }
    except (OSError, subprocess.SubprocessError, json.JSONDecodeError):
        return {
            "process_alive": False,
            "rpc_listening": False,
            "rpc_process_id": None,
            "rpc_settings_match": False,
        }


def validate_airsim_manifest(
    payload: dict[str, Any],
    lines: tuple[str, ...],
    live_state: dict[str, dict[str, bool]],
) -> dict[str, Any]:
    instances = payload.get("instances")
    if not isinstance(instances, list):
        return {"passed": False, "error": "AirSim process manifest has no instances"}
    by_line = {
        str(item.get("line")): item for item in instances if isinstance(item, dict)
    }
    results: dict[str, dict[str, Any]] = {}
    settings_paths: list[str] = []
    for line in lines:
        spec = LINE_SPECS[line]
        item = by_line.get(line)
        state = live_state.get(line, {})
        settings_path = str(item.get("settings_path", "")) if item else ""
        settings_paths.append(settings_path)
        passed = bool(item) and all(
            (
                item.get("rpc_port") == spec["rpc_port"],
                bool(item.get("pid")),
                bool(item.get("settings_sha256")),
                settings_path.lower().endswith(f"\\{line}\\settings.json"),
                any("-settings=" in str(arg) for arg in item.get("arguments", [])),
                state.get("process_alive") is True,
                state.get("rpc_listening") is True,
                state.get("rpc_settings_match") is True,
            )
        )
        results[line] = {
            "passed": passed,
            "pid": item.get("pid") if item else None,
            "rpc_port": item.get("rpc_port") if item else None,
            "settings_path": settings_path or None,
            "settings_sha256": item.get("settings_sha256") if item else None,
            "process_alive": state.get("process_alive", False),
            "rpc_listening": state.get("rpc_listening", False),
            "rpc_process_id": state.get("rpc_process_id"),
            "rpc_settings_match": state.get("rpc_settings_match", False),
            "role": "仿真/孪生",
        }
    isolation_ok = len(settings_paths) == len(set(settings_paths)) and all(settings_paths)
    standard_untouched = (
        payload.get("standard_settings_sha256_before")
        == payload.get("standard_settings_sha256_after")
    )
    return {
        "passed": (
            all(item["passed"] for item in results.values())
            and isolation_ok
            and standard_untouched
        ),
        "instances": results,
        "settings_profiles_isolated": bool(isolation_ok),
        "standard_settings_untouched": standard_untouched,
    }


def read_airsim_status(runtime: Path, lines: tuple[str, ...]) -> dict[str, Any]:
    manifest_path = runtime / "airsim/live-processes.json"
    if not manifest_path.is_file():
        return {
            "passed": False,
            "manifest": str(manifest_path),
            "error": "AirSim live-process manifest is missing",
        }
    try:
        payload = json.loads(manifest_path.read_text(encoding="utf-8-sig"))
    except (OSError, json.JSONDecodeError) as exc:
        return {"passed": False, "manifest": str(manifest_path), "error": str(exc)}
    by_line = {
        str(item.get("line")): item
        for item in payload.get("instances", [])
        if isinstance(item, dict)
    }
    live_state = {
        line: _powershell_instance_state(
            int(by_line.get(line, {}).get("pid", -1)),
            int(LINE_SPECS[line]["rpc_port"]),
            str(by_line.get(line, {}).get("settings_path", "")),
        )
        for line in lines
    }
    result = validate_airsim_manifest(payload, lines, live_state)
    result["manifest"] = str(manifest_path.resolve())
    return result


def _launch_missing_airsim(lines: tuple[str, ...], runtime: Path) -> None:
    status = read_airsim_status(runtime, lines)
    missing = tuple(
        line
        for line in lines
        if not status.get("instances", {}).get(line, {}).get("passed", False)
    )
    if not missing:
        return
    if not shutil_which("powershell.exe"):
        raise RuntimeError("powershell.exe is required to start actual AirSim UE worlds")
    line_argument = "all" if set(missing) == {"px4", "apm"} else missing[0]
    script = _wsl_windows_path(ROOT / "deploy/parallel_domain/start_all.ps1")
    _run_checked(
        [
            "powershell.exe",
            "-NoProfile",
            "-ExecutionPolicy",
            "Bypass",
            "-File",
            script,
            "-Line",
            line_argument,
            "-SkipSITL",
            "-RuntimeDirectoryWsl",
            str(runtime.resolve()),
        ]
    )


def shutil_which(command: str) -> str | None:
    # Kept local so tests can import this tool without extra platform packages.
    from shutil import which

    return which(command)


def ensure_live_stack(
    lines: tuple[str, ...],
    runtime: Path,
    *,
    start_stack: bool,
    timeout_s: float,
) -> dict[str, Any]:
    if start_stack:
        _start_missing_sitl(lines, runtime)
        _launch_missing_airsim(lines, runtime)
    deadline = time.monotonic() + timeout_s
    latest: dict[str, Any] = {}
    while time.monotonic() < deadline:
        sessions = {
            line: {
                name: _tmux_session_exists(name)
                for name in LINE_SPECS[line]["sessions"]
            }
            for line in lines
        }
        airsim = read_airsim_status(runtime, lines)
        latest = {"sitl_sessions": sessions, "airsim": airsim}
        if all(all(item.values()) for item in sessions.values()) and airsim.get("passed"):
            return {**latest, "passed": True}
        time.sleep(1.0)
    raise RuntimeError("live SITL/AirSim stack did not become ready: " + json.dumps(latest))


def _read_events(path: Path) -> list[dict[str, Any]]:
    if not path.is_file():
        return []
    events: list[dict[str, Any]] = []
    for raw in path.read_text(encoding="utf-8").splitlines():
        if not raw.strip():
            continue
        try:
            value = json.loads(raw)
        except json.JSONDecodeError:
            continue
        if isinstance(value, dict):
            events.append(value)
    return events


async def _wait_event(
    audit_path: Path,
    event_type: str,
    mission_id: UUID,
    timeout_s: float,
    predicate: Callable[[dict[str, Any]], bool] | None = None,
) -> tuple[int, dict[str, Any]]:
    deadline = asyncio.get_running_loop().time() + timeout_s
    while asyncio.get_running_loop().time() < deadline:
        for index, event in enumerate(_read_events(audit_path)):
            payload = event.get("payload")
            if (
                event.get("event_type") == event_type
                and isinstance(payload, dict)
                and payload.get("mission_id") == str(mission_id)
                and (predicate is None or predicate(event))
            ):
                return index, event
        await asyncio.sleep(0.25)
    raise TimeoutError(f"timed out waiting for {event_type}: {mission_id}")


async def _wait_bridge_fresh(
    service: ParallelDomainService,
    lines: tuple[str, ...],
    timeout_s: float,
) -> dict[str, Any]:
    deadline = asyncio.get_running_loop().time() + timeout_s
    latest: dict[str, Any] = {}
    while asyncio.get_running_loop().time() < deadline:
        latest = service.core.status()
        ready = True
        for line in lines:
            state = latest.get("lines", {}).get(line, {})
            state_set = state.get("state_set", {})
            ready = ready and all(
                (
                    state.get("real_link", {}).get("connected") is True,
                    state.get("virtual_link", {}).get("connected") is True,
                    state.get("mirror", {}).get("state") == "fresh",
                    isinstance(state_set.get("observed"), dict),
                    isinstance(state_set.get("predicted"), dict),
                )
            )
        if ready:
            return latest
        await asyncio.sleep(0.5)
    raise RuntimeError("bridge telemetry did not become fresh: " + json.dumps(latest))


def _payload(event: dict[str, Any]) -> dict[str, Any]:
    value = event.get("payload")
    return value if isinstance(value, dict) else {}


async def _approval_digest(
    line: str,
    mission: MissionPlan,
    store: BridgeStore,
    *,
    confirm_sitl: bool,
) -> str:
    request = store.load_approval_request(mission.mission_id)
    digest = confirmation_digest_for(request)
    print(
        json.dumps(
            {
                "gate": "人工确认门禁",
                "line": line,
                "mission_id": str(mission.mission_id),
                "mission_hash": request.mission_hash,
                "predicted_evidence_hash": request.predicted_evidence_hash,
                "confirmation_digest": digest,
                "target": LINE_SPECS[line]["real_label"],
            },
            ensure_ascii=False,
            indent=2,
        )
    )
    if confirm_sitl:
        return digest
    entered = await asyncio.to_thread(
        input,
        f"输入上面的 confirmation_digest 以批准 {line.upper()} SITL 实机替身执行：",
    )
    if entered.strip() != digest:
        raise RuntimeError(f"operator confirmation rejected for {line}")
    return digest


def _load_mission_evidence(root: Path, mission_id: UUID) -> dict[TrajectoryRole, TrajectoryEvidence]:
    found: dict[TrajectoryRole, TrajectoryEvidence] = {}
    for path in sorted((root / "evidence").glob("*.json")):
        evidence = load_evidence_file(path)
        if evidence.mission_id == mission_id:
            found[evidence.role] = evidence
    if set(found) != set(TrajectoryRole):
        missing = sorted(item.value for item in set(TrajectoryRole) - set(found))
        raise RuntimeError("mission evidence is incomplete: " + ", ".join(missing))
    return found


def _git_commit() -> str | None:
    completed = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    return completed.stdout.strip() if completed.returncode == 0 else None


def _package_mission(
    state_directory: Path,
    output_directory: Path,
    line: str,
    mission: MissionPlan,
    software_sha256: str,
) -> dict[str, Any]:
    calibration_directory = output_directory / "calibration" / line
    calibration_directory.mkdir(parents=True, exist_ok=False)
    draft = load_georeference(
        ROOT / "configs/parallel_domain/sitl-georeference-draft.yaml"
    )
    dataset_path = (
        ROOT
        / "configs/parallel_domain/calibration"
        / LINE_SPECS[line]["calibration_fixture"]
    )
    dataset = load_calibration_dataset(dataset_path)
    frozen, calibration_report, receipt = freeze_georeference(draft, dataset)
    active = load_georeference(ROOT / "configs/parallel_domain/sitl-georeference.yaml")
    if frozen.config_hash != active.config_hash:
        raise RuntimeError("frozen calibration does not match active bridge calibration")
    georeference_path = calibration_directory / "georeference.yaml"
    report_path = calibration_directory / "report.json"
    receipt_path = calibration_directory / "receipt.json"
    GeoReferenceStore(georeference_path).save(frozen)
    write_calibration_report(report_path, calibration_report)
    write_freeze_receipt(receipt_path, receipt)
    package_path = output_directory / f"{line}-live-evidence.zip"
    manifest = build_live_evidence_package(
        state_directory=state_directory,
        mission_id=mission.mission_id,
        output=package_path,
        georeference=frozen,
        versions={
            "aeromind_apm_lite": "working-tree",
            "bridge": "parallel-domain-v1",
            "flight_controller": LINE_SPECS[line]["flight_controller"],
            "simulator": LINE_SPECS[line]["real_simulator"] + "+AirSim-1.8.1",
            "world": "parallel-sitl",
        },
        software_sha256=software_sha256,
        software_hash_scope=str(ROOT.resolve()),
        software_commit=_git_commit(),
        random_seed=20260817,
        calibration_dataset=dataset,
        calibration_report=calibration_report,
        calibration_receipt=receipt,
    )
    verified = verify_live_evidence_package(package_path)
    return {
        "path": str(package_path.resolve()),
        "verified": verified.content_sha256 == manifest.content_sha256,
        "content_sha256": verified.content_sha256,
        "preview_only": verified.preview_only,
        "trajectory_acceptance_passed": verified.acceptance_passed,
        "calibration_sha256": verified.frame_calibration_sha256,
    }


def comparison_report_pairs(package_path: Path) -> tuple[tuple[str, str], ...]:
    """Read the packaged metrics, rather than inferring them from evidence hashes."""
    with zipfile.ZipFile(package_path) as archive:
        manifest = json.loads(archive.read("manifest.json"))
        artifacts = manifest.get("artifacts", [])
        comparison = next(
            (
                item
                for item in artifacts
                if isinstance(item, dict) and item.get("name") == "comparison-report"
            ),
            None,
        )
        if comparison is None:
            raise RuntimeError("live evidence package has no comparison-report artifact")
        envelope = json.loads(archive.read(comparison["relative_path"]))
    report = TrajectoryComparisonReport.model_validate(envelope["report"])
    return tuple(
        (item.reference_role.value, item.compared_role.value)
        for item in report.comparisons
    )


def _interpolate(samples: tuple[Any, ...], time_ms: int) -> tuple[float, float, float]:
    if time_ms <= samples[0].time_from_start_ms:
        return samples[0].position_map_m
    if time_ms >= samples[-1].time_from_start_ms:
        return samples[-1].position_map_m
    for left, right in zip(samples, samples[1:]):
        if left.time_from_start_ms <= time_ms <= right.time_from_start_ms:
            span = right.time_from_start_ms - left.time_from_start_ms
            fraction = (time_ms - left.time_from_start_ms) / span
            return tuple(
                left.position_map_m[index]
                + fraction * (right.position_map_m[index] - left.position_map_m[index])
                for index in range(3)
            )
    return samples[-1].position_map_m


def trajectory_error_curve(
    reference: TrajectoryEvidence,
    compared: TrajectoryEvidence,
) -> list[dict[str, float]]:
    start = max(reference.samples[0].time_from_start_ms, compared.samples[0].time_from_start_ms)
    end = min(reference.samples[-1].time_from_start_ms, compared.samples[-1].time_from_start_ms)
    result: list[dict[str, float]] = []
    for sample in reference.samples:
        time_ms = sample.time_from_start_ms
        if not start <= time_ms <= end:
            continue
        other = _interpolate(compared.samples, time_ms)
        delta = tuple(sample.position_map_m[index] - other[index] for index in range(3))
        result.append(
            {
                "t_s": time_ms / 1000.0,
                "horizontal_m": math.hypot(delta[0], delta[1]),
                "vertical_m": abs(delta[2]),
                "three_d_m": math.sqrt(sum(value * value for value in delta)),
            }
        )
    return result


def build_report_payload(
    state_directory: Path,
    line_missions: dict[str, MissionPlan],
    line_results: dict[str, dict[str, Any]],
) -> dict[str, Any]:
    lines: dict[str, Any] = {}
    for line, mission in line_missions.items():
        evidence = _load_mission_evidence(state_directory, mission.mission_id)
        trajectories = {
            role.value: [
                {
                    "t_s": sample.time_from_start_ms / 1000.0,
                    "x": sample.position_map_m[0],
                    "y": sample.position_map_m[1],
                    "z": sample.position_map_m[2],
                }
                for sample in evidence[role].samples
            ]
            for role in TrajectoryRole
        }
        pairs = (
            (TrajectoryRole.PLANNED, TrajectoryRole.PREDICTED),
            (TrajectoryRole.PLANNED, TrajectoryRole.OBSERVED),
            (TrajectoryRole.PREDICTED, TrajectoryRole.OBSERVED),
        )
        errors = {
            f"{left.value}-{right.value}": trajectory_error_curve(
                evidence[left], evidence[right]
            )
            for left, right in pairs
        }
        lines[line] = {
            "mission_id": str(mission.mission_id),
            "vehicle_id": mission.vehicle_id,
            "real_label": LINE_SPECS[line]["real_label"],
            "twin_label": LINE_SPECS[line]["twin_label"],
            "trajectories": trajectories,
            "errors": errors,
            "evidence_package": line_results[line]["evidence_package"],
            "assertions": line_results[line]["assertions"],
        }
    return {
        "title": "翼策·平行域 live 验收",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "scope": "SITL 实机替身 + AirSim 孪生；不是外场真机",
        "lines": lines,
    }


def write_same_screen_report(path: Path, payload: dict[str, Any]) -> None:
    encoded = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).replace(
        "</", "<\\/"
    )
    html = _REPORT_HTML.replace("__REPORT_DATA__", encoded)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(html, encoding="utf-8")


async def _run_line(
    config: BridgeConfig,
    service: ParallelDomainService,
    line: str,
    output_directory: Path,
    *,
    confirm_sitl: bool,
    operator: str,
    timeout_s: float,
    software_sha256: str,
) -> tuple[MissionPlan, dict[str, Any]]:
    store = BridgeStore(config.state_directory)
    audit_path = config.state_directory / "audit/events.jsonl"
    denied = make_mission(line, unauthorized=True)
    write_plan_to_spool(config, denied)
    denied_index, denied_event = await _wait_event(
        audit_path,
        "rehearsal_rejected",
        denied.mission_id,
        timeout_s,
        predicate=lambda event: _payload(event).get("command") == "rtl",
    )
    denied_payload = _payload(denied_event)
    denied_dispatched = any(
        event.get("event_type") == "real_mission_dispatched"
        and _payload(event).get("mission_id") == str(denied.mission_id)
        for event in _read_events(audit_path)
    )

    initial_status = service.core.status()["lines"][line]
    initial_observed_sequence = int(initial_status["mirror"]["observed_sequence"])
    mission = make_mission(line)
    write_plan_to_spool(config, mission)
    approval_index, _approval_event = await _wait_event(
        audit_path, "approval_requested", mission.mission_id, timeout_s
    )
    digest = await _approval_digest(
        line,
        mission,
        store,
        confirm_sitl=confirm_sitl,
    )
    write_approval_to_spool(
        config,
        ApprovalCommand(
            mission_id=mission.mission_id,
            mission_hash=mission.mission_hash,
            operator_id=operator,
            confirmation_digest=digest,
        ),
    )
    event_names = (
        "operator_confirmed",
        "real_dispatch_authorization_claimed",
        "real_dispatch_authorization_consumed",
        "real_mission_dispatched",
        "observed_trajectory_completed",
        "predicted_observed_compared",
    )
    events: dict[str, tuple[int, dict[str, Any]]] = {}
    for event_type in event_names:
        events[event_type] = await _wait_event(
            audit_path, event_type, mission.mission_id, timeout_s
        )
    event_order = [approval_index] + [events[name][0] for name in event_names]
    final_status = service.core.status()["lines"][line]
    observed_sequence_advanced = (
        int(final_status["mirror"]["observed_sequence"]) > initial_observed_sequence
    )
    state_set = final_status["state_set"]
    real_payload = _payload(events["real_mission_dispatched"][1])
    evidence_package = _package_mission(
        config.state_directory,
        output_directory,
        line,
        mission,
        software_sha256,
    )
    packaged_pairs = comparison_report_pairs(Path(evidence_package["path"]))
    assertions = {
        "airsim_prediction_present": state_set.get("predicted", {}).get("source")
        == "predicted",
        "real_observed_mirrored": state_set.get("observed", {}).get("source")
        == "observed"
        and observed_sequence_advanced,
        "unauthorized_rtl_rejected": (
            denied_payload.get("command") == "rtl"
            and "capability_whitelist" in denied_payload.get("reason", "")
            and not denied_dispatched
        ),
        "human_confirmation_recorded": "operator_confirmed" in events,
        "event_chain_ordered": event_order == sorted(event_order),
        "task_level_execution_only": real_payload.get("execution_semantic")
        in {"px4_offboard_task", "ardupilot_guided_task"},
        "live_evidence_verified": evidence_package["verified"] is True,
        "three_pair_error_report": packaged_pairs
        == (
            ("planned", "predicted"),
            ("planned", "observed"),
            ("predicted", "observed"),
        ),
    }
    return mission, {
        "passed": all(assertions.values()),
        "line": line,
        "vehicle_id": mission.vehicle_id,
        "mission_id": str(mission.mission_id),
        "unauthorized_mission_id": str(denied.mission_id),
        "unauthorized_audit_index": denied_index,
        "execution_semantic": real_payload.get("execution_semantic"),
        "real_role": LINE_SPECS[line]["real_label"],
        "twin_role": LINE_SPECS[line]["twin_label"],
        "assertions": assertions,
        "evidence_package": evidence_package,
    }


def _stop_conflicting_bridges() -> dict[str, bool]:
    state = {
        name: _tmux_session_exists(name) for name in ("pd-control", "pd-live-faults")
    }
    for name, running in state.items():
        if running:
            _run_checked(["tmux", "kill-session", "-t", name])
    return state


def _restore_normal_bridge(previous: dict[str, bool]) -> None:
    if not previous.get("pd-control") or _tmux_session_exists("pd-control"):
        return
    command = (
        f"exec {shlex.quote(str(ROOT / 'deploy/sim/start-parallel-control.sh'))} "
        f"{shlex.quote(str(ROOT / 'configs/parallel_domain/bridge-control.yaml'))}"
    )
    _run_checked(
        [
            "tmux",
            "new-session",
            "-d",
            "-s",
            "pd-control",
            "-c",
            str(ROOT),
            command,
        ]
    )


async def run_demo(args: argparse.Namespace, stack_status: dict[str, Any]) -> dict[str, Any]:
    lines = selected_lines(args.line)
    base_config = load_bridge_config(args.config)
    chosen = tuple(line for line in base_config.lines if line.line_id in lines)
    if len(chosen) != len(lines):
        raise RuntimeError("selected line is missing from bridge config")
    state_directory = args.output / "state"
    config = base_config.model_copy(
        update={"state_directory": state_directory, "lines": chosen}
    )
    service = ParallelDomainService(config)
    task = asyncio.create_task(service.run())
    missions: dict[str, MissionPlan] = {}
    line_results: dict[str, dict[str, Any]] = {}
    try:
        await _wait_bridge_fresh(service, lines, args.telemetry_timeout_s)
        software_sha256 = software_tree_sha256(ROOT)
        for line in lines:
            mission, result = await _run_line(
                config,
                service,
                line,
                args.output,
                confirm_sitl=args.confirm_sitl,
                operator=args.operator,
                timeout_s=args.mission_timeout_s,
                software_sha256=software_sha256,
            )
            missions[line] = mission
            line_results[line] = result
    finally:
        service.stop()
        try:
            await asyncio.wait_for(task, timeout=10.0)
        except asyncio.TimeoutError:
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)

    report_path = args.output / "same-screen-report.html"
    report_payload = build_report_payload(state_directory, missions, line_results)
    write_same_screen_report(report_path, report_payload)
    result = {
        "schema_version": "1.0",
        "run_id": str(uuid4()),
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "selected_lines": list(lines),
        "stack": stack_status,
        "lines": line_results,
        "same_screen_report": str(report_path.resolve()),
        "state_directory": str(state_directory.resolve()),
        "passed": stack_status.get("passed") is True
        and all(item["passed"] for item in line_results.values()),
        "scope": {
            "real_substitute": "Gazebo/ArduPilot native SITL (simulation/HIL)",
            "digital_twin": "AirSim UE + SITL (simulation/twin)",
            "outdoor_real_aircraft": "not connected; user integration is future work",
        },
    }
    (args.output / "acceptance.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return result


def _default_output() -> Path:
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return ROOT / ".runtime/parallel-domain-e2e" / f"{stamp}-{uuid4().hex[:8]}"


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--line", choices=("px4", "apm", "all"), default="all")
    parser.add_argument(
        "--config",
        type=Path,
        default=ROOT / "configs/parallel_domain/bridge-control.yaml",
    )
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument(
        "--sitl-runtime",
        type=Path,
        default=ROOT / ".runtime/parallel-domain-sitl",
    )
    parser.add_argument("--no-start-stack", action="store_true")
    parser.add_argument("--startup-timeout-s", type=float, default=240.0)
    parser.add_argument("--telemetry-timeout-s", type=float, default=60.0)
    parser.add_argument("--mission-timeout-s", type=float, default=300.0)
    parser.add_argument("--operator", default="parallel-domain-operator")
    parser.add_argument(
        "--confirm-sitl",
        action="store_true",
        help="explicitly approve generated digests for simulation/HIL substitutes",
    )
    args = parser.parse_args(argv)
    args.output = (args.output or _default_output()).resolve()
    if args.output.exists():
        raise SystemExit(f"output directory already exists: {args.output}")
    args.output.mkdir(parents=True)
    lines = selected_lines(args.line)
    previous_bridges: dict[str, bool] = {}
    try:
        stack_status = ensure_live_stack(
            lines,
            args.sitl_runtime,
            start_stack=not args.no_start_stack,
            timeout_s=args.startup_timeout_s,
        )
        previous_bridges = _stop_conflicting_bridges()
        result = asyncio.run(run_demo(args, stack_status))
    except Exception as exc:
        result = {
            "schema_version": "1.0",
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
            "selected_lines": list(lines),
            "passed": False,
            "failure": {"type": type(exc).__name__, "message": str(exc)},
            "scope": {
                "real_substitute": "Gazebo/ArduPilot native SITL (simulation/HIL)",
                "digital_twin": "AirSim UE + SITL (simulation/twin)",
                "outdoor_real_aircraft": "not connected; user integration is future work",
            },
        }
        (args.output / "acceptance.json").write_text(
            json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
    finally:
        if previous_bridges:
            _restore_normal_bridge(previous_bridges)
    print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))
    return 0 if result["passed"] else 1


_REPORT_HTML = r'''<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>翼策·平行域 live 验收</title>
  <style>
    :root { color-scheme: light; font-family: Inter, "Microsoft YaHei", sans-serif; }
    * { box-sizing: border-box; }
    body { margin: 0; color: #17212b; background: #f4f6f7; }
    header { background: #17212b; color: white; padding: 22px max(24px, calc((100% - 1440px)/2)); }
    h1 { margin: 0; font-size: 28px; letter-spacing: 0; }
    header p { margin: 8px 0 0; color: #d7e0e5; }
    .notice { background: #fff3cd; color: #5d4500; padding: 12px max(24px, calc((100% - 1440px)/2)); border-bottom: 1px solid #e6cf82; }
    main { max-width: 1440px; margin: 0 auto; padding: 20px 24px 40px; }
    section { background: white; border: 1px solid #d8dee2; margin-bottom: 18px; }
    .section-head { padding: 14px 16px; border-bottom: 1px solid #d8dee2; display: flex; gap: 14px; align-items: baseline; flex-wrap: wrap; }
    h2 { margin: 0; font-size: 18px; letter-spacing: 0; }
    .meta { color: #56636d; font-size: 13px; }
    .legend { display: flex; flex-wrap: wrap; gap: 14px; padding: 12px 16px 0; font-size: 13px; }
    .legend span::before { content: ""; display: inline-block; width: 18px; height: 3px; margin: 0 6px 3px 0; background: var(--color); }
    .plots { display: grid; grid-template-columns: 1fr 1fr; gap: 0; }
    .plot { padding: 12px 16px 16px; min-width: 0; }
    .plot + .plot { border-left: 1px solid #d8dee2; }
    .plot h3 { margin: 0 0 8px; font-size: 14px; letter-spacing: 0; }
    canvas { display: block; width: 100%; height: 340px; border: 1px solid #d8dee2; background: #fbfcfc; }
    table { width: 100%; border-collapse: collapse; font-size: 13px; }
    th, td { padding: 8px 10px; border-top: 1px solid #e4e8ea; text-align: left; overflow-wrap: anywhere; }
    th { color: #56636d; font-weight: 600; }
    .ok { color: #137333; font-weight: 700; }
    .fail { color: #b3261e; font-weight: 700; }
    @media (max-width: 850px) { .plots { grid-template-columns: 1fr; } .plot + .plot { border-left: 0; border-top: 1px solid #d8dee2; } }
  </style>
</head>
<body>
  <header><h1>翼策·平行域 live 验收</h1><p id="generated"></p></header>
  <div class="notice">边界声明：实机替身是 Gazebo / ArduPilot 原生 SITL（仿真/HIL）；AirSim 是仿真/孪生。此报告不代表外场真机已接入。</div>
  <main id="content"></main>
  <script>
    const report = __REPORT_DATA__;
    document.getElementById('generated').textContent = `${report.generated_at_utc} · ${report.scope}`;
    const colors = {planned:'#52606d', predicted:'#c56a00', observed:'#087f5b', 'planned-predicted':'#2563a6', 'planned-observed':'#b3261e', 'predicted-observed':'#7a4fa3'};
    function extent(series, fields) {
      const values = [];
      series.forEach(items => items.forEach(item => fields.forEach(field => values.push(item[field]))));
      let lo = Math.min(...values), hi = Math.max(...values); if (lo === hi) { lo -= 1; hi += 1; }
      const pad = (hi-lo)*0.08; return [lo-pad, hi+pad];
    }
    function draw(canvas, series, xField, yField, xLabel, yLabel) {
      const ratio = window.devicePixelRatio || 1, box = canvas.getBoundingClientRect();
      canvas.width = Math.max(600, box.width*ratio); canvas.height = 340*ratio;
      const ctx = canvas.getContext('2d'); ctx.scale(ratio,ratio); const w=canvas.width/ratio,h=canvas.height/ratio;
      const pad={l:56,r:18,t:18,b:42}; const [xmin,xmax]=extent(Object.values(series),[xField]); const [ymin,ymax]=extent(Object.values(series),[yField]);
      const sx=x=>pad.l+(x-xmin)/(xmax-xmin)*(w-pad.l-pad.r), sy=y=>h-pad.b-(y-ymin)/(ymax-ymin)*(h-pad.t-pad.b);
      ctx.strokeStyle='#d8dee2'; ctx.lineWidth=1; for(let i=0;i<=5;i++){const x=pad.l+i*(w-pad.l-pad.r)/5,y=pad.t+i*(h-pad.t-pad.b)/5;ctx.beginPath();ctx.moveTo(x,pad.t);ctx.lineTo(x,h-pad.b);ctx.stroke();ctx.beginPath();ctx.moveTo(pad.l,y);ctx.lineTo(w-pad.r,y);ctx.stroke();}
      ctx.fillStyle='#56636d'; ctx.font='12px sans-serif'; ctx.fillText(xLabel,w/2-25,h-12); ctx.save();ctx.translate(14,h/2+30);ctx.rotate(-Math.PI/2);ctx.fillText(yLabel,0,0);ctx.restore();
      Object.entries(series).forEach(([name,items])=>{if(!items.length)return;ctx.strokeStyle=colors[name];ctx.lineWidth=2;ctx.beginPath();items.forEach((item,i)=>{const x=sx(item[xField]),y=sy(item[yField]);if(i===0)ctx.moveTo(x,y);else ctx.lineTo(x,y);});ctx.stroke();});
    }
    Object.entries(report.lines).forEach(([line,data])=>{
      const section=document.createElement('section');
      section.innerHTML=`<div class="section-head"><h2>${line.toUpperCase()} · vehicle ${data.vehicle_id}</h2><span class="meta">mission ${data.mission_id}</span></div><div class="legend"><span style="--color:${colors.planned}">planned 任务目标</span><span style="--color:${colors.predicted}">${data.twin_label}</span><span style="--color:${colors.observed}">${data.real_label}</span></div><div class="plots"><div class="plot"><h3>map ENU 轨迹同屏对照</h3><canvas class="trajectory"></canvas></div><div class="plot"><h3>三维误差曲线（wall-time 相对对齐）</h3><canvas class="errors"></canvas></div></div><table><tr><th>证据包</th><td>${data.evidence_package.path}</td></tr><tr><th>content SHA-256</th><td>${data.evidence_package.content_sha256}</td></tr><tr><th>轨迹指标结论</th><td class="${data.evidence_package.trajectory_acceptance_passed?'ok':'fail'}">${String(data.evidence_package.trajectory_acceptance_passed)}</td></tr><tr><th>流程断言</th><td class="${Object.values(data.assertions).every(Boolean)?'ok':'fail'}">${Object.entries(data.assertions).map(([k,v])=>`${k}=${v}`).join(' · ')}</td></tr></table>`;
      document.getElementById('content').appendChild(section);
      draw(section.querySelector('.trajectory'),data.trajectories,'x','y','map X / m','map Y / m');
      draw(section.querySelector('.errors'),data.errors,'t_s','three_d_m','time / s','3D error / m');
    });
  </script>
</body>
</html>
'''


if __name__ == "__main__":
    raise SystemExit(main())

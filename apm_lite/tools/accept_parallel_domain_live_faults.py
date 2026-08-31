#!/usr/bin/env python3
"""Reproduce dual-line live gate and safety-recovery faults against SITL."""

from __future__ import annotations

import argparse
import asyncio
import json
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any, Callable
from uuid import UUID, uuid4

from aeromind_apm_lite.ground.parallel_domain.config import (
    BridgeConfig,
    load_bridge_config,
)
from aeromind_apm_lite.ground.parallel_domain.core import confirmation_digest_for
from aeromind_apm_lite.ground.parallel_domain.models import (
    ApprovalCommand,
    LiveFaultCommand,
    LiveFaultType,
    MissionAction,
    MissionPlan,
    MissionStep,
    RealDispatchRequest,
    utc_now,
)
from aeromind_apm_lite.ground.parallel_domain.service import (
    write_approval_to_spool,
    write_dispatch_to_spool,
    write_fault_to_spool,
    write_plan_to_spool,
)
from aeromind_apm_lite.ground.parallel_domain.storage import BridgeStore


Event = dict[str, Any]
Predicate = Callable[[Event], bool]


def _read_events(path: Path) -> list[Event]:
    if not path.is_file():
        return []
    events = []
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
    *,
    event_type: str,
    mission_id: UUID,
    timeout_s: float,
    predicate: Predicate | None = None,
) -> tuple[int, Event]:
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
    raise TimeoutError(f"timed out waiting for {event_type} on mission {mission_id}")


def _plan(
    line_id: str,
    vehicle_id: int,
    *,
    terminal: MissionAction = MissionAction.LAND,
    waypoint_x_m: float = 4.0,
) -> MissionPlan:
    home_y = 0.0 if line_id == "px4" else 3.0
    return MissionPlan(
        line_id=line_id,
        vehicle_id=vehicle_id,
        frame_calibration_id="parallel-sitl-locked-v1",
        cruise_speed_m_s=2.0,
        rehearsal_timeout_s=240.0,
        steps=(
            MissionStep(action=MissionAction.TAKEOFF, altitude_m=3.0),
            MissionStep(
                action=MissionAction.WAYPOINT,
                position_map_m=(waypoint_x_m, home_y, 3.0),
                acceptance_radius_m=1.5,
            ),
            MissionStep(action=terminal),
        ),
    )


def _approval(store: BridgeStore, mission: MissionPlan, operator: str) -> ApprovalCommand:
    request = store.load_approval_request(mission.mission_id)
    return ApprovalCommand(
        mission_id=mission.mission_id,
        mission_hash=mission.mission_hash,
        operator_id=operator,
        confirmation_digest=confirmation_digest_for(request),
    )


def _required_structured_fields(event: Event) -> bool:
    payload = event.get("payload")
    return isinstance(payload, dict) and {
        "reason",
        "wall_time",
        "vehicle_id",
        "line_id",
        "command",
    } <= set(payload)


def _event_payload(event: Event) -> dict[str, Any]:
    payload = event.get("payload")
    return payload if isinstance(payload, dict) else {}


async def run(config: BridgeConfig, output: Path, events_output: Path) -> dict[str, Any]:
    if not config.live_fault_injection_enabled:
        raise ValueError("acceptance requires live_fault_injection_enabled=true")
    if any(line.policy.approval_ttl_s != 30 for line in config.lines):
        raise ValueError("acceptance profile must use the strict 30 second approval TTL")
    if any(not line.real.force_arm_for_sitl for line in config.lines):
        raise ValueError("acceptance profile must explicitly target force-arm SITL endpoints")

    store = BridgeStore(config.state_directory)
    audit_path = config.state_directory / "audit/events.jsonl"
    run_id = uuid4()
    lines = {line.line_id: line for line in config.lines}
    expected = {"px4": 1, "apm": 2}
    if {line_id: lines[line_id].vehicle_id for line_id in expected} != expected:
        raise ValueError("acceptance requires px4/1 and apm/2 lines")

    selected: list[Event] = []
    results: dict[str, dict[str, Any]] = {line_id: {} for line_id in expected}

    denied = {
        line_id: _plan(
            line_id,
            vehicle_id,
            terminal=MissionAction.RTL,
        )
        for line_id, vehicle_id in expected.items()
    }
    for mission in denied.values():
        write_plan_to_spool(config, mission)
    for line_id, mission in denied.items():
        index, event = await _wait_event(
            audit_path,
            event_type="rehearsal_rejected",
            mission_id=mission.mission_id,
            timeout_s=30.0,
            predicate=lambda item: _event_payload(item).get("command") == "rtl",
        )
        selected.append(event)
        results[line_id]["unauthorized"] = {
            "mission_id": str(mission.mission_id),
            "audit_index": index,
            "structured": _required_structured_fields(event),
            "reason": _event_payload(event).get("reason"),
            "command": _event_payload(event).get("command"),
        }

    expired = {
        line_id: _plan(line_id, vehicle_id)
        for line_id, vehicle_id in expected.items()
    }
    for mission in expired.values():
        write_plan_to_spool(config, mission)
    expiry_times = []
    for mission in expired.values():
        _index, event = await _wait_event(
            audit_path,
            event_type="approval_requested",
            mission_id=mission.mission_id,
            timeout_s=240.0,
        )
        selected.append(event)
        expiry_times.append(store.load_approval_request(mission.mission_id).expires_at_utc)
    latest_expiry = max(expiry_times)
    delay = max(0.0, (latest_expiry - utc_now()).total_seconds()) + 1.0
    await asyncio.sleep(delay)
    for line_id, mission in expired.items():
        write_approval_to_spool(
            config,
            _approval(store, mission, f"live-expiry-{run_id}-{line_id}"),
        )
    for line_id, mission in expired.items():
        index, event = await _wait_event(
            audit_path,
            event_type="approval_rejected",
            mission_id=mission.mission_id,
            timeout_s=30.0,
            predicate=lambda item: (
                _event_payload(item).get("reason") == "approval_ttl_expired"
            ),
        )
        selected.append(event)
        dispatched = any(
            item.get("event_type") == "real_mission_dispatched"
            and _event_payload(item).get("mission_id") == str(mission.mission_id)
            for item in _read_events(audit_path)
        )
        results[line_id]["expired"] = {
            "mission_id": str(mission.mission_id),
            "audit_index": index,
            "structured": _required_structured_fields(event),
            "reason": _event_payload(event).get("reason"),
            "real_dispatch_absent": not dispatched,
        }

    recovery = {
        line_id: _plan(line_id, vehicle_id, waypoint_x_m=20.0)
        for line_id, vehicle_id in expected.items()
    }
    for mission in recovery.values():
        write_plan_to_spool(config, mission)
    for mission in recovery.values():
        _index, event = await _wait_event(
            audit_path,
            event_type="approval_requested",
            mission_id=mission.mission_id,
            timeout_s=240.0,
        )
        selected.append(event)

    for line_id, mission in recovery.items():
        write_dispatch_to_spool(
            config,
            RealDispatchRequest(
                mission_id=mission.mission_id,
                line_id=line_id,
                vehicle_id=mission.vehicle_id,
            ),
        )
    for mission in recovery.values():
        _index, event = await _wait_event(
            audit_path,
            event_type="real_dispatch_rejected",
            mission_id=mission.mission_id,
            timeout_s=30.0,
            predicate=lambda item: (
                _event_payload(item).get("reason")
                == "operator_confirmation_missing"
            ),
        )
        selected.append(event)

    faults: dict[str, LiveFaultCommand] = {}
    for line_id, mission in recovery.items():
        write_approval_to_spool(
            config,
            _approval(store, mission, f"live-recovery-{run_id}-{line_id}"),
        )
        requested_at = utc_now()
        fault = LiveFaultCommand(
            mission_id=mission.mission_id,
            line_id=line_id,
            vehicle_id=mission.vehicle_id,
            fault=LiveFaultType.HEARTBEAT_GPS_LOSS,
            requested_at_utc=requested_at,
            expires_at_utc=requested_at + timedelta(seconds=30),
        )
        faults[line_id] = fault
        write_fault_to_spool(config, fault)

    for line_id, mission in recovery.items():
        event_names = (
            "operator_confirmed",
            "real_mission_dispatched",
            "live_fault_injected",
            "safety_hold",
            "recovery_land",
            "observed_trajectory_completed",
        )
        found: dict[str, tuple[int, Event]] = {}
        for event_type in event_names:
            found[event_type] = await _wait_event(
                audit_path,
                event_type=event_type,
                mission_id=mission.mission_id,
                timeout_s=180.0,
            )
            selected.append(found[event_type][1])
        unconfirmed = await _wait_event(
            audit_path,
            event_type="real_dispatch_rejected",
            mission_id=mission.mission_id,
            timeout_s=5.0,
            predicate=lambda item: (
                _event_payload(item).get("reason")
                == "operator_confirmation_missing"
            ),
        )
        selected.append(unconfirmed[1])
        order = [unconfirmed[0]] + [found[name][0] for name in event_names]
        hold_payload = _event_payload(found["safety_hold"][1])
        land_payload = _event_payload(found["recovery_land"][1])
        results[line_id]["confirmed_recovery"] = {
            "mission_id": str(mission.mission_id),
            "fault_id": str(faults[line_id].fault_id),
            "event_order_monotonic": order == sorted(order),
            "unconfirmed_rejected": _required_structured_fields(unconfirmed[1]),
            "operator_confirmed": True,
            "real_observed_completed": True,
            "fault_injected": _required_structured_fields(
                found["live_fault_injected"][1]
            ),
            "safety_hold": {
                "structured": _required_structured_fields(found["safety_hold"][1]),
                "command": hold_payload.get("command"),
            },
            "recovery_land": {
                "structured": _required_structured_fields(found["recovery_land"][1]),
                "command": land_payload.get("command"),
            },
        }

    for line_result in results.values():
        line_result["passed"] = (
            line_result["unauthorized"]["structured"]
            and line_result["unauthorized"]["command"] == "rtl"
            and line_result["expired"]["structured"]
            and line_result["expired"]["real_dispatch_absent"]
            and line_result["confirmed_recovery"]["event_order_monotonic"]
            and line_result["confirmed_recovery"]["unconfirmed_rejected"]
            and line_result["confirmed_recovery"]["fault_injected"]
            and line_result["confirmed_recovery"]["safety_hold"]["structured"]
            and line_result["confirmed_recovery"]["recovery_land"]["structured"]
            and line_result["confirmed_recovery"]["real_observed_completed"]
        )

    result = {
        "schema_version": "1.0",
        "gate": "DUAL_LINE_LIVE_FAULT_RECOVERY",
        "run_id": str(run_id),
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "state_directory": str(config.state_directory.resolve()),
        "injection_layer": "live_telemetry_health_gate",
        "passed": all(item["passed"] for item in results.values()),
        "lines": results,
        "limitations": [
            "heartbeat/GPS loss is injected at the live bridge health gate",
            "the injector does not claim a physical RF or UDP outage",
            "all recovery commands target explicit Gazebo/native-SITL endpoints",
        ],
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    unique_events = {json.dumps(item, sort_keys=True): item for item in selected}
    events_output.parent.mkdir(parents=True, exist_ok=True)
    events_output.write_text(
        "".join(
            json.dumps(item, sort_keys=True, separators=(",", ":")) + "\n"
            for item in unique_events.values()
        ),
        encoding="utf-8",
    )
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        type=Path,
        default=Path("configs/parallel_domain/bridge-live-faults.yaml"),
    )
    parser.add_argument("--output", type=Path)
    parser.add_argument("--events-output", type=Path)
    args = parser.parse_args()
    config = load_bridge_config(args.config)
    output = args.output or config.state_directory / "live-fault-acceptance.json"
    events_output = (
        args.events_output
        or config.state_directory / "live-fault-acceptance-events.jsonl"
    )
    result = asyncio.run(run(config, output, events_output))
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Acceptance for dual-line observation and task-gated live control."""

from __future__ import annotations

import argparse
import asyncio
import json
import tempfile
from datetime import datetime, timedelta, timezone
from pathlib import Path

from aeromind_apm_lite.common.contracts import TwinSource
from aeromind_apm_lite.common.coordinates import (
    CalibrationStatus,
    GeoReference,
    Vec3,
    VehicleHome,
    Wgs84Position,
    map_to_geodetic,
)
from aeromind_apm_lite.common.trajectory import load_evidence_file
from aeromind_apm_lite.ground.parallel_domain.adapters import FakeFlightControllerAdapter
from aeromind_apm_lite.ground.parallel_domain.config import (
    MavlinkEndpointConfig,
    ParallelLineConfig,
)
from aeromind_apm_lite.ground.parallel_domain.core import ParallelDomainCore
from aeromind_apm_lite.ground.parallel_domain.models import (
    AdapterKind,
    EndpointRole,
    MirrorState,
    MissionAction,
    ObservedReplayRecord,
    SafetyPolicy,
)


class AcceptanceClock:
    def __init__(self) -> None:
        self.monotonic_s = 1_000.0
        self.wall = datetime(2026, 8, 16, 2, 0, tzinfo=timezone.utc)

    def monotonic(self) -> float:
        return self.monotonic_s

    def wall_time(self) -> datetime:
        return self.wall

    def advance(self, seconds: float = 1.0) -> None:
        self.monotonic_s += seconds
        self.wall += timedelta(seconds=seconds)


def build_georeference() -> GeoReference:
    origin = Wgs84Position(
        latitude_deg=47.641468,
        longitude_deg=-122.140165,
        altitude_m=122.0,
    )
    return GeoReference(
        calibration_id="parallel-live-acceptance-v1",
        status=CalibrationStatus.SURVEYED,
        map_origin_wgs84=origin,
        map_x_heading_from_true_north_deg=0.0,
        airsim_origin_wgs84=origin,
        vehicle_homes=(
            VehicleHome(vehicle_id=1, map_position_m=(0.0, 0.0, 0.0)),
            VehicleHome(vehicle_id=2, map_position_m=(0.0, 3.0, 0.0)),
        ),
    )


def _line_config(
    line_id: str,
    vehicle_id: int,
    kind: AdapterKind,
    policy: SafetyPolicy,
) -> ParallelLineConfig:
    real_port = 14540 if kind == AdapterKind.PX4 else 14550
    dialect = "common" if kind == AdapterKind.PX4 else "ardupilotmega"
    return ParallelLineConfig(
        line_id=line_id,
        adapter=kind,
        vehicle_id=vehicle_id,
        platform_type=f"{line_id}_multirotor",
        real=MavlinkEndpointConfig(
            endpoint=f"udpin:0.0.0.0:{real_port}",
            target_system=vehicle_id,
            dialect=dialect,
        ),
        virtual=MavlinkEndpointConfig(
            endpoint=f"udpin:0.0.0.0:{real_port + 1}",
            target_system=vehicle_id,
            dialect=dialect,
        ),
        policy=policy,
    )


async def _emit_real(
    adapter: FakeFlightControllerAdapter,
    clock: AcceptanceClock,
    georeference: GeoReference,
    *,
    x_m: float,
    sim_time_s: float,
    connection_generation: int = 1,
    **health: object,
) -> None:
    clock.advance()
    assert georeference.map_origin_wgs84 is not None
    position = map_to_geodetic(
        Vec3(x_m, 0.0, 2.0),
        georeference.venue_calibration(),
        georeference.map_origin_wgs84.coordinate(),
    )
    await adapter.emit(
        latitude_deg=position.latitude_deg,
        longitude_deg=position.longitude_deg,
        altitude_m=position.altitude_m,
        local_position_ned_m=(x_m, 0.0, -2.0),
        mission_phase="manual_flight",
        received_monotonic_s=clock.monotonic_s,
        sampled_at_utc=clock.wall,
        sim_time_s=sim_time_s,
        connection_generation=connection_generation,
        **health,
    )


async def _exercise_line(
    core: ParallelDomainCore,
    line_id: str,
    real: FakeFlightControllerAdapter,
    virtual: FakeFlightControllerAdapter,
    clock: AcceptanceClock,
    georeference: GeoReference,
    sim_scale: float,
) -> dict[str, object]:
    transitions: list[str] = []
    sim_tick = 1
    await _emit_real(real, clock, georeference, x_m=1.0, sim_time_s=sim_scale)
    transitions.append(core._lines[line_id].mirror_status.state.value)

    for fault, values in (
        ("heartbeat", {"heartbeat_age_s": 4.0}),
        ("gps_fix", {"gps_fix_type": 1}),
        ("link_jitter", {"link_ok": False}),
    ):
        before = core._lines[line_id].state_set.observed
        count_before = len(core.store.read_observed(line_id))
        sim_tick += 1
        await _emit_real(
            real,
            clock,
            georeference,
            x_m=50.0 + sim_tick,
            sim_time_s=sim_scale * sim_tick,
            **values,
        )
        assert core._lines[line_id].mirror_status.state == MirrorState.STALE
        assert core._lines[line_id].state_set.observed == before
        assert len(core.store.read_observed(line_id)) == count_before
        transitions.append(f"{fault}:stale")

        sim_tick += 1
        await _emit_real(
            real,
            clock,
            georeference,
            x_m=60.0 + sim_tick,
            sim_time_s=sim_scale * sim_tick,
        )
        assert core._lines[line_id].mirror_status.state == MirrorState.RECOVERING
        assert core._lines[line_id].state_set.observed == before
        transitions.append(f"{fault}:recovering")

        sim_tick += 1
        await _emit_real(
            real,
            clock,
            georeference,
            x_m=float(sim_tick),
            sim_time_s=sim_scale * sim_tick,
        )
        assert core._lines[line_id].mirror_status.state == MirrorState.FRESH
        transitions.append(f"{fault}:fresh")

    before_silence = core._lines[line_id].state_set.observed
    count_before_silence = len(core.store.read_observed(line_id))
    clock.advance(10.0)
    await core.refresh_freshness()
    assert core._lines[line_id].mirror_status.state == MirrorState.STALE
    assert core._lines[line_id].mirror_status.reason == "telemetry_timeout"
    assert core._lines[line_id].state_set.observed == before_silence
    assert len(core.store.read_observed(line_id)) == count_before_silence
    transitions.append("silence_10s:stale")

    await _emit_real(
        real,
        clock,
        georeference,
        x_m=70.0,
        sim_time_s=0.1,
        connection_generation=2,
    )
    assert core._lines[line_id].mirror_status.state == MirrorState.RECOVERING
    assert core._lines[line_id].state_set.observed == before_silence
    transitions.append("reconnect:recovering")
    await _emit_real(
        real,
        clock,
        georeference,
        x_m=11.0,
        sim_time_s=1.1,
        connection_generation=2,
    )
    assert core._lines[line_id].mirror_status.state == MirrorState.FRESH
    transitions.append("reconnect:fresh")

    observed_before_prediction = core._lines[line_id].state_set.observed
    clock.advance()
    await virtual.emit(
        latitude_deg=47.641468,
        longitude_deg=-122.140165,
        altitude_m=122.0,
        local_position_ned_m=(12.0, 0.0, -2.0),
        mission_phase="rehearsal",
        received_monotonic_s=clock.monotonic_s,
        sampled_at_utc=clock.wall,
        sim_time_s=sim_scale * 20.0,
    )
    state_set = core._lines[line_id].state_set
    assert state_set.observed == observed_before_prediction
    assert state_set.predicted is not None
    assert state_set.predicted.source == TwinSource.PREDICTED

    records = core.store.read_observed(line_id)
    required_fields = {
        "vehicle_id",
        "line",
        "source",
        "frame",
        "wall_time",
        "sim_time",
        "monotonic_time_s",
        "frame_calibration_id",
        "frame_calibration_sha256",
        "calibration_status",
        "preview_only",
    }
    encoded = records[0].model_dump(mode="json")
    sequences_contiguous = [item.sequence for item in records] == list(
        range(1, len(records) + 1)
    )
    times_monotonic = all(
        left.monotonic_time_s < right.monotonic_time_s
        and left.wall_time <= right.wall_time
        and (left.sim_time or 0.0) <= (right.sim_time or 0.0)
        for left, right in zip(records, records[1:])
    )
    first_sim_delta_s = records[1].sim_time - records[0].sim_time
    sim_speed_profile_ok = abs(first_sim_delta_s - 3.0 * sim_scale) < 1e-9
    no_control = not real.executions and not virtual.executions
    return {
        "vehicle_id": real.vehicle_id,
        "observed_count": len(records),
        "observed_sequences_contiguous": sequences_contiguous,
        "timestamps_monotonic": times_monotonic,
        "sim_time_scale": sim_scale,
        "sim_speed_profile_ok": sim_speed_profile_ok,
        "required_replay_fields": required_fields <= set(encoded),
        "frame": records[0].frame.value,
        "source": records[0].source.value,
        "first_position_map_m": records[0].position.model_dump(mode="json"),
        "calibration_id": records[0].frame_calibration_id,
        "calibration_sha256": records[0].frame_calibration_sha256,
        "state_slots_isolated": state_set.observed == observed_before_prediction,
        "no_control_dispatched": no_control,
        "transitions": transitions,
        "passed": (
            sequences_contiguous
            and times_monotonic
            and sim_speed_profile_ok
            and required_fields <= set(encoded)
            and no_control
            and state_set.observed == observed_before_prediction
        ),
    }


def validate_live_runtime(state_directory: Path) -> dict[str, object]:
    """Validate persisted output from an observation-only bridge without binding UDP."""

    expected = {"px4": (1, 0.5, 2.0), "apm": (2, 5.0, 20.0)}
    line_results: dict[str, dict[str, object]] = {}
    calibration_hashes: set[str] = set()
    forbidden_events: list[str] = []
    audit_path = state_directory / "audit/events.jsonl"
    if audit_path.is_file():
        for raw in audit_path.read_text(encoding="utf-8").splitlines():
            if not raw.strip():
                continue
            event_type = str(json.loads(raw).get("event_type", ""))
            if event_type in {
                "operator_confirmed",
                "real_mission_dispatched",
                "virtual_mission_dispatched",
            }:
                forbidden_events.append(event_type)

    for line_id, (vehicle_id, min_scale, max_scale) in expected.items():
        replay_path = state_directory / f"replay/{line_id}/observed.jsonl"
        status_path = state_directory / f"twin/{line_id}/real-status.json"
        state_set_path = state_directory / f"twin/{line_id}/state-set.json"
        missing = [
            str(path.relative_to(state_directory))
            for path in (replay_path, status_path, state_set_path)
            if not path.is_file()
        ]
        if missing:
            line_results[line_id] = {"passed": False, "missing_files": missing}
            continue
        raw_records = [
            json.loads(raw)
            for raw in replay_path.read_text(encoding="utf-8").splitlines()
            if raw.strip()
        ]
        records = [ObservedReplayRecord.model_validate(item) for item in raw_records]
        status = json.loads(status_path.read_text(encoding="utf-8"))
        state_set = json.loads(state_set_path.read_text(encoding="utf-8"))
        required_fields = {
            "vehicle_id",
            "line",
            "source",
            "wall_time",
            "sim_time",
            "monotonic_time_s",
            "frame",
            "frame_calibration_id",
            "frame_calibration_sha256",
        }
        fields_ok = bool(raw_records) and required_fields <= set(raw_records[0])
        identity_ok = bool(records) and all(
            item.vehicle_id == vehicle_id
            and item.line.value == line_id
            and item.source.value == "observed"
            and item.frame.value == "map"
            for item in records
        )
        sequence_ok = bool(records) and [item.sequence for item in records] == list(
            range(records[0].sequence, records[0].sequence + len(records))
        )
        time_order_ok = len(records) >= 2 and all(
            left.monotonic_time_s < right.monotonic_time_s
            and left.wall_time <= right.wall_time
            and left.sim_time is not None
            and right.sim_time is not None
            and left.sim_time <= right.sim_time
            for left, right in zip(records, records[1:])
        )
        active_deltas = [
            (
                (right.wall_time - left.wall_time).total_seconds(),
                float(right.sim_time) - float(left.sim_time),
            )
            for left, right in zip(records, records[1:])
            if left.sim_time is not None
            and right.sim_time is not None
            and 0.0 < (right.wall_time - left.wall_time).total_seconds() <= 2.0
        ]
        active_wall_delta_s = sum(item[0] for item in active_deltas)
        active_sim_delta_s = sum(item[1] for item in active_deltas)
        sim_wall_scale = (
            active_sim_delta_s / active_wall_delta_s
            if active_wall_delta_s > 0.0
            else 0.0
        )
        scale_ok = min_scale <= sim_wall_scale <= max_scale
        local_scale_ok = bool(records) and max(
            max(abs(item.position.x), abs(item.position.y), abs(item.position.z))
            for item in records
        ) < 10_000.0
        calibration_ok = bool(records) and all(
            item.frame_calibration_id
            and len(item.frame_calibration_sha256) == 64
            and item.frame_calibration_sha256 == records[0].frame_calibration_sha256
            and item.calibration_status.value == "surveyed"
            and item.preview_only is False
            for item in records
        )
        if records:
            calibration_hashes.add(records[0].frame_calibration_sha256)
        slot = state_set.get("observed")
        state_slot_ok = bool(records) and isinstance(slot, dict) and (
            slot.get("source") == "observed"
            and slot.get("vehicle_id") == vehicle_id
            and float(slot.get("received_monotonic_s", -1.0))
            >= records[-1].monotonic_time_s
        )
        status_ok = bool(records) and (
            status.get("state") == "fresh"
            and status.get("mirror_active") is True
            and int(status.get("observed_sequence", -1)) >= records[-1].sequence
        )
        passed = all(
            (
                fields_ok,
                identity_ok,
                sequence_ok,
                time_order_ok,
                scale_ok,
                local_scale_ok,
                calibration_ok,
                state_slot_ok,
                status_ok,
            )
        )
        line_results[line_id] = {
            "passed": passed,
            "vehicle_id": vehicle_id,
            "observed_count": len(records),
            "status": status.get("state"),
            "sequences_contiguous": sequence_ok,
            "timestamps_monotonic": time_order_ok,
            "required_replay_fields": fields_ok,
            "map_position_is_local_scale": local_scale_ok,
            "surveyed_calibration_not_preview": calibration_ok,
            "state_slot_matches_last_observed": state_slot_ok,
            "sim_to_wall_scale": sim_wall_scale,
            "sim_scale_excludes_offline_gaps_over_s": 2.0,
            "expected_sim_to_wall_scale": [min_scale, max_scale],
        }

    passed = (
        len(line_results) == len(expected)
        and all(bool(item["passed"]) for item in line_results.values())
        and len(calibration_hashes) == 1
        and not forbidden_events
    )
    return {
        "passed": passed,
        "state_directory": str(state_directory.resolve()),
        "comparison_clock": "wall_time",
        "lines": line_results,
        "single_calibration_sha256": len(calibration_hashes) == 1,
        "observation_only_no_dispatch_events": not forbidden_events,
        "forbidden_events": forbidden_events,
    }


def validate_live_control_runtime(state_directory: Path) -> dict[str, object]:
    """Cross-check live control audit, evidence, and comparison artifacts."""

    audit_path = state_directory / "audit/events.jsonl"
    if not audit_path.is_file():
        return {
            "passed": False,
            "state_directory": str(state_directory.resolve()),
            "error": "audit/events.jsonl is missing",
        }
    events = [
        json.loads(raw)
        for raw in audit_path.read_text(encoding="utf-8").splitlines()
        if raw.strip()
    ]
    indexed_events = list(enumerate(events))
    events_by_mission: dict[str, list[tuple[int, dict[str, object]]]] = {}
    for index, event in indexed_events:
        payload = event.get("payload")
        if not isinstance(payload, dict) or payload.get("mission_id") is None:
            continue
        events_by_mission.setdefault(str(payload["mission_id"]), []).append(
            (index, event)
        )

    evidence_by_hash = {}
    for path in sorted((state_directory / "evidence").glob("*.json")):
        evidence = load_evidence_file(path)
        evidence_by_hash[evidence.evidence_hash] = evidence

    comparisons = []
    for path in sorted((state_directory / "comparisons").glob("*.json")):
        payload = json.loads(path.read_text(encoding="utf-8"))
        payload["_path"] = str(path.resolve())
        comparisons.append(payload)

    required_events = (
        "virtual_mission_dispatched",
        "approval_requested",
        "operator_confirmed",
        "real_dispatch_authorization_claimed",
        "real_dispatch_authorization_consumed",
        "real_mission_dispatched",
        "observed_trajectory_completed",
        "predicted_observed_compared",
    )
    expected_lines = {
        "px4": (1, "px4", "px4_offboard_task"),
        "apm": (2, "ardupilot", "ardupilot_guided_task"),
    }
    line_results: dict[str, dict[str, object]] = {}
    calibration_hashes: set[str] = set()
    for line_id, (vehicle_id, adapter, semantic) in expected_lines.items():
        candidates = [item for item in comparisons if item.get("line") == line_id]
        if not candidates:
            line_results[line_id] = {
                "passed": False,
                "error": "no live comparison artifact",
            }
            continue
        comparison = max(candidates, key=lambda item: str(item["generated_at_utc"]))
        mission_id = str(comparison["mission_id"])
        mission_events = events_by_mission.get(mission_id, [])
        first_event: dict[str, tuple[int, dict[str, object]]] = {}
        for index, event in mission_events:
            first_event.setdefault(str(event.get("event_type")), (index, event))
        event_chain_complete = all(name in first_event for name in required_events)
        event_chain_ordered = event_chain_complete and [
            first_event[name][0] for name in required_events
        ] == sorted(first_event[name][0] for name in required_events)

        report = comparison.get("report", {})
        report_hashes = report.get("evidence_hashes", {}) if isinstance(report, dict) else {}
        predicted_hash = report_hashes.get("predicted")
        observed_hash = report_hashes.get("observed")
        predicted = evidence_by_hash.get(predicted_hash)
        observed = evidence_by_hash.get(observed_hash)
        evidence_found = predicted is not None and observed is not None
        evidence_sources_ok = bool(evidence_found) and (
            predicted.role.value == "predicted"
            and predicted.source.value == "airsim"
            and observed.role.value == "observed"
            and observed.source.value == "real_telemetry"
            and predicted.coordinate_frame == "map"
            and observed.coordinate_frame == "map"
        )
        sample_contract_ok = bool(evidence_found) and all(
            len(item.samples) >= 2
            and all(sample.observed_at_utc is not None for sample in item.samples)
            and all(
                left.time_from_start_ms < right.time_from_start_ms
                and left.observed_at_utc <= right.observed_at_utc
                for left, right in zip(item.samples, item.samples[1:])
            )
            for item in (predicted, observed)
        )
        map_scale_ok = bool(evidence_found) and all(
            max(abs(value) for value in sample.position_map_m) < 10_000.0
            for item in (predicted, observed)
            for sample in item.samples
        )
        apm_global_samples_ok = line_id != "apm" or (
            bool(predicted)
            and all(
                sample.source_wgs84 is not None
                and sample.gps_quality is not None
                and sample.gps_quality.fix_type >= 3
                for sample in predicted.samples
            )
        )

        virtual_payload = (
            first_event.get("virtual_mission_dispatched", (0, {}))[1].get("payload", {})
        )
        real_payload = (
            first_event.get("real_mission_dispatched", (0, {}))[1].get("payload", {})
        )
        approval_payload = (
            first_event.get("approval_requested", (0, {}))[1].get("payload", {})
        )
        semantics_ok = all(
            isinstance(payload, dict)
            and payload.get("adapter") == adapter
            and payload.get("execution_semantic") == semantic
            and payload.get("role") == role
            for payload, role in ((virtual_payload, "virtual"), (real_payload, "real"))
        )
        approved_evidence_ok = (
            isinstance(approval_payload, dict)
            and approval_payload.get("predicted_evidence_hash") == predicted_hash
        )
        pair_metrics = report.get("comparisons", []) if isinstance(report, dict) else []
        metrics_ok = (
            bool(pair_metrics)
            and report.get("metrics_within_thresholds") is True
            and all(item.get("within_thresholds") is True for item in pair_metrics)
        )
        wall_time_ok = (
            comparison.get("alignment_clock") == "wall_time"
            and comparison.get("alignment_method")
            == "relative_start_linear_interpolation"
        )
        identity_ok = (
            comparison.get("vehicle_id") == vehicle_id
            and evidence_found
            and predicted.vehicle_id == vehicle_id
            and observed.vehicle_id == vehicle_id
        )
        calibration_ok = bool(evidence_found) and (
            predicted.frame_calibration_hash == observed.frame_calibration_hash
            and predicted.calibration_status.value == "surveyed"
            and observed.calibration_status.value == "surveyed"
            and not predicted.preview_only
            and not observed.preview_only
        )
        if calibration_ok:
            calibration_hashes.add(predicted.frame_calibration_hash)
        passed = all(
            (
                event_chain_ordered,
                evidence_sources_ok,
                sample_contract_ok,
                map_scale_ok,
                apm_global_samples_ok,
                semantics_ok,
                approved_evidence_ok,
                metrics_ok,
                wall_time_ok,
                identity_ok,
                calibration_ok,
            )
        )
        line_results[line_id] = {
            "passed": passed,
            "mission_id": mission_id,
            "comparison": comparison["_path"],
            "execution_semantic": semantic,
            "event_chain_complete_and_ordered": event_chain_ordered,
            "predicted_source": predicted.source.value if predicted else None,
            "observed_source": observed.source.value if observed else None,
            "predicted_sample_count": len(predicted.samples) if predicted else 0,
            "observed_sample_count": len(observed.samples) if observed else 0,
            "map_position_is_local_scale": map_scale_ok,
            "apm_predicted_uses_valid_global_gps_only": apm_global_samples_ok,
            "approved_evidence_hash_matches": approved_evidence_ok,
            "alignment_clock": comparison.get("alignment_clock"),
            "metrics_within_thresholds": metrics_ok,
        }

    capability_rejections = []
    ttl_rejections = []
    replay_rejections = []
    for event in events:
        event_type = str(event.get("event_type", ""))
        payload = event.get("payload")
        if not isinstance(payload, dict):
            continue
        if event_type == "rehearsal_rejected" and any(
            isinstance(check, dict)
            and check.get("name") == "capability_whitelist"
            and check.get("passed") is False
            for check in payload.get("checks", [])
        ):
            capability_rejections.append(payload)
        if (
            event_type == "approval_rejected"
            and payload.get("reason") == "approval_ttl_expired"
        ):
            ttl_rejections.append(payload)
        if event_type in {
            "approval_replay_rejected",
            "real_dispatch_replay_rejected",
            "spool_approval_replay_ignored",
        }:
            replay_rejections.append(payload)

    dispatched_missions = {
        str(event.get("payload", {}).get("mission_id"))
        for event in events
        if event.get("event_type") == "real_mission_dispatched"
        and isinstance(event.get("payload"), dict)
    }
    capability_stopped_before_dispatch = bool(capability_rejections) and all(
        str(payload.get("mission_id")) not in dispatched_missions
        for payload in capability_rejections
    )
    ttl_stopped_before_dispatch = bool(ttl_rejections) and all(
        str(payload.get("mission_id")) not in dispatched_missions
        for payload in ttl_rejections
    )
    rejection_results = {
        "capability_whitelist_rejected": capability_stopped_before_dispatch,
        "approval_ttl_rejected": ttl_stopped_before_dispatch,
        "replay_rejected_or_ignored": bool(replay_rejections),
        "capability_mission_ids": [
            str(payload.get("mission_id")) for payload in capability_rejections
        ],
        "expired_mission_ids": [
            str(payload.get("mission_id")) for payload in ttl_rejections
        ],
    }
    passed = (
        all(item.get("passed") is True for item in line_results.values())
        and len(line_results) == len(expected_lines)
        and len(calibration_hashes) == 1
        and all(
            rejection_results[name] is True
            for name in (
                "capability_whitelist_rejected",
                "approval_ttl_rejected",
                "replay_rejected_or_ignored",
            )
        )
    )
    return {
        "gate": "LIVE_VIRTUAL_TO_REAL_CONTROL",
        "passed": passed,
        "state_directory": str(state_directory.resolve()),
        "lines": line_results,
        "rejections": rejection_results,
        "single_calibration_sha256": len(calibration_hashes) == 1,
    }


async def run(
    output: Path,
    live_state_directory: Path | None = None,
    live_control_state_directory: Path | None = None,
) -> dict[str, object]:
    georeference = build_georeference()
    policy = SafetyPolicy(allowed_actions=tuple(MissionAction))
    clock = AcceptanceClock()
    with tempfile.TemporaryDirectory(prefix="parallel-domain-live-accept-") as state:
        core = ParallelDomainCore(
            georeference=georeference,
            state_directory=state,
            telemetry_stale_after_s=3.0,
            recovery_samples=2,
            monotonic_clock=clock.monotonic,
            wall_clock=clock.wall_time,
        )
        adapters = {}
        for line_id, vehicle_id, kind in (
            ("px4", 1, AdapterKind.PX4),
            ("apm", 2, AdapterKind.ARDUPILOT),
        ):
            real = FakeFlightControllerAdapter(
                kind=kind,
                line_id=line_id,
                vehicle_id=vehicle_id,
                role=EndpointRole.REAL,
                telemetry_callback=lambda telemetry, line=line_id: core.on_telemetry(
                    line, EndpointRole.REAL, telemetry
                ),
            )
            virtual = FakeFlightControllerAdapter(
                kind=kind,
                line_id=line_id,
                vehicle_id=vehicle_id,
                role=EndpointRole.VIRTUAL,
                telemetry_callback=lambda telemetry, line=line_id: core.on_telemetry(
                    line, EndpointRole.VIRTUAL, telemetry
                ),
            )
            core.add_line(
                _line_config(line_id, vehicle_id, kind, policy),
                real=real,
                virtual=virtual,
            )
            adapters[line_id] = (real, virtual)

        await core.start()
        line_results = {
            "px4": await _exercise_line(
                core, "px4", *adapters["px4"], clock, georeference, 1.0
            ),
            "apm": await _exercise_line(
                core, "apm", *adapters["apm"], clock, georeference, 10.0
            ),
        }
        merged = core.store.merged_observed(["px4", "apm"])
        merged_by_wall_time = [item.wall_time for item in merged] == sorted(
            item.wall_time for item in merged
        )
        positions = [
            line_results[line]["first_position_map_m"] for line in ("px4", "apm")
        ]
        cross_line_position_error_m = max(
            abs(float(positions[0][axis]) - float(positions[1][axis]))
            for axis in ("x", "y", "z")
        )
        await core.stop()

    result = {
        "gate": "LIVE_REAL_TO_VIRTUAL_OBSERVATION",
        "passed": (
            all(bool(item["passed"]) for item in line_results.values())
            and merged_by_wall_time
            and cross_line_position_error_m < 1e-6
        ),
        "lines": line_results,
        "cross_line": {
            "comparison_clock": "wall_time",
            "merged_by_wall_time": merged_by_wall_time,
            "same_gps_map_error_m": cross_line_position_error_m,
        },
        "calibration": {
            "status": georeference.status.value,
            "id": georeference.calibration_id,
            "sha256": georeference.config_hash,
        },
        "limitations": [
            "offline acceptance injects deterministic MAVLink telemetry",
            "manual SITL flight and QGC plotting remain operator-run acceptance steps",
            (
                "UDP packet loss cannot be recovered by the bridge; "
                "rejected samples are never fabricated"
            ),
        ],
    }
    if live_state_directory is not None:
        runtime = validate_live_runtime(live_state_directory)
        result["runtime"] = runtime
        result["passed"] = bool(result["passed"]) and bool(runtime["passed"])
    if live_control_state_directory is not None:
        control_runtime = validate_live_control_runtime(live_control_state_directory)
        result["gate"] = "PARALLEL_DOMAIN_BIDIRECTIONAL_LIVE"
        result["control_runtime"] = control_runtime
        result["passed"] = bool(result["passed"]) and bool(
            control_runtime["passed"]
        )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(result, ensure_ascii=True, indent=2) + "\n",
        encoding="utf-8",
    )
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("parallel-domain-live-acceptance.json"),
    )
    parser.add_argument(
        "--live-state-directory",
        type=Path,
        help="also validate replay/status files from a currently running live bridge",
    )
    parser.add_argument(
        "--live-control-state-directory",
        type=Path,
        help="validate virtual rehearsal, gated dispatch, and live comparison artifacts",
    )
    args = parser.parse_args()
    result = asyncio.run(
        run(
            args.output,
            args.live_state_directory,
            args.live_control_state_directory,
        )
    )
    print(json.dumps(result, ensure_ascii=True, indent=2))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

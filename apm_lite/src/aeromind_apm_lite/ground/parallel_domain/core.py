"""Unified virtual rehearsal and real dispatch core.

The core deliberately knows no PX4 or ArduPilot mode names.  It only talks to
the ``FlightControllerAdapter`` contract and owns the twin/evidence/safety
workflow shared by both lines.
"""

from __future__ import annotations

import asyncio
import math
import time
from dataclasses import dataclass, field
from datetime import datetime, timedelta, timezone
from collections.abc import Callable
from typing import Any
from uuid import UUID

from aeromind_apm_lite.common.contracts import (
    CoordinateFrame,
    FidelityLevel,
    HealthStatus,
    TwinSource,
    TwinState,
    TwinStateSet,
    TwinLine,
    Vector3,
)
from aeromind_apm_lite.common.contracts.models import Quaternion
from aeromind_apm_lite.common.coordinates import (
    CalibrationStatus,
    GeoReference,
    Vec3,
    geodetic_to_map,
    local_ned_to_map,
    map_to_enu,
)
from aeromind_apm_lite.common.trajectory import (
    ArtifactVersions,
    GpsQuality,
    TrajectoryEvidence,
    TrajectoryEvidenceStore,
    TrajectoryRole,
    TrajectorySample,
    TrajectorySource,
    compare_trajectories,
)

from .adapters.base import FlightControllerAdapter
from .config import ParallelLineConfig
from .models import (
    AdapterTelemetry,
    ApprovalReceipt,
    ApprovalRequest,
    DispatchRecord,
    EndpointRole,
    GateCheck,
    GateDecision,
    LiveTrajectoryComparison,
    LiveFaultCommand,
    LiveFaultType,
    MissionAction,
    MissionPlan,
    LiveMirrorStatus,
    MirrorState,
    ObservedReplayRecord,
    SafetyActionRecord,
    SafetyPolicy,
    TwinTelemetryRecord,
    canonical_hash,
    utc_now,
)
from .storage import BridgeStore


class GateRejected(ValueError):
    def __init__(self, decision: GateDecision) -> None:
        self.decision = decision
        failed = "; ".join(
            f"{check.name}: {check.detail}"
            for check in decision.checks
            if not check.passed
        )
        super().__init__(failed or "safety gate rejected the mission")


class BridgeStateError(RuntimeError):
    pass


@dataclass
class _MissionSession:
    plan: MissionPlan
    status: str = "new"
    predicted_records: list[tuple[float, AdapterTelemetry, Vec3]] = field(default_factory=list)
    observed_records: list[tuple[float, AdapterTelemetry, Vec3]] = field(default_factory=list)
    predicted_evidence: TrajectoryEvidence | None = None
    planned_evidence: TrajectoryEvidence | None = None
    observed_evidence: TrajectoryEvidence | None = None
    approval_request: ApprovalRequest | None = None
    approval_receipt: ApprovalReceipt | None = None
    comparison: LiveTrajectoryComparison | None = None


@dataclass
class _LineRuntime:
    config: ParallelLineConfig
    real: FlightControllerAdapter
    virtual: FlightControllerAdapter
    sessions: dict[UUID, _MissionSession] = field(default_factory=dict)
    state_set: TwinStateSet | None = None
    mirror_status: LiveMirrorStatus | None = None
    last_arrival_by_role: dict[EndpointRole, float] = field(default_factory=dict)
    last_signature_by_role: dict[EndpointRole, tuple[object, ...]] = field(
        default_factory=dict
    )
    last_connection_generation_by_role: dict[EndpointRole, int] = field(
        default_factory=dict
    )
    last_raw_sim_time_by_role: dict[EndpointRole, float] = field(default_factory=dict)
    sim_offset_by_role: dict[EndpointRole, float] = field(default_factory=dict)
    last_sim_time_by_role: dict[EndpointRole, float] = field(default_factory=dict)
    observed_sequence: int = 0
    received_real_telemetry: bool = False
    pending_fault: LiveFaultCommand | None = None
    consumed_fault_ids: set[UUID] = field(default_factory=set)


class ParallelDomainCore:
    """One core instance can run PX4 and ArduPilot lines concurrently."""

    def __init__(
        self,
        *,
        georeference: GeoReference,
        state_directory: str,
        telemetry_stale_after_s: float = 3.0,
        recovery_samples: int = 2,
        live_fault_injection_enabled: bool = False,
        safety_hold_duration_s: float = 1.0,
        monotonic_clock: Callable[[], float] = time.monotonic,
        wall_clock: Callable[[], datetime] = utc_now,
    ) -> None:
        if not georeference.is_complete:
            raise ValueError("parallel-domain core requires a complete GeoReference")
        self.georeference = georeference
        if telemetry_stale_after_s <= 0.0 or recovery_samples < 1:
            raise ValueError("freshness thresholds must be positive")
        if safety_hold_duration_s < 0.0:
            raise ValueError("safety hold duration must not be negative")
        self.telemetry_stale_after_s = telemetry_stale_after_s
        self.recovery_samples = recovery_samples
        self.live_fault_injection_enabled = live_fault_injection_enabled
        self.safety_hold_duration_s = safety_hold_duration_s
        self._monotonic_clock = monotonic_clock
        self._wall_clock = wall_clock
        self.store = BridgeStore(state_directory)
        self.evidence_store = TrajectoryEvidenceStore(
            f"{state_directory}/evidence"
        )
        self._lines: dict[str, _LineRuntime] = {}
        self._running = False

    def add_line(
        self,
        config: ParallelLineConfig,
        *,
        real: FlightControllerAdapter,
        virtual: FlightControllerAdapter,
    ) -> None:
        if config.line_id in self._lines:
            raise ValueError(f"line {config.line_id} is already registered")
        now_monotonic = self._monotonic_clock()
        now_wall = self._utc_wall_time()
        twin_line = TwinLine(config.line_id)
        state_set = self.store.load_state_set(config.line_id) or TwinStateSet(
            twin_id=f"{config.line_id}-{config.vehicle_id}",
            vehicle_id=config.vehicle_id,
            line=twin_line,
        )
        last_observed = self.store.last_observed(config.line_id)
        if last_observed is not None:
            state_set = state_set.updated(last_observed.state)
        runtime = _LineRuntime(
            config=config,
            real=real,
            virtual=virtual,
            state_set=state_set,
            observed_sequence=(
                last_observed.sequence if last_observed is not None else 0
            ),
            mirror_status=LiveMirrorStatus(
                line=twin_line,
                vehicle_id=config.vehicle_id,
                observed_sequence=(
                    last_observed.sequence if last_observed is not None else 0
                ),
                last_fresh_monotonic_s=(
                    last_observed.monotonic_time_s
                    if last_observed is not None
                    else None
                ),
                last_fresh_wall_time=(
                    last_observed.wall_time if last_observed is not None else None
                ),
                updated_monotonic_s=now_monotonic,
                updated_wall_time=now_wall,
            ),
        )
        self._lines[config.line_id] = runtime
        self.store.save_state_set(config.line_id, state_set)
        assert runtime.mirror_status is not None
        self.store.save_mirror_status(runtime.mirror_status)

    async def start(self) -> None:
        if self._running:
            raise BridgeStateError("parallel-domain core is already running")
        self._running = True
        try:
            for runtime in self._lines.values():
                await runtime.real.start()
                await runtime.virtual.start()
        except Exception:
            await self.stop()
            raise
        self.store.append_event("bridge_started", {"lines": sorted(self._lines)})

    async def stop(self) -> None:
        for runtime in self._lines.values():
            await runtime.real.stop()
            await runtime.virtual.stop()
        self._running = False
        self.store.append_event("bridge_stopped", {"lines": sorted(self._lines)})

    async def submit_rehearsal(self, plan: MissionPlan) -> DispatchRecord:
        runtime = self._runtime_for(plan)
        decision = self._gate(plan, runtime.config.policy, stage="rehearsal")
        if not decision.allowed:
            self.store.append_event(
                "rehearsal_rejected",
                self._structured_rejection(
                    plan,
                    command=self._rejected_command(plan, decision),
                    reason=self._decision_reason(decision),
                    decision=decision,
                ),
            )
            raise GateRejected(decision)
        if plan.mission_id in runtime.sessions:
            raise BridgeStateError(f"mission {plan.mission_id} already exists")
        session = _MissionSession(plan=plan, status="rehearsal_dispatching")
        runtime.sessions[plan.mission_id] = session
        self.store.save_plan(plan)
        result = await runtime.virtual.execute(plan)
        session.status = "rehearsing"
        self.store.append_event("virtual_mission_dispatched", result)
        return result

    async def finish_rehearsal(self, mission_id: UUID | str) -> ApprovalRequest:
        session, _runtime = self._session(mission_id)
        if session.status not in {"rehearsing", "rehearsal_dispatching"}:
            if session.approval_request is not None:
                return session.approval_request
            raise BridgeStateError(f"mission {mission_id} is not in rehearsal")
        if len(session.predicted_records) < 2:
            raise BridgeStateError("rehearsal needs at least two virtual position samples")
        planned, predicted = self._build_evidence(session, EndpointRole.VIRTUAL)
        self.evidence_store.save(planned)
        self.evidence_store.save(predicted)
        requested_at = self._utc_wall_time()
        request = ApprovalRequest(
            mission_id=session.plan.mission_id,
            mission_hash=session.plan.mission_hash,
            line_id=session.plan.line_id,
            vehicle_id=session.plan.vehicle_id,
            predicted_evidence_id=predicted.evidence_id,
            predicted_evidence_hash=predicted.evidence_hash,
            requested_at_utc=requested_at,
            expires_at_utc=requested_at
            + timedelta(seconds=self._runtime_for(session.plan).config.policy.approval_ttl_s),
        )
        session.planned_evidence = planned
        session.predicted_evidence = predicted
        session.approval_request = request
        session.status = "pending_confirmation"
        self.store.save_approval_request(request)
        self.store.append_event("approval_requested", request)
        return request

    async def approve(
        self,
        mission_id: UUID | str,
        *,
        mission_hash: str,
        operator_id: str,
        confirmation_digest: str,
    ) -> ApprovalReceipt:
        session, _runtime = self._session(mission_id)
        request = session.approval_request or self.store.load_approval_request(mission_id)
        if session.approval_receipt is not None:
            self.store.append_event(
                "approval_replay_rejected",
                {"mission_id": str(mission_id), "reason": "approval_already_recorded"},
            )
            raise BridgeStateError("approval request has already been confirmed")
        try:
            existing_receipt = self.store.load_approval_receipt(mission_id)
        except ValueError:
            existing_receipt = None
        if existing_receipt is not None:
            self.store.append_event(
                "approval_replay_rejected",
                {"mission_id": str(mission_id), "reason": "receipt_ledger_exists"},
            )
            raise BridgeStateError("approval request has already been confirmed")
        if request.mission_hash != mission_hash:
            self.store.append_event(
                "approval_rejected",
                {"mission_id": str(mission_id), "reason": "mission_hash_mismatch"},
            )
            raise BridgeStateError("approval mission hash does not match the pending request")
        expected_digest = confirmation_digest_for(request)
        if confirmation_digest != expected_digest:
            self.store.append_event(
                "approval_rejected",
                {"mission_id": str(mission_id), "reason": "confirmation_digest_mismatch"},
            )
            raise BridgeStateError("confirmation digest does not match the request")
        approved_at = self._utc_wall_time()
        if approved_at >= request.expires_at_utc:
            self.store.append_event(
                "approval_rejected",
                self._structured_rejection(
                    session.plan,
                    command="operator_approval",
                    reason="approval_ttl_expired",
                ),
            )
            raise BridgeStateError("approval request has expired")
        receipt = ApprovalReceipt(
            request_id=request.request_id,
            mission_id=request.mission_id,
            mission_hash=request.mission_hash,
            predicted_evidence_hash=request.predicted_evidence_hash,
            operator_id=operator_id,
            confirmation_digest=confirmation_digest,
            approved_at_utc=approved_at,
        )
        session.approval_request = request
        session.approval_receipt = receipt
        session.status = "approved"
        self.store.save_plan(session.plan)
        try:
            self.store.save_approval_receipt(receipt)
        except ValueError as exc:
            self.store.append_event(
                "approval_replay_rejected",
                {"mission_id": str(mission_id), "reason": "receipt_ledger_race"},
            )
            raise BridgeStateError("approval request has already been confirmed") from exc
        self.store.append_event("operator_confirmed", receipt)
        return receipt

    async def dispatch_real(
        self,
        mission_id: UUID | str,
        receipt: ApprovalReceipt | None = None,
    ) -> DispatchRecord:
        session, runtime = self._session(mission_id)
        active_receipt = receipt or session.approval_receipt
        if active_receipt is None:
            try:
                active_receipt = self.store.load_approval_receipt(mission_id)
            except ValueError as exc:
                self.store.append_event(
                    "real_dispatch_rejected",
                    self._structured_rejection(
                        session.plan,
                        command="dispatch_real",
                        reason="operator_confirmation_missing",
                    ),
                )
                raise BridgeStateError(
                    "real dispatch requires operator confirmation"
                ) from exc
        prior_use = self.store.load_dispatch_authorization(mission_id)
        if prior_use is not None:
            self.store.append_event(
                "real_dispatch_replay_rejected",
                {
                    "mission_id": str(mission_id),
                    "authorization_status": prior_use.status,
                    "receipt_hash": prior_use.receipt_hash,
                },
            )
            raise BridgeStateError("approval receipt has already been consumed")
        if active_receipt.mission_id != session.plan.mission_id:
            raise BridgeStateError("approval receipt mission_id mismatch")
        if active_receipt.mission_hash != session.plan.mission_hash:
            raise BridgeStateError("approval receipt mission hash mismatch")
        request = session.approval_request
        if request is None:
            request = self.store.load_approval_request(mission_id)
            session.approval_request = request
        if active_receipt.request_id != request.request_id:
            raise BridgeStateError("approval receipt request_id mismatch")
        if active_receipt.confirmation_digest != confirmation_digest_for(request):
            raise BridgeStateError("approval receipt confirmation digest mismatch")
        if active_receipt.approved_at_utc >= request.expires_at_utc:
            raise BridgeStateError("approval receipt was issued after request expiry")
        if self._utc_wall_time() >= request.expires_at_utc:
            self.store.append_event(
                "real_dispatch_rejected",
                self._structured_rejection(
                    session.plan,
                    command="dispatch_real",
                    reason="approval_ttl_expired",
                ),
            )
            raise BridgeStateError("approval request has expired")
        if session.predicted_evidence is None:
            session.predicted_evidence = self.evidence_store.load(request.predicted_evidence_id)
        if active_receipt.predicted_evidence_hash != session.predicted_evidence.evidence_hash:
            raise BridgeStateError("approval receipt predicted evidence hash mismatch")
        decision = self._gate(session.plan, runtime.config.policy, stage="real_dispatch")
        if not decision.allowed:
            self.store.append_event("real_dispatch_rejected", decision)
            raise GateRejected(decision)
        latest = runtime.real.latest_telemetry()
        checks = self._real_health_checks(latest, runtime.config.policy)
        if not all(check.passed for check in checks):
            rejected = GateDecision(
                mission_id=session.plan.mission_id,
                stage="real_dispatch",
                checks=decision.checks + tuple(checks),
            )
            self.store.append_event("real_health_gate_rejected", rejected)
            raise GateRejected(rejected)
        try:
            authorization = self.store.claim_dispatch(active_receipt)
        except ValueError as exc:
            self.store.append_event(
                "real_dispatch_replay_rejected",
                {"mission_id": str(mission_id), "reason": "authorization_claim_exists"},
            )
            raise BridgeStateError("approval receipt has already been consumed") from exc
        self.store.append_event("real_dispatch_authorization_claimed", authorization)
        session.approval_receipt = active_receipt
        session.status = "real_dispatching"
        try:
            result = await runtime.real.execute(session.plan)
        except Exception as exc:
            session.status = "real_dispatch_failed"
            failed = self.store.complete_dispatch_authorization(
                mission_id,
                status="failed",
                detail=f"{type(exc).__name__}: {exc}",
            )
            self.store.append_event("real_dispatch_failed", failed)
            raise
        completed = self.store.complete_dispatch_authorization(
            mission_id,
            status="dispatched",
            detail=result.detail,
        )
        session.status = "executing_real"
        self.store.append_event("real_dispatch_authorization_consumed", completed)
        self.store.append_event("real_mission_dispatched", result)
        return result

    def arm_live_fault(self, command: LiveFaultCommand) -> None:
        if not self.live_fault_injection_enabled:
            raise BridgeStateError("live fault injection is disabled")
        runtime = self._lines.get(command.line_id)
        if runtime is None or runtime.config.vehicle_id != command.vehicle_id:
            raise BridgeStateError("fault injection target does not match a live line")
        if self._utc_wall_time() >= command.expires_at_utc:
            raise BridgeStateError("fault injection command has expired")
        session = runtime.sessions.get(command.mission_id)
        if session is None or session.status != "executing_real":
            raise BridgeStateError("fault injection requires an executing real mission")
        if (
            command.fault_id in runtime.consumed_fault_ids
            or runtime.pending_fault is not None
        ):
            raise BridgeStateError("fault injection command is duplicate or already pending")
        runtime.pending_fault = command
        self.store.append_event(
            "live_fault_armed",
            {
                "fault_id": str(command.fault_id),
                "mission_id": str(command.mission_id),
                "line_id": command.line_id,
                "vehicle_id": command.vehicle_id,
                "command": command.fault.value,
                "reason": "authorized_sitl_live_fault_injection",
                "wall_time": self._utc_wall_time().isoformat(),
            },
        )

    def approval_digest(self, mission_id: UUID | str) -> str:
        session, _runtime = self._session(mission_id)
        request = session.approval_request or self.store.load_approval_request(mission_id)
        return confirmation_digest_for(request)

    def approval_request(self, mission_id: UUID | str) -> ApprovalRequest:
        session, _runtime = self._session(mission_id)
        return session.approval_request or self.store.load_approval_request(mission_id)

    def status(self) -> dict[str, Any]:
        return {
            "running": self._running,
            "lines": {
                line_id: {
                    "vehicle_id": runtime.config.vehicle_id,
                    "missions": {
                        str(mission_id): session.status
                        for mission_id, session in runtime.sessions.items()
                    },
                    "comparisons": {
                        str(mission_id): session.comparison.model_dump(mode="json")
                        for mission_id, session in runtime.sessions.items()
                        if session.comparison is not None
                    },
                    "real": _telemetry_payload(runtime.real.latest_telemetry()),
                    "virtual": _telemetry_payload(runtime.virtual.latest_telemetry()),
                    "real_link": runtime.real.link_status(),
                    "virtual_link": runtime.virtual.link_status(),
                    "mirror": (
                        runtime.mirror_status.model_dump(mode="json")
                        if runtime.mirror_status is not None
                        else None
                    ),
                    "state_set": (
                        runtime.state_set.model_dump(mode="json")
                        if runtime.state_set is not None
                        else None
                    ),
                }
                for line_id, runtime in self._lines.items()
            },
        }

    def restore_persisted(self) -> None:
        """Rehydrate pending approval state after a bridge restart."""
        plans_root = self.store.root / "plans"
        if not plans_root.is_dir():
            return
        for path in sorted(plans_root.glob("*.json")):
            try:
                plan = self.store.load_plan(path.stem)
                runtime = self._runtime_for(plan)
            except (OSError, ValueError, BridgeStateError):
                continue
            if plan.mission_id in runtime.sessions:
                continue
            # A plan is only resumable after its virtual rehearsal produced an
            # approval request.  A bridge restart must leave an interrupted
            # rehearsal eligible for a fresh dispatch instead of falsely
            # treating it as pending confirmation forever.
            session = _MissionSession(plan=plan, status="pending_confirmation")
            request_path = self.store.root / "approval_requests" / f"{plan.mission_id}.json"
            if not request_path.is_file():
                continue
            if request_path.is_file():
                try:
                    request = self.store.load_approval_request(plan.mission_id)
                    session.approval_request = request
                    session.predicted_evidence = self.evidence_store.load(
                        request.predicted_evidence_id
                    )
                except (OSError, ValueError):
                    continue
            receipt_path = self.store.root / "approvals" / f"{plan.mission_id}.json"
            if receipt_path.is_file():
                try:
                    session.approval_receipt = self.store.load_approval_receipt(
                        plan.mission_id
                    )
                    session.status = "approved"
                except (OSError, ValueError):
                    pass
            runtime.sessions[plan.mission_id] = session

    async def on_telemetry(
        self,
        line_id: str,
        role: EndpointRole,
        telemetry: AdapterTelemetry,
    ) -> None:
        runtime = self._lines.get(line_id)
        if runtime is None:
            raise BridgeStateError(f"telemetry arrived for unknown line {line_id}")
        if telemetry.vehicle_id != runtime.config.vehicle_id:
            self.store.append_event(
                "telemetry_rejected",
                {
                    "line_id": line_id,
                    "role": role.value,
                    "reason": "vehicle_id_mismatch",
                    "expected_vehicle_id": runtime.config.vehicle_id,
                    "received_vehicle_id": telemetry.vehicle_id,
                },
            )
            return
        if not self._accept_arrival(runtime, role, telemetry):
            return
        injected_fault: LiveFaultCommand | None = None
        if role == EndpointRole.REAL:
            telemetry, injected_fault = self._apply_pending_fault(runtime, telemetry)
        previous_generation = runtime.last_connection_generation_by_role.get(role)
        connection_changed = (
            previous_generation is not None
            and previous_generation != telemetry.connection_generation
        )
        runtime.last_connection_generation_by_role[role] = telemetry.connection_generation

        normalized_sim_time = self._normalize_sim_time(runtime, role, telemetry.sim_time_s)
        telemetry = AdapterTelemetry.model_validate(
            {
                **telemetry.model_dump(mode="python"),
                "sim_time_s": normalized_sim_time,
            }
        )
        position = self._map_position(runtime.config, telemetry)
        mirrored_monotonic_s = max(
            self._monotonic_clock(), telemetry.received_monotonic_s
        )
        mirrored_wall_time = max(self._utc_wall_time(), telemetry.sampled_at_utc)

        if role == EndpointRole.REAL:
            runtime.received_real_telemetry = True
            if connection_changed:
                self._set_mirror_status(
                    runtime,
                    state=MirrorState.STALE,
                    reason="connection_reestablished",
                    recovery_samples=0,
                    now_monotonic=mirrored_monotonic_s,
                    now_wall=mirrored_wall_time,
                )
            healthy, reason = self._real_sample_fresh(runtime, telemetry)
            mirror_state = self._advance_mirror_state(
                runtime,
                healthy=healthy,
                reason=reason,
                now_monotonic=mirrored_monotonic_s,
                now_wall=mirrored_wall_time,
            )
            if mirror_state != MirrorState.FRESH:
                self.store.append_event(
                    "telemetry_not_mirrored",
                    {
                        "line_id": line_id,
                        "role": role.value,
                        "mirror_state": mirror_state.value,
                        "reason": runtime.mirror_status.reason,
                        "source_sequence": telemetry.source_sequence,
                        "connection_generation": telemetry.connection_generation,
                    },
                )
                session = self._active_real_session(runtime)
                if session is not None:
                    safety_reason = (
                        f"injected_{injected_fault.fault.value}:{reason}"
                        if injected_fault is not None
                        else reason
                    )
                    await self._trigger_safety_recovery(
                        runtime,
                        session,
                        reason=safety_reason,
                        fault=injected_fault,
                    )
                return
        else:
            mirror_state = MirrorState.FRESH

        state = self._twin_state(
            runtime.config,
            role,
            telemetry,
            position,
            normalized_sim_time=normalized_sim_time,
            mirrored_monotonic_s=mirrored_monotonic_s,
            mirrored_wall_time=mirrored_wall_time,
        )
        assert runtime.state_set is not None
        previous_observed = runtime.state_set.observed
        if role == EndpointRole.REAL:
            next_sequence = runtime.observed_sequence + 1
            observed = ObservedReplayRecord(
                sequence=next_sequence,
                source_sequence=telemetry.source_sequence,
                twin_id=state.twin_id,
                vehicle_id=state.vehicle_id,
                line=state.line,
                frame_calibration_id=self.georeference.calibration_id,
                frame_calibration_sha256=self.georeference.config_hash,
                calibration_status=self.georeference.status,
                preview_only=(
                    self.georeference.status != CalibrationStatus.SURVEYED
                ),
                position=state.position,
                attitude=state.attitude,
                mode=state.mode,
                task_phase=state.task_phase,
                wall_time=state.wall_time,
                sim_time=state.sim_time,
                monotonic_time_s=telemetry.received_monotonic_s,
                mirrored_monotonic_s=mirrored_monotonic_s,
                mirrored_wall_time=mirrored_wall_time,
                state=state,
            )
            self.store.append_observed(observed)
            runtime.observed_sequence = next_sequence
            runtime.state_set = runtime.state_set.updated(state)
            self.store.save_state_set(line_id, runtime.state_set)
            self._set_mirror_status(
                runtime,
                state=MirrorState.FRESH,
                reason="telemetry_fresh",
                recovery_samples=0,
                observed_sequence=runtime.observed_sequence,
                last_fresh_monotonic_s=telemetry.received_monotonic_s,
                last_fresh_wall_time=telemetry.sampled_at_utc,
                now_monotonic=mirrored_monotonic_s,
                now_wall=mirrored_wall_time,
            )
        else:
            runtime.state_set = runtime.state_set.updated(state)
            if runtime.state_set.observed != previous_observed:
                raise BridgeStateError("predicted state update replaced observed state")
            self.store.save_state_set(line_id, runtime.state_set)

        record = TwinTelemetryRecord(
            line_id=line_id,
            role=role,
            flight_mode=telemetry.mode,
            mission_sequence=telemetry.mission_sequence,
            gps_fix_type=telemetry.gps_fix_type,
            gps_hdop=telemetry.gps_hdop,
            satellites_visible=telemetry.satellites_visible,
            mirror_state=mirror_state,
            received_monotonic_s=telemetry.received_monotonic_s,
            received_wall_time=telemetry.sampled_at_utc,
            mirrored_monotonic_s=mirrored_monotonic_s,
            mirrored_wall_time=mirrored_wall_time,
            state=state.model_dump(mode="json"),
        )
        self.store.save_latest(line_id, role.value, record)
        # High-rate telemetry is already persisted in the role-specific replay
        # stream. Audit remains reserved for transitions and safety decisions so
        # a four-link live bridge does not fsync every sample.
        for session in runtime.sessions.values():
            if session.status not in {
                "rehearsing",
                "pending_confirmation",
                "approved",
                "executing_real",
                "safety_hold",
                "recovery_land",
            }:
                continue
            if role == EndpointRole.VIRTUAL and session.status == "rehearsing":
                self._record(session.predicted_records, telemetry, position)
                if telemetry.mission_complete:
                    try:
                        await self.finish_rehearsal(session.plan.mission_id)
                    except BridgeStateError:
                        pass
            elif role == EndpointRole.REAL and session.status in {
                "executing_real",
                "safety_hold",
                "recovery_land",
            }:
                self._record(session.observed_records, telemetry, position)
                if telemetry.mission_complete:
                    await self._finish_observed(session)

    async def refresh_freshness(self) -> None:
        """Expire silent real telemetry without manufacturing a replacement sample."""

        now_monotonic = self._monotonic_clock()
        now_wall = self._utc_wall_time()
        for runtime in self._lines.values():
            status = runtime.mirror_status
            assert status is not None
            if status.state == MirrorState.WAITING and not runtime.received_real_telemetry:
                continue
            last_arrival = runtime.last_arrival_by_role.get(EndpointRole.REAL)
            link = runtime.real.link_status()
            if not bool(link.get("connected", True)):
                reason = "link_disconnected"
            elif last_arrival is None:
                reason = "waiting_for_telemetry"
            elif now_monotonic - last_arrival > self.telemetry_stale_after_s:
                reason = "telemetry_timeout"
            else:
                continue
            if status.state != MirrorState.STALE or status.reason != reason:
                self._set_mirror_status(
                    runtime,
                    state=MirrorState.STALE,
                    reason=reason,
                    recovery_samples=0,
                    now_monotonic=now_monotonic,
                    now_wall=now_wall,
                )
            session = self._active_real_session(runtime)
            if session is not None:
                await self._trigger_safety_recovery(
                    runtime,
                    session,
                    reason=reason,
                    fault=None,
                )

    def _apply_pending_fault(
        self,
        runtime: _LineRuntime,
        telemetry: AdapterTelemetry,
    ) -> tuple[AdapterTelemetry, LiveFaultCommand | None]:
        command = runtime.pending_fault
        if command is None:
            return telemetry, None
        runtime.pending_fault = None
        runtime.consumed_fault_ids.add(command.fault_id)
        now = self._utc_wall_time()
        if now >= command.expires_at_utc:
            self.store.append_event(
                "live_fault_rejected",
                {
                    "fault_id": str(command.fault_id),
                    "mission_id": str(command.mission_id),
                    "line_id": command.line_id,
                    "vehicle_id": command.vehicle_id,
                    "command": command.fault.value,
                    "reason": "fault_command_expired_before_sample",
                    "wall_time": now.isoformat(),
                },
            )
            return telemetry, None
        if command.fault != LiveFaultType.HEARTBEAT_GPS_LOSS:
            raise BridgeStateError(f"unsupported live fault {command.fault.value}")
        injected = AdapterTelemetry.model_validate(
            {
                **telemetry.model_dump(mode="python"),
                "heartbeat_age_s": self.telemetry_stale_after_s + 1.0,
                "gps_fix_type": 1,
                "gps_age_s": self.telemetry_stale_after_s + 1.0,
                "link_ok": False,
            }
        )
        self.store.append_event(
            "live_fault_injected",
            {
                "fault_id": str(command.fault_id),
                "mission_id": str(command.mission_id),
                "line_id": command.line_id,
                "vehicle_id": command.vehicle_id,
                "command": command.fault.value,
                "reason": "heartbeat_lost+gps_fix_lost",
                "wall_time": now.isoformat(),
                "injection_layer": "live_telemetry_health_gate",
                "source_sequence": telemetry.source_sequence,
            },
        )
        return injected, command

    def _active_real_session(self, runtime: _LineRuntime) -> _MissionSession | None:
        return next(
            (
                session
                for session in runtime.sessions.values()
                if session.status == "executing_real"
            ),
            None,
        )

    async def _trigger_safety_recovery(
        self,
        runtime: _LineRuntime,
        session: _MissionSession,
        *,
        reason: str,
        fault: LiveFaultCommand | None,
    ) -> None:
        if session.status != "executing_real":
            return
        fault_id = str(fault.fault_id) if fault is not None else None
        try:
            session.status = "safety_hold"
            hold_command = await runtime.real.safety_hold()
            hold = SafetyActionRecord(
                mission_id=session.plan.mission_id,
                line_id=session.plan.line_id,
                vehicle_id=session.plan.vehicle_id,
                action="safety_hold",
                command=hold_command,
                reason=reason,
                wall_time=self._utc_wall_time(),
            )
            self.store.append_event(
                "safety_hold",
                {**hold.model_dump(mode="json"), "fault_id": fault_id},
            )
            await asyncio.sleep(self.safety_hold_duration_s)
            land_command = await runtime.real.recovery_land()
            session.status = "recovery_land"
            land = SafetyActionRecord(
                mission_id=session.plan.mission_id,
                line_id=session.plan.line_id,
                vehicle_id=session.plan.vehicle_id,
                action="recovery_land",
                command=land_command,
                reason=reason,
                wall_time=self._utc_wall_time(),
            )
            self.store.append_event(
                "recovery_land",
                {**land.model_dump(mode="json"), "fault_id": fault_id},
            )
        except Exception as exc:
            session.status = "safety_recovery_failed"
            self.store.append_event(
                "safety_recovery_failed",
                {
                    "fault_id": fault_id,
                    "mission_id": str(session.plan.mission_id),
                    "line_id": session.plan.line_id,
                    "vehicle_id": session.plan.vehicle_id,
                    "command": "safety_hold_then_recovery_land",
                    "reason": f"{type(exc).__name__}: {exc}",
                    "wall_time": self._utc_wall_time().isoformat(),
                },
            )

    def _accept_arrival(
        self,
        runtime: _LineRuntime,
        role: EndpointRole,
        telemetry: AdapterTelemetry,
    ) -> bool:
        last_arrival = runtime.last_arrival_by_role.get(role)
        signature = (
            telemetry.connection_generation,
            telemetry.source_sequence,
            telemetry.sample_kind,
        )
        duplicate = (
            telemetry.source_sequence > 0
            and runtime.last_signature_by_role.get(role) == signature
        )
        backwards = last_arrival is not None and telemetry.received_monotonic_s <= last_arrival
        if duplicate or backwards:
            self.store.append_event(
                "telemetry_rejected",
                {
                    "line_id": runtime.config.line_id,
                    "role": role.value,
                    "reason": "duplicate" if duplicate else "arrival_not_monotonic",
                    "source_sequence": telemetry.source_sequence,
                    "connection_generation": telemetry.connection_generation,
                    "received_monotonic_s": telemetry.received_monotonic_s,
                },
            )
            return False
        runtime.last_arrival_by_role[role] = telemetry.received_monotonic_s
        runtime.last_signature_by_role[role] = signature
        return True

    def _real_sample_fresh(
        self,
        runtime: _LineRuntime,
        telemetry: AdapterTelemetry,
    ) -> tuple[bool, str]:
        if telemetry.sample_kind != "global_position":
            return False, "real_sample_not_global_position"
        if not telemetry.link_ok:
            return False, "heartbeat_lost"
        if telemetry.heartbeat_age_s is None:
            return False, "heartbeat_missing"
        if telemetry.heartbeat_age_s > self.telemetry_stale_after_s:
            return False, "heartbeat_stale"
        if not telemetry.health_ok:
            return False, "flight_controller_degraded"
        if (telemetry.gps_fix_type or 0) < runtime.config.policy.min_gps_fix_type:
            return False, "gps_fix_lost"
        if telemetry.gps_age_s is None:
            return False, "gps_missing"
        if telemetry.gps_age_s > self.telemetry_stale_after_s:
            return False, "gps_stale"
        age = max(0.0, self._monotonic_clock() - telemetry.received_monotonic_s)
        if age > self.telemetry_stale_after_s:
            return False, "telemetry_stale_on_arrival"
        previous = runtime.state_set.observed if runtime.state_set is not None else None
        if previous is not None and telemetry.sampled_at_utc < previous.wall_time:
            return False, "wall_time_moved_backwards"
        return True, "telemetry_fresh"

    def _advance_mirror_state(
        self,
        runtime: _LineRuntime,
        *,
        healthy: bool,
        reason: str,
        now_monotonic: float,
        now_wall: datetime,
    ) -> MirrorState:
        status = runtime.mirror_status
        assert status is not None
        if not healthy:
            self._set_mirror_status(
                runtime,
                state=MirrorState.STALE,
                reason=reason,
                recovery_samples=0,
                now_monotonic=now_monotonic,
                now_wall=now_wall,
            )
            return MirrorState.STALE
        if status.state in {MirrorState.STALE, MirrorState.RECOVERING}:
            recovered = status.recovery_samples + 1
            if recovered < self.recovery_samples:
                self._set_mirror_status(
                    runtime,
                    state=MirrorState.RECOVERING,
                    reason=f"healthy_sample_{recovered}_of_{self.recovery_samples}",
                    recovery_samples=recovered,
                    now_monotonic=now_monotonic,
                    now_wall=now_wall,
                )
                return MirrorState.RECOVERING
        return MirrorState.FRESH

    def _set_mirror_status(
        self,
        runtime: _LineRuntime,
        *,
        state: MirrorState,
        reason: str,
        recovery_samples: int,
        now_monotonic: float,
        now_wall: datetime,
        observed_sequence: int | None = None,
        last_fresh_monotonic_s: float | None = None,
        last_fresh_wall_time: datetime | None = None,
    ) -> None:
        previous = runtime.mirror_status
        assert previous is not None
        transitioned = previous.state != state
        payload = previous.model_dump(mode="python")
        payload.update(
            {
                "state": state,
                "mirror_active": state == MirrorState.FRESH,
                "reason": reason,
                "transition_sequence": previous.transition_sequence + int(transitioned),
                "recovery_samples": recovery_samples,
                "updated_monotonic_s": max(now_monotonic, previous.updated_monotonic_s),
                "updated_wall_time": max(now_wall, previous.updated_wall_time),
            }
        )
        if observed_sequence is not None:
            payload["observed_sequence"] = observed_sequence
        if last_fresh_monotonic_s is not None:
            payload["last_fresh_monotonic_s"] = last_fresh_monotonic_s
        if last_fresh_wall_time is not None:
            payload["last_fresh_wall_time"] = last_fresh_wall_time
        runtime.mirror_status = LiveMirrorStatus.model_validate(payload)
        self.store.save_mirror_status(runtime.mirror_status)
        if transitioned:
            self.store.append_event(
                "mirror_state_changed",
                {
                    "line_id": runtime.config.line_id,
                    "from": previous.state.value,
                    "to": state.value,
                    "reason": reason,
                    "transition_sequence": runtime.mirror_status.transition_sequence,
                },
            )

    def _normalize_sim_time(
        self,
        runtime: _LineRuntime,
        role: EndpointRole,
        raw_sim_time: float | None,
    ) -> float | None:
        if raw_sim_time is None:
            return None
        previous_raw = runtime.last_raw_sim_time_by_role.get(role)
        offset = runtime.sim_offset_by_role.get(role, 0.0)
        previous = runtime.last_sim_time_by_role.get(role)
        if previous_raw is not None and raw_sim_time < previous_raw:
            offset = max(offset, (previous or 0.0) - raw_sim_time)
            runtime.sim_offset_by_role[role] = offset
        normalized = raw_sim_time + offset
        if previous is not None:
            normalized = max(normalized, previous)
        runtime.last_raw_sim_time_by_role[role] = raw_sim_time
        runtime.last_sim_time_by_role[role] = normalized
        return normalized

    def _utc_wall_time(self) -> datetime:
        value = self._wall_clock()
        if value.tzinfo is None or value.utcoffset() is None:
            raise ValueError("wall clock must return a timezone-aware datetime")
        return value.astimezone(timezone.utc)

    def _record(
        self,
        records: list[tuple[float, AdapterTelemetry, Vec3]],
        telemetry: AdapterTelemetry,
        position: Vec3,
    ) -> None:
        monotonic = telemetry.received_monotonic_s
        if records and monotonic <= records[-1][0]:
            monotonic = records[-1][0] + 0.001
        records.append((monotonic, telemetry, position))

    async def _finish_observed(self, session: _MissionSession) -> None:
        if len(session.observed_records) < 2:
            return
        _planned, observed = self._build_evidence(session, EndpointRole.REAL)
        self.evidence_store.save(observed)
        session.observed_evidence = observed
        if session.predicted_evidence is None:
            request = session.approval_request or self.store.load_approval_request(
                session.plan.mission_id
            )
            session.predicted_evidence = self.evidence_store.load(
                request.predicted_evidence_id
            )
        report = compare_trajectories((session.predicted_evidence, observed))
        predicted_start = session.predicted_evidence.samples[0].observed_at_utc
        observed_start = observed.samples[0].observed_at_utc
        if predicted_start is None or observed_start is None:
            raise BridgeStateError("live comparison requires wall timestamps")
        comparison = LiveTrajectoryComparison(
            mission_id=session.plan.mission_id,
            line=TwinLine(session.plan.line_id),
            vehicle_id=session.plan.vehicle_id,
            predicted_started_at_utc=predicted_start,
            observed_started_at_utc=observed_start,
            report=report,
        )
        self.store.save_comparison(comparison)
        session.comparison = comparison
        session.status = "completed"
        self.store.append_event("observed_trajectory_completed", observed)
        self.store.append_event("predicted_observed_compared", comparison)

    def _build_evidence(
        self,
        session: _MissionSession,
        role: EndpointRole,
    ) -> tuple[TrajectoryEvidence, TrajectoryEvidence]:
        records = (
            session.predicted_records
            if role == EndpointRole.VIRTUAL
            else session.observed_records
        )
        if len(records) < 2:
            raise BridgeStateError("trajectory evidence needs at least two samples")
        first_wall_time = records[0][1].sampled_at_utc
        predicted_samples: list[TrajectorySample] = []
        for monotonic, telemetry, position in records:
            sample_wall_time = telemetry.sampled_at_utc
            if (
                predicted_samples
                and predicted_samples[-1].observed_at_utc is not None
                and sample_wall_time <= predicted_samples[-1].observed_at_utc
            ):
                sample_wall_time = predicted_samples[-1].observed_at_utc + timedelta(
                    microseconds=1
                )
            elapsed_ms = round((sample_wall_time - first_wall_time).total_seconds() * 1_000.0)
            if predicted_samples and elapsed_ms <= predicted_samples[-1].time_from_start_ms:
                elapsed_ms = predicted_samples[-1].time_from_start_ms + 1
            mirrored_monotonic_s = max(time.monotonic(), monotonic)
            mirrored_wall_time = datetime.now(timezone.utc)
            predicted_samples.append(
                TrajectorySample(
                    time_from_start_ms=elapsed_ms,
                    position_map_m=(position.x, position.y, position.z),
                    phase=telemetry.mission_phase,
                    observed_at_utc=sample_wall_time,
                    received_monotonic_s=monotonic,
                    received_at_utc=telemetry.sampled_at_utc,
                    mirrored_monotonic_s=mirrored_monotonic_s,
                    mirrored_at_utc=mirrored_wall_time,
                    sim_time_s=telemetry.sim_time_s,
                    source_wgs84=telemetry.global_position,
                    gps_quality=GpsQuality(
                        fix_type=telemetry.gps_fix_type,
                        hdop=telemetry.gps_hdop,
                        satellites_visible=telemetry.satellites_visible,
                    ),
                )
            )
        planned_samples = tuple(
            TrajectorySample(
                time_from_start_ms=sample.time_from_start_ms,
                position_map_m=_route_position(
                    session.plan,
                    index,
                    len(records),
                    next(
                        home.map_position_m
                        for home in self.georeference.vehicle_homes
                        if home.vehicle_id == session.plan.vehicle_id
                    ),
                ),
                phase=sample.phase,
            )
            for index, sample in enumerate(predicted_samples)
        )
        common = {
            "mission_id": session.plan.mission_id,
            "vehicle_id": session.plan.vehicle_id,
            "frame_calibration_id": self.georeference.calibration_id,
            "frame_calibration_hash": self.georeference.config_hash,
            "calibration_status": self.georeference.status,
            "preview_only": self.georeference.status != CalibrationStatus.SURVEYED,
            "producer_version": "parallel-domain-v1",
            "started_at_utc": predicted_samples[0].observed_at_utc,
            "versions": ArtifactVersions(),
            "source_metadata": {"bridge": "parallel-domain-v1"},
        }
        planned = TrajectoryEvidence(
            role=TrajectoryRole.PLANNED,
            source=TrajectorySource.MISSION_PLAN,
            samples=planned_samples,
            **common,
        )
        measured = TrajectoryEvidence(
            role=(
                TrajectoryRole.PREDICTED
                if role == EndpointRole.VIRTUAL
                else TrajectoryRole.OBSERVED
            ),
            source=(
                TrajectorySource.AIRSIM
                if role == EndpointRole.VIRTUAL
                else TrajectorySource.REAL_TELEMETRY
            ),
            samples=tuple(predicted_samples),
            **{
                **common,
                "source_metadata": {
                    "bridge": "parallel-domain-v1",
                    "endpoint_role": role.value,
                    "trajectory_contract": "predicted"
                    if role == EndpointRole.VIRTUAL
                    else "observed",
                },
            },
        )
        return planned, measured

    def _gate(
        self,
        plan: MissionPlan,
        policy: SafetyPolicy,
        *,
        stage: str,
    ) -> GateDecision:
        checks: list[GateCheck] = []
        checks.append(
            GateCheck(
                name="calibration",
                passed=plan.frame_calibration_id == self.georeference.calibration_id,
                detail=(
                    "map calibration matches active georeference"
                    if plan.frame_calibration_id == self.georeference.calibration_id
                    else "mission calibration does not match active georeference"
                ),
            )
        )
        unsupported = sorted(
            {
                step.action.value
                for step in plan.steps
                if step.action not in policy.allowed_actions
            }
        )
        checks.append(
            GateCheck(
                name="capability_whitelist",
                passed=not unsupported,
                detail="all mission actions are whitelisted"
                if not unsupported
                else f"unsupported actions: {', '.join(unsupported)}",
            )
        )
        takeoff = plan.steps[0].altitude_m
        altitude_values = [
            step.position_map_m[2]
            for step in plan.steps
            if step.position_map_m is not None
        ]
        altitude_values.extend([takeoff] if takeoff is not None else [])
        max_altitude = max(altitude_values or [0.0])
        home = next(
            home
            for home in self.georeference.vehicle_homes
            if home.vehicle_id == plan.vehicle_id
        )
        home_x, home_y, home_z = home.map_position_m
        max_distance = max(
            (
                math.hypot(
                    step.position_map_m[0] - home_x,
                    step.position_map_m[1] - home_y,
                )
                for step in plan.steps
                if step.position_map_m is not None
            ),
            default=0.0,
        )
        altitude_ok = (
            takeoff is not None
            and takeoff >= policy.min_takeoff_altitude_m
            and max_altitude <= policy.max_altitude_m
            and all(value >= home_z for value in altitude_values)
        )
        checks.append(
            GateCheck(
                name="parameter_bounds",
                passed=altitude_ok
                and plan.cruise_speed_m_s <= policy.max_speed_m_s
                and max_distance <= policy.max_distance_from_home_m,
                detail=(
                    "altitude, speed and horizontal range are within policy"
                    if altitude_ok
                    and plan.cruise_speed_m_s <= policy.max_speed_m_s
                    and max_distance <= policy.max_distance_from_home_m
                    else (
                        f"altitude={max_altitude:.2f}m, speed={plan.cruise_speed_m_s:.2f}m/s, "
                        f"range={max_distance:.2f}m exceed policy"
                    )
                ),
            )
        )
        if stage == "real_dispatch" and policy.confirmation_required:
            checks.append(
                GateCheck(
                    name="operator_confirmation",
                    passed=True,
                    detail="receipt is checked by dispatch_real before this gate result is used",
                )
            )
        return GateDecision(
            mission_id=plan.mission_id,
            stage="rehearsal" if stage == "rehearsal" else "real_dispatch",
            checks=tuple(checks),
        )

    def _structured_rejection(
        self,
        plan: MissionPlan,
        *,
        command: str,
        reason: str,
        decision: GateDecision | None = None,
    ) -> dict[str, object]:
        payload: dict[str, object] = {
            "mission_id": str(plan.mission_id),
            "line_id": plan.line_id,
            "vehicle_id": plan.vehicle_id,
            "command": command,
            "reason": reason,
            "wall_time": self._utc_wall_time().isoformat(),
        }
        if decision is not None:
            payload.update(decision.model_dump(mode="json"))
        return payload

    @staticmethod
    def _decision_reason(decision: GateDecision) -> str:
        return "; ".join(
            f"{check.name}:{check.detail}"
            for check in decision.checks
            if not check.passed
        )

    @staticmethod
    def _rejected_command(plan: MissionPlan, decision: GateDecision) -> str:
        capability_failed = any(
            check.name == "capability_whitelist" and not check.passed
            for check in decision.checks
        )
        if capability_failed:
            detail = next(
                check.detail
                for check in decision.checks
                if check.name == "capability_whitelist" and not check.passed
            )
            return detail.partition(":")[2].split(",", maxsplit=1)[0].strip()
        return "mission:" + ",".join(step.action.value for step in plan.steps)

    def _real_health_checks(
        self,
        telemetry: AdapterTelemetry | None,
        policy: SafetyPolicy,
    ) -> tuple[GateCheck, ...]:
        if telemetry is None:
            return (
                GateCheck(
                    name="real_telemetry_fresh",
                    passed=False,
                    detail="no real FCU telemetry has been observed",
                ),
            )
        age = max(0.0, self._monotonic_clock() - telemetry.received_monotonic_s)
        return (
            GateCheck(
                name="real_telemetry_fresh",
                passed=age <= policy.max_telemetry_age_s,
                detail=f"real telemetry age={age:.3f}s",
            ),
            GateCheck(
                name="real_link_health",
                passed=telemetry.link_ok and telemetry.health_ok,
                detail="FCU link and health are nominal"
                if telemetry.link_ok and telemetry.health_ok
                else "FCU link or health is not nominal",
            ),
            GateCheck(
                name="real_gps_fix",
                passed=(telemetry.gps_fix_type or 0) >= policy.min_gps_fix_type,
                detail=f"GPS fix_type={telemetry.gps_fix_type}",
            ),
        )

    def _map_position(
        self,
        config: ParallelLineConfig,
        telemetry: AdapterTelemetry,
    ) -> Vec3:
        calibration = self.georeference.venue_calibration()
        assert self.georeference.map_origin_wgs84 is not None
        if telemetry.global_position is not None:
            return geodetic_to_map(
                telemetry.global_position.coordinate(),
                calibration,
                self.georeference.map_origin_wgs84.coordinate(),
            )
        assert telemetry.local_position_ned_m is not None
        return self._map_local_position(config, telemetry.local_position_ned_m)

    def _map_local_position(
        self,
        config: ParallelLineConfig,
        local_position_ned_m: tuple[float, float, float],
    ) -> Vec3:
        calibration = self.georeference.venue_calibration()
        home = next(
            home
            for home in self.georeference.vehicle_homes
            if home.vehicle_id == config.vehicle_id
        )
        home_enu = map_to_enu(Vec3(*home.map_position_m), calibration)
        return local_ned_to_map(
            Vec3(*local_position_ned_m),
            calibration,
            home_enu,
        )

    def _twin_state(
        self,
        config: ParallelLineConfig,
        role: EndpointRole,
        telemetry: AdapterTelemetry,
        position: Vec3,
        *,
        normalized_sim_time: float | None,
        mirrored_monotonic_s: float,
        mirrored_wall_time: datetime,
    ) -> TwinState:
        velocity = None
        if (
            telemetry.velocity_ned_m_s is not None
            and telemetry.local_position_ned_m is not None
        ):
            base = self._map_local_position(config, telemetry.local_position_ned_m)
            moved = self._map_local_position(
                config,
                tuple(
                    telemetry.local_position_ned_m[i]
                    + telemetry.velocity_ned_m_s[i]
                    for i in range(3)
                ),
            )
            velocity = Vector3(
                x=moved.x - base.x,
                y=moved.y - base.y,
                z=moved.z - base.z,
            )
        attitude = None
        if telemetry.attitude_quaternion_wxyz is not None:
            attitude = Quaternion(
                w=telemetry.attitude_quaternion_wxyz[0],
                x=telemetry.attitude_quaternion_wxyz[1],
                y=telemetry.attitude_quaternion_wxyz[2],
                z=telemetry.attitude_quaternion_wxyz[3],
            )
        nominal = telemetry.link_ok and telemetry.health_ok
        source = TwinSource.OBSERVED if role == EndpointRole.REAL else TwinSource.PREDICTED
        timestamps = (
            {"mapped_at_utc": telemetry.sampled_at_utc}
            if role == EndpointRole.REAL
            else {"simulated_at_utc": telemetry.sampled_at_utc}
        )
        return TwinState(
            twin_id=f"{config.line_id}-{config.vehicle_id}",
            vehicle_id=config.vehicle_id,
            line=config.line_id,
            platform_type=config.platform_type,
            frame=CoordinateFrame.MAP,
            frame_calibration_id=self.georeference.calibration_id,
            position_m=Vector3(x=position.x, y=position.y, z=position.z),
            velocity_m_s=velocity,
            attitude=attitude,
            energy_remaining=(
                telemetry.battery_remaining
                if telemetry.battery_remaining is not None
                else 1.0
            ),
            communication_quality=1.0 if telemetry.link_ok else 0.0,
            mode=telemetry.mode,
            task_phase=telemetry.mission_phase,
            mission_phase=telemetry.mission_phase,
            health_status=HealthStatus.NOMINAL if nominal else HealthStatus.DEGRADED,
            source=source,
            confidence=0.95 if (telemetry.gps_fix_type or 0) >= 3 else 0.6,
            fidelity=FidelityLevel.L2,
            sim_time=normalized_sim_time,
            wall_time=telemetry.sampled_at_utc,
            received_monotonic_s=telemetry.received_monotonic_s,
            received_wall_time=telemetry.sampled_at_utc,
            mirrored_monotonic_s=mirrored_monotonic_s,
            mirrored_wall_time=mirrored_wall_time,
            sampled_at_utc=telemetry.sampled_at_utc,
            **timestamps,
        )

    def _runtime_for(self, plan: MissionPlan) -> _LineRuntime:
        runtime = self._lines.get(plan.line_id)
        if runtime is None:
            raise BridgeStateError(f"unknown line_id {plan.line_id}")
        if plan.vehicle_id != runtime.config.vehicle_id:
            raise BridgeStateError("mission vehicle_id does not match line")
        return runtime

    def _session(self, mission_id: UUID | str) -> tuple[_MissionSession, _LineRuntime]:
        parsed = UUID(str(mission_id))
        for runtime in self._lines.values():
            if parsed in runtime.sessions:
                return runtime.sessions[parsed], runtime
        raise BridgeStateError(f"unknown mission_id {mission_id}")


def confirmation_digest_for(request: ApprovalRequest) -> str:
    return canonical_hash(
        {
            "mission_id": str(request.mission_id),
            "mission_hash": request.mission_hash,
            "predicted_evidence_hash": request.predicted_evidence_hash,
        }
    )


def _route_position(
    plan: MissionPlan,
    index: int,
    count: int,
    home_position: tuple[float, float, float],
) -> tuple[float, float, float]:
    # Planned evidence is a deterministic polyline interpolation over the task steps.
    points: list[tuple[float, float, float]] = []
    first = plan.steps[0]
    assert first.altitude_m is not None
    points.append(
        (home_position[0], home_position[1], home_position[2] + first.altitude_m)
    )
    for step in plan.steps[1:]:
        if step.position_map_m is not None:
            points.append(step.position_map_m)
        elif step.action in {MissionAction.LAND, MissionAction.RTL}:
            points.append(home_position)
    if len(points) == 1:
        points.append(points[0])
    fraction = index / max(1, count - 1)
    scaled = fraction * (len(points) - 1)
    left = min(len(points) - 2, int(scaled))
    ratio = scaled - left
    return tuple(
        points[left][axis] + ratio * (points[left + 1][axis] - points[left][axis])
        for axis in range(3)
    )  # type: ignore[return-value]


def _telemetry_payload(value: AdapterTelemetry | None) -> dict[str, Any] | None:
    return value.model_dump(mode="json") if value is not None else None


__all__ = [
    "BridgeStateError",
    "GateRejected",
    "ParallelDomainCore",
    "confirmation_digest_for",
]

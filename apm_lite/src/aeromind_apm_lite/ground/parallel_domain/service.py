"""Resident spool service for the four MAVLink endpoints."""

from __future__ import annotations

import asyncio
from pathlib import Path

from aeromind_apm_lite.common.coordinates import load_georeference

from .adapters import ArduPilotAdapter, Px4Adapter
from .config import BridgeConfig
from .core import GateRejected, ParallelDomainCore
from .models import (
    AdapterKind,
    ApprovalCommand,
    EndpointRole,
    LiveFaultCommand,
    MissionPlan,
    RealDispatchRequest,
)
from .storage import _atomic_json, _read_json


def build_core(config: BridgeConfig) -> ParallelDomainCore:
    georeference = load_georeference(config.georeference)
    core = ParallelDomainCore(
        georeference=georeference,
        state_directory=str(config.state_directory),
        telemetry_stale_after_s=config.telemetry_stale_after_s,
        recovery_samples=config.recovery_samples,
        live_fault_injection_enabled=config.live_fault_injection_enabled,
        safety_hold_duration_s=config.safety_hold_duration_s,
    )
    for line in config.lines:
        real_callback = _callback(core, line.line_id, EndpointRole.REAL)
        virtual_callback = _callback(core, line.line_id, EndpointRole.VIRTUAL)
        adapter_type = Px4Adapter if line.adapter == AdapterKind.PX4 else ArduPilotAdapter
        real = adapter_type(
            endpoint=line.real,
            georeference=georeference,
            line_id=line.line_id,
            vehicle_id=line.vehicle_id,
            platform_type=line.platform_type,
            role=EndpointRole.REAL,
            telemetry_callback=real_callback,
            reconnect_delay_s=config.reconnect_delay_s,
            silence_timeout_s=config.mavlink_silence_timeout_s,
            sample_interval_s=config.telemetry_sample_interval_s,
        )
        virtual = adapter_type(
            endpoint=line.virtual,
            georeference=georeference,
            line_id=line.line_id,
            vehicle_id=line.vehicle_id,
            platform_type=line.platform_type,
            role=EndpointRole.VIRTUAL,
            telemetry_callback=virtual_callback,
            reconnect_delay_s=config.reconnect_delay_s,
            silence_timeout_s=config.mavlink_silence_timeout_s,
            sample_interval_s=config.telemetry_sample_interval_s,
        )
        core.add_line(line, real=real, virtual=virtual)
    core.restore_persisted()
    return core


class ParallelDomainService:
    def __init__(self, config: BridgeConfig) -> None:
        self.config = config
        self.core = build_core(config)
        self._stop_event = asyncio.Event()
        self._seen_plans: set[str] = set()
        self._seen_approvals: set[str] = set()
        self._seen_dispatches: set[str] = set()
        self._seen_faults: set[str] = set()

    async def run(self) -> None:
        self._stop_event.clear()
        await self.core.start()
        try:
            while not self._stop_event.is_set():
                await self.core.refresh_freshness()
                if not self.config.observation_only:
                    await self._process_spool()
                await asyncio.sleep(self.config.poll_interval_s)
        finally:
            await self.core.stop()

    def stop(self) -> None:
        self._stop_event.set()

    async def _process_spool(self) -> None:
        root = self.config.state_directory
        for path in sorted((root / "inbox" / "plans").glob("*.json")):
            if path.name in self._seen_plans:
                continue
            try:
                plan = MissionPlan.model_validate(_read_json(path))
                status = self.core.status()["lines"].get(plan.line_id, {}).get("missions", {}).get(str(plan.mission_id))
                if status is None:
                    await self.core.submit_rehearsal(plan)
                self._seen_plans.add(path.name)
            except Exception as exc:
                self.core.store.append_event(
                    "spool_plan_error",
                    {"path": str(path), "error": str(exc)},
                )
                self._seen_plans.add(path.name)
        for path in sorted((root / "inbox" / "approvals").glob("*.json")):
            if path.name in self._seen_approvals:
                continue
            try:
                command = ApprovalCommand.model_validate(_read_json(path))
                authorization = self.core.store.load_dispatch_authorization(
                    command.mission_id
                )
                if authorization is not None:
                    self.core.store.append_event(
                        "spool_approval_replay_ignored",
                        {
                            "path": str(path),
                            "mission_id": str(command.mission_id),
                            "authorization_status": authorization.status,
                        },
                    )
                    self._seen_approvals.add(path.name)
                    continue
                try:
                    receipt = self.core.store.load_approval_receipt(command.mission_id)
                except ValueError:
                    receipt = await self.core.approve(
                        command.mission_id,
                        mission_hash=command.mission_hash,
                        operator_id=command.operator_id,
                        confirmation_digest=command.confirmation_digest,
                    )
                await self.core.dispatch_real(command.mission_id, receipt)
            except Exception as exc:
                self.core.store.append_event(
                    "spool_approval_error",
                    {"path": str(path), "error": str(exc)},
                )
                if isinstance(exc, GateRejected):
                    # Keep the already-approved command pending until real telemetry
                    # becomes healthy, without issuing a second approval receipt.
                    continue
            self._seen_approvals.add(path.name)
        for path in sorted((root / "inbox" / "dispatches").glob("*.json")):
            if path.name in self._seen_dispatches:
                continue
            try:
                command = RealDispatchRequest.model_validate(_read_json(path))
                plan = self.core.store.load_plan(command.mission_id)
                if (
                    plan.line_id != command.line_id
                    or plan.vehicle_id != command.vehicle_id
                ):
                    raise ValueError("dispatch request target does not match mission")
                await self.core.dispatch_real(command.mission_id)
            except Exception as exc:
                self.core.store.append_event(
                    "spool_dispatch_error",
                    {"path": str(path), "error": str(exc)},
                )
            self._seen_dispatches.add(path.name)
        for path in sorted((root / "inbox" / "faults").glob("*.json")):
            if path.name in self._seen_faults:
                continue
            try:
                command = LiveFaultCommand.model_validate(_read_json(path))
                if not self.config.live_fault_injection_enabled:
                    raise ValueError("live fault injection is disabled")
                self.core.arm_live_fault(command)
            except Exception as exc:
                self.core.store.append_event(
                    "spool_fault_error",
                    {"path": str(path), "error": str(exc)},
                )
            self._seen_faults.add(path.name)


def write_plan_to_spool(config: BridgeConfig, plan: MissionPlan) -> Path:
    path = config.state_directory / "inbox" / "plans" / f"{plan.mission_id}.json"
    _atomic_json(path, plan.model_dump(mode="json"))
    return path


def write_approval_to_spool(config: BridgeConfig, command: ApprovalCommand) -> Path:
    path = config.state_directory / "inbox" / "approvals" / f"{command.mission_id}.json"
    _atomic_json(path, command.model_dump(mode="json"))
    return path


def write_dispatch_to_spool(
    config: BridgeConfig,
    command: RealDispatchRequest,
) -> Path:
    path = config.state_directory / "inbox" / "dispatches" / f"{command.request_id}.json"
    _atomic_json(path, command.model_dump(mode="json"))
    return path


def write_fault_to_spool(config: BridgeConfig, command: LiveFaultCommand) -> Path:
    path = config.state_directory / "inbox" / "faults" / f"{command.fault_id}.json"
    _atomic_json(path, command.model_dump(mode="json"))
    return path


def _callback(core: ParallelDomainCore, line_id: str, role: EndpointRole):
    async def receive(telemetry):
        await core.on_telemetry(line_id, role, telemetry)

    return receive


__all__ = [
    "ParallelDomainService",
    "build_core",
    "write_approval_to_spool",
    "write_dispatch_to_spool",
    "write_fault_to_spool",
    "write_plan_to_spool",
]

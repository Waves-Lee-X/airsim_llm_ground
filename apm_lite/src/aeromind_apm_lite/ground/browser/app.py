"""FastAPI adapter for the trusted AeroMind APM Lite ground process."""

from __future__ import annotations

import argparse
import asyncio
import time
from contextlib import asynccontextmanager
from dataclasses import fields, replace
from datetime import datetime, timezone
from enum import Enum
from pathlib import Path
from collections.abc import Mapping
from typing import Any, AsyncIterator
from uuid import UUID
from fastapi import Body, FastAPI, HTTPException, Response, WebSocket
from fastapi.staticfiles import StaticFiles
from pydantic import AliasChoices, BaseModel, ConfigDict, Field
from starlette.websockets import WebSocketDisconnect

from aeromind_apm_lite.common.contracts import (
    AckStatus,
    MissionAck,
    VehicleCommand,
    VehicleCommandType,
)
from aeromind_apm_lite.common.credentials import load_shared_secret
from aeromind_apm_lite.common.coordinates import (
    GeoReference,
    GeoReferenceConflict,
    GeoReferenceError,
    GeoReferenceStore,
)
from aeromind_apm_lite.common.trajectory import (
    TrajectoryEvidence,
    TrajectoryEvidenceError,
    TrajectoryEvidenceStore,
    TrajectoryThresholds,
    compare_trajectories,
)
from aeromind_apm_lite.ground.server import VehicleNotConnected
from aeromind_apm_lite.onboard.navigation_mission import (
    EKF_REQUIRED_GPS_NAVIGATION_FLAGS,
)

from .camera import (
    AirSimCameraBridge,
    AirSimCameraConfig,
    CameraBridge,
    CameraFrame,
    CameraUnavailable,
    RtspCameraBridge,
    RtspCameraConfig,
)
from .runtime import ManualRuntime, ManualRuntimeConfig, ManualRuntimeMode
from .semantic import (
    SemanticBusy,
    SemanticService,
    SemanticUnavailable,
)


class BrowserAction(str, Enum):
    ARM = "arm"
    DISARM = "disarm"
    TAKEOFF = "takeoff"
    HOLD = "hold"
    LAND = "land"
    RTL = "rtl"


class CommandRequest(BaseModel):
    model_config = ConfigDict(extra="forbid", populate_by_name=True)

    altitude_m: float | None = Field(
        default=None,
        gt=0.0,
        le=120.0,
        validation_alias=AliasChoices(
            "altitude_m",
            "target_altitude_m",
            "altitude",
        ),
    )
    reason: str = Field(default="browser operator command", max_length=256)
    ttl_ms: int | None = Field(default=None, ge=100, le=300_000)
    completion_timeout_s: float = Field(default=75.0, ge=1.0, le=310.0)


class SerialConnectionRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")

    port: str = Field(min_length=1, max_length=256)
    baudrate: int = Field(default=57_600, ge=300, le=4_000_000)


class VisualAnalysisRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")

    prompt: str = Field(default="", max_length=2_000)


class MissionParseRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")

    instruction: str = Field(min_length=1, max_length=4_000)


class TrajectoryReportRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")

    evidence_ids: tuple[UUID, ...] = Field(min_length=2, max_length=3)
    thresholds: TrajectoryThresholds = Field(default_factory=TrajectoryThresholds)


class EventBroker:
    """Bounded fan-out for browser clients; no vehicle credentials cross it."""

    def __init__(self, queue_size: int = 64) -> None:
        self._queue_size = queue_size
        self._subscribers: set[asyncio.Queue[dict[str, Any]]] = set()
        self._next_event_id = 0

    async def publish(self, event_type: str, payload: dict[str, Any]) -> None:
        self._next_event_id += 1
        event = {
            "event_id": self._next_event_id,
            "event_type": event_type,
            "type": event_type,
            "emitted_at_utc": datetime.now(timezone.utc).isoformat(),
            "payload": payload,
        }
        if event_type == "vehicle_telemetry":
            event["telemetry"] = payload.get("telemetry")
        elif event_type == "command_result":
            event["result"] = payload
        for queue in tuple(self._subscribers):
            if queue.full():
                try:
                    queue.get_nowait()
                except asyncio.QueueEmpty:
                    pass
            try:
                queue.put_nowait(event)
            except asyncio.QueueFull:
                pass

    @asynccontextmanager
    async def subscribe(self) -> AsyncIterator[asyncio.Queue[dict[str, Any]]]:
        queue: asyncio.Queue[dict[str, Any]] = asyncio.Queue(
            maxsize=self._queue_size
        )
        self._subscribers.add(queue)
        try:
            yield queue
        finally:
            self._subscribers.discard(queue)


class BrowserGateway:
    """Aggregate signed internal messages into browser-safe public state."""

    _TERMINAL_STATUSES = (
        AckStatus.COMPLETED,
        AckStatus.FAILED,
        AckStatus.CANCELLED,
        AckStatus.EXPIRED,
    )

    def __init__(
        self,
        runtime: ManualRuntime,
        *,
        camera: CameraBridge | None = None,
        semantic: SemanticService | None = None,
        georeference: GeoReferenceStore | None = None,
        trajectory_evidence: TrajectoryEvidenceStore | None = None,
        telemetry_interval_s: float = 0.2,
    ) -> None:
        if telemetry_interval_s <= 0.0:
            raise ValueError("telemetry_interval_s must be positive")
        self.runtime = runtime
        self.camera = camera
        self.semantic = semantic or SemanticService()
        self.georeference = georeference or GeoReferenceStore()
        self.trajectory_evidence = trajectory_evidence or TrajectoryEvidenceStore(
            Path.home() / ".aeromind" / "trajectory-evidence"
        )
        self.events = EventBroker()
        self._telemetry_interval_s = telemetry_interval_s
        self._telemetry_task: asyncio.Task[None] | None = None
        self._command_tasks: set[asyncio.Task[dict[str, Any]]] = set()
        self._runtime_lock = asyncio.Lock()
        self._georeference_lock = asyncio.Lock()
        self._runtime_error: str | None = None
        self._serial_reconnecting = False

    async def start(self) -> None:
        if self.camera is not None:
            await self.camera.start()
        try:
            await self.runtime.start()
        except Exception as exc:
            if self.runtime.config.mode != ManualRuntimeMode.REAL_SERIAL:
                if self.camera is not None:
                    await self.camera.stop()
                raise
            self._runtime_error = str(exc)
        except BaseException:
            if self.camera is not None:
                await self.camera.stop()
            raise
        self._telemetry_task = asyncio.create_task(
            self._telemetry_loop(),
            name="browser-telemetry-publisher",
        )
        await self.events.publish("service_status", self.status_payload())

    async def stop(self) -> None:
        telemetry_task = self._telemetry_task
        self._telemetry_task = None
        if telemetry_task is not None:
            telemetry_task.cancel()
            await asyncio.gather(telemetry_task, return_exceptions=True)
        command_tasks = tuple(self._command_tasks)
        for task in command_tasks:
            task.cancel()
        await asyncio.gather(*command_tasks, return_exceptions=True)
        try:
            await self.runtime.stop()
        finally:
            if self.camera is not None:
                await self.camera.stop()

    def public_config(self) -> dict[str, Any]:
        payload = self.runtime.public_config()
        payload["camera"] = self.camera_status_payload()
        payload["semantic"] = self.semantic.status_payload()
        payload["serial_runtime_configurable"] = (
            self.runtime.config.mode == ManualRuntimeMode.REAL_SERIAL
        )
        reference = self.georeference.value
        payload["georeference"] = {
            "calibration_id": reference.calibration_id,
            "status": reference.status.value,
            "config_hash": reference.config_hash,
            "complete": reference.is_complete,
        }
        return payload

    def georeference_payload(self) -> dict[str, Any]:
        return self.georeference.public_payload()

    async def update_georeference(
        self,
        reference: GeoReference,
    ) -> dict[str, Any]:
        async with self._georeference_lock:
            try:
                await asyncio.to_thread(self.georeference.save, reference)
            except GeoReferenceConflict as exc:
                raise HTTPException(status_code=409, detail=str(exc)) from exc
            except GeoReferenceError as exc:
                raise HTTPException(status_code=500, detail=str(exc)) from exc
        payload = self.georeference_payload()
        await self.events.publish("georeference_updated", payload)
        return payload

    async def save_trajectory_evidence(
        self,
        evidence: TrajectoryEvidence,
    ) -> dict[str, Any]:
        reference = self.georeference.value
        if evidence.frame_calibration_id != reference.calibration_id:
            raise HTTPException(
                status_code=409,
                detail="trajectory calibration_id does not match the active GeoReference",
            )
        if evidence.frame_calibration_hash != reference.config_hash:
            raise HTTPException(
                status_code=409,
                detail="trajectory calibration hash does not match the active GeoReference",
            )
        if evidence.calibration_status != reference.status:
            raise HTTPException(
                status_code=409,
                detail="trajectory calibration status does not match the active GeoReference",
            )
        try:
            saved = await asyncio.to_thread(self.trajectory_evidence.save, evidence)
        except TrajectoryEvidenceError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        payload = saved.public_payload()
        await self.events.publish(
            "trajectory_evidence_saved",
            {
                "evidence_id": str(saved.evidence_id),
                "evidence_hash": saved.evidence_hash,
                "role": saved.role.value,
            },
        )
        return payload

    async def load_trajectory_evidence(
        self,
        evidence_id: UUID,
    ) -> dict[str, Any]:
        try:
            evidence = await asyncio.to_thread(
                self.trajectory_evidence.load,
                evidence_id,
            )
            return evidence.public_payload()
        except TrajectoryEvidenceError as exc:
            raise HTTPException(status_code=404, detail=str(exc)) from exc

    async def trajectory_report(
        self,
        request: TrajectoryReportRequest,
    ) -> dict[str, Any]:
        if len(set(request.evidence_ids)) != len(request.evidence_ids):
            raise HTTPException(status_code=422, detail="evidence_ids must be unique")

        def build_report() -> dict[str, Any]:
            evidence = [
                self.trajectory_evidence.load(evidence_id)
                for evidence_id in request.evidence_ids
            ]
            return compare_trajectories(evidence, request.thresholds).public_payload()

        try:
            return await asyncio.to_thread(build_report)
        except TrajectoryEvidenceError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    async def available_serial_ports(self) -> list[dict[str, Any]]:
        def enumerate_ports() -> list[dict[str, Any]]:
            from serial.tools import list_ports

            return [
                {
                    "device": item.device,
                    "description": item.description,
                    "hwid": item.hwid,
                    "manufacturer": item.manufacturer,
                    "vid": item.vid,
                    "pid": item.pid,
                    "serial_number": item.serial_number,
                }
                for item in sorted(
                    list_ports.comports(), key=lambda port: port.device
                )
            ]

        return await asyncio.to_thread(enumerate_ports)

    async def reconfigure_serial(
        self,
        *,
        port: str,
        baudrate: int,
    ) -> dict[str, Any]:
        if self.runtime.config.mode != ManualRuntimeMode.REAL_SERIAL:
            raise HTTPException(
                status_code=409,
                detail=(
                    "serial settings are only available in real_serial mode"
                ),
            )
        if any(not task.done() for task in self._command_tasks):
            raise HTTPException(
                status_code=409,
                detail=(
                    "wait for the active flight command before reconnecting"
                ),
            )

        normalized_port = port.strip()
        if not normalized_port:
            raise HTTPException(
                status_code=422, detail="serial port is required"
            )

        async with self._runtime_lock:
            old_config = self.runtime.config
            new_config = replace(
                old_config,
                ground_serial_port=normalized_port,
                ground_serial_baudrate=baudrate,
            )
            self._serial_reconnecting = True
            self._runtime_error = None
            await self.events.publish("service_status", self.status_payload())
            try:
                await self.runtime.stop()
                self.runtime.config = new_config
                try:
                    await self.runtime.start()
                except Exception as connect_error:
                    self.runtime.config = old_config
                    try:
                        await self.runtime.start()
                    except Exception as rollback_error:
                        self._runtime_error = (
                            f"failed to open {normalized_port} at {baudrate}: "
                            f"{connect_error}; rollback failed: "
                            f"{rollback_error}"
                        )
                    else:
                        self._runtime_error = (
                            f"failed to open {normalized_port} at {baudrate}: "
                            f"{connect_error}; restored "
                            f"{old_config.ground_serial_port} at "
                            f"{old_config.ground_serial_baudrate}"
                        )
                    raise HTTPException(
                        status_code=503,
                        detail=self._runtime_error,
                    ) from connect_error
                self._runtime_error = None
            finally:
                self._serial_reconnecting = False

        payload = {
            "port": self.runtime.config.ground_serial_port,
            "baudrate": self.runtime.config.ground_serial_baudrate,
            "connected": self.runtime.started,
            "detail": "serial link opened; waiting for the onboard agent",
        }
        await self.events.publish("service_status", self.status_payload())
        return payload

    def camera_status_payload(self) -> dict[str, Any]:
        if self.camera is not None:
            return self.camera.status_payload()
        payload = dict(self.runtime.public_config()["camera"])
        payload.setdefault("state", "disabled")
        payload.setdefault("stream_url", "/api/camera/frame")
        return payload

    async def camera_frame(self) -> CameraFrame:
        if self.camera is None:
            raise CameraUnavailable("camera bridge is disabled")
        return await self.camera.get_frame()

    def health_payload(self) -> dict[str, Any]:
        status = self.status_payload()
        return {
            "status": "ok" if self.runtime.started else "starting",
            "ready": status["vehicle_connected"],
            "runtime_mode": self.runtime.config.mode.value,
            "vehicle_id": self.runtime.config.vehicle_id,
            "fcu_link_ok": status["fcu_link_ok"],
        }

    def status_payload(self) -> dict[str, Any]:
        vehicle_connected = False
        telemetry = self._link_snapshot_payload()
        heartbeat = None
        session_info = None
        if self.runtime.started:
            session_info = self.runtime.server.session_info(
                self.runtime.config.vehicle_id
            )
            vehicle_connected = session_info is not None
            if vehicle_connected:
                try:
                    heartbeat = self.runtime.server.last_heartbeat(
                        self.runtime.config.vehicle_id
                    )
                except VehicleNotConnected:
                    heartbeat = None
        serial_stats = (
            getattr(self.runtime.server, "serial_stats", {})
            if self.runtime.started
            else {}
        )
        return {
            "runtime_started": self.runtime.started,
            "runtime_error": self._runtime_error,
            "runtime_mode": self.runtime.config.mode.value,
            "vehicle_id": self.runtime.config.vehicle_id,
            "vehicle_connected": vehicle_connected,
            "onboard_agent_connected": self.runtime.agent_connected,
            "onboard_agent": {
                "connected": vehicle_connected,
                "session_id": (
                    str(session_info.session_id)
                    if session_info is not None
                    else None
                ),
                "session_age_s": (
                    max(0.0, time.monotonic() - session_info.connected_monotonic_s)
                    if session_info is not None
                    else None
                ),
                "software_version": (
                    heartbeat.software_version if heartbeat is not None else None
                ),
                "configuration_hash": (
                    heartbeat.configuration_hash if heartbeat is not None else None
                ),
            },
            "command_output_enabled": (
                heartbeat.command_output_enabled
                if heartbeat is not None
                else self.runtime.config.mode != ManualRuntimeMode.REAL_SERIAL
            ),
            "allowed_commands": (
                [command.value for command in heartbeat.allowed_commands]
                if heartbeat is not None
                else (
                    [command.value for command in VehicleCommandType]
                    if self.runtime.config.mode != ManualRuntimeMode.REAL_SERIAL
                    else []
                )
            ),
            "fcu_ready": bool(
                (
                    getattr(self.runtime.link, "ready", False)
                    and self.runtime.config.mode == ManualRuntimeMode.SITL
                )
                or (
                    self.runtime.config.mode == ManualRuntimeMode.REAL_SERIAL
                    and telemetry is not None
                    and telemetry["fcu_link_ok"]
                )
            ),
            "fcu_link_ok": bool(
                telemetry is not None and telemetry["fcu_link_ok"]
            ),
            "armed": telemetry["armed"] if telemetry is not None else None,
            "mode": telemetry["mode"] if telemetry is not None else None,
            "coordinate_frame": "local_ned",
            "telemetry": telemetry,
            "commands_are_simulated": (
                self.runtime.config.mode == ManualRuntimeMode.DEMO
            ),
            "external_processes_managed": False,
            "ground_link": {
                "transport": (
                    "serial"
                    if self.runtime.config.mode
                    == ManualRuntimeMode.REAL_SERIAL
                    else "loopback_websocket"
                ),
                "stats": serial_stats,
                "port": self.runtime.config.ground_serial_port,
                "baudrate": self.runtime.config.ground_serial_baudrate,
                "reconnecting": self._serial_reconnecting,
                "error": self._runtime_error,
            },
            "camera": self.camera_status_payload(),
            "semantic": self.semantic.status_payload(),
        }

    async def analyze_current_frame(self, prompt: str) -> dict[str, Any]:
        frame = await self.camera_frame()
        payload = await self.semantic.analyze_frame(frame, prompt)
        await self.events.publish("visual_semantic_result", payload)
        return payload

    async def parse_mission(self, instruction: str) -> dict[str, Any]:
        payload = await self.semantic.parse_mission(instruction)
        await self.events.publish("mission_parse_result", payload)
        return payload

    def telemetry_payload(self) -> dict[str, Any]:
        telemetry = self._link_snapshot_payload()
        return {
            "available": telemetry is not None,
            "vehicle_id": self.runtime.config.vehicle_id,
            "coordinate_frame": "local_ned",
            "telemetry": telemetry,
        }

    def _link_snapshot_payload(self) -> dict[str, Any] | None:
        link = self.runtime.link
        if link is None:
            if (
                not self.runtime.started
                or self.runtime.config.mode != ManualRuntimeMode.REAL_SERIAL
            ):
                return None
            try:
                telemetry = self.runtime.server.latest_telemetry(
                    self.runtime.config.vehicle_id
                )
            except VehicleNotConnected:
                return None
            if telemetry is None:
                return None
            payload = telemetry.model_dump(mode="json")
            health = payload["health"]
            payload.update(
                {
                    "fcu_link_ok": health["fcu_link_ok"],
                    "gps_fix_type": health["gps_fix_type"],
                    "gps_hdop": health["gps_hdop"],
                    "gps_healthy": health["gps_healthy"],
                    "prearm_ok": health["prearm_ok"],
                    "ekf_ok": health["ekf_ok"],
                }
            )
            return payload
        snapshot = link.telemetry_snapshot()
        if snapshot.armed is None or snapshot.mode is None:
            return None
        payload: dict[str, Any] = {}
        for field in fields(snapshot):
            value = getattr(snapshot, field.name)
            payload[field.name] = (
                dict(value) if isinstance(value, Mapping) else value
            )
        payload["frame"] = "local_ned"
        position = snapshot.local_position_ned_m
        velocity = snapshot.velocity_ned_m_s
        payload["position_m"] = (
            {"x": position[0], "y": position[1], "z": position[2]}
            if position is not None
            else None
        )
        payload["velocity_m_s"] = (
            {"x": velocity[0], "y": velocity[1], "z": velocity[2]}
            if velocity is not None
            else None
        )
        payload["health"] = {
            "fcu_link_ok": snapshot.fcu_link_ok,
            "gps_fix_type": snapshot.gps_fix_type or 0,
            "gps_hdop": snapshot.gps_hdop,
            "gps_healthy": snapshot.gps_healthy,
            "prearm_ok": snapshot.prearm_ok,
            "ekf_ok": (
                snapshot.ekf_flags is not None
                and snapshot.ekf_flags
                & EKF_REQUIRED_GPS_NAVIGATION_FLAGS
                == EKF_REQUIRED_GPS_NAVIGATION_FLAGS
            ),
        }
        return payload

    async def issue_command(
        self,
        vehicle_id: int,
        action: BrowserAction,
        request: CommandRequest,
    ) -> dict[str, Any]:
        if vehicle_id != self.runtime.config.vehicle_id:
            raise HTTPException(
                status_code=404,
                detail="vehicle is not registered",
            )
        if action == BrowserAction.TAKEOFF and request.altitude_m is None:
            raise HTTPException(
                status_code=422,
                detail="takeoff requires altitude_m",
            )
        if action != BrowserAction.TAKEOFF and request.altitude_m is not None:
            raise HTTPException(
                status_code=422,
                detail="altitude_m is only valid for takeoff",
            )
        if not self.runtime.started:
            raise HTTPException(status_code=503, detail="runtime is not ready")

        command_type = VehicleCommandType(action.value)
        if self.runtime.config.mode == ManualRuntimeMode.REAL_SERIAL:
            try:
                heartbeat = self.runtime.server.last_heartbeat(vehicle_id)
            except VehicleNotConnected as exc:
                raise HTTPException(status_code=503, detail=str(exc)) from exc
            allowed_commands = (
                set(heartbeat.allowed_commands)
                if heartbeat is not None and heartbeat.command_output_enabled
                else set()
            )
            if command_type not in allowed_commands:
                raise HTTPException(
                    status_code=403,
                    detail=(
                        f"{action.value} is disabled by the onboard command allowlist"
                    ),
                )
        try:
            command = await self.runtime.server.send_command(
                vehicle_id,
                command_type,
                target_altitude_m=request.altitude_m,
                reason=request.reason,
                ttl_ms=request.ttl_ms or self.runtime.config.command_ttl_ms,
            )
        except VehicleNotConnected as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc

        await self.events.publish(
            "command_progress",
            {
                "command_id": str(command.message_id),
                "vehicle_id": vehicle_id,
                "command": action.value,
                "stage": "sent",
                "simulated": (
                    self.runtime.config.mode == ManualRuntimeMode.DEMO
                ),
            },
        )
        task = asyncio.create_task(
            self._track_command(command, request.completion_timeout_s),
            name=f"browser-command-{command.message_id}",
        )
        self._command_tasks.add(task)
        task.add_done_callback(self._command_tasks.discard)
        return await asyncio.shield(task)

    async def _track_command(
        self,
        command: VehicleCommand,
        timeout_s: float,
    ) -> dict[str, Any]:
        try:
            accepted = await self.runtime.server.wait_for_ack(
                command.vehicle_id,
                command.message_id,
                AckStatus.ACCEPTED,
                timeout_s=min(5.0, timeout_s),
            )
        except (asyncio.TimeoutError, VehicleNotConnected) as exc:
            return await self._timeout_outcome(command, str(exc))
        await self._publish_ack(command, accepted)

        running_task = asyncio.create_task(
            self._wait_and_publish(command, AckStatus.RUNNING, timeout_s)
        )
        terminal_tasks = {
            asyncio.create_task(
                self._wait_and_publish(command, status, timeout_s)
            ): status
            for status in self._TERMINAL_STATUSES
        }
        done, pending = await asyncio.wait(
            set(terminal_tasks),
            timeout=timeout_s,
            return_when=asyncio.FIRST_COMPLETED,
        )
        terminal: MissionAck | None = None
        for task in done:
            try:
                terminal = task.result()
            except (asyncio.TimeoutError, VehicleNotConnected):
                continue
            else:
                break

        running: MissionAck | None = None
        if running_task.done():
            try:
                running = running_task.result()
            except (asyncio.TimeoutError, VehicleNotConnected):
                pass

        for task in (*pending, *done, running_task):
            if not task.done():
                task.cancel()
        await asyncio.gather(
            *terminal_tasks,
            running_task,
            return_exceptions=True,
        )

        if terminal is None:
            return await self._timeout_outcome(
                command,
                f"no terminal ACK within {timeout_s:g}s",
                accepted=accepted,
                running=running,
            )
        outcome = self._outcome_payload(
            command,
            accepted=accepted,
            running=running,
            terminal=terminal,
        )
        await self.events.publish("command_result", outcome)
        return outcome

    async def _wait_and_publish(
        self,
        command: VehicleCommand,
        status: AckStatus,
        timeout_s: float,
    ) -> MissionAck:
        ack = await self.runtime.server.wait_for_ack(
            command.vehicle_id,
            command.message_id,
            status,
            timeout_s=timeout_s,
        )
        await self._publish_ack(command, ack)
        return ack

    async def _publish_ack(
        self,
        command: VehicleCommand,
        ack: MissionAck,
    ) -> None:
        await self.events.publish(
            "command_progress",
            {
                "command_id": str(command.message_id),
                "vehicle_id": command.vehicle_id,
                "command": command.command.value,
                "stage": ack.status.value,
                "detail": ack.detail,
                "apm_command_result": ack.apm_command_result,
                "physical_completion_confirmed": (
                    ack.physical_completion_confirmed
                ),
                "simulated": (
                    self.runtime.config.mode == ManualRuntimeMode.DEMO
                ),
            },
        )

    async def _timeout_outcome(
        self,
        command: VehicleCommand,
        detail: str,
        *,
        accepted: MissionAck | None = None,
        running: MissionAck | None = None,
    ) -> dict[str, Any]:
        outcome = self._outcome_payload(
            command,
            accepted=accepted,
            running=running,
            terminal=None,
            timeout_detail=detail,
        )
        await self.events.publish("command_result", outcome)
        return outcome

    def _outcome_payload(
        self,
        command: VehicleCommand,
        *,
        accepted: MissionAck | None,
        running: MissionAck | None,
        terminal: MissionAck | None,
        timeout_detail: str = "",
    ) -> dict[str, Any]:
        fcu_evidence = running or terminal
        terminal_status = (
            terminal.status.value if terminal else "gateway_timeout"
        )
        terminal_detail = (
            terminal.detail if terminal is not None else timeout_detail
        )
        physical_confirmed = bool(
            terminal is not None and terminal.physical_completion_confirmed
        )
        fcu_ack_payload = {
            "applicable": self.runtime.config.mode != ManualRuntimeMode.DEMO,
            "received": bool(
                fcu_evidence is not None
                and fcu_evidence.apm_command_result is not None
            ),
            "result": (
                fcu_evidence.apm_command_result
                if fcu_evidence is not None
                else None
            ),
            "detail": (
                fcu_evidence.detail
                if fcu_evidence is not None
                else "no FCU ACK evidence"
            ),
        }
        return {
            "schema_version": "1.0",
            "command_id": str(command.message_id),
            "request_id": str(command.message_id),
            "vehicle_id": command.vehicle_id,
            "command": command.command.value,
            "simulated": self.runtime.config.mode == ManualRuntimeMode.DEMO,
            "application": {
                "accepted": accepted is not None,
                "status": (
                    accepted.status.value if accepted is not None else None
                ),
                "detail": (
                    accepted.detail if accepted is not None else timeout_detail
                ),
            },
            "fcu_ack": fcu_ack_payload,
            "mavlink_ack": fcu_ack_payload,
            "physical_completion": {
                "confirmed": physical_confirmed,
                "detail": terminal_detail,
            },
            "terminal": {
                "status": terminal_status,
                "detail": terminal_detail,
            },
            "status": terminal_status,
            "detail": terminal_detail,
            "successful": (
                terminal_status == AckStatus.COMPLETED.value
                and physical_confirmed
            ),
        }

    async def _telemetry_loop(self) -> None:
        while True:
            payload = self.telemetry_payload()
            if payload["available"]:
                await self.events.publish("vehicle_telemetry", payload)
            await asyncio.sleep(self._telemetry_interval_s)


def _default_static_dir() -> Path | None:
    project_web = Path(__file__).resolve().parents[4] / "web"
    if project_web.is_dir():
        return project_web
    return None


def _default_georeference_path() -> Path:
    return Path(__file__).resolve().parents[4] / "configs/calibration/venue.yaml"


def _default_trajectory_evidence_dir() -> Path:
    return Path.home() / ".aeromind" / "trajectory-evidence"


def create_app(
    runtime: ManualRuntime | None = None,
    *,
    camera: CameraBridge | None = None,
    semantic: SemanticService | None = None,
    georeference: GeoReferenceStore | None = None,
    trajectory_evidence: TrajectoryEvidenceStore | None = None,
    static_dir: str | Path | None = None,
) -> FastAPI:
    runtime = runtime or ManualRuntime()
    gateway = BrowserGateway(
        runtime,
        camera=camera,
        semantic=semantic,
        georeference=georeference,
        trajectory_evidence=trajectory_evidence,
    )

    @asynccontextmanager
    async def lifespan(_: FastAPI) -> AsyncIterator[None]:
        await gateway.start()
        try:
            yield
        finally:
            await gateway.stop()

    app = FastAPI(
        title="AeroMind APM Lite Ground Station",
        version="0.1.0",
        lifespan=lifespan,
    )
    app.state.gateway = gateway

    @app.get("/api/health")
    async def health() -> dict[str, Any]:
        return gateway.health_payload()

    @app.get("/api/config")
    async def config() -> dict[str, Any]:
        return gateway.public_config()

    @app.get("/api/status")
    async def status() -> dict[str, Any]:
        return gateway.status_payload()

    @app.get("/api/georeference")
    async def georeference_config() -> dict[str, Any]:
        return gateway.georeference_payload()

    @app.put("/api/georeference")
    async def update_georeference_config(
        payload: GeoReference,
    ) -> dict[str, Any]:
        return await gateway.update_georeference(payload)

    @app.get("/api/trajectory/evidence")
    async def trajectory_evidence_list() -> dict[str, Any]:
        try:
            items = await asyncio.to_thread(
                gateway.trajectory_evidence.list_summaries
            )
        except TrajectoryEvidenceError as exc:
            raise HTTPException(status_code=500, detail=str(exc)) from exc
        return {"items": items}

    @app.post("/api/trajectory/evidence", status_code=201)
    async def save_trajectory_evidence(
        payload: TrajectoryEvidence,
    ) -> dict[str, Any]:
        return await gateway.save_trajectory_evidence(payload)

    @app.get("/api/trajectory/evidence/{evidence_id}")
    async def get_trajectory_evidence(evidence_id: UUID) -> dict[str, Any]:
        return await gateway.load_trajectory_evidence(evidence_id)

    @app.post("/api/trajectory/reports")
    async def create_trajectory_report(
        payload: TrajectoryReportRequest,
    ) -> dict[str, Any]:
        return await gateway.trajectory_report(payload)

    @app.get("/api/serial/ports")
    async def serial_ports() -> dict[str, Any]:
        return {
            "available": (
                gateway.runtime.config.mode == ManualRuntimeMode.REAL_SERIAL
            ),
            "ports": await gateway.available_serial_ports(),
            "selected_port": gateway.runtime.config.ground_serial_port,
            "selected_baudrate": (
                gateway.runtime.config.ground_serial_baudrate
            ),
        }

    @app.put("/api/serial/config")
    async def update_serial_config(
        payload: SerialConnectionRequest,
    ) -> dict[str, Any]:
        return await gateway.reconfigure_serial(
            port=payload.port,
            baudrate=payload.baudrate,
        )

    @app.get("/api/vehicles/{vehicle_id}/telemetry")
    async def vehicle_telemetry(vehicle_id: int) -> dict[str, Any]:
        if vehicle_id != gateway.runtime.config.vehicle_id:
            raise HTTPException(
                status_code=404,
                detail="vehicle is not registered",
            )
        return gateway.telemetry_payload()

    @app.get("/api/vehicle/telemetry")
    async def default_vehicle_telemetry() -> dict[str, Any]:
        return gateway.telemetry_payload()

    @app.get("/api/camera/status")
    async def camera_status() -> dict[str, Any]:
        return gateway.camera_status_payload()

    @app.get("/api/camera/frame")
    async def camera_frame() -> Response:
        try:
            frame = await gateway.camera_frame()
        except CameraUnavailable as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc
        return Response(
            content=frame.data,
            media_type=frame.media_type,
            headers={
                "Cache-Control": (
                    "no-store, no-cache, must-revalidate, max-age=0"
                ),
                "X-Camera-Sequence": str(frame.sequence),
                "X-Camera-Captured-At": frame.captured_at_utc.isoformat(),
            },
        )

    @app.get("/api/semantic/status")
    async def semantic_status() -> dict[str, Any]:
        return gateway.semantic.status_payload()

    @app.get("/api/semantic/latest")
    async def latest_semantic_result() -> dict[str, Any]:
        return gateway.semantic.latest_payload()

    @app.post("/api/semantic/vision/analyze")
    async def analyze_visual_semantics(
        payload: VisualAnalysisRequest | None = Body(default=None),
    ) -> dict[str, Any]:
        try:
            return await gateway.analyze_current_frame(
                (payload or VisualAnalysisRequest()).prompt
            )
        except CameraUnavailable as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc
        except SemanticUnavailable as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc
        except SemanticBusy as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        except (RuntimeError, ValueError) as exc:
            raise HTTPException(status_code=502, detail=str(exc)) from exc

    @app.post("/api/semantic/mission/parse")
    async def parse_mission_semantics(
        payload: MissionParseRequest,
    ) -> dict[str, Any]:
        try:
            return await gateway.parse_mission(payload.instruction)
        except SemanticUnavailable as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc
        except SemanticBusy as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        except (RuntimeError, ValueError) as exc:
            raise HTTPException(status_code=502, detail=str(exc)) from exc

    @app.post("/api/vehicles/{vehicle_id}/commands/{action}")
    async def vehicle_command(
        vehicle_id: int,
        action: BrowserAction,
        payload: CommandRequest | None = Body(default=None),
    ) -> dict[str, Any]:
        return await gateway.issue_command(
            vehicle_id,
            action,
            payload or CommandRequest(),
        )

    control_actions = {
        "arm": BrowserAction.ARM,
        "disarm": BrowserAction.DISARM,
        "takeoff": BrowserAction.TAKEOFF,
        "hold": BrowserAction.HOLD,
        "land": BrowserAction.LAND,
        "return_home": BrowserAction.RTL,
        "rtl": BrowserAction.RTL,
    }

    @app.post("/api/control/{control_action}")
    async def compatibility_command(
        control_action: str,
        payload: CommandRequest | None = Body(default=None),
    ) -> dict[str, Any]:
        action = control_actions.get(control_action)
        if action is None:
            raise HTTPException(
                status_code=404,
                detail="unknown control action",
            )
        return await gateway.issue_command(
            gateway.runtime.config.vehicle_id,
            action,
            payload or CommandRequest(),
        )

    @app.websocket("/ws")
    async def event_stream(websocket: WebSocket) -> None:
        await websocket.accept()
        try:
            status_payload = gateway.status_payload()
            telemetry_payload = gateway.telemetry_payload()
            await websocket.send_json(
                {
                    "event_id": 0,
                    "event_type": "snapshot",
                    "type": "snapshot",
                    "emitted_at_utc": datetime.now(timezone.utc).isoformat(),
                    "payload": {
                        **status_payload,
                        "telemetry": telemetry_payload["telemetry"],
                    },
                    "telemetry": telemetry_payload["telemetry"],
                    **status_payload,
                }
            )
            async with gateway.events.subscribe() as queue:
                while True:
                    event = await queue.get()
                    await websocket.send_json(event)
        except WebSocketDisconnect:
            return

    selected_static_dir = (
        Path(static_dir).resolve()
        if static_dir is not None
        else _default_static_dir()
    )
    if selected_static_dir is not None:
        if not selected_static_dir.is_dir():
            raise ValueError(
                f"static directory does not exist: {selected_static_dir}"
            )
        app.mount(
            "/",
            StaticFiles(directory=selected_static_dir, html=True),
            name="ground-station-web",
        )
    else:

        @app.get("/")
        async def service_root() -> dict[str, Any]:
            return {
                "service": "AeroMind APM Lite Ground Station",
                "api": "/api/status",
                "websocket": "/ws",
            }

    return app


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run against a user-managed SITL/AirSim session"
    )
    parser.add_argument(
        "--mode", choices=("sitl", "demo", "real_serial"), default="sitl"
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--fcu-endpoint", default="udpin:0.0.0.0:14550")
    parser.add_argument("--vehicle-id", type=int, default=1)
    parser.add_argument("--vehicle-name")
    parser.add_argument("--frame-calibration-id")
    parser.add_argument(
        "--serial-port",
        default="COM3",
        help="Initial P9 ground USB serial port (default: COM3).",
    )
    parser.add_argument("--serial-baud", type=int, default=57_600)
    parser.add_argument(
        "--secret-env",
        help=(
            "Environment variable containing the selected vehicle shared "
            "secret."
        ),
    )
    parser.add_argument("--static-dir", type=Path)
    parser.add_argument(
        "--georeference-config",
        type=Path,
        default=_default_georeference_path(),
        help="Active venue GeoReference YAML path.",
    )
    parser.add_argument(
        "--trajectory-evidence-dir",
        type=Path,
        default=_default_trajectory_evidence_dir(),
        help="Directory for immutable planned/predicted/observed evidence.",
    )
    parser.add_argument(
        "--airsim-host",
        help=(
            "AirSim RPC host; defaults to the Windows gateway detected from "
            "WSL."
        ),
    )
    parser.add_argument("--airsim-port", type=int, default=41451)
    parser.add_argument("--airsim-vehicle", default="Drone1")
    parser.add_argument("--airsim-camera", default="front_center")
    parser.add_argument("--camera-fps", type=float, default=15.0)
    parser.add_argument("--camera-timeout", type=float, default=2.0)
    parser.add_argument(
        "--rtsp-url",
        help="Onboard RGB RTSP stream, for example rtsp://192.168.1.109:15544/cam.",
    )
    parser.add_argument(
        "--disable-camera",
        action="store_true",
        help="Run the ground station without a camera bridge.",
    )
    return parser


def _default_airsim_host() -> str:
    try:
        from aeromind_apm_lite.ground.simulation.config import (
            discover_windows_host_ipv4,
        )

        return discover_windows_host_ipv4()
    except (OSError, RuntimeError, ValueError):
        return "127.0.0.1"


def main(argv: list[str] | None = None) -> int:
    args = build_argument_parser().parse_args(argv)
    mode = ManualRuntimeMode(args.mode)
    if mode == ManualRuntimeMode.REAL_SERIAL and not args.secret_env:
        raise SystemExit("--secret-env is required in real_serial mode")
    if mode == ManualRuntimeMode.REAL_SERIAL and not args.frame_calibration_id:
        raise SystemExit(
            "--frame-calibration-id is required in real_serial mode"
        )
    shared_secret = (
        load_shared_secret(args.secret_env)
        if args.secret_env is not None
        else None
    )
    runtime = ManualRuntime(
        ManualRuntimeConfig(
            mode=mode,
            vehicle_id=args.vehicle_id,
            vehicle_name=args.vehicle_name or f"uav{args.vehicle_id}",
            fcu_endpoint=args.fcu_endpoint,
            frame_calibration_id=(
                args.frame_calibration_id or "manual-sim-local-ned-v1"
            ),
            ground_serial_port=args.serial_port,
            ground_serial_baudrate=args.serial_baud,
        ),
        shared_secret=shared_secret,
    )
    camera = None
    if not args.disable_camera and args.rtsp_url:
        camera = RtspCameraBridge(
            RtspCameraConfig(
                stream_url=args.rtsp_url,
                capture_fps=args.camera_fps,
                open_timeout_ms=max(1, int(args.camera_timeout * 1_000)),
                read_timeout_ms=max(1, int(args.camera_timeout * 1_000)),
            )
        )
    elif not args.disable_camera and mode != ManualRuntimeMode.REAL_SERIAL:
        camera = AirSimCameraBridge(
            AirSimCameraConfig(
                rpc_host=args.airsim_host or _default_airsim_host(),
                rpc_port=args.airsim_port,
                vehicle_name=args.airsim_vehicle,
                camera_name=args.airsim_camera,
                capture_fps=args.camera_fps,
                request_timeout_s=args.camera_timeout,
            )
        )
    try:
        georeference = GeoReferenceStore(args.georeference_config)
    except GeoReferenceError as exc:
        raise SystemExit(str(exc)) from exc
    app = create_app(
        runtime,
        camera=camera,
        georeference=georeference,
        trajectory_evidence=TrajectoryEvidenceStore(args.trajectory_evidence_dir),
        static_dir=args.static_dir,
    )
    import uvicorn

    uvicorn.run(app, host=args.host, port=args.port)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

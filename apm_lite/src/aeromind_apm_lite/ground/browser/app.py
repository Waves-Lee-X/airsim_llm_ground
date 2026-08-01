"""FastAPI adapter for the trusted AeroMind APM Lite ground process."""

from __future__ import annotations

import argparse
import asyncio
import hashlib
import math
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
from pydantic import (
    AliasChoices,
    BaseModel,
    ConfigDict,
    Field,
    model_validator,
)
from starlette.websockets import WebSocketDisconnect

from aeromind_apm_lite.common.contracts import (
    AckStatus,
    MissionAck,
    VehicleCommand,
    VehicleCommandType,
)
from aeromind_apm_lite.common.credentials import load_shared_secret
from aeromind_apm_lite.common.coordinates import (
    GeodeticPosition,
    GeoReference,
    GeoReferenceConflict,
    GeoReferenceError,
    GeoReferenceStore,
    geodetic_to_local_ned,
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

from .fleet_executor import (
    FleetMissionConfig,
    FleetMissionError,
    FleetMissionRunner,
)
from .mission_planner import MissionPlanner, PlanStep
from .fleet_service import (
    FleetConfigError,
    FleetService,
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
from .runtime import (
    FleetRuntime,
    HybridRuntime,
    ManualRuntime,
    ManualRuntimeConfig,
    ManualRuntimeMode,
)
from .mission_agent import (
    AgentDraftBlocked,
    AgentDraftConflict,
    AgentDraftExpired,
    AgentDraftNotFound,
    AgentSessionNotFound,
    MissionAgentService,
)
from .semantic import (
    SemanticBusy,
    SemanticService,
    SemanticUnavailable,
)
from aeromind_apm_lite.common.formation import (
    FleetVehicleState,
    FormationType,
)
from .vision_evidence import (
    ColorDetector,
    ColorUnavailable,
    VisionAnalysisEvidence,
    VisionConsensusTracker,
    VisionEvidenceError,
    VisionEvidenceStore,
    VisionFrameInfo,
    cross_validate_color,
)


class BrowserAction(str, Enum):
    ARM = "arm"
    DISARM = "disarm"
    TAKEOFF = "takeoff"
    HOLD = "hold"
    LAND = "land"
    RTL = "rtl"
    GOTO = "goto"


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
    target_position_ned_m: tuple[float, float, float] | None = Field(
        default=None,
        validation_alias=AliasChoices(
            "target_position_ned_m",
            "target_position",
            "position_ned_m",
        ),
    )
    reason: str = Field(default="browser operator command", max_length=256)
    ttl_ms: int | None = Field(default=None, ge=100, le=300_000)
    completion_timeout_s: float = Field(default=75.0, ge=1.0, le=310.0)

    @model_validator(mode="after")
    def command_parameters_are_consistent(self) -> "CommandRequest":
        if self.altitude_m is not None and self.target_position_ned_m is not None:
            raise ValueError(
                "altitude_m and target_position_ned_m cannot be combined"
            )
        if self.target_position_ned_m is not None:
            if not all(
                isinstance(value, (int, float)) and math.isfinite(float(value))
                for value in self.target_position_ned_m
            ):
                raise ValueError("target_position_ned_m must contain finite values")
            if any(
                abs(float(value)) > 100_000.0 for value in self.target_position_ned_m
            ):
                raise ValueError("target_position_ned_m values are out of range")
        return self


class SerialConnectionRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")

    port: str = Field(min_length=1, max_length=256)
    baudrate: int = Field(default=57_600, ge=300, le=4_000_000)


class VisualAnalysisRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")

    prompt: str = Field(default="", max_length=2_000)
    vehicle_id: int | None = Field(default=None, ge=1, le=255)


class MissionParseRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")

    instruction: str = Field(min_length=1, max_length=4_000)


class AgentMessageRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")

    message: str = Field(min_length=1, max_length=4_000)
    vehicle_id: int | None = Field(default=None, ge=1, le=255)


class AgentConfirmationRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")

    confirmed: bool
    vehicle_id: int | None = Field(default=None, ge=1, le=255)


class FleetMissionRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")

    formation: str = Field(pattern="^(line|v|diamond)$")
    formations: list[str] | None = Field(
        default=None,
        min_length=1,
        max_length=8,
        description="Optional formation sequence; falls back to `formation`.",
    )
    leader_target_map_m: tuple[float, float, float]
    spacing_m: float = Field(default=3.0, ge=0.5, le=50.0)
    altitude_m: float = Field(default=2.0, ge=0.5, le=20.0)
    hold_s: float = Field(default=3.0, ge=0.0, le=120.0)
    vehicle_ids: tuple[int, ...] = Field(
        default=(1, 2, 3, 4), min_length=2, max_length=8
    )
    land_after: bool = True

    @model_validator(mode="after")
    def formations_are_valid(self) -> "FleetMissionRequest":
        if self.formations is not None:
            for formation in self.formations:
                if formation not in {"line", "v", "diamond"}:
                    raise ValueError(
                        "formations entries must be line, v or diamond"
                    )
        return self


class FleetFormationRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")

    formation: str = Field(default="line", pattern="^(line|v|diamond)$")
    leader_target_map_m: tuple[float, float, float] | None = None
    spacing_m: float | None = Field(default=None, ge=0.5, le=50.0)
    transition_progress: float | None = Field(default=None, ge=0.0, le=1.0)
    vehicle_ids: tuple[int, ...] | None = Field(default=None, min_length=2, max_length=8)


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
        runtime: ManualRuntime | HybridRuntime,
        *,
        camera: CameraBridge | None = None,
        cameras: Mapping[int, CameraBridge] | None = None,
        semantic: SemanticService | None = None,
        mission_agent: MissionAgentService | None = None,
        georeference: GeoReferenceStore | None = None,
        trajectory_evidence: TrajectoryEvidenceStore | None = None,
        vision_evidence: VisionEvidenceStore | None = None,
        color_detector: ColorDetector | None = None,
        vision_consensus: VisionConsensusTracker | None = None,
        vision_producer_version: str = "aeromind-apm-lite-ground",
        fleet: FleetService | None = None,
        planner_analyzer: Any | None = None,
        telemetry_interval_s: float = 0.2,
    ) -> None:
        if telemetry_interval_s <= 0.0:
            raise ValueError("telemetry_interval_s must be positive")
        self.runtime = runtime
        self.cameras = dict(cameras or {})
        if camera is not None:
            self.cameras.setdefault(runtime.default_vehicle_id, camera)
        self.semantic = semantic or SemanticService()
        self.mission_agent = mission_agent or MissionAgentService(self.semantic)
        self.georeference = georeference or GeoReferenceStore()
        self.trajectory_evidence = trajectory_evidence or TrajectoryEvidenceStore(
            Path.home() / ".aeromind" / "trajectory-evidence"
        )
        self.vision_evidence = vision_evidence or VisionEvidenceStore(
            Path.home() / ".aeromind" / "visual-evidence"
        )
        self.color_detector = color_detector or ColorDetector()
        self.vision_consensus = vision_consensus or VisionConsensusTracker()
        self.vision_producer_version = vision_producer_version
        self.fleet = fleet or FleetService()
        self.fleet_mission = FleetMissionRunner(runtime)
        self.mission_planner = MissionPlanner(runtime)

        def _planner_analyzer(vehicle_id: int, prompt: str):
            # The planner analyzer interface is (vehicle_id, prompt); the
            # frame analyzer takes (prompt, vehicle_id).
            return self.analyze_current_frame(prompt, vehicle_id)

        self.mission_planner.analyzer = (
            planner_analyzer if planner_analyzer is not None else _planner_analyzer
        )
        self.events = EventBroker()
        self._telemetry_interval_s = telemetry_interval_s
        self._telemetry_task: asyncio.Task[None] | None = None
        self._command_tasks: set[asyncio.Task[dict[str, Any]]] = set()
        self._runtime_lock = asyncio.Lock()
        self._georeference_lock = asyncio.Lock()
        self._runtime_errors: dict[int, str] = {}
        self._serial_reconnecting = False

    async def start(self) -> None:
        await asyncio.gather(*(camera.start() for camera in self.cameras.values()))
        try:
            await self.runtime.start()
        except Exception as exc:
            if self.runtime.config.mode != ManualRuntimeMode.REAL_SERIAL:
                await asyncio.gather(
                    *(camera.stop() for camera in self.cameras.values()),
                    return_exceptions=True,
                )
                raise
            self._runtime_errors[self.runtime.default_vehicle_id] = str(exc)
        except BaseException:
            await asyncio.gather(
                *(camera.stop() for camera in self.cameras.values()),
                return_exceptions=True,
            )
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
            await asyncio.gather(
                *(camera.stop() for camera in self.cameras.values()),
                return_exceptions=True,
            )

    def _vehicle_id(self, vehicle_id: int | None = None) -> int:
        selected = self.runtime.default_vehicle_id if vehicle_id is None else vehicle_id
        try:
            self.runtime.config_for(selected)
        except KeyError as exc:
            raise HTTPException(status_code=404, detail="vehicle is not registered") from exc
        return selected

    def _runtime_error(self, vehicle_id: int) -> str | None:
        return self._runtime_errors.get(vehicle_id) or self.runtime.error_for(vehicle_id)

    def public_config(self) -> dict[str, Any]:
        payload = self.runtime.public_config()
        payload["camera"] = self.camera_status_payload(self.runtime.default_vehicle_id)
        for vehicle in payload.get("vehicles", []):
            vehicle_id = int(vehicle["vehicle_id"])
            vehicle["camera"] = self.camera_status_payload(vehicle_id)
        payload["semantic"] = self.semantic.status_payload()
        payload["mission_agent"] = self.mission_agent.status_payload()
        payload["serial_runtime_configurable"] = self.runtime.serial_runtime is not None
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
        serial_runtime = self.runtime.serial_runtime
        if serial_runtime is None:
            raise HTTPException(
                status_code=409,
                detail=(
                    "serial settings require a real_serial vehicle"
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
            vehicle_id = serial_runtime.config.vehicle_id
            old_config = serial_runtime.config
            new_config = replace(
                old_config,
                ground_serial_port=normalized_port,
                ground_serial_baudrate=baudrate,
            )
            self._serial_reconnecting = True
            self._runtime_errors.pop(vehicle_id, None)
            if isinstance(self.runtime, HybridRuntime):
                self.runtime.clear_error(vehicle_id)
            await self.events.publish(
                "service_status", self.status_payload(vehicle_id)
            )
            try:
                await serial_runtime.stop()
                serial_runtime.config = new_config
                try:
                    await serial_runtime.start()
                except Exception as connect_error:
                    serial_runtime.config = old_config
                    try:
                        await serial_runtime.start()
                    except Exception as rollback_error:
                        error = (
                            f"failed to open {normalized_port} at {baudrate}: "
                            f"{connect_error}; rollback failed: "
                            f"{rollback_error}"
                        )
                    else:
                        error = (
                            f"failed to open {normalized_port} at {baudrate}: "
                            f"{connect_error}; restored "
                            f"{old_config.ground_serial_port} at "
                            f"{old_config.ground_serial_baudrate}"
                        )
                    self._runtime_errors[vehicle_id] = error
                    if isinstance(self.runtime, HybridRuntime):
                        self.runtime.set_error(vehicle_id, error)
                    raise HTTPException(
                        status_code=503,
                        detail=error,
                    ) from connect_error
                self._runtime_errors.pop(vehicle_id, None)
                if isinstance(self.runtime, HybridRuntime):
                    self.runtime.clear_error(vehicle_id)
            finally:
                self._serial_reconnecting = False

        payload = {
            "vehicle_id": vehicle_id,
            "port": serial_runtime.config.ground_serial_port,
            "baudrate": serial_runtime.config.ground_serial_baudrate,
            "connected": serial_runtime.started,
            "detail": "serial link opened; waiting for the onboard agent",
        }
        await self.events.publish(
            "service_status", self.status_payload(vehicle_id)
        )
        return payload

    def camera_status_payload(self, vehicle_id: int | None = None) -> dict[str, Any]:
        selected = self._vehicle_id(vehicle_id)
        camera = self.cameras.get(selected)
        if camera is not None:
            payload = camera.status_payload()
            payload["vehicle_id"] = selected
            return payload
        payload = dict(self.runtime.config_for(selected).public_payload()["camera"])
        payload["vehicle_id"] = selected
        payload.setdefault("state", "disabled")
        payload.setdefault("stream_url", "/api/camera/frame")
        return payload

    async def camera_frame(self, vehicle_id: int | None = None) -> CameraFrame:
        selected = self._vehicle_id(vehicle_id)
        camera = self.cameras.get(selected)
        if camera is None:
            raise CameraUnavailable("camera bridge is disabled")
        return await camera.get_frame()

    def health_payload(self) -> dict[str, Any]:
        selected = self.runtime.default_vehicle_id
        status = self.status_payload(selected)
        payload = {
            "status": "ok" if self.runtime.started else "starting",
            "ready": status["vehicle_connected"],
            "runtime_mode": (
                ManualRuntimeMode.HYBRID.value
                if isinstance(self.runtime, HybridRuntime)
                else self.runtime.config.mode.value
            ),
            "vehicle_id": selected,
            "fcu_link_ok": status["fcu_link_ok"],
        }
        if isinstance(self.runtime, HybridRuntime):
            payload["vehicle_count"] = len(self.runtime.vehicle_ids)
        return payload

    def status_payload(self, vehicle_id: int | None = None) -> dict[str, Any]:
        selected = self._vehicle_id(vehicle_id)
        config = self.runtime.config_for(selected)
        runtime_started = self.runtime.started_for(selected)
        vehicle_connected = False
        telemetry = self._link_snapshot_payload(selected)
        heartbeat = None
        session_info = None
        server = None
        if runtime_started:
            server = self.runtime.server_for(selected)
            session_info = server.session_info(selected)
            vehicle_connected = session_info is not None
            if vehicle_connected:
                try:
                    heartbeat = server.last_heartbeat(selected)
                except VehicleNotConnected:
                    heartbeat = None
        serial_stats = (
            getattr(server, "serial_stats", {})
            if server is not None and config.mode == ManualRuntimeMode.REAL_SERIAL
            else {}
        )
        return {
            "runtime_started": runtime_started,
            "runtime_error": self._runtime_error(selected),
            "runtime_mode": config.mode.value,
            "station_mode": (
                ManualRuntimeMode.HYBRID.value
                if isinstance(self.runtime, HybridRuntime)
                else config.mode.value
            ),
            "deployment_mode": (
                "real" if config.mode == ManualRuntimeMode.REAL_SERIAL else "sim"
            ),
            "vehicle_id": selected,
            "vehicle_name": config.vehicle_name,
            "vehicle_connected": vehicle_connected,
            "onboard_agent_connected": (
                self.runtime.agent_connected_for(selected) if runtime_started else False
            ),
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
                else config.mode != ManualRuntimeMode.REAL_SERIAL
            ),
            "allowed_commands": (
                [command.value for command in heartbeat.allowed_commands]
                if heartbeat is not None
                else (
                    [command.value for command in VehicleCommandType]
                    if config.mode != ManualRuntimeMode.REAL_SERIAL
                    else []
                )
            ),
            "fcu_ready": bool(
                (
                    getattr(self.runtime.link_for(selected), "ready", False)
                    and config.mode == ManualRuntimeMode.SITL
                )
                or (
                    config.mode == ManualRuntimeMode.REAL_SERIAL
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
                config.mode == ManualRuntimeMode.DEMO
            ),
            "external_processes_managed": False,
            "ground_link": {
                "transport": (
                    "serial"
                    if config.mode == ManualRuntimeMode.REAL_SERIAL
                    else "loopback_websocket"
                ),
                "stats": serial_stats,
                "port": config.ground_serial_port,
                "baudrate": config.ground_serial_baudrate,
                "reconnecting": (
                    self._serial_reconnecting
                    and config.mode == ManualRuntimeMode.REAL_SERIAL
                ),
                "error": self._runtime_error(selected),
            },
            "camera": self.camera_status_payload(selected),
            "semantic": self.semantic.status_payload(),
            "mission_agent": self.mission_agent.status_payload(),
            "vision": {
                "consensus": self.vision_consensus.consensus_payload(),
                "evidence_available": self.vision_evidence.root.is_dir(),
            },
            "fleet": self.fleet_status_payload(),
        }

    async def analyze_current_frame(
        self,
        prompt: str,
        vehicle_id: int | None = None,
    ) -> dict[str, Any]:
        selected = self._vehicle_id(vehicle_id)
        frame = await self.camera_frame(selected)
        payload = await self.semantic.analyze_frame(frame, prompt)
        payload["vehicle_id"] = selected
        payload["cross_validation"] = self._cross_validate_visual(payload, frame)
        payload["consensus"] = self.vision_consensus.append(payload)
        payload["evidence"] = await self._save_vision_evidence(payload, frame)
        await self.events.publish("visual_semantic_result", payload)
        return payload

    def _cross_validate_visual(
        self,
        payload: dict[str, Any],
        frame: CameraFrame,
    ) -> dict[str, Any]:
        detection = None
        try:
            detection = self.color_detector.detect(
                frame.data,
                sequence=frame.sequence,
                captured_at_utc=frame.captured_at_utc,
                media_type=frame.media_type,
            )
        except ColorUnavailable:
            detection = None
        result = payload.get("result") or {}
        target = result.get("target") or {}
        return cross_validate_color(target.get("color"), detection)

    async def _save_vision_evidence(
        self,
        payload: dict[str, Any],
        frame: CameraFrame,
    ) -> dict[str, Any]:
        try:
            evidence = VisionAnalysisEvidence(
                vehicle_id=payload["vehicle_id"],
                frame=VisionFrameInfo(
                    sequence=frame.sequence,
                    captured_at_utc=frame.captured_at_utc,
                    media_type=frame.media_type,
                    size_bytes=len(frame.data),
                    sha256=hashlib.sha256(frame.data).hexdigest(),
                ),
                vision_model=payload.get("model") or "unknown",
                prompt=payload.get("prompt") or "",
                result=payload.get("result") or {},
                raw_response=payload.get("raw_response") or "",
                cross_validation=payload.get("cross_validation") or {},
                consensus=payload.get("consensus") or {},
                producer_version=self.vision_producer_version,
            )
            saved = await asyncio.to_thread(
                self.vision_evidence.save,
                evidence,
                frame.data,
            )
            return {
                "evidence_id": str(saved.evidence_id),
                "evidence_hash": saved.evidence_hash,
            }
        except VisionEvidenceError as exc:
            return {"error": str(exc)}

    def _fleet_states(self) -> dict[int, FleetVehicleState]:
        states: dict[int, FleetVehicleState] = {}
        for vehicle_id in self.fleet.vehicle_ids:
            try:
                snapshot = self._link_snapshot_payload(vehicle_id)
            except KeyError:
                snapshot = None
            if snapshot is None:
                states[vehicle_id] = FleetVehicleState(vehicle_id=vehicle_id)
                continue
            position = snapshot.get("position_m")
            states[vehicle_id] = FleetVehicleState(
                vehicle_id=vehicle_id,
                ready=bool(position is not None),
                link_ok=True,
                last_seen_age_s=0.0,
                position_map_m=(
                    (
                        float(position["x"]),
                        float(position["y"]),
                        float(position["z"]),
                    )
                    if isinstance(position, dict)
                    else None
                ),
            )
        return states

    def fleet_status_payload(self) -> dict[str, Any]:
        payload = self.fleet.status_payload(self._fleet_states())
        payload["execution"] = self.fleet_mission.status_payload()
        return payload

    async def start_fleet_mission(
        self,
        payload: FleetMissionRequest,
    ) -> dict[str, Any]:
        config = FleetMissionConfig(
            formation=FormationType(payload.formation),
            formations=(
                tuple(FormationType(value) for value in payload.formations)
                if payload.formations is not None
                else None
            ),
            leader_target_map_m=tuple(
                float(value) for value in payload.leader_target_map_m
            ),
            spacing_m=payload.spacing_m,
            altitude_m=payload.altitude_m,
            hold_s=payload.hold_s,
            vehicle_ids=tuple(payload.vehicle_ids),
            land_after=payload.land_after,
        )
        try:
            return await self.fleet_mission.start(config)
        except FleetMissionError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    def fleet_mission_status_payload(self) -> dict[str, Any]:
        return self.fleet_mission.status_payload()

    def fleet_map_payload(self) -> dict[str, Any]:
        """Aggregate the real fleet positions onto one shared north-up map.

        Each FCU reports NED relative to its own home, and in multi-SITL runs
        every vehicle is spawned at a different point, so raw per-vehicle local
        NED would collapse the whole fleet onto a single pixel.  The shared map
        therefore derives every position from WGS84 GPS relative to a stable
        fleet origin (the first online vehicle's home), matching what the
        simulator scene shows.
        """
        vehicles: list[dict[str, Any]] = []
        seen: set[int] = set()
        snapshots: dict[int, tuple[dict[str, Any] | None, str | None]] = {}
        for vehicle_id in (*self.runtime.vehicle_ids, *self.fleet.vehicle_ids):
            if vehicle_id in seen:
                continue
            seen.add(vehicle_id)
            vehicle_name: str | None = None
            try:
                snapshot = self._link_snapshot_payload(vehicle_id)
                vehicle_name = self.runtime.config_for(vehicle_id).vehicle_name
            except KeyError:
                snapshot = None
            snapshots[vehicle_id] = (snapshot, vehicle_name)

        origin_deg_m: list[float] | None = None
        origin: GeodeticPosition | None = None
        for snapshot, _name in snapshots.values():
            if snapshot is None:
                continue
            candidate = snapshot.get("home_position_deg_m") or snapshot.get(
                "global_position_deg_m"
            )
            if (
                isinstance(candidate, (list, tuple))
                and len(candidate) >= 3
                and all(isinstance(value, (int, float)) for value in candidate[:3])
            ):
                origin = GeodeticPosition(
                    float(candidate[0]),
                    float(candidate[1]),
                    float(candidate[2]),
                )
                origin_deg_m = [
                    origin.latitude_deg,
                    origin.longitude_deg,
                    origin.altitude_m,
                ]
                break

        for vehicle_id, (snapshot, vehicle_name) in snapshots.items():
            if snapshot is None:
                vehicles.append(
                    {
                        "vehicle_id": vehicle_id,
                        "vehicle_name": vehicle_name,
                        "available": False,
                    }
                )
                continue
            position = snapshot.get("position_m")
            velocity = snapshot.get("velocity_m_s")
            global_position = snapshot.get("global_position_deg_m")
            shared_position: dict[str, float] | None = None
            if (
                origin is not None
                and isinstance(global_position, (list, tuple))
                and len(global_position) >= 3
                and all(
                    isinstance(value, (int, float)) for value in global_position[:3]
                )
            ):
                shared_ned = geodetic_to_local_ned(
                    GeodeticPosition(
                        float(global_position[0]),
                        float(global_position[1]),
                        float(global_position[2]),
                    ),
                    origin,
                )
                shared_position = {
                    "x": float(shared_ned.x),
                    "y": float(shared_ned.y),
                    "z": float(shared_ned.z),
                }
            if shared_position is None and isinstance(position, dict):
                shared_position = {
                    "x": float(position["x"]),
                    "y": float(position["y"]),
                    "z": float(position["z"]),
                }
            yaw_rad = None
            attitude = snapshot.get("attitude_rpy_rad")
            if (
                isinstance(attitude, (list, tuple))
                and len(attitude) >= 3
                and all(isinstance(value, (int, float)) for value in attitude[:3])
            ):
                yaw_rad = float(attitude[2])
            field_ages = snapshot.get("field_ages_s")
            vehicles.append(
                {
                    "vehicle_id": vehicle_id,
                    "vehicle_name": vehicle_name,
                    "available": True,
                    "fcu_link_ok": bool(snapshot.get("fcu_link_ok")),
                    "mode": snapshot.get("mode"),
                    "armed": bool(snapshot.get("armed")),
                    "position_m": shared_position,
                    "local_position_ned_m": (
                        {
                            "x": float(position["x"]),
                            "y": float(position["y"]),
                            "z": float(position["z"]),
                        }
                        if isinstance(position, dict)
                        else None
                    ),
                    "global_position_deg_m": (
                        [float(value) for value in global_position[:3]]
                        if isinstance(global_position, (list, tuple))
                        and len(global_position) >= 3
                        else None
                    ),
                    "velocity_m_s": (
                        {
                            "x": float(velocity["x"]),
                            "y": float(velocity["y"]),
                            "z": float(velocity["z"]),
                        }
                        if isinstance(velocity, dict)
                        else None
                    ),
                    "relative_altitude_m": snapshot.get("relative_altitude_m"),
                    "landed_state": snapshot.get("landed_state"),
                    "heading_deg": (
                        round(math.degrees(yaw_rad), 1)
                        if yaw_rad is not None
                        else None
                    ),
                    "gps_fix_type": snapshot.get("gps_fix_type"),
                    "satellites_visible": snapshot.get("satellites_visible"),
                    "heartbeat_age_s": (
                        field_ages.get("heartbeat")
                        if isinstance(field_ages, Mapping)
                        else None
                    ),
                }
            )
        return {
            "coordinate_frame": "shared_ned",
            "origin_deg_m": origin_deg_m,
            "generated_monotonic_s": time.monotonic(),
            "vehicles": vehicles,
            "formation": {
                "formation": self.fleet.formation.value,
                "leader_target_map_m": list(self.fleet.leader_target_map_m),
                "spacing_m": self.fleet.spacing_m,
                "vehicle_ids": list(self.fleet.vehicle_ids),
            },
        }

    def set_fleet_formation(
        self,
        payload: FleetFormationRequest,
    ) -> dict[str, Any]:
        return self.fleet.set_formation(
            FormationType(payload.formation),
            leader_target_map_m=payload.leader_target_map_m,
            spacing_m=payload.spacing_m,
            transition_progress=payload.transition_progress,
            vehicle_ids=payload.vehicle_ids,
        )

    async def parse_mission(self, instruction: str) -> dict[str, Any]:
        payload = await self.semantic.parse_mission(instruction)
        await self.events.publish("mission_parse_result", payload)
        return payload

    def mission_agent_context(
        self,
        vehicle_id: int | None = None,
    ) -> dict[str, Any]:
        selected = self._vehicle_id(vehicle_id)
        config = self.runtime.config_for(selected)
        status = self.status_payload(selected)
        telemetry = status.get("telemetry")
        if not isinstance(telemetry, dict):
            telemetry = {}
        health = telemetry.get("health")
        if not isinstance(health, dict):
            health = {}
        selected_telemetry = {
            key: telemetry.get(key)
            for key in (
                "armed",
                "mode",
                "battery_remaining",
                "battery_voltage_v",
                "local_position_ned_m",
                "velocity_ned_m_s",
                "global_position_deg_m",
                "home_position_deg_m",
                "landed_state",
                "last_status_text",
            )
        }
        selected_telemetry["health"] = {
            key: health.get(key)
            for key in (
                "fcu_link_ok",
                "gps_fix_type",
                "gps_hdop",
                "gps_healthy",
                "prearm_ok",
                "ekf_ok",
            )
        }
        latest_visual = self.semantic.latest_payload().get("visual")
        visual_result = (
            latest_visual.get("result")
            if isinstance(latest_visual, dict)
            else None
        )
        reference = self.georeference.value
        fleet_vehicles: list[dict[str, Any]] = []
        for fleet_vehicle_id in self.runtime.vehicle_ids:
            try:
                fleet_snapshot = self._link_snapshot_payload(fleet_vehicle_id)
            except KeyError:
                fleet_snapshot = None
            if fleet_snapshot is None:
                fleet_vehicles.append(
                    {
                        "vehicle_id": fleet_vehicle_id,
                        "available": False,
                    }
                )
                continue
            fleet_vehicles.append(
                {
                    "vehicle_id": fleet_vehicle_id,
                    "vehicle_name": (
                        fleet_snapshot.get("vehicle_name")
                        or self.runtime.config_for(fleet_vehicle_id).vehicle_name
                    ),
                    "available": True,
                    "fcu_link_ok": bool(fleet_snapshot.get("fcu_link_ok")),
                    "mode": fleet_snapshot.get("mode"),
                    "armed": bool(fleet_snapshot.get("armed")),
                    "position_m": fleet_snapshot.get("position_m"),
                }
            )
        return {
            "fleet": {"vehicles": fleet_vehicles},
            "mission_planner": self.mission_planner.status_payload(),
            "captured_at_utc": datetime.now(timezone.utc).isoformat(),
            "vehicle_id": selected,
            "vehicle_name": config.vehicle_name,
            "deployment_mode": (
                "real"
                if config.mode == ManualRuntimeMode.REAL_SERIAL
                else config.mode.value
            ),
            "agent_connected": bool(
                status.get("onboard_agent_connected")
                or status.get("vehicle_connected")
            ),
            "fcu_link_ok": status.get("fcu_link_ok") is True,
            "command_output_enabled": status.get("command_output_enabled") is True,
            "allowed_commands": status.get("allowed_commands", []),
            "telemetry": selected_telemetry,
            "ground_link": {
                "transport": status.get("ground_link", {}).get("transport"),
                "reconnecting": status.get("ground_link", {}).get("reconnecting"),
                "error": status.get("ground_link", {}).get("error"),
            },
            "camera": self.camera_status_payload(selected),
            "latest_visual_result": visual_result,
            "georeference": {
                "calibration_id": reference.calibration_id,
                "status": reference.status.value,
                "complete": reference.is_complete,
                "config_hash": reference.config_hash,
            },
        }

    async def chat_with_mission_agent(
        self,
        session_id: UUID,
        message: str,
        vehicle_id: int | None = None,
    ) -> dict[str, Any]:
        payload = await self.mission_agent.chat(
            session_id,
            message,
            self.mission_agent_context(vehicle_id),
        )
        await self.events.publish("mission_agent_reply", payload)
        return payload

    async def confirm_agent_draft(
        self,
        draft_id: UUID,
        vehicle_id: int | None = None,
    ) -> dict[str, Any]:
        claim = self.mission_agent.claim_draft(
            draft_id,
            self.mission_agent_context(vehicle_id),
        )
        action = str(claim["action"]).lower()
        arguments = claim["arguments"] or {}
        reason = (
            f"operator-confirmed Mission Agent: {claim['reason']}"
        )[:256]
        if action == "plan":
            steps = [
                PlanStep(
                    action=str(step["action"]).lower(),
                    vehicle_ids=tuple(int(value) for value in step["vehicle_ids"]),
                    params=dict(step.get("arguments") or {}),
                )
                for step in arguments.get("steps", [])
            ]
            try:
                command = await self.mission_planner.start(steps)
            except FleetMissionError as exc:
                self.mission_agent.record_execution(
                    draft_id,
                    succeeded=False,
                    result={"detail": str(exc)},
                )
                raise HTTPException(status_code=409, detail=str(exc)) from exc
            except Exception as exc:
                detail = exc.detail if isinstance(exc, HTTPException) else str(exc)
                self.mission_agent.record_execution(
                    draft_id,
                    succeeded=False,
                    result={"detail": str(detail)},
                )
                raise
        elif action == "goto":
            request = CommandRequest(
                target_position_ned_m=tuple(arguments["target_position_ned_m"]),
                reason=reason,
            )
            try:
                command = await self.issue_command(
                    claim["vehicle_id"],
                    BrowserAction.GOTO,
                    request,
                )
            except Exception as exc:
                detail = exc.detail if isinstance(exc, HTTPException) else str(exc)
                self.mission_agent.record_execution(
                    draft_id,
                    succeeded=False,
                    result={"detail": str(detail)},
                )
                raise
        elif action == "formation":
            payload = FleetMissionRequest(
                formation=arguments["formation"],
                leader_target_map_m=tuple(arguments["leader_target_map_m"]),
                spacing_m=arguments["spacing_m"],
                altitude_m=arguments["altitude_m"],
                hold_s=arguments["hold_s"],
                vehicle_ids=tuple(arguments["vehicle_ids"]),
            )
            try:
                command = await self.start_fleet_mission(payload)
            except Exception as exc:
                detail = exc.detail if isinstance(exc, HTTPException) else str(exc)
                self.mission_agent.record_execution(
                    draft_id,
                    succeeded=False,
                    result={"detail": str(detail)},
                )
                raise
        else:
            request = CommandRequest(
                altitude_m=arguments.get("altitude_m"),
                reason=reason,
            )
            try:
                command = await self.issue_command(
                    claim["vehicle_id"],
                    BrowserAction(action),
                    request,
                )
            except Exception as exc:
                detail = exc.detail if isinstance(exc, HTTPException) else str(exc)
                self.mission_agent.record_execution(
                    draft_id,
                    succeeded=False,
                    result={"detail": str(detail)},
                )
                raise
        draft = self.mission_agent.record_execution(
            draft_id,
            succeeded=command.get("successful") is True,
            result=command,
        )
        payload = {
            "kind": "mission_agent_execution",
            "draft": draft,
            "command_result": command,
        }
        await self.events.publish("mission_agent_execution", payload)
        return payload

    async def cancel_agent_draft(self, draft_id: UUID) -> dict[str, Any]:
        payload = self.mission_agent.cancel_draft(draft_id)
        await self.events.publish("mission_agent_draft_updated", payload)
        return payload

    def telemetry_payload(self, vehicle_id: int | None = None) -> dict[str, Any]:
        selected = self._vehicle_id(vehicle_id)
        telemetry = self._link_snapshot_payload(selected)
        return {
            "available": telemetry is not None,
            "vehicle_id": selected,
            "coordinate_frame": "local_ned",
            "telemetry": telemetry,
        }

    def _link_snapshot_payload(self, vehicle_id: int) -> dict[str, Any] | None:
        config = self.runtime.config_for(vehicle_id)
        link = self.runtime.link_for(vehicle_id)
        if link is None:
            if (
                not self.runtime.started_for(vehicle_id)
                or config.mode != ManualRuntimeMode.REAL_SERIAL
            ):
                return None
            try:
                telemetry = self.runtime.server_for(vehicle_id).latest_telemetry(
                    vehicle_id
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
        selected = self._vehicle_id(vehicle_id)
        config = self.runtime.config_for(selected)
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
        if action == BrowserAction.GOTO and request.target_position_ned_m is None:
            raise HTTPException(
                status_code=422,
                detail="goto requires target_position_ned_m",
            )
        if action != BrowserAction.GOTO and request.target_position_ned_m is not None:
            raise HTTPException(
                status_code=422,
                detail="target_position_ned_m is only valid for goto",
            )
        if not self.runtime.started_for(selected):
            raise HTTPException(status_code=503, detail="runtime is not ready")

        command_type = VehicleCommandType(action.value)
        server = self.runtime.server_for(selected)
        if config.mode == ManualRuntimeMode.REAL_SERIAL:
            try:
                heartbeat = server.last_heartbeat(selected)
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
            command = await server.send_command(
                selected,
                command_type,
                target_altitude_m=request.altitude_m,
                target_position_ned_m=request.target_position_ned_m,
                reason=request.reason,
                ttl_ms=request.ttl_ms or config.command_ttl_ms,
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
                    config.mode == ManualRuntimeMode.DEMO
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
        server = self.runtime.server_for(command.vehicle_id)
        try:
            accepted = await server.wait_for_ack(
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
        ack = await self.runtime.server_for(command.vehicle_id).wait_for_ack(
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
                    self.runtime.config_for(command.vehicle_id).mode
                    == ManualRuntimeMode.DEMO
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
        config = self.runtime.config_for(command.vehicle_id)
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
            "applicable": config.mode != ManualRuntimeMode.DEMO,
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
            "simulated": config.mode == ManualRuntimeMode.DEMO,
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
            for vehicle_id in self.runtime.vehicle_ids:
                payload = self.telemetry_payload(vehicle_id)
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


def _default_vision_evidence_dir() -> Path:
    return Path.home() / ".aeromind" / "visual-evidence"


def create_app(
    runtime: ManualRuntime | HybridRuntime | FleetRuntime | None = None,
    *,
    camera: CameraBridge | None = None,
    cameras: Mapping[int, CameraBridge] | None = None,
    semantic: SemanticService | None = None,
    mission_agent: MissionAgentService | None = None,
    georeference: GeoReferenceStore | None = None,
    trajectory_evidence: TrajectoryEvidenceStore | None = None,
    vision_evidence: VisionEvidenceStore | None = None,
    color_detector: ColorDetector | None = None,
    vision_consensus: VisionConsensusTracker | None = None,
    fleet: FleetService | None = None,
    planner_analyzer: Any | None = None,
    static_dir: str | Path | None = None,
) -> FastAPI:
    runtime = runtime or ManualRuntime()
    gateway = BrowserGateway(
        runtime,
        camera=camera,
        cameras=cameras,
        semantic=semantic,
        mission_agent=mission_agent,
        georeference=georeference,
        trajectory_evidence=trajectory_evidence,
        vision_evidence=vision_evidence,
        color_detector=color_detector,
        vision_consensus=vision_consensus,
        fleet=fleet,
        planner_analyzer=planner_analyzer,
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
    async def status(vehicle_id: int | None = None) -> dict[str, Any]:
        return gateway.status_payload(vehicle_id)

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
        serial_runtime = gateway.runtime.serial_runtime
        return {
            "available": serial_runtime is not None,
            "ports": await gateway.available_serial_ports(),
            "vehicle_id": (
                serial_runtime.config.vehicle_id if serial_runtime is not None else None
            ),
            "selected_port": (
                serial_runtime.config.ground_serial_port
                if serial_runtime is not None
                else None
            ),
            "selected_baudrate": (
                serial_runtime.config.ground_serial_baudrate
                if serial_runtime is not None
                else None
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
        return gateway.telemetry_payload(vehicle_id)

    @app.get("/api/vehicle/telemetry")
    async def default_vehicle_telemetry() -> dict[str, Any]:
        return gateway.telemetry_payload(gateway.runtime.default_vehicle_id)

    @app.get("/api/camera/status")
    async def camera_status(vehicle_id: int | None = None) -> dict[str, Any]:
        return gateway.camera_status_payload(vehicle_id)

    @app.get("/api/camera/frame")
    async def camera_frame(vehicle_id: int | None = None) -> Response:
        try:
            frame = await gateway.camera_frame(vehicle_id)
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
        request = payload or VisualAnalysisRequest()
        try:
            return await gateway.analyze_current_frame(
                request.prompt,
                request.vehicle_id,
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

    @app.post("/api/vision/crosscheck")
    async def crosscheck_current_frame(
        request: VisualAnalysisRequest,
    ) -> dict[str, Any]:
        selected = gateway._vehicle_id(request.vehicle_id)
        frame = await gateway.camera_frame(selected)
        detection = None
        detector_error = None
        try:
            detection = gateway.color_detector.detect(
                frame.data,
                sequence=frame.sequence,
                captured_at_utc=frame.captured_at_utc,
                media_type=frame.media_type,
            )
        except ColorUnavailable as exc:
            detector_error = str(exc)
        latest = gateway.semantic.latest_payload().get("visual")
        vlm_color = None
        vlm_analyzed = isinstance(latest, dict)
        if vlm_analyzed:
            target = (latest.get("result") or {}).get("target") or {}
            vlm_color = target.get("color")
        cross = cross_validate_color(
            vlm_color,
            detection,
            vlm_analyzed=vlm_analyzed,
        )
        return {
            "vehicle_id": selected,
            "frame": {
                "sequence": frame.sequence,
                "captured_at_utc": frame.captured_at_utc.isoformat(),
            },
            "cross_validation": cross,
            "detector_error": detector_error,
            "consensus": gateway.vision_consensus.consensus_payload(),
        }

    @app.get("/api/vision/evidence")
    async def vision_evidence_list() -> dict[str, Any]:
        try:
            evidence = await asyncio.to_thread(
                gateway.vision_evidence.list_summaries
            )
        except VisionEvidenceError as exc:
            raise HTTPException(status_code=500, detail=str(exc)) from exc
        return {"evidence": evidence}

    @app.get("/api/vision/evidence/{evidence_id}")
    async def get_vision_evidence(evidence_id: UUID) -> dict[str, Any]:
        try:
            evidence = await asyncio.to_thread(
                gateway.vision_evidence.load,
                evidence_id,
            )
        except VisionEvidenceError as exc:
            raise HTTPException(status_code=404, detail=str(exc)) from exc
        return evidence.public_payload()

    @app.get("/api/vision/evidence/{evidence_id}/frame")
    async def get_vision_evidence_frame(evidence_id: UUID) -> Response:
        try:
            evidence = await asyncio.to_thread(
                gateway.vision_evidence.load,
                evidence_id,
            )
            frame_data = await asyncio.to_thread(
                gateway.vision_evidence.load_frame,
                evidence_id,
            )
        except VisionEvidenceError as exc:
            raise HTTPException(status_code=404, detail=str(exc)) from exc
        if frame_data is None:
            raise HTTPException(
                status_code=404,
                detail="frame snapshot is not stored",
            )
        return Response(
            content=frame_data,
            media_type=evidence.frame.media_type,
        )

    @app.get("/api/fleet/status")
    async def fleet_status() -> dict[str, Any]:
        return gateway.fleet_status_payload()

    @app.get("/api/fleet/map")
    async def fleet_map() -> dict[str, Any]:
        return gateway.fleet_map_payload()

    @app.post("/api/fleet/execute")
    async def fleet_execute(
        payload: FleetMissionRequest,
    ) -> dict[str, Any]:
        return await gateway.start_fleet_mission(payload)

    @app.get("/api/fleet/execute")
    async def fleet_execute_status() -> dict[str, Any]:
        return gateway.fleet_mission_status_payload()

    @app.post("/api/fleet/execute/cancel")
    async def fleet_execute_cancel() -> dict[str, Any]:
        return await gateway.fleet_mission.cancel()

    @app.get("/api/planner/status")
    async def planner_status() -> dict[str, Any]:
        return gateway.mission_planner.status_payload()

    @app.post("/api/planner/cancel")
    async def planner_cancel() -> dict[str, Any]:
        return await gateway.mission_planner.cancel()

    @app.post("/api/fleet/formation")
    async def set_fleet_formation(
        payload: FleetFormationRequest,
    ) -> dict[str, Any]:
        try:
            return gateway.set_fleet_formation(payload)
        except (FleetConfigError, ValueError) as exc:
            raise HTTPException(status_code=422, detail=str(exc)) from exc

    @app.get("/api/agent/status")
    async def mission_agent_status() -> dict[str, Any]:
        return gateway.mission_agent.status_payload()

    @app.post("/api/agent/sessions")
    async def create_mission_agent_session() -> dict[str, Any]:
        return gateway.mission_agent.create_session()

    @app.get("/api/agent/sessions/{session_id}")
    async def mission_agent_session(session_id: UUID) -> dict[str, Any]:
        try:
            return gateway.mission_agent.session_payload(session_id)
        except AgentSessionNotFound as exc:
            raise HTTPException(status_code=404, detail=str(exc)) from exc

    @app.delete("/api/agent/sessions/{session_id}")
    async def close_mission_agent_session(session_id: UUID) -> dict[str, Any]:
        try:
            return gateway.mission_agent.close_session(session_id)
        except AgentSessionNotFound as exc:
            raise HTTPException(status_code=404, detail=str(exc)) from exc

    @app.post("/api/agent/sessions/{session_id}/messages")
    async def send_mission_agent_message(
        session_id: UUID,
        payload: AgentMessageRequest,
    ) -> dict[str, Any]:
        try:
            return await gateway.chat_with_mission_agent(
                session_id,
                payload.message,
                payload.vehicle_id,
            )
        except AgentSessionNotFound as exc:
            raise HTTPException(status_code=404, detail=str(exc)) from exc
        except SemanticUnavailable as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc
        except SemanticBusy as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        except (RuntimeError, ValueError) as exc:
            raise HTTPException(status_code=502, detail=str(exc)) from exc

    @app.post("/api/agent/drafts/{draft_id}/confirm")
    async def confirm_mission_agent_draft(
        draft_id: UUID,
        payload: AgentConfirmationRequest,
    ) -> dict[str, Any]:
        if payload.confirmed is not True:
            raise HTTPException(status_code=422, detail="必须明确确认任务草案")
        try:
            return await gateway.confirm_agent_draft(draft_id, payload.vehicle_id)
        except AgentDraftNotFound as exc:
            raise HTTPException(status_code=404, detail=str(exc)) from exc
        except AgentDraftExpired as exc:
            raise HTTPException(status_code=410, detail=str(exc)) from exc
        except (AgentDraftBlocked, AgentDraftConflict) as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    @app.post("/api/agent/drafts/{draft_id}/cancel")
    async def cancel_mission_agent_draft(draft_id: UUID) -> dict[str, Any]:
        try:
            return await gateway.cancel_agent_draft(draft_id)
        except AgentDraftNotFound as exc:
            raise HTTPException(status_code=404, detail=str(exc)) from exc
        except AgentDraftConflict as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

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
            gateway.runtime.default_vehicle_id,
            action,
            payload or CommandRequest(),
        )

    @app.websocket("/ws")
    async def event_stream(websocket: WebSocket) -> None:
        await websocket.accept()
        try:
            selected = gateway.runtime.default_vehicle_id
            status_payload = gateway.status_payload(selected)
            telemetry_payload = gateway.telemetry_payload(selected)
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
        description="Run the Lite Web ground station in simulation, real, or hybrid mode"
    )
    parser.add_argument(
        "--mode",
        choices=("sitl", "demo", "real_serial", "hybrid"),
        default="sitl",
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--fcu-endpoint", default="udpin:0.0.0.0:14550")
    parser.add_argument("--vehicle-id", type=int, default=1)
    parser.add_argument("--vehicle-name")
    parser.add_argument("--sim-vehicle-id", type=int, default=1)
    parser.add_argument("--sim-vehicle-count", type=int, default=1)
    parser.add_argument(
        "--sim-vehicle-ids",
        default="",
        help="Comma-separated ground vehicle ids for the sim fleet.",
    )
    parser.add_argument(
        "--sim-sysids",
        default="",
        help="Comma-separated MAVLink system ids for the sim fleet.",
    )
    parser.add_argument("--sim-vehicle-name", default="SITL UAV 1")
    parser.add_argument("--real-vehicle-id", type=int, default=3)
    parser.add_argument("--real-vehicle-name", default="实机 UAV 3")
    parser.add_argument(
        "--sim-frame-calibration-id",
        default="manual-sim-local-ned-v1",
    )
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
        "--vision-evidence-dir",
        type=Path,
        default=_default_vision_evidence_dir(),
        help="Directory for immutable visual-analysis evidence records.",
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
        help="Onboard RGB RTSP stream, for example rtsp://192.168.1.110:15544/cam.",
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
    real_bridge_enabled = mode == ManualRuntimeMode.REAL_SERIAL or (
        mode == ManualRuntimeMode.HYBRID
        and bool(args.real_vehicle_id)
        and args.real_vehicle_id > 0
    )
    if real_bridge_enabled and not args.secret_env:
        raise SystemExit(
            "--secret-env is required when a real serial bridge is enabled"
        )
    if real_bridge_enabled and not args.frame_calibration_id:
        raise SystemExit(
            "--frame-calibration-id is required when a real serial bridge is enabled"
        )
    shared_secret = (
        load_shared_secret(args.secret_env)
        if args.secret_env is not None
        else None
    )
    if mode == ManualRuntimeMode.HYBRID:
        try:
            sim_runtime_count = max(1, args.sim_vehicle_count)
            if args.sim_vehicle_ids and args.sim_sysids:
                sim_vehicle_ids = [
                    int(value) for value in args.sim_vehicle_ids.split(",")
                ]
                sim_sysids = [int(value) for value in args.sim_sysids.split(",")]
                if len(sim_vehicle_ids) != len(sim_sysids):
                    raise SystemExit(
                        "--sim-vehicle-ids and --sim-sysids must have equal length"
                    )
                sim_runtime_count = len(sim_vehicle_ids)
            else:
                sim_vehicle_ids = [
                    args.sim_vehicle_id + index for index in range(sim_runtime_count)
                ]
                sim_sysids = list(sim_vehicle_ids)
            sim_runtimes = [
                ManualRuntime(
                    ManualRuntimeConfig(
                        mode=ManualRuntimeMode.SITL,
                        vehicle_id=sim_vehicle_ids[index],
                        mavlink_target_system=sim_sysids[index],
                        vehicle_name=(
                            f"SITL UAV {sim_sysids[index]}"
                            if sim_runtime_count > 1
                            else args.sim_vehicle_name
                        ),
                        fcu_endpoint=(
                            f"udpin:0.0.0.0:{14550 + 10 * index}"
                            if sim_runtime_count > 1
                            else args.fcu_endpoint
                        ),
                        frame_calibration_id=args.sim_frame_calibration_id,
                    )
                )
                for index in range(sim_runtime_count)
            ]
            real_runtime = (
                ManualRuntime(
                    ManualRuntimeConfig(
                        mode=ManualRuntimeMode.REAL_SERIAL,
                        vehicle_id=args.real_vehicle_id,
                        vehicle_name=args.real_vehicle_name,
                        frame_calibration_id=args.frame_calibration_id,
                        ground_serial_port=args.serial_port,
                        ground_serial_baudrate=args.serial_baud,
                    ),
                    shared_secret=shared_secret,
                )
                if args.real_vehicle_id and args.real_vehicle_id > 0
                else None
            )
            runtime: ManualRuntime | HybridRuntime | FleetRuntime = FleetRuntime(
                sim_runtimes,
                real_runtime,
                default_vehicle_id=args.sim_vehicle_id,
            )
        except ValueError as exc:
            raise SystemExit(str(exc)) from exc
    else:
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
    cameras: dict[int, CameraBridge] = {}
    if not args.disable_camera and mode in {
        ManualRuntimeMode.SITL,
        ManualRuntimeMode.DEMO,
        ManualRuntimeMode.HYBRID,
    }:
        if mode == ManualRuntimeMode.HYBRID and (
            args.sim_vehicle_count > 1 or args.sim_vehicle_ids
        ):
            for sim_index in range(sim_runtime_count):
                vehicle_id = sim_vehicle_ids[sim_index]
                cameras[vehicle_id] = AirSimCameraBridge(
                    AirSimCameraConfig(
                        rpc_host=args.airsim_host or _default_airsim_host(),
                        rpc_port=args.airsim_port,
                        vehicle_name=f"Drone{sim_index + 1}",
                        camera_name=args.airsim_camera,
                        capture_fps=args.camera_fps,
                        request_timeout_s=args.camera_timeout,
                    )
                )
        else:
            simulation_vehicle_id = (
                args.sim_vehicle_id
                if mode == ManualRuntimeMode.HYBRID
                else args.vehicle_id
            )
            cameras[simulation_vehicle_id] = AirSimCameraBridge(
                AirSimCameraConfig(
                    rpc_host=args.airsim_host or _default_airsim_host(),
                    rpc_port=args.airsim_port,
                    vehicle_name=args.airsim_vehicle,
                    camera_name=args.airsim_camera,
                    capture_fps=args.camera_fps,
                    request_timeout_s=args.camera_timeout,
                )
            )
    if not args.disable_camera and args.rtsp_url:
        real_vehicle_id = (
            args.real_vehicle_id
            if mode == ManualRuntimeMode.HYBRID
            else args.vehicle_id
        )
        cameras[real_vehicle_id] = RtspCameraBridge(
            RtspCameraConfig(
                stream_url=args.rtsp_url,
                capture_fps=args.camera_fps,
                open_timeout_ms=max(1, int(args.camera_timeout * 1_000)),
                read_timeout_ms=max(1, int(args.camera_timeout * 1_000)),
            )
        )
    try:
        georeference = GeoReferenceStore(args.georeference_config)
    except GeoReferenceError as exc:
        raise SystemExit(str(exc)) from exc
    app = create_app(
        runtime,
        cameras=cameras,
        georeference=georeference,
        trajectory_evidence=TrajectoryEvidenceStore(args.trajectory_evidence_dir),
        vision_evidence=VisionEvidenceStore(args.vision_evidence_dir),
        static_dir=args.static_dir,
    )
    import uvicorn

    uvicorn.run(app, host=args.host, port=args.port)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

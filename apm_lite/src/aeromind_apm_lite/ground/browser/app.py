"""FastAPI adapter for the trusted AeroMind APM Lite ground process."""

from __future__ import annotations

import argparse
import asyncio
from contextlib import asynccontextmanager
from dataclasses import fields
from datetime import datetime, timezone
from enum import Enum
from pathlib import Path
from collections.abc import Mapping
from typing import Any, AsyncIterator
from fastapi import Body, FastAPI, HTTPException, WebSocket
from fastapi.staticfiles import StaticFiles
from pydantic import AliasChoices, BaseModel, ConfigDict, Field
from starlette.websockets import WebSocketDisconnect

from aeromind_apm_lite.common.contracts import (
    AckStatus,
    MissionAck,
    VehicleCommand,
    VehicleCommandType,
)
from aeromind_apm_lite.ground.server import VehicleNotConnected

from .runtime import ManualRuntime, ManualRuntimeConfig, ManualRuntimeMode


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
        telemetry_interval_s: float = 0.2,
    ) -> None:
        if telemetry_interval_s <= 0.0:
            raise ValueError("telemetry_interval_s must be positive")
        self.runtime = runtime
        self.events = EventBroker()
        self._telemetry_interval_s = telemetry_interval_s
        self._telemetry_task: asyncio.Task[None] | None = None
        self._command_tasks: set[asyncio.Task[dict[str, Any]]] = set()

    async def start(self) -> None:
        await self.runtime.start()
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
        await self.runtime.stop()

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
        if self.runtime.started:
            vehicle_connected = (
                self.runtime.server.session_info(
                    self.runtime.config.vehicle_id
                )
                is not None
            )
        return {
            "runtime_started": self.runtime.started,
            "runtime_mode": self.runtime.config.mode.value,
            "vehicle_id": self.runtime.config.vehicle_id,
            "vehicle_connected": vehicle_connected,
            "onboard_agent_connected": self.runtime.agent_connected,
            "fcu_ready": bool(
                getattr(self.runtime.link, "ready", False)
                and self.runtime.config.mode == ManualRuntimeMode.SITL
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
        }

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
            return None
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
            "gps_healthy": snapshot.gps_healthy,
            "prearm_ok": snapshot.prearm_ok,
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
            "applicable": self.runtime.config.mode == ManualRuntimeMode.SITL,
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


def create_app(
    runtime: ManualRuntime | None = None,
    *,
    static_dir: str | Path | None = None,
) -> FastAPI:
    runtime = runtime or ManualRuntime()
    gateway = BrowserGateway(runtime)

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
        return gateway.runtime.public_config()

    @app.get("/api/status")
    async def status() -> dict[str, Any]:
        return gateway.status_payload()

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
        return gateway.runtime.public_config()["camera"]

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
    parser.add_argument("--mode", choices=("sitl", "demo"), default="sitl")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--fcu-endpoint", default="udpin:0.0.0.0:14550")
    parser.add_argument("--vehicle-id", type=int, default=1)
    parser.add_argument("--static-dir", type=Path)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_argument_parser().parse_args(argv)
    runtime = ManualRuntime(
        ManualRuntimeConfig(
            mode=ManualRuntimeMode(args.mode),
            vehicle_id=args.vehicle_id,
            fcu_endpoint=args.fcu_endpoint,
        )
    )
    app = create_app(runtime, static_dir=args.static_dir)
    import uvicorn

    uvicorn.run(app, host=args.host, port=args.port)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

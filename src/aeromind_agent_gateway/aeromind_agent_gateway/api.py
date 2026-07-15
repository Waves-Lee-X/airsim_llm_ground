"""FastAPI HTTP and WebSocket surface for Agent sessions."""

from __future__ import annotations

import asyncio
from contextlib import asynccontextmanager
import hmac
import uuid
from typing import Any, Iterable

from fastapi import FastAPI, HTTPException, Query, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware

from .config import GatewayConfig
from .ros_state import RosStateBridge
from .session_manager import SessionManager


def create_app(
    config: GatewayConfig,
    sessions: SessionManager,
    ros_state: RosStateBridge,
    adapters: Iterable[Any] = (),
) -> FastAPI:
    channel_adapters = tuple(adapters)

    @asynccontextmanager
    async def lifespan(_app: FastAPI):
        await sessions.start()
        for adapter in channel_adapters:
            await adapter.start()
        try:
            yield
        finally:
            for adapter in reversed(channel_adapters):
                await adapter.stop()
            await sessions.close()

    app = FastAPI(
        title="Drone Agent Gateway",
        version="0.1.0",
        lifespan=lifespan,
    )
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_credentials=False,
        allow_methods=["GET", "POST", "DELETE", "OPTIONS"],
        allow_headers=["Authorization", "Content-Type"],
    )

    @app.get("/health")
    async def health():
        snapshot = ros_state.snapshot()
        return {
            "success": True,
            "service": "agent_gateway",
            "runtime": "multi_provider_agent",
            "default_model": config.default_model,
            "default_provider": config.default_provider,
            "providers": sessions.provider_catalog(),
            "ros_state_available": snapshot["available"],
            "authentication": "token" if config.access_token else "local_dev",
            "feishu_enabled": config.feishu_enabled,
            "feishu_vision_enabled": bool(config.vlm_api_url and config.vlm_model),
            "vlm_model": config.vlm_model or None,
            "identity_binding_count": len(config.identity_bindings),
            "semantic_memory_enabled": config.semantic_memory_enabled,
            "semantic_memory_model": (
                config.semantic_memory_model
                if config.semantic_memory_enabled
                else None
            ),
        }

    @app.post("/api/sessions")
    async def create_session(payload: dict):
        _require_http_token(config, payload.get("token", ""))
        try:
            return sessions.ensure_session(
                payload.get("session_id"),
                user_id=str(payload.get("user_id") or "web-local"),
                channel=str(payload.get("channel") or "web"),
            )
        except PermissionError as exc:
            raise HTTPException(status_code=403, detail=str(exc)) from exc

    @app.get("/api/skills")
    async def gateway_skills(token: str = Query(default="")):
        _require_http_token(config, token)
        return {"success": True, "skills": sessions.skill_catalog()}

    @app.get("/api/capabilities")
    async def gateway_capabilities(token: str = Query(default="")):
        _require_http_token(config, token)
        return {"success": True, "capabilities": sessions.capability_catalog()}

    @app.get("/api/sessions/{session_id}/history")
    async def session_history(
        session_id: str,
        token: str = Query(default=""),
        limit: int = Query(default=100, ge=1, le=500),
    ):
        _require_http_token(config, token)
        result = sessions.history(session_id, limit=limit)
        if result["session"] is None:
            raise HTTPException(status_code=404, detail="session not found")
        return result

    @app.get("/api/sessions/{session_id}/events")
    async def session_events(
        session_id: str,
        token: str = Query(default=""),
        after: int = Query(default=0, ge=0),
    ):
        _require_http_token(config, token)
        return {"events": sessions.events_after(session_id, sequence=after)}

    @app.get("/api/operator/state")
    async def operator_state(
        user_id: str = Query(default="web-local"),
        token: str = Query(default=""),
    ):
        _require_http_token(config, token)
        return sessions.operator_state(user_id)

    @app.get("/api/operator/timeline")
    async def operator_timeline(
        user_id: str = Query(default="web-local"),
        token: str = Query(default=""),
        limit: int = Query(default=100, ge=1, le=500),
    ):
        _require_http_token(config, token)
        return {"messages": sessions.operator_timeline(user_id, limit=limit)}

    @app.get("/api/operator/memories")
    async def operator_memories(
        user_id: str = Query(default="web-local"),
        token: str = Query(default=""),
        query: str = Query(default=""),
        limit: int = Query(default=20, ge=1, le=100),
    ):
        _require_http_token(config, token)
        return {
            "memories": sessions.semantic_memories(
                user_id, query=query, limit=limit
            )
        }

    @app.delete("/api/operator/memories/{memory_id}")
    async def delete_operator_memory(
        memory_id: str,
        user_id: str = Query(default="web-local"),
        token: str = Query(default=""),
    ):
        _require_http_token(config, token)
        if not sessions.delete_semantic_memory(user_id, memory_id):
            raise HTTPException(status_code=404, detail="memory not found")
        return {"success": True, "memory_id": memory_id}

    @app.post("/api/workflows/{workflow_id}/{command}")
    async def control_workflow(
        workflow_id: str,
        command: str,
        payload: dict,
    ):
        _require_http_token(config, payload.get("token", ""))
        try:
            return await sessions.control_workflow(
                str(payload.get("user_id") or "web-local"),
                workflow_id,
                command,
            )
        except ValueError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    @app.websocket("/ws/agent")
    async def agent_websocket(
        websocket: WebSocket,
        session_id: str = Query(default=""),
        user_id: str = Query(default="web-local"),
        channel: str = Query(default="web"),
        token: str = Query(default=""),
        after: int = Query(default=0),
    ):
        if not _valid_token(config, token):
            await websocket.close(code=4401, reason="invalid token")
            return

        try:
            session = sessions.ensure_session(
                session_id or f"session-{uuid.uuid4().hex}",
                user_id=user_id,
                channel=channel,
            )
        except PermissionError:
            await websocket.close(code=4403, reason="session belongs to another user")
            return
        active_session_id = session["id"]
        principal = session["user_id"]
        await websocket.accept()
        queue = await sessions.subscribe_user(principal)
        sender = asyncio.create_task(_send_events(websocket, queue))
        try:
            operator = sessions.operator_state(principal)
            await websocket.send_json(
                {
                    "type": "session.ready",
                    "session_id": active_session_id,
                    "session": session,
                    "history": sessions.history(active_session_id)["messages"],
                    "timeline": sessions.operator_timeline(principal),
                    "events": sessions.events_after(active_session_id, sequence=after),
                    "operator": operator,
                    "providers": sessions.provider_catalog(),
                    "skills": sessions.skill_catalog(),
                    "capabilities": sessions.capability_catalog(),
                    "pending_confirmations": operator["pending_confirmations"],
                    "missions": operator["missions"],
                }
            )
            while True:
                payload = await websocket.receive_json()
                message_type = str(payload.get("type", ""))
                request_id = str(payload.get("request_id") or uuid.uuid4().hex)
                if message_type == "chat.send":
                    content = str(payload.get("content", ""))
                    try:
                        confirmation = await sessions.respond_confirmation_message(
                            principal, content
                        )
                    except (ValueError, PermissionError) as exc:
                        await websocket.send_json(
                            {
                                "type": "error",
                                "session_id": active_session_id,
                                "request_id": request_id,
                                "message": str(exc),
                            }
                        )
                    else:
                        if confirmation is None:
                            sessions.submit_chat(
                                active_session_id,
                                content,
                                request_id=request_id,
                            )
                elif message_type == "model.switch":
                    try:
                        await sessions.switch_model(
                            active_session_id,
                            str(payload.get("model", "")),
                            provider=str(payload.get("provider", "")) or None,
                        )
                    except (ValueError, RuntimeError) as exc:
                        await websocket.send_json(
                            {
                                "type": "error",
                                "session_id": active_session_id,
                                "request_id": request_id,
                                "message": str(exc),
                            }
                        )
                elif message_type == "assistant.interrupt":
                    await sessions.interrupt(active_session_id)
                elif message_type == "workflow.control":
                    try:
                        await sessions.control_workflow(
                            principal,
                            str(payload.get("workflow_id", "")),
                            str(payload.get("command", "")),
                        )
                    except ValueError as exc:
                        await websocket.send_json(
                            {
                                "type": "error",
                                "session_id": active_session_id,
                                "request_id": request_id,
                                "message": str(exc),
                            }
                        )
                elif message_type == "confirmation.respond":
                    try:
                        await sessions.respond_confirmation_for_user(
                            user_id=principal,
                            confirmation_id=str(payload.get("confirmation_id", "")),
                            decision=str(payload.get("decision", "")),
                        )
                    except (ValueError, PermissionError) as exc:
                        await websocket.send_json(
                            {
                                "type": "error",
                                "session_id": active_session_id,
                                "request_id": request_id,
                                "message": str(exc),
                            }
                        )
                elif message_type == "session.history":
                    await websocket.send_json(
                        {
                            "type": "session.history",
                            "session_id": active_session_id,
                            **sessions.history(active_session_id),
                        }
                    )
                elif message_type == "session.clear":
                    try:
                        await sessions.clear_session(active_session_id, principal)
                    except (ValueError, PermissionError, RuntimeError) as exc:
                        await websocket.send_json(
                            {
                                "type": "error",
                                "session_id": active_session_id,
                                "request_id": request_id,
                                "message": str(exc),
                            }
                        )
                elif message_type == "ping":
                    await websocket.send_json(
                        {"type": "pong", "session_id": active_session_id}
                    )
                else:
                    await websocket.send_json(
                        {
                            "type": "error",
                            "session_id": active_session_id,
                            "request_id": request_id,
                            "message": f"不支持的消息类型: {message_type}",
                        }
                    )
        except WebSocketDisconnect:
            pass
        finally:
            sender.cancel()
            await asyncio.gather(sender, return_exceptions=True)
            await sessions.unsubscribe_user(principal, queue)

    return app


async def _send_events(websocket: WebSocket, queue: asyncio.Queue):
    while True:
        event = await queue.get()
        await websocket.send_json(event)


def _valid_token(config: GatewayConfig, token: str) -> bool:
    if not config.access_token:
        return True
    return hmac.compare_digest(config.access_token, token)


def _require_http_token(config: GatewayConfig, token: str):
    if not _valid_token(config, str(token)):
        raise HTTPException(status_code=401, detail="invalid token")

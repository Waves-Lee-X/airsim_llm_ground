"""Persistent, user-bound two-stage approval for flight actions."""

from __future__ import annotations

import asyncio
import time
from typing import Any, Awaitable, Callable

from .events import EventBus
from .store import SessionStore
from .workflow import validate_workflow


ProgressCallback = Callable[[str, dict[str, Any]], Awaitable[None]]
ExecuteCallback = Callable[
    [str, dict[str, Any], ProgressCallback], Awaitable[dict[str, Any]]
]


ACTION_SPECS = {
    "arm": {"risk": "high", "summary": "解锁无人机"},
    "disarm": {"risk": "high", "summary": "加锁无人机"},
    "takeoff": {"risk": "high", "summary": "起飞到 {altitude:.1f} 米"},
    "land": {"risk": "high", "summary": "降落"},
    "move": {"risk": "high", "summary": "向{direction}飞行 {distance:.1f} 米"},
    "return_home": {"risk": "high", "summary": "触发 PX4 原生 RTL 返航"},
    "hover": {"risk": "medium", "summary": "取消自主目标并悬停"},
    "workflow": {"risk": "high", "summary": "执行组合任务：{name}"},
}


class ConfirmationCenter:
    def __init__(
        self,
        store: SessionStore,
        events: EventBus,
        execute: ExecuteCallback,
        ttl_seconds: float = 60.0,
    ):
        self._store = store
        self._events = events
        self._execute = execute
        self._ttl_seconds = ttl_seconds
        self._tasks: set[asyncio.Task] = set()
        self._scheduled_expiries: set[str] = set()

    async def start(self):
        for item in self._store.pending_confirmations_all():
            self._schedule_expiry(item)

    async def create(
        self,
        session_id: str,
        user_id: str,
        action: str,
        args: dict[str, Any],
    ) -> dict[str, Any]:
        normalized = validate_action(action, args)
        spec = ACTION_SPECS[action]
        summary = spec["summary"].format(**normalized)
        item = self._store.create_confirmation(
            session_id=session_id,
            user_id=user_id,
            action=action,
            args=normalized,
            summary=summary,
            risk_level=spec["risk"],
            ttl_seconds=self._ttl_seconds,
        )
        mission = self._store.create_gateway_mission(item)
        await self._events.emit(
            session_id,
            "mission.updated",
            mission=mission,
        )
        await self._events.emit(
            session_id,
            "confirmation.required",
            confirmation=item,
            mission=mission,
        )
        self._schedule_expiry(item)
        return {
            "success": True,
            "status": "pending_confirmation",
            "confirmation_id": item["id"],
            "summary": item["summary"],
            "risk_level": item["risk_level"],
            "expires_at": item["expires_at"],
            "message": "控制请求已创建，必须由当前用户确认后才会执行。",
        }

    def _schedule_expiry(self, item: dict[str, Any]):
        confirmation_id = item["id"]
        if confirmation_id in self._scheduled_expiries:
            return
        self._scheduled_expiries.add(confirmation_id)
        expiry_task = asyncio.create_task(self._expire_confirmation(item))
        self._tasks.add(expiry_task)
        expiry_task.add_done_callback(
            lambda task, cid=confirmation_id: self._expiry_done(cid, task)
        )

    def _expiry_done(self, confirmation_id: str, task: asyncio.Task):
        self._scheduled_expiries.discard(confirmation_id)
        self._tasks.discard(task)

    async def _expire_confirmation(self, item: dict[str, Any]):
        delay = max(0.0, float(item["expires_at"]) - time.time())
        await asyncio.sleep(delay)
        current = self._store.get_confirmation(item["id"])
        if current is None or current["status"] != "pending":
            return
        self._store.pending_confirmations(item["session_id"])
        current = self._store.get_confirmation(item["id"])
        mission = self._store.gateway_mission_for_confirmation(item["id"])
        if current is None or current["status"] != "expired":
            return
        await self._events.emit(
            item["session_id"],
            "confirmation.expired",
            confirmation=current,
            mission=mission,
        )
        if mission is not None:
            await self._events.emit(
                item["session_id"], "mission.updated", mission=mission
            )

    async def respond(
        self,
        confirmation_id: str,
        user_id: str,
        decision: str,
    ) -> dict[str, Any]:
        normalized_decision = decision.strip().lower()
        if normalized_decision not in {"approve", "cancel"}:
            raise ValueError("decision 必须为 approve 或 cancel")
        item = self._store.resolve_confirmation(
            confirmation_id, user_id, normalized_decision
        )
        mission = self._store.update_gateway_mission(
            confirmation_id,
            "executing" if normalized_decision == "approve" else "cancelled",
            phase="dispatching" if normalized_decision == "approve" else "cancelled",
        )
        await self._events.emit(
            item["session_id"],
            "confirmation.resolved",
            confirmation=item,
            decision=normalized_decision,
            mission=mission,
        )
        if mission is not None:
            await self._events.emit(
                item["session_id"], "mission.updated", mission=mission
            )
        if normalized_decision == "approve":
            task = asyncio.create_task(self._execute_confirmation(item))
            self._tasks.add(task)
            task.add_done_callback(self._tasks.discard)
        return item

    async def _execute_confirmation(self, item: dict[str, Any]):
        session_id = item["session_id"]
        await self._events.emit(
            session_id,
            "control.started",
            confirmation_id=item["id"],
            action=item["action"],
            args=item["args"],
        )

        async def progress(phase: str, details: dict[str, Any]):
            mission = self._store.update_gateway_mission(
                item["id"],
                "executing",
                result=details,
                phase=phase,
                ros_mission_id=details.get("ros_mission_id"),
                physical_complete=False,
            )
            await self._events.emit(
                session_id,
                "control.progress",
                confirmation_id=item["id"],
                confirmation=self._store.get_confirmation(item["id"]),
                action=item["action"],
                phase=phase,
                details=details,
                mission=mission,
            )
            if mission is not None:
                await self._events.emit(
                    session_id, "mission.updated", mission=mission
                )

        try:
            result = await self._execute(item["action"], item["args"], progress)
            success = bool(result.get("success"))
        except Exception as exc:
            result = {"success": False, "message": str(exc)}
            success = False
        final_status = (
            "completed"
            if success
            else ("cancelled" if result.get("status") == "cancelled" else "failed")
        )
        completed = self._store.complete_confirmation(
            item["id"], final_status, result
        )
        mission = self._store.update_gateway_mission(
            item["id"],
            final_status,
            result,
            phase=final_status,
            ros_mission_id=result.get("ros_mission_id"),
            physical_complete=bool(result.get("physical_complete", False)),
        )
        await self._events.emit(
            session_id,
            "control.completed",
            confirmation=completed,
            success=success,
            status=final_status,
            result=result,
            mission=mission,
        )
        if mission is not None:
            await self._events.emit(
                session_id, "mission.updated", mission=mission
            )

    def pending(self, session_id: str):
        return self._store.pending_confirmations(session_id)

    async def close(self):
        for task in list(self._tasks):
            task.cancel()
        if self._tasks:
            await asyncio.gather(*self._tasks, return_exceptions=True)
        self._scheduled_expiries.clear()


def validate_action(action: str, args: dict[str, Any]) -> dict[str, Any]:
    if action not in ACTION_SPECS:
        raise ValueError(f"不支持的控制动作: {action}")
    if action == "takeoff":
        altitude = float(args.get("altitude", 10.0))
        if not 1.0 <= altitude <= 30.0:
            raise ValueError("起飞高度必须在 1 到 30 米之间")
        return {"altitude": altitude}
    if action == "move":
        direction = str(args.get("direction", "")).strip().lower()
        aliases = {
            "forward": "前方",
            "backward": "后方",
            "left": "左侧",
            "right": "右侧",
            "up": "上方",
            "down": "下方",
            "前": "前方",
            "后": "后方",
            "左": "左侧",
            "右": "右侧",
            "上": "上方",
            "下": "下方",
        }
        if direction not in aliases:
            raise ValueError("移动方向必须为 forward/backward/left/right/up/down")
        distance = float(args.get("distance", 0.0))
        if not 0.5 <= distance <= 100.0:
            raise ValueError("移动距离必须在 0.5 到 100 米之间")
        return {"direction": aliases[direction], "distance": distance}
    if action == "workflow":
        return validate_workflow(args)
    return {}

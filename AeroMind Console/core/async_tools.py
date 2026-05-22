from __future__ import annotations

import asyncio
from typing import Any, Callable, Coroutine

from core.agent_tools import AgentToolRuntime
from core.airsim_adapter import AirSimAdapter
from core.safety_gate import SafetyGate
from core.task_executor import TaskExecutor


class AsyncToolRuntime:
    def __init__(
        self,
        airsim: AirSimAdapter,
        executor: TaskExecutor,
        safety_gate: SafetyGate,
    ) -> None:
        self._airsim = airsim
        self._executor = executor
        self._safety_gate = safety_gate
        self._loop = asyncio.get_event_loop()

    async def takeoff(self, altitude_m: float = 8.0) -> dict[str, Any]:
        return await self._run_sync(lambda: self._executor.takeoff(altitude_m))

    async def hover(self) -> dict[str, Any]:
        return await self._run_sync(lambda: self._executor.hover())

    async def stop(self) -> dict[str, Any]:
        return await self._run_sync(lambda: self._executor.stop())

    async def land(self, confirmed: bool = False) -> dict[str, Any]:
        return await self._run_sync(lambda: self._executor.land())

    async def return_home(self, safe_altitude_m: float = 8.0, confirmed: bool = False) -> dict[str, Any]:
        return await self._run_sync(lambda: self._executor.return_home(safe_altitude_m))

    async def goto_local(self, x: float, y: float, z: float, speed_mps: float = 2.0) -> dict[str, Any]:
        return await self._run_sync(lambda: self._executor.goto_local(x, y, z, speed_mps))

    async def waypoint_route(
        self,
        waypoints: list[dict[str, float]],
        speed_mps: float = 2.0,
        avoidance: bool = True,
        hold_at_end: bool = True,
    ) -> dict[str, Any]:
        return await self._run_sync(
            lambda: self._executor.waypoint_route(waypoints, speed_mps, avoidance, hold_at_end)
        )

    async def search_area(
        self,
        area: dict[str, float],
        altitude_m: float = 8.0,
        speed_mps: float = 2.0,
        spacing_m: float = 10.0,
        **kwargs: Any,
    ) -> dict[str, Any]:
        return await self._run_sync(
            lambda: self._executor.search_area(area, altitude_m, speed_mps, spacing_m, **kwargs)
        )

    async def formation_flight(
        self,
        count: int = 3,
        shape: str = "v",
        distance_m: float = 40.0,
        altitude_m: float = 8.0,
        spacing_m: float = 6.0,
        speed_mps: float = 2.0,
    ) -> dict[str, Any]:
        return await self._run_sync(
            lambda: self._executor.formation_flight(count, shape, distance_m, altitude_m, spacing_m, speed_mps)
        )

    async def recover_from_collision(self, climb_m: float = 8.0, backoff_m: float = 10.0) -> dict[str, Any]:
        return await self._run_sync(lambda: self._executor.recover_from_collision(climb_m, backoff_m))

    async def detect_objects(self, camera: str = "front_center", target: str = "object") -> dict[str, Any]:
        return await self._run_sync(lambda: self._executor.detect_objects(camera, target))

    async def telemetry(self) -> dict[str, Any]:
        return await self._run_sync(lambda: self._airsim.telemetry().to_api())

    async def obstacle_status(self) -> dict[str, Any]:
        return await self._run_sync(lambda: self._airsim.obstacle_status().to_api())

    async def collision_status(self) -> dict[str, Any]:
        return await self._run_sync(lambda: self._airsim.collision_status().to_api())

    async def distance_sensors_status(self) -> dict[str, Any]:
        return await self._run_sync(lambda: self._airsim.distance_sensors_status().to_api())

    async def _run_sync(self, func: Callable[[], Any]) -> Any:
        return await self._loop.run_in_executor(None, func)


class AsyncTaskRunner:
    def __init__(self, tool_runtime: AsyncToolRuntime) -> None:
        self._tools = tool_runtime
        self._task_future: asyncio.Future | None = None
        self._task_status = "idle"

    @property
    def status(self) -> str:
        return self._task_status

    async def run_task(self, plan: list[dict[str, Any]]) -> dict[str, Any]:
        if self._task_future and not self._task_future.done():
            return {"error": "Task already running", "status": self._task_status}

        self._task_status = "running"
        results = []

        try:
            for step in plan:
                tool = step.get("tool")
                args = step.get("args", {})

                if tool == "takeoff":
                    result = await self._tools.takeoff(**args)
                elif tool == "hover":
                    result = await self._tools.hover()
                elif tool == "stop":
                    result = await self._tools.stop()
                elif tool == "land":
                    result = await self._tools.land(**args)
                elif tool == "return_home":
                    result = await self._tools.return_home(**args)
                elif tool == "goto_local":
                    result = await self._tools.goto_local(**args)
                elif tool == "waypoint_route":
                    result = await self._tools.waypoint_route(**args)
                elif tool == "search_area":
                    result = await self._tools.search_area(**args)
                elif tool == "formation_flight":
                    result = await self._tools.formation_flight(**args)
                elif tool == "recover_from_collision":
                    result = await self._tools.recover_from_collision(**args)
                elif tool == "detect_objects":
                    result = await self._tools.detect_objects(**args)
                else:
                    result = {"error": f"Unknown tool: {tool}", "step": step}

                results.append(result)

                if result.get("error"):
                    self._task_status = "failed"
                    return {"status": "failed", "results": results}

                await asyncio.sleep(0.1)

            self._task_status = "completed"
            return {"status": "completed", "results": results}

        except Exception as exc:
            self._task_status = "error"
            return {"status": "error", "error": str(exc), "results": results}

    async def cancel_task(self) -> dict[str, Any]:
        if self._task_future and not self._task_future.done():
            self._task_future.cancel()
            await self._tools.stop()
            self._task_status = "cancelled"
            return {"status": "cancelled"}
        return {"status": "no_task_running"}

    async def stop(self) -> dict[str, Any]:
        await self._tools.stop()
        self._task_status = "stopped"
        return {"status": "stopped"}
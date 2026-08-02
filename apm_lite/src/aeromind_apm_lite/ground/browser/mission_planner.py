"""Multi-step mission planner executor for the browser ground station.

Turns an operator-confirmed agent plan (a list of atomic steps) into real
flight actions. Steps reuse the same FlightCommandService / fly_formation
primitives as the fleet mission runner; every step waits for physical
completion evidence and failures trigger a safety landing for the involved
vehicles. Only simulation (SITL) vehicles are allowed.
"""

from __future__ import annotations

import asyncio
import math
import time
from dataclasses import dataclass
from typing import Any, Mapping, Sequence

from aeromind_apm_lite.common.formation import FormationType
from aeromind_apm_lite.ground.browser.fleet_executor import (
    FleetMissionError,
    fly_formation,
    require_completed_command,
)
from aeromind_apm_lite.ground.browser.runtime import ManualRuntimeMode
from aeromind_apm_lite.ground.browser.visual_navigation import (
    estimate_target_ned,
    target_from_depth,
)
from aeromind_apm_lite.onboard.flight_commands import FlightCommandService
from aeromind_apm_lite.onboard.mavlink.models import MavLandedState

_STEP_TIMEOUT_S = 120.0
_MAX_TARGET_RANGE_M = 150.0
_TAKEOFF_TIMEOUT_S = 90.0
_TAKEOFF_MAX_ATTEMPTS = 3
_TAKEOFF_ATTEMPT_WINDOW_S = 60.0
_TAKEOFF_SETTLE_WAIT_S = 20.0
_TAKEOFF_RETRY_GAP_S = 5.0
_LAND_TIMEOUT_S = 60.0


def _line_intersection(
    a: tuple[float, float],
    u: tuple[float, float],
    b: tuple[float, float],
    v: tuple[float, float],
    *,
    max_range_m: float = 300.0,
) -> tuple[float, float] | None:
    """Intersect ray ``a + t*u`` with ray ``b + s*v`` in the horizontal plane."""
    denom = u[0] * v[1] - u[1] * v[0]
    if abs(denom) <= 1e-9:
        return None
    t = ((b[0] - a[0]) * v[1] - (b[1] - a[1]) * v[0]) / denom
    if not math.isfinite(t) or t <= 0.0 or t > max_range_m:
        return None
    return (a[0] + t * u[0], a[1] + t * u[1])


def _standoff_point(
    target: tuple[float, float],
    from_position: tuple[float, float],
    standoff_m: float,
) -> tuple[float, float]:
    """Return the point ``standoff_m`` before ``target`` on the line from
    ``from_position`` toward ``target`` (used to stop in front of a target)."""
    dx = target[0] - from_position[0]
    dy = target[1] - from_position[1]
    distance = math.hypot(dx, dy)
    if distance <= 1e-6:
        return target
    scale = standoff_m / distance
    return (target[0] - dx * scale, target[1] - dy * scale)


def _recenter_leg_m(distance_m: float, standoff_m: float) -> float:
    """Clamp the recenter leg so the vehicle never crosses the standoff
    safety margin (1.5 m) while centering the target in frame."""
    leg = 5.0
    if standoff_m > 0.0:
        leg = min(leg, max(0.0, float(distance_m) - standoff_m - 1.5))
    return leg


_PLAN_ACTIONS = frozenset(
    {"arm", "disarm", "takeoff", "goto", "formation", "hold", "land", "analyze"}
)


@dataclass(frozen=True)
class PlanStep:
    action: str
    vehicle_ids: tuple[int, ...]
    params: dict[str, Any]


class MissionPlanner:
    """Run one multi-step plan at a time and expose progress."""

    def __init__(self, runtime: Any, *, clock: Any = time.monotonic) -> None:
        self._runtime = runtime
        self._clock = clock
        self._task: asyncio.Task[None] | None = None
        self._cancel_event = asyncio.Event()
        self._phase = "idle"
        self._stage = ""
        self._steps: list[dict[str, Any]] = []
        self._results: list[dict[str, Any]] = []
        self._error: str | None = None
        self._services: dict[int, FlightCommandService] = {}
        self.analyzer: Any | None = None

    @property
    def busy(self) -> bool:
        return self._task is not None and not self._task.done()

    def status_payload(self) -> dict[str, Any]:
        return {
            "phase": self._phase,
            "stage": self._stage,
            "busy": self.busy,
            "error": self._error,
            "steps": [dict(item) for item in self._steps],
            "results": [dict(item) for item in self._results],
        }

    async def start(self, steps: Sequence[PlanStep]) -> dict[str, Any]:
        if self.busy:
            raise FleetMissionError("计划正在执行中，请先取消或等待完成")
        if not 1 <= len(steps) <= 20:
            raise FleetMissionError("计划步骤数必须在 1 到 20 之间")
        self._validate_steps(steps)
        self._steps = [
            {
                "index": index + 1,
                "action": step.action,
                "vehicle_ids": list(step.vehicle_ids),
                "params": dict(step.params),
                "state": "pending",
                "detail": "",
            }
            for index, step in enumerate(steps)
        ]
        self._results = []
        self._error = None
        self._cancel_event = asyncio.Event()
        self._phase = "starting"
        self._stage = "检查链路"
        self._services = self._build_services(
            {vehicle_id for step in steps for vehicle_id in step.vehicle_ids}
        )
        self._task = asyncio.create_task(
            self._run(tuple(steps)),
            name="mission-planner",
        )
        payload = self.status_payload()
        payload["successful"] = True
        return payload

    async def cancel(self) -> dict[str, Any]:
        if self.busy:
            self._cancel_event.set()
            self._phase = "cancelling"
            self._stage = "正在安全降落"
        return self.status_payload()

    def _validate_steps(self, steps: Sequence[PlanStep]) -> None:
        for index, step in enumerate(steps, start=1):
            action = str(step.action).lower()
            if action not in _PLAN_ACTIONS:
                raise FleetMissionError(f"第 {index} 步动作 {action!r} 不受支持")
            if not step.vehicle_ids:
                raise FleetMissionError(f"第 {index} 步缺少参与飞机")
            if len(set(step.vehicle_ids)) != len(step.vehicle_ids):
                raise FleetMissionError(f"第 {index} 步参与飞机重复")
            if action == "takeoff":
                altitude = step.params.get("altitude_m")
                if not isinstance(altitude, (int, float)) or not 0.5 <= float(altitude) <= 20.0:
                    raise FleetMissionError(f"第 {index} 步起飞高度必须在 0.5-20 米")
            if action == "goto":
                position = step.params.get("target_position_ned_m")
                if (
                    not isinstance(position, (list, tuple))
                    or len(position) != 3
                    or not all(
                        isinstance(value, (int, float)) for value in position
                    )
                ):
                    raise FleetMissionError(f"第 {index} 步 GOTO 目标无效")
            if action == "analyze":
                if len(step.vehicle_ids) != 1:
                    raise FleetMissionError(f"第 {index} 步 analyze 只能指定一架飞机")
                prompt = step.params.get("prompt")
                if prompt is not None and not isinstance(prompt, str):
                    raise FleetMissionError(f"第 {index} 步 analyze 提示词必须是字符串")
                navigate = step.params.get("navigate_after")
                if navigate is not None and not isinstance(navigate, bool):
                    raise FleetMissionError(
                        f"第 {index} 步 navigate_after 必须是布尔值"
                    )
            if action == "formation":
                formation = step.params.get("formation")
                if formation not in {"line", "v", "diamond"}:
                    raise FleetMissionError(f"第 {index} 步队形必须是 line/v/diamond")
                leader = step.params.get("leader_target_map_m")
                if (
                    not isinstance(leader, (list, tuple))
                    or len(leader) != 3
                    or not all(
                        isinstance(value, (int, float)) for value in leader
                    )
                ):
                    raise FleetMissionError(f"第 {index} 步领机目标无效")
                spacing = step.params.get("spacing_m")
                if not isinstance(spacing, (int, float)) or not 0.5 <= float(spacing) <= 50.0:
                    raise FleetMissionError(f"第 {index} 步间距必须在 0.5-50 米")
                altitude = step.params.get("altitude_m")
                if not isinstance(altitude, (int, float)) or not 0.5 <= float(altitude) <= 20.0:
                    raise FleetMissionError(f"第 {index} 步编队高度必须在 0.5-20 米")

    def _build_services(
        self,
        vehicle_ids: set[int],
    ) -> dict[int, FlightCommandService]:
        services: dict[int, FlightCommandService] = {}
        for vehicle_id in vehicle_ids:
            try:
                runtime = self._runtime.runtime_for(vehicle_id)
            except KeyError as exc:
                raise FleetMissionError(f"飞机 {vehicle_id} 未注册") from exc
            if runtime.config.mode != ManualRuntimeMode.SITL:
                raise FleetMissionError(
                    f"飞机 {vehicle_id} 不是仿真机，计划执行仅支持仿真"
                )
            link = runtime.link
            if link is None or not getattr(link, "ready", False):
                raise FleetMissionError(f"飞机 {vehicle_id} 链路未就绪")
            services[vehicle_id] = FlightCommandService(link)
        return services

    async def _run(self, steps: tuple[PlanStep, ...]) -> None:
        try:
            self._phase = "running"
            for step_index, step in enumerate(steps):
                self._check_cancel()
                await self._execute_step(step, step_index)
            self._phase = "done" if not self._cancel_event.is_set() else "cancelled"
            self._stage = "计划执行完成" if self._phase == "done" else "已取消"
        except asyncio.CancelledError:
            self._phase = "cancelled"
            self._stage = "计划被取消"
            raise
        except FleetMissionError as exc:
            if self._cancel_event.is_set():
                self._phase = "cancelled"
                self._stage = "已取消"
            else:
                self._phase = "failed"
                self._stage = "计划执行失败"
            self._error = str(exc)
            await self._safety_land()
        except Exception as exc:  # pragma: no cover - defensive
            self._phase = "failed"
            self._stage = "计划执行失败"
            self._error = f"{type(exc).__name__}: {exc}"
            await self._safety_land()

    def _check_cancel(self) -> None:
        if self._cancel_event.is_set():
            raise FleetMissionError("计划已取消")

    async def _sleep_or_cancel(self, seconds: float) -> None:
        try:
            await asyncio.wait_for(self._cancel_event.wait(), timeout=seconds)
        except asyncio.TimeoutError:
            return
        raise FleetMissionError("计划已取消")

    async def _execute_step(self, step: PlanStep, step_index: int) -> None:
        action = step.action.lower()
        entry = self._steps[step_index]
        entry.update({"state": "running", "detail": f"执行 {action}"})
        self._stage = f"步骤 {entry['index']}/{len(self._steps)}：{action}"
        params = step.params
        services = self._services

        async def for_each(step_name: str, build: Any, timeout_s: float) -> None:
            await asyncio.gather(
                *(
                    self._await_handle(
                        build(vehicle_id),
                        vehicle_id,
                        step_name,
                        timeout_s,
                    )
                    for vehicle_id in step.vehicle_ids
                )
            )

        if action == "arm":
            # ArduPilot only allows arming from GUIDED; make the plan robust
            # when the model omitted the mode switch.
            await for_each(
                "set_mode",
                lambda vid: services[vid].set_mode("GUIDED"),
                _STEP_TIMEOUT_S,
            )
            await for_each("arm", lambda vid: services[vid].arm(), _STEP_TIMEOUT_S)
        elif action == "disarm":
            await for_each("disarm", lambda vid: services[vid].disarm(), _STEP_TIMEOUT_S)
        elif action == "takeoff":
            altitude = float(params["altitude_m"])
            await for_each(
                "set_mode",
                lambda vid: services[vid].set_mode("GUIDED"),
                _STEP_TIMEOUT_S,
            )
            await self._ensure_armed(step.vehicle_ids, services)
            await asyncio.gather(
                *(
                    self._takeoff_with_retry(
                        vehicle_id,
                        services[vehicle_id],
                        altitude,
                    )
                    for vehicle_id in step.vehicle_ids
                )
            )
        elif action == "hold":
            # GUIDED position-hold instead of LOITER: in this SITL setup the
            # LOITER mode drops the aircraft to the ground, and a GUIDED hold
            # keeps the mode ready for the following visual-navigation steps.
            await for_each(
                "set_mode",
                lambda vid: services[vid].set_mode("GUIDED"),
                _STEP_TIMEOUT_S,
            )
            for vehicle_id in step.vehicle_ids:
                current = await self._current_position(vehicle_id)
                await self._await_handle(
                    services[vehicle_id].goto_local_ned(
                        current[0],
                        current[1],
                        current[2],
                        expires_monotonic_s=self._clock() + _STEP_TIMEOUT_S,
                    ),
                    vehicle_id,
                    "hold_position",
                    _STEP_TIMEOUT_S,
                )
        elif action == "land":
            await for_each("land", lambda vid: services[vid].land(), _LAND_TIMEOUT_S)
        elif action == "goto":
            position = tuple(float(value) for value in params["target_position_ned_m"])
            await for_each(
                "goto",
                lambda vid: services[vid].goto_local_ned(
                    position[0],
                    position[1],
                    position[2],
                    expires_monotonic_s=self._clock() + _STEP_TIMEOUT_S,
                ),
                _STEP_TIMEOUT_S,
            )
        elif action == "analyze":
            if self.analyzer is None:
                raise FleetMissionError("analyze 步骤需要视觉分析服务")
            vehicle_id = step.vehicle_ids[0]
            prompt = str(
                params.get("prompt")
                or "描述当前画面中的目标、颜色、编号与风险"
            )
            analysis = await self.analyzer(vehicle_id, prompt)
            result_text = str(analysis.get("result") or analysis)
            navigate_detail = ""
            if params.get("navigate_after"):
                navigate_detail = await self._navigate_to_visual_target(
                    vehicle_id,
                    services[vehicle_id],
                    analysis,
                    params,
                )
            entry.update(
                {
                    "state": "done",
                    "detail": f"analyze: {result_text[:140]}{navigate_detail}",
                }
            )
            self._results.append(
                {
                    "vehicle_id": vehicle_id,
                    "step": "analyze",
                    "status": "completed",
                    "detail": f"{result_text[:380]}{navigate_detail}",
                    "at_monotonic_s": round(self._clock(), 2),
                }
            )
        elif action == "formation":
            results = await fly_formation(
                services,
                self._runtime,
                step.vehicle_ids,
                FormationType(params["formation"]),
                tuple(float(value) for value in params["leader_target_map_m"]),
                float(params.get("spacing_m", 3.0)),
                float(params.get("altitude_m", 2.0)),
                clock=self._clock,
                step_timeout_s=_STEP_TIMEOUT_S,
                on_step=lambda vid, name, detail: None,
            )
            self._results.extend(results)
        else:  # pragma: no cover - validated
            raise FleetMissionError(f"不支持的步骤动作 {action!r}")

        entry.update({"state": "done", "detail": f"{action} 完成"})

    async def _current_position(
        self,
        vehicle_id: int,
    ) -> tuple[float, float, float]:
        link = self._runtime.link_for(vehicle_id)
        if link is None:
            raise FleetMissionError(f"飞机 {vehicle_id} 链路不可用")
        snapshot = link.telemetry_snapshot()
        position = snapshot.local_position_ned_m
        if position is None:
            raise FleetMissionError(f"飞机 {vehicle_id} 当前位置不可用")
        return (float(position[0]), float(position[1]), float(position[2]))

    async def _approach_target_direction(
        self,
        vehicle_id: int,
        analysis: dict[str, Any],
        service: FlightCommandService,
        approach_distance_m: float = 15.0,
    ) -> float | None:
        'Fly toward the horizontal bearing of the detected target.'
        try:
            result = analysis.get("result")
            if not isinstance(result, dict):
                return None
            target = result.get("target")
            if not isinstance(target, dict):
                return None
            if target.get("found") is False:
                return None
            center_x = target.get("center_x")
            frame_size = analysis.get("frame_size_px")
            fov = analysis.get("camera_fov_degrees")
            if (
                center_x is None
                or not isinstance(frame_size, (list, tuple))
                or len(frame_size) != 2
                or fov is None
            ):
                return None
            link = self._runtime.link_for(vehicle_id)
            if link is None:
                return None
            snapshot = link.telemetry_snapshot()
            position = snapshot.local_position_ned_m
            attitude = snapshot.attitude_rpy_rad
            if position is None:
                return None
            yaw = float(attitude[2]) if attitude is not None else 0.0
            horizontal = (float(center_x) - 0.5) * 2.0
            half = math.radians(float(fov)) / 2.0
            h_angle = horizontal * half
            dir_b_x = math.cos(h_angle)
            dir_b_y = math.sin(h_angle)
            dir_x = math.cos(yaw) * dir_b_x - math.sin(yaw) * dir_b_y
            dir_y = math.sin(yaw) * dir_b_x + math.cos(yaw) * dir_b_y
            norm = math.hypot(dir_x, dir_y)
            if norm <= 1e-6:
                return None
            dx = dir_x / norm
            dy = dir_y / norm
            current = await self._current_position(vehicle_id)
            await self._await_handle(
                service.goto_local_ned(
                    current[0] + dx * approach_distance_m,
                    current[1] + dy * approach_distance_m,
                    current[2],
                    yaw_rad=math.atan2(dy, dx),
                    expires_monotonic_s=self._clock() + _STEP_TIMEOUT_S,
                ),
                vehicle_id,
                "approach_target",
                _STEP_TIMEOUT_S,
            )
            return approach_distance_m
        except (TypeError, ValueError):
            return None

    def _analysis_target_center(
        self,
        analysis: dict[str, Any],
    ) -> tuple[float, float] | None:
        """Normalized target center from the VLM or the deterministic fallbacks."""
        result = analysis.get("result")
        if isinstance(result, dict):
            target = result.get("target")
            if isinstance(target, dict) and target.get("found") is False:
                # The model explicitly says the target is absent; blob
                # fallbacks would navigate to a random salient spot.
                return None
            if isinstance(target, dict) and target.get("found") is not False:
                cx = target.get("center_x")
                cy = target.get("center_y")
                if (
                    isinstance(cx, (int, float))
                    and isinstance(cy, (int, float))
                    and 0.0 <= float(cx) <= 1.0
                    and 0.0 <= float(cy) <= 1.0
                ):
                    return (float(cx), float(cy))
        salient = analysis.get("salient_center")
        if (
            isinstance(salient, (list, tuple))
            and len(salient) == 2
            and all(isinstance(value, (int, float)) for value in salient)
            and 0.0 <= float(salient[0]) <= 1.0
            and 0.0 <= float(salient[1]) <= 1.0
        ):
            return (float(salient[0]), float(salient[1]))
        opencv = analysis.get("opencv_detection")
        bbox = opencv.get("bbox") if isinstance(opencv, dict) else None
        frame_size = analysis.get("frame_size_px")
        if (
            isinstance(bbox, (list, tuple))
            and len(bbox) == 4
            and all(isinstance(value, (int, float)) for value in bbox)
            and isinstance(frame_size, (list, tuple))
            and len(frame_size) == 2
            and frame_size[0] > 0
            and frame_size[1] > 0
        ):
            bx, by, bw, bh = (float(value) for value in bbox)
            return (
                (bx + bw / 2.0) / float(frame_size[0]),
                (by + bh / 2.0) / float(frame_size[1]),
            )
        return None

    async def _depth_target_local(
        self,
        vehicle_id: int,
        analysis: dict[str, Any],
    ) -> tuple[float, float, float] | None:
        """Local NED position of the target surface from the depth reading.

        The depth pixel hits the object surface; the standoff distance is
        measured from this surface point (the object's outside), so large
        objects keep the drone well clear instead of parking under them.
        """
        depth_target = analysis.get("depth_target")
        if not isinstance(depth_target, dict):
            return None
        distance = float(depth_target.get("distance_m") or 0.0)
        if distance <= 0.05:
            return None
        center = self._analysis_target_center(analysis)
        frame_size = analysis.get("frame_size_px")
        fov = analysis.get("camera_fov_degrees")
        if (
            center is None
            or not isinstance(frame_size, (list, tuple))
            or len(frame_size) != 2
            or fov is None
        ):
            return None
        link = self._runtime.link_for(vehicle_id)
        if link is None:
            return None
        snapshot = link.telemetry_snapshot()
        position = snapshot.local_position_ned_m
        attitude = snapshot.attitude_rpy_rad
        if position is None:
            return None
        yaw = float(attitude[2]) if attitude is not None else 0.0
        return target_from_depth(
            frame_width_px=int(frame_size[0]),
            frame_height_px=int(frame_size[1]),
            fov_degrees=float(fov),
            target_center_normalized=center,
            vehicle_position_ned=position,
            vehicle_yaw_rad=yaw,
            distance_m=distance,
        )

    async def _locate_visual_target(
        self,
        vehicle_id: int,
        analysis: dict[str, Any],
    ) -> tuple[float, float, float] | None:
        depth_local = await self._depth_target_local(vehicle_id, analysis)
        if depth_local is not None:
            return depth_local
        try:
            result = analysis.get("result")
            if not isinstance(result, dict):
                return None
            target = result.get("target")
            if not isinstance(target, dict):
                return None
            if target.get("found") is False:
                # The model explicitly says the target is absent; blob
                # fallbacks would navigate to a random salient spot.
                return None
            center_x = target.get("center_x")
            center_y = target.get("center_y")
            frame_size = analysis.get("frame_size_px")
            fov = analysis.get("camera_fov_degrees")
            if (
                not isinstance(frame_size, (list, tuple))
                or len(frame_size) != 2
                or fov is None
            ):
                return None
            if center_x is None or center_y is None:
                # Fall back to the deterministic salient blob (non-background
                # color) or the OpenCV palette blob when the VLM did not
                # report pixel coordinates.
                salient = analysis.get("salient_center")
                if (
                    isinstance(salient, (list, tuple))
                    and len(salient) == 2
                    and all(isinstance(v, (int, float)) for v in salient)
                    and 0.0 <= float(salient[0]) <= 1.0
                    and 0.0 <= float(salient[1]) <= 1.0
                ):
                    center_x = float(salient[0])
                    center_y = float(salient[1])
                opencv = analysis.get("opencv_detection")
                bbox = opencv.get("bbox") if isinstance(opencv, dict) else None
                if (
                    not isinstance(bbox, (list, tuple))
                    or len(bbox) != 4
                    or not all(isinstance(v, (int, float)) for v in bbox)
                ):
                    return None
                bx, by, bw, bh = (float(value) for value in bbox)
                if frame_size[0] <= 0 or frame_size[1] <= 0:
                    return None
                center_x = (bx + bw / 2.0) / float(frame_size[0])
                center_y = (by + bh / 2.0) / float(frame_size[1])
            if not (0.0 <= center_x <= 1.0 and 0.0 <= center_y <= 1.0):
                return None
            link = self._runtime.link_for(vehicle_id)
            if link is None:
                return None
            snapshot = link.telemetry_snapshot()
            position = snapshot.local_position_ned_m
            attitude = snapshot.attitude_rpy_rad
            if position is None:
                return None
            yaw = float(attitude[2]) if attitude is not None else 0.0
            return estimate_target_ned(
                frame_width_px=int(frame_size[0]),
                frame_height_px=int(frame_size[1]),
                fov_degrees=float(fov),
                target_center_normalized=(float(center_x), float(center_y)),
                vehicle_position_ned=position,
                vehicle_yaw_rad=yaw,
            )
        except (TypeError, ValueError):
            return None

    def _target_bearing_unit(
        self,
        analysis: dict[str, Any],
        yaw_rad: float,
    ) -> tuple[float, float] | None:
        """Unit NED direction (north, east) of the detected target center."""
        try:
            result = analysis.get("result")
            if not isinstance(result, dict):
                return None
            target = result.get("target")
            if not isinstance(target, dict):
                return None
            if target.get("found") is False:
                return None
            center_x = target.get("center_x")
            center_y = target.get("center_y")
            frame_size = analysis.get("frame_size_px")
            fov = analysis.get("camera_fov_degrees")
            if not isinstance(frame_size, (list, tuple)) or len(frame_size) != 2:
                return None
            if center_x is None or center_y is None:
                # Deterministic fallback: salient blob or OpenCV palette blob.
                salient = analysis.get("salient_center")
                if (
                    isinstance(salient, (list, tuple))
                    and len(salient) == 2
                    and all(isinstance(v, (int, float)) for v in salient)
                    and 0.0 <= float(salient[0]) <= 1.0
                    and 0.0 <= float(salient[1]) <= 1.0
                ):
                    center_x = float(salient[0])
                    center_y = float(salient[1])
                else:
                    opencv = analysis.get("opencv_detection")
                    bbox = opencv.get("bbox") if isinstance(opencv, dict) else None
                    if (
                        not isinstance(bbox, (list, tuple))
                        or len(bbox) != 4
                        or not all(isinstance(v, (int, float)) for v in bbox)
                        or frame_size[0] <= 0
                    ):
                        return None
                    bx, by, bw, bh = (float(value) for value in bbox)
                    center_x = (bx + bw / 2.0) / float(frame_size[0])
                    center_y = (by + bh / 2.0) / float(frame_size[1])
            if fov is None:
                return None
            horizontal = (float(center_x) - 0.5) * 2.0
            half = math.radians(float(fov)) / 2.0
            h_angle = horizontal * half
            dir_b_x = math.cos(h_angle)
            dir_b_y = math.sin(h_angle)
            dir_x = math.cos(yaw_rad) * dir_b_x - math.sin(yaw_rad) * dir_b_y
            dir_y = math.sin(yaw_rad) * dir_b_x + math.cos(yaw_rad) * dir_b_y
            norm = math.hypot(dir_x, dir_y)
            if norm <= 1e-6:
                return None
            return (dir_x / norm, dir_y / norm)
        except (TypeError, ValueError):
            return None

    async def _triangulate_visual_target(
        self,
        vehicle_id: int,
        service: FlightCommandService,
        analysis: dict[str, Any],
        probe_distance_m: float,
    ) -> tuple[float, float, float] | None:
        """Estimate the target's horizontal position from two bearings.

        Flies ``probe_distance_m`` forward along the current heading, takes a
        second visual analysis, then intersects the two bearing rays.  This
        works even when the flat-ground ray never meets the ground plane
        (level front camera at altitude), so the drone can still stop a
        requested distance in front of the target.
        """
        if probe_distance_m <= 0.0:
            return None
        link = self._runtime.link_for(vehicle_id)
        if link is None:
            return None
        start = await self._current_position(vehicle_id)
        snapshot = link.telemetry_snapshot()
        yaw1 = (
            float(snapshot.attitude_rpy_rad[2])
            if snapshot.attitude_rpy_rad is not None
            else 0.0
        )
        bearing1 = self._target_bearing_unit(analysis, yaw1)
        if bearing1 is None:
            return None
        # Probe perpendicular to the target ray so the two bearing rays
        # intersect strongly; flying along the target ray would leave them
        # (near-)parallel and make triangulation fail.
        probe_dir = (-bearing1[1], bearing1[0])
        probe_target = (
            start[0] + probe_dir[0] * probe_distance_m,
            start[1] + probe_dir[1] * probe_distance_m,
            start[2],
        )
        await self._await_handle(
            service.goto_local_ned(
                probe_target[0],
                probe_target[1],
                probe_target[2],
                yaw_rad=yaw1,
                expires_monotonic_s=self._clock() + _STEP_TIMEOUT_S,
            ),
            vehicle_id,
            "probe_move",
            _STEP_TIMEOUT_S,
        )
        analysis2 = await self.analyzer(
            vehicle_id,
            "重新定位目标：必须返回 target.center_x 与 "
            "target.center_y（绘制目标中心在画面中的归一化坐标 0-1），"
            "找不到目标则 found=false；不要只用文字描述",
        )
        snapshot2 = link.telemetry_snapshot()
        yaw2 = (
            float(snapshot2.attitude_rpy_rad[2])
            if snapshot2.attitude_rpy_rad is not None
            else yaw1
        )
        bearing2 = self._target_bearing_unit(analysis2, yaw2)
        if bearing2 is None:
            return None
        b = (probe_target[0], probe_target[1])
        point = _line_intersection(
            (start[0], start[1]),
            bearing1,
            b,
            bearing2,
        )
        if point is None:
            return None
        return (point[0], point[1], start[2])

    async def _fly_to_visual_standoff(
        self,
        vehicle_id: int,
        service: FlightCommandService,
        target_ned: tuple[float, float, float],
        standoff_distance_m: float,
        source_label: str,
    ) -> str:
        """Fly to the target (or ``standoff_distance_m`` in front of it)."""
        current = await self._current_position(vehicle_id)
        target_x, target_y = float(target_ned[0]), float(target_ned[1])
        tx, ty = target_x, target_y
        if standoff_distance_m > 0.0:
            tx, ty = _standoff_point(
                (target_x, target_y),
                (current[0], current[1]),
                standoff_distance_m,
            )
        # Face the target from the (standoff) point, not from the start.
        yaw_rad = math.atan2(target_y - ty, target_x - tx)
        await self._await_handle(
            service.goto_local_ned(
                tx,
                ty,
                current[2],
                yaw_rad=yaw_rad,
                expires_monotonic_s=self._clock() + _STEP_TIMEOUT_S,
            ),
            vehicle_id,
            "navigate_to_target",
            _STEP_TIMEOUT_S,
        )
        if standoff_distance_m > 0.0:
            return (
                f"；{source_label}，已飞往目标前方 "
                f"{standoff_distance_m:.0f} m ({tx:.1f}, {ty:.1f}) 并朝向目标"
            )
        return f"；{source_label}，已飞往目标 ({tx:.1f}, {ty:.1f})"

    async def _verify_target_in_view(
        self,
        vehicle_id: int,
        service: FlightCommandService,
        params: Mapping[str, Any],
        standoff_m: float,
    ) -> str | None:
        """Require the target back in frame and centered before the analyze
        step may complete (the blind fallback approach is not proof)."""
        prompt = str(
            params.get("prompt")
            or "识别画面中的目标物体（锥桶/球体）"
        )
        for _round in range(4):
            analysis = await self.analyzer(vehicle_id, prompt)
            center = self._analysis_target_center(analysis)
            target_ned = await self._locate_visual_target(vehicle_id, analysis)
            if (
                center is not None
                and abs(float(center[0]) - 0.5) <= 0.25
                and target_ned is not None
                and await self._target_reachable(vehicle_id, target_ned)
            ):
                target_ned = await self._recenter_and_relocalize(
                    vehicle_id,
                    service,
                    analysis,
                    params,
                    target_ned,
                    standoff_m,
                )
                if await self._target_reachable(vehicle_id, target_ned):
                    return await self._fly_to_visual_standoff(
                        vehicle_id,
                        service,
                        target_ned,
                        standoff_m,
                        "最终校验定位",
                    )
            coverage = self._frame_coverage_ratio(analysis)
            if coverage is not None and coverage > 0.55:
                backed = await self._back_away_from_target(
                    vehicle_id,
                    service,
                    analysis,
                )
                if backed:
                    continue
            analysis, target_ned = await self._search_visual_target(
                vehicle_id,
                service,
                analysis,
                params,
            )
            if (
                target_ned is not None
                and await self._target_reachable(vehicle_id, target_ned)
            ):
                target_ned = await self._recenter_and_relocalize(
                    vehicle_id,
                    service,
                    analysis,
                    params,
                    target_ned,
                    standoff_m,
                )
                if await self._target_reachable(vehicle_id, target_ned):
                    return await self._fly_to_visual_standoff(
                        vehicle_id,
                        service,
                        target_ned,
                        standoff_m,
                        "搜索后定位",
                    )
        return None

    def _frame_coverage_ratio(
        self,
        analysis: dict[str, Any],
    ) -> float | None:
        """Fraction of the frame covered by the detected target blob."""
        cross = analysis.get("cross_validation")
        if isinstance(cross, dict):
            detection = cross.get("detection")
            if isinstance(detection, dict):
                ratio = detection.get("coverage_ratio")
                if isinstance(ratio, (int, float)) and ratio > 0.0:
                    return float(ratio)
        opencv = analysis.get("opencv_detection")
        frame_size = analysis.get("frame_size_px")
        bbox = opencv.get("bbox") if isinstance(opencv, dict) else None
        if (
            isinstance(bbox, (list, tuple))
            and len(bbox) == 4
            and all(isinstance(value, (int, float)) for value in bbox)
            and isinstance(frame_size, (list, tuple))
            and len(frame_size) == 2
            and frame_size[0] > 0
            and frame_size[1] > 0
        ):
            bx, by, bw, bh = (float(value) for value in bbox)
            return min(
                1.0,
                (bw * bh) / (float(frame_size[0]) * float(frame_size[1])),
            )
        return None

    async def _back_away_from_target(
        self,
        vehicle_id: int,
        service: FlightCommandService,
        analysis: dict[str, Any],
    ) -> bool:
        """Move 6 m away from the target direction when it fills the frame."""
        link = self._runtime.link_for(vehicle_id)
        if link is None:
            return False
        snapshot = link.telemetry_snapshot()
        position = snapshot.local_position_ned_m
        if position is None:
            return False
        yaw = (
            float(snapshot.attitude_rpy_rad[2])
            if snapshot.attitude_rpy_rad is not None
            else 0.0
        )
        center = self._analysis_target_center(analysis)
        if center is not None:
            frame_size = analysis.get("frame_size_px")
            fov = analysis.get("camera_fov_degrees")
            if (
                isinstance(frame_size, (list, tuple))
                and len(frame_size) == 2
                and fov is not None
            ):
                horizontal = (float(center[0]) - 0.5) * 2.0
                h_angle = horizontal * math.radians(float(fov)) / 2.0
                dir_b_x = math.cos(h_angle)
                dir_b_y = math.sin(h_angle)
                dir_x = math.cos(yaw) * dir_b_x - math.sin(yaw) * dir_b_y
                dir_y = math.sin(yaw) * dir_b_x + math.cos(yaw) * dir_b_y
                norm = math.hypot(dir_x, dir_y)
                if norm > 1e-6:
                    dx, dy = dir_x / norm, dir_y / norm
                    await self._await_handle(
                        service.goto_local_ned(
                            position[0] - dx * 6.0,
                            position[1] - dy * 6.0,
                            position[2],
                            yaw_rad=math.atan2(dy, dx),
                            expires_monotonic_s=(
                                self._clock() + _STEP_TIMEOUT_S
                            ),
                        ),
                        vehicle_id,
                        "back_away",
                        _STEP_TIMEOUT_S,
                    )
                    return True
        await self._await_handle(
            service.goto_local_ned(
                position[0] - math.cos(yaw) * 6.0,
                position[1] - math.sin(yaw) * 6.0,
                position[2],
                yaw_rad=yaw,
                expires_monotonic_s=self._clock() + _STEP_TIMEOUT_S,
            ),
            vehicle_id,
            "back_away",
            _STEP_TIMEOUT_S,
        )
        return True

    async def _target_reachable(
        self,
        vehicle_id: int,
        target_ned: tuple[float, float, float] | None,
    ) -> bool:
        """Sanity guard: only navigate to targets within a bounded range.

        The lower bound rejects depth readings that hit the ground or a
        nearby object (a credible target is at least 3 m away)."""
        if target_ned is None:
            return False
        current = await self._current_position(vehicle_id)
        distance = math.hypot(
            float(target_ned[0]) - current[0],
            float(target_ned[1]) - current[1],
        )
        return 2.0 <= distance <= _MAX_TARGET_RANGE_M

    async def _recenter_and_relocalize(
        self,
        vehicle_id: int,
        service: FlightCommandService,
        analysis: dict[str, Any],
        params: Mapping[str, Any],
        target_ned: tuple[float, float, float],
        standoff_m: float = 0.0,
    ) -> tuple[float, float, float]:
        """Move toward the target in short legs, re-analyzing until the target
        sits near the frame center (edge/occluded pixels localize poorly)."""
        prompt = str(
            params.get("prompt")
            or "识别画面中的目标物体（锥桶/球体）"
        )
        current_best = target_ned
        for _attempt in range(3):
            current = await self._current_position(vehicle_id)
            bearing = math.atan2(
                float(current_best[1]) - current[1],
                float(current_best[0]) - current[0],
            )
            # Never let the recenter leg overshoot the requested standoff:
            # approaching too close fills the frame, blinds the VLM and can
            # make the reachability guard discard a valid target.
            distance_m = math.hypot(
                float(current_best[0]) - current[0],
                float(current_best[1]) - current[1],
            )
            leg_m = _recenter_leg_m(distance_m, standoff_m)
            if leg_m >= 0.75:
                try:
                    await self._await_handle(
                        service.goto_local_ned(
                            current[0] + math.cos(bearing) * leg_m,
                            current[1] + math.sin(bearing) * leg_m,
                            current[2],
                            yaw_rad=bearing,
                            expires_monotonic_s=(
                                self._clock() + _STEP_TIMEOUT_S
                            ),
                        ),
                        vehicle_id,
                        "recenter_yaw",
                        _STEP_TIMEOUT_S,
                    )
                except FleetMissionError as exc:
                    raise FleetMissionError(
                        f"{exc} [pos=({current[0]:.1f},{current[1]:.1f}) "
                        f"target=({current_best[0]:.1f},"
                        f"{current_best[1]:.1f})]"
                    ) from exc
            analysis = await self.analyzer(vehicle_id, prompt)
            refined = await self._locate_visual_target(vehicle_id, analysis)
            if refined is not None and await self._target_reachable(vehicle_id, refined):
                current_best = refined
                center = self._analysis_target_center(analysis)
                if center is not None and abs(float(center[0]) - 0.5) <= 0.25:
                    return current_best
                continue
            # Re-analysis was not credible (target lost / ground hit): keep
            # the original target instead of chasing an invalid point.
            return current_best
        return current_best

    async def _search_visual_target(
        self,
        vehicle_id: int,
        service: FlightCommandService,
        analysis: dict[str, Any],
        params: Mapping[str, Any],
    ) -> tuple[dict[str, Any], tuple[float, float, float] | None]:
        """Rotate the aircraft in place and re-analyze until the target is
        localized, so the closed loop works from any starting heading."""
        search_steps = int(params.get("search_steps", 6) or 6)
        step_deg = float(params.get("search_step_deg", 60.0) or 60.0)
        if search_steps <= 0 or step_deg <= 0.0:
            return analysis, None
        prompt = str(
            params.get("prompt")
            or "识别画面中的目标物体（锥桶/球体）"
        )
        leg_m = float(params.get("search_leg_m", 3.0) or 3.0)
        current = await self._current_position(vehicle_id)
        link = self._runtime.link_for(vehicle_id)
        if link is None:
            return analysis, None
        snapshot = link.telemetry_snapshot()
        yaw = (
            float(snapshot.attitude_rpy_rad[2])
            if snapshot.attitude_rpy_rad is not None
            else 0.0
        )
        for _step in range(max(0, search_steps)):
            yaw += math.radians(step_deg)
            # A short moving leg completes reliably (yaw-in-place gotos do
            # rotate but their physical-completion check rarely fires).
            await self._await_handle(
                service.goto_local_ned(
                    current[0] + math.cos(yaw) * leg_m,
                    current[1] + math.sin(yaw) * leg_m,
                    current[2],
                    yaw_rad=yaw,
                    expires_monotonic_s=self._clock() + _STEP_TIMEOUT_S,
                ),
                vehicle_id,
                "search_yaw",
                _STEP_TIMEOUT_S,
            )
            analysis = await self.analyzer(vehicle_id, prompt)
            target = await self._locate_visual_target(vehicle_id, analysis)
            self._results.append(
                {
                    "vehicle_id": vehicle_id,
                    "step": "search_probe",
                    "status": "completed",
                    "detail": (
                        f"\u641c\u7d22\u65b9\u4f4d {math.degrees(yaw):.0f}\u00b0 "
                        f"found={target is not None}"
                    ),
                    "at_monotonic_s": round(self._clock(), 2),
                }
            )
            if target is not None:
                return analysis, target
        return analysis, None

    async def _ensure_guided(
        self,
        vehicle_id: int,
        service: FlightCommandService,
    ) -> None:
        """Make sure the vehicle is in GUIDED before issuing position targets.

        Hold steps leave the vehicle in LOITER, where position targets are
        ignored; switching back to GUIDED keeps the visual-navigation chain
        working in mixed fleet+vision plans.
        """
        link = self._runtime.link_for(vehicle_id)
        if link is None:
            return
        snapshot = link.telemetry_snapshot()
        if snapshot.mode == "GUIDED":
            return
        await self._await_handle(
            service.set_mode("GUIDED"),
            vehicle_id,
            "set_mode_guided",
            _STEP_TIMEOUT_S,
        )

    async def _navigate_to_visual_target(
        self,
        vehicle_id: int,
        service: FlightCommandService,
        analysis: dict[str, Any],
        params: Mapping[str, Any],
    ) -> str:
        standoff = max(0.0, float(params.get("standoff_distance_m", 0.0) or 0.0))
        probe = max(0.0, float(params.get("probe_distance_m", 8.0) or 8.0))
        await self._ensure_guided(vehicle_id, service)
        target_ned = await self._locate_visual_target(vehicle_id, analysis)
        if not await self._target_reachable(vehicle_id, target_ned):
            target_ned = None
        if target_ned is None:
            # Target not in view or not localizable: rotate and re-analyze.
            analysis, target_ned = await self._search_visual_target(
                vehicle_id,
                service,
                analysis,
                params,
            )
            if not await self._target_reachable(vehicle_id, target_ned):
                target_ned = None
        if target_ned is not None:
            target_ned = await self._recenter_and_relocalize(
                vehicle_id,
                service,
                analysis,
                params,
                target_ned,
                standoff,
            )
            if not await self._target_reachable(vehicle_id, target_ned):
                target_ned = None
        if target_ned is not None:
            return await self._fly_to_visual_standoff(
                vehicle_id,
                service,
                target_ned,
                standoff,
                "已定位目标",
            )
        if probe > 0.0:
            triangulated = await self._triangulate_visual_target(
                vehicle_id,
                service,
                analysis,
                probe,
            )
            if triangulated is not None:
                return await self._fly_to_visual_standoff(
                    vehicle_id,
                    service,
                    triangulated,
                    standoff,
                    "三角测距定位",
                )
        approach = await self._approach_target_direction(
            vehicle_id,
            analysis,
            service,
            approach_distance_m=float(params.get("approach_distance_m", 15.0)),
        )
        if approach is None:
            return "；未能定位目标，未导航"
        verified = await self._verify_target_in_view(
            vehicle_id,
            service,
            params,
            standoff,
        )
        if verified is not None:
            return verified
        raise FleetMissionError(
            "最终视觉校验未通过：目标未回到画面中央，停止继续飞行"
        )

    async def _ensure_armed(
        self,
        vehicle_ids: Sequence[int],
        services: dict[int, FlightCommandService],
    ) -> None:
        for vehicle_id in vehicle_ids:
            link = self._runtime.link_for(vehicle_id)
            if link is None:
                raise FleetMissionError(f"飞机 {vehicle_id} 链路不可用")
            snapshot = link.telemetry_snapshot()
            if snapshot.armed is True:
                continue
            await self._await_handle(
                services[vehicle_id].arm(),
                vehicle_id,
                "arm",
                _STEP_TIMEOUT_S,
            )

    async def _wait_takeoff_ready(self, vehicle_id: int) -> None:
        """Wait until the FCU reports ON_GROUND before sending NAV_TAKEOFF.

        ArduCopter ignores NAV_TAKEOFF while its landing detector has not
        settled; this short bounded check keeps the first attempt well-timed
        without blocking an already-ready FCU.
        """
        deadline = self._clock() + _TAKEOFF_SETTLE_WAIT_S
        while self._clock() < deadline:
            link = self._runtime.link_for(vehicle_id)
            if link is None:
                return
            snapshot = link.telemetry_snapshot()
            relative_altitude = snapshot.relative_altitude_m
            landed_state = snapshot.landed_state
            if relative_altitude is not None and relative_altitude > 0.5:
                return  # already airborne
            if (
                landed_state == MavLandedState.ON_GROUND
                and relative_altitude is not None
                and abs(relative_altitude) < 0.5
            ):
                return  # settled on the ground
            await asyncio.sleep(1.0)

    async def _takeoff_with_retry(
        self,
        vehicle_id: int,
        service: FlightCommandService,
        altitude_m: float,
    ) -> None:
        """Take off with a bounded retry loop.

        The FCU can silently ignore NAV_TAKEOFF before its landing detector
        settles; each attempt waits for physical completion inside a short
        window and the loop retries after a settle gap instead of failing the
        whole plan on the first rejection.
        """
        await self._wait_takeoff_ready(vehicle_id)
        last_error: FleetMissionError | None = None
        force_rearm = False
        for attempt in range(1, _TAKEOFF_MAX_ATTEMPTS + 1):
            try:
                link = self._runtime.link_for(vehicle_id)
                if link is not None:
                    snapshot = link.telemetry_snapshot()
                    if force_rearm or snapshot.armed is not True:
                        # A failed attempt may leave the FCU disarmed while
                        # the telemetry snapshot is stale; re-arm so the next
                        # NAV_TAKEOFF is not rejected with "motors not armed".
                        await self._await_handle(
                            service.arm(),
                            vehicle_id,
                            "arm_retry",
                            _STEP_TIMEOUT_S,
                        )
                        force_rearm = False
                await self._await_handle(
                    service.takeoff(
                        altitude_m,
                        expires_monotonic_s=(
                            self._clock() + _TAKEOFF_ATTEMPT_WINDOW_S
                        ),
                    ),
                    vehicle_id,
                    "takeoff",
                    _TAKEOFF_ATTEMPT_WINDOW_S,
                )
                return
            except FleetMissionError as exc:
                last_error = exc
                if attempt >= _TAKEOFF_MAX_ATTEMPTS:
                    raise FleetMissionError(
                        f"飞机 {vehicle_id} 起飞在 {_TAKEOFF_MAX_ATTEMPTS} "
                        f"次尝试后失败: {exc}"
                    ) from exc
                force_rearm = True
                self._results.append(
                    {
                        "vehicle_id": vehicle_id,
                        "step": "takeoff_retry",
                        "status": "retrying",
                        "detail": (
                            f"第 {attempt} 次起飞未确认物理完成，"
                            f"{int(_TAKEOFF_RETRY_GAP_S)}s 后重试"
                        ),
                        "at_monotonic_s": round(self._clock(), 2),
                    }
                )
                await asyncio.sleep(_TAKEOFF_RETRY_GAP_S)
        raise FleetMissionError(  # pragma: no cover - defensive
            f"飞机 {vehicle_id} 起飞失败: {last_error}"
        )

    async def _await_handle(
        self,
        handle: Any,
        vehicle_id: int,
        step: str,
        timeout_s: float,
    ) -> None:
        self._check_cancel()
        try:
            result = await asyncio.wait_for(handle.result, timeout=timeout_s)
        except asyncio.TimeoutError as exc:
            raise FleetMissionError(f"飞机 {vehicle_id} 步骤 {step} 超时") from exc
        require_completed_command(result, vehicle_id, step)
        self._results.append(
            {
                "vehicle_id": vehicle_id,
                "step": step,
                "status": result.status.value,
                "detail": str(result.detail),
                "at_monotonic_s": round(self._clock(), 2),
            }
        )

    async def _safety_land(self) -> None:
        if not self._services:
            return
        self._stage = "安全降落"
        await asyncio.gather(
            *(
                self._await_handle(
                    service.land(),
                    vehicle_id,
                    "safety_land",
                    _LAND_TIMEOUT_S,
                )
                for vehicle_id, service in self._services.items()
            ),
            return_exceptions=True,
        )

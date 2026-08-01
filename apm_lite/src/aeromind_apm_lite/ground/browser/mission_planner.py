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
from typing import Any, Sequence

from aeromind_apm_lite.common.formation import FormationType
from aeromind_apm_lite.ground.browser.fleet_executor import (
    FleetMissionError,
    fly_formation,
)
from aeromind_apm_lite.ground.browser.runtime import ManualRuntimeMode
from aeromind_apm_lite.ground.browser.visual_navigation import estimate_target_ned
from aeromind_apm_lite.onboard.flight_commands import FlightCommandService
from aeromind_apm_lite.onboard.mavlink.models import CommandStatus

_STEP_TIMEOUT_S = 120.0
_TAKEOFF_TIMEOUT_S = 90.0
_LAND_TIMEOUT_S = 60.0

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
            await for_each(
                "takeoff",
                lambda vid: services[vid].takeoff(altitude),
                _TAKEOFF_TIMEOUT_S,
            )
        elif action == "hold":
            await for_each(
                "hold",
                lambda vid: services[vid].set_mode("LOITER"),
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
                target_ned = await self._locate_visual_target(
                    vehicle_id,
                    analysis,
                )
                if target_ned is None:
                    approach = await self._approach_target_direction(
                        vehicle_id,
                        analysis,
                        services[vehicle_id],
                    )
                    if approach is None:
                        navigate_detail = "；未能定位目标，未导航"
                    else:
                        navigate_detail = (
                            "；目标较远，已朝目标方向前进 "
                            f"{approach:.0f} m（建议再次分析）"
                        )
                else:
                    current = await self._current_position(vehicle_id)
                    await self._await_handle(
                        services[vehicle_id].goto_local_ned(
                            target_ned[0],
                            target_ned[1],
                            current[2],
                            expires_monotonic_s=self._clock() + _STEP_TIMEOUT_S,
                        ),
                        vehicle_id,
                        "navigate_to_target",
                        _STEP_TIMEOUT_S,
                    )
                    navigate_detail = (
                        f"；已飞往目标 ({target_ned[0]:.1f}, {target_ned[1]:.1f})"
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
                    expires_monotonic_s=self._clock() + _STEP_TIMEOUT_S,
                ),
                vehicle_id,
                "approach_target",
                _STEP_TIMEOUT_S,
            )
            return approach_distance_m
        except (TypeError, ValueError):
            return None

    async def _locate_visual_target(
        self,
        vehicle_id: int,
        analysis: dict[str, Any],
    ) -> tuple[float, float, float] | None:
        try:
            result = analysis.get("result")
            if not isinstance(result, dict):
                return None
            target = result.get("target")
            if not isinstance(target, dict):
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
        if result.status not in {
            CommandStatus.COMPLETED,
            CommandStatus.PREEMPTED,
        }:
            raise FleetMissionError(
                f"飞机 {vehicle_id} 步骤 {step} 失败: {result.detail}"
            )
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

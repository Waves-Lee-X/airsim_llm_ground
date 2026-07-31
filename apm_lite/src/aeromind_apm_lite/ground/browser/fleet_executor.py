"""Fleet mission executor for the browser ground station.

Drives the SITL simulation vehicles through takeoff -> formation -> land using
the runtime's own :class:`FlightCommandService` (one-owner ApmLink), so the
web ground station can run formation tests without a separate acceptance
process. Only simulation (SITL) vehicles are allowed; real vehicles keep the
onboard allowlist gate and are never touched by this executor.
"""

from __future__ import annotations

import asyncio
import time
from dataclasses import dataclass
from typing import Any

from aeromind_apm_lite.common.formation import FormationType, formation_offsets
from aeromind_apm_lite.ground.browser.runtime import ManualRuntimeMode
from aeromind_apm_lite.onboard.flight_commands import FlightCommandService
from aeromind_apm_lite.onboard.mavlink.models import CommandStatus

_STEP_TIMEOUT_S = 120.0
_TAKEOFF_TIMEOUT_S = 90.0
_LAND_TIMEOUT_S = 60.0


class FleetMissionError(RuntimeError):
    """Raised when a fleet mission cannot start or fails mid-flight."""


@dataclass(frozen=True)
class FleetMissionConfig:
    formation: FormationType
    leader_target_map_m: tuple[float, float, float]
    spacing_m: float = 3.0
    altitude_m: float = 2.0
    hold_s: float = 3.0
    vehicle_ids: tuple[int, ...] = (1, 2, 3, 4)
    land_after: bool = True


class FleetMissionRunner:
    """Run one fleet mission at a time and expose progress to the browser."""

    def __init__(self, runtime: Any, *, clock: Any = time.monotonic) -> None:
        self._runtime = runtime
        self._clock = clock
        self._task: asyncio.Task[None] | None = None
        self._cancel_event = asyncio.Event()
        self._phase = "idle"
        self._stage = ""
        self._vehicles: list[dict[str, Any]] = []
        self._error: str | None = None
        self._config: FleetMissionConfig | None = None
        self._started_monotonic_s: float | None = None
        self._results: list[dict[str, Any]] = []

    @property
    def busy(self) -> bool:
        return self._task is not None and not self._task.done()

    def status_payload(self) -> dict[str, Any]:
        return {
            "phase": self._phase,
            "stage": self._stage,
            "busy": self.busy,
            "error": self._error,
            "vehicles": [dict(item) for item in self._vehicles],
            "results": [dict(item) for item in self._results],
            "config": (
                {
                    "formation": self._config.formation.value,
                    "leader_target_map_m": list(
                        self._config.leader_target_map_m
                    ),
                    "spacing_m": self._config.spacing_m,
                    "altitude_m": self._config.altitude_m,
                    "hold_s": self._config.hold_s,
                    "vehicle_ids": list(self._config.vehicle_ids),
                }
                if self._config is not None
                else None
            ),
        }

    async def start(self, config: FleetMissionConfig) -> dict[str, Any]:
        if self.busy:
            raise FleetMissionError(
                "编队任务正在执行中，请先取消或等待完成"
            )
        self._validate_config(config)
        for vehicle_id in config.vehicle_ids:
            try:
                runtime = self._runtime.runtime_for(vehicle_id)
            except KeyError as exc:
                raise FleetMissionError(
                    f"飞机 {vehicle_id} 未注册"
                ) from exc
            if runtime.config.mode != ManualRuntimeMode.SITL:
                raise FleetMissionError(
                    f"飞机 {vehicle_id} 不是仿真机，编队执行仅支持仿真"
                )
            link = runtime.link
            if link is None or not getattr(link, "ready", False):
                raise FleetMissionError(f"飞机 {vehicle_id} 链路未就绪")
        self._config = config
        self._cancel_event = asyncio.Event()
        self._phase = "starting"
        self._stage = "检查链路"
        self._error = None
        self._vehicles = [
            {
                "vehicle_id": vehicle_id,
                "step": "pending",
                "state": "pending",
                "detail": "",
            }
            for vehicle_id in config.vehicle_ids
        ]
        self._started_monotonic_s = self._clock()
        self._results = []
        self._task = asyncio.create_task(
            self._run(config),
            name="fleet-mission",
        )
        return self.status_payload()

    async def cancel(self) -> dict[str, Any]:
        if self.busy:
            self._cancel_event.set()
            self._phase = "cancelling"
            self._stage = "正在降落并解锁"
        return self.status_payload()

    def _validate_config(self, config: FleetMissionConfig) -> None:
        if not 2 <= len(config.vehicle_ids) <= 8:
            raise FleetMissionError("编队任务需要 2 到 8 架飞机")
        if not 0.5 <= config.spacing_m <= 50.0:
            raise FleetMissionError("间距必须在 0.5 到 50 米之间")
        if not 0.5 <= config.altitude_m <= 20.0:
            raise FleetMissionError("高度必须在 0.5 到 20 米之间")
        if not 0.0 <= config.hold_s <= 120.0:
            raise FleetMissionError("保持时间必须在 0 到 120 秒之间")
        leader = config.leader_target_map_m
        if len(leader) != 3 or not all(
            isinstance(value, (int, float)) for value in leader
        ):
            raise FleetMissionError("领机目标必须是 (北, 东, 下) 三个数值")

    def _vehicle(self, vehicle_id: int) -> dict[str, Any]:
        for item in self._vehicles:
            if item["vehicle_id"] == vehicle_id:
                return item
        raise FleetMissionError(f"vehicle {vehicle_id} is not part of the mission")

    def _set_step(self, vehicle_id: int, step: str, detail: str = "") -> None:
        self._vehicle(vehicle_id).update(
            {"step": step, "state": "running", "detail": detail}
        )

    def _finish_step(self, vehicle_id: int, detail: str = "") -> None:
        self._vehicle(vehicle_id).update(
            {"step": "done", "state": "done", "detail": detail}
        )

    async def _run(self, config: FleetMissionConfig) -> None:
        try:
            services: dict[int, FlightCommandService] = {}
            for vehicle_id in config.vehicle_ids:
                try:
                    runtime = self._runtime.runtime_for(vehicle_id)
                except KeyError as exc:
                    raise FleetMissionError(
                        f"飞机 {vehicle_id} 未注册"
                    ) from exc
                if runtime.config.mode != ManualRuntimeMode.SITL:
                    raise FleetMissionError(
                        f"飞机 {vehicle_id} 不是仿真机，编队执行仅支持仿真"
                    )
                link = runtime.link
                if link is None or not getattr(link, "ready", False):
                    raise FleetMissionError(
                        f"飞机 {vehicle_id} 链路未就绪"
                    )
                services[vehicle_id] = FlightCommandService(link)

            self._phase = "running"
            self._stage = "起飞阶段"
            for vehicle_id in config.vehicle_ids:
                self._check_cancel()
                service = services[vehicle_id]
                self._set_step(vehicle_id, "set_mode", "进入 GUIDED")
                await self._await_handle(
                    service.set_mode("GUIDED"),
                    vehicle_id,
                    "set_mode",
                    _STEP_TIMEOUT_S,
                )
                self._set_step(vehicle_id, "arm", "解锁")
                await self._await_handle(
                    service.arm(),
                    vehicle_id,
                    "arm",
                    _STEP_TIMEOUT_S,
                )
                self._set_step(
                    vehicle_id,
                    "takeoff",
                    f"起飞到 {config.altitude_m} m",
                )
                await self._await_handle(
                    service.takeoff(config.altitude_m),
                    vehicle_id,
                    "takeoff",
                    _TAKEOFF_TIMEOUT_S,
                )
                self._finish_step(vehicle_id, "已起飞")

            self._stage = "编队飞行"
            offsets = formation_offsets(
                config.formation,
                len(config.vehicle_ids),
                config.spacing_m,
            )
            leader = config.leader_target_map_m
            for index, vehicle_id in enumerate(config.vehicle_ids):
                self._check_cancel()
                service = services[vehicle_id]
                offset = offsets[index]
                target = (
                    leader[0] + offset[0],
                    leader[1] + offset[1],
                    -config.altitude_m,
                )
                self._set_step(
                    vehicle_id,
                    "goto",
                    f"飞往槽位 ({target[0]:.1f}, {target[1]:.1f})",
                )
                await self._await_handle(
                    service.goto_local_ned(
                        target[0],
                        target[1],
                        target[2],
                        expires_monotonic_s=self._clock() + _STEP_TIMEOUT_S,
                    ),
                    vehicle_id,
                    "goto",
                    _STEP_TIMEOUT_S,
                )
                self._finish_step(vehicle_id, "已到位")

            self._stage = f"保持队形 {config.hold_s} s"
            for vehicle_id in config.vehicle_ids:
                self._set_step(vehicle_id, "hold", "保持队形")
            if config.hold_s > 0:
                await self._sleep_or_cancel(config.hold_s)
            for vehicle_id in config.vehicle_ids:
                self._finish_step(vehicle_id, "编队完成")

            if config.land_after:
                self._stage = "降落阶段"
                for vehicle_id in config.vehicle_ids:
                    self._set_step(vehicle_id, "land", "降落")
                await asyncio.gather(
                    *(
                        self._await_handle(
                            services[vehicle_id].land(),
                            vehicle_id,
                            "land",
                            _LAND_TIMEOUT_S,
                        )
                        for vehicle_id in config.vehicle_ids
                    )
                )
                for vehicle_id in config.vehicle_ids:
                    self._finish_step(vehicle_id, "已降落")
                await self._sleep_or_cancel(5.0)
                self._stage = "收尾（解锁）"
                await asyncio.gather(
                    *(
                        self._await_handle(
                            services[vehicle_id].disarm(),
                            vehicle_id,
                            "disarm",
                            _STEP_TIMEOUT_S,
                        )
                        for vehicle_id in config.vehicle_ids
                    ),
                    return_exceptions=True,
                )
                for vehicle_id in config.vehicle_ids:
                    self._vehicle(vehicle_id)["detail"] = "已降落并解锁"

            self._phase = "done" if not self._cancel_event.is_set() else "cancelled"
            self._stage = "编队任务完成" if self._phase == "done" else "已取消"
        except asyncio.CancelledError:
            self._phase = "cancelled"
            self._stage = "任务被取消"
            raise
        except FleetMissionError as exc:
            self._phase = "failed"
            self._stage = "任务失败"
            self._error = str(exc)
            await self._safety_land(services if "services" in locals() else {})
        except Exception as exc:  # pragma: no cover - defensive
            self._phase = "failed"
            self._stage = "任务失败"
            self._error = f"{type(exc).__name__}: {exc}"
            await self._safety_land(services if "services" in locals() else {})

    def _check_cancel(self) -> None:
        if self._cancel_event.is_set():
            raise FleetMissionError("编队任务已取消")

    async def _sleep_or_cancel(self, seconds: float) -> None:
        try:
            await asyncio.wait_for(self._cancel_event.wait(), timeout=seconds)
        except asyncio.TimeoutError:
            return
        raise FleetMissionError("编队任务已取消")

    async def _await_handle(
        self,
        handle: Any,
        vehicle_id: int,
        step: str,
        timeout_s: float,
    ) -> Any:
        self._check_cancel()
        try:
            result = await asyncio.wait_for(handle.result, timeout=timeout_s)
        except asyncio.TimeoutError as exc:
            raise FleetMissionError(
                f"飞机 {vehicle_id} 步骤 {step} 超时"
            ) from exc
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
        self._vehicle(vehicle_id)["detail"] = (
            f"{step} -> {result.status.value}: {result.detail}"
        )
        return result

    async def _safety_land(self, services: dict[int, FlightCommandService]) -> None:
        if not services:
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
                for vehicle_id, service in services.items()
            ),
            return_exceptions=True,
        )

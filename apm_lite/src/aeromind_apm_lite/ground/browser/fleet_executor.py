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
from typing import Any, Sequence

from aeromind_apm_lite.common.coordinates import (
    GeodeticPosition,
    Vec3,
    enu_to_geodetic,
    geodetic_to_local_ned,
)
from aeromind_apm_lite.common.formation import FormationType, formation_offsets
from aeromind_apm_lite.ground.browser.airsim_settings import AirSimSpawnMap
from aeromind_apm_lite.ground.browser.runtime import ManualRuntimeMode
from aeromind_apm_lite.onboard.flight_commands import FlightCommandService
from aeromind_apm_lite.onboard.mavlink.models import CommandStatus

_STEP_TIMEOUT_S = 120.0
_DEMO_MAP_ORIGIN = GeodeticPosition(47.641468, -122.140165, 0.0)
_TAKEOFF_TIMEOUT_S = 90.0
_LAND_TIMEOUT_S = 60.0


class FleetMissionError(RuntimeError):
    """Raised when a fleet mission cannot start or fails mid-flight."""


def require_completed_command(result: Any, vehicle_id: int, step: str) -> None:
    """Reject command outcomes without confirmed physical completion."""
    if result.status != CommandStatus.COMPLETED:
        raise FleetMissionError(
            f"飞机 {vehicle_id} 步骤 {step} 失败: {result.detail}"
        )


@dataclass(frozen=True)
class FleetMissionConfig:
    formation: FormationType
    leader_target_map_m: tuple[float, float, float]
    spacing_m: float = 3.0
    altitude_m: float = 2.0
    hold_s: float = 3.0
    vehicle_ids: tuple[int, ...] = (1, 2, 3, 4)
    land_after: bool = True
    formations: tuple[FormationType, ...] | None = None


def map_to_vehicle_local_ned(
    map_point: tuple[float, float, float],
    origin: GeodeticPosition,
    home: GeodeticPosition,
) -> tuple[float, float, float]:
    """Convert a map-frame (north, east, down) point to vehicle-local NED.

    The map frame is anchored at ``origin`` (the fleet reference home).  Every
    SITL vehicle has its own home (spawn point), so one physical target is a
    different local-NED point for each vehicle.
    """
    offset_enu = Vec3(map_point[1], map_point[0], -map_point[2])
    target = enu_to_geodetic(offset_enu, origin)
    ned = geodetic_to_local_ned(target, home)
    return (float(ned.x), float(ned.y), float(ned.z))


async def fly_formation(
    services: dict[int, FlightCommandService],
    runtime: Any,
    vehicle_ids: Sequence[int],
    formation: FormationType,
    leader_target_map_m: tuple[float, float, float],
    spacing_m: float,
    altitude_m: float,
    *,
    clock: Any = time.monotonic,
    step_timeout_s: float = _STEP_TIMEOUT_S,
    on_step: Any = None,
) -> list[dict[str, Any]]:
    """Fly ``vehicle_ids`` into ``formation`` with vertical-lane transitions.

    Every vehicle first climbs into its own altitude lane (1.5 m apart), then
    morphs horizontally inside the lane, then levels off at the formation
    altitude, so crossing paths stay vertically separated.  Slot targets are
    converted from the shared map frame into each vehicle's local NED using
    its own home, so the physical layout matches the intended shape.
    Returns per-step command results for evidence.
    """
    results: list[dict[str, Any]] = []
    ids = tuple(int(vehicle_id) for vehicle_id in vehicle_ids)
    offsets = formation_offsets(formation, len(ids), spacing_m)

    async def vehicle_origin(vehicle_id: int) -> GeodeticPosition | None:
        """Derive the vehicle's local-NED origin from live telemetry.

        local NED = current position - origin (as NED vectors), so the
        origin is ``enu_to_geodetic((-east, -north, +down), current_gps)``.
        The EKF origin is fixed at FCU boot, so this stays valid for the
        whole flight no matter where AirSim currently places the vehicles
        or how ArduPilot re-anchors the HOME_POSITION message.
        """
        link = runtime.link_for(vehicle_id)
        if link is None:
            raise FleetMissionError(f"飞机 {vehicle_id} 链路不可用")
        snapshot = link.telemetry_snapshot()
        gps = snapshot.global_position_deg_m
        local = snapshot.local_position_ned_m
        if not (
            gps is not None
            and len(gps) >= 3
            and local is not None
            and len(local) >= 3
            and all(
                isinstance(value, (int, float))
                for value in (*gps[:3], *local[:3])
            )
        ):
            return None
        try:
            current = GeodeticPosition(
                float(gps[0]),
                float(gps[1]),
                float(gps[2]),
            )
            return enu_to_geodetic(
                Vec3(
                    -float(local[1]),
                    -float(local[0]),
                    float(local[2]),
                ),
                current,
            )
        except (TypeError, ValueError):
            return None

    async def telemetry_home(vehicle_id: int) -> GeodeticPosition | None:
        link = runtime.link_for(vehicle_id)
        if link is None:
            raise FleetMissionError(f"飞机 {vehicle_id} 链路不可用")
        snapshot = link.telemetry_snapshot()
        for candidate in (
            snapshot.home_position_deg_m,
            snapshot.global_position_deg_m,
        ):
            if (
                candidate is not None
                and len(candidate) >= 3
                and all(
                    isinstance(value, (int, float)) for value in candidate[:3]
                )
            ):
                return GeodeticPosition(
                    float(candidate[0]),
                    float(candidate[1]),
                    float(candidate[2]),
                )
        return None

    spawn_map = AirSimSpawnMap()
    origins: dict[int, GeodeticPosition] = {}
    anchor: GeodeticPosition | None = None
    for vehicle_id in ids:
        origin = await vehicle_origin(vehicle_id)
        if origin is None:
            # Fall back to the AirSim spawn layout, then telemetry home.
            origin = spawn_map.home_for(vehicle_id)
        if origin is None:
            origin = await telemetry_home(vehicle_id)
        if origin is None:
            # Demo links carry no geodetic data: fall back to a synthetic
            # shared origin so relative offsets stay map-identical.
            anchor = anchor or _DEMO_MAP_ORIGIN
            origin = anchor
        origins[vehicle_id] = origin
    anchor = origins[ids[0]]
    slots: dict[int, tuple[float, float, float]] = {}
    for index, vehicle_id in enumerate(ids):
        offset = offsets[index]
        slots[vehicle_id] = map_to_vehicle_local_ned(
            (
                leader_target_map_m[0] + offset[0],
                leader_target_map_m[1] + offset[1],
                -altitude_m,
            ),
            anchor,
            origins[vehicle_id],
        )
    lanes = {
        vehicle_id: -(altitude_m + 1.5 * (index + 1))
        for index, vehicle_id in enumerate(ids)
    }

    async def current_position(vehicle_id: int) -> tuple[float, float, float]:
        link = runtime.link_for(vehicle_id)
        if link is None:
            raise FleetMissionError(f"飞机 {vehicle_id} 链路不可用")
        snapshot = link.telemetry_snapshot()
        position = snapshot.local_position_ned_m
        if position is None:
            raise FleetMissionError(f"飞机 {vehicle_id} 当前位置不可用")
        return (float(position[0]), float(position[1]), float(position[2]))

    async def await_handle(handle: Any, vehicle_id: int, step: str) -> Any:
        try:
            result = await asyncio.wait_for(handle.result, timeout=step_timeout_s)
        except asyncio.TimeoutError as exc:
            raise FleetMissionError(f"飞机 {vehicle_id} 步骤 {step} 超时") from exc
        require_completed_command(result, vehicle_id, step)
        results.append(
            {
                "vehicle_id": vehicle_id,
                "step": step,
                "status": result.status.value,
                "detail": str(result.detail),
                "at_monotonic_s": round(clock(), 2),
            }
        )
        return result

    def step(vehicle_id: int, name: str, detail: str = "") -> None:
        if on_step is not None:
            on_step(vehicle_id, name, detail)

    # 1) climb into the lane altitudes (horizontal hold)
    positions: dict[int, tuple[float, float, float]] = {}
    for vehicle_id in ids:
        positions[vehicle_id] = await current_position(vehicle_id)
        step(vehicle_id, "lane", f"进入高度层 {abs(lanes[vehicle_id]):.1f} m")
    await asyncio.gather(
        *(
            await_handle(
                services[vehicle_id].goto_local_ned(
                    positions[vehicle_id][0],
                    positions[vehicle_id][1],
                    lanes[vehicle_id],
                    expires_monotonic_s=clock() + step_timeout_s,
                ),
                vehicle_id,
                "lane",
            )
            for vehicle_id in ids
        )
    )

    # 2) morph horizontally inside the lanes
    for vehicle_id in ids:
        target = slots[vehicle_id]
        step(vehicle_id, "goto", f"飞往槽位 ({target[0]:.1f}, {target[1]:.1f})")
    await asyncio.gather(
        *(
            await_handle(
                services[vehicle_id].goto_local_ned(
                    slots[vehicle_id][0],
                    slots[vehicle_id][1],
                    lanes[vehicle_id],
                    expires_monotonic_s=clock() + step_timeout_s,
                ),
                vehicle_id,
                "goto",
            )
            for vehicle_id in ids
        )
    )

    # 3) level off into the formation altitude
    for vehicle_id in ids:
        step(vehicle_id, "level", "统一高度")
    await asyncio.gather(
        *(
            await_handle(
                services[vehicle_id].goto_local_ned(
                    slots[vehicle_id][0],
                    slots[vehicle_id][1],
                    slots[vehicle_id][2],
                    expires_monotonic_s=clock() + step_timeout_s,
                ),
                vehicle_id,
                "level",
            )
            for vehicle_id in ids
        )
    )
    return results


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
                    "formations": (
                        [formation.value for formation in self._config.formations]
                        if self._config.formations is not None
                        else None
                    ),
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
        if config.formations is not None and not 1 <= len(config.formations) <= 8:
            raise FleetMissionError("队形序列长度必须在 1 到 8 之间")

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
            sequence = config.formations or (config.formation,)
            for formation_index, formation in enumerate(sequence):
                self._stage = (
                    f"队形 {formation_index + 1}/{len(sequence)}"
                    f"：{formation.value}"
                )
                await self._fly_formation(
                    services,
                    config,
                    formation,
                )
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
            if self._cancel_event.is_set():
                self._phase = "cancelled"
                self._stage = "已取消"
            else:
                self._phase = "failed"
                self._stage = "任务失败"
            self._error = str(exc)
            await self._safety_land(services if "services" in locals() else {})
        except Exception as exc:  # pragma: no cover - defensive
            self._phase = "failed"
            self._stage = "任务失败"
            self._error = f"{type(exc).__name__}: {exc}"
            await self._safety_land(services if "services" in locals() else {})

    async def _fly_formation(
        self,
        services: dict[int, FlightCommandService],
        config: FleetMissionConfig,
        formation: FormationType,
    ) -> None:
        """Fly one formation (shared lane-transition logic)."""
        await fly_formation(
            services,
            self._runtime,
            config.vehicle_ids,
            formation,
            config.leader_target_map_m,
            config.spacing_m,
            config.altitude_m,
            clock=self._clock,
            step_timeout_s=_STEP_TIMEOUT_S,
            on_step=self._set_step,
        )
        for vehicle_id in config.vehicle_ids:
            self._finish_step(vehicle_id, "已到位")

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

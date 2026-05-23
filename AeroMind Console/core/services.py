from __future__ import annotations

import math
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Callable

from core.di import Inject
from core.path_planner import Waypoint, lawnmower_path
from core.task_schema import MissionArea, MissionPlan


@dataclass(frozen=True)
class EventItem:
    time: str
    level: str
    message: str


class EventManager:
    def __init__(self, log_dir: Path | None = None) -> None:
        self._events: list[EventItem] = []
        self._log_file: Path | None = log_dir / "aeromind.log" if log_dir else None
        if self._log_file:
            self._log_file.parent.mkdir(parents=True, exist_ok=True)

    def log(self, message: str, level: str = "INFO") -> None:
        now = datetime.now()
        item = EventItem(
            time=now.strftime("%H:%M:%S"),
            level=level,
            message=message,
        )
        self._events.insert(0, item)
        self._events = self._events[:80]
        self._write_log_file(now, level, message)

    def _write_log_file(self, now: datetime, level: str, message: str) -> None:
        if self._log_file is None:
            return
        try:
            with self._log_file.open("a", encoding="utf-8") as handle:
                handle.write(f"{now.isoformat(timespec='seconds')} {level.upper():<7} {message}\n")
        except Exception as log_err:
            print(f"[aeromind] log file write failed: {log_err}", file=sys.stderr)

    def get_events(self) -> list[dict[str, str]]:
        return [{"time": e.time, "level": e.level, "message": e.message} for e in self._events]

    def clear(self) -> None:
        self._events.clear()
        if self._log_file:
            try:
                self._log_file.write_text("", encoding="utf-8")
            except Exception as clear_err:
                print(f"[aeromind] log file clear failed: {clear_err}", file=sys.stderr)
        self.log("Logs cleared.", "INFO")

    def get_log_file_lines(self) -> list[str]:
        if not self._log_file or not self._log_file.exists():
            return []
        try:
            return self._log_file.read_text(encoding="utf-8", errors="replace").splitlines()[-300:]
        except Exception as exc:
            return [f"Failed to read log file: {exc}"]


class MissionState:
    def __init__(self) -> None:
        self._task_status = "idle"
        self._current_task = "等待任务"
        self._plan: list[dict[str, str]] = []
        self._start_time = time.time()

    @property
    def status(self) -> str:
        return self._task_status

    @property
    def current_task(self) -> str:
        return self._current_task

    @property
    def plan(self) -> list[dict[str, str]]:
        return self._plan

    @property
    def uptime_s(self) -> float:
        return round(time.time() - self._start_time, 1)

    def set_mission(self, mission: MissionPlan) -> None:
        self._current_task = mission.task
        self._plan = [step.to_api() for step in mission.plan]
        self._task_status = "planning"

    def set_status(self, status: str, message: str | None = None) -> None:
        self._task_status = status
        if message:
            self._current_task = message

    def reset(self) -> None:
        self._task_status = "idle"
        self._current_task = "等待任务"
        self._plan = []


class RouteManager:
    def __init__(self) -> None:
        self._trail: list[dict[str, float]] = []
        self._route: list[dict[str, float]] = []
        self._search_area: dict[str, float] | None = None
        self._targets: list[dict[str, float | str]] = []

    @property
    def trail(self) -> list[dict[str, float]]:
        return self._trail[-240:]

    @property
    def route(self) -> list[dict[str, float]]:
        return self._route

    @property
    def search_area(self) -> dict[str, float] | None:
        return self._search_area

    @property
    def targets(self) -> list[dict[str, float | str]]:
        return self._targets

    def update_trail(self, x: float, y: float, z: float) -> None:
        point = {"x": x, "y": y, "z": z}
        if not self._trail:
            self._trail.append(point)
            return
        last = self._trail[-1]
        distance_sq = (last["x"] - x) ** 2 + (last["y"] - y) ** 2 + (last["z"] - z) ** 2
        if distance_sq >= 0.25:
            self._trail.append(point)
            self._trail = self._trail[-300:]

    def set_route(self, route: list[dict[str, float]]) -> None:
        self._route = route

    def set_search_area(self, area: dict[str, float] | None) -> None:
        self._search_area = area

    def clear_targets(self) -> None:
        self._targets = []

    def generate_search_route(self, area: MissionArea, altitude_m: float, spacing_m: float = 10.0) -> None:
        self._route = [
            point.to_api()
            for point in lawnmower_path(area, altitude_m, spacing_m=spacing_m)
        ]
        self._search_area = area.to_api()

    def clear(self) -> None:
        self._trail.clear()
        self._route.clear()
        self._search_area = None
        self._targets.clear()


class MissionContext:
    def __init__(
        self,
        event_manager: EventManager,
        mission_state: MissionState,
        route_manager: RouteManager,
    ) -> None:
        self.events = event_manager
        self.mission = mission_state
        self.routes = route_manager
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, Optional

import numpy as np


@dataclass(frozen=True)
class FormationSlot:
    vehicle_name: str
    offset_x: float
    offset_y: float
    offset_z: float
    role: str = "follower"

    def to_api(self) -> dict[str, Any]:
        return {
            "vehicle_name": self.vehicle_name,
            "offset_x": self.offset_x,
            "offset_y": self.offset_y,
            "offset_z": self.offset_z,
            "role": self.role,
        }


@dataclass(frozen=True)
class FormationPattern:
    name: str
    description: str
    min_vehicles: int
    max_vehicles: int
    slots: list[FormationSlot]

    def to_api(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "description": self.description,
            "min_vehicles": self.min_vehicles,
            "max_vehicles": self.max_vehicles,
            "slots": [slot.to_api() for slot in self.slots],
        }


class FormationManager:
    def __init__(self) -> None:
        self._patterns: dict[str, Callable[[int, float], FormationPattern]] = {
            "v": self._create_v_formation,
            "line": self._create_line_formation,
            "delta": self._create_delta_formation,
            "diamond": self._create_diamond_formation,
            "triangle": self._create_triangle_formation,
            "column": self._create_column_formation,
        }

    def get_pattern_names(self) -> list[str]:
        return list(self._patterns.keys())

    def get_pattern(self, name: str, count: int, spacing_m: float) -> Optional[FormationPattern]:
        creator = self._patterns.get(name.lower())
        if not creator:
            return None
        return creator(count, spacing_m)

    def create_pattern(self, name: str, count: int, spacing_m: float) -> FormationPattern:
        pattern = self.get_pattern(name, count, spacing_m)
        if pattern is None:
            raise ValueError(f"Unknown formation pattern: {name}")
        return pattern

    def _create_v_formation(self, count: int, spacing_m: float) -> FormationPattern:
        slots = []
        half_count = count // 2
        vehicles = [f"Drone{i+1}" for i in range(count)]
        
        slots.append(FormationSlot(vehicle_name=vehicles[0], offset_x=0, offset_y=0, offset_z=0, role="leader"))
        
        for i in range(1, count):
            side = -1 if i <= half_count else 1
            row = (i + 1) // 2
            offset_x = -row * spacing_m * 0.8
            offset_y = side * (i - (i > half_count)) * spacing_m * 0.6
            offset_z = 0
            slots.append(FormationSlot(vehicle_name=vehicles[i], offset_x=offset_x, offset_y=offset_y, offset_z=offset_z))
        
        return FormationPattern(
            name="v",
            description="V形编队 - 经典的箭头形状",
            min_vehicles=2,
            max_vehicles=6,
            slots=slots,
        )

    def _create_line_formation(self, count: int, spacing_m: float) -> FormationPattern:
        slots = []
        vehicles = [f"Drone{i+1}" for i in range(count)]
        start_pos = -(count - 1) * spacing_m / 2
        
        for i, vehicle in enumerate(vehicles):
            role = "leader" if i == count // 2 else "follower"
            offset_x = 0
            offset_y = start_pos + i * spacing_m
            offset_z = 0
            slots.append(FormationSlot(vehicle_name=vehicle, offset_x=offset_x, offset_y=offset_y, offset_z=offset_z, role=role))
        
        return FormationPattern(
            name="line",
            description="一字编队 - 水平排列",
            min_vehicles=2,
            max_vehicles=6,
            slots=slots,
        )

    def _create_delta_formation(self, count: int, spacing_m: float) -> FormationPattern:
        slots = []
        vehicles = [f"Drone{i+1}" for i in range(count)]
        
        slots.append(FormationSlot(vehicle_name=vehicles[0], offset_x=0, offset_y=0, offset_z=0, role="leader"))
        
        if count >= 2:
            slots.append(FormationSlot(vehicle_name=vehicles[1], offset_x=-spacing_m, offset_y=-spacing_m, offset_z=0))
        if count >= 3:
            slots.append(FormationSlot(vehicle_name=vehicles[2], offset_x=-spacing_m, offset_y=spacing_m, offset_z=0))
        if count >= 4:
            slots.append(FormationSlot(vehicle_name=vehicles[3], offset_x=-2 * spacing_m, offset_y=-spacing_m * 1.5, offset_z=0))
        if count >= 5:
            slots.append(FormationSlot(vehicle_name=vehicles[4], offset_x=-2 * spacing_m, offset_y=spacing_m * 1.5, offset_z=0))
        if count >= 6:
            slots.append(FormationSlot(vehicle_name=vehicles[5], offset_x=-3 * spacing_m, offset_y=0, offset_z=0))
        
        return FormationPattern(
            name="delta",
            description="三角翼编队 - 战斗机风格",
            min_vehicles=2,
            max_vehicles=6,
            slots=slots,
        )

    def _create_diamond_formation(self, count: int, spacing_m: float) -> FormationPattern:
        slots = []
        vehicles = [f"Drone{i+1}" for i in range(count)]
        
        slots.append(FormationSlot(vehicle_name=vehicles[0], offset_x=0, offset_y=0, offset_z=0, role="leader"))
        
        if count >= 2:
            slots.append(FormationSlot(vehicle_name=vehicles[1], offset_x=-spacing_m, offset_y=0, offset_z=0))
        if count >= 3:
            slots.append(FormationSlot(vehicle_name=vehicles[2], offset_x=0, offset_y=-spacing_m, offset_z=0))
        if count >= 4:
            slots.append(FormationSlot(vehicle_name=vehicles[3], offset_x=0, offset_y=spacing_m, offset_z=0))
        if count >= 5:
            slots.append(FormationSlot(vehicle_name=vehicles[4], offset_x=-spacing_m * 1.5, offset_y=-spacing_m * 0.7, offset_z=0))
        if count >= 6:
            slots.append(FormationSlot(vehicle_name=vehicles[5], offset_x=-spacing_m * 1.5, offset_y=spacing_m * 0.7, offset_z=0))
        
        return FormationPattern(
            name="diamond",
            description="菱形编队 - 四面对称",
            min_vehicles=2,
            max_vehicles=6,
            slots=slots,
        )

    def _create_triangle_formation(self, count: int, spacing_m: float) -> FormationPattern:
        slots = []
        vehicles = [f"Drone{i+1}" for i in range(count)]
        
        slots.append(FormationSlot(vehicle_name=vehicles[0], offset_x=0, offset_y=0, offset_z=0, role="leader"))
        
        if count >= 2:
            slots.append(FormationSlot(vehicle_name=vehicles[1], offset_x=-spacing_m, offset_y=-spacing_m, offset_z=0))
        if count >= 3:
            slots.append(FormationSlot(vehicle_name=vehicles[2], offset_x=-spacing_m, offset_y=spacing_m, offset_z=0))
        if count >= 4:
            slots.append(FormationSlot(vehicle_name=vehicles[3], offset_x=-2 * spacing_m, offset_y=-spacing_m * 1.5, offset_z=0))
        if count >= 5:
            slots.append(FormationSlot(vehicle_name=vehicles[4], offset_x=-2 * spacing_m, offset_y=spacing_m * 1.5, offset_z=0))
        if count >= 6:
            slots.append(FormationSlot(vehicle_name=vehicles[5], offset_x=-3 * spacing_m, offset_y=0, offset_z=0))
        
        return FormationPattern(
            name="triangle",
            description="三角形编队 - 前尖后宽",
            min_vehicles=2,
            max_vehicles=6,
            slots=slots,
        )

    def _create_column_formation(self, count: int, spacing_m: float) -> FormationPattern:
        slots = []
        vehicles = [f"Drone{i+1}" for i in range(count)]
        
        for i, vehicle in enumerate(vehicles):
            role = "leader" if i == 0 else "follower"
            offset_x = -i * spacing_m
            offset_y = 0
            offset_z = 0
            slots.append(FormationSlot(vehicle_name=vehicle, offset_x=offset_x, offset_y=offset_y, offset_z=offset_z, role=role))
        
        return FormationPattern(
            name="column",
            description="纵队编队 - 前后排列",
            min_vehicles=2,
            max_vehicles=6,
            slots=slots,
        )

    def get_all_patterns(self, count: int, spacing_m: float) -> list[FormationPattern]:
        return [creator(count, spacing_m) for creator in self._patterns.values()]


class SwarmCoordinator:
    def __init__(self, airsim_adapter) -> None:
        self._airsim = airsim_adapter
        self._formation_manager = FormationManager()
        self._active_formation: Optional[FormationPattern] = None
        self._current_leader = "Drone1"

    def set_formation(self, pattern_name: str, count: int, spacing_m: float) -> bool:
        pattern = self._formation_manager.get_pattern(pattern_name, count, spacing_m)
        if pattern is None:
            return False
        self._active_formation = pattern
        return True

    def get_formation(self) -> Optional[FormationPattern]:
        return self._active_formation

    def get_supported_patterns(self) -> list[str]:
        return self._formation_manager.get_pattern_names()

    def calculate_goal_positions(self, leader_x: float, leader_y: float, leader_z: float) -> dict[str, tuple[float, float, float]]:
        if self._active_formation is None:
            return {}
        
        positions = {}
        for slot in self._active_formation.slots:
            x = leader_x + slot.offset_x
            y = leader_y + slot.offset_y
            z = leader_z + slot.offset_z
            positions[slot.vehicle_name] = (x, y, z)
        return positions

    def move_formation(self, x: float, y: float, z: float, speed_mps: float) -> dict[str, Any]:
        if self._active_formation is None:
            return {"error": "No active formation"}
        
        goal_positions = self.calculate_goal_positions(x, y, z)
        
        results = {}
        for vehicle_name, (gx, gy, gz) in goal_positions.items():
            try:
                self._airsim.switch_vehicle(vehicle_name)
                result = self._airsim.goto_local(gx, gy, gz, speed_mps)
                results[vehicle_name] = result
            except Exception as exc:
                results[vehicle_name] = {"error": str(exc)}
        
        return {"status": "moving", "results": results}

    def takeoff_formation(self, altitude_m: float) -> dict[str, Any]:
        if self._active_formation is None:
            return {"error": "No active formation"}
        
        results = {}
        for slot in self._active_formation.slots:
            try:
                self._airsim.switch_vehicle(slot.vehicle_name)
                result = self._airsim.takeoff(altitude_m)
                results[slot.vehicle_name] = result
            except Exception as exc:
                results[slot.vehicle_name] = {"error": str(exc)}
        
        return {"status": "taking_off", "results": results}

    def land_formation(self) -> dict[str, Any]:
        if self._active_formation is None:
            return {"error": "No active formation"}
        
        results = {}
        for slot in self._active_formation.slots:
            try:
                self._airsim.switch_vehicle(slot.vehicle_name)
                result = self._airsim.land()
                results[slot.vehicle_name] = result
            except Exception as exc:
                results[slot.vehicle_name] = {"error": str(exc)}
        
        return {"status": "landing", "results": results}

    def get_formation_status(self) -> dict[str, Any]:
        if self._active_formation is None:
            return {"active": False, "pattern": None}
        
        status = {
            "active": True,
            "pattern": self._active_formation.name,
            "vehicles": [],
        }
        
        for slot in self._active_formation.slots:
            try:
                self._airsim.switch_vehicle(slot.vehicle_name)
                telemetry = self._airsim.telemetry()
                status["vehicles"].append({
                    "name": slot.vehicle_name,
                    "role": slot.role,
                    "x": float(telemetry.x),
                    "y": float(telemetry.y),
                    "z": float(telemetry.z),
                    "altitude": float(telemetry.altitude_m),
                    "speed": float(telemetry.speed_mps),
                })
            except Exception:
                status["vehicles"].append({
                    "name": slot.vehicle_name,
                    "role": slot.role,
                    "error": "unavailable",
                })
        
        return status
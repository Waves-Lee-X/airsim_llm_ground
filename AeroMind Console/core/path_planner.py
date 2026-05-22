from __future__ import annotations

from dataclasses import dataclass

from core.task_schema import MissionArea


@dataclass(frozen=True)
class Waypoint:
    x: float
    y: float
    z: float

    def to_api(self) -> dict[str, float]:
        return {"x": self.x, "y": self.y, "z": self.z}


def lawnmower_path(area: MissionArea, altitude_m: float, spacing_m: float = 10.0) -> list[Waypoint]:
    """Generate a simple rectangular coverage path in AirSim local coordinates."""
    z = -abs(float(altitude_m))
    y = area.y_min
    direction = 1
    points: list[Waypoint] = []
    while y <= area.y_max + 1e-6:
        if direction > 0:
            points.append(Waypoint(area.x_min, y, z))
            points.append(Waypoint(area.x_max, y, z))
        else:
            points.append(Waypoint(area.x_max, y, z))
            points.append(Waypoint(area.x_min, y, z))
        y += max(1.0, spacing_m)
        direction *= -1
    return points


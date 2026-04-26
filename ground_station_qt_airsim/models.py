from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field


@dataclass
class TelemetryData:
    sysid: int
    vehicle_name: str
    display_name: str
    lat: float = 0.0
    lon: float = 0.0
    alt: float = 0.0
    yaw_deg: float = 0.0
    pitch_deg: float = 0.0
    roll_deg: float = 0.0
    speed: float = 0.0
    batt: float = 0.0
    armed: bool = False
    mode: str = "SIM"
    stars: int = 0
    gps_valid: bool = False
    local_valid: bool = False
    local_x: float = 0.0
    local_y: float = 0.0
    local_z: float = 0.0
    leader_id: int = 0
    team_no: int = 0
    follow_enabled: bool = False
    last_update: float = 0.0
    trail: deque[tuple[float, float, float, float, float]] = field(
        default_factory=lambda: deque(maxlen=200)
    )

    def is_online(self, now: float, timeout: float = 2.5) -> bool:
        return self.last_update > 0 and (now - self.last_update) <= timeout


@dataclass
class VehicleBinding:
    sysid: int
    vehicle_name: str
    display_name: str


@dataclass
class TeamAssignment:
    leader_id: int = 0
    team_no: int = 0
    follow_enabled: bool = False


@dataclass
class CommandRecord:
    index: int
    target_id: int
    command: str
    status: str
    issued_at: str
    detail: str = ""


@dataclass
class Waypoint:
    lat: float
    lon: float
    alt_m: float


def metres_between_local(a: TelemetryData, x: float, y: float, z: float) -> float:
    dx = a.local_x - x
    dy = a.local_y - y
    dz = a.local_z - z
    return math.sqrt(dx * dx + dy * dy + dz * dz)

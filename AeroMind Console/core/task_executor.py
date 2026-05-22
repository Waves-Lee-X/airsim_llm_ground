from __future__ import annotations

from core.airsim_adapter import AirSimAdapter
from core.task_schema import MissionPlan


class TaskExecutor:
    """Basic AirSim command executor."""

    def __init__(self, adapter: AirSimAdapter) -> None:
        self.adapter = adapter
        self.current_mission: MissionPlan | None = None

    def start(self, mission: MissionPlan) -> None:
        self.current_mission = mission

    def takeoff(self, altitude_m: float = 8.0) -> None:
        self.adapter.takeoff(altitude_m=altitude_m)

    def pause(self) -> None:
        self.adapter.hover()

    def hover(self) -> None:
        self.adapter.hover()

    def stop(self) -> None:
        self.adapter.stop()

    def hard_stop(self) -> dict[str, float]:
        return self.adapter.hard_stop()

    def land(self) -> None:
        self.adapter.land()

    def return_to_launch(self, altitude_m: float = 8.0) -> None:
        self.adapter.return_to_launch(altitude_m=altitude_m)

    def goto_local(
        self,
        x: float,
        y: float,
        z: float,
        speed_mps: float = 3.0,
        face_target: bool = True,
    ) -> None:
        self.adapter.goto_local(x=x, y=y, z=z, speed_mps=speed_mps, face_target=face_target)

    def move_velocity(
        self,
        vx: float,
        vy: float,
        vz: float,
        duration_s: float = 0.2,
        yaw_rate_deg_s: float = 0.0,
    ) -> None:
        self.adapter.move_velocity(vx=vx, vy=vy, vz=vz, duration_s=duration_s, yaw_rate_deg_s=yaw_rate_deg_s)

    def face_local(self, x: float, y: float) -> None:
        self.adapter.face_local(x=x, y=y)

    def recover_from_collision(
        self,
        climb_m: float = 8.0,
        backoff_m: float = 10.0,
        force_relocate: bool = True,
    ) -> dict[str, object]:
        return self.adapter.recover_from_collision(
            climb_m=climb_m,
            backoff_m=backoff_m,
            force_relocate=force_relocate,
        )

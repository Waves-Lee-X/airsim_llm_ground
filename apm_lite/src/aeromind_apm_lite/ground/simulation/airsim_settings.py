"""AirSim 1.8.1 settings generation for ArduPilot lock-step vehicles."""

from __future__ import annotations

import json
import os
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .config import (
    AIRSIM_SETTINGS_VERSION,
    AIRSIM_VERSION,
    SimulationConfigError,
    SimulationLaunchConfig,
)


def _front_rgb_depth_camera() -> dict[str, Any]:
    return {
        "X": 0.5,
        "Y": 0.0,
        "Z": -0.15,
        "Pitch": 0.0,
        "Roll": 0.0,
        "Yaw": 0.0,
        "CaptureSettings": [
            {
                "ImageType": 0,
                "Width": 640,
                "Height": 360,
                "FOV_Degrees": 95,
                "MotionBlurAmount": 0,
            },
            {
                "ImageType": 3,
                "Width": 640,
                "Height": 360,
                "FOV_Degrees": 95,
                "MotionBlurAmount": 0,
            },
        ],
    }


@dataclass(frozen=True)
class AirSimSettings:
    """A reproducible settings document and its required AirSim runtime version."""

    launch: SimulationLaunchConfig
    required_runtime_version: str = AIRSIM_VERSION

    def __post_init__(self) -> None:
        if self.required_runtime_version != AIRSIM_VERSION:
            raise SimulationConfigError(
                f"AirSim runtime must remain pinned to {AIRSIM_VERSION}"
            )

    def as_dict(self) -> dict[str, Any]:
        origin = self.launch.instances[0].home
        vehicles: dict[str, Any] = {}
        for instance in self.launch.instances:
            pose = instance.initial_pose
            vehicles[instance.vehicle_name] = {
                "VehicleType": "ArduCopter",
                "AutoCreate": True,
                "UseSerial": False,
                "LockStep": True,
                "LocalHostIp": "0.0.0.0",
                "UdpIp": self.launch.wsl_ip,
                "UdpPort": instance.sensor_port,
                "ControlPort": instance.control_port,
                "X": pose.x_m,
                "Y": pose.y_m,
                "Z": pose.z_m,
                "Yaw": pose.yaw_deg,
                "Cameras": {"front_center": _front_rgb_depth_camera()},
            }
        return {
            "SeeDocsAt": "https://microsoft.github.io/AirSim/settings/",
            "SettingsVersion": AIRSIM_SETTINGS_VERSION,
            "SimMode": "Multirotor",
            "ClockType": "SteppableClock",
            "RpcEnabled": True,
            "RpcPort": 41451,
            "LocalHostIp": "0.0.0.0",
            "ViewMode": "FlyWithMe",
            "OriginGeopoint": {
                "Latitude": origin.latitude,
                "Longitude": origin.longitude,
                "Altitude": origin.altitude_m,
            },
            "Vehicles": vehicles,
        }

    def to_json(self) -> str:
        return json.dumps(
            self.as_dict(), ensure_ascii=True, allow_nan=False, indent=2
        ) + "\n"

    def write(self, path: Path | None = None) -> Path:
        target = path or self.launch.paths.airsim_settings_path
        target = Path(target)
        if not target.is_absolute() or target.name != "settings.json":
            raise SimulationConfigError(
                "AirSim settings output must be an absolute settings.json path"
            )
        target.parent.mkdir(parents=True, exist_ok=True)
        descriptor, temporary_name = tempfile.mkstemp(
            prefix=".settings-", suffix=".json", dir=target.parent
        )
        temporary_path = Path(temporary_name)
        try:
            with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as stream:
                stream.write(self.to_json())
                stream.flush()
                os.fsync(stream.fileno())
            temporary_path.replace(target)
        except BaseException:
            temporary_path.unlink(missing_ok=True)
            raise
        return target

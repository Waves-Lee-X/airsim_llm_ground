"""Read the active AirSim settings.json to learn the simulated spawn layout.

The SITL vehicles' EKF/local-NED frames are anchored at their spawn GPS
positions, while the MAVLink HOME_POSITION telemetry can be re-anchored by
ArduPilot (e.g. on arming).  Formation targets therefore convert map-frame
points using these stable spawn positions instead of live telemetry homes.
"""
from __future__ import annotations

import json
from pathlib import Path

from aeromind_apm_lite.common.coordinates import (
    GeodeticPosition,
    Vec3,
    enu_to_geodetic,
)

_AIRSIM_VEHICLE_NAMES = ("Drone1", "Drone2", "Drone3", "Drone4")


def default_airsim_settings_path() -> Path:
    """Return the settings file AirSim itself loads on this machine."""
    return Path.home() / "Documents" / "AirSim" / "settings.json"


class AirSimSpawnMap:
    """Per-vehicle spawn GPS derived from the AirSim world layout."""

    def __init__(
        self,
        path: str | Path | None = None,
    ) -> None:
        self._homes: dict[int, GeodeticPosition] = {}
        try:
            payload = json.loads(
                (Path(path) if path else default_airsim_settings_path()).read_text(
                    encoding="utf-8"
                )
            )
        except (OSError, ValueError, TypeError):
            return
        origin = payload.get("OriginGeopoint") or {}
        if (
            not isinstance(origin, dict)
            or "Latitude" not in origin
            or "Longitude" not in origin
        ):
            return
        try:
            base = GeodeticPosition(
                float(origin["Latitude"]),
                float(origin["Longitude"]),
                float(origin.get("Altitude", 0.0)),
            )
        except (TypeError, ValueError):
            return
        vehicles = payload.get("Vehicles") or {}
        if not isinstance(vehicles, dict):
            return
        for index, name in enumerate(_AIRSIM_VEHICLE_NAMES, start=1):
            vehicle = vehicles.get(name)
            if not isinstance(vehicle, dict):
                continue
            try:
                # AirSim world offset is NED (X north, Y east, Z down).
                offset_enu = Vec3(
                    float(vehicle.get("Y", 0.0)),
                    float(vehicle.get("X", 0.0)),
                    -float(vehicle.get("Z", 0.0)),
                )
            except (TypeError, ValueError):
                continue
            self._homes[index] = enu_to_geodetic(offset_enu, base)

    def home_for(self, vehicle_id: int) -> GeodeticPosition | None:
        return self._homes.get(int(vehicle_id))

    def known_ids(self) -> tuple[int, ...]:
        return tuple(sorted(self._homes))
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

try:
    import yaml
except Exception:  # pragma: no cover
    yaml = None


@dataclass
class UiConfig:
    title: str = "AirSim 基础地面站"
    theme_mode: str = "light"
    geometry: str = "1480x920"
    min_width: int = 1200
    min_height: int = 760
    dashboard_interval_ms: int = 250
    default_selected_uav: int = 1
    default_altitude_m: float = -8.0
    default_map_center_lat: float = 34.3850148
    default_map_center_lon: float = 108.9848592


@dataclass
class AirSimConfig:
    host: str = "127.0.0.1"
    port: int = 41451
    timeout_s: float = 10.0
    poll_interval_s: float = 0.2
    default_speed_mps: float = 4.0
    takeoff_height_m: float = 8.0
    auto_enable_api_control: bool = True
    use_gps_on_map: bool = True
    vehicle_names: list[str] | None = None


@dataclass
class FormationConfig:
    enabled: bool = True
    refresh_interval_s: float = 0.6
    default_spacing_m: float = 6.0
    default_altitude_offset_m: float = 0.0


@dataclass
class GroundStationAirSimConfig:
    ui: UiConfig
    airsim: AirSimConfig
    formation: FormationConfig


def default_config() -> GroundStationAirSimConfig:
    return GroundStationAirSimConfig(
        ui=UiConfig(),
        airsim=AirSimConfig(),
        formation=FormationConfig(),
    )


def _as_dict(value: Any) -> dict[str, Any]:
    return value if isinstance(value, dict) else {}


def load_config(file_path: str | Path = "config_airsim.yaml") -> GroundStationAirSimConfig:
    base = default_config()
    path = Path(file_path)
    if yaml is None or not path.exists():
        return base

    with path.open("r", encoding="utf-8") as handle:
        data = _as_dict(yaml.safe_load(handle) or {})

    ui = _as_dict(data.get("ui"))
    airsim_cfg = _as_dict(data.get("airsim"))
    formation = _as_dict(data.get("formation"))

    names_raw = airsim_cfg.get("vehicle_names", base.airsim.vehicle_names)
    vehicle_names = None
    if isinstance(names_raw, list):
        vehicle_names = [str(item).strip() for item in names_raw if str(item).strip()]

    return GroundStationAirSimConfig(
        ui=UiConfig(
            title=str(ui.get("title", base.ui.title)),
            theme_mode=str(ui.get("theme_mode", base.ui.theme_mode)).strip().lower() or base.ui.theme_mode,
            geometry=str(ui.get("geometry", base.ui.geometry)),
            min_width=int(ui.get("min_width", base.ui.min_width)),
            min_height=int(ui.get("min_height", base.ui.min_height)),
            dashboard_interval_ms=int(ui.get("dashboard_interval_ms", base.ui.dashboard_interval_ms)),
            default_selected_uav=int(ui.get("default_selected_uav", base.ui.default_selected_uav)),
            default_altitude_m=float(ui.get("default_altitude_m", base.ui.default_altitude_m)),
            default_map_center_lat=float(ui.get("default_map_center_lat", base.ui.default_map_center_lat)),
            default_map_center_lon=float(ui.get("default_map_center_lon", base.ui.default_map_center_lon)),
        ),
        airsim=AirSimConfig(
            host=str(airsim_cfg.get("host", base.airsim.host)),
            port=int(airsim_cfg.get("port", base.airsim.port)),
            timeout_s=float(airsim_cfg.get("timeout_s", base.airsim.timeout_s)),
            poll_interval_s=float(airsim_cfg.get("poll_interval_s", base.airsim.poll_interval_s)),
            default_speed_mps=float(airsim_cfg.get("default_speed_mps", base.airsim.default_speed_mps)),
            takeoff_height_m=float(airsim_cfg.get("takeoff_height_m", base.airsim.takeoff_height_m)),
            auto_enable_api_control=bool(
                airsim_cfg.get("auto_enable_api_control", base.airsim.auto_enable_api_control)
            ),
            use_gps_on_map=bool(airsim_cfg.get("use_gps_on_map", base.airsim.use_gps_on_map)),
            vehicle_names=vehicle_names,
        ),
        formation=FormationConfig(
            enabled=bool(formation.get("enabled", base.formation.enabled)),
            refresh_interval_s=float(formation.get("refresh_interval_s", base.formation.refresh_interval_s)),
            default_spacing_m=float(formation.get("default_spacing_m", base.formation.default_spacing_m)),
            default_altitude_offset_m=float(
                formation.get("default_altitude_offset_m", base.formation.default_altitude_offset_m)
            ),
        ),
    )

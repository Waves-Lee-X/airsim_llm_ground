"""Render scene-independent AirSim settings for a manually owned simulator."""

from __future__ import annotations

import argparse
import ipaddress
import json
import math
import os
import shlex
import tempfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from .config import (
    AIRSIM_SETTINGS_VERSION,
    DEFAULT_ARDUPILOT_ROOT,
    DEFAULT_M1_PARAMETER_FILE,
    ArduPilotBuild,
    SimulationConfigError,
    SitlInstanceConfig,
    airsim_control_port,
    airsim_sensor_port,
    discover_windows_host_ipv4,
    discover_wsl_ipv4,
)
from .lifecycle import sitl_argv

DEFAULT_MANUAL_SETTINGS_TEMPLATE = (
    Path(__file__).resolve().parents[4]
    / "configs"
    / "sim"
    / "settings.manual.template.json"
)
_MANUAL_INSTANCE = 0


def _unicast_ipv4(value: str, *, label: str) -> str:
    try:
        address = ipaddress.ip_address(value)
    except (TypeError, ValueError) as exc:
        raise SimulationConfigError(f"{label} must be an IPv4 address") from exc
    if not isinstance(address, ipaddress.IPv4Address):
        raise SimulationConfigError(f"{label} must be an IPv4 address")
    if address.is_unspecified or address.is_loopback or address.is_multicast:
        raise SimulationConfigError(
            f"{label} must be a non-loopback unicast IPv4 address"
        )
    return str(address)


def _finite(value: float, *, label: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise SimulationConfigError(f"{label} must be numeric") from exc
    if not math.isfinite(number):
        raise SimulationConfigError(f"{label} must be finite")
    return number


@dataclass(frozen=True, slots=True)
class ManualCameraConfig:
    """Configurable forward monocular RGB baseline; no depth sensor is implied."""

    width_px: int = 1280
    height_px: int = 720
    fov_degrees: float = 90.0
    x_m: float = 0.35
    y_m: float = 0.0
    z_m: float = -0.05
    pitch_deg: float = 0.0
    roll_deg: float = 0.0
    yaw_deg: float = 0.0

    def __post_init__(self) -> None:
        for name, value, minimum, maximum in (
            ("camera width", self.width_px, 160, 7680),
            ("camera height", self.height_px, 120, 4320),
        ):
            if isinstance(value, bool) or not isinstance(value, int):
                raise SimulationConfigError(f"{name} must be an integer")
            if not minimum <= value <= maximum:
                raise SimulationConfigError(
                    f"{name} must be in [{minimum}, {maximum}]"
                )

        fov = _finite(self.fov_degrees, label="camera FOV")
        if not 10.0 <= fov <= 170.0:
            raise SimulationConfigError("camera FOV must be in [10, 170] degrees")
        object.__setattr__(self, "fov_degrees", fov)

        for field_name in ("x_m", "y_m", "z_m"):
            value = _finite(getattr(self, field_name), label=f"camera {field_name}")
            if not -10.0 <= value <= 10.0:
                raise SimulationConfigError(
                    f"camera {field_name} must be in [-10, 10] metres"
                )
            object.__setattr__(self, field_name, value)

        for field_name, minimum, maximum in (
            ("pitch_deg", -90.0, 90.0),
            ("roll_deg", -180.0, 180.0),
            ("yaw_deg", -180.0, 180.0),
        ):
            value = _finite(
                getattr(self, field_name), label=f"camera {field_name}"
            )
            if not minimum <= value <= maximum:
                raise SimulationConfigError(
                    f"camera {field_name} must be in [{minimum:g}, {maximum:g}]"
                )
            object.__setattr__(self, field_name, value)

    def as_airsim_dict(self) -> dict[str, Any]:
        return {
            "X": self.x_m,
            "Y": self.y_m,
            "Z": self.z_m,
            "Pitch": self.pitch_deg,
            "Roll": self.roll_deg,
            "Yaw": self.yaw_deg,
            "CaptureSettings": [
                {
                    "ImageType": 0,
                    "Width": self.width_px,
                    "Height": self.height_px,
                    "FOV_Degrees": self.fov_degrees,
                    "MotionBlurAmount": 0,
                }
            ],
        }


@dataclass(frozen=True, slots=True)
class ManualSimulationConfig:
    """Addresses and camera profile for user-owned AirSim and SITL processes."""

    wsl_ip: str | None = None
    windows_host_ip: str | None = None
    camera: ManualCameraConfig = field(default_factory=ManualCameraConfig)

    def __post_init__(self) -> None:
        wsl_ip = self.wsl_ip
        if wsl_ip is None:
            wsl_ip = discover_wsl_ipv4()
        windows_host_ip = self.windows_host_ip
        if windows_host_ip is None:
            windows_host_ip = discover_windows_host_ipv4()
        object.__setattr__(
            self,
            "wsl_ip",
            _unicast_ipv4(wsl_ip, label="WSL IP"),
        )
        object.__setattr__(
            self,
            "windows_host_ip",
            _unicast_ipv4(windows_host_ip, label="Windows host IP"),
        )


def _load_template(path: Path) -> dict[str, Any]:
    template_path = Path(path).expanduser()
    try:
        document = json.loads(template_path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise SimulationConfigError(
            f"could not read manual AirSim template {template_path}: {exc}"
        ) from exc
    except json.JSONDecodeError as exc:
        raise SimulationConfigError(
            f"manual AirSim template is invalid JSON: {template_path}: {exc}"
        ) from exc
    if not isinstance(document, dict):
        raise SimulationConfigError("manual AirSim template root must be an object")
    try:
        vehicle = document["Vehicles"]["Drone1"]
    except (KeyError, TypeError) as exc:
        raise SimulationConfigError(
            "manual AirSim template must define Vehicles.Drone1"
        ) from exc
    if not isinstance(vehicle, dict):
        raise SimulationConfigError(
            "manual AirSim template Vehicles.Drone1 must be an object"
        )
    if document.get("SettingsVersion") != AIRSIM_SETTINGS_VERSION:
        raise SimulationConfigError(
            f"manual AirSim template must use settings version "
            f"{AIRSIM_SETTINGS_VERSION}"
        )
    return document


def render_manual_settings(
    config: ManualSimulationConfig,
    *,
    template_path: Path = DEFAULT_MANUAL_SETTINGS_TEMPLATE,
) -> dict[str, Any]:
    """Render one ArduCopter vehicle for any AirSim-enabled UE scene."""

    document = _load_template(template_path)
    vehicle = document["Vehicles"]["Drone1"]
    document.update(
        {
            "SimMode": "Multirotor",
            "ClockType": "SteppableClock",
            "LocalHostIp": "0.0.0.0",
        }
    )
    vehicle.update(
        {
            "VehicleType": "ArduCopter",
            "AutoCreate": True,
            "UseSerial": False,
            "LockStep": True,
            "LocalHostIp": "0.0.0.0",
            "UdpIp": config.wsl_ip,
            "UdpPort": airsim_sensor_port(_MANUAL_INSTANCE),
            "ControlPort": airsim_control_port(_MANUAL_INSTANCE),
            "Cameras": {"front_center": config.camera.as_airsim_dict()},
        }
    )
    return document


def write_manual_settings(
    config: ManualSimulationConfig,
    output_path: Path,
    *,
    template_path: Path = DEFAULT_MANUAL_SETTINGS_TEMPLATE,
) -> Path:
    """Atomically write settings.json without starting AirSim or SITL."""

    target = Path(output_path).expanduser()
    if not target.is_absolute() or target.name != "settings.json":
        raise SimulationConfigError(
            "manual AirSim output must be an absolute settings.json path"
        )
    document = render_manual_settings(config, template_path=template_path)
    encoded = json.dumps(
        document,
        ensure_ascii=True,
        allow_nan=False,
        indent=2,
    ) + "\n"
    target.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=".settings-manual-",
        suffix=".json",
        dir=target.parent,
    )
    temporary_path = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as stream:
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
        temporary_path.replace(target)
    except BaseException:
        temporary_path.unlink(missing_ok=True)
        raise
    return target


def build_manual_sitl_argv(
    config: ManualSimulationConfig,
    *,
    ardupilot_root: Path = DEFAULT_ARDUPILOT_ROOT,
    parameter_file: Path = DEFAULT_M1_PARAMETER_FILE,
) -> tuple[str, ...]:
    """Return, but never execute, the matching single-vehicle SITL command."""

    root = Path(ardupilot_root).expanduser()
    build = ArduPilotBuild(
        source_root=root,
        executable=root / "build" / "sitl" / "bin" / "arducopter",
    )
    instance = SitlInstanceConfig(
        instance=_MANUAL_INSTANCE,
        build=build,
        simulator_address=config.windows_host_ip,
        parameter_file=Path(parameter_file).expanduser(),
    )
    return sitl_argv(instance)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Render AirSim settings for manually started UE/AirSim and ArduCopter "
            "SITL processes. This command starts no process."
        )
    )
    parser.add_argument(
        "--output",
        type=Path,
        required=True,
        help="Target settings.json path (use a /mnt/c/... path for Windows AirSim).",
    )
    parser.add_argument("--template", type=Path, default=DEFAULT_MANUAL_SETTINGS_TEMPLATE)
    parser.add_argument("--wsl-ip", help="Override automatic WSL IPv4 discovery.")
    parser.add_argument(
        "--windows-host-ip",
        help="Override automatic Windows host/default-gateway IPv4 discovery.",
    )
    parser.add_argument("--camera-width", type=int, default=1280)
    parser.add_argument("--camera-height", type=int, default=720)
    parser.add_argument("--camera-fov", type=float, default=90.0)
    parser.add_argument("--camera-x", type=float, default=0.35)
    parser.add_argument("--camera-y", type=float, default=0.0)
    parser.add_argument("--camera-z", type=float, default=-0.05)
    parser.add_argument("--camera-pitch", type=float, default=0.0)
    parser.add_argument("--camera-roll", type=float, default=0.0)
    parser.add_argument("--camera-yaw", type=float, default=0.0)
    parser.add_argument("--ardupilot-root", type=Path, default=DEFAULT_ARDUPILOT_ROOT)
    parser.add_argument(
        "--parameter-file",
        type=Path,
        default=DEFAULT_M1_PARAMETER_FILE,
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = _parser()
    args = parser.parse_args(argv)
    try:
        camera = ManualCameraConfig(
            width_px=args.camera_width,
            height_px=args.camera_height,
            fov_degrees=args.camera_fov,
            x_m=args.camera_x,
            y_m=args.camera_y,
            z_m=args.camera_z,
            pitch_deg=args.camera_pitch,
            roll_deg=args.camera_roll,
            yaw_deg=args.camera_yaw,
        )
        config = ManualSimulationConfig(
            wsl_ip=args.wsl_ip,
            windows_host_ip=args.windows_host_ip,
            camera=camera,
        )
        output_path = args.output.expanduser().resolve()
        settings_path = write_manual_settings(
            config,
            output_path,
            template_path=args.template,
        )
        argv_result = build_manual_sitl_argv(
            config,
            ardupilot_root=args.ardupilot_root.expanduser().resolve(),
            parameter_file=args.parameter_file.expanduser().resolve(),
        )
    except (OSError, SimulationConfigError) as exc:
        parser.error(str(exc))

    result = {
        "settings_path": str(settings_path),
        "wsl_ip": config.wsl_ip,
        "windows_host_ip": config.windows_host_ip,
        "camera": "front monocular Scene RGB (AirSim ImageType 0)",
        "sitl_argv": list(argv_result),
        "sitl_command": shlex.join(argv_result),
        "processes_started": False,
    }
    print(json.dumps(result, ensure_ascii=True, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

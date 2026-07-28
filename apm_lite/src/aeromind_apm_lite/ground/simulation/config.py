"""Validated configuration for the ROS-free ArduPilot/AirSim runtime."""

from __future__ import annotations

import ipaddress
import math
import os
import re
import socket
import struct
from dataclasses import dataclass, field
from pathlib import Path

AIRSIM_VERSION = "1.8.1"
AIRSIM_SETTINGS_VERSION = 1.2
ARDUPILOT_COMMIT = "1511f27194f1dcc3728270883047bdf022b3fd53"
ARDUPILOT_MODEL = "airsim-copter"
DEFAULT_ARDUPILOT_ROOT = Path("/home/waves/ardupilot")
DEFAULT_ARDUPILOT_EXECUTABLE = (
    DEFAULT_ARDUPILOT_ROOT / "build" / "sitl" / "bin" / "arducopter"
)
DEFAULT_M1_PARAMETER_FILE = (
    Path(__file__).resolve().parents[4]
    / "configs"
    / "sim"
    / "arducopter-m1.parm"
)
DEFAULT_ROUTE_PATH = Path("/proc/net/route")

_SENSOR_PORT_BASE = 9003
_CONTROL_PORT_BASE = 9002
_MAVLINK_PORT_BASE = 14550
_PORT_STRIDE = 10
_MAX_INSTANCE = 254
_VEHICLE_NAME = re.compile(r"^[A-Za-z][A-Za-z0-9_-]{0,31}$")


class SimulationConfigError(ValueError):
    """Raised when a simulation configuration is internally inconsistent."""


def _validate_instance(instance: int) -> int:
    if isinstance(instance, bool) or not isinstance(instance, int):
        raise SimulationConfigError("SITL instance must be an integer")
    if not 0 <= instance <= _MAX_INSTANCE:
        raise SimulationConfigError(
            f"SITL instance must be between 0 and {_MAX_INSTANCE}"
        )
    return instance


def airsim_sensor_port(instance: int) -> int:
    return _SENSOR_PORT_BASE + _PORT_STRIDE * _validate_instance(instance)


def airsim_control_port(instance: int) -> int:
    return _CONTROL_PORT_BASE + _PORT_STRIDE * _validate_instance(instance)


def mavlink_udp_port(instance: int) -> int:
    return _MAVLINK_PORT_BASE + _PORT_STRIDE * _validate_instance(instance)


def _ipv4(value: str, *, label: str, allow_loopback: bool = True) -> str:
    try:
        address = ipaddress.ip_address(value)
    except ValueError as exc:
        raise SimulationConfigError(f"{label} must be an IPv4 address") from exc
    if not isinstance(address, ipaddress.IPv4Address):
        raise SimulationConfigError(f"{label} must be an IPv4 address")
    if address.is_unspecified or address.is_multicast:
        raise SimulationConfigError(f"{label} must be a unicast IPv4 address")
    if address.is_loopback and not allow_loopback:
        raise SimulationConfigError(f"{label} must not be a loopback address")
    return str(address)


def discover_wsl_ipv4() -> str:
    """Return the current routed WSL IPv4 address without invoking a shell."""

    candidates: list[str] = []
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # UDP connect selects a route but sends no packet.
        probe.connect(("8.8.8.8", 53))
        candidates.append(str(probe.getsockname()[0]))
    except OSError:
        pass
    finally:
        probe.close()

    try:
        candidates.extend(
            result[4][0]
            for result in socket.getaddrinfo(
                socket.gethostname(), None, socket.AF_INET, socket.SOCK_DGRAM
            )
        )
    except OSError:
        pass

    for candidate in candidates:
        try:
            return _ipv4(candidate, label="WSL IP", allow_loopback=False)
        except SimulationConfigError:
            continue
    raise SimulationConfigError(
        "could not discover a non-loopback WSL IPv4 address; provide wsl_ip explicitly"
    )


def discover_windows_host_ipv4(
    route_path: Path | str = DEFAULT_ROUTE_PATH,
) -> str:
    """Return the Windows host gateway advertised by the WSL routing table."""

    path = Path(route_path)
    try:
        rows = path.read_text(encoding="ascii").splitlines()[1:]
    except OSError as exc:
        raise SimulationConfigError(
            f"could not read WSL routing table {path}: {exc}"
        ) from exc

    candidates: list[tuple[int, str]] = []
    for row in rows:
        fields = row.split()
        if len(fields) < 8:
            continue
        destination, gateway_hex, flags_hex, metric_text, mask = (
            fields[1],
            fields[2],
            fields[3],
            fields[6],
            fields[7],
        )
        try:
            flags = int(flags_hex, 16)
            metric = int(metric_text)
            gateway = socket.inet_ntoa(
                struct.pack("<I", int(gateway_hex, 16))
            )
        except (OSError, ValueError, struct.error):
            continue
        if destination != "00000000" or mask != "00000000":
            continue
        if not flags & 0x1 or not flags & 0x2:
            continue
        try:
            address = _ipv4(
                gateway,
                label="Windows host gateway",
                allow_loopback=False,
            )
        except SimulationConfigError:
            continue
        candidates.append((metric, address))

    if not candidates:
        raise SimulationConfigError(
            "could not discover the Windows host default gateway from "
            f"{path}; provide simulator_address explicitly"
        )
    return min(candidates)[1]


def _absolute_path(value: Path | str, *, label: str) -> Path:
    path = Path(value).expanduser()
    if not path.is_absolute():
        raise SimulationConfigError(f"{label} must be an absolute path")
    return path


def _finite(value: float, *, label: str) -> float:
    number = float(value)
    if not math.isfinite(number):
        raise SimulationConfigError(f"{label} must be finite")
    return number


def _number_arg(value: float) -> str:
    return f"{value:.8f}".rstrip("0").rstrip(".") or "0"


@dataclass(frozen=True)
class SitlHome:
    latitude: float = 47.641468
    longitude: float = -122.140165
    altitude_m: float = 0.0
    yaw_deg: float = 0.0

    def __post_init__(self) -> None:
        latitude = _finite(self.latitude, label="home latitude")
        longitude = _finite(self.longitude, label="home longitude")
        altitude = _finite(self.altitude_m, label="home altitude")
        yaw = _finite(self.yaw_deg, label="home yaw")
        if not -90.0 <= latitude <= 90.0:
            raise SimulationConfigError("home latitude must be between -90 and 90")
        if not -180.0 <= longitude <= 180.0:
            raise SimulationConfigError("home longitude must be between -180 and 180")
        if not -1000.0 <= altitude <= 10000.0:
            raise SimulationConfigError("home altitude is outside the supported range")
        if not 0.0 <= yaw < 360.0:
            raise SimulationConfigError("home yaw must be in [0, 360)")
        object.__setattr__(self, "latitude", latitude)
        object.__setattr__(self, "longitude", longitude)
        object.__setattr__(self, "altitude_m", altitude)
        object.__setattr__(self, "yaw_deg", yaw)

    @property
    def argv_value(self) -> str:
        return ",".join(
            _number_arg(value)
            for value in (
                self.latitude,
                self.longitude,
                self.altitude_m,
                self.yaw_deg,
            )
        )


@dataclass(frozen=True)
class InitialPoseNed:
    x_m: float = 0.0
    y_m: float = 0.0
    z_m: float = 0.0
    yaw_deg: float = 0.0

    def __post_init__(self) -> None:
        for field_name in ("x_m", "y_m", "z_m", "yaw_deg"):
            value = _finite(getattr(self, field_name), label=field_name)
            object.__setattr__(self, field_name, value)
        if not 0.0 <= self.yaw_deg < 360.0:
            raise SimulationConfigError("initial pose yaw must be in [0, 360)")


@dataclass(frozen=True)
class MavlinkUdpEndpoint:
    host: str
    port: int

    def __post_init__(self) -> None:
        host = _ipv4(self.host, label="MAVLink destination")
        if isinstance(self.port, bool) or not isinstance(self.port, int):
            raise SimulationConfigError("MAVLink UDP port must be an integer")
        if not 1024 <= self.port <= 65535:
            raise SimulationConfigError("MAVLink UDP port must be in [1024, 65535]")
        object.__setattr__(self, "host", host)

    @classmethod
    def for_instance(
        cls, instance: int, host: str = "127.0.0.1"
    ) -> "MavlinkUdpEndpoint":
        return cls(host=host, port=mavlink_udp_port(instance))

    @property
    def serial0_value(self) -> str:
        return f"udpclient:{self.host}:{self.port}"

    @property
    def pymavlink_listener(self) -> str:
        return f"udpin:0.0.0.0:{self.port}"


@dataclass(frozen=True)
class ArduPilotBuild:
    source_root: Path = DEFAULT_ARDUPILOT_ROOT
    executable: Path = DEFAULT_ARDUPILOT_EXECUTABLE
    source_commit: str = ARDUPILOT_COMMIT

    def __post_init__(self) -> None:
        root = _absolute_path(self.source_root, label="ArduPilot source root")
        executable = _absolute_path(self.executable, label="ArduPilot executable")
        expected = root / "build" / "sitl" / "bin" / "arducopter"
        if executable != expected:
            raise SimulationConfigError(
                f"ArduPilot executable must be the SITL build at {expected}"
            )
        if self.source_commit != ARDUPILOT_COMMIT:
            raise SimulationConfigError(
                f"ArduPilot commit must remain pinned to {ARDUPILOT_COMMIT}"
            )
        object.__setattr__(self, "source_root", root)
        object.__setattr__(self, "executable", executable)

    @property
    def defaults_files(self) -> tuple[Path, Path]:
        defaults = self.source_root / "Tools" / "autotest" / "default_params"
        return defaults / "copter.parm", defaults / "airsim-quadX.parm"

    def validate_runtime_files(self, *additional_files: Path) -> None:
        required = (self.executable, *self.defaults_files, *additional_files)
        missing = [str(path) for path in required if not path.is_file()]
        if missing:
            raise SimulationConfigError(
                "missing ArduPilot runtime file(s): " + ", ".join(missing)
            )
        if not os.access(self.executable, os.X_OK):
            raise SimulationConfigError(
                f"ArduPilot executable is not executable: {self.executable}"
            )


@dataclass(frozen=True)
class SitlInstanceConfig:
    instance: int
    build: ArduPilotBuild = field(default_factory=ArduPilotBuild)
    home: SitlHome = field(default_factory=SitlHome)
    mavlink: MavlinkUdpEndpoint | None = None
    simulator_address: str = field(default_factory=discover_windows_host_ipv4)
    model: str = ARDUPILOT_MODEL
    vehicle_name: str | None = None
    initial_pose: InitialPoseNed = field(default_factory=InitialPoseNed)
    parameter_file: Path = DEFAULT_M1_PARAMETER_FILE
    gcs_system_id: int | None = None

    def __post_init__(self) -> None:
        instance = _validate_instance(self.instance)
        if self.model != ARDUPILOT_MODEL:
            raise SimulationConfigError(
                f"SITL model must remain pinned to {ARDUPILOT_MODEL}"
            )
        simulator_address = _ipv4(
            self.simulator_address,
            label="AirSim host address",
            allow_loopback=False,
        )
        parameter_file = _absolute_path(
            self.parameter_file,
            label="SITL parameter file",
        )
        endpoint = self.mavlink or MavlinkUdpEndpoint.for_instance(instance)
        expected_port = mavlink_udp_port(instance)
        if endpoint.port != expected_port:
            raise SimulationConfigError(
                f"instance {instance} requires MAVLink UDP port {expected_port}"
            )
        vehicle_name = self.vehicle_name or f"Drone{instance + 1}"
        if not _VEHICLE_NAME.fullmatch(vehicle_name):
            raise SimulationConfigError("AirSim vehicle_name has an invalid format")
        gcs_system_id = (
            201 + instance if self.gcs_system_id is None else self.gcs_system_id
        )
        if not 200 <= gcs_system_id <= 254:
            raise SimulationConfigError("GCS system id must be in [200, 254]")
        object.__setattr__(self, "instance", instance)
        object.__setattr__(self, "mavlink", endpoint)
        object.__setattr__(self, "simulator_address", simulator_address)
        object.__setattr__(self, "vehicle_name", vehicle_name)
        object.__setattr__(self, "parameter_file", parameter_file)
        object.__setattr__(self, "gcs_system_id", gcs_system_id)

    @property
    def system_id(self) -> int:
        return self.instance + 1

    @property
    def sensor_port(self) -> int:
        return airsim_sensor_port(self.instance)

    @property
    def control_port(self) -> int:
        return airsim_control_port(self.instance)


@dataclass(frozen=True)
class SimulationPaths:
    state_directory: Path
    airsim_settings_path: Path

    def __post_init__(self) -> None:
        state = _absolute_path(self.state_directory, label="simulation state directory")
        settings = _absolute_path(
            self.airsim_settings_path, label="AirSim settings path"
        )
        if state == Path(state.anchor):
            raise SimulationConfigError("simulation state directory must not be a root")
        if settings.name != "settings.json":
            raise SimulationConfigError("AirSim settings path must end in settings.json")
        object.__setattr__(self, "state_directory", state)
        object.__setattr__(self, "airsim_settings_path", settings)

    @property
    def log_directory(self) -> Path:
        return self.state_directory / "logs"

    def instance_directory(self, instance: int) -> Path:
        return self.state_directory / f"sitl-{_validate_instance(instance)}"

    def sitl_log_path(self, instance: int) -> Path:
        return self.log_directory / f"sitl-{_validate_instance(instance)}.log"

    @property
    def airsim_log_path(self) -> Path:
        return self.log_directory / "airsim.log"


@dataclass(frozen=True)
class AirSimProcessConfig:
    """Optional AirSim executable; manual AirSim startup remains supported."""

    executable: Path
    arguments: tuple[str, ...] = ()
    working_directory: Path | None = None

    def __post_init__(self) -> None:
        executable = _absolute_path(self.executable, label="AirSim executable")
        working_directory = self.working_directory
        if working_directory is not None:
            working_directory = _absolute_path(
                working_directory, label="AirSim working directory"
            )
        arguments = tuple(str(argument) for argument in self.arguments)
        if any("\x00" in argument for argument in arguments):
            raise SimulationConfigError("AirSim arguments must not contain NUL")
        if any(argument.lower().startswith("-settings") for argument in arguments):
            raise SimulationConfigError(
                "AirSim -settings is lifecycle-owned; do not provide it in arguments"
            )
        object.__setattr__(self, "executable", executable)
        object.__setattr__(self, "working_directory", working_directory)
        object.__setattr__(self, "arguments", arguments)

    def validate_runtime_files(self) -> None:
        if not self.executable.is_file():
            raise SimulationConfigError(
                f"AirSim executable does not exist: {self.executable}"
            )
        if self.working_directory is not None and not self.working_directory.is_dir():
            raise SimulationConfigError(
                f"AirSim working directory does not exist: {self.working_directory}"
            )


@dataclass(frozen=True)
class SimulationLaunchConfig:
    instances: tuple[SitlInstanceConfig, ...]
    paths: SimulationPaths
    wsl_ip: str = field(default_factory=discover_wsl_ipv4)
    airsim_process: AirSimProcessConfig | None = None
    startup_timeout_s: float = 10.0
    startup_grace_s: float = 0.25
    terminate_timeout_s: float = 5.0
    kill_timeout_s: float = 2.0

    def __post_init__(self) -> None:
        instances = tuple(self.instances)
        if not 1 <= len(instances) <= 32:
            raise SimulationConfigError("simulation requires between 1 and 32 instances")
        identifiers = [instance.instance for instance in instances]
        names = [instance.vehicle_name for instance in instances]
        mavlink_ports = [instance.mavlink.port for instance in instances]
        sensor_ports = [instance.sensor_port for instance in instances]
        control_ports = [instance.control_port for instance in instances]
        gcs_system_ids = [instance.gcs_system_id for instance in instances]
        for label, values in (
            ("instance", identifiers),
            ("vehicle_name", names),
            ("MAVLink port", mavlink_ports),
            ("AirSim sensor port", sensor_ports),
            ("AirSim control port", control_ports),
            ("GCS system id", gcs_system_ids),
        ):
            if len(values) != len(set(values)):
                raise SimulationConfigError(f"duplicate {label} in simulation launch")

        origin = instances[0].home
        if any(instance.home != origin for instance in instances[1:]):
            raise SimulationConfigError(
                "all AirSim vehicles must share one geodetic home; use initial_pose for offsets"
            )
        wsl_ip = _ipv4(self.wsl_ip, label="WSL IP", allow_loopback=False)
        startup_timeout = _finite(self.startup_timeout_s, label="startup timeout")
        startup_grace = _finite(self.startup_grace_s, label="startup grace")
        terminate_timeout = _finite(
            self.terminate_timeout_s, label="terminate timeout"
        )
        kill_timeout = _finite(self.kill_timeout_s, label="kill timeout")
        if not 0.1 <= startup_timeout <= 120.0:
            raise SimulationConfigError("startup timeout must be in [0.1, 120] seconds")
        if not 0.0 <= startup_grace < startup_timeout:
            raise SimulationConfigError("startup grace must be shorter than startup timeout")
        if not 0.1 <= terminate_timeout <= 60.0:
            raise SimulationConfigError("terminate timeout must be in [0.1, 60] seconds")
        if not 0.1 <= kill_timeout <= 30.0:
            raise SimulationConfigError("kill timeout must be in [0.1, 30] seconds")
        object.__setattr__(self, "instances", instances)
        object.__setattr__(self, "wsl_ip", wsl_ip)
        object.__setattr__(self, "startup_timeout_s", startup_timeout)
        object.__setattr__(self, "startup_grace_s", startup_grace)
        object.__setattr__(self, "terminate_timeout_s", terminate_timeout)
        object.__setattr__(self, "kill_timeout_s", kill_timeout)

"""Strict deployment configuration for simulation and real vehicles."""

from __future__ import annotations

from enum import Enum
from pathlib import Path
from typing import Literal

import yaml
from pydantic import BaseModel, ConfigDict, Field, ValidationError, model_validator


class DeploymentMode(str, Enum):
    SIM = "sim"
    REAL = "real"


class FcuTransport(str, Enum):
    UDP = "udp"
    SERIAL = "serial"


class SensorSourceKind(str, Enum):
    NONE = "none"
    AIRSIM = "airsim"
    REALSENSE = "realsense"
    REPLAY = "replay"


class ConfigModel(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True, allow_inf_nan=False)


class FcuConnection(ConfigModel):
    transport: FcuTransport
    endpoint: str = Field(min_length=1, max_length=256)
    baudrate: int | None = Field(default=None, ge=9_600, le=2_000_000)
    source_system: int = Field(ge=200, le=254)
    source_component: int = Field(default=191, ge=1, le=255)
    target_system: int = Field(ge=1, le=255)
    target_component: int = Field(default=1, ge=1, le=255)

    @model_validator(mode="after")
    def transport_fields_match(self) -> "FcuConnection":
        if self.transport == FcuTransport.SERIAL and self.baudrate is None:
            raise ValueError("serial FCU connections require baudrate")
        if self.transport == FcuTransport.UDP and self.baudrate is not None:
            raise ValueError("UDP FCU connections must not define baudrate")
        if self.transport == FcuTransport.SERIAL and not self.endpoint.startswith("/dev/"):
            raise ValueError("serial FCU endpoint must be an absolute /dev path")
        if self.transport == FcuTransport.UDP and not self.endpoint.startswith(
            ("udp:", "udpin:", "udpout:")
        ):
            raise ValueError("UDP FCU endpoint must use udp, udpin or udpout syntax")
        return self


class VehicleRuntimeConfig(ConfigModel):
    vehicle_id: int = Field(ge=1, le=255)
    name: str = Field(min_length=1, max_length=64)
    runtime_host: str = Field(
        min_length=1,
        max_length=128,
        pattern=r"^[A-Za-z0-9][A-Za-z0-9._-]*$",
    )
    frame_calibration_id: str = Field(min_length=1, max_length=128)
    credential_env: str = Field(pattern=r"^[A-Z][A-Z0-9_]*$")
    sensor_source: SensorSourceKind
    fcu: FcuConnection
    setpoint_rate_hz: float = Field(default=20.0, ge=10.0, le=50.0)
    trajectory_timeout_ms: int = Field(default=1_000, ge=250, le=10_000)
    companion_heartbeat_hz: float = Field(default=1.0, ge=0.5, le=5.0)
    startup_timeout_s: float = Field(default=5.0, ge=1.0, le=60.0)
    heartbeat_timeout_s: float = Field(default=3.0, ge=0.5, le=30.0)
    ack_timeout_s: float = Field(default=2.0, ge=0.2, le=30.0)
    physical_timeout_s: float = Field(default=30.0, ge=1.0, le=300.0)
    telemetry_freshness_s: float = Field(default=1.0, ge=0.1, le=10.0)
    completion_hold_s: float = Field(default=0.5, ge=0.1, le=5.0)
    guided_mode: str = Field(default="GUIDED", pattern=r"^[A-Z][A-Z0-9_]{1,31}$")
    hold_mode: str = Field(default="LOITER", pattern=r"^[A-Z][A-Z0-9_]{1,31}$")

    @model_validator(mode="after")
    def target_system_matches_vehicle(self) -> "VehicleRuntimeConfig":
        if self.vehicle_id != self.fcu.target_system:
            raise ValueError("vehicle_id must equal fcu.target_system")
        return self


class FleetConfig(ConfigModel):
    schema_version: Literal["1.0"] = "1.0"
    mode: DeploymentMode
    fleet_name: str = Field(min_length=1, max_length=64)
    vehicles: list[VehicleRuntimeConfig] = Field(min_length=1, max_length=32)

    @model_validator(mode="after")
    def validate_fleet(self) -> "FleetConfig":
        vehicle_ids = [vehicle.vehicle_id for vehicle in self.vehicles]
        source_systems = [vehicle.fcu.source_system for vehicle in self.vehicles]
        endpoint_scopes = [
            (vehicle.runtime_host, vehicle.fcu.endpoint) for vehicle in self.vehicles
        ]
        credentials = [vehicle.credential_env for vehicle in self.vehicles]

        for label, values in (
            ("vehicle_id", vehicle_ids),
            ("source_system", source_systems),
            ("runtime host and FCU endpoint", endpoint_scopes),
            ("credential_env", credentials),
        ):
            if len(values) != len(set(values)):
                raise ValueError(f"fleet contains duplicate {label}")

        expected_transport = (
            FcuTransport.UDP if self.mode == DeploymentMode.SIM else FcuTransport.SERIAL
        )
        allowed_sensors = (
            {SensorSourceKind.AIRSIM}
            if self.mode == DeploymentMode.SIM
            else {SensorSourceKind.NONE, SensorSourceKind.REALSENSE}
        )
        for vehicle in self.vehicles:
            if vehicle.fcu.transport != expected_transport:
                raise ValueError(f"{self.mode.value} mode requires {expected_transport.value} FCU")
            if vehicle.sensor_source not in allowed_sensors:
                choices = " or ".join(sorted(sensor.value for sensor in allowed_sensors))
                raise ValueError(
                    f"{self.mode.value} mode requires {choices} sensor source"
                )
        return self


class ConfigError(ValueError):
    pass


def load_fleet_config(path: str | Path) -> FleetConfig:
    config_path = Path(path)
    try:
        raw = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise ConfigError(f"failed to read {config_path}: {exc}") from exc
    try:
        return FleetConfig.model_validate(raw)
    except ValidationError as exc:
        raise ConfigError(f"invalid fleet config {config_path}: {exc}") from exc

"""YAML configuration for the four-endpoint bridge."""

from __future__ import annotations

from pathlib import Path
from typing import Literal

import yaml
from pydantic import Field, ValidationError, field_validator, model_validator

from .models import AdapterKind, ParallelModel, SafetyPolicy


class MavlinkEndpointConfig(ParallelModel):
    endpoint: str = Field(min_length=1, max_length=256)
    simulator: Literal["gazebo", "native-sitl", "airsim"] | None = None
    dialect: Literal["common", "ardupilotmega"] = "common"
    source_system: int = Field(default=250, ge=1, le=255)
    source_component: int = Field(default=190, ge=1, le=255)
    target_system: int = Field(ge=1, le=255)
    target_component: int = Field(default=1, ge=1, le=255)
    force_arm_for_sitl: bool = False

    @field_validator("endpoint")
    @classmethod
    def endpoint_is_udp_listener(cls, value: str) -> str:
        if not value.startswith("udpin:"):
            raise ValueError("bridge endpoints must use explicit udpin: URLs")
        return value


class ParallelLineConfig(ParallelModel):
    line_id: str = Field(
        min_length=1,
        max_length=64,
        pattern=r"^[a-z][a-z0-9_-]*$",
    )
    adapter: AdapterKind
    vehicle_id: int = Field(ge=1, le=255)
    platform_type: str = Field(
        min_length=1,
        max_length=64,
        pattern=r"^[a-z][a-z0-9_-]*$",
    )
    real: MavlinkEndpointConfig
    virtual: MavlinkEndpointConfig
    policy: SafetyPolicy

    @model_validator(mode="after")
    def target_ids_match_vehicle(self) -> "ParallelLineConfig":
        for role, endpoint in (("real", self.real), ("virtual", self.virtual)):
            if endpoint.target_system != self.vehicle_id:
                raise ValueError(
                    f"{role} target_system must equal vehicle_id {self.vehicle_id}"
                )
        if self.real.simulator not in {None, "gazebo", "native-sitl"}:
            raise ValueError("real endpoint simulator must be gazebo or native-sitl")
        if self.virtual.simulator not in {None, "airsim"}:
            raise ValueError("virtual endpoint simulator must be airsim")
        expected_dialect = "common" if self.adapter == AdapterKind.PX4 else "ardupilotmega"
        for role, endpoint in (("real", self.real), ("virtual", self.virtual)):
            if endpoint.dialect != expected_dialect:
                raise ValueError(f"{role} endpoint dialect must be {expected_dialect}")
        return self


class BridgeConfig(ParallelModel):
    name: str = Field(default="yice-parallel-domain", min_length=1, max_length=128)
    georeference: Path
    state_directory: Path
    poll_interval_s: float = Field(default=0.25, ge=0.05, le=5.0)
    observation_only: bool = True
    live_fault_injection_enabled: bool = False
    safety_hold_duration_s: float = Field(default=1.0, ge=0.1, le=10.0)
    telemetry_stale_after_s: float = Field(default=3.0, ge=0.25, le=60.0)
    recovery_samples: int = Field(default=2, ge=1, le=100)
    reconnect_delay_s: float = Field(default=1.0, ge=0.05, le=60.0)
    mavlink_silence_timeout_s: float = Field(default=5.0, ge=0.5, le=120.0)
    telemetry_sample_interval_s: float = Field(default=0.1, ge=0.01, le=1.0)
    lines: tuple[ParallelLineConfig, ...] = Field(min_length=1, max_length=16)

    @model_validator(mode="after")
    def lines_are_unambiguous(self) -> "BridgeConfig":
        line_ids = [line.line_id for line in self.lines]
        vehicle_ids = [line.vehicle_id for line in self.lines]
        endpoints = [
            endpoint.endpoint
            for line in self.lines
            for endpoint in (line.real, line.virtual)
        ]
        if len(line_ids) != len(set(line_ids)):
            raise ValueError("line_id values must be unique")
        if len(vehicle_ids) != len(set(vehicle_ids)):
            raise ValueError("vehicle_id values must be unique")
        if len(endpoints) != len(set(endpoints)):
            raise ValueError("MAVLink endpoints must be unique")
        if self.live_fault_injection_enabled:
            if self.observation_only:
                raise ValueError("live fault injection requires a control bridge")
            if any(line.real.simulator is None for line in self.lines):
                raise ValueError("live fault injection requires explicit SITL simulators")
        return self


class BridgeConfigError(ValueError):
    pass


def load_bridge_config(path: str | Path) -> BridgeConfig:
    config_path = Path(path).resolve()
    try:
        payload = yaml.safe_load(config_path.read_text(encoding="utf-8"))
        config = BridgeConfig.model_validate(payload)
    except (OSError, yaml.YAMLError, ValidationError) as exc:
        raise BridgeConfigError(f"invalid bridge config {config_path}: {exc}") from exc
    base = config_path.parent
    return config.model_copy(
        update={
            "georeference": _resolve(base, config.georeference),
            "state_directory": _resolve(base, config.state_directory),
        }
    )


def _resolve(base: Path, value: Path) -> Path:
    return value if value.is_absolute() else (base / value).resolve()


__all__ = [
    "BridgeConfig",
    "BridgeConfigError",
    "MavlinkEndpointConfig",
    "ParallelLineConfig",
    "load_bridge_config",
]

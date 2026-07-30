"""Versioned venue georeference models and YAML persistence."""

from __future__ import annotations

import hashlib
import json
import math
import os
import tempfile
from enum import Enum
from pathlib import Path
from typing import Any, Literal

import yaml
from pydantic import BaseModel, ConfigDict, Field, ValidationError, model_validator

from .frames import (
    GeodeticPosition,
    Vec3,
    VenueCalibration,
    airsim_ned_to_map,
    geodetic_to_enu,
    geodetic_to_map,
    local_ned_to_map,
    map_to_airsim_ned,
    map_to_geodetic,
    map_to_local_ned,
)


class CalibrationStatus(str, Enum):
    DRAFT = "draft"
    SURVEYED = "surveyed"


class AltitudeDatum(str, Enum):
    AMSL = "amsl"
    WGS84_ELLIPSOID = "wgs84_ellipsoid"


class GeoReferenceModel(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True, allow_inf_nan=False)


class Wgs84Position(GeoReferenceModel):
    latitude_deg: float = Field(ge=-90.0, le=90.0)
    longitude_deg: float = Field(ge=-180.0, le=180.0)
    altitude_m: float = Field(ge=-1_000.0, le=20_000.0)

    def coordinate(self) -> GeodeticPosition:
        return GeodeticPosition(
            latitude_deg=self.latitude_deg,
            longitude_deg=self.longitude_deg,
            altitude_m=self.altitude_m,
        )


class MapDefinition(GeoReferenceModel):
    origin_marker: str = Field(default="O", min_length=1, max_length=32)
    positive_x_marker: str = Field(default="X1", min_length=1, max_length=32)
    x_axis_reference: Literal["true_north"] = "true_north"
    z_axis: Literal["up"] = "up"
    units: Literal["m"] = "m"


class HeightReference(GeoReferenceModel):
    datum: AltitudeDatum = AltitudeDatum.AMSL
    map_z_zero: Literal["map_origin"] = "map_origin"


class VehicleHome(GeoReferenceModel):
    vehicle_id: int = Field(ge=1, le=255)
    map_position_m: tuple[float, float, float]
    expected_wgs84: Wgs84Position | None = None


class GeoReference(GeoReferenceModel):
    """One immutable-by-id mapping between the real venue and AirSim."""

    schema_version: Literal["1.0"] = "1.0"
    calibration_id: str = Field(
        min_length=1,
        max_length=128,
        pattern=r"^[A-Za-z0-9][A-Za-z0-9._-]*$",
    )
    status: CalibrationStatus = CalibrationStatus.DRAFT
    map_definition: MapDefinition = Field(default_factory=MapDefinition)
    map_origin_wgs84: Wgs84Position | None = None
    map_x_heading_from_true_north_deg: float | None = Field(
        default=None,
        ge=-180.0,
        le=180.0,
    )
    airsim_origin_wgs84: Wgs84Position | None = None
    map_scale: Literal[1.0] = 1.0
    height_reference: HeightReference = Field(default_factory=HeightReference)
    vehicle_homes: tuple[VehicleHome, ...] = Field(
        min_length=1,
        max_length=32,
    )
    notes: str = Field(default="", max_length=1_000)

    @model_validator(mode="after")
    def validate_reference(self) -> "GeoReference":
        vehicle_ids = [home.vehicle_id for home in self.vehicle_homes]
        if len(vehicle_ids) != len(set(vehicle_ids)):
            raise ValueError("vehicle_homes contains duplicate vehicle_id")
        if self.status == CalibrationStatus.SURVEYED:
            if self.map_origin_wgs84 is None:
                raise ValueError("surveyed calibration requires map_origin_wgs84")
            if self.map_x_heading_from_true_north_deg is None:
                raise ValueError(
                    "surveyed calibration requires map X heading from true north"
                )
            if self.airsim_origin_wgs84 is None:
                raise ValueError("surveyed calibration requires airsim_origin_wgs84")
        return self

    @property
    def config_hash(self) -> str:
        canonical = json.dumps(
            self.model_dump(mode="json"),
            ensure_ascii=True,
            allow_nan=False,
            sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8")
        return hashlib.sha256(canonical).hexdigest()

    @property
    def is_complete(self) -> bool:
        return all(
            value is not None
            for value in (
                self.map_origin_wgs84,
                self.map_x_heading_from_true_north_deg,
                self.airsim_origin_wgs84,
            )
        )

    def venue_calibration(
        self,
        *,
        enu_origin: Wgs84Position | None = None,
    ) -> VenueCalibration:
        if not self.is_complete:
            raise ValueError("georeference is incomplete")
        assert self.map_origin_wgs84 is not None
        assert self.map_x_heading_from_true_north_deg is not None
        reference = enu_origin or self.map_origin_wgs84
        map_origin_enu = geodetic_to_enu(
            self.map_origin_wgs84.coordinate(),
            reference.coordinate(),
        )
        yaw_from_east = math.radians(
            90.0 - self.map_x_heading_from_true_north_deg
        )
        return VenueCalibration(
            calibration_id=self.calibration_id,
            map_origin_enu_m=map_origin_enu,
            map_x_yaw_from_east_rad=yaw_from_east,
        )

    def coordinate_round_trip_report(self) -> dict[str, Any]:
        if not self.is_complete:
            return {
                "ready": False,
                "passed": False,
                "detail": "map origin, map X heading and AirSim origin are required",
                "points": [],
            }

        assert self.map_origin_wgs84 is not None
        assert self.airsim_origin_wgs84 is not None
        map_calibration = self.venue_calibration()
        airsim_calibration = self.venue_calibration(
            enu_origin=self.airsim_origin_wgs84
        )
        points = [("map_origin", Vec3(0.0, 0.0, 0.0), None)]
        points.extend(
            (
                f"uav{home.vehicle_id}_home",
                Vec3(*home.map_position_m),
                home,
            )
            for home in self.vehicle_homes
        )

        results: list[dict[str, Any]] = []
        maximum_error = 0.0
        for label, map_point, home in points:
            geodetic = map_to_geodetic(
                map_point,
                map_calibration,
                self.map_origin_wgs84.coordinate(),
            )
            geodetic_back = geodetic_to_map(
                geodetic,
                map_calibration,
                self.map_origin_wgs84.coordinate(),
            )
            airsim_ned = map_to_airsim_ned(map_point, airsim_calibration)
            airsim_back = airsim_ned_to_map(airsim_ned, airsim_calibration)
            errors = {
                "wgs84_map_m": _distance(map_point, geodetic_back),
                "airsim_map_m": _distance(map_point, airsim_back),
            }
            if home is not None:
                home_enu = geodetic_to_enu(
                    geodetic,
                    self.map_origin_wgs84.coordinate(),
                )
                local_ned = map_to_local_ned(
                    map_point,
                    map_calibration,
                    home_enu,
                )
                local_back = local_ned_to_map(
                    local_ned,
                    map_calibration,
                    home_enu,
                )
                errors["local_ned_map_m"] = _distance(map_point, local_back)
            maximum_error = max(maximum_error, *errors.values())
            results.append(
                {
                    "label": label,
                    "map_position_m": [map_point.x, map_point.y, map_point.z],
                    "wgs84": {
                        "latitude_deg": geodetic.latitude_deg,
                        "longitude_deg": geodetic.longitude_deg,
                        "altitude_m": geodetic.altitude_m,
                    },
                    "airsim_ned_m": [
                        airsim_ned.x,
                        airsim_ned.y,
                        airsim_ned.z,
                    ],
                    "errors_m": errors,
                }
            )
        return {
            "ready": True,
            "passed": maximum_error <= 0.01,
            "tolerance_m": 0.01,
            "maximum_error_m": maximum_error,
            "points": results,
        }


class GeoReferenceError(ValueError):
    pass


class GeoReferenceConflict(GeoReferenceError):
    pass


def default_georeference() -> GeoReference:
    return GeoReference(
        calibration_id="venue-draft",
        vehicle_homes=tuple(
            VehicleHome(vehicle_id=index, map_position_m=(0.0, 3.0 * (index - 1), 0.0))
            for index in range(1, 5)
        ),
    )


def load_georeference(path: str | Path) -> GeoReference:
    config_path = Path(path)
    try:
        raw = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise GeoReferenceError(f"failed to read {config_path}: {exc}") from exc
    try:
        return GeoReference.model_validate(raw)
    except ValidationError as exc:
        raise GeoReferenceError(
            f"invalid georeference config {config_path}: {exc}"
        ) from exc


class GeoReferenceStore:
    """Hold the active config and persist updates atomically when configured."""

    def __init__(self, path: str | Path | None = None) -> None:
        self.path = Path(path) if path is not None else None
        self._value = (
            load_georeference(self.path)
            if self.path is not None and self.path.is_file()
            else default_georeference()
        )

    @property
    def value(self) -> GeoReference:
        return self._value

    def save(self, value: GeoReference) -> GeoReference:
        current = self._value
        if (
            current.status == CalibrationStatus.SURVEYED
            and current.calibration_id == value.calibration_id
            and current.config_hash != value.config_hash
        ):
            raise GeoReferenceConflict(
                "surveyed calibration is immutable; use a new calibration_id"
            )
        if self.path is not None:
            _atomic_yaml_write(self.path, value.model_dump(mode="json"))
        self._value = value
        return value

    def public_payload(self) -> dict[str, Any]:
        value = self._value
        return {
            "configuration": value.model_dump(mode="json"),
            "config_hash": value.config_hash,
            "immutable": value.status == CalibrationStatus.SURVEYED,
            "round_trip_report": value.coordinate_round_trip_report(),
        }


def _distance(left: Vec3, right: Vec3) -> float:
    return math.sqrt(
        (left.x - right.x) ** 2
        + (left.y - right.y) ** 2
        + (left.z - right.z) ** 2
    )


def _atomic_yaml_write(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            newline="\n",
            dir=path.parent,
            prefix=f".{path.name}.",
            suffix=".tmp",
            delete=False,
        ) as temporary:
            temporary_path = Path(temporary.name)
            yaml.safe_dump(
                payload,
                temporary,
                allow_unicode=True,
                sort_keys=False,
            )
            temporary.flush()
            os.fsync(temporary.fileno())
        os.replace(temporary_path, path)
    except OSError as exc:
        raise GeoReferenceError(f"failed to write {path}: {exc}") from exc
    finally:
        if temporary_path is not None and temporary_path.exists():
            temporary_path.unlink()

"""Offline GPS CSV replay into normalized map-frame trajectory evidence."""

from __future__ import annotations

import csv
import hashlib
import math
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from uuid import UUID

from aeromind_apm_lite.common.coordinates import (
    CalibrationStatus,
    GeoReference,
    GeodeticPosition,
    Vec3,
    geodetic_to_map,
)

from .evidence import (
    ArtifactVersions,
    GpsQuality,
    TrajectoryEvidence,
    TrajectoryRole,
    TrajectorySample,
    TrajectorySource,
)


@dataclass(frozen=True)
class GpsLogRecord:
    observed_at_utc: datetime
    latitude_deg: float
    longitude_deg: float
    altitude_m: float
    fix_type: int | None = None
    hdop: float | None = None
    satellites_visible: int | None = None

    def __post_init__(self) -> None:
        timestamp = self.observed_at_utc
        if timestamp.tzinfo is None or timestamp.utcoffset() is None:
            raise ValueError("GPS timestamp must include a timezone")
        object.__setattr__(self, "observed_at_utc", timestamp.astimezone(timezone.utc))
        GeodeticPosition(self.latitude_deg, self.longitude_deg, self.altitude_m)
        if self.fix_type is not None and not 0 <= self.fix_type <= 6:
            raise ValueError("GPS fix_type must be in [0, 6]")
        if self.hdop is not None and not 0.0 <= self.hdop <= 100.0:
            raise ValueError("GPS hdop must be in [0, 100]")
        if self.satellites_visible is not None and not 0 <= self.satellites_visible <= 255:
            raise ValueError("GPS satellites_visible must be in [0, 255]")


class GpsReplayError(ValueError):
    pass


_COLUMN_ALIASES = {
    "timestamp": ("observed_at_utc", "timestamp_utc", "timestamp", "time_utc"),
    "latitude": ("latitude_deg", "latitude", "lat"),
    "longitude": ("longitude_deg", "longitude", "lon", "lng"),
    "altitude": ("altitude_m", "altitude", "alt_m", "alt"),
    "fix_type": ("gps_fix_type", "fix_type"),
    "hdop": ("gps_hdop", "hdop"),
    "satellites": ("satellites_visible", "satellites", "sat_count"),
}


def read_gps_csv(path: str | Path) -> tuple[GpsLogRecord, ...]:
    source = Path(path)
    try:
        with source.open("r", encoding="utf-8-sig", newline="") as stream:
            reader = csv.DictReader(stream)
            if reader.fieldnames is None:
                raise GpsReplayError("GPS CSV has no header")
            columns = _resolve_columns(reader.fieldnames)
            records = tuple(
                _parse_row(row, line_number, columns)
                for line_number, row in enumerate(reader, start=2)
                if any(str(value or "").strip() for value in row.values())
            )
    except OSError as exc:
        raise GpsReplayError(f"failed to read GPS CSV {source}: {exc}") from exc
    if len(records) < 2:
        raise GpsReplayError("GPS CSV must contain at least two records")
    for previous, current in zip(records, records[1:]):
        if current.observed_at_utc < previous.observed_at_utc:
            raise GpsReplayError("GPS CSV timestamps must not move backwards")
    return records


def replay_gps_csv(
    path: str | Path,
    georeference: GeoReference,
    *,
    mission_id: UUID,
    vehicle_id: int,
    producer_version: str,
    versions: ArtifactVersions | None = None,
    allow_draft: bool = False,
) -> TrajectoryEvidence:
    if not georeference.is_complete:
        raise GpsReplayError("GPS replay requires a complete GeoReference")
    if georeference.status != CalibrationStatus.SURVEYED and not allow_draft:
        raise GpsReplayError(
            "GPS replay requires surveyed calibration; use allow_draft only for preview"
        )
    assert georeference.map_origin_wgs84 is not None
    source = Path(path)
    records = read_gps_csv(source)
    calibration = georeference.venue_calibration()
    started_at = records[0].observed_at_utc
    samples: list[TrajectorySample] = []
    duplicate_timestamps = 0
    for record in records:
        elapsed_ms = round((record.observed_at_utc - started_at).total_seconds() * 1_000)
        if samples and elapsed_ms <= samples[-1].time_from_start_ms:
            duplicate_timestamps += 1
            continue
        source_wgs84 = GeodeticPosition(
            latitude_deg=record.latitude_deg,
            longitude_deg=record.longitude_deg,
            altitude_m=record.altitude_m,
        )
        point_map = geodetic_to_map(
            source_wgs84,
            calibration,
            georeference.map_origin_wgs84.coordinate(),
        )
        samples.append(
            TrajectorySample(
                time_from_start_ms=elapsed_ms,
                position_map_m=_vec_tuple(point_map),
                observed_at_utc=record.observed_at_utc,
                source_wgs84={
                    "latitude_deg": record.latitude_deg,
                    "longitude_deg": record.longitude_deg,
                    "altitude_m": record.altitude_m,
                },
                gps_quality=GpsQuality(
                    fix_type=record.fix_type,
                    hdop=record.hdop,
                    satellites_visible=record.satellites_visible,
                ),
            )
        )
    if len(samples) < 2:
        raise GpsReplayError("GPS replay produced fewer than two unique samples")
    try:
        source_hash = hashlib.sha256(source.read_bytes()).hexdigest()
    except OSError as exc:
        raise GpsReplayError(f"failed to hash GPS CSV {source}: {exc}") from exc
    return TrajectoryEvidence(
        mission_id=mission_id,
        vehicle_id=vehicle_id,
        role=TrajectoryRole.OBSERVED,
        source=TrajectorySource.GPS_LOG_REPLAY,
        frame_calibration_id=georeference.calibration_id,
        frame_calibration_hash=georeference.config_hash,
        calibration_status=georeference.status,
        preview_only=georeference.status != CalibrationStatus.SURVEYED,
        producer_version=producer_version,
        started_at_utc=started_at,
        versions=versions or ArtifactVersions(),
        source_metadata={
            "input_format": "gps_csv_v1",
            "source_file_sha256": source_hash,
            "input_record_count": len(records),
            "emitted_sample_count": len(samples),
            "duplicate_timestamp_count": duplicate_timestamps,
        },
        samples=tuple(samples),
    )


def _resolve_columns(fieldnames: list[str]) -> dict[str, str | None]:
    available = {name.strip().lower(): name for name in fieldnames}
    resolved: dict[str, str | None] = {}
    for label, aliases in _COLUMN_ALIASES.items():
        resolved[label] = next(
            (available[alias] for alias in aliases if alias in available),
            None,
        )
    missing = [
        label
        for label in ("timestamp", "latitude", "longitude", "altitude")
        if resolved[label] is None
    ]
    if missing:
        raise GpsReplayError("GPS CSV missing required columns: " + ", ".join(missing))
    return resolved


def _parse_row(
    row: dict[str, str | None],
    line_number: int,
    columns: dict[str, str | None],
) -> GpsLogRecord:
    try:
        timestamp = _parse_timestamp(_required_value(row, columns["timestamp"]))
        latitude = float(_required_value(row, columns["latitude"]))
        longitude = float(_required_value(row, columns["longitude"]))
        altitude = float(_required_value(row, columns["altitude"]))
        if not all(math.isfinite(value) for value in (latitude, longitude, altitude)):
            raise ValueError("coordinates must be finite")
        return GpsLogRecord(
            observed_at_utc=timestamp,
            latitude_deg=latitude,
            longitude_deg=longitude,
            altitude_m=altitude,
            fix_type=_optional_int(row, columns["fix_type"]),
            hdop=_optional_float(row, columns["hdop"]),
            satellites_visible=_optional_int(row, columns["satellites"]),
        )
    except (TypeError, ValueError) as exc:
        raise GpsReplayError(f"invalid GPS CSV row {line_number}: {exc}") from exc


def _parse_timestamp(value: str) -> datetime:
    normalized = value[:-1] + "+00:00" if value.endswith("Z") else value
    parsed = datetime.fromisoformat(normalized)
    if parsed.tzinfo is None or parsed.utcoffset() is None:
        raise ValueError("timestamp must include a timezone")
    return parsed.astimezone(timezone.utc)


def _required_value(row: dict[str, str | None], column: str | None) -> str:
    if column is None:
        raise ValueError("required column was not resolved")
    value = (row.get(column) or "").strip()
    if not value:
        raise ValueError(f"{column} is empty")
    return value


def _optional_int(row: dict[str, str | None], column: str | None) -> int | None:
    value = (row.get(column) or "").strip() if column is not None else ""
    return int(value) if value else None


def _optional_float(row: dict[str, str | None], column: str | None) -> float | None:
    value = (row.get(column) or "").strip() if column is not None else ""
    return float(value) if value else None


def _vec_tuple(value: Vec3) -> tuple[float, float, float]:
    return value.x, value.y, value.z

"""Capture SITL navigation observations as planned and predicted evidence."""

from __future__ import annotations

import hashlib
import json
from datetime import datetime, timedelta, timezone

from aeromind_apm_lite.common.coordinates import (
    CalibrationStatus,
    GeoReference,
    Vec3,
    local_ned_to_map,
    map_to_enu,
)
from aeromind_apm_lite.common.trajectory import (
    ArtifactVersions,
    GpsQuality,
    TrajectoryEvidence,
    TrajectorySample,
    TrajectoryRole,
    TrajectorySource,
)
from aeromind_apm_lite.onboard import NavigationMissionPlan, NavigationObservation
from aeromind_apm_lite.onboard.mavlink import TelemetrySnapshot


class NavigationTrajectoryRecorder:
    """Record raw SITL telemetry before observation-layer fault injection."""

    def __init__(
        self,
        georeference: GeoReference,
        plan: NavigationMissionPlan,
        *,
        vehicle_id: int,
        producer_version: str,
        versions: ArtifactVersions | None = None,
        started_at_utc: datetime | None = None,
    ) -> None:
        if not georeference.is_complete:
            raise ValueError("trajectory recording requires a complete GeoReference")
        homes = [
            home for home in georeference.vehicle_homes if home.vehicle_id == vehicle_id
        ]
        if len(homes) != 1:
            raise ValueError(f"GeoReference requires exactly one Home for vehicle {vehicle_id}")
        timestamp = started_at_utc or datetime.now(timezone.utc)
        if timestamp.tzinfo is None or timestamp.utcoffset() is None:
            raise ValueError("started_at_utc must include a timezone")
        self.georeference = georeference
        self.plan = plan
        self.vehicle_id = vehicle_id
        self.producer_version = producer_version
        self.started_at_utc = timestamp.astimezone(timezone.utc)
        self._calibration = georeference.venue_calibration()
        self._home_map = Vec3(*homes[0].map_position_m)
        self._home_enu = map_to_enu(self._home_map, self._calibration)
        self._versions = (versions or ArtifactVersions()).model_copy(
            update={"mission_plan_hash": _mission_plan_hash(plan)}
        )
        self._started_monotonic_s: float | None = None
        self._records: list[tuple[str, int, TelemetrySnapshot]] = []
        self._timestamp_adjustments = 0

    def observation_filter(
        self,
        phase: str,
        observation: NavigationObservation,
    ) -> NavigationObservation:
        self.record_snapshot(
            phase,
            observation.snapshot,
            observation.observed_monotonic_s,
        )
        return observation

    def record_snapshot(
        self,
        phase: str,
        snapshot: TelemetrySnapshot,
        observed_monotonic_s: float,
    ) -> None:
        if snapshot.local_position_ned_m is None:
            return
        if self._started_monotonic_s is None:
            self._started_monotonic_s = observed_monotonic_s
        elapsed_ms = round((observed_monotonic_s - self._started_monotonic_s) * 1_000)
        if self._records and elapsed_ms <= self._records[-1][1]:
            elapsed_ms = self._records[-1][1] + 1
            self._timestamp_adjustments += 1
        self._records.append((phase, elapsed_ms, snapshot))

    def finalize(self) -> tuple[TrajectoryEvidence, TrajectoryEvidence]:
        if len(self._records) < 2:
            raise ValueError("trajectory recording requires at least two position observations")
        predicted_samples = tuple(
            self._predicted_sample(phase, elapsed_ms, snapshot)
            for phase, elapsed_ms, snapshot in self._records
        )
        planned_samples = tuple(
            TrajectorySample(
                time_from_start_ms=elapsed_ms,
                position_map_m=_vec_tuple(
                    self._planned_map_position(phase)
                ),
                phase=phase,
            )
            for phase, elapsed_ms, _snapshot in self._records
        )
        common = {
            "mission_id": self.plan.mission_id,
            "vehicle_id": self.vehicle_id,
            "frame_calibration_id": self.georeference.calibration_id,
            "frame_calibration_hash": self.georeference.config_hash,
            "calibration_status": self.georeference.status,
            "preview_only": self.georeference.status != CalibrationStatus.SURVEYED,
            "producer_version": self.producer_version,
            "started_at_utc": self.started_at_utc,
            "versions": self._versions,
            "source_metadata": {
                "recording_mode": "navigation_observation_v1",
                "sample_count": len(self._records),
                "timestamp_quantization_adjustments": self._timestamp_adjustments,
            },
        }
        planned = TrajectoryEvidence(
            role=TrajectoryRole.PLANNED,
            source=TrajectorySource.MISSION_PLAN,
            samples=planned_samples,
            **common,
        )
        predicted = TrajectoryEvidence(
            role=TrajectoryRole.PREDICTED,
            source=TrajectorySource.SITL,
            samples=predicted_samples,
            **common,
        )
        return planned, predicted

    def _predicted_sample(
        self,
        phase: str,
        elapsed_ms: int,
        snapshot: TelemetrySnapshot,
    ) -> TrajectorySample:
        assert snapshot.local_position_ned_m is not None
        point_map = local_ned_to_map(
            Vec3(*snapshot.local_position_ned_m),
            self._calibration,
            self._home_enu,
        )
        wgs84 = None
        if snapshot.global_position_deg_m is not None:
            wgs84 = {
                "latitude_deg": snapshot.global_position_deg_m[0],
                "longitude_deg": snapshot.global_position_deg_m[1],
                "altitude_m": snapshot.global_position_deg_m[2],
            }
        return TrajectorySample(
            time_from_start_ms=elapsed_ms,
            position_map_m=_vec_tuple(point_map),
            phase=phase,
            observed_at_utc=self.started_at_utc + timedelta(milliseconds=elapsed_ms),
            source_wgs84=wgs84,
            gps_quality=GpsQuality(
                fix_type=snapshot.gps_fix_type,
                hdop=snapshot.gps_hdop,
                satellites_visible=snapshot.satellites_visible,
            ),
        )

    def _planned_map_position(self, phase: str) -> Vec3:
        if phase in {"preflight", "guided", "arm", "landed"}:
            target_ned = (0.0, 0.0, 0.0)
        elif phase == "takeoff":
            target_ned = (0.0, 0.0, -self.plan.takeoff_altitude_m)
        else:
            target_ned = self.plan.target_position_ned_m
        return local_ned_to_map(
            Vec3(*target_ned),
            self._calibration,
            self._home_enu,
        )


def _mission_plan_hash(plan: NavigationMissionPlan) -> str:
    payload = {
        "mission_id": str(plan.mission_id),
        "target_position_ned_m": list(plan.target_position_ned_m),
        "takeoff_altitude_m": plan.takeoff_altitude_m,
        "yaw_rad": plan.yaw_rad,
        "mission_timeout_s": plan.mission_timeout_s,
        "goto_timeout_s": plan.goto_timeout_s,
    }
    encoded = json.dumps(
        payload,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _vec_tuple(value: Vec3) -> tuple[float, float, float]:
    return value.x, value.y, value.z

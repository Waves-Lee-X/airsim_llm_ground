"""FCU telemetry reduction with independent per-category freshness."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from .mavlink.models import MavlinkEnvelope, TelemetrySnapshot

MAV_MODE_FLAG_SAFETY_ARMED = 128
MAV_SYS_STATUS_SENSOR_GPS = 1 << 5
MAV_SYS_STATUS_PREARM_CHECK = 1 << 28


@dataclass
class TelemetryAccumulator:
    armed: bool | None = None
    mode: str | None = None
    relative_altitude_m: float | None = None
    local_position_ned_m: tuple[float, float, float] | None = None
    velocity_ned_m_s: tuple[float, float, float] | None = None
    global_position_deg_m: tuple[float, float, float] | None = None
    attitude_rpy_rad: tuple[float, float, float] | None = None
    battery_remaining: float | None = None
    battery_voltage_v: float | None = None
    gps_fix_type: int | None = None
    satellites_visible: int | None = None
    gps_healthy: bool | None = None
    prearm_ok: bool | None = None
    ekf_flags: int | None = None
    landed_state: int | None = None
    home_position_deg_m: tuple[float, float, float] | None = None
    home_position_ned_m: tuple[float, float, float] | None = None
    last_status_text: str | None = None
    received_at: dict[str, float] = field(default_factory=dict)

    def touch(self, field_name: str, now: float) -> None:
        self.received_at[field_name] = now

    def fresh(self, field_name: str, now: float, maximum_age_s: float) -> bool:
        received = self.received_at.get(field_name)
        return received is not None and now - received <= maximum_age_s

    def snapshot(self, now: float, heartbeat_timeout_s: float) -> TelemetrySnapshot:
        heartbeat_at = self.received_at.get("heartbeat")
        return TelemetrySnapshot(
            observed_monotonic_s=now,
            fcu_link_ok=(
                heartbeat_at is not None and now - heartbeat_at <= heartbeat_timeout_s
            ),
            armed=self.armed,
            mode=self.mode,
            relative_altitude_m=self.relative_altitude_m,
            local_position_ned_m=self.local_position_ned_m,
            velocity_ned_m_s=self.velocity_ned_m_s,
            global_position_deg_m=self.global_position_deg_m,
            attitude_rpy_rad=self.attitude_rpy_rad,
            battery_remaining=self.battery_remaining,
            battery_voltage_v=self.battery_voltage_v,
            gps_fix_type=self.gps_fix_type,
            satellites_visible=self.satellites_visible,
            gps_healthy=self.gps_healthy,
            prearm_ok=self.prearm_ok,
            ekf_flags=self.ekf_flags,
            landed_state=self.landed_state,
            home_position_deg_m=self.home_position_deg_m,
            home_position_ned_m=self.home_position_ned_m,
            last_status_text=self.last_status_text,
            last_heartbeat_monotonic_s=heartbeat_at,
            field_ages_s={key: max(0.0, now - value) for key, value in self.received_at.items()},
        )

    def reduce(self, message: MavlinkEnvelope, now: float) -> None:
        fields = message.fields
        if message.name == "HEARTBEAT":
            self.armed = bool(int(fields.get("base_mode", 0)) & MAV_MODE_FLAG_SAFETY_ARMED)
            mode = str(fields.get("mode_name", fields.get("custom_mode", "UNKNOWN")))
            self.mode = mode.upper()
            self.touch("heartbeat", now)
        elif message.name == "GLOBAL_POSITION_INT":
            self.global_position_deg_m = (
                float(fields.get("lat", 0)) / 1e7,
                float(fields.get("lon", 0)) / 1e7,
                float(fields.get("alt", 0)) / 1000.0,
            )
            self.relative_altitude_m = float(fields.get("relative_alt", 0)) / 1000.0
            self.velocity_ned_m_s = (
                float(fields.get("vx", 0)) / 100.0,
                float(fields.get("vy", 0)) / 100.0,
                float(fields.get("vz", 0)) / 100.0,
            )
            self.touch("global_position", now)
            self.touch("velocity", now)
        elif message.name == "LOCAL_POSITION_NED":
            self.local_position_ned_m = tuple(
                float(fields.get(axis, 0.0)) for axis in ("x", "y", "z")
            )
            self.velocity_ned_m_s = tuple(
                float(fields.get(axis, 0.0)) for axis in ("vx", "vy", "vz")
            )
            self.touch("local_position", now)
            self.touch("velocity", now)
        elif message.name == "ATTITUDE":
            self.attitude_rpy_rad = tuple(
                float(fields.get(axis, 0.0)) for axis in ("roll", "pitch", "yaw")
            )
            self.touch("attitude", now)
        elif message.name == "ATTITUDE_QUATERNION":
            self.attitude_rpy_rad = quaternion_to_rpy(
                float(fields.get("q1", 1.0)),
                float(fields.get("q2", 0.0)),
                float(fields.get("q3", 0.0)),
                float(fields.get("q4", 0.0)),
            )
            self.touch("attitude", now)
        elif message.name == "SYS_STATUS":
            remaining = int(fields.get("battery_remaining", -1))
            voltage_mv = int(fields.get("voltage_battery", 0xFFFF))
            self.battery_remaining = remaining / 100.0 if remaining >= 0 else None
            self.battery_voltage_v = (
                voltage_mv / 1000.0 if voltage_mv not in {0, 0xFFFF} else None
            )
            sensors_present = int(fields.get("onboard_control_sensors_present", 0))
            sensors_enabled = int(fields.get("onboard_control_sensors_enabled", 0))
            sensors_healthy = int(fields.get("onboard_control_sensors_health", 0))
            if (
                sensors_present & MAV_SYS_STATUS_SENSOR_GPS
                and sensors_enabled & MAV_SYS_STATUS_SENSOR_GPS
            ):
                self.gps_healthy = bool(sensors_healthy & MAV_SYS_STATUS_SENSOR_GPS)
                self.touch("gps_health", now)
            if (
                sensors_present & MAV_SYS_STATUS_PREARM_CHECK
                and sensors_enabled & MAV_SYS_STATUS_PREARM_CHECK
            ):
                self.prearm_ok = bool(
                    sensors_healthy & MAV_SYS_STATUS_PREARM_CHECK
                )
                self.touch("prearm", now)
            self.touch("battery", now)
        elif message.name == "BATTERY_STATUS":
            remaining = int(fields.get("battery_remaining", -1))
            self.battery_remaining = remaining / 100.0 if remaining >= 0 else None
            self.touch("battery", now)
        elif message.name == "GPS_RAW_INT":
            self.gps_fix_type = int(fields.get("fix_type", 0))
            self.satellites_visible = int(fields.get("satellites_visible", 0))
            self.touch("gps", now)
        elif message.name == "EKF_STATUS_REPORT":
            self.ekf_flags = int(fields.get("flags", 0))
            self.touch("ekf", now)
        elif message.name == "EXTENDED_SYS_STATE":
            self.landed_state = int(fields.get("landed_state", 0))
            self.touch("landed_state", now)
        elif message.name == "HOME_POSITION":
            self.home_position_deg_m = (
                float(fields.get("latitude", 0)) / 1e7,
                float(fields.get("longitude", 0)) / 1e7,
                float(fields.get("altitude", 0)) / 1000.0,
            )
            self.home_position_ned_m = tuple(
                float(fields.get(axis, 0.0)) for axis in ("x", "y", "z")
            )
            self.touch("home", now)
        elif message.name == "STATUSTEXT":
            text = fields.get("text", "")
            if isinstance(text, bytes):
                text = text.decode("utf-8", errors="replace")
            self.last_status_text = str(text).rstrip("\x00")
            self.touch("status_text", now)


def horizontal_global_distance_m(
    current: tuple[float, float, float],
    home: tuple[float, float, float],
) -> float:
    lat1, lon1 = math.radians(current[0]), math.radians(current[1])
    lat2, lon2 = math.radians(home[0]), math.radians(home[1])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    value = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(
        dlon / 2
    ) ** 2
    return 2.0 * 6_378_137.0 * math.asin(min(1.0, math.sqrt(value)))


def quaternion_to_rpy(
    w: float,
    x: float,
    y: float,
    z: float,
) -> tuple[float, float, float]:
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sin_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sin_pitch)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw

"""ArduPilot heartbeat validation and immutable FCU identity parsing."""

from __future__ import annotations

from .mavlink.models import FcuIdentity, MavlinkEnvelope

MAV_AUTOPILOT_ARDUPILOTMEGA = 3
MAV_TYPE_QUADROTOR = 2


def validate_heartbeat(
    heartbeat: MavlinkEnvelope,
    *,
    expected_vehicle_type: int,
) -> None:
    autopilot_type = int(heartbeat.fields.get("autopilot", -1))
    vehicle_type = int(heartbeat.fields.get("type", -1))
    if autopilot_type != MAV_AUTOPILOT_ARDUPILOTMEGA:
        raise ValueError(f"expected ArduPilot autopilot type 3, got {autopilot_type}")
    if vehicle_type != expected_vehicle_type:
        raise ValueError(f"expected MAV vehicle type {expected_vehicle_type}, got {vehicle_type}")


def identity_from_messages(
    heartbeat: MavlinkEnvelope,
    version: MavlinkEnvelope,
) -> FcuIdentity:
    raw = int(version.fields.get("flight_sw_version", 0))
    custom = version.fields.get("flight_custom_version", b"")
    if isinstance(custom, str):
        custom_bytes = custom.encode("utf-8", errors="replace")
    else:
        custom_bytes = bytes(custom)
    return FcuIdentity(
        system_id=heartbeat.source_system,
        component_id=heartbeat.source_component,
        autopilot_type=int(heartbeat.fields.get("autopilot", -1)),
        vehicle_type=int(heartbeat.fields.get("type", -1)),
        flight_sw_version_raw=raw,
        flight_sw_version=format_flight_version(raw),
        board_version=int(version.fields.get("board_version", 0)),
        vendor_id=int(version.fields.get("vendor_id", 0)),
        product_id=int(version.fields.get("product_id", 0)),
        uid=int(version.fields.get("uid", 0)),
        capabilities=int(version.fields.get("capabilities", 0)),
        custom_version_hex=custom_bytes.hex(),
    )


def format_flight_version(raw: int) -> str:
    return f"{(raw >> 24) & 0xFF}.{(raw >> 16) & 0xFF}.{(raw >> 8) & 0xFF}-{raw & 0xFF}"

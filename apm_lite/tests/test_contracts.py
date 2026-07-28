from datetime import datetime, timedelta, timezone
from uuid import uuid4

import pytest
from pydantic import ValidationError

from aeromind_apm_lite.common.contracts import (
    AckStatus,
    ContractError,
    ContractErrorCode,
    CoordinateFrame,
    IngressError,
    IngressErrorCode,
    IngressGuard,
    MissionAck,
    TrajectorySegment,
    VehicleCommand,
    VehicleCommandType,
    decode_message,
    encode_message,
    sign_message,
)
from aeromind_apm_lite.common.contracts.models import (
    TrajectoryPoint,
    Vector3,
    VehicleHealth,
    VehicleTelemetry,
)


def base_fields(*, sequence: int = 1):
    return {
        "vehicle_id": 1,
        "sequence": sequence,
        "session_id": uuid4(),
        "created_at_utc": datetime.now(timezone.utc),
    }


def test_vehicle_command_round_trip_is_typed_and_strict():
    command = VehicleCommand(
        **base_fields(),
        command=VehicleCommandType.TAKEOFF,
        target_altitude_m=2.0,
    )

    decoded = decode_message(encode_message(command))

    assert decoded == command
    assert isinstance(decoded, VehicleCommand)


def test_unknown_version_and_type_have_stable_error_codes():
    command = VehicleCommand(**base_fields(), command=VehicleCommandType.ARM)
    raw = command.model_dump(mode="json")
    raw["schema_version"] = "2.0"
    with pytest.raises(ContractError) as version_error:
        decode_message(raw)
    assert version_error.value.code == ContractErrorCode.UNSUPPORTED_VERSION

    raw["schema_version"] = "1.0"
    raw["message_type"] = "raw_motor_command"
    with pytest.raises(ContractError) as type_error:
        decode_message(raw)
    assert type_error.value.code == ContractErrorCode.UNSUPPORTED_MESSAGE_TYPE


def test_takeoff_requires_altitude_and_naive_time_is_rejected():
    with pytest.raises(ValidationError):
        VehicleCommand(**base_fields(), command=VehicleCommandType.TAKEOFF)

    fields = base_fields()
    fields["created_at_utc"] = datetime.now()
    with pytest.raises(ValidationError):
        VehicleCommand(**fields, command=VehicleCommandType.ARM)


def test_trajectory_requires_calibration_and_strictly_increasing_time():
    fields = base_fields()
    points = [
        TrajectoryPoint(time_from_start_ms=100, position_m=Vector3(x=0, y=0, z=2)),
        TrajectoryPoint(time_from_start_ms=100, position_m=Vector3(x=1, y=0, z=2)),
    ]
    with pytest.raises(ValidationError):
        TrajectorySegment(
            **fields,
            mission_id=uuid4(),
            frame=CoordinateFrame.MAP,
            frame_calibration_id="venue-v1",
            starts_at_utc=datetime.now(timezone.utc),
            points=points,
            max_speed_m_s=1.0,
            max_accel_m_s2=0.5,
        )


def test_completed_ack_requires_physical_telemetry_confirmation():
    fields = base_fields()
    with pytest.raises(ValidationError):
        MissionAck(
            **fields,
            acknowledged_message_id=uuid4(),
            status=AckStatus.COMPLETED,
        )


def test_ingress_uses_receiver_local_ttl_and_rejects_replay():
    session_id = uuid4()
    fields = base_fields(sequence=10)
    fields["session_id"] = session_id
    fields["created_at_utc"] = datetime.now(timezone.utc) - timedelta(days=1)
    command = VehicleCommand(
        **fields,
        command=VehicleCommandType.HOLD,
        ttl_ms=500,
    )
    guard = IngressGuard(
        expected_vehicle_id=1,
        expected_session_id=session_id,
        expected_calibration_id="venue-v1",
    )

    received = guard.receive(command, received_monotonic_s=100.0)
    received.ensure_fresh(100.49)
    with pytest.raises(IngressError) as expired:
        received.ensure_fresh(100.51)
    assert expired.value.code == IngressErrorCode.EXPIRED

    with pytest.raises(IngressError) as duplicate:
        guard.receive(command, received_monotonic_s=101.0)
    assert duplicate.value.code == IngressErrorCode.DUPLICATE_MESSAGE

    replay = command.model_copy(update={"message_id": uuid4(), "sequence": 9})
    with pytest.raises(IngressError) as replay_error:
        guard.receive(replay, received_monotonic_s=101.0)
    assert replay_error.value.code == IngressErrorCode.REPLAYED_SEQUENCE


def test_ingress_rejects_wrong_vehicle_session_and_calibration():
    session_id = uuid4()
    guard = IngressGuard(
        expected_vehicle_id=1,
        expected_session_id=session_id,
        expected_calibration_id="venue-v1",
    )
    base = base_fields()
    base["session_id"] = session_id

    wrong_vehicle = VehicleCommand(
        **{**base, "vehicle_id": 2}, command=VehicleCommandType.ARM
    )
    with pytest.raises(IngressError) as vehicle_error:
        guard.receive(wrong_vehicle)
    assert vehicle_error.value.code == IngressErrorCode.WRONG_VEHICLE

    wrong_session = VehicleCommand(
        **{**base, "session_id": uuid4()}, command=VehicleCommandType.ARM
    )
    with pytest.raises(IngressError) as session_error:
        guard.receive(wrong_session)
    assert session_error.value.code == IngressErrorCode.WRONG_SESSION

    points = [
        TrajectoryPoint(time_from_start_ms=0, position_m=Vector3(x=0, y=0, z=2)),
        TrajectoryPoint(time_from_start_ms=1000, position_m=Vector3(x=1, y=0, z=2)),
    ]
    wrong_calibration = TrajectorySegment(
        **base,
        mission_id=uuid4(),
        frame=CoordinateFrame.MAP,
        frame_calibration_id="venue-v2",
        starts_at_utc=datetime.now(timezone.utc),
        points=points,
        max_speed_m_s=1.0,
        max_accel_m_s2=0.5,
    )
    with pytest.raises(IngressError) as calibration_error:
        guard.receive(wrong_calibration)
    assert calibration_error.value.code == IngressErrorCode.WRONG_CALIBRATION


def test_ingress_authenticates_message_before_accepting_it():
    secret = b"vehicle-1-secret-that-is-at-least-32-bytes"
    session_id = uuid4()
    fields = base_fields()
    fields["session_id"] = session_id
    command = VehicleCommand(**fields, command=VehicleCommandType.HOLD)
    guard = IngressGuard(
        expected_vehicle_id=1,
        expected_session_id=session_id,
        expected_calibration_id="venue-v1",
        shared_secret=secret,
    )

    with pytest.raises(IngressError) as missing_signature:
        guard.receive(command)
    assert missing_signature.value.code == IngressErrorCode.AUTHENTICATION_FAILED

    signature = sign_message(command, secret)
    tampered = command.model_copy(update={"command": VehicleCommandType.RTL})
    with pytest.raises(IngressError) as tampered_signature:
        guard.receive(tampered, signature=signature)
    assert tampered_signature.value.code == IngressErrorCode.AUTHENTICATION_FAILED

    received = guard.receive(command, signature=signature, received_monotonic_s=10.0)
    assert received.message == command


def test_geometry_rejects_non_finite_values_and_missing_frame():
    with pytest.raises(ValidationError):
        Vector3(x=float("nan"), y=0.0, z=0.0)

    with pytest.raises(ValidationError):
        VehicleTelemetry(
            **base_fields(),
            observed_at_utc=datetime.now(timezone.utc),
            armed=False,
            mode="STABILIZE",
            position_m=Vector3(x=0.0, y=0.0, z=0.0),
            health=VehicleHealth(fcu_link_ok=True),
        )

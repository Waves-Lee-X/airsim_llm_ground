"""Versioned messages used by simulation and real vehicles."""

from .codec import ContractError, ContractErrorCode, decode_message, encode_message
from .auth import new_session_challenge, session_proof, sign_message, verify_message_signature
from .ingress import IngressError, IngressErrorCode, IngressGuard, ReceivedMessage
from .models import (
    SCHEMA_VERSION,
    AckStatus,
    CoordinateFrame,
    FormationCommand,
    FormationShape,
    Heartbeat,
    MissionAck,
    SemanticObservation,
    TrajectorySegment,
    VehicleCommand,
    VehicleCommandType,
    Vector3,
    VehicleTelemetry,
    WireMessage,
)

__all__ = [
    "SCHEMA_VERSION",
    "AckStatus",
    "ContractError",
    "ContractErrorCode",
    "CoordinateFrame",
    "FormationCommand",
    "FormationShape",
    "Heartbeat",
    "IngressError",
    "IngressErrorCode",
    "IngressGuard",
    "MissionAck",
    "ReceivedMessage",
    "SemanticObservation",
    "TrajectorySegment",
    "VehicleCommand",
    "VehicleCommandType",
    "Vector3",
    "VehicleTelemetry",
    "WireMessage",
    "decode_message",
    "encode_message",
    "new_session_challenge",
    "session_proof",
    "sign_message",
    "verify_message_signature",
]

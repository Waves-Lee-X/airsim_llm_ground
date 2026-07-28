"""Strict WebSocket framing and mutual challenge authentication."""

from __future__ import annotations

import hmac
import json
from typing import Any, Literal
from uuid import UUID

from pydantic import BaseModel, ConfigDict, Field, ValidationError

from aeromind_apm_lite.common.contracts import (
    SCHEMA_VERSION,
    WireMessage,
    decode_message,
    session_proof,
    sign_message,
)


class ProtocolError(ValueError):
    """The peer sent a malformed or contextually invalid protocol frame."""


class AuthenticationError(ProtocolError):
    """The peer couldn't prove possession of the per-vehicle secret."""


class _Frame(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)

    protocol_version: Literal["1.0"] = SCHEMA_VERSION
    frame_type: str
    vehicle_id: int = Field(ge=1, le=255)
    session_id: UUID


class ClientHello(_Frame):
    frame_type: Literal["client_hello"] = "client_hello"
    client_challenge: str = Field(min_length=32, max_length=128)


class ServerChallenge(_Frame):
    frame_type: Literal["server_challenge"] = "server_challenge"
    server_challenge: str = Field(min_length=32, max_length=128)
    server_proof: str = Field(pattern=r"^[0-9a-f]{64}$")


class ClientProof(_Frame):
    frame_type: Literal["client_proof"] = "client_proof"
    client_proof: str = Field(pattern=r"^[0-9a-f]{64}$")


class SessionAccepted(_Frame):
    frame_type: Literal["session_accepted"] = "session_accepted"


class SignedMessageFrame(_Frame):
    frame_type: Literal["signed_message"] = "signed_message"
    algorithm: Literal["hmac-sha256"] = "hmac-sha256"
    message: dict[str, Any]
    signature: str = Field(pattern=r"^[0-9a-f]{64}$")


HandshakeFrame = ClientHello | ServerChallenge | ClientProof | SessionAccepted

_HANDSHAKE_TYPES: dict[str, type[_Frame]] = {
    "client_hello": ClientHello,
    "server_challenge": ServerChallenge,
    "client_proof": ClientProof,
    "session_accepted": SessionAccepted,
}


def _json_object(payload: str | bytes) -> dict[str, Any]:
    try:
        if isinstance(payload, bytes):
            payload = payload.decode("utf-8")
        value = json.loads(payload)
    except (UnicodeDecodeError, json.JSONDecodeError, TypeError) as exc:
        raise ProtocolError(f"invalid JSON frame: {exc}") from exc
    if not isinstance(value, dict):
        raise ProtocolError("protocol frame must be a JSON object")
    return value


def decode_handshake_frame(payload: str | bytes) -> HandshakeFrame:
    data = _json_object(payload)
    frame_type = data.get("frame_type")
    model = _HANDSHAKE_TYPES.get(frame_type)
    if model is None:
        raise ProtocolError(f"unexpected handshake frame type {frame_type!r}")
    try:
        return model.model_validate(data)
    except ValidationError as exc:
        raise ProtocolError(f"invalid {frame_type} frame: {exc}") from exc


def encode_frame(frame: _Frame) -> str:
    return frame.model_dump_json()


def authentication_proof(
    *,
    role: Literal["client", "server"],
    challenge: str,
    vehicle_id: int,
    session_id: UUID,
    secret: bytes,
) -> str:
    """Bind a one-time challenge to a role, vehicle and connection session."""
    return session_proof(
        challenge=f"aeromind-apm-lite:{role}:{challenge}",
        vehicle_id=vehicle_id,
        session_id=session_id,
        secret=secret,
    )


def verify_authentication_proof(
    proof: str,
    *,
    role: Literal["client", "server"],
    challenge: str,
    vehicle_id: int,
    session_id: UUID,
    secret: bytes,
) -> bool:
    try:
        expected = authentication_proof(
            role=role,
            challenge=challenge,
            vehicle_id=vehicle_id,
            session_id=session_id,
            secret=secret,
        )
    except ValueError:
        return False
    return hmac.compare_digest(expected, proof)


def encode_signed_message(message: WireMessage, secret: bytes) -> str:
    frame = SignedMessageFrame(
        vehicle_id=message.vehicle_id,
        session_id=message.session_id,
        message=message.model_dump(mode="json"),
        signature=sign_message(message, secret),
    )
    return frame.model_dump_json()


def decode_signed_message(payload: str | bytes) -> tuple[WireMessage, str]:
    data = _json_object(payload)
    try:
        frame = SignedMessageFrame.model_validate(data)
    except ValidationError as exc:
        raise ProtocolError(f"invalid signed message frame: {exc}") from exc
    try:
        message = decode_message(frame.message)
    except ValueError as exc:
        raise ProtocolError(f"invalid wire message: {exc}") from exc
    if message.vehicle_id != frame.vehicle_id or message.session_id != frame.session_id:
        raise ProtocolError("signed frame identity does not match its wire message")
    return message, frame.signature

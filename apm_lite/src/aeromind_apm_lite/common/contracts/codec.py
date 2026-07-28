"""JSON encoding and explicit contract errors for v1 wire messages."""

from __future__ import annotations

import json
from enum import Enum
from typing import Any

from pydantic import TypeAdapter, ValidationError

from .models import SCHEMA_VERSION, WireMessage

_WIRE_ADAPTER = TypeAdapter(WireMessage)
_MESSAGE_TYPES = {
    "vehicle_command",
    "trajectory_segment",
    "formation_command",
    "vehicle_telemetry",
    "semantic_observation",
    "mission_ack",
    "heartbeat",
}


class ContractErrorCode(str, Enum):
    INVALID_JSON = "invalid_json"
    UNSUPPORTED_VERSION = "unsupported_version"
    UNSUPPORTED_MESSAGE_TYPE = "unsupported_message_type"
    INVALID_PAYLOAD = "invalid_payload"


class ContractError(ValueError):
    def __init__(self, code: ContractErrorCode, detail: str):
        super().__init__(detail)
        self.code = code
        self.detail = detail


def encode_message(message: WireMessage) -> str:
    return message.model_dump_json()


def decode_message(payload: str | bytes | bytearray | dict[str, Any]) -> WireMessage:
    if isinstance(payload, dict):
        data = payload
    else:
        try:
            if isinstance(payload, (bytes, bytearray)):
                payload = payload.decode("utf-8")
            data = json.loads(payload)
        except (UnicodeDecodeError, json.JSONDecodeError, TypeError) as exc:
            raise ContractError(ContractErrorCode.INVALID_JSON, str(exc)) from exc

    if not isinstance(data, dict):
        raise ContractError(ContractErrorCode.INVALID_PAYLOAD, "message must be a JSON object")

    version = data.get("schema_version")
    if version != SCHEMA_VERSION:
        raise ContractError(
            ContractErrorCode.UNSUPPORTED_VERSION,
            f"unsupported schema_version {version!r}; expected {SCHEMA_VERSION!r}",
        )

    message_type = data.get("message_type")
    if message_type not in _MESSAGE_TYPES:
        raise ContractError(
            ContractErrorCode.UNSUPPORTED_MESSAGE_TYPE,
            f"unsupported message_type {message_type!r}",
        )

    try:
        return _WIRE_ADAPTER.validate_python(data)
    except ValidationError as exc:
        raise ContractError(ContractErrorCode.INVALID_PAYLOAD, str(exc)) from exc


def wire_message_json_schema() -> dict[str, Any]:
    return _WIRE_ADAPTER.json_schema()

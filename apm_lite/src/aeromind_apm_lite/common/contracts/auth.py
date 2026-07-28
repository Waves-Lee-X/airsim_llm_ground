"""HMAC helpers for per-vehicle message and session authentication."""

from __future__ import annotations

import hashlib
import hmac
import json
import secrets
from uuid import UUID

from .models import MessageBase


def _require_secret(secret: bytes) -> None:
    if len(secret) < 32:
        raise ValueError("vehicle shared secret must contain at least 32 bytes")


def canonical_message_bytes(message: MessageBase) -> bytes:
    payload = message.model_dump(mode="json")
    return json.dumps(
        payload,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")


def sign_message(message: MessageBase, secret: bytes) -> str:
    _require_secret(secret)
    return hmac.new(secret, canonical_message_bytes(message), hashlib.sha256).hexdigest()


def verify_message_signature(message: MessageBase, signature: str, secret: bytes) -> bool:
    try:
        expected = sign_message(message, secret)
    except ValueError:
        return False
    return hmac.compare_digest(expected, signature)


def new_session_challenge() -> str:
    return secrets.token_urlsafe(32)


def session_proof(
    *, challenge: str, vehicle_id: int, session_id: UUID, secret: bytes
) -> str:
    _require_secret(secret)
    payload = f"{challenge}:{vehicle_id}:{session_id}".encode("utf-8")
    return hmac.new(secret, payload, hashlib.sha256).hexdigest()

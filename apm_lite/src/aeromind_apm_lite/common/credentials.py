"""Load per-vehicle secrets without placing credentials in repository files."""

from __future__ import annotations

import base64
import binascii
import os


class CredentialError(ValueError):
    pass


def decode_shared_secret(value: str) -> bytes:
    if value.startswith("hex:"):
        try:
            secret = bytes.fromhex(value[4:])
        except ValueError as exc:
            raise CredentialError("shared secret has invalid hex encoding") from exc
    elif value.startswith("base64:"):
        try:
            secret = base64.b64decode(value[7:], validate=True)
        except (ValueError, binascii.Error) as exc:
            raise CredentialError("shared secret has invalid base64 encoding") from exc
    else:
        secret = value.encode("utf-8")
    if len(secret) < 32:
        raise CredentialError("shared secret must contain at least 32 bytes")
    if len(secret) > 4096:
        raise CredentialError("shared secret must not exceed 4096 bytes")
    return secret


def load_shared_secret(environment_name: str) -> bytes:
    if not environment_name:
        raise CredentialError("credential environment name must not be empty")
    value = os.environ.get(environment_name)
    if value is None:
        raise CredentialError(
            f"required credential environment variable {environment_name} is not set"
        )
    return decode_shared_secret(value)

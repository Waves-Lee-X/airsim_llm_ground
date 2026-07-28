"""Authenticated, resource-bounded WebSocket communication primitives."""

from .protocol import (
    AuthenticationError,
    ClientHello,
    ClientProof,
    ProtocolError,
    ServerChallenge,
    SessionAccepted,
    SignedMessageFrame,
    authentication_proof,
    decode_handshake_frame,
    decode_signed_message,
    encode_frame,
    encode_signed_message,
    verify_authentication_proof,
)
from .queue import BoundedLatestQueue, QueueClosed

__all__ = [
    "AuthenticationError",
    "BoundedLatestQueue",
    "ClientHello",
    "ClientProof",
    "ProtocolError",
    "QueueClosed",
    "ServerChallenge",
    "SessionAccepted",
    "SignedMessageFrame",
    "authentication_proof",
    "decode_handshake_frame",
    "decode_signed_message",
    "encode_frame",
    "encode_signed_message",
    "verify_authentication_proof",
]

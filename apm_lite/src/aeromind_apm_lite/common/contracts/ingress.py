"""Receiver-local TTL and anti-replay checks for actionable messages."""

from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass
from enum import Enum
from typing import Generic, TypeVar
from uuid import UUID

from .auth import verify_message_signature
from .models import CoordinateFrame, MessageBase

MessageT = TypeVar("MessageT", bound=MessageBase)


class IngressErrorCode(str, Enum):
    AUTHENTICATION_FAILED = "authentication_failed"
    WRONG_VEHICLE = "wrong_vehicle"
    WRONG_SESSION = "wrong_session"
    WRONG_CALIBRATION = "wrong_calibration"
    DUPLICATE_MESSAGE = "duplicate_message"
    REPLAYED_SEQUENCE = "replayed_sequence"
    EXPIRED = "expired"
    INVALID_RECEIPT_TIME = "invalid_receipt_time"


class IngressError(ValueError):
    def __init__(self, code: IngressErrorCode, detail: str):
        super().__init__(detail)
        self.code = code
        self.detail = detail


@dataclass(frozen=True)
class ReceivedMessage(Generic[MessageT]):
    message: MessageT
    received_monotonic_s: float

    def age_ms(self, now_monotonic_s: float | None = None) -> float:
        now = time.monotonic() if now_monotonic_s is None else now_monotonic_s
        if now < self.received_monotonic_s:
            raise IngressError(
                IngressErrorCode.INVALID_RECEIPT_TIME,
                "receiver monotonic clock moved backwards",
            )
        return (now - self.received_monotonic_s) * 1_000.0

    def ensure_fresh(self, now_monotonic_s: float | None = None) -> None:
        age_ms = self.age_ms(now_monotonic_s)
        if age_ms > self.message.ttl_ms:
            raise IngressError(
                IngressErrorCode.EXPIRED,
                f"message age {age_ms:.1f} ms exceeds ttl {self.message.ttl_ms} ms",
            )


class IngressGuard:
    """Validates one authenticated vehicle session and rejects replayed commands."""

    def __init__(
        self,
        *,
        expected_vehicle_id: int,
        expected_session_id: UUID,
        expected_calibration_id: str | None,
        shared_secret: bytes | None = None,
        replay_window_size: int = 2_048,
    ) -> None:
        if not 1 <= expected_vehicle_id <= 255:
            raise ValueError("expected_vehicle_id must be in [1, 255]")
        if replay_window_size < 1:
            raise ValueError("replay_window_size must be positive")
        self.expected_vehicle_id = expected_vehicle_id
        self.expected_session_id = expected_session_id
        self.expected_calibration_id = expected_calibration_id
        if shared_secret is not None and len(shared_secret) < 32:
            raise ValueError("shared_secret must contain at least 32 bytes")
        self._shared_secret = shared_secret
        self._highest_sequence = -1
        self._message_ids: set[UUID] = set()
        self._message_order: deque[UUID] = deque()
        self._replay_window_size = replay_window_size

    def receive(
        self,
        message: MessageT,
        *,
        signature: str | None = None,
        received_monotonic_s: float | None = None,
    ) -> ReceivedMessage[MessageT]:
        if self._shared_secret is not None and (
            signature is None
            or not verify_message_signature(message, signature, self._shared_secret)
        ):
            raise IngressError(
                IngressErrorCode.AUTHENTICATION_FAILED,
                "message signature is missing or invalid",
            )
        if message.vehicle_id != self.expected_vehicle_id:
            raise IngressError(
                IngressErrorCode.WRONG_VEHICLE,
                f"message targets vehicle {message.vehicle_id}, expected {self.expected_vehicle_id}",
            )
        if message.session_id != self.expected_session_id:
            raise IngressError(IngressErrorCode.WRONG_SESSION, "message session is not active")
        if (
            message.frame != CoordinateFrame.NONE
            and message.frame_calibration_id != self.expected_calibration_id
        ):
            raise IngressError(
                IngressErrorCode.WRONG_CALIBRATION,
                "message frame calibration does not match the active vehicle calibration",
            )
        if message.message_id in self._message_ids:
            raise IngressError(IngressErrorCode.DUPLICATE_MESSAGE, "message_id was already accepted")
        if message.sequence <= self._highest_sequence:
            raise IngressError(
                IngressErrorCode.REPLAYED_SEQUENCE,
                f"sequence {message.sequence} is not newer than {self._highest_sequence}",
            )

        self._highest_sequence = message.sequence
        self._remember(message.message_id)
        received_at = time.monotonic() if received_monotonic_s is None else received_monotonic_s
        return ReceivedMessage(message=message, received_monotonic_s=received_at)

    def _remember(self, message_id: UUID) -> None:
        self._message_ids.add(message_id)
        self._message_order.append(message_id)
        while len(self._message_order) > self._replay_window_size:
            evicted = self._message_order.popleft()
            self._message_ids.discard(evicted)

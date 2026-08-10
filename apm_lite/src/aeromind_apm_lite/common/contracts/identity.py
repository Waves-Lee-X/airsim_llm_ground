"""Signed, versioned identity documents for vehicles and their embodied agents."""

from __future__ import annotations

import hashlib
import hmac
import json
from datetime import datetime, timedelta, timezone
from enum import Enum
from typing import Literal

from pydantic import Field, field_validator, model_validator

from .models import CoordinateFrame, StrictModel
from .twin import VehicleProfile


IDENTITY_SCHEMA_VERSION = "1.0"


def utc_now() -> datetime:
    return datetime.now(timezone.utc)


def _as_utc(value: datetime) -> datetime:
    if value.tzinfo is None or value.utcoffset() is None:
        raise ValueError("timestamp must include a timezone")
    return value.astimezone(timezone.utc)


def _canonical_hash(payload: object) -> str:
    encoded = json.dumps(
        payload,
        ensure_ascii=True,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _require_secret(secret: bytes) -> None:
    if len(secret) < 32:
        raise ValueError("identity signing secret must contain at least 32 bytes")


class IdentityModel(StrictModel):
    model_config = StrictModel.model_config
    schema_version: Literal["1.0"] = IDENTITY_SCHEMA_VERSION


class AutonomyLevel(str, Enum):
    A0 = "A0"
    A1 = "A1"
    A2 = "A2"
    A3 = "A3"
    A4 = "A4"


class IdentityMetadata(IdentityModel):
    vehicle_id: int = Field(ge=1, le=65_535)
    serial_number: str = Field(
        min_length=1,
        max_length=128,
        pattern=r"^[A-Za-z0-9][A-Za-z0-9._-]*$",
    )
    display_name: str = Field(min_length=1, max_length=64)
    platform_type: str = Field(
        min_length=1,
        max_length=64,
        pattern=r"^[a-z][a-z0-9_-]*$",
    )
    vendor: str = Field(min_length=1, max_length=128)
    firmware_version: str = Field(min_length=1, max_length=128)
    document_version: str = Field(
        default="1",
        min_length=1,
        max_length=32,
        pattern=r"^[0-9]+(?:\.[0-9]+)*$",
    )
    registered_at_utc: datetime = Field(default_factory=utc_now)

    _registered_at_is_utc = field_validator("registered_at_utc")(_as_utc)


class CapabilityProfile(VehicleProfile):
    """Static profile extended with machine-readable operating limits."""

    coordinate_frames: tuple[CoordinateFrame, ...] = (CoordinateFrame.MAP,)
    limits: dict[str, float] = Field(default_factory=dict)


class EmbodiedAgentProfile(IdentityModel):
    model_id: str = Field(min_length=1, max_length=128)
    model_version: str = Field(min_length=1, max_length=128)
    input_modalities: tuple[str, ...] = Field(min_length=1)
    skill_ids: tuple[str, ...] = Field(min_length=1)
    autonomy_level: AutonomyLevel = AutonomyLevel.A1
    decision_rate_hz: float = Field(gt=0.0, le=10.0)
    compute_budget_mflops: float = Field(gt=0.0, le=1_000_000.0)
    fallback_policy: str = Field(min_length=1, max_length=64)

    @field_validator("input_modalities", "skill_ids")
    @classmethod
    def catalog_entries_are_unique(cls, values: tuple[str, ...]) -> tuple[str, ...]:
        normalized = tuple(sorted({value.strip().lower() for value in values}))
        if any(not value for value in normalized):
            raise ValueError("catalog entries must not be empty")
        return normalized


class AuthorizationProfile(IdentityModel):
    allowed_commands: tuple[str, ...] = Field(min_length=1)
    confirmation_required: tuple[str, ...] = ()
    frame_calibration_id: str = Field(min_length=1, max_length=128)
    max_altitude_m: float = Field(gt=0.0, le=500.0)
    max_distance_m: float = Field(gt=0.0, le=100_000.0)
    geofence_id: str | None = Field(default=None, max_length=128)

    @model_validator(mode="after")
    def confirmation_is_subset_of_allowed(self) -> "AuthorizationProfile":
        allowed = set(self.allowed_commands)
        if not set(self.confirmation_required).issubset(allowed):
            raise ValueError("confirmation_required must be a subset of allowed_commands")
        return self


class IdentityAttestation(IdentityModel):
    key_id: str = Field(min_length=1, max_length=128)
    algorithm: Literal["hmac-sha256"] = "hmac-sha256"
    document_hash: str = Field(pattern=r"^[0-9a-f]{64}$")
    signature: str = Field(pattern=r"^[0-9a-f]{64}$")
    issued_at_utc: datetime
    expires_at_utc: datetime
    revoked: bool = False
    revocation_reason: str | None = Field(default=None, max_length=256)

    @field_validator("issued_at_utc", "expires_at_utc")
    @classmethod
    def attestation_timestamps_are_utc(cls, value: datetime) -> datetime:
        return _as_utc(value)

    @model_validator(mode="after")
    def expiry_is_after_issue(self) -> "IdentityAttestation":
        if self.expires_at_utc <= self.issued_at_utc:
            raise ValueError("identity document expiry must be after issue time")
        if self.revoked and not self.revocation_reason:
            raise ValueError("revoked identity document requires revocation_reason")
        return self


class VehicleIdentityDocument(IdentityModel):
    """A signed digital passport for one vehicle and its onboard agent."""

    document_id: str = Field(
        min_length=1,
        max_length=128,
        pattern=r"^[A-Za-z0-9][A-Za-z0-9._-]*$",
    )
    identity: IdentityMetadata
    capabilities: CapabilityProfile
    embodied_agent: EmbodiedAgentProfile
    authorization: AuthorizationProfile
    attestation: IdentityAttestation

    @model_validator(mode="after")
    def identity_fields_match(self) -> "VehicleIdentityDocument":
        if self.identity.vehicle_id != self.capabilities.vehicle_id:
            raise ValueError("identity and capabilities vehicle_id values do not match")
        if self.identity.platform_type != self.capabilities.platform_type:
            raise ValueError("identity and capabilities platform_type values do not match")
        if not set(self.embodied_agent.skill_ids).issubset(set(self.capabilities.capabilities)):
            raise ValueError("embodied agent skill_ids must be declared capabilities")
        return self

    def _signed_payload(self) -> dict[str, object]:
        payload = self.model_dump(mode="json")
        attestation = payload["attestation"]
        if isinstance(attestation, dict):
            attestation.pop("document_hash", None)
            attestation.pop("signature", None)
        return payload

    @property
    def document_hash(self) -> str:
        return _canonical_hash(self._signed_payload())

    def verify_integrity(self) -> bool:
        return self.attestation.document_hash == self.document_hash

    def is_time_valid(self, now: datetime | None = None) -> bool:
        current = _as_utc(now or utc_now())
        return self.attestation.issued_at_utc <= current < self.attestation.expires_at_utc

    def verify_signature(self, secret: bytes) -> bool:
        try:
            _require_secret(secret)
        except ValueError:
            return False
        if not self.verify_integrity():
            return False
        expected = hmac.new(
            secret,
            self.attestation.document_hash.encode("ascii"),
            hashlib.sha256,
        ).hexdigest()
        return hmac.compare_digest(expected, self.attestation.signature)

    def verify(self, secret: bytes, *, now: datetime | None = None) -> bool:
        return (
            self.verify_signature(secret)
            and not self.attestation.revoked
            and self.is_time_valid(now)
        )

    def to_vehicle_profile(self) -> VehicleProfile:
        payload = self.capabilities.model_dump(mode="python")
        payload.pop("schema_version", None)
        payload.pop("coordinate_frames", None)
        payload.pop("limits", None)
        payload["display_name"] = self.identity.display_name
        return VehicleProfile.model_validate(payload)

    @classmethod
    def create(
        cls,
        *,
        document_id: str,
        identity: IdentityMetadata,
        capabilities: CapabilityProfile,
        embodied_agent: EmbodiedAgentProfile,
        authorization: AuthorizationProfile,
        key_id: str,
        secret: bytes,
        issued_at_utc: datetime,
        expires_at_utc: datetime | None = None,
    ) -> "VehicleIdentityDocument":
        _require_secret(secret)
        issued = _as_utc(issued_at_utc)
        expires = _as_utc(
            expires_at_utc or issued + timedelta(days=365)
        )
        unsigned_attestation = IdentityAttestation(
            key_id=key_id,
            issued_at_utc=issued,
            expires_at_utc=expires,
            document_hash="0" * 64,
            signature="0" * 64,
        )
        unsigned = cls(
            document_id=document_id,
            identity=identity,
            capabilities=capabilities,
            embodied_agent=embodied_agent,
            authorization=authorization,
            attestation=unsigned_attestation,
        )
        document_hash = unsigned.document_hash
        signature = hmac.new(
            secret,
            document_hash.encode("ascii"),
            hashlib.sha256,
        ).hexdigest()
        attestation = unsigned_attestation.model_copy(
            update={"document_hash": document_hash, "signature": signature}
        )
        return unsigned.model_copy(update={"attestation": attestation})

    def revoke(self, *, secret: bytes, reason: str) -> "VehicleIdentityDocument":
        _require_secret(secret)
        if not reason.strip():
            raise ValueError("revocation reason must not be empty")
        attestation = self.attestation.model_copy(
            update={"revoked": True, "revocation_reason": reason.strip()}
        )
        candidate = self.model_copy(update={"attestation": attestation})
        document_hash = candidate.document_hash
        signature = hmac.new(
            secret,
            document_hash.encode("ascii"),
            hashlib.sha256,
        ).hexdigest()
        return candidate.model_copy(
            update={
                "attestation": attestation.model_copy(
                    update={"document_hash": document_hash, "signature": signature}
                )
            }
        )


__all__ = [
    "IDENTITY_SCHEMA_VERSION",
    "AuthorizationProfile",
    "AutonomyLevel",
    "CapabilityProfile",
    "EmbodiedAgentProfile",
    "IdentityAttestation",
    "IdentityMetadata",
    "VehicleIdentityDocument",
]

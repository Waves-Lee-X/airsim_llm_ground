from datetime import datetime, timedelta, timezone

import pytest
from pydantic import ValidationError

from aeromind_apm_lite.common.contracts import (
    AuthorizationProfile,
    AutonomyLevel,
    CapabilityProfile,
    CoordinateFrame,
    EmbodiedAgentProfile,
    IdentityMetadata,
    VehicleIdentityDocument,
)
from aeromind_apm_lite.common.identity import RegistryError, VehicleRegistry


NOW = datetime(2026, 8, 10, 8, 0, tzinfo=timezone.utc)
SECRET = b"identity-signing-secret-for-test-32-bytes"


def make_document(*, vehicle_id=3):
    capabilities = CapabilityProfile(
        vehicle_id=vehicle_id,
        display_name=f"UAV{vehicle_id}",
        platform_type="multirotor",
        sensors=("rgb", "gps", "imu"),
        payloads=("camera",),
        max_endurance_min=24.0,
        max_speed_m_s=12.0,
        capabilities=("capture", "goto", "search"),
        whitelist_level=1,
        coordinate_frames=(CoordinateFrame.MAP,),
        limits={"reserve_energy": 0.15},
    )
    return VehicleIdentityDocument.create(
        document_id=f"uav-{vehicle_id}-identity-v1",
        identity=IdentityMetadata(
            vehicle_id=vehicle_id,
            serial_number=f"AERO-{vehicle_id:03d}",
            display_name=f"UAV{vehicle_id}",
            platform_type="multirotor",
            vendor="AeroMind",
            firmware_version="arducopter-4.7",
            registered_at_utc=NOW,
        ),
        capabilities=capabilities,
        embodied_agent=EmbodiedAgentProfile(
            model_id="edge-agent",
            model_version="0.1.0",
            input_modalities=("telemetry", "rgb"),
            skill_ids=("capture", "goto", "search"),
            autonomy_level=AutonomyLevel.A2,
            decision_rate_hz=1.0,
            compute_budget_mflops=20_000,
            fallback_policy="hold_then_rtl",
        ),
        authorization=AuthorizationProfile(
            allowed_commands=("hold", "goto", "rtl"),
            confirmation_required=("rtl",),
            frame_calibration_id="test-field-v1",
            max_altitude_m=30.0,
            max_distance_m=200.0,
            geofence_id="test-geofence-v1",
        ),
        key_id="ground-identity-key-v1",
        secret=SECRET,
        issued_at_utc=NOW,
        expires_at_utc=NOW + timedelta(days=30),
    )


def test_identity_document_is_signed_and_tamper_evident():
    document = make_document()

    assert document.verify(SECRET, now=NOW)
    assert document.verify_integrity()
    assert document.to_vehicle_profile().vehicle_id == 3

    tampered = document.model_copy(
        update={
            "capabilities": document.capabilities.model_copy(
                update={"max_speed_m_s": 20.0}
            )
        }
    )
    assert not tampered.verify(SECRET, now=NOW)


def test_identity_document_rejects_unknown_embodied_skill_and_expired_docs():
    payload = make_document().model_dump(mode="python")
    payload["embodied_agent"] = EmbodiedAgentProfile(
        model_id="edge-agent",
        model_version="0.1.0",
        input_modalities=("telemetry",),
        skill_ids=("unknown_skill",),
        autonomy_level=AutonomyLevel.A1,
        decision_rate_hz=1.0,
        compute_budget_mflops=10_000,
        fallback_policy="hold",
    )
    with pytest.raises(ValidationError, match="declared capabilities"):
        VehicleIdentityDocument.model_validate(payload)

    expired = make_document().model_copy(
        update={
            "attestation": make_document().attestation.model_copy(
                update={
                    "issued_at_utc": NOW - timedelta(days=30),
                    "expires_at_utc": NOW - timedelta(days=1),
                }
            )
        }
    )
    assert not expired.is_time_valid(NOW)


def test_registry_rejects_bad_identity_and_filters_revoked_document():
    document = make_document()
    registry = VehicleRegistry()
    registry.register_document(document, verification_secret=SECRET)
    assert registry.get_document(3).document_id == document.document_id
    assert registry.query(required_capabilities={"search"}, at_utc=NOW)

    revoked = document.revoke(secret=SECRET, reason="maintenance")
    revoked_registry = VehicleRegistry()
    revoked_registry.register_document(revoked, verification_secret=SECRET)
    assert revoked_registry.query(required_capabilities={"search"}, at_utc=NOW) == ()

    broken = document.model_copy(
        update={"attestation": document.attestation.model_copy(update={"signature": "0" * 64})}
    )
    with pytest.raises(RegistryError, match="signature"):
        registry.register_document(broken, verification_secret=SECRET, replace=True)


def test_identity_registry_persists_documents_and_requires_verification(tmp_path):
    path = tmp_path / "registry.json"
    registry = VehicleRegistry()
    registry.register_document(make_document(), verification_secret=SECRET)
    registry.save(path)

    loaded = VehicleRegistry.load(path, verification_secret=SECRET)
    assert loaded.get_document(3).verify(SECRET, now=NOW)
    with pytest.raises(RegistryError, match="signature"):
        VehicleRegistry.load(path, verification_secret=b"wrong-secret-that-is-long-enough-32")

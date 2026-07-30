import pytest

from aeromind_apm_lite.common.credentials import (
    CredentialError,
    decode_shared_secret,
    load_shared_secret,
)


def test_shared_secret_supports_text_hex_and_base64(monkeypatch):
    raw = b"0123456789abcdef0123456789abcdef"
    assert decode_shared_secret(raw.decode()) == raw
    assert decode_shared_secret("hex:" + raw.hex()) == raw
    assert (
        decode_shared_secret("base64:MDEyMzQ1Njc4OWFiY2RlZjAxMjM0NTY3ODlhYmNkZWY=")
        == raw
    )
    monkeypatch.setenv("AEROMIND_UAV3_TOKEN", raw.decode())
    assert load_shared_secret("AEROMIND_UAV3_TOKEN") == raw


def test_shared_secret_fails_closed(monkeypatch):
    with pytest.raises(CredentialError, match="at least 32"):
        decode_shared_secret("too-short")
    with pytest.raises(CredentialError, match="invalid hex"):
        decode_shared_secret("hex:not-hex")
    monkeypatch.delenv("AEROMIND_UAV3_TOKEN", raising=False)
    with pytest.raises(CredentialError, match="is not set"):
        load_shared_secret("AEROMIND_UAV3_TOKEN")

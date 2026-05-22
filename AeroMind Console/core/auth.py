from __future__ import annotations

import time
import hashlib
from dataclasses import dataclass, field
from typing import Any, Callable
from http import HTTPStatus


@dataclass(frozen=True)
class AuthConfig:
    enabled: bool = False
    api_key: str = ""
    rate_limit_requests: int = 30
    rate_limit_window_s: int = 60
    allowed_ips: tuple[str, ...] = ("127.0.0.1", "::1")


@dataclass
class RateLimitEntry:
    count: int = 0
    window_start: float = 0.0


class AuthManager:
    def __init__(self, config: AuthConfig | None = None) -> None:
        self.config = config or AuthConfig()
        self._rate_limits: dict[str, RateLimitEntry] = {}
        self._api_key_hash = self._hash_key(self.config.api_key) if self.config.api_key else ""

    @staticmethod
    def _hash_key(key: str) -> str:
        return hashlib.sha256(key.encode()).hexdigest()

    def _check_ip(self, client_ip: str) -> bool:
        if not self.config.allowed_ips:
            return True
        return client_ip in self.config.allowed_ips

    def _check_api_key(self, provided_key: str) -> bool:
        if not self.config.api_key:
            return True
        if not provided_key:
            return False
        return self._hash_key(provided_key) == self._api_key_hash

    def _check_rate_limit(self, client_ip: str) -> tuple[bool, dict[str, int]]:
        if self.config.rate_limit_requests <= 0:
            return True, {}
        
        now = time.time()
        entry = self._rate_limits.get(client_ip, RateLimitEntry())
        
        if now - entry.window_start > self.config.rate_limit_window_s:
            entry = RateLimitEntry(count=1, window_start=now)
        else:
            entry.count += 1
        
        self._rate_limits[client_ip] = entry
        
        remaining = max(0, self.config.rate_limit_requests - entry.count)
        reset_in = max(0, int(self.config.rate_limit_window_s - (now - entry.window_start)))
        
        if entry.count > self.config.rate_limit_requests:
            return False, {"remaining": remaining, "reset_in": reset_in}
        
        return True, {"remaining": remaining, "reset_in": reset_in}

    def authenticate(self, handler: Callable[..., dict[str, Any]], client_ip: str, headers: dict[str, str]) -> tuple[int, dict[str, Any], dict[str, str]]:
        if not self.config.enabled:
            try:
                return HTTPStatus.OK, handler(), {}
            except Exception as exc:
                return HTTPStatus.INTERNAL_SERVER_ERROR, {"detail": str(exc)}, {}

        if not self._check_ip(client_ip):
            return HTTPStatus.FORBIDDEN, {"detail": "IP not allowed"}, {}

        api_key = headers.get("X-API-Key", "").strip()
        if not self._check_api_key(api_key):
            return HTTPStatus.UNAUTHORIZED, {"detail": "Invalid API key"}, {}

        allowed, rate_info = self._check_rate_limit(client_ip)
        if not allowed:
            return (
                HTTPStatus.TOO_MANY_REQUESTS,
                {"detail": "Rate limit exceeded", "rate_limit": rate_info},
                {"Retry-After": str(rate_info.get("reset_in", 60))},
            )

        try:
            return HTTPStatus.OK, handler(), {"X-RateLimit-Remaining": str(rate_info.get("remaining", 0))}
        except Exception as exc:
            return HTTPStatus.INTERNAL_SERVER_ERROR, {"detail": str(exc)}, {}

    def generate_api_key(self, prefix: str = "AEROMIND_") -> str:
        import secrets
        return prefix + secrets.token_urlsafe(32)
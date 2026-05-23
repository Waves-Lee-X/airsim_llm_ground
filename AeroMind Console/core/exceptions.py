from __future__ import annotations

from dataclasses import dataclass, field
from http import HTTPStatus
from typing import Any, Optional


class AeroMindError(Exception):
    error_code: str = "AM_ERROR"
    http_status: int = HTTPStatus.INTERNAL_SERVER_ERROR
    message: str = "An error occurred"

    def __init__(self, message: str | None = None, **kwargs: Any) -> None:
        super().__init__(message or self.message)
        self.details = kwargs

    def to_api(self) -> dict[str, Any]:
        result = {
            "error": self.error_code,
            "message": str(self),
        }
        if self.details:
            result["details"] = self.details
        return result


class AuthError(AeroMindError):
    error_code = "AM_AUTH_ERROR"
    http_status = HTTPStatus.UNAUTHORIZED
    message = "Authentication failed"


class RateLimitError(AeroMindError):
    error_code = "AM_RATE_LIMIT"
    http_status = HTTPStatus.TOO_MANY_REQUESTS
    message = "Rate limit exceeded"


class ValidationError(AeroMindError):
    error_code = "AM_VALIDATION_ERROR"
    http_status = HTTPStatus.BAD_REQUEST
    message = "Validation failed"


class SafetyError(AeroMindError):
    error_code = "AM_SAFETY_ERROR"
    http_status = HTTPStatus.FORBIDDEN
    message = "Safety check failed"


class AirSimError(AeroMindError):
    error_code = "AM_AIRSIM_ERROR"
    http_status = HTTPStatus.SERVICE_UNAVAILABLE
    message = "AirSim connection error"


class AirSimFatalError(AirSimError):
    """Unrecoverable AirSim error requiring a full disconnect/reconnect cycle."""

    error_code = "AM_AIRSIM_FATAL"
    message = "AirSim fatal error — reconnect required"


class TaskError(AeroMindError):
    error_code = "AM_TASK_ERROR"
    http_status = HTTPStatus.INTERNAL_SERVER_ERROR
    message = "Task execution error"


class NotFoundError(AeroMindError):
    error_code = "AM_NOT_FOUND"
    http_status = HTTPStatus.NOT_FOUND
    message = "Resource not found"


class ConflictError(AeroMindError):
    error_code = "AM_CONFLICT"
    http_status = HTTPStatus.CONFLICT
    message = "Conflict detected"


class PreconditionError(AeroMindError):
    error_code = "AM_PRECONDITION_ERROR"
    http_status = HTTPStatus.PRECONDITION_FAILED
    message = "Precondition not met"


@dataclass(frozen=True)
class ErrorResponse:
    error: str
    message: str
    details: dict[str, Any] = field(default_factory=dict)
    timestamp: str = field(default_factory=lambda: __import__("datetime").datetime.now().isoformat())

    @classmethod
    def from_exception(cls, exc: AeroMindError) -> "ErrorResponse":
        return cls(
            error=exc.error_code,
            message=str(exc),
            details=exc.details,
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "error": self.error,
            "message": self.message,
            "details": self.details,
            "timestamp": self.timestamp,
        }


class ErrorHandler:
    @staticmethod
    def handle_exception(exc: Exception) -> tuple[int, dict[str, Any]]:
        if isinstance(exc, AeroMindError):
            return exc.http_status, ErrorResponse.from_exception(exc).to_dict()
        
        if isinstance(exc, ValueError):
            return HTTPStatus.BAD_REQUEST, ErrorResponse(
                error="AM_VALIDATION_ERROR",
                message=str(exc),
            ).to_dict()
        
        if isinstance(exc, TypeError):
            return HTTPStatus.BAD_REQUEST, ErrorResponse(
                error="AM_VALIDATION_ERROR",
                message=f"Invalid argument type: {exc}",
            ).to_dict()
        
        import traceback
        return HTTPStatus.INTERNAL_SERVER_ERROR, ErrorResponse(
            error="AM_UNKNOWN_ERROR",
            message="An unexpected error occurred",
            details={
                "exception_type": type(exc).__name__,
                "traceback": traceback.format_exc()[:2000],
            },
        ).to_dict()

    @staticmethod
    def ensure_success(result: dict[str, Any], default_error: str = "Operation failed") -> dict[str, Any]:
        if result.get("ok", True) is False:
            raise TaskError(result.get("error", default_error), details=result)
        return result
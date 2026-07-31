"""Browser-facing gateway for the manually managed SITL workflow."""

from .runtime import HybridRuntime, ManualRuntime, ManualRuntimeConfig, ManualRuntimeMode

__all__ = [
    "AirSimCameraBridge",
    "AirSimCameraConfig",
    "RtspCameraBridge",
    "RtspCameraConfig",
    "BrowserGateway",
    "HybridRuntime",
    "ManualRuntime",
    "ManualRuntimeConfig",
    "ManualRuntimeMode",
    "create_app",
]


def __getattr__(name):
    if name in {
        "AirSimCameraBridge",
        "AirSimCameraConfig",
        "RtspCameraBridge",
        "RtspCameraConfig",
    }:
        from .camera import (
            AirSimCameraBridge,
            AirSimCameraConfig,
            RtspCameraBridge,
            RtspCameraConfig,
        )

        return {
            "AirSimCameraBridge": AirSimCameraBridge,
            "AirSimCameraConfig": AirSimCameraConfig,
            "RtspCameraBridge": RtspCameraBridge,
            "RtspCameraConfig": RtspCameraConfig,
        }[name]
    if name in {"BrowserGateway", "create_app"}:
        from .app import BrowserGateway, create_app

        return {
            "BrowserGateway": BrowserGateway,
            "create_app": create_app,
        }[name]
    raise AttributeError(name)

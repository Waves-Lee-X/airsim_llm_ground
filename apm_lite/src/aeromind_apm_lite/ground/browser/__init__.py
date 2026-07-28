"""Browser-facing gateway for the manually managed SITL workflow."""

from .runtime import ManualRuntime, ManualRuntimeConfig, ManualRuntimeMode

__all__ = [
    "AirSimCameraBridge",
    "AirSimCameraConfig",
    "BrowserGateway",
    "ManualRuntime",
    "ManualRuntimeConfig",
    "ManualRuntimeMode",
    "create_app",
]


def __getattr__(name):
    if name in {"AirSimCameraBridge", "AirSimCameraConfig"}:
        from .camera import AirSimCameraBridge, AirSimCameraConfig

        return {
            "AirSimCameraBridge": AirSimCameraBridge,
            "AirSimCameraConfig": AirSimCameraConfig,
        }[name]
    if name in {"BrowserGateway", "create_app"}:
        from .app import BrowserGateway, create_app

        return {
            "BrowserGateway": BrowserGateway,
            "create_app": create_app,
        }[name]
    raise AttributeError(name)

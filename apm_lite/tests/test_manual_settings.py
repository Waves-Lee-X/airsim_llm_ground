import json
from pathlib import Path

import pytest

from aeromind_apm_lite.ground.simulation.config import SimulationConfigError
from aeromind_apm_lite.ground.simulation.manual_settings import (
    DEFAULT_MANUAL_SETTINGS_TEMPLATE,
    ManualCameraConfig,
    ManualSimulationConfig,
    build_manual_sitl_argv,
    main,
    render_manual_settings,
    write_manual_settings,
)


def manual_config(**kwargs):
    values = {
        "wsl_ip": "172.24.80.10",
        "windows_host_ip": "172.24.80.1",
    }
    values.update(kwargs)
    return ManualSimulationConfig(**values)


def test_manual_template_is_scene_independent_rgb_only():
    template = json.loads(DEFAULT_MANUAL_SETTINGS_TEMPLATE.read_text(encoding="utf-8"))
    vehicle = template["Vehicles"]["Drone1"]
    captures = vehicle["Cameras"]["front_center"]["CaptureSettings"]

    assert vehicle["VehicleType"] == "ArduCopter"
    assert vehicle["LockStep"] is True
    assert vehicle["UdpIp"] == "${AEROMIND_WSL_IPV4}"
    assert vehicle["UdpPort"] == 9003
    assert vehicle["ControlPort"] == 9002
    assert [capture["ImageType"] for capture in captures] == [0]
    assert "Depth" not in DEFAULT_MANUAL_SETTINGS_TEMPLATE.read_text(encoding="utf-8")


def test_render_uses_current_addresses_and_configurable_scene_camera():
    camera = ManualCameraConfig(
        width_px=640,
        height_px=480,
        fov_degrees=78,
        x_m=0.3,
        z_m=-0.08,
        pitch_deg=-5,
    )
    document = render_manual_settings(manual_config(camera=camera))
    vehicle = document["Vehicles"]["Drone1"]
    camera_document = vehicle["Cameras"]["front_center"]

    assert document["ClockType"] == "SteppableClock"
    assert vehicle["UdpIp"] == "172.24.80.10"
    assert vehicle["UdpPort"] == 9003
    assert vehicle["ControlPort"] == 9002
    assert camera_document["X"] == 0.3
    assert camera_document["Z"] == -0.08
    assert camera_document["Pitch"] == -5
    assert camera_document["CaptureSettings"] == [
        {
            "ImageType": 0,
            "Width": 640,
            "Height": 480,
            "FOV_Degrees": 78.0,
            "MotionBlurAmount": 0,
        }
    ]


def test_address_discovery_and_explicit_overrides(monkeypatch):
    import aeromind_apm_lite.ground.simulation.manual_settings as module

    monkeypatch.setattr(module, "discover_wsl_ipv4", lambda: "172.30.16.20")
    monkeypatch.setattr(
        module,
        "discover_windows_host_ipv4",
        lambda: "172.30.16.1",
    )
    discovered = ManualSimulationConfig()

    assert discovered.wsl_ip == "172.30.16.20"
    assert discovered.windows_host_ip == "172.30.16.1"

    monkeypatch.setattr(
        module,
        "discover_wsl_ipv4",
        lambda: pytest.fail("explicit WSL address must skip discovery"),
    )
    monkeypatch.setattr(
        module,
        "discover_windows_host_ipv4",
        lambda: pytest.fail("explicit host address must skip discovery"),
    )
    overridden = manual_config()

    assert overridden.wsl_ip == "172.24.80.10"
    assert overridden.windows_host_ip == "172.24.80.1"


def test_writer_is_atomic_and_requires_an_absolute_settings_filename(tmp_path):
    target = tmp_path / "Documents" / "AirSim" / "settings.json"

    assert write_manual_settings(manual_config(), target) == target
    vehicle = json.loads(target.read_text(encoding="utf-8"))["Vehicles"]["Drone1"]
    assert vehicle["UdpIp"] == "172.24.80.10"
    assert not list(target.parent.glob(".settings-manual-*.json"))

    with pytest.raises(SimulationConfigError, match="absolute settings.json"):
        write_manual_settings(manual_config(), Path("settings.json"))
    with pytest.raises(SimulationConfigError, match="absolute settings.json"):
        write_manual_settings(manual_config(), tmp_path / "manual.json")


def test_manual_sitl_command_uses_discovered_host_without_executing_it():
    argv = build_manual_sitl_argv(manual_config())

    assert argv[argv.index("--model") + 1] == "airsim-copter"
    assert argv[argv.index("--sim-address") + 1] == "172.24.80.1"
    assert argv[argv.index("--serial0") + 1] == "udpclient:127.0.0.1:14550"


def test_cli_writes_settings_and_reports_that_it_started_no_process(
    tmp_path,
    capsys,
):
    target = tmp_path / "settings.json"

    assert main(
        [
            "--output",
            str(target),
            "--wsl-ip",
            "172.24.80.10",
            "--windows-host-ip",
            "172.24.80.1",
            "--camera-width",
            "640",
            "--camera-height",
            "480",
        ]
    ) == 0

    result = json.loads(capsys.readouterr().out)
    settings = json.loads(target.read_text(encoding="utf-8"))
    captures = settings["Vehicles"]["Drone1"]["Cameras"]["front_center"][
        "CaptureSettings"
    ]
    assert result["settings_path"] == str(target)
    assert result["processes_started"] is False
    assert result["windows_host_ip"] == "172.24.80.1"
    assert "--sim-address 172.24.80.1" in result["sitl_command"]
    assert captures[0]["Width"] == 640
    assert captures[0]["Height"] == 480


@pytest.mark.parametrize(
    "camera",
    [
        lambda: ManualCameraConfig(width_px=0),
        lambda: ManualCameraConfig(fov_degrees=180),
        lambda: ManualCameraConfig(pitch_deg=-91),
    ],
)
def test_camera_profile_rejects_unphysical_values(camera):
    with pytest.raises(SimulationConfigError):
        camera()

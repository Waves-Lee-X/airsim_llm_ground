import json
from pathlib import Path

import pytest

from aeromind_apm_lite.common.config import load_fleet_config
from aeromind_apm_lite.ground.simulation import (
    ARDUPILOT_COMMIT,
    AirSimProcessConfig,
    AirSimSettings,
    ArduPilotBuild,
    SimulationConfigError,
    SimulationLaunchConfig,
    SimulationPaths,
    SitlInstanceConfig,
    discover_windows_host_ipv4,
    sitl_argv,
    validate_m1_parameters,
    windows_path_from_wsl,
)

ROOT = Path(__file__).resolve().parents[1]


def launch_config(tmp_path, *, wsl_ip="192.168.18.156", simulator_address=None):
    instance_kwargs = {"instance": 0}
    if simulator_address is not None:
        instance_kwargs["simulator_address"] = simulator_address
    return SimulationLaunchConfig(
        instances=(SitlInstanceConfig(**instance_kwargs),),
        paths=SimulationPaths(
            state_directory=tmp_path / "state",
            airsim_settings_path=tmp_path / "state" / "airsim" / "settings.json",
        ),
        wsl_ip=wsl_ip,
    )


def test_windows_host_is_discovered_from_lowest_metric_default_route(tmp_path):
    route = tmp_path / "route"
    route.write_text(
        "Iface Destination Gateway Flags RefCnt Use Metric Mask MTU Window IRTT\n"
        "eth0 00000000 0110A8C0 0003 0 0 50 00000000 0 0 0\n"
        "eth1 00000000 0100000A 0003 0 0 100 00000000 0 0 0\n",
        encoding="ascii",
    )

    assert discover_windows_host_ipv4(route) == "192.168.16.1"


def test_windows_host_discovery_rejects_route_without_gateway(tmp_path):
    route = tmp_path / "route"
    route.write_text(
        "Iface Destination Gateway Flags RefCnt Use Metric Mask MTU Window IRTT\n"
        "eth0 00000000 00000000 0001 0 0 0 00000000 0 0 0\n",
        encoding="ascii",
    )

    with pytest.raises(SimulationConfigError, match="could not discover"):
        discover_windows_host_ipv4(route)


def test_default_runtime_addresses_are_dynamic_and_not_loopback(tmp_path):
    launch = launch_config(tmp_path)

    assert launch.wsl_ip != "127.0.0.1"
    assert launch.instances[0].simulator_address != "127.0.0.1"


def test_airsim_settings_use_dynamic_wsl_ip_and_isolated_ports(tmp_path):
    launch = launch_config(tmp_path)
    document = AirSimSettings(launch).as_dict()
    vehicle = document["Vehicles"]["Drone1"]

    assert document["LocalHostIp"] == "0.0.0.0"
    assert vehicle["UdpIp"] == "192.168.18.156"
    assert vehicle["UdpPort"] == 9003
    assert vehicle["ControlPort"] == 9002
    assert vehicle["VehicleType"] == "ArduCopter"
    assert vehicle["LockStep"] is True
    assert "front_center" in vehicle["Cameras"]

    path = AirSimSettings(launch).write()
    assert path == launch.paths.airsim_settings_path
    assert json.loads(path.read_text(encoding="utf-8")) == document


def test_template_never_hardcodes_a_stale_wsl_ip():
    template = (ROOT / "configs/sim/airsim.m1.template.json").read_text(
        encoding="utf-8"
    )

    assert "${AEROMIND_WSL_IPV4}" in template
    assert "192.168.18.156" not in template
    assert json.loads(template)["Vehicles"]["Drone1"]["VehicleType"] == "ArduCopter"


def test_m1_fleet_is_single_vehicle_and_has_usable_trajectory_ttl():
    fleet = load_fleet_config(ROOT / "configs/sim/fleet.m1.yaml")

    assert len(fleet.vehicles) == 1
    vehicle = fleet.vehicles[0]
    assert vehicle.vehicle_id == 1
    assert vehicle.fcu.source_system == 201
    assert vehicle.fcu.endpoint == "udpin:0.0.0.0:14550"
    assert vehicle.trajectory_timeout_ms == 10_000


def test_m1_parameter_freeze_and_raw_sitl_argv_include_custom_defaults():
    instance = SitlInstanceConfig(
        instance=0,
        simulator_address="192.168.16.1",
    )

    validate_m1_parameters(instance)
    argv = sitl_argv(instance)
    defaults = argv[argv.index("--defaults") + 1]

    assert instance.parameter_file == ROOT / "configs/sim/arducopter-m1.parm"
    assert str(instance.parameter_file) in defaults.split(",")
    assert argv[argv.index("--sim-address") + 1] == "192.168.16.1"
    assert argv[argv.index("--serial0") + 1] == "udpclient:127.0.0.1:14550"
    assert "--wipe" in argv


def test_parameter_freeze_detects_wrong_gcs_id(tmp_path):
    source = (ROOT / "configs/sim/arducopter-m1.parm").read_text(encoding="ascii")
    parameter_file = tmp_path / "wrong.parm"
    parameter_file.write_text(
        source.replace("MAV_GCS_SYSID 201", "MAV_GCS_SYSID 255"),
        encoding="ascii",
    )
    instance = SitlInstanceConfig(
        instance=0,
        simulator_address="192.168.16.1",
        parameter_file=parameter_file,
    )

    with pytest.raises(SimulationConfigError, match="MAV_GCS_SYSID must be 201"):
        validate_m1_parameters(instance)


@pytest.mark.parametrize(
    ("name", "expected", "replacement"),
    [
        ("SIM_GPS1_HZ", "5", "10"),
        ("GPS1_RATE_MS", "200", "100"),
        ("GPS1_DELAY_MS", "200", "20"),
    ],
)
def test_parameter_freeze_detects_gps_rate_regression(
    tmp_path,
    name,
    expected,
    replacement,
):
    source = (ROOT / "configs/sim/arducopter-m1.parm").read_text(encoding="ascii")
    parameter_file = tmp_path / "wrong-gps-rate.parm"
    parameter_file.write_text(
        source.replace(f"{name} {expected}", f"{name} {replacement}"),
        encoding="ascii",
    )
    instance = SitlInstanceConfig(
        instance=0,
        simulator_address="192.168.16.1",
        parameter_file=parameter_file,
    )

    with pytest.raises(SimulationConfigError, match=rf"{name} must be"):
        validate_m1_parameters(instance)


def test_build_commit_and_model_cannot_drift(tmp_path):
    with pytest.raises(SimulationConfigError, match="commit must remain pinned"):
        ArduPilotBuild(source_commit="0" * 40)
    with pytest.raises(SimulationConfigError, match="model must remain pinned"):
        SitlInstanceConfig(
            instance=0,
            model="quad",
            simulator_address="192.168.16.1",
        )
    assert ARDUPILOT_COMMIT == "1511f27194f1dcc3728270883047bdf022b3fd53"


def test_airsim_settings_argument_is_owned_by_lifecycle(tmp_path):
    executable = tmp_path / "AirSim.exe"
    executable.touch()

    with pytest.raises(SimulationConfigError, match="lifecycle-owned"):
        AirSimProcessConfig(executable, arguments=("-settings=other.json",))


def test_wsl_paths_translate_for_windows_processes():
    assert windows_path_from_wsl(Path("/mnt/d/AirSim/settings.json")) == (
        "D:\\AirSim\\settings.json"
    )
    assert windows_path_from_wsl(
        Path("/home/waves/state/settings.json"),
        distro_name="Ubuntu-22.04",
    ) == (
        "\\\\wsl.localhost\\Ubuntu-22.04\\home\\waves\\state\\settings.json"
    )

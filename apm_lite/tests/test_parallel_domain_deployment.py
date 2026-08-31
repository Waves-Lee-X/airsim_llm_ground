from __future__ import annotations

import json
from pathlib import Path

import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEPLOY_ROOT = PROJECT_ROOT / "deploy" / "parallel_domain"


def test_parallel_domain_instance_table_is_disjoint_and_explicit():
    payload = yaml.safe_load((DEPLOY_ROOT / "worlds.yaml").read_text(encoding="utf-8"))
    instances = payload["instances"]
    assert [
        (item["line"], item["role"], item["vehicle_id"], item["mavlink_endpoint"])
        for item in instances
    ] == [
        ("px4", "real", 1, "udpin:0.0.0.0:14540"),
        ("px4", "virtual", 1, "udpin:0.0.0.0:14541"),
        ("apm", "real", 2, "udpin:0.0.0.0:14550"),
        ("apm", "virtual", 2, "udpin:0.0.0.0:14551"),
    ]
    assert len({item["mavlink_endpoint"] for item in instances}) == 4
    assert {item["mavlink_dialect"] for item in instances} == {"common", "ardupilotmega"}
    assert instances[0]["gazebo_qgc_udp"] == 14640
    assert instances[1]["qgc_udp"] == 14641
    assert instances[2]["sitl_speedup"] == 10
    assert instances[3]["sitl_speedup"] == 10
    assert payload["georeference"]["calibration_id"] == "parallel-sitl-locked-v1"
    assert payload["georeference"]["status"] == "surveyed"


def test_airsim_templates_are_valid_and_have_distinct_transport_ports():
    px4 = json.loads(
        (PROJECT_ROOT / "configs/parallel_domain/airsim/px4.settings.json").read_text()
    )
    apm = json.loads(
        (PROJECT_ROOT / "configs/parallel_domain/airsim/apm.settings.json").read_text()
    )
    assert px4["Vehicles"]["Drone1"]["TcpPort"] == 4561
    assert apm["Vehicles"]["Drone1"]["UdpPort"] == 9003
    assert apm["Vehicles"]["Drone1"]["ControlPort"] == 9002
    assert px4["ApiServerPort"] == 41451
    assert apm["ApiServerPort"] == 41452
    assert px4["ApiServerPort"] != apm["ApiServerPort"]


def test_windows_launcher_uses_two_explicit_settings_profiles():
    script = (DEPLOY_ROOT / "start_all.ps1").read_text(encoding="utf-8")
    assert "-settings=" in script
    assert "AirSimProfileRoot" in script
    assert "standardHashBefore" in script and "standardHashAfter" in script
    assert "ApiServerPort" in script
    assert "live-processes.json" in script
    assert "SkipSITL" in script
    assert "$PSScriptRoot" in script
    assert "/home/waves/aeromind_ws/apm_lite" not in script
    assert "System.Security.Cryptography.SHA256" in script
    assert "Get-FileHash" not in script


def test_control_bridge_is_separate_from_observation_bridge():
    observation = yaml.safe_load(
        (PROJECT_ROOT / "configs/parallel_domain/bridge.yaml").read_text(
            encoding="utf-8"
        )
    )
    control = yaml.safe_load(
        (PROJECT_ROOT / "configs/parallel_domain/bridge-control.yaml").read_text(
            encoding="utf-8"
        )
    )
    assert observation["observation_only"] is True
    assert control["observation_only"] is False
    assert observation["state_directory"] != control["state_directory"]
    assert [line["line_id"] for line in observation["lines"]] == [
        line["line_id"] for line in control["lines"]
    ]
    assert all(
        endpoint.get("force_arm_for_sitl") is True
        for line in control["lines"]
        for endpoint in (line["real"], line["virtual"])
    )
    assert all(
        endpoint.get("force_arm_for_sitl", False) is False
        for line in observation["lines"]
        for endpoint in (line["real"], line["virtual"])
    )


def test_all_runtime_helpers_are_present_and_executable():
    for name in (
        "start_all.sh",
        "stop_all.sh",
        "run_px4_gazebo.sh",
        "run_px4_airsim.sh",
        "run_apm_native.sh",
        "run_apm_airsim.sh",
        "../sim/start-parallel-observer.sh",
        "../sim/start-parallel-control.sh",
    ):
        path = DEPLOY_ROOT / name
        assert path.is_file() and path.stat().st_mode & 0o111


def test_px4_gazebo_uses_the_shared_georeference_origin():
    script = (DEPLOY_ROOT / "run_px4_gazebo.sh").read_text(encoding="utf-8")
    assert 'PX4_HOME_LAT="${PX4_HOME_LAT:-47.641468}"' in script
    assert 'PX4_HOME_LON="${PX4_HOME_LON:--122.140165}"' in script
    assert 'PX4_HOME_ALT="${PX4_HOME_ALT:-122.0}"' in script
    assert "mktemp --suffix=.sdf" in script
    assert 'mv -f "$SDF_TMP" "$RUNTIME_DIR/$MODEL.sdf"' in script


def test_px4_airsim_virtual_telemetry_is_forwarded_to_bridge_port():
    script = (DEPLOY_ROOT / "run_px4_airsim.sh").read_text(encoding="utf-8")
    assert "PX4_SIM_HOST_ADDR=\"$AIRSIM_HOST\"" in script
    assert "14541" in script


def test_apm_native_sitl_publishes_the_read_only_bridge_message_set():
    params = (PROJECT_ROOT / "configs/parallel_domain/apm-native-sitl.parm").read_text(
        encoding="utf-8"
    )
    assert "MAV1_EXT_STAT 2" in params
    assert "MAV1_POSITION 10" in params
    assert "MAV1_EXTRA1 10" in params


def test_live_bridge_uses_locked_sitl_calibration_not_draft_preview():
    bridge = yaml.safe_load(
        (PROJECT_ROOT / "configs/parallel_domain/bridge.yaml").read_text(
            encoding="utf-8"
        )
    )
    assert bridge["georeference"] == "sitl-georeference.yaml"
    georeference = yaml.safe_load(
        (PROJECT_ROOT / "configs/parallel_domain/sitl-georeference.yaml").read_text(
            encoding="utf-8"
        )
    )
    assert georeference["status"] == "surveyed"
    assert "not an outdoor venue survey" in georeference["notes"]


def test_preflight_probe_can_gate_only_live_real_observed_telemetry():
    script = (PROJECT_ROOT / "tools/probe_parallel_domain.py").read_text(
        encoding="utf-8"
    )
    assert 'choices=("all", "real", "virtual")' in script
    assert '"--require-observed-telemetry"' in script
    for message_type in (
        "GPS_RAW_INT",
        "GLOBAL_POSITION_INT",
        "ATTITUDE_QUATERNION",
        "ATTITUDE",
    ):
        assert message_type in script

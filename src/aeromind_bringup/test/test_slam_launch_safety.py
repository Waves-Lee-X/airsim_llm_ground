from pathlib import Path


def test_slam_launch_keeps_flight_control_in_px4_local_odom():
    source = (
        Path(__file__).parents[1] / "launch" / "aeromind_slam.launch.py"
    ).read_text(encoding="utf-8")
    assert '"autonomy_odom_topic": "/sensor/odometry"' in source
    assert '"semantic_world_frame": "map"' in source
    assert '"output_odom_topic": "/localization/odometry"' in source

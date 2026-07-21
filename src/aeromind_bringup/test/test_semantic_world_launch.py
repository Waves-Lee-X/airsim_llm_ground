from pathlib import Path


def test_primary_launches_include_fusion_and_tracker_nodes():
    launch_dir = Path(__file__).parents[1] / "launch"
    for filename in ("aeromind_px4.launch.py", "aeromind_all.launch.py"):
        source = (launch_dir / filename).read_text(encoding="utf-8")
        assert 'executable="semantic_fusion_node"' in source
        assert 'executable="object_tracker_node"' in source
        assert "semantic_world_enabled" in source

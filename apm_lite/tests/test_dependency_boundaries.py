import ast
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = ROOT / "src" / "aeromind_apm_lite"
FORBIDDEN_IMPORTS = ("rclpy", "px4_msgs", "mavros")
FORBIDDEN_AIRSIM_CONTROL_CALLS = (
    "moveToPositionAsync",
    "moveByVelocityAsync",
    "moveOnPathAsync",
    "takeoffAsync",
    "landAsync",
)


def imported_modules(path: Path):
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            yield from (alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            yield node.module


def test_lite_source_has_no_ros_px4_or_mavros_imports():
    violations = []
    for path in SOURCE_ROOT.rglob("*.py"):
        for module in imported_modules(path):
            if module.startswith(FORBIDDEN_IMPORTS):
                violations.append(f"{path.relative_to(ROOT)}: {module}")
    assert violations == []


def test_lite_source_does_not_bypass_sitl_with_airsim_flight_calls():
    violations = []
    for path in SOURCE_ROOT.rglob("*.py"):
        text = path.read_text(encoding="utf-8")
        for call in FORBIDDEN_AIRSIM_CONTROL_CALLS:
            if call in text:
                violations.append(f"{path.relative_to(ROOT)}: {call}")
    assert violations == []

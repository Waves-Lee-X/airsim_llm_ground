import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_committed_protocol_schema_contains_wire_fleet_georeference_and_trajectory():
    schema_path = ROOT / "schemas" / "protocol-v1.schema.json"
    schema = json.loads(schema_path.read_text(encoding="utf-8"))

    assert schema["schema_version"] == "1.0"
    assert set(schema["schemas"]) == {
        "fleet_config",
        "georeference",
        "trajectory_comparison_report",
        "trajectory_evidence",
        "wire_message",
    }
    reference = schema["schemas"]["georeference"]
    assert reference["properties"]["schema_version"]["const"] == "1.0"
    assert "vehicle_homes" in reference["required"]

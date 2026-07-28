import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_committed_protocol_schema_contains_wire_and_fleet_contracts():
    schema_path = ROOT / "schemas" / "protocol-v1.schema.json"
    schema = json.loads(schema_path.read_text(encoding="utf-8"))

    assert schema["schema_version"] == "1.0"
    assert set(schema["schemas"]) == {"fleet_config", "wire_message"}

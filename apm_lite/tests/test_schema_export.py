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
        "vision_analysis_evidence",
        "color_detection",
        "wire_message",
    }
    assert (
        schema["schemas"]["vision_analysis_evidence"]["properties"][
            "schema_version"
        ]["const"]
        == "1.0"
    )
    assert "color" in schema["schemas"]["color_detection"]["required"]
    reference = schema["schemas"]["georeference"]
    assert reference["properties"]["schema_version"]["const"] == "1.0"
    assert "vehicle_homes" in reference["required"]


def test_committed_m1_schemas_cover_twin_deduction_and_evidence_contracts():
    expected = {
        "twin-v1.schema.json": {
            "twin_state",
            "vehicle_profile",
            "mission_graph",
            "world_event",
            "strategy_spec",
            "metric_record",
        },
        "deduction-v1.schema.json": {
            "baseline_snapshot",
            "branch_spec",
            "branch_result",
            "deduction_run",
            "pareto_report",
        },
        "evidence-v1.schema.json": {
            "experiment_lock",
            "evidence_manifest",
            "evidence_package",
        },
    }

    for filename, schemas in expected.items():
        payload = json.loads((ROOT / "schemas" / filename).read_text(encoding="utf-8"))
        assert payload["schema_version"] == "1.0"
        assert set(payload["schemas"]) == schemas

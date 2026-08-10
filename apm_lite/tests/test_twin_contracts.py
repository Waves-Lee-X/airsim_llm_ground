from datetime import datetime, timedelta, timezone
from uuid import uuid4

import pytest
from pydantic import ValidationError

from aeromind_apm_lite.common.contracts import CoordinateFrame, Vector3
from aeromind_apm_lite.common.contracts.twin import (
    FidelityLevel,
    HealthStatus,
    MetricDirection,
    MetricRecord,
    MissionEdge,
    MissionGraph,
    MissionNode,
    StrategyKind,
    StrategySpec,
    TwinSource,
    TwinState,
    VehicleProfile,
    WorldEvent,
    source_for_runtime_mode,
)


NOW = datetime(2026, 8, 10, 8, 0, tzinfo=timezone.utc)


def twin_state(*, source=TwinSource.SIM, vehicle_id=1):
    timestamps = (
        {"simulated_at_utc": NOW}
        if source in {TwinSource.SIM, TwinSource.PREDICTED}
        else {"mapped_at_utc": NOW}
    )
    return TwinState(
        vehicle_id=vehicle_id,
        platform_type="multirotor",
        frame=CoordinateFrame.MAP,
        frame_calibration_id="test-field-v1",
        position_m=Vector3(x=float(vehicle_id), y=0.0, z=3.0),
        velocity_m_s=Vector3(x=1.0, y=0.0, z=0.0),
        energy_remaining=0.8,
        communication_quality=0.9,
        mission_phase="search",
        health_status=HealthStatus.NOMINAL,
        source=source,
        confidence=0.95,
        fidelity=FidelityLevel.L0,
        sampled_at_utc=NOW,
        **timestamps,
    )


def test_twin_state_keeps_runtime_sources_and_clocks_separate():
    assert source_for_runtime_mode("demo") == TwinSource.SIM
    assert source_for_runtime_mode("sitl") == TwinSource.SIM
    assert source_for_runtime_mode("real") == TwinSource.REAL
    assert twin_state(source=TwinSource.SIM).simulated_at_utc == NOW
    assert twin_state(source=TwinSource.REAL).mapped_at_utc == NOW

    payload = twin_state().model_dump(mode="json")
    payload["source"] = "real"
    with pytest.raises(ValidationError, match="mapped_at_utc"):
        TwinState.model_validate(payload)

    payload = twin_state().model_dump(mode="json")
    payload["mapped_at_utc"] = NOW.isoformat()
    with pytest.raises(ValidationError, match="must not mix"):
        TwinState.model_validate(payload)


def test_twin_contracts_are_strict_and_graph_dependencies_are_acyclic():
    payload = twin_state().model_dump(mode="python")
    payload["sampled_at_utc"] = datetime.now()
    with pytest.raises(ValidationError, match="timezone"):
        TwinState.model_validate(payload)

    nodes = (
        MissionNode(node_id="survey", action="survey"),
        MissionNode(node_id="report", action="report"),
    )
    graph = MissionGraph(
        mission_id=uuid4(),
        nodes=nodes,
        edges=(MissionEdge(source_node_id="survey", target_node_id="report"),),
    )
    assert graph.nodes[0].node_id == "survey"

    with pytest.raises(ValidationError, match="cycle"):
        MissionGraph(
            mission_id=uuid4(),
            nodes=nodes,
            edges=(
                MissionEdge(source_node_id="survey", target_node_id="report"),
                MissionEdge(source_node_id="report", target_node_id="survey"),
            ),
        )


def test_profile_event_strategy_and_metric_share_versioned_contracts():
    profile = VehicleProfile(
        vehicle_id=1,
        display_name="UAV1",
        platform_type="multirotor",
        sensors=("rgb", "gps"),
        payloads=("camera",),
        max_endurance_min=24.0,
        max_speed_m_s=12.0,
        capabilities=("survey", "capture", "relay"),
        whitelist_level=2,
    )
    event = WorldEvent(
        event_type="link_degradation",
        occurs_at_utc=NOW + timedelta(seconds=5),
        affected_vehicle_ids=(1,),
        parameters={"quality": 0.25},
        source=TwinSource.SIM,
    )
    strategy = StrategySpec(
        strategy_id="central-v1",
        kind=StrategyKind.CENTRALIZED_OPTIMIZATION,
        random_seed=42,
        parameters={"cruise_speed_m_s": 8.0},
        metric_names=("task_completion_rate", "elapsed_time_s"),
    )
    metric = MetricRecord(
        branch_id="branch-1",
        metric_name="task_completion_rate",
        value=0.9,
        unit="ratio",
        direction=MetricDirection.MAXIMIZE,
        source="l0",
        recorded_at_utc=NOW,
    )

    assert profile.schema_version == event.schema_version == strategy.schema_version == "1.0"
    assert metric.direction == MetricDirection.MAXIMIZE

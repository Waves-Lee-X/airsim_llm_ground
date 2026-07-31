"""Formation geometry, slot assignment, separation and fleet loss handling."""

import pytest

from aeromind_apm_lite.common.formation import (
    FleetCoordinator,
    FleetPlanError,
    FleetVehicleState,
    FormationType,
    assign_slots,
    formation_offsets,
    minimum_pairwise_distance,
    transition_offsets,
    validate_spacing,
    validate_trajectory_separation,
)


def test_line_formation_offsets_are_centered_and_spaced():
    offsets = formation_offsets(FormationType.LINE, 4, spacing_m=3.0)
    assert offsets == (
        (0.0, -4.5),
        (0.0, -1.5),
        (0.0, 1.5),
        (0.0, 4.5),
    )
    assert minimum_pairwise_distance(offsets) == pytest.approx(3.0)
    validate_spacing(offsets, min_spacing_m=3.0)


def test_all_formations_respect_minimum_spacing():
    for formation in FormationType:
        for count in (2, 3, 4, 5, 6):
            offsets = formation_offsets(formation, count, spacing_m=3.0)
            assert minimum_pairwise_distance(offsets) >= 3.0 - 1e-6
            validate_spacing(offsets, min_spacing_m=3.0)


def test_v_formation_has_symmetric_wings():
    offsets = formation_offsets(FormationType.V, 5, spacing_m=2.0)
    assert offsets[0] == (0.0, 0.0)
    left = offsets[1]
    right = offsets[2]
    assert left[0] == right[0]
    assert left[1] == pytest.approx(-right[1])
    assert left[0] < 0.0


def test_diamond_formation_orders_center_front_left_right_rear():
    offsets = formation_offsets(FormationType.DIAMOND, 5, spacing_m=3.0)
    assert offsets[0] == (0.0, 0.0)
    assert offsets[1] == (3.0, 0.0)
    assert offsets[2] == (0.0, -3.0)
    assert offsets[3] == (0.0, 3.0)
    assert offsets[4] == (-3.0, 0.0)


def test_spacing_validation_rejects_too_close_slots():
    offsets = ((0.0, 0.0), (1.0, 0.0), (0.0, 3.0))
    with pytest.raises(ValueError):
        validate_spacing(offsets, min_spacing_m=3.0)


def test_transition_offsets_interpolate_and_keep_slot_order():
    start = formation_offsets(FormationType.LINE, 4, spacing_m=3.0)
    end = formation_offsets(FormationType.DIAMOND, 4, spacing_m=3.0)
    assert transition_offsets(
        FormationType.LINE, FormationType.DIAMOND, 4, 3.0, 0.0
    ) == start
    assert transition_offsets(
        FormationType.LINE, FormationType.DIAMOND, 4, 3.0, 1.0
    ) == end
    middle = transition_offsets(
        FormationType.LINE, FormationType.DIAMOND, 4, 3.0, 0.5
    )
    for index in range(4):
        sx, sy = start[index]
        ex, ey = end[index]
        middle_x, middle_y = middle[index]
        assert middle_x == pytest.approx((sx + ex) / 2.0)
        assert middle_y == pytest.approx((sy + ey) / 2.0)
    with pytest.raises(ValueError):
        transition_offsets(FormationType.LINE, FormationType.DIAMOND, 4, 3.0, 1.5)


def test_slot_assignment_is_deterministic_and_nearest():
    offsets = formation_offsets(FormationType.LINE, 3, spacing_m=3.0)
    positions = {
        1: (0.2, -4.6, 0.0),
        2: (0.0, 1.6, 0.0),
        3: (0.1, -1.4, 0.0),
    }
    first = assign_slots([1, 2, 3], positions, offsets)
    second = assign_slots([3, 2, 1], positions, offsets)
    assert first == second
    assert first[1] == 0  # nearest to (0, -4.5)
    assert first[2] == 2  # nearest to (0, 1.5)
    assert first[3] == 1  # nearest to (0, -1.5)


def test_slot_assignment_requires_matching_counts():
    with pytest.raises(FleetPlanError):
        assign_slots([1, 2], {}, formation_offsets(FormationType.LINE, 3, 3.0))


def _crossing_trajectories():
    return {
        1: [(0.0, 0.0, 0.0, -2.0), (10.0, 6.0, 0.0, -2.0)],
        2: [(0.0, 6.0, 0.0, -2.0), (10.0, 0.0, 0.0, -2.0)],
    }


def test_trajectory_separation_detects_crossing_paths():
    with pytest.raises(FleetPlanError):
        validate_trajectory_separation(
            _crossing_trajectories(),
            min_distance_m=3.0,
        )


def test_trajectory_separation_accepts_parallel_lanes():
    trajectories = {
        1: [(0.0, 0.0, 0.0, -2.0), (10.0, 0.0, 0.0, -2.0)],
        2: [(0.0, 0.0, 5.0, -2.0), (10.0, 0.0, 5.0, -2.0)],
    }
    report = validate_trajectory_separation(
        trajectories,
        min_distance_m=3.0,
    )
    assert report["min_distance_m"] == pytest.approx(5.0)


def _ready_state(vehicle_id, x, y):
    return FleetVehicleState(
        vehicle_id=vehicle_id,
        ready=True,
        link_ok=True,
        last_seen_age_s=0.1,
        position_map_m=(x, y, -2.0),
    )


def test_coordinator_emits_goto_directives_when_all_ready():
    coordinator = FleetCoordinator(min_spacing_m=3.0)
    states = {
        1: _ready_state(1, 0.0, -4.6),
        2: _ready_state(2, 0.0, -1.4),
        3: _ready_state(3, 0.0, 1.6),
        4: _ready_state(4, 0.0, 4.4),
    }
    decision = coordinator.plan(
        states,
        FormationType.LINE,
        leader_target_map_m=(10.0, 0.0, -2.0),
    )
    assert decision.phase == "formation_hold"
    by_id = {directive.vehicle_id: directive for directive in decision.directives}
    assert all(directive.action == "goto" for directive in decision.directives)
    assert by_id[1].target_map_m == (10.0, -4.5, -2.0)
    assert by_id[2].target_map_m == (10.0, -1.5, -2.0)
    assert by_id[3].target_map_m == (10.0, 1.5, -2.0)
    assert by_id[4].target_map_m == (10.0, 4.5, -2.0)


def test_coordinator_holds_until_all_vehicles_ready():
    coordinator = FleetCoordinator(min_spacing_m=3.0)
    states = {
        1: _ready_state(1, 0.0, -1.5),
        2: FleetVehicleState(
            vehicle_id=2,
            ready=False,
            link_ok=True,
            last_seen_age_s=0.1,
            position_map_m=(0.0, 1.5, -2.0),
        ),
    }
    decision = coordinator.plan(
        states,
        FormationType.LINE,
        leader_target_map_m=(10.0, 0.0, -2.0),
    )
    assert decision.phase == "syncing"
    assert all(directive.action == "hold" for directive in decision.directives)


def test_coordinator_holds_on_stale_telemetry():
    coordinator = FleetCoordinator(min_spacing_m=3.0, sync_timeout_s=5.0)
    states = {
        1: _ready_state(1, 0.0, -1.5),
        2: FleetVehicleState(
            vehicle_id=2,
            ready=True,
            link_ok=True,
            last_seen_age_s=12.0,
            position_map_m=(0.0, 1.5, -2.0),
        ),
    }
    decision = coordinator.plan(
        states,
        FormationType.LINE,
        leader_target_map_m=(10.0, 0.0, -2.0),
    )
    assert decision.phase == "syncing"
    assert all(directive.action == "hold" for directive in decision.directives)


def test_coordinator_degrades_formation_when_one_vehicle_is_lost():
    coordinator = FleetCoordinator(min_spacing_m=3.0, lost_timeout_s=5.0)
    states = {
        1: _ready_state(1, 0.0, -3.0),
        2: _ready_state(2, 0.0, 0.0),
        3: _ready_state(3, 0.0, 3.0),
        4: FleetVehicleState(
            vehicle_id=4,
            ready=True,
            link_ok=False,
            last_seen_age_s=30.0,
            position_map_m=(0.0, 6.0, -2.0),
        ),
    }
    decision = coordinator.plan(
        states,
        FormationType.LINE,
        leader_target_map_m=(10.0, 0.0, -2.0),
    )
    assert decision.phase == "degraded"
    assert [directive.vehicle_id for directive in decision.directives] == [1, 2, 3]
    assert all(directive.action == "goto" for directive in decision.directives)
    assert all("degraded" in directive.reason for directive in decision.directives)


def test_coordinator_aborts_when_too_many_vehicles_are_lost():
    coordinator = FleetCoordinator(min_spacing_m=3.0, lost_timeout_s=5.0)
    states = {
        1: _ready_state(1, 0.0, -3.0),
        2: FleetVehicleState(
            vehicle_id=2,
            ready=True,
            link_ok=False,
            last_seen_age_s=30.0,
            position_map_m=(0.0, 0.0, -2.0),
        ),
    }
    decision = coordinator.plan(
        states,
        FormationType.LINE,
        leader_target_map_m=(10.0, 0.0, -2.0),
    )
    assert decision.phase == "aborted"
    assert all(directive.action == "hold" for directive in decision.directives)


def test_coordinator_requires_positive_spacing():
    with pytest.raises(ValueError):
        FleetCoordinator(min_spacing_m=0.0)


def test_fleet_launch_config_generates_isolated_four_vehicle_sitl():
    from aeromind_apm_lite.ground.simulation.config import (
        SimulationConfigError,
        fleet_launch_config,
    )

    config = fleet_launch_config(4, spacing_m=3.0)
    assert len(config.instances) == 4
    assert [instance.vehicle_name for instance in config.instances] == [
        "Drone1",
        "Drone2",
        "Drone3",
        "Drone4",
    ]
    assert [instance.system_id for instance in config.instances] == [1, 2, 3, 4]
    ports = [instance.mavlink.port for instance in config.instances]
    assert len(ports) == len(set(ports))
    sensor_ports = [instance.sensor_port for instance in config.instances]
    assert len(sensor_ports) == len(set(sensor_ports))
    initial_ys = [instance.initial_pose.y_m for instance in config.instances]
    assert initial_ys == [-4.5, -1.5, 1.5, 4.5]
    with pytest.raises(SimulationConfigError):
        fleet_launch_config(1)


def test_coordinator_waits_for_never_seen_vehicles_instead_of_aborting():
    coordinator = FleetCoordinator(min_spacing_m=3.0, lost_timeout_s=5.0)
    states = {
        1: _ready_state(1, 0.0, -1.5),
        2: FleetVehicleState(
            vehicle_id=2,
            ready=False,
            link_ok=False,
            last_seen_age_s=None,
        ),
    }
    decision = coordinator.plan(
        states,
        FormationType.LINE,
        leader_target_map_m=(10.0, 0.0, -2.0),
    )
    assert decision.phase == "syncing"
    assert all(directive.action == "hold" for directive in decision.directives)

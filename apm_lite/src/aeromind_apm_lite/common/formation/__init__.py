"""Fleet formation geometry and deterministic fleet planning."""

from .fleet import (
    FleetCoordinator,
    FleetDecision,
    FleetDirective,
    FleetPlanError,
    FleetVehicleState,
    assign_slots,
    trajectory_separation_report,
    validate_trajectory_separation,
)
from .formations import (
    FORMATION_LABELS,
    FormationType,
    formation_offsets,
    minimum_pairwise_distance,
    transition_offsets,
    validate_spacing,
)

__all__ = [
    "FORMATION_LABELS",
    "FormationType",
    "FleetCoordinator",
    "FleetDecision",
    "FleetDirective",
    "FleetPlanError",
    "FleetVehicleState",
    "assign_slots",
    "formation_offsets",
    "minimum_pairwise_distance",
    "trajectory_separation_report",
    "transition_offsets",
    "validate_spacing",
    "validate_trajectory_separation",
]

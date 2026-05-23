"""Centralized navigation and safety configuration for AeroMind."""

from __future__ import annotations
from dataclasses import dataclass, field


@dataclass
class NavigationConfig:
    """Single source of truth for all navigation-related tuning parameters."""

    # ── P-controller (goto_local) ──
    control_dt: float = 0.16
    accel_limit: float = 1.35
    kp_xy: float = 0.85
    kp_z: float = 0.75
    damp: float = 0.35
    max_vertical_speed: float = 1.6

    # ── Position hold ──
    hold_tick_s: float = 0.20
    hold_kp: float = 0.35
    hold_max_velocity: float = 0.45

    # ── Default mission parameters ──
    default_altitude_m: float = 8.0
    default_speed_mps: float = 2.0
    default_lookahead_m: float = 3.0
    default_replan_interval_s: float = 1.0
    default_control_dt_s: float = 0.2
    default_obstacle_distance_m: float = 8.0
    default_avoidance_offset_m: float = 6.0

    # ── Arrival thresholds ──
    arrival_distance_m: float = 1.5
    arrival_altitude_error_m: float = 1.2
    arrival_velocity_mps: float = 0.8

    # ── Distance safety ──
    distance_emergency_threshold_m: float = 2.6
    side_emergency_threshold_m: float = 1.6

    # ── Adaptive speed ──
    density_factor_min: float = 0.4
    density_factor_multiplier: float = 2.5
    speed_min_mps: float = 0.3

    # ── Temporal grid / A* planning ──
    grid_half_height: float = 8.0
    alt_slice_half_height: float = 2.0
    alt_offset_m: float = 4.0
    max_waypoints: int = 24
    max_alt_waypoints: int = 12

    # ── Safety bounds ──
    x_bound_m: float = 120.0
    y_bound_m: float = 120.0
    min_altitude_m: float = 1.0
    max_altitude_m: float = 30.0
    min_speed_mps: float = 0.5
    max_speed_mps: float = 4.0

    # ── Recovery ──
    recovery_climb_m: float = 8.0
    recovery_backoff_m: float = 10.0

    # ── Timeouts ──
    mission_prepare_timeout_s: float = 5.0
    takeoff_timeout_s: float = 30.0

    # ── Health check ──
    heartbeat_interval_s: float = 2.0
    heartbeat_failure_threshold: int = 3


# Singleton instance
nav_config = NavigationConfig()

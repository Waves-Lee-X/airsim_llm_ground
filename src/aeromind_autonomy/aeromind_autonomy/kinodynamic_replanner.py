"""Local kinodynamic-style motion primitive replanner."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .local_esdf import LocalEsdfMap
from .minimum_snap import sample_minimum_snap, sample_minimum_snap_boundary


@dataclass
class ReplanResult:
    state: str
    strategy: str
    collision_free: bool
    target_distance: float
    nearest_obstacle: float
    command_velocity: tuple[float, float, float]
    trajectory: list[dict]
    message: str


class KinodynamicReplanner:
    def __init__(
        self,
        safety_radius: float = 1.2,
        max_speed: float = 2.2,
        horizon_sec: float = 2.5,
        sample_count: int = 16,
        start_ignore_radius: float = 0.9,
        arrival_distance: float = 0.6,
        arrival_speed: float = 0.3,
        blocked_enter_clearance: float | None = None,
        blocked_exit_clearance: float | None = None,
        max_acceleration: float = 1.5,
        min_altitude: float = 1.0,
        strategy_switch_penalty: float = 0.8,
    ):
        self.safety_radius = safety_radius
        self.max_speed = max_speed
        self.horizon_sec = horizon_sec
        self.sample_count = sample_count
        self.start_ignore_radius = start_ignore_radius
        self.arrival_distance = arrival_distance
        self.arrival_speed = arrival_speed
        self.max_acceleration = max_acceleration
        self.min_altitude = min_altitude
        self.strategy_switch_penalty = max(0.0, strategy_switch_penalty)
        self._last_tracking_strategy = None
        self.blocked_enter_clearance = (
            blocked_enter_clearance if blocked_enter_clearance is not None else safety_radius
        )
        self.blocked_exit_clearance = (
            blocked_exit_clearance if blocked_exit_clearance is not None else safety_radius * 1.35
        )
        self._blocked_latched = False

    def replan(
        self,
        esdf: LocalEsdfMap,
        position: tuple[float, float, float],
        velocity: tuple[float, float, float],
        goal: tuple[float, float, float] | None,
    ) -> ReplanResult:
        nearest = esdf.nearest_obstacle(position)
        if goal is None:
            self._last_tracking_strategy = None
            trajectory = sample_minimum_snap(
                position, position, self.horizon_sec, self.sample_count
            )
            return ReplanResult(
                state="HOLD",
                strategy="hover_no_goal",
                collision_free=True,
                target_distance=0.0,
                nearest_obstacle=nearest,
                command_velocity=(0.0, 0.0, 0.0),
                trajectory=trajectory,
                message="未设置自主飞行目标，保持悬停",
            )

        to_goal = tuple(goal[i] - position[i] for i in range(3))
        target_distance = math.sqrt(sum(v * v for v in to_goal))
        speed_norm = math.sqrt(sum(v * v for v in velocity))
        if target_distance < self.arrival_distance and speed_norm < self.arrival_speed:
            trajectory = sample_minimum_snap(
                position, goal, self.horizon_sec, self.sample_count
            )
            self._blocked_latched = False
            self._last_tracking_strategy = None
            return ReplanResult(
                state="ARRIVED",
                strategy="goal_reached",
                collision_free=True,
                target_distance=target_distance,
                nearest_obstacle=nearest,
                command_velocity=(0.0, 0.0, 0.0),
                trajectory=trajectory,
                message="已到达局部目标附近",
            )

        candidates = self._motion_primitives(to_goal, velocity)
        evaluated = []
        for name, candidate_velocity in candidates:
            end = tuple(position[i] + candidate_velocity[i] * self.horizon_sec for i in range(3))
            if end[2] < self.min_altitude:
                continue
            trajectory = sample_minimum_snap_boundary(
                position,
                end,
                self.horizon_sec,
                self.sample_count,
                start_velocity=velocity,
                end_velocity=candidate_velocity,
            )
            points = [
                sample["position"]
                for sample in trajectory
                if math.dist(sample["position"], position) >= self.start_ignore_radius
            ]
            if not points:
                points = [trajectory[-1]["position"]]
            clearance = esdf.trajectory_clearance(
                points,
                max_radius=max(3.0, self.safety_radius * 2.5),
            )
            progress = target_distance - math.dist(end, goal)
            effort = math.dist(candidate_velocity, velocity)
            alignment = self._alignment(candidate_velocity, to_goal)
            score = (
                progress * 3.0
                + clearance * 1.8
                + alignment * 0.8
                - effort * 0.4
            )
            if (
                self._last_tracking_strategy is not None
                and name != self._last_tracking_strategy
            ):
                score -= self.strategy_switch_penalty
            safe = clearance >= self.safety_radius
            if not safe:
                score -= (self.safety_radius - clearance) * 10.0
            entry = (score, safe, name, candidate_velocity, trajectory, clearance)
            evaluated.append(entry)

        safe_candidates = [entry for entry in evaluated if entry[1]]
        best = max(safe_candidates or evaluated, key=lambda entry: entry[0], default=None)

        if best is None:
            trajectory = sample_minimum_snap(
                position, position, self.horizon_sec, self.sample_count
            )
            return ReplanResult(
                state="HOLD",
                strategy="no_candidate",
                collision_free=False,
                target_distance=target_distance,
                nearest_obstacle=nearest,
                command_velocity=(0.0, 0.0, 0.0),
                trajectory=trajectory,
                message="未找到可行运动基元，悬停等待重规划",
            )

        _, safe, strategy, command_velocity, trajectory, clearance = best
        if clearance < self.blocked_enter_clearance:
            self._blocked_latched = True
        elif clearance >= self.blocked_exit_clearance:
            self._blocked_latched = False

        if not safe or self._blocked_latched:
            self._last_tracking_strategy = None
            command_velocity = (0.0, 0.0, 0.0)
            trajectory = sample_minimum_snap(
                position, position, self.horizon_sec, self.sample_count
            )
            if self._blocked_latched and clearance >= self.safety_radius:
                reason = (
                    f"轨迹 clearance={clearance:.2f} m，仍未达到脱离阻塞阈值 "
                    f"{self.blocked_exit_clearance:.2f} m"
                )
            else:
                reason = f"轨迹最小安全距离 {clearance:.2f} m，小于安全半径"
            return ReplanResult(
                state="BLOCKED",
                strategy="safe_hover",
                collision_free=False,
                target_distance=target_distance,
                nearest_obstacle=nearest,
                command_velocity=command_velocity,
                trajectory=trajectory,
                message=f"{reason}，悬停",
            )

        self._last_tracking_strategy = strategy
        return ReplanResult(
            state="TRACKING",
            strategy=strategy,
            collision_free=True,
            target_distance=target_distance,
            nearest_obstacle=nearest,
            command_velocity=command_velocity,
            trajectory=trajectory,
            message=f"实时重规划完成：{strategy}，轨迹 clearance={clearance:.2f} m",
        )

    def _motion_primitives(
        self,
        to_goal: tuple[float, float, float],
        velocity: tuple[float, float, float],
    ):
        horizontal = math.hypot(to_goal[0], to_goal[1])
        if horizontal > 1e-6:
            forward = (to_goal[0] / horizontal, to_goal[1] / horizontal)
        else:
            forward = (1.0, 0.0)
        distance = math.sqrt(sum(value * value for value in to_goal))
        speed = min(self.max_speed, max(0.35, distance / self.horizon_sec))
        vertical_goal = max(-0.8, min(0.8, to_goal[2] / self.horizon_sec))
        definitions = [
            ("direct_goal", 0.0, 1.0, vertical_goal),
            ("slow_goal", 0.0, 0.55, vertical_goal),
            ("left_30", 30.0, 0.75, vertical_goal),
            ("right_30", -30.0, 0.75, vertical_goal),
            ("left_replan", 55.0, 0.65, vertical_goal),
            ("right_replan", -55.0, 0.65, vertical_goal),
            ("left_75", 75.0, 0.55, vertical_goal),
            ("right_75", -75.0, 0.55, vertical_goal),
            ("climb_replan", 0.0, 0.55, max(0.8, vertical_goal)),
            ("descend_replan", 0.0, 0.55, min(-0.5, vertical_goal)),
        ]
        candidates = []
        for name, angle_deg, scale, vz in definitions:
            angle = math.radians(angle_deg)
            direction = (
                forward[0] * math.cos(angle) - forward[1] * math.sin(angle),
                forward[0] * math.sin(angle) + forward[1] * math.cos(angle),
            )
            desired = (direction[0] * speed * scale, direction[1] * speed * scale, vz)
            candidates.append((name, self._limit_acceleration(velocity, desired)))
        return candidates

    def _limit_acceleration(self, current, desired):
        delta = tuple(desired[i] - current[i] for i in range(3))
        norm = math.sqrt(sum(value * value for value in delta))
        limit = max(0.1, self.max_acceleration) * self.horizon_sec
        if norm <= limit or norm <= 1e-9:
            return desired
        scale = limit / norm
        return tuple(current[i] + delta[i] * scale for i in range(3))

    @staticmethod
    def _alignment(velocity, to_goal):
        velocity_norm = math.sqrt(sum(value * value for value in velocity))
        goal_norm = math.sqrt(sum(value * value for value in to_goal))
        if velocity_norm <= 1e-9 or goal_norm <= 1e-9:
            return 0.0
        return sum(velocity[i] * to_goal[i] for i in range(3)) / (
            velocity_norm * goal_norm
        )

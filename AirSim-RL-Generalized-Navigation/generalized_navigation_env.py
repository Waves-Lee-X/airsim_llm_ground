from __future__ import annotations

from dataclasses import dataclass

import airsim
import numpy as np

from UAV_Autonomous_navigation import AirSimDroneEnv


@dataclass
class TrainingArea:
    x_min: float = -60.0
    x_max: float = 80.0
    y_min: float = -50.0
    y_max: float = 50.0
    z: float = -2.0
    min_start_goal_distance: float = 20.0
    max_start_goal_distance: float = 120.0


class GeneralizedAirSimDroneEnv(AirSimDroneEnv):
    """AirSim navigation environment with randomized start and goal per episode.

    The original project mostly trained around a fixed goal. This environment
    changes the task distribution: each episode samples a new local-NED start
    and a new local-NED target, so the policy must learn a reusable navigation
    behavior instead of memorizing a single route.
    """

    def __init__(
        self,
        area: TrainingArea | None = None,
        max_episode_steps: int = 400,
        success_radius_m: float = 1.5,
        target_altitude: float = -2.0,
    ) -> None:
        self.area = area or TrainingArea(z=target_altitude)
        self.max_episode_steps = int(max_episode_steps)
        self.success_radius_m = float(success_radius_m)
        self.episode_step = 0
        self.start_position = np.array([0.0, 0.0, self.area.z], dtype=np.float32)
        super().__init__()
        self.fixed_altitude = float(self.area.z)
        self.target_position = self.sample_target_position()

    def sample_xy_pair(self) -> tuple[np.ndarray, np.ndarray]:
        for _ in range(200):
            start = np.array(
                [
                    np.random.uniform(self.area.x_min, self.area.x_max),
                    np.random.uniform(self.area.y_min, self.area.y_max),
                    self.area.z,
                ],
                dtype=np.float32,
            )
            goal = np.array(
                [
                    np.random.uniform(self.area.x_min, self.area.x_max),
                    np.random.uniform(self.area.y_min, self.area.y_max),
                    self.area.z,
                ],
                dtype=np.float32,
            )
            distance = np.linalg.norm(goal[:2] - start[:2])
            if self.area.min_start_goal_distance <= distance <= self.area.max_start_goal_distance:
                return start, goal
        return start, goal

    def sample_target_position(self) -> np.ndarray:
        _start, goal = self.sample_xy_pair()
        return goal.astype(np.float32)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.episode_step = 0
        self.start_position, self.target_position = self.sample_xy_pair()
        self._place_drone(self.start_position)
        self.obstacle_velocity = np.array([0.0, 0.0])
        return self._build_state(), {
            "start": self.start_position.copy(),
            "target": self.target_position.copy(),
        }

    def step(self, action):
        self.episode_step += 1
        prev_position = self._current_position()
        prev_distance = np.linalg.norm(self.target_position[:2] - prev_position[:2])

        action = np.nan_to_num(np.asarray(action, dtype=np.float32), nan=0.0, posinf=0.0, neginf=0.0)
        vx = float(np.clip(action[0], -2.0, 2.0))
        vy = float(np.clip(action[1], -2.0, 2.0))
        self.client.moveByVelocityAsync(vx, vy, 0.0, 1.0, vehicle_name="Drone1").join()

        state = self._build_state()
        current_position = state[:3]
        current_distance = np.linalg.norm(self.target_position[:2] - current_position[:2])
        front_distance = float(state[6])
        movement = np.array([vx, vy], dtype=np.float32)

        progress_reward = (prev_distance - current_distance) * 8.0
        distance_penalty = -0.05 * current_distance
        speed_penalty = -0.02 * float(np.linalg.norm(movement))
        near_obstacle_penalty = -8.0 if front_distance < 2.0 else 0.0
        danger_penalty = -25.0 if front_distance < 1.0 else 0.0
        reward = progress_reward + distance_penalty + speed_penalty + near_obstacle_penalty + danger_penalty

        collision = self.client.simGetCollisionInfo(vehicle_name="Drone1").has_collided
        terminated = current_distance <= self.success_radius_m
        truncated = self.episode_step >= self.max_episode_steps or bool(collision)
        if terminated:
            reward += 120.0
        if collision:
            reward -= 120.0

        info = {
            "start": self.start_position.copy(),
            "target": self.target_position.copy(),
            "distance_to_target": float(current_distance),
            "collision_count": int(bool(collision)),
            "log": "Avoiding obstacle" if front_distance < 3.0 else "",
            "success": int(terminated),
        }
        return state, float(reward), bool(terminated), bool(truncated), info

    def _place_drone(self, position: np.ndarray) -> None:
        self.client.enableApiControl(True, vehicle_name="Drone1")
        self.client.armDisarm(True, vehicle_name="Drone1")
        pose = airsim.Pose(
            airsim.Vector3r(float(position[0]), float(position[1]), float(position[2])),
            airsim.to_quaternion(0.0, 0.0, 0.0),
        )
        self.client.simSetVehiclePose(pose, True, vehicle_name="Drone1")
        self.client.hoverAsync(vehicle_name="Drone1").join()

    def _current_position(self) -> np.ndarray:
        drone_state = self.client.getMultirotorState(vehicle_name="Drone1").kinematics_estimated.position
        return np.array([drone_state.x_val, drone_state.y_val, self.fixed_altitude], dtype=np.float32)

    def _build_state(self) -> np.ndarray:
        current_pos = self._current_position()
        front_distance, left_free_space, right_free_space, left_edge, right_edge = self.get_lidar_data()
        if np.isnan(front_distance):
            front_distance = 10.0
        if np.isnan(left_free_space):
            left_free_space = 0.5
        if np.isnan(right_free_space):
            right_free_space = -0.5
        if np.isnan(left_edge):
            left_edge = 0.0
        if np.isnan(right_edge):
            right_edge = 0.0
        obstacle_position = np.array([current_pos[0] + front_distance, current_pos[1]])
        self.predict_obstacle_position(obstacle_position)
        if np.isnan(self.obstacle_velocity).any():
            self.obstacle_velocity = np.array([0.0, 0.0])
        return np.concatenate(
            [
                current_pos,
                self.target_position,
                [front_distance, left_free_space, right_free_space],
                self.obstacle_velocity.flatten(),
                [left_edge, right_edge],
            ]
        ).astype(np.float32)

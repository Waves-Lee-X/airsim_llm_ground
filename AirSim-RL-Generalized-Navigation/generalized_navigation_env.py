from __future__ import annotations

from dataclasses import dataclass

import airsim
import numpy as np
from gymnasium import spaces

from UAV_Autonomous_navigation import AirSimDroneEnv


@dataclass
class TrainingArea:
    x_min: float = -60.0
    x_max: float = 80.0
    y_min: float = -50.0
    y_max: float = 50.0
    z_min: float = -8.0
    z_max: float = -2.0
    min_start_goal_distance: float = 20.0
    max_start_goal_distance: float = 120.0

    @classmethod
    def from_fixed_altitude(cls, z: float, **kwargs) -> "TrainingArea":
        return cls(z_min=float(z), z_max=float(z), **kwargs)


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
        lidar_mode: str = "3d",
    ) -> None:
        if lidar_mode not in {"basic", "3d"}:
            raise ValueError("lidar_mode must be 'basic' or '3d'")
        self.area = area or TrainingArea.from_fixed_altitude(target_altitude)
        self.lidar_mode = lidar_mode
        self.max_episode_steps = int(max_episode_steps)
        self.success_radius_m = float(success_radius_m)
        self.episode_step = 0
        self.start_position = np.array([0.0, 0.0, self._sample_z()], dtype=np.float32)
        super().__init__()
        state_dim = 13 if self.lidar_mode == "basic" else 16
        self.observation_space = spaces.Box(low=-100, high=100, shape=(state_dim,), dtype=np.float32)
        self.target_position = self.sample_target_position()

    def _sample_z(self) -> float:
        if np.isclose(self.area.z_min, self.area.z_max):
            return float(self.area.z_min)
        return float(np.random.uniform(self.area.z_min, self.area.z_max))

    def sample_xyz_pair(self) -> tuple[np.ndarray, np.ndarray]:
        for _ in range(200):
            start = np.array(
                [
                    np.random.uniform(self.area.x_min, self.area.x_max),
                    np.random.uniform(self.area.y_min, self.area.y_max),
                    self._sample_z(),
                ],
                dtype=np.float32,
            )
            goal = np.array(
                [
                    np.random.uniform(self.area.x_min, self.area.x_max),
                    np.random.uniform(self.area.y_min, self.area.y_max),
                    self._sample_z(),
                ],
                dtype=np.float32,
            )
            distance = np.linalg.norm(goal - start)
            if self.area.min_start_goal_distance <= distance <= self.area.max_start_goal_distance:
                return start, goal
        return start, goal

    def sample_target_position(self) -> np.ndarray:
        _start, goal = self.sample_xyz_pair()
        return goal.astype(np.float32)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.episode_step = 0
        self.start_position, self.target_position = self.sample_xyz_pair()
        self._place_drone(self.start_position)
        self.obstacle_velocity = np.array([0.0, 0.0])
        return self._build_state(), {
            "start": self.start_position.copy(),
            "target": self.target_position.copy(),
        }

    def step(self, action):
        self.episode_step += 1
        prev_position = self._current_position()
        prev_distance = np.linalg.norm(self.target_position - prev_position)

        action = np.nan_to_num(np.asarray(action, dtype=np.float32), nan=0.0, posinf=0.0, neginf=0.0)
        vx = float(np.clip(action[0], -2.0, 2.0))
        vy = float(np.clip(action[1], -2.0, 2.0))
        vz = float(np.clip(action[2], -2.0, 2.0))
        if self.lidar_mode == "basic" and np.isclose(self.area.z_min, self.area.z_max):
            vz = 0.0
        self.client.moveByVelocityAsync(vx, vy, vz, 1.0, vehicle_name="Drone1").join()

        state = self._build_state()
        current_position = state[:3]
        current_distance = np.linalg.norm(self.target_position - current_position)
        front_distance = float(state[6])
        up_distance = float(state[10]) if self.lidar_mode == "3d" else 10.0
        down_distance = float(state[11]) if self.lidar_mode == "3d" else 10.0
        movement = np.array([vx, vy, vz], dtype=np.float32)

        progress_reward = (prev_distance - current_distance) * 8.0
        distance_penalty = -0.05 * current_distance
        speed_penalty = -0.02 * float(np.linalg.norm(movement))
        altitude_violation = max(0.0, current_position[2] - self.area.z_max) + max(0.0, self.area.z_min - current_position[2])
        altitude_penalty = -5.0 * float(altitude_violation)
        near_obstacle_penalty = -8.0 if front_distance < 2.0 else 0.0
        vertical_obstacle_penalty = -8.0 if (vz < -0.1 and up_distance < 2.0) or (vz > 0.1 and down_distance < 2.0) else 0.0
        danger_penalty = -25.0 if front_distance < 1.0 else 0.0
        reward = (
            progress_reward
            + distance_penalty
            + speed_penalty
            + altitude_penalty
            + near_obstacle_penalty
            + vertical_obstacle_penalty
            + danger_penalty
        )

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
            "altitude_error": float(abs(self.target_position[2] - current_position[2])),
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
        return np.array([drone_state.x_val, drone_state.y_val, drone_state.z_val], dtype=np.float32)

    def _build_state(self) -> np.ndarray:
        current_pos = self._current_position()
        lidar_features = self._get_lidar_basic_features() if self.lidar_mode == "basic" else self._get_lidar_3d_features()
        front_distance = lidar_features["front"]
        left_free_space = lidar_features["left"]
        right_free_space = lidar_features["right"]
        left_edge = lidar_features["left_edge"]
        right_edge = lidar_features["right_edge"]
        obstacle_position = np.array([current_pos[0] + front_distance, current_pos[1]])
        self.predict_obstacle_position(obstacle_position)
        if np.isnan(self.obstacle_velocity).any():
            self.obstacle_velocity = np.array([0.0, 0.0])
        lidar_state = [
            lidar_features["front"],
            lidar_features["left"],
            lidar_features["right"],
        ]
        if self.lidar_mode == "3d":
            lidar_state.extend(
                [
                    lidar_features["back"],
                    lidar_features["up"],
                    lidar_features["down"],
                ]
            )

        return np.concatenate(
            [
                current_pos,
                self.target_position,
                lidar_state,
                self.obstacle_velocity.flatten(),
                [left_edge, right_edge],
            ]
        ).astype(np.float32)

    def _get_lidar_basic_features(self) -> dict[str, float]:
        front_distance, left_free_space, right_free_space, left_edge, right_edge = self.get_lidar_data()
        values = {
            "front": front_distance,
            "left": left_free_space,
            "right": right_free_space,
            "left_edge": left_edge,
            "right_edge": right_edge,
        }
        defaults = {
            "front": 10.0,
            "left": 0.5,
            "right": -0.5,
            "left_edge": 0.0,
            "right_edge": 0.0,
        }
        for key, default in defaults.items():
            if np.isnan(values[key]):
                values[key] = default
        return values

    def _get_lidar_3d_features(self) -> dict[str, float]:
        defaults = {
            "front": 10.0,
            "left": 0.5,
            "right": -0.5,
            "back": 10.0,
            "up": 10.0,
            "down": 10.0,
            "left_edge": 0.0,
            "right_edge": 0.0,
        }
        try:
            lidar_data = self.client.getLidarData(lidar_name="LidarSensor1", vehicle_name="Drone1")
            if len(lidar_data.point_cloud) < 3:
                return defaults
            points = np.array(lidar_data.point_cloud, dtype=np.float32).reshape(-1, 3)
        except Exception:
            return defaults

        points = points[np.all(np.isfinite(points), axis=1)]
        if len(points) == 0:
            return defaults

        front = points[(points[:, 0] > 0.1) & (np.abs(points[:, 1]) < 0.6) & (np.abs(points[:, 2]) < 0.8)]
        back = points[(points[:, 0] < -0.1) & (np.abs(points[:, 1]) < 0.8) & (np.abs(points[:, 2]) < 0.8)]
        left = points[(points[:, 1] > 0.1) & (np.abs(points[:, 0]) < 1.5) & (np.abs(points[:, 2]) < 0.8)]
        right = points[(points[:, 1] < -0.1) & (np.abs(points[:, 0]) < 1.5) & (np.abs(points[:, 2]) < 0.8)]
        up = points[(points[:, 2] < -0.1) & (np.abs(points[:, 0]) < 1.0) & (np.abs(points[:, 1]) < 1.0)]
        down = points[(points[:, 2] > 0.1) & (np.abs(points[:, 0]) < 1.0) & (np.abs(points[:, 1]) < 1.0)]

        features = defaults.copy()
        if len(front) > 0:
            features["front"] = float(np.clip(np.min(front[:, 0]), 0.1, 100.0))
            features["left_edge"] = float(np.min(front[:, 1]))
            features["right_edge"] = float(np.max(front[:, 1]))
        if len(back) > 0:
            features["back"] = float(np.clip(abs(np.max(back[:, 0])), 0.1, 100.0))
        if len(left) > 0:
            features["left"] = float(np.clip(np.min(left[:, 1]), 0.1, 10.0))
        if len(right) > 0:
            features["right"] = float(np.clip(np.max(right[:, 1]), -10.0, -0.1))
        if len(up) > 0:
            features["up"] = float(np.clip(abs(np.max(up[:, 2])), 0.1, 100.0))
        if len(down) > 0:
            features["down"] = float(np.clip(np.min(down[:, 2]), 0.1, 100.0))
        return features

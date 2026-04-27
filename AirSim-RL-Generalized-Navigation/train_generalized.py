from __future__ import annotations

import argparse
import csv
import os
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import torch

from generalized_navigation_env import GeneralizedAirSimDroneEnv, TrainingArea
from UAV_Autonomous_navigation import SACAgent
from visualization_utils import generate_training_visualizations


class ThreeDExpertPolicy:
    def expert_action(self, state):
        current_pos = np.asarray(state[:3], dtype=np.float32)
        target_pos = np.asarray(state[3:6], dtype=np.float32)
        front_distance = float(state[6])
        left_free_space = float(state[7])
        right_free_space = float(state[8])
        up_distance = float(state[10])
        down_distance = float(state[11])

        direction = target_pos - current_pos
        norm = float(np.linalg.norm(direction))
        if norm > 1e-6:
            direction = direction / norm
        action = direction * 2.0

        if front_distance < 2.0:
            side_step = -1.5 if abs(right_free_space) > left_free_space else 1.5
            action[0] = min(action[0], 0.5)
            action[1] = side_step
        elif front_distance < 3.0:
            side_step = -1.0 if abs(right_free_space) > left_free_space else 1.0
            action[0] = min(action[0], 1.0)
            action[1] += side_step

        if action[2] < -0.1 and up_distance < 2.0:
            action[2] = 0.0
        elif action[2] > 0.1 and down_distance < 2.0:
            action[2] = 0.0

        return np.clip(action, -2.0, 2.0)


class GeneralizedSACAgent(SACAgent):
    def __init__(self, *args, expert_weight: float = 0.2, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.expert_weight = float(np.clip(expert_weight, 0.0, 1.0))
        self.expert_action = ThreeDExpertPolicy()

    def select_action(self, state):
        if np.isnan(state).any():
            return np.array([1.0, 0.0, 0.0])
        state_tensor = torch.tensor(state, dtype=torch.float32, device=self.device).unsqueeze(0)
        try:
            sac_action, _ = self.actor.sample(state_tensor)
            sac_action = sac_action.detach().cpu().numpy().flatten()
            if self.expert_weight <= 0.0:
                action = sac_action
            else:
                expert_action = np.asarray(self.expert_action.expert_action(state), dtype=np.float32)
                action = self.expert_weight * expert_action + (1.0 - self.expert_weight) * sac_action
            return np.clip(action, -1.0, 1.0)
        except Exception as exc:
            print(f"select_action failed: {exc}")
            return np.array([1.0, 0.0, 0.0])


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Train SAC navigation with randomized starts and goals.")
    parser.add_argument("--episodes", type=int, default=300)
    parser.add_argument("--max-steps", type=int, default=400)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--train-start", type=int, default=1000)
    parser.add_argument("--save-every", type=int, default=50)
    parser.add_argument("--model-in", default="", help="Optional checkpoint to warm-start from.")
    parser.add_argument("--run-dir", default="", help="Output directory. Default: runs/<timestamp>.")
    parser.add_argument("--model-out", default="", help="Final checkpoint path. Default: <run-dir>/sac_model_generalized_final_<timestamp>.pth")
    parser.add_argument("--csv", default="", help="Training CSV path. Default: <run-dir>/training_data_generalized_<timestamp>.csv")
    parser.add_argument("--expert-weight", type=float, default=0.2, help="0.2 matches original project; 0.0 is pure SAC.")
    parser.add_argument("--x-min", type=float, default=-60.0)
    parser.add_argument("--x-max", type=float, default=80.0)
    parser.add_argument("--y-min", type=float, default=-50.0)
    parser.add_argument("--y-max", type=float, default=50.0)
    parser.add_argument("--z", type=float, default=None, help="Optional fixed NED altitude for backward-compatible 2D-style training.")
    parser.add_argument("--z-min", type=float, default=-8.0)
    parser.add_argument("--z-max", type=float, default=-2.0)
    parser.add_argument("--min-distance", type=float, default=20.0)
    parser.add_argument("--max-distance", type=float, default=120.0)
    return parser.parse_args()


def prepare_output_paths(args: argparse.Namespace) -> tuple[str, Path, Path, Path, Path]:
    run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = Path(args.run_dir) if args.run_dir else Path("runs") / run_id
    run_dir.mkdir(parents=True, exist_ok=True)
    csv_path = Path(args.csv) if args.csv else run_dir / f"training_data_generalized_{run_id}.csv"
    model_out = Path(args.model_out) if args.model_out else run_dir / f"sac_model_generalized_final_{run_id}.pth"
    latest_model = run_dir / "sac_model_generalized_latest.pth"
    return run_id, run_dir, csv_path, model_out, latest_model


def main() -> None:
    args = parse_args()
    run_id, run_dir, csv_path, model_out, latest_model = prepare_output_paths(args)
    print(f"Run id: {run_id}")
    print(f"Output directory: {run_dir}")
    print(f"Training CSV: {csv_path}")
    print(f"Latest checkpoint: {latest_model}")
    area = TrainingArea(
        x_min=args.x_min,
        x_max=args.x_max,
        y_min=args.y_min,
        y_max=args.y_max,
        z_min=args.z if args.z is not None else args.z_min,
        z_max=args.z if args.z is not None else args.z_max,
        min_start_goal_distance=args.min_distance,
        max_start_goal_distance=args.max_distance,
    )
    env = GeneralizedAirSimDroneEnv(area=area, max_episode_steps=args.max_steps)
    agent = GeneralizedSACAgent(state_dim=env.observation_space.shape[0], action_dim=env.action_space.shape[0], expert_weight=args.expert_weight)

    if args.model_in and os.path.exists(args.model_in):
        print(f"Loading checkpoint: {args.model_in}")
        agent.load(args.model_in)

    with open(csv_path, mode="w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(
            [
                "Episode",
                "Start_X",
                "Start_Y",
                "Start_Z",
                "Target_X",
                "Target_Y",
                "Target_Z",
                "Total_Reward",
                "Collisions",
                "Steps",
                "Navigation_Time",
                "Path_Length",
                "Obstacle_Avoidance_Count",
                "Final_Distance",
                "Success",
            ]
        )

    completed_episode = 0
    try:
        for episode in range(args.episodes):
            state, reset_info = env.reset()
            total_reward = 0.0
            collisions = 0
            steps = 0
            path_length = 0.0
            obstacle_avoidance_count = 0
            success = 0
            final_distance = float("inf")
            prev_pos = state[:3]
            start_time = time.time()

            for _ in range(args.max_steps):
                action = agent.select_action(state)
                next_state, reward, terminated, truncated, info = env.step(action)

                current_pos = next_state[:3]
                path_length += float(np.linalg.norm(current_pos - prev_pos))
                prev_pos = current_pos
                total_reward += float(reward)
                steps += 1
                collisions += int(info.get("collision_count", 0))
                final_distance = float(info.get("distance_to_target", final_distance))
                if "Avoiding obstacle" in info.get("log", ""):
                    obstacle_avoidance_count += 1

                done = terminated or truncated
                agent.replay_buffer.push(state, action, reward, next_state, done)
                state = next_state

                if len(agent.replay_buffer) > args.train_start:
                    agent.train(args.batch_size)

                if done:
                    success = int(terminated)
                    break

            completed_episode = episode + 1
            navigation_time = time.time() - start_time
            start = reset_info["start"]
            target = reset_info["target"]
            with open(csv_path, mode="a", newline="") as file:
                writer = csv.writer(file)
                writer.writerow(
                    [
                        completed_episode,
                        float(start[0]),
                        float(start[1]),
                        float(start[2]),
                        float(target[0]),
                        float(target[1]),
                        float(target[2]),
                        total_reward,
                        collisions,
                        steps,
                        navigation_time,
                        path_length,
                        obstacle_avoidance_count,
                        final_distance,
                        success,
                    ]
                )

            print(
                f"Episode {completed_episode}/{args.episodes}: "
                f"start=({start[0]:.1f},{start[1]:.1f},{start[2]:.1f}) "
                f"target=({target[0]:.1f},{target[1]:.1f},{target[2]:.1f}) "
                f"reward={total_reward:.1f}, steps={steps}, final={final_distance:.2f}m, "
                f"success={success}, collisions={collisions}, time={navigation_time:.1f}s"
            )

            if completed_episode % args.save_every == 0:
                checkpoint = run_dir / f"sac_model_generalized_ep{completed_episode}_{run_id}.pth"
                agent.save(str(checkpoint))
                agent.save(str(latest_model))
                print(f"Saved checkpoint: {checkpoint}")
                print(f"Updated latest checkpoint: {latest_model}")

    except KeyboardInterrupt:
        interrupted_model = run_dir / f"sac_model_generalized_interrupted_ep{completed_episode}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.pth"
        agent.save(str(interrupted_model))
        agent.save(str(latest_model))
        print("\nTraining interrupted by user.")
        print(f"Saved interrupted checkpoint: {interrupted_model}")
        print(f"Updated latest checkpoint: {latest_model}")
    else:
        agent.save(str(model_out))
        agent.save(str(latest_model))
        print(f"Training finished. Final model saved to {model_out}")
        print(f"Updated latest checkpoint: {latest_model}")
    finally:
        try:
            generate_training_visualizations(str(csv_path), output_dir=str(run_dir / "visualization_generalized"))
        except Exception as exc:
            print(f"Visualization generation skipped: {exc}")


if __name__ == "__main__":
    main()

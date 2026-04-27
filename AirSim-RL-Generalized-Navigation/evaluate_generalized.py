from __future__ import annotations

import argparse
import csv
import time
from datetime import datetime
from pathlib import Path

import numpy as np

from generalized_navigation_env import GeneralizedAirSimDroneEnv, TrainingArea
from train_generalized import GeneralizedSACAgent


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Evaluate a generalized AirSim SAC navigation model.")
    parser.add_argument("--model", default="sac_model_generalized_final.pth")
    parser.add_argument("--episodes", type=int, default=50)
    parser.add_argument("--max-steps", type=int, default=400)
    parser.add_argument("--run-dir", default="", help="Evaluation output directory. Default: evals/<timestamp>.")
    parser.add_argument("--csv", default="", help="Evaluation CSV path. Default: <run-dir>/evaluation_generalized_<timestamp>.csv")
    parser.add_argument("--expert-weight", type=float, default=0.0, help="Usually 0.0 for evaluating the learned policy itself.")
    parser.add_argument("--x-min", type=float, default=-60.0)
    parser.add_argument("--x-max", type=float, default=80.0)
    parser.add_argument("--y-min", type=float, default=-50.0)
    parser.add_argument("--y-max", type=float, default=50.0)
    parser.add_argument("--z", type=float, default=None, help="Optional fixed NED altitude for backward-compatible 2D-style evaluation.")
    parser.add_argument("--z-min", type=float, default=-8.0)
    parser.add_argument("--z-max", type=float, default=-2.0)
    parser.add_argument("--lidar-mode", choices=["basic", "3d"], default="3d", help="basic keeps the old 13D state; 3d uses six-direction LiDAR and a 16D state.")
    parser.add_argument("--min-distance", type=float, default=20.0)
    parser.add_argument("--max-distance", type=float, default=120.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = Path(args.run_dir) if args.run_dir else Path("evals") / run_id
    run_dir.mkdir(parents=True, exist_ok=True)
    csv_path = Path(args.csv) if args.csv else run_dir / f"evaluation_generalized_{run_id}.csv"
    print(f"Evaluation id: {run_id}")
    print(f"Model: {args.model}")
    print(f"Evaluation CSV: {csv_path}")
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
    env = GeneralizedAirSimDroneEnv(area=area, max_episode_steps=args.max_steps, lidar_mode=args.lidar_mode)
    agent = GeneralizedSACAgent(state_dim=env.observation_space.shape[0], action_dim=env.action_space.shape[0], expert_weight=args.expert_weight)
    agent.load(args.model)

    rows = []
    for episode in range(args.episodes):
        state, reset_info = env.reset()
        path_length = 0.0
        collisions = 0
        final_distance = float("inf")
        success = 0
        prev_pos = state[:3]
        start_time = time.time()
        steps = 0
        for _ in range(args.max_steps):
            action = agent.select_action(state)
            next_state, _reward, terminated, truncated, info = env.step(action)
            current_pos = next_state[:3]
            path_length += float(np.linalg.norm(current_pos - prev_pos))
            prev_pos = current_pos
            collisions += int(info.get("collision_count", 0))
            final_distance = float(info.get("distance_to_target", final_distance))
            state = next_state
            steps += 1
            if terminated or truncated:
                success = int(terminated)
                break

        rows.append(
            {
                "Episode": episode + 1,
                "Start_X": float(reset_info["start"][0]),
                "Start_Y": float(reset_info["start"][1]),
                "Start_Z": float(reset_info["start"][2]),
                "Target_X": float(reset_info["target"][0]),
                "Target_Y": float(reset_info["target"][1]),
                "Target_Z": float(reset_info["target"][2]),
                "Success": success,
                "Final_Distance": final_distance,
                "Collisions": collisions,
                "Steps": steps,
                "Navigation_Time": time.time() - start_time,
                "Path_Length": path_length,
            }
        )
        print(
            f"Eval {episode + 1}/{args.episodes}: success={success}, "
            f"final={final_distance:.2f}m, collisions={collisions}, steps={steps}"
        )

    with open(csv_path, mode="w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    success_rate = sum(row["Success"] for row in rows) / len(rows)
    mean_final_distance = sum(row["Final_Distance"] for row in rows) / len(rows)
    mean_path_length = sum(row["Path_Length"] for row in rows) / len(rows)
    total_collisions = sum(row["Collisions"] for row in rows)
    print("\nEvaluation summary")
    print(f"Success rate: {success_rate:.2%}")
    print(f"Mean final distance: {mean_final_distance:.2f} m")
    print(f"Mean path length: {mean_path_length:.2f} m")
    print(f"Total collisions: {total_collisions}")
    print(f"Saved CSV: {csv_path}")


if __name__ == "__main__":
    main()

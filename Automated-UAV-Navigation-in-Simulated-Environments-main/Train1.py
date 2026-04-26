import csv
import os
import time

import gymnasium as gym
import numpy as np

from UAV_Autonomous_navigation import AirSimDroneEnv, SACAgent
from visualization_utils import generate_training_visualizations, zh


# Initialize environment and agent.
env = AirSimDroneEnv()
agent = SACAgent(state_dim=13, action_dim=3)
model_path = "sac_model_final.pth"
if os.path.exists(model_path):
    print(zh(r"\u6b63\u5728\u52a0\u8f7d\u73b0\u6709\u6a21\u578b:"), model_path)
    agent.load(model_path)
else:
    print(zh(r"\u672a\u627e\u5230\u73b0\u6709\u6a21\u578b\u3002\u4ece\u5934\u5f00\u59cb\u8bad\u7ec3\u3002"))

# Allow the user to choose the target position.
print(zh(r"\n\u8bf7\u9009\u62e9\u76ee\u6807\u4f4d\u7f6e\u8bbe\u7f6e:"))
print(zh(r"1. \u4f7f\u7528\u9ed8\u8ba4\u76ee\u6807\u4f4d\u7f6e (71.7, 11.6, -2.0)"))
print(zh(r"2. \u81ea\u5b9a\u4e49\u76ee\u6807\u4f4d\u7f6e"))
choice = input(zh(r"\u8bf7\u8f93\u5165\u9009\u62e9 (1/2): "))

if choice == "2":
    try:
        target_x = float(input(zh(r"\u8bf7\u8f93\u5165\u76ee\u6807\u4f4d\u7f6e X \u5750\u6807: ")))
        target_y = float(input(zh(r"\u8bf7\u8f93\u5165\u76ee\u6807\u4f4d\u7f6e Y \u5750\u6807: ")))
        target_z = -2.0
        env.set_target_position([target_x, target_y, target_z])
    except ValueError:
        print(zh(r"\u8f93\u5165\u65e0\u6548\uff0c\u4f7f\u7528\u9ed8\u8ba4\u76ee\u6807\u4f4d\u7f6e\u3002"))

num_episodes = 50
batch_size = 64
train_start = 1000

csv_filename = "training_data.csv"
with open(csv_filename, mode="w", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(
        [
            "Episode",
            "Total_Reward",
            "Collisions",
            "Steps",
            "Navigation_Time",
            "Path_Length",
            "Obstacle_Avoidance_Count",
            "Success",
        ]
    )

for episode in range(num_episodes):
    state, _ = env.reset()
    total_reward = 0
    collisions = 0
    steps = 0
    start_time = time.time()
    path_length = 0
    prev_pos = state[:3]
    obstacle_avoidance_count = 0
    success = 0

    for t in range(500):
        action = agent.select_action(state)
        next_state, reward, terminated, truncated, info = env.step(action)

        current_pos = next_state[:3]
        path_length += np.linalg.norm(current_pos - prev_pos)
        prev_pos = current_pos

        if "Avoiding obstacle" in info.get("log", "") or "Backing up" in info.get("log", ""):
            obstacle_avoidance_count += 1

        done = terminated or truncated
        agent.replay_buffer.push(state, action, reward, next_state, done)

        state = next_state
        total_reward += reward
        steps += 1
        collisions = info.get("collision_count", 0)

        if len(agent.replay_buffer) > train_start:
            agent.train(batch_size)

        if done:
            if terminated:
                success = 1
            break

    navigation_time = time.time() - start_time

    with open(csv_filename, mode="a", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(
            [
                episode + 1,
                total_reward,
                collisions,
                steps,
                navigation_time,
                path_length,
                obstacle_avoidance_count,
                success,
            ]
        )

    print(
        f"Episode {episode + 1}, Total Reward: {total_reward}, "
        f"Collisions: {collisions}, Steps: {steps}, Time: {navigation_time:.2f}s, "
        f"Path Length: {path_length:.2f}, Avoidance Count: {obstacle_avoidance_count}, "
        f"Success: {success}"
    )

    if (episode + 1) % 50 == 0:
        agent.save(f"sac_model_{episode + 1}.pth")

agent.save("sac_model_final.pth")
print(zh(r"\u8bad\u7ec3\u5b8c\u6210\uff01\u6a21\u578b\u5df2\u4fdd\u5b58\u3002"))

print(zh(r"\u751f\u6210\u8bad\u7ec3\u7ed3\u679c\u53ef\u89c6\u5316..."))
generate_training_visualizations(csv_filename)
print(zh(r"\u53ef\u89c6\u5316\u7ed3\u679c\u5df2\u4fdd\u5b58\u5230 visualization \u76ee\u5f55\u3002"))

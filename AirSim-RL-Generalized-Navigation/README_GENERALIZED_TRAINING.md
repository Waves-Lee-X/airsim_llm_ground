## 3D Path Planning Mode

The generalized training environment supports full 3D reinforcement-learning path planning:

- Start and target positions are sampled in XYZ.
- The action is `[vx, vy, vz]`; `vz` is no longer forced to zero.
- The state includes six LiDAR clearance values: front, left, right, back, up, and down.
- Reward, success radius, path length, and final distance are computed in 3D.
- Training/evaluation CSV files include `Start_Z` and `Target_Z`.

Because the observation vector changed from 13 to 16 dimensions, train a new model for this version.

Example 3D training:

```powershell
python train_generalized.py --episodes 300 --x-min -60 --x-max 80 --y-min -50 --y-max 50 --z-min -10 --z-max -2
```

Example 3D evaluation:

```powershell
python evaluate_generalized.py --model runs\YYYYMMDD_HHMMSS\sac_model_generalized_latest.pth --episodes 50 --z-min -10 --z-max -2
```

For old fixed-altitude behavior, keep using `--z -2`.

# Generalized AirSim RL Navigation Training

这个副本用于训练更有意义的导航策略：每个 episode 随机起点、随机目标点，目标点作为状态输入，让模型学习“从当前位置到任意目标”的局部导航控制，而不是记住固定目标路线。

## 训练在哪里跑

推荐仍然在 AirSim/Unreal 仿真场景里训练，不建议直接在地面站主程序里训练。

原因：

- 训练需要频繁 `reset`、随机摆放无人机、执行上百到上千个 episode。
- 地面站更适合推理和控制，不适合长时间占用 UI 线程做训练。
- AirSim 仿真可以安全碰撞、重置、随机化起点和目标。

建议流程：

1. 启动 Unreal + AirSim 场景。
2. 使用本项目的 `settings.json`，保证有 `Drone1` 和 `LidarSensor1`。
3. 在本目录启动训练脚本。
4. 训练完成后，把 `sac_model_generalized_final.pth` 复制到地面站的模型目录。

## 启动训练

专家辅助训练，收敛更快，接近原项目思路：

```powershell
python train_generalized.py --episodes 300 --expert-weight 0.2
```

每次训练会自动创建时间目录：

```text
runs/YYYYMMDD_HHMMSS/
```

其中会保存：

```text
training_data_generalized_YYYYMMDD_HHMMSS.csv
sac_model_generalized_ep50_YYYYMMDD_HHMMSS.pth
sac_model_generalized_latest.pth
sac_model_generalized_final_YYYYMMDD_HHMMSS.pth
visualization_generalized/
```

如果你用 `Ctrl+C` 中断训练，脚本会保存：

```text
sac_model_generalized_interrupted_epN_YYYYMMDD_HHMMSS.pth
sac_model_generalized_latest.pth
```

纯 SAC 训练，不依赖专家动作：

```powershell
python train_generalized.py --episodes 1000 --expert-weight 0.0
```

限制训练区域：

```powershell
python train_generalized.py --x-min -60 --x-max 80 --y-min -50 --y-max 50 --z -2
```

## 训练结果是什么

训练结束会生成：

- `sac_model_generalized_final.pth`：最终模型权重。
- `sac_model_generalized_50.pth`、`sac_model_generalized_100.pth` 等：中间检查点。
- `training_data_generalized.csv`：每个 episode 的起点、目标、奖励、路径长度、碰撞、成功率。
- `visualization_generalized/`：奖励、成功率、路径长度、导航时间等曲线。

训练后评估：

```powershell
python evaluate_generalized.py --model sac_model_generalized_final.pth --episodes 50
```

如果要测试某个中间权重，例如第 700 轮：

```powershell
python evaluate_generalized.py --model sac_model_generalized_700.pth --episodes 20
```

评估会生成：

- `evaluation_generalized.csv`
- 随机任务成功率
- 平均最终距离
- 平均路径长度
- 总碰撞次数

真正有价值的结果不是“一条最优路线”，而是一个策略函数：

```text
state = 当前坐标 + 目标坐标 + LiDAR障碍物信息
action = 下一秒应该执行的速度 vx/vy
```

如果训练充分，它可以被地面站用于任意点击目标点的局部导航。

## 当前训练边界

这个版本仍然是固定高度平面导航：

- `z` 默认是 `-2.0`
- 动作执行时 `vz = 0`
- 模型主要学习 X/Y 平面内的到达目标和避障

如果要训练真正三维导航，需要增加上/下方向感知，启用 `vz` 动作，并随机化目标高度。

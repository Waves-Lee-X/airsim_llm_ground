## 三维路径规划模式

当前 generalized 训练环境已经支持三维强化学习路径规划：

- 起点和终点可以在 `X/Y/Z` 三个方向随机采样。
- 动作是 `[vx, vy, vz]`，三维模式下 `vz` 不再强制为 `0`。
- 状态包含六个方向的 LiDAR 距离：前、左、右、后、上、下。
- 奖励、成功半径、路径长度、最终距离都按三维距离计算。
- 训练和评估 CSV 会记录 `Start_Z` 和 `Target_Z`。

注意：三维 LiDAR 模式会把状态向量从旧版 `13` 维变成 `16` 维，所以旧的二维 `.pth` 模型不能直接加载到三维模式里继续训练。三维模式需要重新训练新模型。

### 继续训练旧二维模型

如果你要继续使用之前训练出来的固定高度二维模型，请使用 `--lidar-mode basic`。这个模式会保持旧版 `13` 维状态，并且在 `--z -2` 这类固定高度训练中把 `vz` 保持为 `0`。

```powershell
python train_generalized.py --episodes 1000 --z -2 --max-distance 100 --expert-weight 0.2 --lidar-mode basic --model-in runs\你的训练目录\sac_model_generalized_latest.pth
```

评估旧二维模型时也要加 `--lidar-mode basic`：

```powershell
python evaluate_generalized.py --model runs\你的训练目录\sac_model_generalized_latest.pth --z -2 --lidar-mode basic
```

### 训练新的三维模型

如果要训练新的三维路径规划模型，请使用 `--lidar-mode 3d`，并通过 `--z-min` 和 `--z-max` 指定高度随机范围。

```powershell
python train_generalized.py --episodes 300 --x-min -60 --x-max 80 --y-min -50 --y-max 50 --z-min -10 --z-max -2 --lidar-mode 3d
```

评估三维模型：

```powershell
python evaluate_generalized.py --model runs\YYYYMMDD_HHMMSS\sac_model_generalized_latest.pth --episodes 50 --z-min -10 --z-max -2 --lidar-mode 3d
```

简单记忆：

- 继续训练旧二维模型：使用 `--z -2 --lidar-mode basic`
- 训练新三维模型：使用 `--z-min ... --z-max ... --lidar-mode 3d`
- 旧 `13` 维模型不能直接加载到新 `16` 维三维模式中

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

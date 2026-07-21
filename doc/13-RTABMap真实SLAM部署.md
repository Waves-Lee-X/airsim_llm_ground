# RTAB-Map 真实 SLAM 部署

本文说明如何在当前 PX4 + AirSim + ROS 2 系统中部署 RTAB-Map RGB-D SLAM。当前阶段采用 **Shadow Mode**：SLAM 负责建图、回环和 `map` 坐标定位，但不直接接管 PX4 Offboard 控制。

## 1. 为什么先用 RTAB-Map

当前项目已有同轴前视 RGB、Depth、CameraInfo、PX4 里程计和完整 TF。RTAB-Map 可以直接消费这些数据，使用 RGB-D 特征和回环优化发布 `map -> odom`。

OpenVINS/VINS-Fusion 是后续的真正 VIO 选项，但需要高频相机/IMU、精确时间同步和相机-IMU外参标定。先用 RTAB-Map 验证 RGB-D、TF 和地图链路，可以将问题分层定位。

## 2. 安装依赖

本项目的自动安装被 `sudo` 密码阻止，需要在 WSL 终端手动执行：

```bash
sudo apt update
sudo apt install -y ros-humble-rtabmap-ros ros-humble-image-proc
```

验证：

```bash
source /opt/ros/humble/setup.bash
ros2 pkg prefix rtabmap_slam
ros2 pkg executables rtabmap_slam
```

## 3. 坐标系和数据流

```text
PX4 / AirSim odometry
  └─ odom -> base_link                 连续本地运动
       └─ camera_front_center_optical  RGB-D 外参

RTAB-Map
  ├─ RGB + Depth + CameraInfo
  ├─ /sensor/odometry
  └─ map -> odom                       回环/全局校正

slam_pose_node
  └─ /localization/odometry            map 坐标位姿

semantic_fusion_node
  └─ /world_model/objects              map 坐标语义对象
```

当前 `/autonomy/trajectory` 最终会发送给 PX4 本地位置控制器，因此 Autonomy 继续使用 `/sensor/odometry`。后续必须完成 `map` 轨迹到 PX4 `odom` 轨迹的实时反变换和跳变防护，才能让 SLAM 定位驱动飞行。

## 4. 构建项目

```bash
cd ~/aeromind_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  aeromind_bridge aeromind_perception aeromind_bringup \
  aeromind_autonomy aeromind_agent --symlink-install
source install/setup.bash
```

## 5. 启动顺序

PX4、MicroXRCE-DDS Agent 和 Windows AirSim 的启动方式不变。ROS 主系统改为：

```bash
cd ~/aeromind_ws
source ~/.config/aeromind/env
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch aeromind_bringup aeromind_slam.launch.py \
  yolo_enabled:=true \
  rtabmap_args:=-d
```

`-d` 会在启动时清空旧数据库，适合新建图和重复实验。需要继续使用原地图时：

```bash
ros2 launch aeromind_bringup aeromind_slam.launch.py \
  yolo_enabled:=true \
  rtabmap_args:=""
```

数据库默认保存在 `~/.ros/aeromind_rtabmap.db`。

Web/Gateway 仍使用原命令单独启动。

## 6. 部署验收

### 6.1 先检查传感器频率

```bash
ros2 topic hz /sensor/camera/rgb/front_center
ros2 topic hz /sensor/camera/depth/front_center
ros2 topic hz /sensor/odometry
```

建议 RGB/Depth 实际频率不低于 5 Hz，目标 10 Hz；Odometry 不低于 10 Hz。桥接已将前视 RGB-D 改为一次 AirSim RPC 批量同步请求，辅助相机在 SLAM Launch 中默认关闭。

### 6.2 检查 TF 和输出

```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link camera_front_center_optical
ros2 run tf2_ros tf2_echo map odom

ros2 topic echo /localization/status --once
ros2 topic echo /localization/odometry --once
ros2 topic echo /world_model/objects --once
```

一键只读检查：

```bash
ros2 run aeromind_bringup aeromind_slam_check
```

### 6.3 建图动作

不要原地高速旋转。首次测试使用低速、有重叠视野的短轨迹：

1. 原地保持 3～5 秒，等待首帧和 TF 就绪。
2. 向前 3～5 米，悬停 2 秒。
3. 小角度转向或侧移，保证场景特征重叠。
4. 绕小范围闭环回到起点，观察 `map -> odom` 是否产生平滑校正。

## 7. 常见故障

| 现象 | 原因 | 处理 |
|---|---|---|
| `Package 'rtabmap_slam' not found` | 未安装或未 source ROS | 安装 `ros-humble-rtabmap-ros`，重新 source |
| 没有 `map -> odom` | RGB-D 没同步、无里程计或 TF 断链 | 按 6.1/6.2 逐项检查 |
| RTAB-Map 长时间无节点 | 图像频率太低或场景缺少特征 | 降低分辨率、关闭辅助相机、增加纹理 |
| 位姿突然跳变 | 回环校正或错误匹配 | 保持 Shadow Mode，检查回环置信度和场景重复纹理 |
| 态势图语义点未叠加 | 态势轨迹在 `odom`，语义对象在 `map` | 当前为防止混合坐标而主动隐藏，后续增加 map 态势视图 |
| WSL 处理卡顿 | CPU/RPC/图像分辨率过高 | 关闭辅助相机，先使用 640x360 完成基线 |

## 8. 什么时候可以驱动自主飞行

必须同时满足：

- RGB-D 频率、TF 成功率和 `/localization/odometry` 时效达标。
- 固定场景轨迹误差、回环误差和重定位成功率有量化报告。
- 完成 `map` 轨迹到 PX4 `odom` 设定点的反变换。
- `map -> odom` 跳变、SLAM 丢失和回环失败能触发悬停/降级。
- 仿真固定任务连续 10 次至少 9 次成功。

在此之前，不要将 `autonomy_odom_topic` 改为 `/localization/odometry`用于实际轨迹执行。


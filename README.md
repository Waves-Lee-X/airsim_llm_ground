# AeroMind — 无人机自主飞行系统

基于 ROS 2 Humble 的模块化无人机自主飞行平台，支持 AirSim 仿真与 PX4 飞控。

## 模块架构

```
┌─────────────────────────────────────────────────────────────────┐
│                        AeroMind System                          │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌──────────────┐    ┌──────────────────┐    ┌──────────────┐  │
│  │  aeromind_    │    │  aeromind_       │    │  aeromind_   │  │
│  │  agent        │───▶│  planning        │───▶│  control     │  │
│  │  (LLM Agent)  │    │  (路径规划)       │    │  (飞行控制)   │  │
│  └──────────────┘    └────────┬─────────┘    └──────┬───────┘  │
│                               │                      │          │
│                               ▼                      │          │
│                        ┌──────────────────┐          │          │
│                        │  aeromind_       │          │          │
│                        │  perception      │          │          │
│                        │  (感知)           │          │          │
│                        └────────┬─────────┘          │          │
│                                 │                     │          │
│                                 ▼                     ▼          │
│                          ┌──────────────────────────────────┐   │
│                          │  aeromind_bridge                  │   │
│                          │  (AirSim/PX4 桥接)                │   │
│                          └──────────────────────────────────┘   │
│                                                                 │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │  aeromind_interfaces (消息/服务接口定义)                   │  │
│  └──────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

### 数据流

```
Agent (任务编排)
  │  ExecuteTask.srv
  ▼
Planning (路径规划)
  │  PlanPath.srv → Path.msg
  ▼
Control (飞行控制)
  │  Takeoff/Land/ArmDrone.srv
  │  /control/cmd_vel (Twist)
  ▼
Bridge (AirSim / PX4)
  │  /sensor/odometry, /sensor/imu
  │  /control/drone_state
  └──▶ Perception
       /perception/detection, /perception/obstacle_map → Planning
```

## 环境搭建

### 前置条件

- Ubuntu 22.04（WSL 2）
- ROS 2 Humble
- Python 3.10+
- AirSim（Windows 宿主机，仿真模式）

### 安装步骤

```bash
# 1. 安装 ROS 2 Humble（如已安装可跳过）
# 参考: https://docs.ros.org/en/humble/Installation.html

# 2. 安装 colcon 构建工具
sudo apt install python3-colcon-common-extensions

# 3. 安装 AirSim Python 客户端（需要 Windows 宿主机 AirSim 运行中）
pip install airsim

# 4. 进入工作空间并构建
cd ~/aeromind_ws
colcon build --symlink-install

# 5. 加载工作空间环境
source install/setup.bash

# 6. 启动全部节点（AirSim 模式）
ros2 launch aeromind_bringup aeromind_all.launch.py airsim_ip:=<你的AirSim_IP>
```

## PX4 + AirSim 联合仿真环境搭建

方案 A 架构：AirSim（Windows）做物理/视觉引擎 → PX4 SITL（WSL）做飞控 → ROS 2（WSL）做上层自主飞行。

```
Windows 宿主机                         WSL Ubuntu
┌──────────────────────┐             ┌──────────────────────────────┐
│  AirSim (UE 引擎)     │  TCP:4560   │  PX4 SITL                    │
│  "PX4" 模式           │◄──────────►│  make px4_sitl_default       │
│  settings.json:       │             │       │                      │
│   "SitlPort": 4560   │             │  uXRCE-DDS Agent            │
│   "SitlIp": WSL_IP   │             │       │                      │
└──────────────────────┘             │  ┌────▼───────────────────┐  │
                                     │  │ aeromind_bridge (px4)  │  │
                                     │  │ aeromind_control (px4) │  │
                                     │  └────────────────────────┘  │
                                     └──────────────────────────────┘
```

### PX4 环境搭建步骤

```bash
# 1. 安装 PX4 依赖
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh

# 2. 安装 MicroXRCE-DDS Agent
sudo snap install micro-xrce-dds-agent
# 或从源码安装:
# git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
# cd Micro-XRCE-DDS-Agent && mkdir build && cd build
# cmake .. && make && sudo make install

# 3. 克隆 px4_msgs 到工作空间
cd ~/aeromind_ws/src
git clone https://github.com/PX4/px4_msgs.git

# 4. 重新编译工作空间
cd ~/aeromind_ws
colcon build --symlink-install
source install/setup.bash
```

### AirSim 配置（Windows 宿主机）

在 AirSim 的 `settings.json` 中添加 PX4 配置：

```json
{
  "SeeDocsAt": "https://github.com/Microsoft/AirSim/blob/main/docs/settings.md",
  "SettingsVersion": 1.2,
  "SimMode": "Multirotor",
  "Vehicles": {
    "PX4": {
      "VehicleType": "PX4Multirotor",
      "UseSerial": false,
      "SitlIp": "<WSL_IP地址>",
      "SitlPort": 4560,
      "ControlIp": "remote",
      "LocalHostIp": "<Windows_IP地址>"
    }
  }
}
```

> 获取 WSL IP: `hostname -I`（在 WSL 中执行）

### 启动顺序

```bash
# 终端 1 (WSL): 启动 PX4 SITL（自动连接 AirSim TCP:4560）
cd ~/PX4-Autopilot
make px4_sitl_default none_iris PX4_SIM_HOST_ADDR=<192.168.16.1>

# 终端 2 (WSL): 启动 MicroXRCE-DDS Agent（桥接 PX4 ↔ ROS 2）
MicroXRCEAgent udp4 -p 8888
/snap/bin/micro-xrce-dds-agent udp4 -p 8888

# 终端 3 (Windows): 启动 AirSim（UE 编辑器或 .exe）

# 终端 4 (WSL): 启动 ROS 2 AeroMind 节点
cd ~/aeromind_ws
source install/setup.bash
ros2 launch aeromind_bringup aeromind_px4.launch.py
```

### 启动文件说明

| 文件 | 用途 |
|------|------|
| `aeromind_bringup/launch/aeromind_all.launch.py` | AirSim 模式，bridge 直连 AirSim API |
| `aeromind_bringup/launch/aeromind_px4.launch.py` | PX4 模式，bridge/control 通过 uXRCE-DDS 通信 |

## 模块说明

### aeromind_interfaces（消息定义包）

| 消息/服务 | 类型 | 说明 |
|----------|------|------|
| Detection.msg | 消息 | 目标检测结果（类别、置信度、边界框） |
| ObstacleMap.msg | 消息 | 障碍物占据栅格地图 |
| Path.msg | 消息 | 路径航点序列 |
| DroneState.msg | 消息 | 无人机飞控状态 |
| PlanPath.srv | 服务 | 路径规划：输入起点/终点，返回航点 |
| Takeoff.srv | 服务 | 起飞：输入目标高度 |
| Land.srv | 服务 | 降落 |
| ArmDrone.srv | 服务 | 解锁/加锁 |
| ExecuteTask.srv | 服务 | 自然语言任务执行入口 |

### aeromind_bridge — AirSim/PX4 桥接

- **节点**: `airsim_bridge_node`
- **参数**:
  - `mode`（`airsim` / `px4`，默认 `airsim`）— 切换后端
  - `airsim_ip`（默认 `192.168.1.100`）— AirSim 模式下的宿主机 IP
- **发布**:
  - `/sensor/odometry` (nav_msgs/Odometry, 10Hz)
  - `/sensor/imu` (sensor_msgs/Imu, 10Hz)
  - `/control/drone_state` (aeromind_interfaces/DroneState, 10Hz)
- **订阅**:
  - `/control/cmd_vel` (geometry_msgs/Twist)
    - AirSim 模式 → `moveByVelocityAsync`
    - PX4 模式 → `OffboardControlMode` + `TrajectorySetpoint` via `/fmu/in/`
- **PX4 模式内部订阅**:
  - `/fmu/out/vehicle_odometry` → 转换坐标系后发 `/sensor/odometry`
  - `/fmu/out/vehicle_attitude` + `angular_velocity` + `acceleration` → 合成 `/sensor/imu`
  - `/fmu/out/vehicle_status` → `/control/drone_state`

### aeromind_perception — 感知

- **节点**: `perception_node`
- **发布**:
  - `/perception/detection` (aeromind_interfaces/Detection, 1Hz) — 占位
  - `/perception/obstacle_map` (aeromind_interfaces/ObstacleMap, 1Hz) — 占位
- **订阅**:
  - `/sensor/camera/image` (sensor_msgs/Image) — 预留，暂不处理
- **后续**: 挂载 YOLO 检测、OctoMap 建图

### aeromind_planning — 路径规划

- **节点**: `planning_node`
- **服务**: `/planning/plan_path` (aeromind_interfaces/PlanPath)
  - 当前实现：直线插值（10 个航点）
- **订阅**: `/perception/obstacle_map` — 预留 A* 避障
- **后续**: A* 全局规划器 + 局部避障

### aeromind_control — 飞行控制

- **节点**: `control_node`
- **参数**: `px4_mode`（默认 `airsim`，可选 `px4`）
- **服务**:
  - `/control/takeoff` — 起飞到目标高度
  - `/control/land` — 降落
  - `/control/arm` — 解锁/加锁
- **订阅**: `/planning/path` → 逐航点飞行
- **AirSim 模式**: 直接调用 AirSim API（`armDisarm`、`takeoffAsync`、`moveToPositionAsync`）
- **PX4 模式**: 通过 `/fmu/in/vehicle_command` 发送 MAVLink 指令，通过 `/fmu/in/trajectory_setpoint` 控制位置；维持 Offboard 心跳（10Hz）

### aeromind_agent — LLM Agent

- **节点**: `agent_node`
- **参数**: `llm_api_url`（默认 `http://localhost:11434/v1`）、`llm_model`（默认 `llama3`）
- **服务**: `/agent/execute_task` — 接收自然语言任务
- **当前**: 链式调用 planning → control 验证链路
- **后续**: 接入 LLM + 工具调用链

## 接口清单

### Topic 通信

| Topic | 类型 | 方向 | 模块 |
|-------|------|------|------|
| `/sensor/odometry` | nav_msgs/Odometry | bridge → | bridge |
| `/sensor/imu` | sensor_msgs/Imu | bridge → | bridge |
| `/sensor/camera/image` | sensor_msgs/Image | → perception | (预留) |
| `/control/cmd_vel` | geometry_msgs/Twist | → bridge | control |
| `/control/drone_state` | aeromind_interfaces/DroneState | bridge/control → | bridge, control |
| `/perception/detection` | aeromind_interfaces/Detection | perception → | perception |
| `/perception/obstacle_map` | aeromind_interfaces/ObstacleMap | perception → planning | perception |
| `/planning/path` | aeromind_interfaces/Path | planning → control | planning |

### Service 通信

| Service | 类型 | 提供方 | 调用方 |
|---------|------|--------|--------|
| `/planning/plan_path` | PlanPath.srv | planning | agent |
| `/control/takeoff` | Takeoff.srv | control | agent |
| `/control/land` | Land.srv | control | agent |
| `/control/arm` | ArmDrone.srv | control | agent |
| `/agent/execute_task` | ExecuteTask.srv | agent | 外部 |

## 目录结构

```
aeromind_ws/
├── src/
│   ├── aeromind_interfaces/     # 消息和服务定义 (ament_cmake)
│   │   ├── msg/                  #   Detection, ObstacleMap, Path, DroneState
│   │   ├── srv/                  #   PlanPath, Takeoff, Land, ArmDrone, ExecuteTask
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── aeromind_bridge/         # AirSim 桥接 (ament_python)
│   ├── aeromind_perception/     # 感知 (ament_python)
│   ├── aeromind_planning/       # 路径规划 (ament_python)
│   ├── aeromind_control/        # 飞行控制 (ament_python)
│   ├── aeromind_agent/          # LLM Agent (ament_python)
│   ├── aeromind_bringup/        # 启动文件 (ament_cmake)
│   │   └── launch/
│   │       ├── aeromind_all.launch.py
│   │       └── aeromind_px4.launch.py
├── AeroMind Console/            # (旧项目，参考)
├── ground_station_qt_airsim/    # (旧项目，参考)
├── README.md
└── .gitignore
```

## 许可证

Apache-2.0

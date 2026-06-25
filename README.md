# AeroMind — 无人机自主飞行系统

基于 ROS 2 Humble 的模块化无人机自主飞行平台，支持 AirSim 视觉仿真、PX4 飞控固件。

---

## 1. 项目定位

AeroMind 是一个**多模块协作**的无人机自主飞行平台。上层由 LLM Agent 接收自然语言任务，调用路径规划生成航点，通过飞行控制模块驱动无人机执行。底层对接 PX4 飞控，仿真阶段可选 AirSim（视觉+激光雷达）或 PX4 自带 Gazebo/SIH。

**当前状态**：
- ROS 2 模块框架完成（9 个包，全部编译通过）
- PX4 SITL 连接打通，Gazebo 和 SIH 两种仿真模式可用
- ROS 2 arm → Offboard takeoff → 键盘/路径控制 → land 链路可用
- AirSim 相机/LiDAR 可视化已接入，RGB 相机和深度点云 RViz 使用独立配置
- 感知模块（YOLO + OctoMap）和 LLM 推理待接入

---

## 2. 运行模式概览

| 模式 | 仿真器 | 传感器 | 启动文件 | 适用场景 |
|------|--------|--------|---------|---------|
| AirSim 直连 | AirSim（Windows） | SimpleFlight 内置 | `aeromind_all.launch.py` | 快速测试 bridge API |
| AirSim + PX4 | AirSim（Windows）+ PX4（WSL） | AirSim 传感器→PX4→ROS | `aeromind_px4.launch.py` | 完整视觉+飞控仿真 |
| PX4 + Gazebo | Gazebo Classic（WSL） | Gazebo 传感器→PX4→ROS | `aeromind_px4.launch.py` | 纯 Linux 仿真（无需 Windows）|
| PX4 + SIH | SIH（PX4 内置） | 内置物理模型 | `aeromind_px4.launch.py` | 最简测试（arm/takeoff/plan） |

---

## 3. 模块架构

```
┌──────────────────────────────────────────────────────────────────┐
│                       AeroMind System                            │
│                                                                  │
│   ┌─────────────┐     ┌──────────────┐     ┌────────────┐       │
│   │ aeromind_    │     │ aeromind_    │     │ aeromind_  │       │
│   │ agent        │────▶│ planning     │────▶│ control    │       │
│   │ LLM 任务编排  │     │ 路径规划      │     │ 飞行控制    │       │
│   └─────────────┘     └──────┬───────┘     └─────┬──────┘       │
│                              │                    │              │
│                              ▼                    │              │
│                       ┌──────────────┐            │              │
│                       │ aeromind_    │            │              │
│                       │ perception   │            │              │
│                       │ 感知 (YOLO)  │            │              │
│                       └──────┬───────┘            │              │
│                              │                    │              │
│                              ▼                    ▼              │
│                       ┌────────────────────────────────┐        │
│                       │       aeromind_bridge           │        │
│                       │    AirSim / PX4 桥接层           │        │
│                       └────────────────────────────────┘        │
│                              │                                   │
│               ┌──────────────┴──────────────┐                   │
│               ▼                              ▼                   │
│         ┌──────────┐                  ┌──────────┐              │
│         │  AirSim   │                  │  PX4 SITL │              │
│         │ (Windows) │                  │  (WSL)    │              │
│         └──────────┘                  └──────────┘              │
│                                                                  │
│   ┌─────────────────────────────────────────────────────────┐   │
│   │  aeromind_interfaces   消息/服务接口（跨模块通信协议）      │   │
│   └─────────────────────────────────────────────────────────┘   │
└──────────────────────────────────────────────────────────────────┘
```

### 3.1 数据流

```
Agent ──ExecuteTask.srv──▶ Planning ──PlanPath.srv──▶ Path.msg
                                                           │
                                                           ▼
Perception ◀── /sensor/camera/rgb/*            Control ──Takeoff/Land/Arm.srv
    │                                                │
    │  /perception/detection                         │  /fmu/in/vehicle_command
    │  /perception/obstacle_map                      │  /fmu/in/trajectory_setpoint
    ▼                                                ▼
Planning ◀────────────────────────────────── Bridge ──▶ PX4 / AirSim
                                                  │
                                                  │  /sensor/odometry
                                                  │  /sensor/imu
                                                  │  /sensor/camera/rgb/front_center
                                                  │  /sensor/camera/depth/front_center
                                                  │  /sensor/lidar/points
                                                  │  /control/drone_state
                                                  ▼
                                            全部模块可消费
```

### 3.2 包列表（9 个）

| 包名 | 类型 | 职责 | 负责团队 |
|------|------|------|------------|
| `aeromind_interfaces` | ament_cmake | 4 msg + 5 srv 定义 | A. 平台与接口 |
| `px4_msgs` | ament_cmake | PX4 官方消息定义（200+ msg） | 公共依赖 |
| `aeromind_bridge` | ament_python | AirSim/PX4 ↔ ROS 2 传感器与控制桥接 | B. Bridge |
| `aeromind_control` | ament_python | 飞行控制服务（arm/takeoff/land/路径跟随） | C. Control |
| `aeromind_teleop` | ament_python | 键盘遥控（位置/速度双模式） | C. Control |
| `aeromind_perception` | ament_python | YOLO 检测 + OctoMap 建图（占位） | D. Perception |
| `aeromind_planning` | ament_python | 路径规划服务（直线插值→A*） | E. Planning |
| `aeromind_agent` | ament_python | LLM 任务调度（服务链编排） | F. Agent |
| `aeromind_bringup` | ament_cmake | 启动文件汇总 + RViz 配置 | A. 平台与接口 |

---

## 4. 环境搭建（新成员入职指南）

### 4.1 前置条件

- **操作系统**：Ubuntu 22.04（WSL2 或原生）
- **ROS 2**：Humble Hawksbill
- **Python**：3.10+（系统自带）
- **PX4**：需要源码编译（SITL 仿真）
- **AirSim**（可选）：Windows 宿主机，UE 引擎

### 4.2 一键搭建脚本

```bash
# === 1. ROS 2 Humble ===
# 参考 https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

# === 2. 安装依赖 ===
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-pip
pip install numpy empy toml

# === 3. 克隆并编译工作空间 ===
cd ~
git clone <本项目仓库地址> aeromind_ws
cd ~/aeromind_ws/src
git clone https://github.com/PX4/px4_msgs.git

cd ~/aeromind_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# === 4. 安装 ros（PX4 ↔ ROS 2 协议桥） ===
sudo snap install micro-xrce-dds-agent

# === 5. 克隆并编译 PX4（首次约 30 分钟） ===
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
make px4_sitl_default   # 仅编译，不启动仿真
```

### 4.3 验证安装

```bash
# 验证 ROS 2 工作空间
cd ~/aeromind_ws && source install/setup.bash
ros2 pkg list | grep aeromind
# 应输出 8 个 aeromind_* 包

# 验证 PX4 build
cd ~/PX4-Autopilot && ls build/px4_sitl_default/bin/px4
# 应存在 px4 可执行文件
```

---

## 5. 启动方式

### 5.1 模式一：PX4 + SIH（推荐入门，无需外部仿真器）

SIH 是 PX4 内置的仿真器，在 PX4 进程内模拟 IMU/GPS/电机/电池，命令行运行。

```bash
# 终端 1 — PX4 SITL + SIH
cd ~/PX4-Autopilot
make px4_sitl_default sihsim_quadx

# 终端 2 — MicroXRCE-DDS Agent
micro-xrce-dds-agent udp4 -p 8888

# 终端 3 — ROS 2
cd ~/aeromind_ws && source install/setup.bash
ros2 launch aeromind_bringup aeromind_px4.launch.py
```

### 5.2 模式二：PX4 + Gazebo Classic（有 3D 画面）

```bash
# 终端 1
cd ~/PX4-Autopilot
make px4_sitl_default gazebo-classic
# 会弹出一个 Gazebo 窗口（WSL2 下渲染较慢）

# 终端 2、3 同上
```

### 5.3 模式三：PX4 + AirSim（需要 Windows 上的 AirSim）

**Windows 侧**：AirSim settings.json 配置见 [doc/PX4_AirSim_配置说明.md](doc/PX4_AirSim_配置说明.md)

**WSL 侧**：
```bash
# 终端 1 — MicroXRCE-DDS Agent
micro-xrce-dds-agent udp4 -p 8888

# 终端 2 — PX4 SITL（等待 AirSim 连接）
cd ~/PX4-Autopilot
make px4_sitl_default none_iris

# 终端 3 — ROS 2
cd ~/aeromind_ws && source install/setup.bash
ros2 launch aeromind_bringup aeromind_px4.launch.py
# 默认自动检测 WSL2 默认网关作为 AirSim IP；如需指定：
# ros2 launch aeromind_bringup aeromind_px4.launch.py airsim_ip:=192.168.x.x

# 终端 4（Windows） — 启动 AirSim
```

### 5.4 PX4 解锁前参数设置（仅首次）

PX4 默认要求遥控器/GPS/安全开关才能解锁。SITL 调试时需要跳过这些检查：

```
# 在 PX4 SITL 终端中执行（仅首次，param save 后持久化）：
param set COM_RCL_EXCEPT 4     # 允许无遥控器时进入 Offboard
param set COM_ARM_WO_GPS 1     # 允许无 GPS 时解锁
param set NAV_RCL_ACT 0        # 禁用遥控器丢失保护
param set COM_OF_LOSS_T 0      # Offboard 断连不超时
param save
```

> 如果遇到 `Disarmed by auto preflight disarming`，说明有预检未通过。在 PX4 终端执行 `commander check` 查看具体失败项。

### 5.5 起飞测试

```bash
cd ~/aeromind_ws && source install/setup.bash

# 解锁（可选；takeoff 在 PX4 模式下也会自动解锁并切 Offboard）
ros2 service call /control/arm aeromind_interfaces/srv/ArmDrone "{arm: true}"

# 起飞到 10m。PX4 模式下使用 Offboard 位置控制，而不是一次性的 NAV_TAKEOFF。
sleep 1
ros2 service call /control/takeoff aeromind_interfaces/srv/Takeoff "{altitude: 10.0}"

# 降落
ros2 service call /control/land aeromind_interfaces/srv/Land "{}"
```

起飞后可用下面命令确认状态。正常情况下 `mode` 应进入 `OFFBOARD`：

```bash
ros2 topic echo /control/drone_state
```

---

## 6. PX4 仿真参数速查

不同仿真模式下需要设的参数不同。`param save` 会将参数持久化到 `build/px4_sitl_default/rootfs/parameters.bson`。

### 6.1 SIH 模式（最简）

```
param set SYS_HAS_MAG 0          # SIH 无磁罗盘仿真
param set COM_RCL_EXCEPT 4       # 允许无遥控器时进入 Offboard
param set COM_ARM_WO_GPS 1       # 允许无 GPS 时解锁
param save
```

### 6.2 Gazebo 模式

Gazebo 提供完整传感器仿真，通常无需额外参数。如果遇到预检失败：

```
param set SYS_HAS_MAG 0          # 如果磁罗盘数据异常
param set COM_RCL_EXCEPT 4       # 允许无遥控器时进入 Offboard
param save
```

### 6.3 AirSim + PX4 模式

AirSim 不仿真电池/磁罗盘/电源管理芯片，需要绕过对应检查：

```
# 传感器
param set SYS_HAS_MAG 0          # AirSim 磁罗盘不校准
param set COM_ARM_WO_GPS 1       # 允许无 GPS 解锁

# 电池
param set BAT_CRIT_THR -1        # 关闭电池严重低电量保护
param set BAT_LOW_THR -1         # 关闭电池低电量警告
param set BAT_EMERGEN_THR -1     # 关闭电池紧急保护

# 硬件电路断路器（绕过不存在的硬件检查）
param set CBRK_SUPPLY_CHK 894281 # 电源检查
param set CBRK_USB_CHK 197848    # USB 连接检查
param set CBRK_IO_SAFETY 22027   # IO 安全开关

# 失控保护
param set COM_RCL_EXCEPT 4       # 允许无遥控器时进入 Offboard
param set COM_OF_LOSS_T 0        # Offboard 断连不超时
param set NAV_RCL_ACT 0          # 遥控器丢失不返航
param set NAV_DLL_ACT 0          # 数据链丢失不动作

param save
```

### 6.4 清除所有已保存参数（恢复出厂）

```bash
rm -f ~/PX4-Autopilot/build/px4_sitl_default/rootfs/parameters.bson
rm -f ~/PX4-Autopilot/build/px4_sitl_default/rootfs/parameters_backup.bson
```

### 6.5 常用 PX4 调试命令

| 命令 | 用途 |
|------|------|
| `listener vehicle_local_position` | 查看位姿（注意 ref_alt） |
| `listener vehicle_status` | 查看解锁状态 |
| `commander check` | 列出所有未通过的预检 |
| `commander arm -f` | 强制解锁（跳过检查） |
| `commander takeoff` | 内部起飞 |
| `param show` | 列出所有参数 |
| `param set <NAME> <VALUE>` | 修改参数 |

---

## 7. 团队任务分工与集成方案

本项目按“模块独立开发、接口统一集成”的方式协作。每个组员只负责自己的包和对外接口，Web 控制台只通过 ROS topic/service/action 消费能力，不直接修改 PX4 底层话题。

### 7.1 协作原则

| 原则 | 说明 |
|------|------|
| 接口先行 | 所有跨模块通信都通过 `aeromind_interfaces` 或标准 ROS 消息完成 |
| 模块自治 | 每个组员可以独立启动、测试、替换自己的节点 |
| UI 不碰底层 | Web 控制台只调用 `/control/*`、`/planning/*`、`/agent/*`，不直接发布 `/fmu/in/*` |
| 可观测优先 | 每个模块都要有清晰的 topic/service、日志和故障提示 |
| 小步集成 | 新功能先用命令行验证，再接入 Agent，最后接入 Web 控制台 |

接口变更规则：修改 `.msg` 或 `.srv` 前必须通知全员；接口合并后所有人执行 `colcon build --symlink-install`。

### 7.2 总体分工

| 组别 | 负责包/目录 | 核心目标 | 对外交付 |
|------|-------------|----------|----------|
| A. 平台与接口组 | `aeromind_interfaces`, `aeromind_bringup` | 维护通信协议、启动文件、RViz 配置 | 稳定 msg/srv、统一 launch、联调文档 |
| B. Bridge 组 | `aeromind_bridge` | 打通 PX4/AirSim 与 ROS 2 | 传感器话题、飞控状态、相机/LiDAR 数据 |
| C. Control 组 | `aeromind_control`, `aeromind_teleop` | 提供安全可靠的飞行动作 | arm/takeoff/land/hover/path follow/键盘控制 |
| D. Perception 组 | `aeromind_perception` | 目标检测与环境感知 | detection、obstacle map、可视化叠加数据 |
| E. Planning 组 | `aeromind_planning` | 路径规划与避障 | plan_path 服务、路径 topic、路径质量指标 |
| F. Agent 组 | `aeromind_agent` | 自然语言任务解析与编排 | execute_task 服务、结构化任务、执行状态 |
| G. Web Console 组 | 后续新增 `aeromind_web` 或 `web_console/` | 集成控制台界面 | 状态面板、视频流、地图、自然语言输入、操作日志 |

### 7.3 A 组：平台与接口

**职责**：维护所有模块共同依赖的接口、启动入口和集成说明。

**当前接口**：

| 接口名 | 类型 | 用途 |
|--------|------|------|
| `DroneState.msg` | msg | 飞控状态：armed、mode、battery、gps、ekf |
| `Detection.msg` | msg | 目标检测结果 |
| `ObstacleMap.msg` | msg | 占据栅格地图 |
| `Path.msg` | msg | 航点序列 |
| `ArmDrone.srv` | srv | 解锁/加锁 |
| `Takeoff.srv` | srv | 起飞到指定高度 |
| `Land.srv` | srv | 降落 |
| `PlanPath.srv` | srv | 路径规划 |
| `ExecuteTask.srv` | srv | 自然语言任务入口 |

**近期任务**：
- 为 Web 控制台补充更适合 UI 展示的状态接口，例如任务状态、错误码、飞行阶段
- 统一 launch：主系统、视觉、Web 控制台一键启动
- 维护 README、故障排查和联调清单

**验收标准**：
- 新成员能按 README 启动系统
- 任意接口变更都有说明和示例调用
- `colcon build --symlink-install` 全量通过

### 7.4 B 组：Bridge

**职责**：屏蔽 PX4/AirSim 差异，向上提供稳定的传感器和状态数据。

**输入**：

| 来源 | Topic/API |
|------|-----------|
| PX4 | `/fmu/out/vehicle_status*`, `/fmu/out/vehicle_local_position*`, `/fmu/out/sensor_combined`, `/fmu/out/vehicle_attitude` |
| AirSim | `simGetImages`, `getLidarData` |
| 控制台/键盘 | `/control/cmd_vel` |

**输出**：

| Topic | 类型 | 用途 |
|-------|------|------|
| `/sensor/odometry` | `nav_msgs/Odometry` | 位姿/速度 |
| `/sensor/imu` | `sensor_msgs/Imu` | IMU |
| `/control/drone_state` | `DroneState` | UI 状态栏 |
| `/sensor/camera/rgb/front_center` | `sensor_msgs/Image` | Web 视频/RViz |
| `/sensor/camera/depth/front_center` | `sensor_msgs/Image` | 深度点云/建图 |
| `/sensor/lidar/points` | `sensor_msgs/PointCloud2` | 点云/避障 |

**近期任务**：
- 补齐电池、GPS、EKF 健康状态，不再用默认值
- 提供 Web 友好的低频状态汇总 topic，避免前端订阅太多底层 topic
- 稳定相机/LiDAR 频率，必要时降低分辨率或增加参数配置

**验收标准**：
- PX4 模式下 `/sensor/odometry`、`/control/drone_state` 稳定输出
- AirSim 可用时 RGB、Depth、LiDAR 都有数据
- RViz RGB 和 DepthCloud 能显示

### 7.5 C 组：Control

**职责**：把飞行动作封装成安全的 ROS 服务，供 Agent 和 Web 控制台调用。

**对外服务**：

| 服务 | 类型 | 说明 |
|------|------|------|
| `/control/arm` | `ArmDrone` | 解锁/加锁 |
| `/control/takeoff` | `Takeoff` | Offboard 起飞到目标高度 |
| `/control/land` | `Land` | 降落 |

**控制输入**：

| Topic | 类型 | 用途 |
|-------|------|------|
| `/control/cmd_vel` | `geometry_msgs/Twist` | 键盘/Web 虚拟摇杆 |
| `/planning/path` | `Path` | 路径跟随 |

**近期任务**：
- 增加 `hover`、`return_home`、`emergency_stop` 等服务
- 路径跟随从固定等待改成“位置误差阈值 + 超时”
- 订阅 `VehicleCommandAck`，把 PX4 拒绝原因反馈给 UI

**验收标准**：
- Web 控制台按钮调用服务后有明确 success/message
- 起飞后 `/control/drone_state.mode` 进入 `OFFBOARD`
- 失败时不静默，必须给出可读错误

### 7.6 D 组：Perception

**职责**：从相机、深度图、LiDAR 中提取环境信息，输出给 Planning、Agent 和 Web UI。

**输入**：

| Topic | 类型 |
|-------|------|
| `/sensor/camera/rgb/front_center` | `sensor_msgs/Image` |
| `/sensor/camera/depth/front_center` | `sensor_msgs/Image` |
| `/sensor/lidar/points` | `sensor_msgs/PointCloud2` |
| `/sensor/odometry` | `nav_msgs/Odometry` |

**输出**：

| Topic | 类型 | 用途 |
|-------|------|------|
| `/perception/detection` | `Detection` | 目标检测结果 |
| `/perception/obstacle_map` | `ObstacleMap` | 规划避障 |

**近期任务**：
- 接入 YOLO，输出检测框、类别、置信度
- 设计 Web 控制台可叠加显示的检测结果格式
- 用深度图或 LiDAR 生成基础障碍物地图

**验收标准**：
- 给定图像输入时能稳定输出 detection
- Web 控制台能显示检测框或目标列表
- obstacle_map 能被 Planning 消费

### 7.7 E 组：Planning

**职责**：根据任务目标、当前位置和障碍物地图生成可执行路径。

**服务**：

| 服务 | 类型 | 说明 |
|------|------|------|
| `/planning/plan_path` | `PlanPath` | 输入起点/终点，返回 `Path` |

**近期任务**：
- 将当前直线插值升级为 A* 或 RRT
- 增加路径平滑和高度约束
- 发布路径可视化 topic，供 Web 控制台显示

**验收标准**：
- 无障碍时路径近似直线
- 有障碍时能绕障
- 规划结果能被 Control 执行

### 7.8 F 组：Agent

**职责**：把自然语言转成结构化任务，并编排 Planning、Control、Perception。

**服务**：

| 服务 | 类型 | 说明 |
|------|------|------|
| `/agent/execute_task` | `ExecuteTask` | Web 控制台的自然语言入口 |

**建议输出结构**：

```json
{
  "intent": "takeoff",
  "args": {"altitude": 10.0},
  "steps": ["arm", "takeoff"],
  "risk_level": "low"
}
```

**近期任务**：
- 接入 LLM，将自然语言解析为结构化 JSON
- 加入任务确认机制：高风险任务先返回确认，不直接执行
- 输出任务执行进度，供 Web 控制台展示

**验收标准**：
- “起飞到 10 米”能调用 `/control/takeoff`
- “飞到前方 20 米”能调用 Planning + Control
- 失败时返回明确原因，而不是只返回 false

### 7.9 G 组：Web Console

**职责**：做一个浏览器访问的无人机控制台，集成状态、视频、地图、控制按钮和自然语言输入。

**建议技术路线**：

| 层 | 推荐方案 | 说明 |
|----|----------|------|
| 前端 | React / Vue / 原生 Vite | 做仪表盘、视频、地图、任务面板 |
| ROS 网关 | `rosbridge_suite` 或自研 FastAPI + rclpy | 把 ROS topic/service 转成 WebSocket/HTTP |
| 视频 | WebSocket JPEG/MJPEG 或 `rosbridge` 图像订阅 | 先做低帧率稳定显示，后续再优化 |
| 地图/点云 | 先做 2D 路径和状态，点云后续接 Three.js | 避免第一版过重 |

**第一版功能范围**：
- 顶部状态栏：armed、mode、battery、gps、ekf
- 主画面：RGB 相机
- 控制区：Arm、Takeoff、Land、Hover、Emergency Stop
- 自然语言输入框：调用 `/agent/execute_task`
- 日志区：显示 service 返回值、PX4 ack、错误提示

**只允许调用的接口**：

| 功能 | 接口 |
|------|------|
| 状态显示 | `/control/drone_state`, `/sensor/odometry` |
| RGB 视频 | `/sensor/camera/rgb/front_center` |
| 控制按钮 | `/control/arm`, `/control/takeoff`, `/control/land` |
| 自然语言 | `/agent/execute_task` |
| 路径显示 | `/planning/path` 或后续新增 path visualization topic |
| 检测结果 | `/perception/detection` |

**不要直接使用**：
- `/fmu/in/vehicle_command`
- `/fmu/in/trajectory_setpoint`
- `/fmu/in/offboard_control_mode`

这些底层 PX4 话题只由 `aeromind_control` 和 `aeromind_bridge` 管理。

**验收标准**：
- 浏览器能看到实时状态和 RGB 图像
- 按钮能成功调用 ROS 服务并显示结果
- 自然语言输入能调用 Agent 并显示结构化结果
- 后端断开或服务失败时，UI 有明确错误提示

### 7.10 工具模块

| 模块 | 位置 | 功能 |
|------|------|------|
| `vehicle_status_decoder.py` | `aeromind_bridge/` | PX4 VehicleStatus 中文解析器 |
| `geodesic_utils.py` | `aeromind_planning/` | WGS84 GPS↔ENU 坐标转换、距离、方位角 |

---

## 8. 可视化（相机 + RViz）

**⚠️ 依赖**: 需要先启动主系统（bridge 节点提供相机/LiDAR 数据源），否则 RViz 窗口将是空的。PX4 + SIH/Gazebo 只有飞控/里程计数据；RGB、深度图和 LiDAR 来自 AirSim。

先启动 PX4 SITL + ROS 2 节点：

```bash
# 终端 1-3: 按 5.1/5.2/5.3 任一模式启动
ros2 launch aeromind_bringup aeromind_px4.launch.py
```

然后另开终端启动可视化：

```bash
cd ~/aeromind_ws && source install/setup.bash
ros2 launch aeromind_bringup aeromind_vision.launch.py
```

启动内容：
- RGB 图像 + LiDAR RViz2 窗口（`image_lidar.rviz`）
- 深度点云 RViz2 窗口（`depth_cloud.rviz`）

当前可视化话题：

| 内容 | Topic | 类型 | 说明 |
|------|-------|------|------|
| 前视 RGB | `/sensor/camera/rgb/front_center` | `sensor_msgs/Image` | `rgb8` 或 `rgba8` |
| RGB 内参 | `/sensor/camera/rgb/camera_info` | `sensor_msgs/CameraInfo` | RViz Camera 父级内参 |
| 前视深度 | `/sensor/camera/depth/front_center` | `sensor_msgs/Image` | `32FC1` |
| 深度内参 | `/sensor/camera/depth/camera_info` | `sensor_msgs/CameraInfo` | RViz DepthCloud 父级内参 |
| LiDAR | `/sensor/lidar/points` | `sensor_msgs/PointCloud2` | AirSim LiDAR |

可视化 TF：

```
odom
├── camera_front_center_body
│   └── camera_front_center_optical
└── lidar
```

快速验证：

```bash
ros2 topic hz /sensor/camera/rgb/front_center
ros2 topic echo /sensor/camera/rgb/front_center --once
ros2 topic echo /sensor/camera/rgb/camera_info --once

ros2 topic hz /sensor/camera/depth/front_center
ros2 topic echo /sensor/camera/depth/camera_info --once
ros2 run tf2_ros tf2_echo camera_front_center_body camera_front_center_optical
```

> RGB 相机有画面但 DepthCloud 没画面时，优先检查深度图、`/sensor/camera/depth/camera_info` 和 `camera_front_center_body → camera_front_center_optical` TF 是否同时存在。

---

## 9. 多人协作开发规范

### 9.1 分支策略

```
main          ← 稳定版本（所有人的接口变更最终合入这里）
├── feature/interfaces-xxx  ← 平台与接口
├── feature/bridge-xxx      ← Bridge
├── feature/control-xxx     ← Control
├── feature/perception-xxx  ← Perception
├── feature/planning-xxx    ← Planning
├── feature/agent-xxx       ← Agent
└── feature/web-console-xxx ← Web Console
```

### 9.2 接口变更流程

1. **修改前**在群内通知："我要改 `aeromind_interfaces/msg/Path.msg`，加一个 `speed` 字段"
2. 修改 `.msg` / `.srv` 文件
3. **全量编译**：`colcon build --symlink-install`（因为有依赖链）
4. 自己模块测试通过后，**push 到自己的 feature 分支**
5. 其他组员 `git pull` + `colcon build` 更新接口

### 9.3 模块独立测试

每个组员可以在**自己的分支上**独立开发，不需要等其他人。关键是：

- **不改 aeromind_interfaces 的情况下**，只改自己的 `.py`，编译仅需 `--packages-select 你的包名`
- 需要联调时，两个组员都启动自己的节点，通过 ROS 2 topic/service 通信
- 测试数据流：用 `ros2 topic echo` 和 `ros2 service call` 验证输入输出
- Web 控制台接入前，底层模块必须先通过命令行 topic/service 验证

### 9.4 推荐开发方式

```
# 不依赖其他人的节点启动全部系统，只测试自己的：
colcon build --symlink-install --packages-select aeromind_planning
ros2 run aeromind_planning planning_node

# 另开终端，手动调自己的服务看是否正确：
ros2 service call /planning/plan_path aeromind_interfaces/srv/PlanPath "{...}"
```

Web 控制台开发建议：

```
# 第一阶段只连 mock 数据或已稳定 topic
/control/drone_state
/sensor/camera/rgb/front_center
/control/arm
/control/takeoff
/control/land
/agent/execute_task
```

### 9.5 Git 忽略

`build/`、`install/`、`log/` 已在 `.gitignore` 中。不要在仓库里提交编译产物。
`parameters.bson` 这类 PX4 本地文件也不需要提交。

---

## 10. 故障排查

| 现象 | 可能原因 | 解决 |
|------|---------|------|
| `Disarmed by auto preflight disarming` | PX4 预检未通过（无 RC/GPS/安全开关） | 执行 5.4 节参数设置；`commander check` 查看具体失败项 |
| `The message type 'aeromind_interfaces/msg/DroneState' is invalid` | 当前终端未 source 工作空间 | `cd ~/aeromind_ws && source install/setup.bash` |
| bridge 日志 "incompatible QoS" | PX4 用 BEST_EFFORT，ROS 用 RELIABLE | 已修复：代码中设了 `ReliabilityPolicy.BEST_EFFORT` |
| bridge 报 "The 'x' field must be of type 'float'" | PX4 消息字段是 numpy float32 | 已修复：`px4_bridge.py` 中显式 `float()` 转换 |
| ros2 topic echo 无数据 | MicroXRCE-DDS Agent 未启动或 QoS 不匹配 | 检查 Agent 是否在跑，检查 bridge 日志无 QoS 警告 |
| `commander arm` 拒绝 | 预检未通过 | 执行 `commander check` 看失败的项 |
| ROS arm 成功但立即 disarm | 有 failsafe 触发 | 看 PX4 日志找具体 failsafe 类型（常见：RC loss、电池、GPS） |
| ROS takeoff 返回成功但不飞 | PX4 未进入/保持 Offboard，或 setpoint 未持续发布 | 当前代码已改为 Offboard 起飞；检查 `/control/drone_state` 是否为 `OFFBOARD` |
| `vehicle_command_ack` 显示拒绝 | PX4 拒绝命令 | `ros2 topic echo /fmu/out/vehicle_command_ack` 查看 command/result |
| PX4 连不上 AirSim | IP 变了或防火墙 | `hostname -I` 确认 WSL IP，检查 Windows 防火墙 4560 |
| AirSim 相机连接超时 (ETIMEDOUT) | PX4 模式下 AirSim 未启动 | **正常警告**，不影响 PX4 飞控功能，仅相机/LiDAR 不可用 |
| RViz `CompressedPublisher: Image is wrongly formed` | 把 AirSim 压缩图像当原始图像发布 | 当前 bridge 使用 `compress=False`；重新编译并重启 bridge |
| RGB topic 有数据但 RViz Camera 黑屏 | 缺父级 `camera_info` 或 RViz 仍指向旧 compressed topic | 检查 `/sensor/camera/rgb/camera_info`；确认 `image_lidar.rviz` 指向 `/sensor/camera/rgb/front_center` |
| RGB 有画面但 DepthCloud 空 | 深度图、深度 camera_info 或 optical TF 缺失 | 检查 `/sensor/camera/depth/front_center`、`/sensor/camera/depth/camera_info`、`tf2_echo camera_front_center_body camera_front_center_optical` |
| colcon build 失败 | 缺依赖或接口变了 | `rm -rf build/ install/ && colcon build --symlink-install` |
| Gazebo 窗口空白 | WSL2 无 GPU 加速 | 等几分钟或用 SIH 模式替代 |
| teleop 键盘无响应 | 非 TTY 环境（如 launch 中启动） | teleop 必须在独立终端用 `ros2 run` 运行 |
| RViz 无点云/图像 | bridge 未启动或未连 AirSim | 先启动主系统 launch；确认 bridge 日志无相机连接错误 |
| WSL 提示 localhost 代理未镜像 | Windows/WSL 代理提示 | 与 ROS 2 相机/飞控无直接关系，可先忽略 |

---

## 11. 目录结构

```
aeromind_ws/
├── src/
│   ├── aeromind_interfaces/     # 消息/服务定义（ament_cmake，公共依赖）
│   │   ├── msg/                 # Detection, ObstacleMap, Path, DroneState
│   │   ├── srv/                 # PlanPath, Takeoff, Land, ArmDrone, ExecuteTask
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── aeromind_bridge/         # 桥接节点（Bridge 组）
│   │   ├── aeromind_bridge/
│   │   │   ├── airsim_bridge_node.py
│   │   │   ├── camera_bridge.py # AirSim RGB/Depth/LiDAR 桥接
│   │   │   └── px4_bridge.py    # PX4 NED↔ENU 转换工具
│   │   ├── setup.py / setup.cfg / package.xml
│   ├── aeromind_perception/     # 感知节点（Perception 组）
│   ├── aeromind_planning/       # 路径规划节点（Planning 组）
│   ├── aeromind_control/        # 飞行控制节点（Control 组）
│   │   ├── aeromind_control/
│   │   │   ├── control_node.py
│   │   │   └── px4_control.py   # PX4 Offboard 控制器
│   ├── aeromind_agent/          # LLM Agent 节点（Agent 组）
│   ├── aeromind_teleop/         # 键盘遥控节点（Control 组）
│   ├── aeromind_bringup/        # 启动文件（平台与接口组）
│   │   └── launch/
│   │       ├── aeromind_all.launch.py     # AirSim 直连模式
│   │       ├── aeromind_px4.launch.py     # PX4 模式
│   │       ├── aeromind_vision.launch.py  # 相机 + RViz 可视化
│   │       ├── depth_cloud.rviz           # 深度点云 RViz 配置
│   │       └── image_lidar.rviz           # 图像+激光雷达 RViz 配置
│   └── px4_msgs/                # PX4 消息定义（从 GitHub 克隆）
├── doc/
│   ├── PX4_AirSim_配置说明.md    # AirSim + PX4 详细配置文档
│   ├── WSL2_代理配置.md          # WSL2 代理/VPN 配置
│   └── airsim_settings.json     # AirSim 完整配置备份
├── README.md
└── .gitignore
```

## 12. 相关文档

- [PX4 + AirSim 配置详细说明](doc/PX4_AirSim_配置说明.md) — 网络配置、AirSim settings.json、PX4 参数详解
- [PX4 User Guide](https://docs.px4.io/main/en/)
- [ROS 2 Humble Docs](https://docs.ros.org/en/humble/)
- [AirSim Docs](https://microsoft.github.io/AirSim/)

## 许可证

Apache-2.0

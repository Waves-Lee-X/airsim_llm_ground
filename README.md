# AeroMind — 无人机自主飞行系统

基于 ROS 2 Humble 的模块化无人机自主飞行平台，支持 AirSim 视觉仿真、PX4 飞控固件。

---

## 1. 项目定位

AeroMind 是一个**多模块协作**的无人机自主飞行平台。上层由 LLM Agent 接收自然语言任务，调用路径规划生成航点，通过飞行控制模块驱动无人机执行。底层对接 PX4 飞控，仿真阶段可选 AirSim（视觉+激光雷达）或 PX4 自带 Gazebo/SIH。

**当前状态**：
- ROS 2 模块框架完成（7 个包，全部编译通过）
- PX4 SITL 连接打通，Gazebo 和 SIH 两种仿真模式可用
- ROS 2 arm → takeoff → 路径跟随 → land 全链路可实现
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
Perception ◀── /sensor/camera/image            Control ──Takeoff/Land/Arm.srv
    │                                                │
    │  /perception/detection                         │  /fmu/in/vehicle_command
    │  /perception/obstacle_map                      │  /fmu/in/trajectory_setpoint
    ▼                                                ▼
Planning ◀────────────────────────────────── Bridge ──▶ PX4 / AirSim
                                                  │
                                                  │  /sensor/odometry
                                                  │  /sensor/imu
                                                  │  /control/drone_state
                                                  ▼
                                            全部模块可消费
```

### 3.2 包列表（7 个）

| 包名 | 类型 | 职责 | 负责人/状态 |
|------|------|------|------------|
| `aeromind_interfaces` | ament_cmake | 4 msg + 5 srv 定义 | **公共依赖，所有人** |
| `aeromind_bridge` | ament_python | AirSim/PX4 ↔ ROS 2 传感器与控制桥接 | 组员 A |
| `aeromind_perception` | ament_python | YOLO 检测 + OctoMap 建图（占位） | 组员 B |
| `aeromind_planning` | ament_python | 路径规划服务（直线插值→A*） | 组员 C |
| `aeromind_control` | ament_python | 飞行控制服务（arm/takeoff/land/路径跟随） | 组员 D |
| `aeromind_agent` | ament_python | LLM 任务调度（服务链编排） | 组员 E |
| `aeromind_bringup` | ament_cmake | 启动文件汇总 | 公共 |

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

# === 4. 安装 MicroXRCE-DDS Agent（PX4 ↔ ROS 2 协议桥） ===
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
# 应输出 7 个 aeromind_* 包

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

# 终端 4（Windows） — 启动 AirSim
```

### 5.4 起飞测试

```bash
cd ~/aeromind_ws && source install/setup.bash

# 解锁
ros2 service call /control/arm aeromind_interfaces/srv/ArmDrone "{arm: true}"

# 起飞到 10m
sleep 1
ros2 service call /control/takeoff aeromind_interfaces/srv/Takeoff "{altitude: 10.0}"

# 降落
ros2 service call /control/land aeromind_interfaces/srv/Land "{}"
```

---

## 6. PX4 仿真参数速查

不同仿真模式下需要设的参数不同。`param save` 会将参数持久化到 `build/px4_sitl_default/rootfs/parameters.bson`。

### 6.1 SIH 模式（最简）

```
param set SYS_HAS_MAG 0          # SIH 无磁罗盘仿真
param save
```

### 6.2 Gazebo 模式

Gazebo 提供完整传感器仿真，通常无需额外参数。如果遇到预检失败：

```
param set SYS_HAS_MAG 0          # 如果磁罗盘数据异常
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

## 7. 模块开发指南（分发给各组员）

### 7.1 公共基础：aeromind_interfaces

**所有组员必须先读**。定义了模块间通信的全部消息和服务。

| 接口名 | 类型 | 字段 |
|--------|------|------|
| `Detection.msg` | msg | `class_name`, `confidence`, `x/y/width/height` |
| `ObstacleMap.msg` | msg | `timestamp`, `width/height/resolution`, `data[]` |
| `Path.msg` | msg | `header`, `waypoints[]` (PoseStamped 数组) |
| `DroneState.msg` | msg | `armed`, `mode`, `battery`, `gps_fix`, `ekf_healthy` |
| `PlanPath.srv` | srv | request: `start` `goal` (Pose); response: `path` `success` `message` |
| `Takeoff.srv` | srv | request: `altitude`; response: `success` `message` |
| `Land.srv` | srv | 无请求; response: `success` `message` |
| `ArmDrone.srv` | srv | request: `arm`; response: `success` `message` |
| `ExecuteTask.srv` | srv | request: `task_description`; response: `success` `result` `message` |

> **修改接口前务必通知全员**。接口变更 = 所有依赖方需要重新编译。

### 7.2 aeromind_bridge（组员 A）

**职责**：抽象底层飞控/仿真器差异，向上提供统一的传感器话题。

**文件**：
- `airsim_bridge_node.py` — 主节点，根据 `mode` 参数切换后端
- `px4_bridge.py` — PX4 NED↔ENU 坐标转换 + px4_msgs↔ROS 消息转换

**输入**（PX4 模式）：
| Topic | 消息类型 | 来源 |
|-------|---------|------|
| `/fmu/out/vehicle_local_position_v1` | px4_msgs/VehicleLocalPosition | PX4 EKF2 |
| `/fmu/out/vehicle_attitude` | px4_msgs/VehicleAttitude | PX4 Sensors |
| `/fmu/out/sensor_combined` | px4_msgs/SensorCombined | PX4 Sensors |
| `/fmu/out/vehicle_status_v4` | px4_msgs/VehicleStatus | PX4 Commander |
| `/control/cmd_vel` | geometry_msgs/Twist | 外部速度指令 |

**输出**：
| Topic | 消息类型 | 频率 |
|-------|---------|------|
| `/sensor/odometry` | nav_msgs/Odometry | 10Hz |
| `/sensor/imu` | sensor_msgs/Imu | 10Hz |
| `/control/drone_state` | aeromind_interfaces/DroneState | 按事件 |

**坐标转换关键**：PX4 NED (x=北,y=东,z=下) ↔ ROS ENU (x=东,y=北,z=上)。
所有转换在 `px4_bridge.py` 中集中处理。

**QoS 注意事项**：PX4 uXRCE-DDS 发布者使用 `BEST_EFFORT` 可靠性，ROS 2 订阅必须匹配，否则收不到数据。代码中已配置。

**待完成**：
- AirSim 模式下的相机图像桥接（`/sensor/camera/image`）
- PX4 模式下补充电池/GPS/EKF 状态的精确填充

### 7.3 aeromind_perception（组员 B）

**职责**：目标检测 + 环境建图。

**文件**：`perception_node.py`

**输入**：
| Topic | 消息类型 | 状态 |
|-------|---------|------|
| `/sensor/camera/image` | sensor_msgs/Image | 预留，等 bridge 实现后可用 |

**输出**：
| Topic | 消息类型 | 频率 | 状态 |
|-------|---------|------|------|
| `/perception/detection` | aeromind_interfaces/Detection | 1Hz | 占位 |
| `/perception/obstacle_map` | aeromind_interfaces/ObstacleMap | 1Hz | 占位 |

**待实现**：
- 订阅 `/sensor/camera/image`，接入 YOLOv8 ONNX 模型推理
- 将推理结果（类别、置信度、边界框）填充为 `Detection` 消息发布
- 订阅 `/sensor/odometry` + 深度图/激光雷达 → 构建 OctoMap
- 将占据栅格地图发布为 `ObstacleMap`

**开发建议**：
- 先在不连飞控的情况下测试 YOLO，用 ROS 2 bag 或本地图片
- 确认 Detection 消息格式后，再接入实时相机流

### 7.4 aeromind_planning（组员 C）

**职责**：路径规划服务。

**文件**：`planning_node.py`

**服务**：
| 服务 | 类型 | 说明 |
|------|------|------|
| `/planning/plan_path` | aeromind_interfaces/PlanPath | 输入 start+goal，返回 Path |

**输入**：
| Topic | 消息类型 | 用途 |
|-------|---------|------|
| `/perception/obstacle_map` | aeromind_interfaces/ObstacleMap | 避障用（当前仅打印日志） |

**当前实现**：start 和 goal 之间直线插值 10 个航点。

**待实现**：
- A* 全局路径规划（基于 ObstacleMap 栅格地图）
- 局部避障（动态窗口法 DWA 或类似方案）
- 路径平滑（B-spline）
- 中间航点高度优化（地形跟随）

**开发建议**：
- 先在 Python 里写纯算法，用 numpy 测试
- 确认 ObstacleMap 数据格式后接入真实地图
- 可以用 RViz2 可视化路径结果

### 7.5 aeromind_control（组员 D）

**职责**：飞行控制 — 对外暴露 arm/takeoff/land 服务，执行路径跟随。

**文件**：
- `control_node.py` — 主节点，切换到 PX4/AirSim 后端
- `px4_control.py` — PX4 Offboard 控制器（VehicleCommand + TrajectorySetpoint）

**服务**：
| 服务 | 类型 | 实现 |
|------|------|------|
| `/control/arm` | ArmDrone | PX4: VehicleCommand(400); AirSim: armDisarm |
| `/control/takeoff` | Takeoff | PX4: arm→offboard→VehicleCommand(22); AirSim: takeoffAsync |
| `/control/land` | Land | PX4: VehicleCommand(21); AirSim: landAsync |

**输入**：
| Topic | 消息类型 | 用途 |
|-------|---------|------|
| `/planning/path` | aeromind_interfaces/Path | 路径跟随 |

**输出**：
| Topic | 消息类型 | 说明 |
|-------|---------|------|
| `/control/drone_state` | aeromind_interfaces/DroneState | 飞控状态（PX4 模式下从 VehicleStatus 解析） |

**PX4 控制流程**：
1. `Arm` → VehicleCommand(400, param1=1.0)
2. `Takeoff` → VehicleCommand(176, offboard 模式) + 10Hz OffboardControlMode 心跳 + VehicleCommand(22, takeoff)
3. 路径跟随 → 10Hz OffboardControlMode(position) + 逐航点 TrajectorySetpoint
4. `Land` → VehicleCommand(21)

**待完成**：
- 航点到达判断（当前用固定 2s 延时，应改为位置误差阈值判断）
- PX4 模式下的降落逻辑优化（切换到 Land 模式而非 Offboard）
- 异常处理：failsafe 检测与恢复

### 7.6 aeromind_agent（组员 E）

**职责**：接收自然语言任务，编排下游服务调用链。

**文件**：`agent_node.py`

**服务**：
| 服务 | 类型 | 说明 |
|------|------|------|
| `/agent/execute_task` | aeromind_interfaces/ExecuteTask | 接收任务描述，执行调用链 |

**调用链**（当前占位实现）：
1. `execute_task` → 解析任务
2. → 调用 `/planning/plan_path`（硬编码 start/goal）
3. → 调用 `/control/takeoff`
4. → 返回结果

**参数**：
- `llm_api_url`：LLM 服务地址（默认 `http://localhost:11434/v1`，适配 Ollama）
- `llm_model`：模型名（默认 `llama3`）

**待实现**：
- 接入 LLM（Ollama / OpenAI API）解析自然语言为结构化任务
- 工具调用链：planning → control → perception 的编排
- 多任务调度：任务队列 + 优先级
- 任务执行状态反馈

**开发建议**：
- 先用硬编码测试调用链，确认 planning → control 链路正常
- 接入 LLM API 做自然语言 → JSON 任务解析
- 加 `ros2 service call` 测试每个环节

---

## 8. 多人协作开发规范

### 8.1 分支策略

```
main          ← 稳定版本（所有人的接口变更最终合入这里）
├── feature/bridge-xxx    ← 组员 A
├── feature/perception-xxx ← 组员 B
├── feature/planning-xxx  ← 组员 C
├── feature/control-xxx   ← 组员 D
└── feature/agent-xxx     ← 组员 E
```

### 8.2 接口变更流程

1. **修改前**在群内通知："我要改 `aeromind_interfaces/msg/Path.msg`，加一个 `speed` 字段"
2. 修改 `.msg` / `.srv` 文件
3. **全量编译**：`colcon build --symlink-install`（因为有依赖链）
4. 自己模块测试通过后，**push 到自己的 feature 分支**
5. 其他组员 `git pull` + `colcon build` 更新接口

### 8.3 模块独立测试

每个组员可以在**自己的分支上**独立开发，不需要等其他人。关键是：

- **不改 aeromind_interfaces 的情况下**，只改自己的 `.py`，编译仅需 `--packages-select 你的包名`
- 需要联调时，两个组员都启动自己的节点，通过 ROS 2 topic/service 通信
- 测试数据流：用 `ros2 topic echo` 和 `ros2 service call` 验证输入输出

### 8.4 推荐开发方式

```
# 不依赖其他人的节点启动全部系统，只测试自己的：
colcon build --symlink-install --packages-select aeromind_planning
ros2 run aeromind_planning planning_node

# 另开终端，手动调自己的服务看是否正确：
ros2 service call /planning/plan_path aeromind_interfaces/srv/PlanPath "{...}"
```

### 8.5 Git 忽略

`build/`、`install/`、`log/` 已在 `.gitignore` 中。不要在仓库里提交编译产物。
`parameters.bson` 这类 PX4 本地文件也不需要提交。

---

## 9. 故障排查

| 现象 | 可能原因 | 解决 |
|------|---------|------|
| bridge 日志 "incompatible QoS" | PX4 用 BEST_EFFORT，ROS 用 RELIABLE | 已修复：代码中设了 `ReliabilityPolicy.BEST_EFFORT` |
| bridge 报 "The 'x' field must be of type 'float'" | PX4 消息字段是 numpy float32 | 已修复：`px4_bridge.py` 中显式 `float()` 转换 |
| ros2 topic echo 无数据 | MicroXRCE-DDS Agent 未启动或 QoS 不匹配 | 检查 Agent 是否在跑，检查 bridge 日志无 QoS 警告 |
| `commander arm` 拒绝 | 预检未通过 | 执行 `commander check` 看失败的项 |
| ROS arm 成功但立即 disarm | 有 failsafe 触发 | 看 PX4 日志找具体 failsafe 类型 |
| PX4 连不上 AirSim | IP 变了或防火墙 | `hostname -I` 确认 WSL IP，检查 Windows 防火墙 4560 |
| colcon build 失败 | 缺依赖或接口变了 | `rm -rf build/ install/ && colcon build --symlink-install` |
| Gazebo 窗口空白 | WSL2 无 GPU 加速 | 等几分钟或用 SIH 模式替代 |

---

## 10. 目录结构

```
aeromind_ws/
├── src/
│   ├── aeromind_interfaces/     # 消息/服务定义（ament_cmake，公共依赖）
│   │   ├── msg/                 # Detection, ObstacleMap, Path, DroneState
│   │   ├── srv/                 # PlanPath, Takeoff, Land, ArmDrone, ExecuteTask
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── aeromind_bridge/         # 桥接节点（ament_python，组员 A）
│   │   ├── aeromind_bridge/
│   │   │   ├── airsim_bridge_node.py
│   │   │   └── px4_bridge.py    # PX4 NED↔ENU 转换工具
│   │   ├── setup.py / setup.cfg / package.xml
│   ├── aeromind_perception/     # 感知节点（ament_python，组员 B）
│   ├── aeromind_planning/       # 路径规划节点（ament_python，组员 C）
│   ├── aeromind_control/        # 飞行控制节点（ament_python，组员 D）
│   │   ├── aeromind_control/
│   │   │   ├── control_node.py
│   │   │   └── px4_control.py   # PX4 Offboard 控制器
│   ├── aeromind_agent/          # LLM Agent 节点（ament_python，组员 E）
│   ├── aeromind_bringup/        # 启动文件（ament_cmake）
│   │   └── launch/
│   │       ├── aeromind_all.launch.py   # AirSim 直连模式
│   │       └── aeromind_px4.launch.py   # PX4 模式
│   └── px4_msgs/                # PX4 消息定义（从 GitHub 克隆）
├── doc/
│   └── PX4_AirSim_配置说明.md    # AirSim + PX4 详细配置文档
├── AeroMind Console/            # 旧项目代码（不动，参考用）
├── ground_station_qt_airsim/    # 旧项目地面站（不动）
├── README.md
└── .gitignore
```

## 11. 相关文档

- [PX4 + AirSim 配置详细说明](doc/PX4_AirSim_配置说明.md) — 网络配置、AirSim settings.json、PX4 参数详解
- [PX4 User Guide](https://docs.px4.io/main/en/)
- [ROS 2 Humble Docs](https://docs.ros.org/en/humble/)
- [AirSim Docs](https://microsoft.github.io/AirSim/)

## 许可证

Apache-2.0

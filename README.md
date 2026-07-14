# AeroMind — 无人机自主飞行系统

基于 ROS 2 Humble 的模块化无人机自主飞行平台，支持 AirSim 视觉仿真、PX4 飞控固件。

---

## 1. 项目定位

AeroMind 是一个**多模块协作**的无人机自主飞行平台。上层由 LLM Agent 接收自然语言任务，调用路径规划生成航点，通过飞行控制模块驱动无人机执行。底层对接 PX4 飞控，仿真阶段可选 AirSim（视觉+激光雷达）或 PX4 自带 Gazebo/SIH。

**当前状态**：
- ROS 2 模块框架完成（含 Web、Agent、Perception、Autonomy、Control、Bridge 等包）
- PX4 SITL 连接打通，Gazebo 和 SIH 两种仿真模式可用
- ROS 2 arm → takeoff → 路径跟随 → land 全链路可实现
- Web 地面站已支持状态、相机、深度点云、目标检测、自然语言任务、执行链监控
- Agent 已支持 LLM/规则解析、人工确认、MissionManager 复合任务状态机
- 感知模块已支持 RGB 保存、YOLO detection、基础图像语义分析服务
- 自主避障已具备 Local ESDF-style map、kinodynamic replanning、基础恢复绕行策略

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
│   ┌─────────────┐                                                │
│   │ aeromind_   │   HTTP/Web UI                                  │
│   │ web         │◀──────────────────── Browser                   │
│   │ 控制台       │                                                │
│   └──────┬──────┘                                                │
│          │ ROS topic/service                                     │
│          ▼                                                       │
│   ┌─────────────┐     ┌──────────────┐     ┌────────────┐       │
│   │ aeromind_    │     │ aeromind_    │     │ aeromind_  │       │
│   │ agent        │────▶│ autonomy     │────▶│ control    │       │
│   │ LLM + 任务状态机│    │ 实时避障/轨迹 │     │ 飞行控制    │       │
│   └──────┬──────┘     └──────┬───────┘     └─────┬──────┘       │
│          │                    ▲                   │              │
│          ▼                    │                   │              │
│   ┌─────────────┐             │                   │              │
│   │ aeromind_    │────────────┘                   │              │
│   │ planning     │  传统规划/路径服务              │              │
│   └─────────────┘                                 │              │
│                              │                    │              │
│                              ▼                    │              │
│                       ┌──────────────┐            │              │
│                       │ aeromind_    │            │              │
│                       │ perception   │            │              │
│                       │ 感知 (YOLO/VLM)│           │              │
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

### 3.2 包列表

| 包名 | 类型 | 职责 | 负责团队 |
|------|------|------|------------|
| `aeromind_interfaces` | ament_cmake | msg/srv 定义，跨模块通信协议 | A. 平台与接口 |
| `px4_msgs` | ament_cmake | PX4 官方消息定义（200+ msg） | 公共依赖 |
| `aeromind_bridge` | ament_python | AirSim/PX4 ↔ ROS 2 传感器与控制桥接 | 组员 A |
| `aeromind_perception` | ament_python | RGB 保存、YOLO 检测、图像语义分析/VLM 入口 | 组员 B |
| `aeromind_autonomy` | ament_python | Local ESDF-style map、实时重规划、恢复绕行、轨迹输出 | 组员 C |
| `aeromind_planning` | ament_python | 传统路径规划服务 | 组员 C |
| `aeromind_control` | ament_python | 飞行控制服务（arm/takeoff/land/RTL/轨迹执行） | 组员 D |
| `aeromind_agent` | ament_python | LLM 任务调度、技能库、MissionManager 复合任务状态机 | 组员 E |
| `aeromind_web` | ament_python | ROS 网关 + Web 控制台 | 组员 F |
| `aeromind_teleop` | ament_python | 键盘遥控（位置/速度双模式） | 辅助工具 |
| `aeromind_bringup` | ament_cmake | 启动文件汇总 + RViz 配置 | 公共 |

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
# 应输出 aeromind_agent / aeromind_web / aeromind_autonomy 等 aeromind_* 包

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

**Windows 侧**：AirSim settings.json 配置见 [PX4 与 AirSim 配置说明](doc/PX4与AirSim配置说明.md)

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

# 返航（PX4 原生 RTL）
ros2 service call /control/return_home aeromind_interfaces/srv/ReturnHome "{}"

# 保存当前前视 RGB 图片
ros2 service call /perception/capture_image aeromind_interfaces/srv/CaptureImage "{label: front_rgb}"

# 分析当前 RGB 画面（默认规则 + detection 摘要；配置 VLM 后可调用视觉大模型）
ros2 service call /perception/analyze_image aeromind_interfaces/srv/AnalyzeImage \
  "{prompt: '分析当前画面中有什么，是否有风险', use_vlm: false}"
```

起飞后可用下面命令确认状态。正常情况下 `mode` 应进入 `OFFBOARD`：

```bash
ros2 topic echo /control/drone_state
```

### 5.6 Web 控制台 / 独立地面站

Web 控制台用于把常用 ROS topic/service 集成到浏览器界面，适合演示、联调和后续自然语言控制扩展。

当前已经拆成两层：

```text
独立 Web 地面站（web/ground_station）
  ↓ HTTP API
ROS 网关（aeromind_web.web_console_node）
  ↓ ROS 2 topic/service
PX4 / AirSim / Control / Agent / Planning / Perception
```

这意味着前端可以作为普通网页运行在地面站电脑上；ROS 网关可以运行在仿真电脑、WSL、真机伴随计算机或机载计算机上。前端不直接依赖 ROS，只需要能访问 ROS 网关的 HTTP 地址。

先启动 PX4/AirSim 主系统：

```bash
# 终端 1-3: 按 5.1/5.2/5.3 任一模式启动
ros2 launch aeromind_bringup aeromind_px4.launch.py
```

再启动 ROS 网关：

```bash
cd ~/aeromind_ws && source install/setup.bash
ros2 launch aeromind_bringup aeromind_web.launch.py
```

方式 A：直接访问 ROS 网关内置页面：

```text
http://localhost:8080
```

方式 B：启动独立 Web 地面站（推荐后续真机迁移使用）：

```bash
cd ~/aeromind_ws/web/ground_station
python3 -m http.server 5173
```

浏览器打开：

```text
http://localhost:5173
```

页面顶部“ROS 网关”默认是：

```text
http://localhost:8080
```

如果 ROS 网关在真机/伴随计算机上运行，改成：

```text
http://<ROS_GATEWAY_IP>:8080
```

如果端口被占用，可以换端口：

```bash
ros2 launch aeromind_bringup aeromind_web.launch.py port:=8081
```

WSL2 中如果从 Windows 浏览器访问失败，可查看 WSL IP 后使用 `http://<WSL_IP>:8080`：

```bash
hostname -I
```

当前 Web 控制台功能：

| 区域 | 功能 | ROS 接口 |
|------|------|----------|
| 顶部状态栏 | ROS 网关地址、连接状态、飞行模式、EKF、服务状态 | `/api/status` |
| AI 飞行助手 | 对话输入、AI 回复、解析结果、执行反馈 | `/agent/execute_task` |
| 解析结果 | intent、skill、risk、confirm、原始 JSON | Agent 返回的 `parsed_task` |
| 执行链监控 | LLM 输出、计划步骤、ROS 工具调用 | Agent 返回的 `tool_calls` |
| 复合任务态势 | 多步任务当前阶段、完成/失败步骤、失败原因 | `/agent/mission_status` → `/api/status` |
| 技能库 | 状态检查、解锁、起飞、降落等快捷任务 | `/agent/execute_task` |
| 飞行状态 | 解锁、模式、电池、GPS、EKF、里程计 | `/control/drone_state`, `/sensor/odometry` |
| 前视 RGB 相机 | RGB 相机画面 | `/sensor/camera/rgb/front_center` |
| 深度相机 | 分辨率、编码、中心深度、最近/最远有效深度 | `/sensor/camera/depth/front_center` |
| Three.js 深度点云 | 深度图生成的真 3D 点云、坐标网格、AxisColor 着色、鼠标旋转/滚轮缩放 | `/api/depthcloud` ← `/sensor/camera/depth/front_center` |
| LiDAR 点云 | LiDAR 点云兜底数据源 | `/sensor/lidar/points` |
| 飞行控制 | 解锁、加锁、起飞、降落、返航 | `/control/arm`, `/control/takeoff`, `/control/land`, `/control/return_home` |
| 虚拟控制 | 前后左右、升降、偏航、悬停 | `/control/cmd_vel` |
| 自然语言控制 | 自然语言任务输入 | `/agent/execute_task` |
| 图像语义分析 | 规则/YOLO/VLM 画面理解、风险和建议 | `/perception/analyze_image` |
| 事件日志 | 服务返回、错误提示、执行日志 | Web 后端汇总 |

手动控制区的 `起飞高度 m` 默认值 `10` 表示点击“起飞”按钮时调用 `/control/takeoff` 的目标高度为 10 米；它不是手动移动速度。前进、后退、上升、下降、偏航等虚拟控制当前使用前端代码中的固定速度值，后续可升级为速度滑块。

点云区域会优先动态加载 Three.js 渲染真 3D 点云；如果浏览器无法访问 Three.js CDN，会自动回退到 Canvas 点云视图，控制台不会白屏。点云工具条支持重置视角、点大小调节、颜色模式切换（轴向/高度/距离）。

状态和日志支持两条链路：优先通过 `/ws` WebSocket 每秒实时推送；WebSocket 不可用时保留 HTTP 轮询兜底。

Web 后端 HTTP API：

| API | 方法 | 说明 |
|-----|------|------|
| `/api/status` | GET | 获取状态、里程计、检测结果、服务就绪状态 |
| `/api/skills` | GET | 获取 AeroMind Skills 技能库，供前端动态渲染 |
| `/api/camera` | GET | 获取最新 RGB 图像帧 |
| `/api/depthcloud` | GET | 获取由深度相机采样生成的 RViz DepthCloud 风格点云 |
| `/api/pointcloud` | GET | 获取 LiDAR 点云抽样点和元数据 |
| `/api/control/arm` | POST | 解锁/加锁，body: `{"arm": true}` |
| `/api/control/takeoff` | POST | 起飞，body: `{"altitude": 10}` |
| `/api/control/land` | POST | 降落 |
| `/api/control/return_home` | POST | PX4 原生 RTL 返航 |
| `/api/cmd_vel` | POST | 发布虚拟控制速度 |
| `/api/agent/task` | POST | 提交自然语言任务 |
| `/api/agent/chat` | POST | 对话式任务入口，复用 `/agent/execute_task`，前端带 session/history |
| `/ws` | WebSocket | 实时推送状态、服务就绪状态和事件日志 |

### 5.7 底层实时自主避障

当前项目已新增 `aeromind_autonomy` 包，作为无人机底层自主避障的独立模块。它不是 A* 栅格规划路线，而是按照后续真实自主飞行技术栈来设计：

```text
Depth / registered PointCloud2 / VIO Odometry / Goal
  → Rolling World-frame ESDF-style Voxel Map
  → Kinodynamic Replanning
  → Seventh-order Minimum-Snap Trajectory
  → /autonomy/trajectory
  → control_node velocity / position setpoint
```

当前实现是第一版可运行基础：

| 模块 | 当前实现 | 后续替换方向 |
|------|----------|--------------|
| 定位 | 标准 `nav_msgs/Odometry`，默认 `/sensor/odometry`，可切换 VIO/SLAM topic | VINS-Fusion、OpenVINS、ORB-SLAM3 等外部定位器 |
| 建图 | 深度点经机体姿态变换到世界系；滚动体素、射线清障、时间衰减；可融合世界系注册点云 | Voxblox / FIESTA / NVBlox 的预计算 ESDF |
| 重规划 | 动力学受限角度扇区运动基元，优先选择安全候选，再按进度、clearance 和连续性评分 | Fast-Planner / EGO-Planner 全局一致重规划 |
| 轨迹 | 七次单段 minimum-snap，端点速度、加速度和 jerk 为零 | 带连续航点约束的多段 minimum-snap/B-spline 优化 |
| 控制 | `control_node` 执行 `/autonomy/trajectory`，带速度限幅、加速度平滑、高度保护和控制互斥 | PX4 trajectory setpoint 轨迹跟踪 |

当前已补齐的飞行保护逻辑：

- 任务生命周期：到达目标后清空 `/autonomy/goal`，阻塞超过阈值后进入 `BLOCKED_HOLD`，不再无限执行同一条轨迹。
- 基础恢复绕行：阻塞超时后先尝试右绕、左绕、上升三个恢复目标；恢复点到达后继续原始目标，恢复策略用尽后才安全悬停。
- 到达判定：距离和速度同时满足阈值才算完成，减少目标点附近来回摆动。
- 避障滞回：进入阻塞后需要更大的 clearance 才恢复前进，减少障碍物边界处反复切换。
- 深度过滤：剔除过近噪声、过远点和视野边缘异常点，降低误触发 `safe_hover` 的概率。
- 世界坐标融合：深度相机点使用里程计四元数转换到世界系；相机画面右侧对应机体系右侧，避免左右镜像。
- 滚动地图：射线内部清障、障碍时间衰减和局部半径裁剪，避免无人机移动后旧障碍固定在错误位置。
- 定位保护：里程计或深度超时进入 `SENSOR_WAIT` 悬停；检测到 VIO/里程计跳变时清空局部地图。
- VIO 质量门控：拒绝 NaN、异常四元数，以及超过阈值的位置/姿态协方差；连续无有效定位后自动悬停。
- TF 外参：桥接节点动态发布 `odom -> base_link`，传感器静态挂载到 `base_link`；深度图通过 optical frame 的 TF 融合。
- 地图可视化：以世界坐标发布 `/autonomy/esdf_obstacles`，RViz 可与原始 DepthCloud 同屏检查。
- 安全候选优先：直线路径碰撞时会优先选择通过 clearance 校验的侧绕或爬升候选，不再让危险直线分数压过安全绕行。
- 控制保护：自主轨迹执行有速度上限、加速度平滑、最低高度保护；降落期间忽略自主轨迹，起飞期间不被 `hover_no_goal` 打断。
- 取消任务：Web、Agent 或命令行可发布 `/autonomy/cancel`，清空当前自主目标并进入悬停。

发布接口：

| Topic | 类型 | 说明 |
|------|------|------|
| `/autonomy/status` | `aeromind_interfaces/AutonomyStatus` | 自主避障状态、最近障碍物、目标距离、当前策略 |
| `/autonomy/trajectory` | `aeromind_interfaces/Trajectory` | 局部平滑轨迹点，包含位置、速度、加速度 |
| `/autonomy/goal` | `geometry_msgs/PoseStamped` | 自主避障局部目标输入 |
| `/autonomy/cancel` | `std_msgs/Empty` | 取消当前自主目标，进入悬停保持 |
| `/autonomy/esdf_obstacles` | `sensor_msgs/PointCloud2` | 规划器实际使用的世界系占据体素中心 |
| `/control/cmd_vel` | `geometry_msgs/Twist` | 仅当 `autonomy_control:=true` 时由 autonomy 节点发布 |

启动时默认会运行自主避障状态机，并让 `control_node` 订阅执行 `/autonomy/trajectory`。`autonomy_node` 默认不会直接发布 `/control/cmd_vel`，避免和轨迹执行通道抢控制：

```bash
ros2 launch aeromind_bringup aeromind_px4.launch.py
```

相关启动参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `autonomy_enabled` | `true` | 是否启用自主避障重规划 |
| `autonomy_trajectory_execution` | `true` | 是否允许 `control_node` 执行 `/autonomy/trajectory` |
| `autonomy_control` | `false` | 是否允许 `autonomy_node` 直接发布 `/control/cmd_vel` |
| `autonomy_velocity_limit` | `1.2` | `control_node` 执行自主轨迹时的速度上限，单位 m/s |
| `autonomy_accel_limit` | `0.8` | 自主速度指令变化上限，单位 m/s^2 |
| `autonomy_min_altitude` | `1.0` | 自主轨迹执行时的最低高度保护，单位 m |
| `autonomy_blocked_timeout_sec` | `3.0` | 连续阻塞多久后触发恢复策略；恢复策略用尽后进入 `BLOCKED_HOLD` |
| `autonomy_min_depth_m` | `0.7` | 深度避障最小有效距离，低于该值视为噪声 |
| `autonomy_max_depth_m` | `18.0` | 深度避障最大有效距离 |
| `autonomy_map_radius_m` | `24.0` | 世界坐标滚动地图半径，单位 m |
| `autonomy_map_decay_sec` | `4.0` | 未再次观测障碍体素的保留时间 |
| `autonomy_odom_timeout_sec` | `1.0` | 里程计超时后进入悬停的阈值 |
| `autonomy_depth_timeout_sec` | `3.5` | 深度/注册点云超时后禁止盲飞的阈值 |
| `autonomy_odom_topic` | `/sensor/odometry` | PX4、VIO 或 SLAM 的里程计输入 |
| `autonomy_world_cloud_topic` | 空 | 可选的世界坐标注册 `PointCloud2` 输入 |
| `autonomy_max_position_variance` | `2.0` | VIO/SLAM 位置协方差拒绝阈值 |
| `autonomy_max_orientation_variance` | `0.5` | VIO/SLAM 姿态协方差拒绝阈值 |

推荐使用默认方式：`autonomy_trajectory_execution:=true`，`autonomy_control:=false`。

如果只想观察避障状态，不执行轨迹：

```bash
ros2 launch aeromind_bringup aeromind_px4.launch.py autonomy_trajectory_execution:=false
```

如果确认要让底层避障直接发布速度控制，再显式开启；不要和轨迹执行通道同时用于实飞测试：

```bash
ros2 launch aeromind_bringup aeromind_px4.launch.py \
  autonomy_trajectory_execution:=false \
  autonomy_control:=true
```

发布一个目标点示例：

```bash
ros2 topic pub /autonomy/goal geometry_msgs/msg/PoseStamped "{
  header: {frame_id: odom},
  pose: {position: {x: 20.0, y: 0.0, z: 10.0}, orientation: {w: 1.0}}
}" --once
```

接入外部 VIO/SLAM 时，里程计和注册点云必须使用同一世界 frame。例如：

```bash
ros2 launch aeromind_bringup aeromind_px4.launch.py \
  autonomy_odom_topic:=/vio/odometry \
  autonomy_world_cloud_topic:=/slam/registered_cloud
```

`/slam/registered_cloud.header.frame_id` 必须与 `/vio/odometry.header.frame_id` 完全一致，否则点云会被拒绝。当前代码提供标准输入边界，但不会自动启动 VINS-Fusion、OpenVINS 或 ORB-SLAM3；选定定位器后仍需完成相机标定、时间同步和外参标定。

查看避障状态：

```bash
ros2 topic echo /autonomy/status
ros2 topic echo /autonomy/trajectory
```

取消当前自主任务：

```bash
ros2 topic pub /autonomy/cancel std_msgs/msg/Empty "{}" --once
```

Web 控制台的“取消自主任务”按钮和自然语言“悬停/停止”都会走同一个取消链路。

> 安全说明：`autonomy_control:=true` 会让自主避障节点发布 `/control/cmd_vel`。实机或仿真飞行前，应先确认深度图、里程计、目标点和 Web 状态都正常，再开启控制接管。

`/api/agent/task` 当前会返回适合前端展示的结构化结果：

```json
{
  "reply": "我已解析任务：起飞到 10.0 米。请确认后再执行。",
  "parsed_task": {
    "intent": "takeoff",
    "skill": "TakeoffSkill",
    "args": {"altitude": 10.0},
    "risk_level": "low",
    "need_confirm": true,
    "parser": "llm",
    "llm_model": "llama3",
    "reason": "起飞到 10.0 米"
  },
  "tool_calls": [
    {"name": "get_drone_state", "target": "/control/drone_state", "status": "success"}
  ],
  "final_status": "等待用户确认",
  "pending_confirmation": {
    "token": "1780000000000-1",
    "intent": "takeoff",
    "skill": "TakeoffSkill",
    "args": {"altitude": 10.0},
    "summary": "起飞到 10.0 米"
  }
}
```

对于 `arm / disarm / takeoff / land` 这类真实控制动作，Agent 会先返回 `pending_confirmation`。前端点击“确认执行”后，才会发送确认令牌给 Agent，Agent 再调用真实 ROS 服务；点击“取消”不会调用任何控制服务。

真机迁移时，ROS 网关建议这样启动：

```bash
ros2 launch aeromind_bringup aeromind_web.launch.py host:=0.0.0.0 port:=8080
```

地面站电脑只运行 `web/ground_station` 静态前端即可。

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

## 7. 模块开发指南（分发给各组员）

### 7.1 公共基础：aeromind_interfaces

**所有组员必须先读**。定义了模块间通信的全部消息和服务。

| 接口名 | 类型 | 字段 |
|--------|------|------|
| `DroneState.msg` | msg | 飞控状态：armed、mode、battery、gps、ekf |
| `Detection.msg` | msg | 目标检测结果 |
| `DetectionArray.msg` | msg | 多目标检测结果数组 |
| `AutonomyStatus.msg` | msg | 自主避障状态、最近障碍物、目标距离、当前策略 |
| `TrajectoryPoint.msg` | msg | 自主轨迹中的单个轨迹点 |
| `Trajectory.msg` | msg | 自主避障输出的平滑轨迹 |
| `ObstacleMap.msg` | msg | 占据栅格地图 |
| `Path.msg` | msg | 航点序列 |
| `ArmDrone.srv` | srv | 解锁/加锁 |
| `Takeoff.srv` | srv | 起飞到指定高度 |
| `Land.srv` | srv | 降落 |
| `ReturnHome.srv` | srv | PX4 原生 RTL 返航 |
| `CaptureImage.srv` | srv | 保存最近一帧 RGB/灰度图像 |
| `AnalyzeImage.srv` | srv | 图像语义分析/VLM 入口 |
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
| `/control/return_home` | `ReturnHome` | PX4 原生 RTL 返航 |

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
| `/perception/detections` | `DetectionArray` | 多目标检测结果，供 Web 叠框和目标搜索使用 |
| `/perception/image_analysis` | `std_msgs/String` | 最近一次图像语义分析 JSON，供 Web、Agent、Autonomy 复用 |
| `/perception/obstacle_map` | `ObstacleMap` | 规划避障 |

**服务**：

| 服务 | 类型 | 用途 |
|------|------|------|
| `/perception/capture_image` | `CaptureImage` | 保存最近一帧 RGB/灰度图像，返回文件路径 |
| `/perception/analyze_image` | `AnalyzeImage` | 分析最近一帧 RGB 图像，输出场景、目标、风险等级和建议；可选调用 VLM |

图片保存格式：如果环境安装了 `Pillow`，默认保存为 `.png`；如果没有额外图像库，则使用 Python 标准库可直接写出的 `.ppm/.pgm` 裸图像格式。

图像语义分析分两种模式：

| 模式 | 说明 | 是否需要外部模型 |
|------|------|------------------|
| 规则 + detection 摘要 | 根据 RGB 可用性和 `/perception/detections` 生成场景、风险、建议 | 否 |
| VLM 视觉大模型 | 将当前 RGB 图像和检测摘要发送到 OpenAI-compatible Vision API | 是 |

`/perception/analyze_image` 每次完成后会发布 `/perception/image_analysis`。Agent 的任务报告、Web 对话流和 Autonomy 的恢复策略都会读取这个最近语义结果。当前 Autonomy 只把语义建议作为恢复方向优先级参考，例如 VLM/规则建议“右侧更安全”时优先尝试右绕；实际轨迹是否执行仍由 ESDF clearance、重规划结果和控制保护决定。

VLM 常用参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `vlm_enabled` | `false` | 是否启用视觉语言模型调用 |
| `vlm_api_url` | 空 | OpenAI-compatible Vision API 地址，例如 `https://api.openai.com/v1` |
| `vlm_model` | 空 | 视觉模型名，例如 `gpt-4o-mini`、`qwen-vl-plus` |
| `vlm_api_key` | 空 | VLM API Key，也可用环境变量 `AEROMIND_VLM_API_KEY` |
| `auto_analyze_enabled` | `false` | 是否启用自动低频图像语义分析 |
| `auto_analyze_interval_sec` | `8.0` | 自动分析间隔，单位秒 |
| `auto_analyze_use_vlm` | `false` | 自动分析是否调用 VLM；`false` 时只用规则 + detection 摘要 |

启动 VLM 示例：

```bash
export AEROMIND_VLM_API_KEY="你的 Vision API Key"

ros2 launch aeromind_bringup aeromind_px4.launch.py \
  yolo_enabled:=true \
  vlm_enabled:=true \
  vlm_api_url:=https://api.openai.com/v1 \
  vlm_model:=gpt-4o-mini \
  auto_analyze_enabled:=true \
  auto_analyze_use_vlm:=false
```

如果确实需要自动调用 VLM，可将 `auto_analyze_use_vlm:=true`，但会持续消耗 API 调用次数，演示前建议先用 `false` 验证链路。

如果使用阿里云百炼/Qwen-VL 等服务，只要服务兼容 OpenAI 的 `chat/completions + image_url` 格式，就可以替换为对应地址和模型名，例如 `qwen-vl-plus`。`deepseek-chat` 是文本模型，不适合直接做图像分析。

测试图像语义分析：

```bash
ros2 service call /perception/analyze_image aeromind_interfaces/srv/AnalyzeImage \
  "{prompt: '分析当前画面中有什么，是否建议继续前进', use_vlm: true}"
```

**近期任务**：
- 安装 `ultralytics` 后开启 YOLO 检测节点，输出检测框、类别、置信度
- Web 控制台已支持在 RGB 画面叠加检测框
- 用深度图或 LiDAR 生成基础障碍物地图

**验收标准**：
- 给定图像输入时能稳定输出 detection
- Web 控制台能显示检测框或目标列表
- obstacle_map 能被 Planning 消费

YOLO 检测启动示例：

```bash
# WSL/CPU 演示推荐：避免默认安装拉取 CUDA 版 torch
python3 -m pip install --user torch torchvision --index-url https://download.pytorch.org/whl/cpu
python3 -m pip install --user "numpy<2" "opencv-python==4.10.0.84" ultralytics

# 可选：提前下载模型，避免演示时首次启动等待下载
python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"

ros2 launch aeromind_bringup aeromind_px4.launch.py \
  yolo_enabled:=true \
  yolo_model:=yolov8n.pt \
  yolo_confidence:=0.35
```

检测输出：

```bash
ros2 topic echo /perception/detections
ros2 topic echo /perception/detection
```

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

**状态话题**：

| Topic | 类型 | 说明 |
|-------|------|------|
| `/agent/mission_status` | `std_msgs/String` | MissionManager 周期发布 JSON 快照，Web 控制台用于显示复合任务进度 |

**调用链**：
1. `execute_task` 接收自然语言任务
2. 优先调用 OpenAI-compatible LLM 解析为 `intent + skill + args + risk_level + need_confirm`
3. 如果 LLM 不可用、超时或返回非法 JSON，则自动回退到本地规则解析
4. 状态查询类任务立即执行；控制类任务先进行安全检查并返回 `pending_confirmation`
5. 起飞前安全检查包括飞控状态、目标高度、起飞服务、EKF 和 GPS/定位状态；未通过时不会生成确认按钮
6. 用户在 Web 控制台点击“确认执行”后，Agent 才调用受控 ROS 工具，例如 `/control/arm`、`/control/takeoff`、`/control/land`、`/control/return_home`
7. 复合任务进入 MissionManager，由 0.5s tick 推进每一步，持续发布 `/agent/mission_status`
8. 返回 `reply + parsed_task + tool_calls + progress + final_status`

> 重要：大模型只负责“理解任务和选择技能”，不直接发布底层 PX4/ROS 控制话题。真实控制始终由 Agent 的白名单 Skill 调用完成。

**参数**：
- `llm_api_url`：LLM 服务地址（默认 `http://localhost:11434/v1`，适配 Ollama）
- `llm_model`：模型名（默认 `llama3`）
- `llm_enabled`：是否启用 LLM（默认 `true`）
- `llm_timeout_sec`：LLM 解析超时时间（默认 `3.0` 秒）
- `llm_api_key`：LLM API Key。也可通过环境变量 `DEEPSEEK_API_KEY` 或 `OPENAI_API_KEY` 提供，推荐使用环境变量，避免密钥写进命令历史。
- `mission_log_dir`：任务报告保存目录，默认 `~/aeromind_ws/missions`；也可通过环境变量 `AEROMIND_MISSION_DIR` 配置。

**Ollama 本地大模型示例**：

```bash
# 终端 1：启动 Ollama，并确保模型已拉取
ollama serve
ollama pull qwen2.5:7b

# 终端 2：启动 AeroMind，并指定模型
cd ~/aeromind_ws && source install/setup.bash
ros2 launch aeromind_bringup aeromind_px4.launch.py \
  llm_enabled:=true \
  llm_api_url:=http://localhost:11434/v1 \
  llm_model:=qwen2.5:7b
```

如果不想等待大模型，可以关闭 LLM，使用规则解析：

```bash
ros2 launch aeromind_bringup aeromind_px4.launch.py llm_enabled:=false
```

**DeepSeek API 示例**：

DeepSeek 使用 OpenAI-compatible API。推荐先在终端设置环境变量：

```bash
export DEEPSEEK_API_KEY="你的 DeepSeek API Key"

cd ~/aeromind_ws && source install/setup.bash
ros2 launch aeromind_bringup aeromind_px4.launch.py \
  llm_enabled:=true \
  llm_api_url:=https://api.deepseek.com/v1 \
  llm_model:=deepseek-chat
```

也可以直接通过 launch 参数传入密钥，但不推荐长期这样做：

```bash
ros2 launch aeromind_bringup aeromind_px4.launch.py \
  llm_enabled:=true \
  llm_api_url:=https://api.deepseek.com/v1 \
  llm_model:=deepseek-chat \
  llm_api_key:=sk-xxxxxxxx
```

**当前支持**：
- “解锁” → `ArmSkill` → `/control/arm`
- “加锁” → `ArmSkill` → `/control/arm`
- “起飞到10米 / 起飞到二十米” → `TakeoffSkill` → `/control/takeoff`
- “降落” → `LandSkill` → `/control/land`
- “查询状态 / 检查电量 / 当前模式” → `StatusSkill` → `/control/drone_state`
- “检查前方有没有障碍物” → `PerceptionSkill` → 相机/深度/点云摘要分析
- “悬停 / 停止” → `HoverSkill` → `/control/cmd_vel` 零速度
- “飞到前方10米 / 向右飞5米” → `PlanningSkill` → `/autonomy/goal`
- “返航 / 返航并降落” → `ReturnHomeSkill` → `/control/return_home`，触发 PX4 原生 RTL
- “立即急停” → `EmergencyStopSkill` → 取消自主目标、零速度、尝试加锁
- “拍一张前方照片” → `CaptureImageSkill` → `/perception/capture_image`，保存当前 RGB 图像
- “扫描前方区域” → `ScanAreaSkill` → 传感器区域扫描摘要
- “检测人 / 检测person / 检测前方是否有汽车” → `TargetSearchSkill` → YOLO detection 优先，未命中时调用图像语义分析兜底
- “分析当前画面中有什么 / 描述前方图像” → `SemanticImageSkill` → `/perception/analyze_image`
- “起飞，向左飞20米，并检测人” → `MissionSequenceSkill` → 拆解为起飞、移动、目标检测多步任务
- “生成当前任务报告” → `MissionReportSkill` → 状态、感知、最近任务汇总

**当前技能库**：

技能定义已经模块化，核心技能位于 `aeromind_agent/skills/core.py`，扩展技能位于 `aeromind_agent/skills/future.py`，`skill_catalog.py` 负责汇总。ROS 网关通过 `/api/skills` 暴露给前端。后续团队成员新增技能时，优先新增独立 skill spec，再由 Agent 接入执行逻辑。

| Skill | 状态 | 说明 |
|-------|------|------|
| `StatusSkill` | 已接入 | 查询飞控状态 |
| `ArmSkill` | 已接入 | 解锁/加锁 |
| `TakeoffSkill` | 已接入 | 安全起飞 |
| `LandSkill` | 已接入 | 降落 |
| `HoverSkill` | 基础接入 | 发布零速度，进入悬停/停止运动 |
| `PlanningSkill` | 基础接入 | 自然语言相对移动目标 → `/autonomy/goal` |
| `ReturnHomeSkill` | 已接入 | 调用 `/control/return_home`，触发 PX4 原生 RTL |
| `EmergencyStopSkill` | 基础接入 | 取消自主目标、零速度、尝试加锁 |
| `PerceptionSkill` | 基础接入 | RGB、深度、LiDAR、detection 摘要分析 |
| `CaptureImageSkill` | 已接入 | 调用 `/perception/capture_image` 保存当前 RGB 图像，默认输出到 `~/aeromind_ws/missions/captures` |
| `ScanAreaSkill` | 基础接入 | 基于当前传感器缓存生成区域扫描摘要 |
| `TargetSearchSkill` | 增强接入 | YOLO detection 优先，未命中时调用 `/perception/analyze_image` 做语义搜索兜底 |
| `SemanticImageSkill` | 基础接入 | 调用 `/perception/analyze_image`，让规则/YOLO/VLM 参与画面理解 |
| `MissionSequenceSkill` | 已接入状态机 | 将起飞、相对移动、目标检测等复合任务拆解为多步执行链，并通过 `/agent/mission_status` 实时反馈 |
| `MissionReportSkill` | 基础接入 | 汇总飞控状态、里程计、感知结论和最近任务 |

**任务报告落盘**：

复合任务执行时，Agent 会为每个任务生成独立目录：

```text
~/aeromind_ws/missions/<mission_id>/
  mission.json
  report.md
  captures/
```

`mission.json` 会在任务创建、步骤开始、步骤完成、任务结束时持续更新；`report.md` 会在任务结束时生成。Agent 会在任务开始和结束时自动调用 `/perception/capture_image`，并把截图复制到 `captures/`。报告包含任务状态、执行步骤、飞行状态、感知摘要、图像语义分析、截图路径、失败原因和下一步建议。

Web 控制台的“任务态势”卡片会显示最近任务报告列表和截图缩略图。点击报告条目后，`report.md` 内容会显示在“结构化输出”区域；点击“下载”可以下载 `report.md`。

查看最近任务报告：

```bash
ls -td ~/aeromind_ws/missions/mission-* | head -1
```

**待实现**：
- 把基础版 skill 升级为独立可插拔执行器类，而不是集中在 `agent_node.py`
- 根据真机策略配置 PX4 RTL 参数，例如返航高度、失控保护、到达 home 后行为
- 多任务调度：任务队列 + 优先级
- 任务取消、暂停、恢复接口
- 接入云端 OpenAI / Claude / Gemini 等多模型适配器和模型选择界面

**开发建议**：
- 大模型只负责“理解和规划”，不要直接发布 `/fmu/in/*`
- Agent 必须做参数校验、安全检查和工具白名单控制
- 加 `ros2 service call /agent/execute_task ...` 测试每个自然语言任务

### 7.7 aeromind_teleop — 键盘遥控

- **节点**: `teleop_node`
- **参数**: `mode`（`velocity` / `position`，默认 `velocity`）
- **发布**:
  - `/control/cmd_vel` (geometry_msgs/Twist) — 速度或位置增量指令

**速度模式**（默认）：WASD 累积速度、松手渐变归零（`keyboard_velocity.py` 逻辑）。
**位置模式**：每次按键发送单次位移增量（`keyboard_position.py` 逻辑）。

| 按键 | 功能 |
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
└── base_link                         # 动态里程计 TF
    ├── camera_front_center_body      # 静态相机外参
    │   └── camera_front_center_optical
    └── lidar                         # 静态 LiDAR 外参
```

快速验证：

```bash
ros2 topic hz /sensor/camera/rgb/front_center
ros2 topic echo /sensor/camera/rgb/front_center --once
ros2 topic echo /sensor/camera/rgb/camera_info --once

ros2 topic hz /sensor/camera/depth/front_center
ros2 topic echo /sensor/camera/depth/camera_info --once
ros2 run tf2_ros tf2_echo camera_front_center_body camera_front_center_optical
ros2 run tf2_ros tf2_echo odom base_link
ros2 topic echo /autonomy/esdf_obstacles --once
```

> RGB 相机有画面但 DepthCloud 没画面时，优先检查深度图、`/sensor/camera/depth/camera_info` 和 `camera_front_center_body → camera_front_center_optical` TF 是否同时存在。

---

## 8.6 多 Provider Agent 流式会话网关

项目已加入第一阶段 `aeromind_agent_gateway`：

- Claude 使用 `ClaudeSDKClient` 维护原生多轮会话。
- DeepSeek、Qwen 和自定义服务使用 OpenAI 兼容流式 Runtime。
- 使用 WebSocket 推送文字增量、MCP 工具调用和最终结果。
- 使用 SQLite 保存会话、消息、Claude session ID 和可重放事件。
- Web 对话框优先连接 Agent Gateway，连接失败时回退到原有 ROS Agent。
- 支持在同一会话中切换 Claude、DeepSeek、Qwen 或自定义 Provider。
- 开放无人机状态、里程计、检测结果和自主避障状态等只读 MCP 工具。
- 控制类 MCP 只创建持久化确认请求，用户确认前不会产生 ROS 控制副作用。
- 用户确认后，Gateway 会复用现有 `/agent/execute_task` 安全检查、内部确认令牌和 MissionManager。
- Gateway 区分命令受理和物理完成，通过 ROS Mission、飞控状态、里程计及自主规划状态做最终验证。
- 已接入解锁、加锁、起飞、降落、相对移动、RTL 返航和悬停请求。
- 确认记录绑定用户和会话，默认 60 秒过期，重复确认和越权确认会被拒绝。
- 可选飞书长连接入口与 Web 共享同一套会话、事件、MCP 和确认中心。
- 飞书消息按 `message_id` 去重，只接受 `open_id` 白名单用户；群聊必须先 @机器人。
- 按绑定操作员保存有界滚动记忆，使 Web 与飞书可以延续近期对话和 Mission 结果。
- 支持白名单组合工作流：一次确认后顺序执行状态检查、感知检查和飞行动作。
- 工作流支持依赖、条件、失败策略、最多两次重试，以及暂停、恢复和取消。
- 正方形轨迹已作为首个 Gateway 模块化技能接入 Claude、DeepSeek/Qwen 和 Web 技能库。

安装依赖：

```bash
cd ~/aeromind_ws
python3 -m pip install --user -r requirements.txt
```

Claude SDK 可以使用 `ANTHROPIC_API_KEY`，也可以使用本机 Claude Code 已有的登录状态：

```bash
export ANTHROPIC_API_KEY="你的密钥"
export CLAUDE_AGENT_MODEL=sonnet
```

DeepSeek 和 Qwen 使用 OpenAI 兼容接口：

```bash
export DEEPSEEK_API_KEY="你的 DeepSeek Key"
export DASHSCOPE_API_KEY="你的阿里云百炼 Key"
```

默认注册 `deepseek:deepseek-chat` 和 `qwen:qwen-plus`。也可以通过 JSON 注册其他兼容服务，`api_key_env` 指向保存密钥的环境变量：

```bash
export AEROMIND_OPENAI_PROVIDERS='{
  "deepseek": {
    "base_url": "https://api.deepseek.com/v1",
    "api_key_env": "DEEPSEEK_API_KEY",
    "models": ["deepseek-chat"]
  },
  "qwen": {
    "base_url": "https://dashscope.aliyuncs.com/compatible-mode/v1",
    "api_key_env": "DASHSCOPE_API_KEY",
    "models": ["qwen-plus"]
  }
}'
```

编译：

```bash
cd ~/aeromind_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  aeromind_agent_gateway aeromind_web aeromind_bringup
source install/setup.bash
```

启动 Web 与 Agent Gateway：

```bash
ros2 launch aeromind_bringup aeromind_web.launch.py \
  port:=8080 \
  agent_port:=8090 \
  agent_provider:=claude \
  agent_model:=sonnet
```

以 DeepSeek 作为默认对话模型：

```bash
ros2 launch aeromind_bringup aeromind_web.launch.py \
  port:=8080 agent_port:=8090 \
  agent_provider:=deepseek agent_model:=deepseek-chat
```

默认地址：

| 服务 | 地址 | 用途 |
|---|---|---|
| Web 地面站 | `http://localhost:8080` | 遥测、相机、点云和对话界面 |
| Agent 健康检查 | `http://localhost:8090/health` | SDK、模型和 ROS 状态检查 |
| Agent WebSocket | `ws://localhost:8090/ws/agent` | 双向流式会话 |

本地开发可以不配置 `AEROMIND_AGENT_TOKEN`。局域网或真机部署必须配置：

```bash
export AEROMIND_AGENT_TOKEN="生成一个足够长的随机令牌"
```

浏览器端需要配置相同令牌后刷新：

```javascript
localStorage.setItem("aeromind_agent_token", "相同令牌")
```

SQLite 默认保存在：

```text
~/.aeromind/agent_gateway.db
```

### 组合任务与模块化技能

可直接在对话中输入：

```text
飞一个边长 10 米的正方形轨迹，高度 6 米，完成后降落
检查飞控和前方障碍物，安全后起飞到 5 米
```

模型只能创建经过校验的 `workflow`，不能生成 Shell 或任意 ROS 调用。当前步骤动作白名单为：

```text
safety_check, perception_check, arm, disarm, takeoff,
land, move, return_home, hover
```

每个步骤可以声明 `depends_on`、`condition`、`retries` 和 `on_failure`。整套任务只产生一次高风险确认；确认后每个飞行动作仍复用 ROS Agent，并等待 Mission、飞控遥测或自主规划状态完成物理验证。Web 会显示当前步骤，并提供暂停、恢复和取消按钮。

安全边界：暂停或取消会发布 `/autonomy/cancel`。正在执行的 PX4 原生起飞、降落或 RTL 不会被粗暴终止，而是由飞控继续进入安全状态；Gateway 进程重启后，未完成工作流会标记为 `interrupted`，不会自动恢复飞行。

Gateway 技能目录接口：

```text
GET http://localhost:8090/api/skills
```

团队成员可以在独立 Python 模块中注册工作流技能：

```python
from aeromind_agent_gateway.skill_registry import register_skill

register_skill(
    "inspection.demo",
    label="示例巡检",
    version="1.0.0",
    risk_level="high",
    description="执行受控巡检工作流。",
    example_task="执行示例巡检",
    builder=build_inspection_workflow,
)
```

模块必须返回通过 `validate_workflow()` 校验的工作流，再通过受信任环境变量加载：

```bash
export AEROMIND_GATEWAY_SKILL_MODULES="team_inspection.skill"
```

这样模型的 `list_skills`、Gateway `/api/skills` 与 Web 技能库会使用同一份目录。新增底层能力时仍需先定义稳定 ROS 接口并独立验证，技能层只负责任务编排。

### 飞书手机端接入（可选）

1. 在飞书开放平台创建企业自建应用并启用机器人。
2. 为机器人开通接收消息、以机器人身份发送消息和更新消息所需权限。
3. 在事件订阅中选择“使用长连接接收事件”，订阅 `im.message.receive_v1`。
4. 在卡片回调中使用长连接接收 `card.action.trigger`。
5. 从开放平台事件调试数据中取得操作员的 `sender_id.open_id`，加入白名单。

启动前配置环境变量，凭证不要提交到 Git：

```bash
export FEISHU_ENABLED=true
export FEISHU_APP_ID="cli_xxxxxxxxx"
export FEISHU_APP_SECRET="你的应用密钥"
export FEISHU_ALLOWED_OPEN_IDS="ou_xxxxxxxxx,ou_yyyyyyyyy"

# 将本机 Web 与指定飞书用户绑定为同一操作员
export AEROMIND_IDENTITY_BINDINGS='{"web-local":"operator:waves","feishu:ou_xxxxxxxxx":"operator:waves"}'

# 跨端滚动记忆（默认开启，保存最近 12 条消息摘录）
export AEROMIND_MEMORY_ENABLED=true
export AEROMIND_MEMORY_MESSAGE_LIMIT=12

# 独立 Claude 长期语义摘要（默认关闭，每累计 8 条消息触发一次）
export AEROMIND_SEMANTIC_MEMORY_ENABLED=true
export AEROMIND_SEMANTIC_MEMORY_INTERVAL=8
export AEROMIND_SEMANTIC_MEMORY_MODEL=haiku
export AEROMIND_SEMANTIC_MEMORY_BUDGET_USD=0.05

# 飞书上传图片的 VLM 分析（Qwen3-VL-Plus）
export DASHSCOPE_API_KEY="你的百炼 API Key"
export AEROMIND_VLM_API_URL="https://dashscope.aliyuncs.com/compatible-mode/v1"
export AEROMIND_VLM_API_KEY="$DASHSCOPE_API_KEY"
export AEROMIND_VLM_MODEL="qwen3-vl-plus"
export FEISHU_IMAGE_MAX_BYTES=10485760

cd ~/aeromind_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch aeromind_bringup aeromind_web.launch.py \
  port:=8080 agent_port:=8090 agent_model:=sonnet
```

WSL 只需能够主动访问飞书公网，不需要公网 IP 或配置 Webhook。手机端支持直接发送自然语言任务，以及以下命令：

```text
/status
/model sonnet
/model opus
/model haiku
/model deepseek
/model qwen
/model provider:model
/confirm <confirmation_id>
/cancel <confirmation_id>
/help
```

飞行控制仍需要确认。机器人私聊会显示交互式确认卡片，并随任务状态更新为执行中、完成或失败；`/confirm` 和 `/cancel` 文本命令继续作为断线重启后的降级入口。普通模型回复会节流更新同一条消息，避免逐 Token 刷屏。群聊不提供确认按钮，只显示确认编号，控制确认必须回到机器人私聊完成。

白名单用户还可以在机器人私聊中直接发送 JPEG、PNG 或 WEBP 图片。Gateway 会通过飞书消息资源接口下载图片，默认限制为 10 MB 和 2500 万像素，随后调用 `AEROMIND_VLM_*` 配置的视觉模型。图片、`mission.json` 和分析报告会保存到：

```text
missions/mission-feishu-<timestamp>-<message_id>/
├── mission.json
├── report.md
└── captures/
    └── uploaded.png
```

这些任务会出现在 Web 的任务报告列表中，并显示上传图片缩略图。飞书应用至少需要接收单聊消息、以机器人身份发送/更新消息，以及读取同一会话消息资源的权限。

### Web 与飞书跨端任务

`AEROMIND_IDENTITY_BINDINGS` 用于把不同渠道身份映射为同一个内部操作员。完成绑定后：

- 飞书创建的控制请求会立即显示在 Web 任务态势中。
- Web 可以确认或取消飞书发起的任务，飞书卡片同步更新。
- Mission 状态统一保存为 `pending_confirmation`、`executing`、`completed`、`failed`、`cancelled` 或 `expired`。
- 执行阶段进一步记录为 `dispatching`、`accepted`、`verifying`、`completed` 或 `failed`。
- Web 断线重连后会恢复该操作员的待确认任务和最近 Mission。
- Web 与飞书的新会话会读取该操作员的近期滚动记忆和真实 Mission 状态。

滚动记忆保存在 Gateway SQLite 的 `operator_memories` 表中。历史内容会被标记为只读上下文，不能直接触发控制，也不能绕过安全检查和人工确认。滚动摘录本身不会额外调用模型。

启用 `AEROMIND_SEMANTIC_MEMORY_ENABLED` 后，Gateway 会启动与主对话隔离、无 ROS 工具权限的 Claude 摘要会话，提取稳定偏好、项目配置、技术决定、事实和任务结论。条目保存在 `semantic_memories`，经过敏感字段过滤、内容指纹去重和相关性检索后注入后续对话。Web 与飞书聊天会通过操作员统一时间线恢复。该功能会产生额外模型调用，演示前应根据额度调整触发间隔和单次预算。

记忆查询和删除接口：

```text
GET    /api/operator/timeline
GET    /api/operator/memories?query=巡检
DELETE /api/operator/memories/<memory_id>
```

控制服务返回成功只代表命令被 ROS Agent 接受。Gateway 会继续等待物理完成条件：解锁/加锁读取 `DroneState`，起飞和降落读取里程计高度，移动读取 `/autonomy/status` 的到达或阻塞状态，RTL 等待飞控完成并加锁，悬停读取速度与自主规划策略。验证完成后 `physical_complete` 才会写为 `true`；超时或阻塞会将 Mission 标记为失败，并保留验证证据。

身份绑定意味着授权本机 Web 代表对应飞书用户确认控制动作。启用跨端确认时必须同时设置强随机令牌，并且不要把 8090 端口直接暴露到公网：

```bash
export AEROMIND_AGENT_TOKEN="至少32字节的随机令牌"
```

浏览器仍需通过 `localStorage` 配置相同令牌。没有显式身份绑定时，Web 和飞书保持独立，不能跨渠道确认。

完整升级设计见 [多端智能无人机 Agent 升级技术方案](doc/Claude智能体与飞书升级方案.md)。

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

### 8.5 Git 忽略

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
│   ├── aeromind_agent/          # LLM Agent 节点（ament_python，组员 E）
│   ├── aeromind_agent_gateway/  # Claude SDK、多轮会话、MCP 和流式事件
│   ├── aeromind_teleop/         # 键盘遥控节点（ament_python）
│   ├── aeromind_bringup/        # 启动文件（ament_cmake）
│   │   └── launch/
│   │       ├── aeromind_all.launch.py     # AirSim 直连模式
│   │       ├── aeromind_px4.launch.py     # PX4 模式
│   │       ├── aeromind_vision.launch.py  # 相机 + RViz 可视化
│   │       ├── aeromind_web.launch.py     # Web 控制台
│   │       ├── depth_cloud.rviz           # 深度点云 RViz 配置
│   │       └── image_lidar.rviz           # 图像+激光雷达 RViz 配置
│   └── px4_msgs/                # PX4 消息定义（从 GitHub 克隆）
├── doc/
│   ├── PX4与AirSim配置说明.md        # AirSim + PX4 详细配置文档
│   ├── WSL2代理配置.md               # WSL2 代理/VPN 配置
│   ├── Claude智能体与飞书升级方案.md  # Agent SDK 与飞书升级方案
│   ├── 客户汇报项目说明.md           # 面向客户的项目汇报底稿
│   ├── 当前系统状态与升级交接说明.md  # 当前能力和后续升级基线
│   ├── 智能无人机系统升级计划.md      # 智能化升级阶段计划
│   ├── 系统使用手册.md                # 启动、操作、演示和故障排查
│   ├── 项目原理架构与操作手册.md      # 原理、架构和命令手册
│   └── AirSim配置示例.json            # AirSim 完整配置备份
├── README.md
└── .gitignore
```

## 12. 相关文档

- [系统使用手册](doc/系统使用手册.md) — 完整启动、界面操作、对话确认、演示流程和故障排查
- [项目原理、架构与操作手册](doc/项目原理架构与操作手册.md)
- [当前系统现状与升级交接说明](doc/当前系统状态与升级交接说明.md)
- [PX4 + AirSim 配置详细说明](doc/PX4与AirSim配置说明.md) — 网络配置、AirSim settings.json、PX4 参数详解
- [Claude Agent SDK 多端升级方案](doc/Claude智能体与飞书升级方案.md)
- [智能无人机系统升级计划](doc/智能无人机系统升级计划.md)
- [客户汇报项目说明](doc/客户汇报项目说明.md)
- [WSL2 代理配置说明](doc/WSL2代理配置.md)
- [PX4 User Guide](https://docs.px4.io/main/en/)
- [ROS 2 Humble Docs](https://docs.ros.org/en/humble/)
- [AirSim Docs](https://microsoft.github.io/AirSim/)

## 许可证

Apache-2.0

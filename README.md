# 智能无人机自然语言控制系统

基于 ROS 2 Humble、PX4 和 AirSim 的智能无人机研究平台。系统通过 Web 地面站接收自然语言任务，由 LLM/VLM 完成高层理解与任务编排，再由确定性的 ROS 2 节点和 PX4 执行感知、规划、轨迹跟踪与飞行控制。

> 内部 ROS 包沿用 `aeromind_*` 前缀。当前项目适用于科研、仿真验证和系统演示，不应直接视为成熟真机飞行产品。

![系统总架构](doc/架构图/01-系统总架构.png)

## 当前能力

| 领域 | 已实现 | 重要边界 |
|---|---|---|
| 智能交互 | Web 流式多轮对话、历史记录、模型切换、飞书入口 | 模型只负责高层决策，不直接控制电机 |
| 任务执行 | Skills、Workflow、条件分支、人工确认、暂停/恢复/取消 | 只能组合已注册且通过校验的基础能力 |
| 感知 | RGB、深度、LiDAR、YOLO、截图、VLM 图像分析 | 视觉结果不能作为唯一飞行安全依据 |
| 自主飞行 | 局部体素地图、动力学重规划、Minimum Snap、多航点 Action | 尚未接入成熟 VIO/SLAM 和工程级 ESDF 后端 |
| PX4 控制 | 解锁、起飞、降落、RTL、Offboard 轨迹跟踪 | 真机部署前仍需安全监督、标定和场景验收 |
| 可视化 | 飞行状态、RGB、深度、Three.js 点云、任务与日志 | Web 通过 ROS 网关访问数据，不直接加入 ROS 网络 |

完整状态与限制见 [项目概览与当前状态](doc/01-项目概览与当前状态.md)。

## 快速启动

### 1. 加载并构建工作空间

```bash
cd ~/aeromind_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 2. 启动 PX4 与 ROS

完整 AirSim + PX4 模式需要依次启动：

```bash
# 终端 1：PX4 与 ROS 2 的 DDS 桥
micro-xrce-dds-agent udp4 -p 8888

# 终端 2：PX4 SITL
cd ~/PX4-Autopilot
make px4_sitl_default none_iris

# Windows：启动 AirSim

# 终端 3：ROS 主系统
cd ~/aeromind_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch aeromind_bringup aeromind_px4.launch.py
```

不使用 YOLO、VLM 或云端 LLM 时，上述默认命令即可运行基础系统。完整环境、模型参数和其他仿真模式见 [安装配置与启动](doc/03-安装配置与启动.md)。

### 3. 启动 Web 地面站

```bash
cd ~/aeromind_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch aeromind_bringup aeromind_web.launch.py \
  host:=0.0.0.0 port:=8080 agent_port:=8090 \
  agent_provider:=deepseek agent_model:=deepseek-chat
```

访问 `http://localhost:8080`。也可单独托管前端：

```bash
cd ~/aeromind_ws/web/ground_station
python3 -m http.server 5173
```

访问 `http://localhost:5173`，页面中的 ROS 网关保持为 `http://localhost:8080`。

### 4. 最小控制验证

```bash
source ~/aeromind_ws/install/setup.bash

ros2 topic echo /control/drone_state
ros2 service call /control/takeoff aeromind_interfaces/srv/Takeoff "{altitude: 10.0}"
ros2 service call /control/land aeromind_interfaces/srv/Land "{}"
ros2 service call /control/return_home aeromind_interfaces/srv/ReturnHome "{}"
```

飞行前应确认仿真场景、飞控状态、里程计和控制链路正常。详细操作见 [使用与故障排查](doc/04-使用与故障排查.md)。

## 系统组成

| 包或目录 | 职责 |
|---|---|
| `aeromind_interfaces` | 自定义 Message、Service 和 Action 协议 |
| `aeromind_bridge` | AirSim/PX4 数据桥接、坐标转换和 TF |
| `aeromind_perception` | 图像保存、YOLO、VLM 和感知摘要 |
| `aeromind_autonomy` | 局部地图、避障重规划、Minimum Snap 轨迹 |
| `aeromind_control` | PX4 原生指令、Offboard 和轨迹执行 |
| `aeromind_agent` | ROS 任务解析、基础技能和状态验证 |
| `aeromind_agent_gateway` | 流式会话、模型 Runtime、确认、Skills 与 Workflow |
| `aeromind_web` | ROS HTTP/WebSocket 网关 |
| `web/ground_station` | 与 ROS 解耦的独立浏览器地面站 |
| `aeromind_bringup` | 系统 Launch 与 RViz 配置 |

详细模块关系见 [系统架构与数据流](doc/02-系统架构与数据流.md)。

## 文档入口

所有文档从 [文档导航](doc/README.md) 进入：

| 你想做什么 | 阅读文档 |
|---|---|
| 快速理解项目和当前完成度 | [项目概览与当前状态](doc/01-项目概览与当前状态.md) |
| 学习 ROS、PX4、AirSim、TF 和数据链路 | [系统架构与数据流](doc/02-系统架构与数据流.md) |
| 安装、配置模型并完整启动 | [安装配置与启动](doc/03-安装配置与启动.md) |
| 操作 Web/ROS、执行任务、排查问题 | [使用与故障排查](doc/04-使用与故障排查.md) |
| 理解 Agent、Skills 和 Workflow | [Agent、Skills 与 Workflow](doc/05-Agent、Skills与Workflow.md) |
| 开发模块、修改接口和运行测试 | [开发与测试指南](doc/06-开发与测试指南.md) |
| 了解不足和后续研发顺序 | [限制与升级路线](doc/07-限制与升级路线.md) |
| 准备项目汇报或演示 | [汇报与演示指南](doc/08-汇报与演示指南.md) |

## 测试

```bash
cd ~/aeromind_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  src/aeromind_agent_gateway/test \
  src/aeromind_autonomy/test \
  src/aeromind_control/test -q
```

测试通过只表示软件逻辑基线通过，不等于复杂环境避障或真机安全已经验收。

## 安全原则

- 大模型输出必须转换为结构化任务，并通过白名单和参数校验。
- 高风险动作必须经过确认中心，不允许模型伪造确认结果。
- 飞行完成由 PX4 状态、里程计和 Mission 状态验证，不以服务返回作为唯一依据。
- 定位、深度或控制数据超时必须悬停、降落或 RTL，禁止盲飞。
- 真机测试必须从无桨台架、系留和低速小范围测试逐级推进。

## 许可证

本项目当前用于科研与教学验证。正式发布前请补充明确的软件许可证、第三方依赖许可证和飞行安全声明。

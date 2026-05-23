# AeroMind Console — 超详细架构与功能实现说明

## 目录

1. [项目概述](#1-项目概述)
2. [架构总览](#2-架构总览)
3. [后端核心模块](#3-后端核心模块)
   - [3.1 server.py — HTTP/WebSocket 服务层](#31-serverpy--httpwebsocket-服务层)
   - [3.2 core/task_schema.py — 任务数据结构](#32-coretask_schemapy--任务数据结构)
   - [3.3 core/path_planner.py — 路径规划](#33-corepath_plannerpy--路径规划)
   - [3.4 core/occupancy_grid.py — 占用栅格地图](#34-coreoccupancy_gridpy--占用栅格地图)
   - [3.5 core/obstacle_avoidance.py — A*避障路径规划](#35-coreobstacle_avoidancepy--a避障路径规划)
   - [3.6 core/airsim_adapter.py — AirSim 仿真适配层](#36-coreairsim_adapterpy--airsim-仿真适配层)
   - [3.7 core/task_planner.py — LLM+规则混合任务规划](#37-coretask_plannerpy--llm规则混合任务规划)
   - [3.8 core/llm_client.py — 大模型客户端](#38-corellm_clientpy--大模型客户端)
   - [3.9 core/task_executor.py — 任务执行器](#39-coretask_executorpy--任务执行器)
   - [3.10 core/safety_gate.py — 安全门校验](#310-coresafety_gatepy--安全门校验)
   - [3.11 core/validation.py — Pydantic 参数校验](#311-corevalidationpy--pydantic-参数校验)
   - [3.12 core/agent_tools.py — Agent 工具运行时](#312-coreagent_toolspy--agent-工具运行时)
   - [3.13 core/vision_detector.py — YOLO 视觉检测](#313-corevision_detectorpy--yolo-视觉检测)
   - [3.14 core/video_stream.py — MJPEG 视频流](#314-corevideo_streampy--mjpeg-视频流)
   - [3.15 core/formation_manager.py — 编队管理](#315-coreformation_managerpy--编队管理)
   - [3.16 core/preflight.py — 飞行前检查](#316-corepreflightpy--飞行前检查)
   - [3.17 core/replay.py — 轨迹回放](#317-corereplaypy--轨迹回放)
   - [3.18 core/websocket.py — WebSocket 协议实现](#318-corewebsocketpy--websocket-协议实现)
   - [3.19 core/auth.py — 认证与鉴权](#319-coreauthpy--认证与鉴权)
   - [3.20 core/services.py — 事件/状态/路由管理](#320-coreservicespy--事件状态路由管理)
   - [3.21 core/metrics.py — 指标采集与性能追踪](#321-coremetricspy--指标采集与性能追踪)
   - [3.22 core/storage.py — SQLite 持久化](#322-corestoragepy--sqlite-持久化)
   - [3.23 core/exceptions.py — 异常体系](#323-coreexceptionspy--异常体系)
   - [3.24 core/di.py — 依赖注入容器](#324-coredipy--依赖注入容器)
   - [3.25 core/async_tools.py — 异步工具运行时](#325-coreasync_toolspy--异步工具运行时)
4. [前端架构](#4-前端架构)
   - [4.1 index.html — UI 布局](#41-indexhtml--ui-布局)
   - [4.2 app.js — 入口与事件绑定](#42-appjs--入口与事件绑定)
   - [4.3 common.js — 全局状态与工具函数](#43-commonjs--全局状态与工具函数)
   - [4.4 map.js — Canvas 2D 地图引擎](#44-mapjs--canvas-2d-地图引擎)
   - [4.5 ui.js — DOM 渲染与 API 调用](#45-uijs--dom-渲染与-api-调用)
5. [核心数据流](#5-核心数据流)
   - [5.1 任务提交流](#51-任务提交流)
   - [5.2 实时遥测推流](#52-实时遥测推流)
   - [5.3 安全门校验流](#53-安全门校验流)
6. [安全机制](#6-安全机制)
7. [附录：模块依赖图](#7-附录模块依赖图)

---

## 1. 项目概述

AeroMind Console 是一个**自然语言无人机任务控制台**，基于 AirSim 仿真平台。用户输入中文或英文的自然语言指令（如"搜索前方60米区域，发现车辆后悬停并报告坐标"），系统通过 LLM 大模型解析意图、规划任务步骤，经过安全门校验后自动执行飞行任务。

**技术栈**：
- 后端：Python 3.10+，纯标准库 HTTP 服务器（`http.server`），零外部 Web 框架依赖
- 仿真：Microsoft AirSim（Unreal Engine 插件）
- AI：OpenAI 兼容 API（LLM 任务解析）、YOLOv5/v8（视觉检测）
- 前端：原生 JavaScript ES Modules、Canvas 2D、WebSocket

---

## 2. 架构总览

```
┌─────────────────────────────────────────────────────────────┐
│                      浏览器前端                              │
│  index.html → app.js → ui.js / map.js / common.js          │
│  WebSocket (实时推送) + HTTP POST (命令下发)                 │
└──────────────────────┬──────────────────────────────────────┘
                       │ HTTP / WS
┌──────────────────────▼──────────────────────────────────────┐
│                   server.py                                  │
│  MissionConsoleServer (ThreadingHTTPServer)                  │
│  ├─ Handler.do_GET()  → 静态文件 / API / WebSocket           │
│  ├─ Handler.do_POST() → 任务提交 / 飞控命令 / 检测触发       │
│  └─ Handler._handle_ws() → WebSocket 实时推送               │
│                                                              │
│  ConsoleState (全局服务状态)                                  │
│  ├─ AirSimAdapter       → AirSim 连接/控制/传感器            │
│  ├─ TaskPlanner         → LLM + 规则混合规划                 │
│  ├─ SafetyGate          → 安全门参数校验                     │
│  ├─ AgentToolRuntime    → 15种工具调用执行                   │
│  ├─ VisionDetector      → YOLO 目标检测                      │
│  ├─ VideoStream         → MJPEG 相机推流                     │
│  ├─ FormationManager    → 编队队形计算                       │
│  ├─ AuthManager         → API Key + IP + 速率限制            │
│  ├─ EventManager        → 事件日志                           │
│  ├─ RouteManager        → 轨迹/航线管理                      │
│  └─ MissionState        → 任务状态机                         │
└──────────────────────┬──────────────────────────────────────┘
                       │ AirSim API
┌──────────────────────▼──────────────────────────────────────┐
│               Microsoft AirSim (UE4/UE5)                     │
│  多旋翼仿真、LiDAR、距离传感器、碰撞检测、多相机              │
└─────────────────────────────────────────────────────────────┘
```

---

## 3. 后端核心模块

### 3.1 server.py — HTTP/WebSocket 服务层

**文件**：[server.py](../server.py)（~1270行）

**职责**：整个系统的入口和HTTP路由层。使用Python标准库的 `http.server.ThreadingHTTPServer` 实现多线程HTTP服务。

#### 3.1.1 MissionConsoleServer

```python
class MissionConsoleServer(ThreadingHTTPServer):
    def __init__(self, server_address, handler_class):
        super().__init__(server_address, handler_class)
        self.console = ConsoleState()  # 全局服务状态实体
```

替代了原来的全局 `STATE = ConsoleState()` 单例模式。服务状态附着在服务器实例上，每个请求处理器通过 `self.server.console` 访问。这避免了模块级全局变量，支持未来在一个进程中运行多个服务器实例。

#### 3.1.2 GET 路由

| 路径 | 处理器 | 说明 |
|------|--------|------|
| `/` | `send_static` | 重定向到 `/index.html` |
| `/index.html` | `send_static` | 主页面 |
| `/static/*` | `send_static` | 静态文件（CSS、JS模块） |
| `/video/front` | `send_mjpeg` | MJPEG 视频流（multipart/x-mixed-replace） |
| `/camera/latest` | `send_latest_camera_frame` | 最新单帧JPEG |
| `/ws` | `_handle_ws` | **WebSocket 升级**（实时遥测推送） |
| `/api/state` | `console.snapshot` | **完整状态快照**（需认证） |
| `/api/telemetry` | `console.telemetry_snapshot` | **轻量遥测**（需认证） |
| 其他路径 | `send_static` | 尝试作为静态文件返回 |

#### 3.1.3 POST 路由

| 路径 | 处理器 | 说明 |
|------|--------|------|
| `/api/airsim/reconnect` | `reconnect_airsim` | 重新连接AirSim |
| `/api/camera/probe` | `probe_cameras` | 探测可用相机 |
| `/api/camera/select` | `select_camera` | 切换相机视角 |
| `/api/vehicle/select` | `select_vehicle` | 切换无人机 |
| `/api/task/preview` | `preview_task` | **任务预览**（不执行） |
| `/api/task/run` | `run_task` | **执行任务** |
| `/api/task/pause` | `pause_task` | 暂停任务 |
| `/api/task/stop` | `stop_task` | 终止任务 |
| `/api/task/rtl` | `rtl` | 触发返航 |
| `/api/flight/takeoff` | `flight_takeoff` | 直接起飞 |
| `/api/flight/hover` | `flight_hover` | 悬停 |
| `/api/flight/land` | `flight_land` | 降落 |
| `/api/detect/latest` | `detect_latest` | 执行目标检测 |

#### 3.1.4 WebSocket 实现 (`_handle_ws`)

```
客户端请求 GET /ws (带 Upgrade: websocket 头)
  → ws_handshake() 验证握手密钥
  → 返回 101 Switching Protocols
  → 进入推送循环:
      每 200ms: 发送 {"type": "telemetry", "data": ...}
      每 1000ms (tick%5==0): 发送 {"type": "state", "data": ...}
  → 客户端断开: BrokenPipeError → 线程退出
```

每个WebSocket客户端占用一个线程（ThreadingHTTPServer 为每个连接创建新线程），服务器端主动推送，前端无需轮询。

#### 3.1.5 安全措施

- **认证默认启用**：`AuthConfig(enabled=True)`
- **请求体限制**：`MAX_CONTENT_LENGTH = 1MB`
- **路径遍历防护**：`send_static()` 使用 `Path.resolve()` + `relative_to()` 校验
- **MIME类型推断**：`mimetypes.guess_type()` 自动设置 Content-Type

---

### 3.2 core/task_schema.py — 任务数据结构

**文件**：[core/task_schema.py](../core/task_schema.py)（60行）

定义三个不可变数据类（`frozen=True` 的 dataclass），作为任务规划的"领域语言"：

```python
@dataclass(frozen=True)
class MissionArea:       # 矩形任务区域（NED坐标）
    x_min, x_max, y_min, y_max: float

@dataclass(frozen=True)
class PlanStep:          # 单个执行步骤
    name, detail, action: str

@dataclass(frozen=True)
class MissionPlan:       # 完整任务计划
    task, intent, target, altitude_m, strategy: str
    area: MissionArea | None
    plan: list[PlanStep]
    raw: dict  # 保留原始LLM/解析结果，便于调试
```

**设计要点**：
- 所有字段不可变（`frozen=True`），防止运行时意外修改
- 每个类都有 `to_api()` 方法，将内部类型转换为 JSON 可序列化的 dict
- `MissionPlan.raw` 字段保留原始解析上下文，用于前端展示和调试

---

### 3.3 core/path_planner.py — 路径规划

**文件**：[core/path_planner.py](../core/path_planner.py)（35行）

```python
@dataclass(frozen=True)
class Waypoint:
    x: float  # NED X（北/前）
    y: float  # NED Y（东/右）
    z: float  # NED Z（下，负值为上方）

def lawnmower_path(area: MissionArea, altitude_m: float, spacing_m: float = 10.0) -> list[Waypoint]:
```

**算法**：简单的矩形覆盖路径生成（"割草机"模式）：
1. 从 `area.y_min` 开始，从左到右飞行
2. 到达右边界后，向上移动 `spacing_m` 米
3. 从右到左飞行
4. 重复直到覆盖整个矩形区域

**示例**：`MissionArea(0, 60, -20, 20)` + `altitude=8m` + `spacing=10m` 产生：
```
(0, -20, -8) → (60, -20, -8) → (60, -10, -8) → (0, -10, -8) → (0, 0, -8) → ...
```

---

### 3.4 core/occupancy_grid.py — 占用栅格地图

**文件**：[core/occupancy_grid.py](../core/occupancy_grid.py)（108行）

**职责**：将 LiDAR 点云转换为 2D 栅格地图，供 A* 路径规划使用。

#### 核心数据结构

```python
class OccupancyGrid:
    center_x, center_y: float  # 栅格中心（世界坐标）
    size_m: float              # 栅格边长（默认60m）
    resolution_m: float        # 单元格分辨率（默认1.5m）
    inflation_m: float         # 障碍物膨胀半径（默认3.0m）
    blocked: set[(int,int)]    # 膨胀后的障碍单元格
    raw_blocked: set[(int,int)] # 原始LiDAR点占据的单元格
```

#### 关键方法

**`add_world_points(points)`** — 将 LiDAR 世界坐标点集添加到栅格：
1. 每个点调用 `world_to_cell(x, y)` 转换为栅格坐标
2. 加入 `raw_blocked` 集合
3. 调用 `_inflate()` 进行障碍物膨胀

**`_inflate()`** — 障碍物膨胀算法：
```
对每个原始障碍单元格:
  在 inflation_m 半径范围内的所有单元格标记为 blocked
  使用 math.hypot(dx, dy) * resolution_m <= inflation_m 确保圆形膨胀
```

目的：考虑无人机半径和安全距离，在障碍物周围创建缓冲区。

**`nearest_free(cell, max_radius=8)`** — 在受阻单元格附近搜索最近的空闲单元格：
```
如果 cell 本身空闲 → 返回 cell
否则从 radius=1 到 max_radius 进行环形搜索
  遍历正方形边界上的所有单元格
  找到第一个空闲单元格 → 返回
```

**`block_outside_world_bounds(x_min, x_max, y_min, y_max)`** — 将任务区域外的所有单元格标记为障碍，防止路径规划越界。

---

### 3.5 core/obstacle_avoidance.py — A*避障路径规划

**文件**：[core/obstacle_avoidance.py](../core/obstacle_avoidance.py)（136行）

**职责**：在 LiDAR 感知的障碍物地图上运行 A* 搜索，生成安全路径。

#### 入口函数

```python
def plan_local_path(
    start: Waypoint, goal: Waypoint,
    obstacle_points: list[(float, float)],
    grid_size_m=70, resolution_m=1.5, inflation_m=3.5,
    max_waypoints=18,
    bounds: (x_min, x_max, y_min, y_max) | None
) -> AvoidancePlan:
```

**执行流程**：
1. 根据 start/goal 中心创建 `OccupancyGrid`
2. 将 LiDAR 障碍点加入栅格
3. 如果指定了 bounds，将区域外标记为障碍
4. 将 start/goal 转换为栅格坐标，必要时使用 `nearest_free()` 修正
5. 运行 `_astar()` 搜索
6. `_sample_cells()` 降采样为最多 18 个航点
7. 返回 `AvoidancePlan(ok=True, waypoints=[...], grid={...})`

#### A* 搜索算法 (`_astar`)

标准 A* 实现：
- **启发函数**：欧几里得距离 `math.hypot(a[0]-b[0], a[1]-b[1])`
- **邻域**：8方向（含对角线），对角线代价 `√2`，正交代价 `1`
- **数据结构**：`heapq` 优先队列 + `dict` 存储 g_score 和 came_from
- **终止**：找到 goal 或 open_heap 清空

#### 路径降采样 (`_sample_cells`)

当 A* 返回的栅格路径太长时，均匀采样最多 `max_waypoints`（默认18）个点：
```python
step = max(1, len(cells) // max_waypoints)
sampled = cells[step::step]
```

#### 返回值

```python
AvoidancePlan:
    ok: bool           # 是否找到路径
    waypoints: [Waypoint]  # 降采样后的3D航点
    reason: str         # 结果说明
    grid: dict          # 栅格统计信息（供前端展示）
```

---

### 3.6 core/airsim_adapter.py — AirSim 仿真适配层

**文件**：[core/airsim_adapter.py](../core/airsim_adapter.py)（1624行）

**职责**：封装所有与 AirSim 的交互。这是项目中最大的模块，因为需要处理 AirSim API 的各种细节。

#### 3.6.1 数据模型

```python
UavTelemetry           # 无人机遥测：位置(x,y,z)、速度、高度、名称、模式
CameraFrame            # 相机帧：原始字节数据 + 内容类型 + 相机名称
CollisionStatus        # 碰撞状态：是否碰撞、碰撞对象、碰撞点、碰撞法线
ObstacleStatus         # 障碍物状态：阻塞标志、最近距离、扇区点数、风险等级
DistanceSensorsStatus  # 距离传感器：前/左/右距离、紧急方向
```

#### 3.6.2 连接管理

**自动重连机制** (`_reconnect_loop`)：
- 后台 daemon 线程每 3 秒检查连接状态
- 连接断开时自动调用 `_try_connect()` 重连
- 连接正常时执行心跳检测（`getMultirotorState`），连续 3 次失败判定断连

**连接流程** (`_try_connect`)：
1. `import airsim`（动态导入，模块缺失时抛出可捕获异常）
2. `airsim.MultirotorClient(ip, port, timeout_value=1.5)` 创建客户端
3. `client.confirmConnection()` 确认连接
4. `client.listVehicles()` 发现可用无人机
5. 对每架无人机调用 `enableApiControl(True)` + `armDisarm(True)`
6. 预热相机（获取 3 帧）

#### 3.6.3 命令线程模型

为避免阻塞 AirSim API 调用影响 WebSocket 推送和 HTTP 响应，所有飞控命令通过**单线程命令队列**执行：

```python
_command_queue: queue.Queue[CommandJob | None]

def _command_loop(self):
    while True:
        job = self._command_queue.get()
        if job is None: return
        job.result = job.fn()        # 在命令线程中执行 AirSim API 调用
        job.done.set()               # 通知调用线程结果已就绪

def _run_command(self, fn, timeout_s=None):
    job = CommandJob(fn)
    self._command_queue.put(job)
    job.done.wait(timeout=timeout_s)  # 调用线程阻塞等待
    if job.error: raise job.error
    return job.result
```

**设计原理**：AirSim 客户端的某些操作不是线程安全的。通过将所有命令序列化到单个线程中执行，避免了竞态条件。

#### 3.6.4 位置保持机制

每次飞行操作（起飞、goto、悬停......）结束后，自动启动**位置保持线程**：

```python
def _position_hold_loop(self):
    while not stop_event.wait(0.35):
        # 读取当前位置
        state = client.getMultirotorState()
        # 计算位置偏差
        vx = clamp((hold_x - px) * 0.35, -0.45, 0.45)
        vy = clamp((hold_y - py) * 0.35, -0.45, 0.45)
        # 速度修正回到目标点
        client.moveByVelocityZAsync(vx, vy, target_z, 0.49)
```

使用比例控制器（P=0.35），速度上限 0.45 m/s，每 350ms 刷新一次。防止无人机在悬停时漂移。

#### 3.6.5 核心飞行操作

**`takeoff(altitude_m)`**：
1. 调用 `takeoffAsync()` 地面起飞
2. `moveToZAsync(target_z, velocity=2.0)` 爬升
3. 等待到达目标高度（误差 ≤0.6m）
4. 启动位置保持

**`goto_local(x, y, z, speed_mps)`** — 使用 PD 控制器的闭环导航：
- P 增益：`kp_xy=0.85, kp_z=0.75`
- D 增益（阻尼）：`damp=0.35`
- 加速度限制：`1.35 m/s²` 每控制周期
- 停止条件：水平误差 ≤0.6m，垂直误差 ≤0.35m，速度 ≤0.8 m/s

**`move_on_path(path, speed_mps)`** — 使用 AirSim 内置 `moveOnPathAsync` 沿平滑路径飞行。

**`return_to_launch(altitude_m)`** — 先爬升至安全高度，然后 `moveToPositionAsync(0, 0, safe_z)`。

**`recover_from_collision(climb_m, backoff_m)`** — 碰撞恢复：
1. 取消当前任务
2. 向上爬升 `climb_m` 米
3. 向 X 负方向后退 `backoff_m` 米
4. 如果位移不足或被卡住 → 使用 `simSetVehiclePose` 强制传送

#### 3.6.6 障碍物感知

**`obstacle_status(lookahead_m, corridor_width_m)`** — LiDAR 扇区分析：
- 从 LiDAR 点云提取前方点
- 按角度分为 5 个扇区：`hard_left(-90°~-45°) → left(-45°~-15°) → front(-15°~15°) → right(15°~45°) → hard_right(45°~90°)`
- 统计每个扇区的点数和最近距离
- 判断前方是否阻塞（`front_points ≥ 6`）
- 计算风险等级：`critical(阻塞且≤4m) → high(阻塞且≤7m) → medium(前方有点) → low(其他) → clear(无点)`
- 推荐避障方向：`left_score vs right_score`（综合考虑该方向的最近距离和点数）

**`lidar_obstacle_points_world()`** — 获取世界坐标系下的障碍点：
1. 读取 LiDAR 原始点云
2. 滤除过近（<1m）、过远（>35m）和垂直方向偏移过大的点
3. 使用四元数旋转将传感器坐标系转为世界坐标系
4. `_denoise_world_points()` 基于网格的降噪（每 2m×2m 格至少3点才保留）

**`distance_sensors_status()`** — 前/左/右距离传感器：
- 判定紧急状态：前方 ≤2.5m 或侧方 ≤1.5m

#### 3.6.7 对象生成与轨迹绘制

- **`spawn_object`**：在指定位置生成 3D 模型（用于回放中的目标标记）
- **`destroy_object`**：销毁指定对象
- **`plot_line_strip`**：绘制持久化线段（用于回放轨迹可视化）
- **`set_vehicle_pose`**：强制设置无人机位置/姿态（用于回放时的初始定位）
- **`flush_persistent_markers`**：清除所有持久化标记

---

### 3.7 core/task_planner.py — LLM+规则混合任务规划

**文件**：[core/task_planner.py](../core/task_planner.py)（469行）

**职责**：将自然语言文本转换为结构化的 `MissionPlan`。采用**LLM 优先 + 规则回退**策略。

#### 3.7.1 规划入口

```python
class TaskPlanner:
    def __init__(self, llm_client=None):
        self._llm = llm_client  # 可选，None时只使用规则引擎

    def plan(self, text: str) -> MissionPlan:
        # 1. 如果配置了LLM，先尝试LLM解析
        if self._llm is not None:
            llm_result = self._llm.plan_mission(raw)
            if llm_result is not None:
                return self._from_json(raw, llm_result)  # LLM成功
        # 2. LLM失败或未配置 → 使用规则引擎
        return self._plan_rules(raw)
```

#### 3.7.2 规则引擎 (`_plan_rules`)

将用户输入按优先级匹配 8 种意图类型：

| 意图 | 匹配关键词（中/英） | 触发条件示例 |
|------|---------------------|-------------|
| `image_collection` | 采集图像、拍图、dataset | "在前方40米采集200张训练图片" |
| `autonomous_navigation` | 自主导航、闭环导航 | "自主飞到 x=50, y=30" |
| `takeoff` | 起飞、升空、launch | "起飞到15米" |
| `formation_flight` | 编队、队形、formation | "三架V字编队飞行40米" |
| `area_search` | 搜索、搜寻、find | "搜索前方60米区域" |
| `object_or_area_scan` | 扫描、巡检、环绕 | "绕目标建筑扫描一圈" |
| `obstacle_aware_navigation` | 避障、绕障 | "避障飞到目标点" |
| `path_planning` | 路径、航线、waypoint | "规划一条安全航线" |

每种意图有独立的计划生成函数，产生对应的 `PlanStep` 序列。例如 `_area_search_plan`：
```
Step 1: 解析搜索任务 → 提取目标类型、搜索区域和高度约束
Step 2: 生成覆盖航线 → 为指定区域生成往复扫描航点
Step 3: 起飞并进入区域 → 达到安全高度后飞向第一个搜索航点
Step 4: 运行视觉检测 → 读取相机画面并在飞行过程中检测目标
Step 5: 确认目标并悬停 → 发现目标后报告坐标、保存截图并保持悬停
```

#### 3.7.3 参数提取

使用正则表达式从自然语言中提取数值参数：

```python
extract_altitude(text)  # "高度10米" → 10.0
extract_area(text)      # "前方80米左右30米" → MissionArea(0, 80, -30, 30)
extract_speed(text)     # "速度3米每秒" → 3.0
extract_goal_local(text) # "x=50, y=30" → {"x": 50, "y": 30}
extract_count(text)     # "三架"、"5 drones" → 3, 5
extract_shape(text)     # "一字"→"line", 默认"v"
```

#### 3.7.4 JSON 直接输入

如果用户输入本身是合法 JSON，跳过意图推断直接解析：

```python
_try_parse_json(raw)  # 尝试 json.loads
_from_json(task, parsed)  # 从JSON构造 MissionPlan
```

---

### 3.8 core/llm_client.py — 大模型客户端

**文件**：[core/llm_client.py](../core/llm_client.py)（134行）

**职责**：调用 OpenAI 兼容 API（或其他兼容提供商）进行自然语言到结构化任务的转换。**零外部依赖**，仅使用 `urllib.request`。

#### 3.8.1 配置

```python
@dataclass
class LLMConfig:
    api_key: str = ""                         # API密钥
    base_url: str = "https://api.openai.com/v1"  # 兼容OpenAI的API端点
    model: str = "gpt-4o-mini"               # 模型名称
    timeout_s: float = 15.0                   # 请求超时
    max_tokens: int = 1024                    # 最大生成token数
    temperature: float = 0.2                  # 低温度以获得确定性输出
```

#### 3.8.2 Prompt 设计

使用 `MISSION_PROMPT_TEMPLATE` 模板，包含：
1. **角色设定**：你是 AirSim 无人机任务规划器
2. **可用意图列表**：10 种意图及其参数说明
3. **输出格式定义**：严格的 JSON schema
4. **Few-shot 示例**：
   - "起飞到15米高度" → JSON
   - "搜索汽车目标，高度10米，前方80米" → JSON
   - "fly to x=50, y=30 at 12 meters" → JSON

#### 3.8.3 API 调用流程

```python
def plan_mission(self, text: str) -> dict | None:
    prompt = MISSION_PROMPT_TEMPLATE.format(user_text=text)
    payload = json.dumps({
        "model": self.config.model,
        "messages": [{"role": "user", "content": prompt}],
        "max_tokens": self.config.max_tokens,
        "temperature": self.config.temperature,
    })
    req = urllib.request.Request(
        f"{base_url}/chat/completions",
        data=payload.encode("utf-8"),
        headers={...}
    )
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        body = json.loads(resp.read())
    return _extract_json(body["choices"][0]["message"]["content"])
```

#### 3.8.4 JSON 提取 (`_extract_json`)

处理 LLM 返回内容的各种格式：
1. 去除 Markdown 代码块标记（```json ... ```）
2. 尝试直接 `json.loads`
3. 如果失败 → 在文本中搜索第一个 `{` 到最后一个 `}`
4. 再次尝试解析
5. 全部失败 → 返回 None（触发规则引擎回退）

---

### 3.9 core/task_executor.py — 任务执行器

**文件**：[core/task_executor.py](../core/task_executor.py)（72行）

**职责**：对 `AirSimAdapter` 的薄封装层，提供标准化的任务执行接口。

```python
class TaskExecutor:
    def __init__(self, adapter: AirSimAdapter):
        self.adapter = adapter
        self.current_mission: MissionPlan | None = None

    def takeoff(self, altitude_m)     → self.adapter.takeoff(altitude_m)
    def hover(self)                   → self.adapter.hover()
    def land(self)                    → self.adapter.land()
    def goto_local(self, x, y, z, v)  → self.adapter.goto_local(x, y, z, v)
    def move_velocity(self, ...)       → self.adapter.move_velocity(...)
    def recover_from_collision(self,...)→ self.adapter.recover_from_collision(...)
```

这是一个**门面模式**（Facade）应用：未来如果需要替换底层仿真平台（如从 AirSim 切换到 Gazebo），只需修改 Adapter 而不影响上层任务执行逻辑。

---

### 3.10 core/safety_gate.py — 安全门校验

**文件**：[core/safety_gate.py](../core/safety_gate.py)（458行）

**职责**：在执行任何飞控操作之前进行多层次安全检查。这是整个系统最关键的安全保障模块。

#### 3.10.1 SafetyLimits

```python
@dataclass(frozen=True)
class SafetyLimits:
    min_altitude_m = 1.0           # 最低飞行高度
    max_altitude_m = 30.0           # 最高飞行高度
    max_speed_mps = 4.0             # 最大飞行速度
    boundary_x_min/max = -120/120   # X轴飞行边界
    boundary_y_min/max = -120/120   # Y轴飞行边界
    allowed_vehicles = ("Drone1".."Drone6")  # 允许控制的无人机
    allowed_cameras = ("front_center", ...)   # 允许的相机
    critical_tools = ("land", "return_home")  # 关键工具（需二次确认）
```

#### 3.10.2 安全校验流程

```python
def validate(self, tool: str, args: dict, connected: bool = False) -> SafetyDecision:
```

**第一层：工具白名单**
- 检查 tool 是否在 `allowed_tools()` 返回的 15 种工具列表中
- 不在 → `SafetyDecision(accepted=False)`

**第二层：关键工具确认**
- `land` 和 `return_home` 要求 `confirmed=true` 参数
- 未确认 → 拒绝

**第三层：Pydantic 数值校验**（调用 `ToolValidator.validate()`）
- 每种工具有独立的 Pydantic Model
- 校验字段范围、类型、必填项
- 不通过 → 拒绝并返回具体错误

**第四层：实时状态飞行前检查** (`_run_preflight_checks`)
- 如果设置了状态检查器（`set_state_checker`），执行：
  - **碰撞状态**：当前有碰撞 → 非 `recover_from_collision` 工具被阻止
  - **障碍物风险**：高风险 → 警告
  - **距离传感器紧急**：有紧急 → 警告
  - **当前高度过低**：低于紧急阈值 → 警告

**第五层：工具特定参数校验**
- 每类工具有专门的参数范围检查（高度、边界、间距等）
- 对 `search_area` 额外校验：`scan_margin_m`、`obstacle_distance_m`、`avoidance_offset_m`

#### 3.10.3 返回结果

```python
SafetyDecision:
    accepted: bool          # 是否通过
    tool: str               # 标准化后的工具名
    args: dict              # 清理/标准化后的参数
    warnings: [str]         # 警告信息列表
    reason: str             # 拒绝原因
    preflight_checks: dict  # 飞行前检查详情（前端渲染用）
```

---

### 3.11 core/validation.py — Pydantic 参数校验

**文件**：[core/validation.py](../core/validation.py)（186行）

**职责**：使用 Pydantic v2 为每种工具定义严格的参数模型。

#### 参��模型映射

| 工具 | Pydantic Model | 关键校验规则 |
|------|---------------|-------------|
| `takeoff` | `TakeoffParams` | altitude_m: 1.0~30.0m |
| `goto_local` | `GotoLocalParams` | x,y: ±120m, z: -30~0m, speed: 0.5~4.0m/s |
| `waypoint_route` | `WaypointRouteParams` | 1~40个航点 |
| `autonomous_nav` | `AutonomousNavParams` | replan: 0.3~5s, timeout: 5~900s |
| `formation_flight` | `FormationFlightParams` | count: 2~6, shape: v/line/delta |
| `search_area` | `SearchAreaParams` | 含区域、高度、间距、避障参数 |
| `collect_images` | `CollectImagesParams` | max_images: 1~5000, capture_interval: 0.2~30s |
| `return_home` | `ReturnHomeParams` | 需 confirmed=true |
| `replay_trajectory` | `ReplayTrajectoryParams` | log_folder必填, turn: 5~170° |

#### 校验流程

```python
class ToolValidator:
    _SCHEMAS: dict[str, type[BaseModel]] = {...}

    @classmethod
    def validate(cls, tool: str, args: dict) -> ValidationResult:
        schema = cls._SCHEMAS.get(tool)
        if schema is None:
            return ValidationResult.failure([f"Unknown tool: {tool}"])
        validated = schema(**args)           # Pydantic自动校验
        return ValidationResult.success(validated.model_dump())
```

Pydantic 的 `Field(ge=..., le=...)` 约束和 `@model_validator` 自动处理范围检查、类型转换和交叉字段验证。

---

### 3.12 core/agent_tools.py — Agent 工具运行时

**文件**：[core/agent_tools.py](../core/agent_tools.py)（~2033行）

**职责**：实现 15 种 LLM Agent 可调用的工具，每种工具在独立线程中运行，支持中止和进度报告。

#### 3.12.1 工具列表

| 工具 | 实现方法 | 线程安全 | 支持中止 |
|------|---------|---------|---------|
| `takeoff` | `_takeoff` | ✓ | - |
| `hover` | `_hover` | ✓ | - |
| `stop` | `_stop` | ✓ | - |
| `land` | `_land` | ✓ | - |
| `return_home` | `_return_home` | ✓ | - |
| `goto_local` | `_goto_local` | ✓ | ✓ |
| `waypoint_route` | `_waypoint_route` | ✓ | ✓ |
| `autonomous_nav` | `_autonomous_nav` | ✓ | ✓ |
| `formation_flight` | `_formation_flight` | ✓ | ✓ |
| `recover_from_collision` | `_recover_from_collision` | ✓ | - |
| `search_area` | `_search_area` | ✓ | ✓ |
| `collect_images` | `_collect_images` | ✓ | ✓ |
| `replay_trajectory` | `_replay_trajectory` | ✓ | ✓ |
| `detect_objects` | `_detect_objects` | - | - |
| `report_target` | `_report_target` | - | - |

#### 3.12.2 任务中止机制

```python
def _prepare_new_mission(self, timeout=3.0):
    self._mission_stop.set()           # 通知旧任务线程停止
    old = self._mission_thread
    if old is not None and old.is_alive():
        old.join(timeout=timeout)      # 等待旧线程退出
    self._mission_stop.clear()         # 重置停止标志

def call_tool(self, tool, args):
    self._prepare_new_mission()        # 确保上一次任务已停止
    self._mission_thread = threading.Thread(
        target=self._mission_runner,
        args=(tool, args),
        daemon=True
    )
    self._mission_thread.start()
```

每个长时间运行的工具（如 `_autonomous_nav`、`_search_area`）在其主循环中检查 `self._mission_stop.is_set()`：
```python
while not self._mission_stop.is_set():
    # 执行一个航点/一个控制周期
    ...
```

#### 3.12.3 进度报告 (`_push_progress`)

```python
def _push_progress(self, status, message, **extra):
    with self._mission_lock:
        self._mission_progress.update({
            "status": status,
            "message": message,
            "updated_at": time.time(),
            **extra
        })
```

前端通过 WebSocket 获取 `agent_progress`，实时显示当前任务阶段、航点进度、编队状态等。

#### 3.12.4 关键工具实现

**`_search_area`** — 区域搜索（最复杂的工具）：
1. 解析搜索区域和参数
2. 如果启用 `pre_scan`：
   - 起飞后先调用 `preflight.blocked_zones_from_points()` 分析 LiDAR 点
   - 计算 `assessment_risk` 和 `area_coverage`
   - 高风险且 `pre_scan_stop_on_high_risk=true` → 终止
3. 生成 lawnmower 路径
4. 如果启用 `planned_avoidance`：
   - 调用 `preflight.replan_route_around_zones()` 重新规划绕过障碍的路线
5. 沿路线飞行，每个到达点执行 `detect_objects`
6. 检测到目标 → 报告并悬停

**`_autonomous_nav`** — 闭环自主导航：
1. 获取当前 LiDAR 障碍点
2. 调用 `plan_local_path()` 生成 A* 路径
3. 取路径的前 `lookahead_m` 米作为短期目标
4. 使用 `move_velocity` 进行 PD 控制飞行
5. 每个 `replan_interval_s` 秒重新规划
6. 到达目标或超时

**`_collect_images`** — 数据集采集：
1. 创建 `datasets/yolo_collect/<dataset_name>/images/` 目录
2. 沿 lawnmower 路径飞行
3. 按 `capture_interval_s` 间隔拍摄相机帧
4. YOLO 自动标注并保存 labels

**`_replay_trajectory`** — 轨迹回放：
1. `replay.load_replay_frames(log_folder)` — 从 JSON 文件加载轨迹
2. `replay.deduplicate_replay_frames(frames, min_dist_m)` — 去重（位置变化<0.5m的帧）
3. `replay.split_replay_by_turns(waypoints, threshold)` — 按大角度转弯分段
4. 在地面生成目标对象（`spawn_object`）
5. 按航点逐个飞行，绘制轨迹线

---

### 3.13 core/vision_detector.py — YOLO 视觉检测

**文件**：[core/vision_detector.py](../core/vision_detector.py)（363行）

**职责**：使用 YOLO 模型进行目标检测，支持 YOLOv8 (ultralytics) 和 YOLOv5 (torch.hub)。

#### 3.13.1 模型加载策略

```python
_load_model():
    1. 尝试: from ultralytics import YOLO; YOLO(model_path)    # YOLOv8
    2. 失败且错误含"YOLOv5 forwards compatible":
       → _try_load_yolov5_model()
         → torch.hub.load('ultralytics/yolov5', 'custom', path=model_path)
         → 包装为 YOLOv5Wrapper（提供与YOLOv8兼容的接口）
    3. 全部失败: _load_error 记录原因
```

#### 3.13.2 兼容性包装

为了统一 YOLOv5 和 YOLOv8 的接口差异，定义了三个包装类：

```python
YOLOv5Wrapper        # 包装YOLOv5模型，提供 __call__(image, conf) 接口
YOLOv5ResultWrapper  # 包装检测结果，提供 .boxes 属性
YOLOv5BoxesWrapper   # 包装检测框列表，支持迭代
YOLOv5BoxWrapper     # 包装单个检测框，提供 .xyxy, .conf, .cls
```

#### 3.13.3 异步检测

```python
self._pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="yolo-detect")

def detect_async(self, image, target_filter=None) -> Future[DetectionResult]:
    return self._pool.submit(self.detect, image, target_filter)
```

使用 `max_workers=1` 的单线程池确保 GPU 推理串行执行，防止多个并发请求导致的显存溢出。

#### 3.13.4 检测流程

```python
def detect(image: bytes, target_filter=None) -> DetectionResult:
    1. _decode_image() → numpy数组 (cv2.imdecode)
    2. model(image_array, conf=self.confidence, verbose=False)
    3. _parse_ultralytics(results) → [Detection(label, confidence, bbox)]
    4. target_filter → _filter_by_target() (按标签名过滤)
    5. 返回 DetectionResult(ok=True, detections=[...], elapsed_ms=...)
```

---

### 3.14 core/video_stream.py — MJPEG 视频流

**文件**：[core/video_stream.py](../core/video_stream.py)（95行）

**职责**：从 AirSim 持续捕获相机画面，以 MJPEG 格式推送给浏览器。

#### 3.14.1 采集线程

```python
_capture_loop():
    间隔 = 1.0 / target_fps (默认8fps)
    循环:
        frame = adapter.camera_frame()     # 从AirSim获取一帧
        if frame: 更新 last_frame
        else:     failed_count++
                  if failed >= 5: _try_switch_camera()  # 自动切换备用相机
        sleep(interval - elapsed)
```

#### 3.14.2 MJPEG 推送

```python
# server.py 中的 send_mjpeg():
self.send_response(200)
self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=aeromind-frame")
for frame in video_stream.frames(interval_s=0.08):
    self.wfile.write(
        f"--aeromind-frame\r\n"
        f"Content-Type: {frame.content_type}\r\n"
        f"Content-Length: {len(frame.data)}\r\n\r\n".encode()
    )
    self.wfile.write(frame.data)
    self.wfile.write(b"\r\n")
```

浏览器 `<img src="/video/front">` 自动解析 MJPEG 流并持续更新图像。

#### 3.14.3 相机自动切换

当连续 5 次获取帧失败时，自动尝试切换到下一个可用相机：
```python
_try_switch_camera():
    for camera_name in adapter.camera_candidates:
        if camera_name != 当前相机:
            adapter.select_camera(camera_name)
            break
```

---

### 3.15 core/formation_manager.py — 编队管理

**文件**：[core/formation_manager.py](../core/formation_manager.py)（325行）

**职责**：计算多无人机编队队形，管理编队协同飞行。

#### 3.15.1 支持的队形

| 队形 | 方法 | 排列描述 |
|------|------|---------|
| V 字形 | `_create_v_formation` | 经典的箭头形状，长机在前、僚机左右对称展开 |
| 一字形 | `_create_line_formation` | 水平排列，中间为长机 |
| 三角翼 | `_create_delta_formation` | 战斗机风格，前尖后宽 |
| 菱形 | `_create_diamond_formation` | 四面对称，长机前、三僚机后 |
| 三角形 | `_create_triangle_formation` | 前尖后宽的稳定三角 |
| 纵队 | `_create_column_formation` | 前后排列 |

每种队形为 2-6 架无人机计算相对偏移量。长机的偏移为 (0, 0, 0)，僚机按队形规则偏移。

#### 3.15.2 编队协同

```python
class SwarmCoordinator:
    def move_formation(self, leader_x, leader_y, leader_z, speed):
        goal_positions = calculate_goal_positions(leader_x, leader_y, leader_z)
        for vehicle_name, (gx, gy, gz) in goal_positions.items():
            switch_vehicle(vehicle_name)
            goto_local(gx, gy, gz, speed)
```

核心思想：长机按航线飞行，僚机的目标位置 = 长机位置 + 队形相对偏移。

---

### 3.16 core/preflight.py — 飞行前检查

**文件**：[core/preflight.py](../core/preflight.py)（187行）

**职责**：在任务执行前基于 LiDAR 点云分析区域安全性。

**纯函数设计**：所有函数无副作用、无类依赖，从 `agent_tools.py` 提取而来。

#### 核心函数

**`blocked_zones_from_points(area, points)`** — 将 LiDAR 点聚合为阻塞区域：
1. 将点按 2m×2m 单元格分组
2. 点数 ≥4 的单元格标记为阻塞区
3. 返回按点数降序排列的阻塞区列表

**`count_route_hits(route, zones)`** — 统计航线穿越阻塞区的次数：
- 使用 `segment_intersects_zone()` 检查每个航线段是否与阻塞区相交（padding=1.5m）

**`replan_route_around_zones(route, zones, padding_m)`** — 绕开阻塞区重新规划航线：
- 对于每条水平航线段（y不变），计算阻塞区占用的 x 区间
- 使用 `subtract_intervals()` 计算安全区间
- 放弃过短的片段（<3m）
- 返回新的航点序列

**`area_coverage(area, zones)`** — 计算阻塞区占搜索面积的比例

**`assessment_risk(coverage, route_hits)`** — 风险评估：
- `critical`: route_hits ≥3 或 coverage ≥28%
- `high`: route_hits ≥1 或 coverage ≥12%
- `medium`: coverage ≥4%
- `low`: 其他

**`inset_area(area, margin_m)`** — 从搜索区域向内收缩边距

---

### 3.17 core/replay.py — 轨迹回放

**文件**：[core/replay.py](../core/replay.py)（93行）

**职责**：从录制的 JSON 轨迹文件中加载、去重、分段并执行回放。

**纯函数设计**，从 `agent_tools.py` 提取而来。

**`load_replay_frames(log_folder)`** — 加载轨迹帧：
- 扫描目录中的 `.json` 文件，按数字文件名排序
- 解析 JSON 内容为帧列表

**`deduplicate_replay_frames(frames, min_dist_m=0.5)`** — 去重：
- 位置变化 <0.5m 的连续帧合并
- 提取每帧的 `position` 和 `orientation`

**`turn_angle_deg(waypoints, i)`** — 计算航点 i 处的转弯角度：
- 使用 `atan2` 计算入方向和出方向的夹角

**`split_replay_by_turns(waypoints, threshold_deg)`** — 按大角度转弯分割轨迹：
- 转弯超过阈值的位置作为分段点

**`closest_spawn_area(coord, areas)`** — 找到最近的出生区域（从 `map_spawnarea_info.json` 匹配）

---

### 3.18 core/websocket.py — WebSocket 协议实现

**文件**：[core/websocket.py](../core/websocket.py)（104行）

**职责**：实现 RFC 6455 WebSocket 协议的核心原语。**纯标准库实现，零外部依赖。**

#### 3.18.1 握手

```python
WS_MAGIC = b"258EAFA5-E914-47DA-95CA-C5AB0DC85B11"  # RFC 6455 定义

def ws_accept_key(client_key: str) -> str:
    # SHA1(client_key + MAGIC) → Base64
    digest = hashlib.sha1(client_key.encode() + WS_MAGIC).digest()
    return base64.b64encode(digest).decode()
```

#### 3.18.2 帧协议

**发送文本帧** (`ws_send_text`)：
```
字节0:    0x81 (FIN=1, RSV=0, Opcode=1/text)
字节1:    长度编码:
           0-125   → 直接
           126-65535 → 0x7E + 2字节大端
           >65535  → 0x7F + 8字节大端
字节n+:   UTF-8 payload
```

**接收帧** (`ws_recv_frame`)：
```
读取字节0 → 提取 opcode
读取字节1 → 提取 MASK 位和长度
按长度读取扩展长度字段
读取4字节 mask key
读取 payload 并解掩码: byte ^ mask_key[i%4]
返回 (opcode, payload)
```

#### 3.18.3 心跳

```python
def ws_serve(wfile, rfile, on_message, heartbeat_s=15.0):
    while True:
        opcode, data = ws_recv_frame(rfile)
        if opcode == 0x8 (close): ws_send_close(wfile); return
        if opcode == 0x9 (ping):  ws_send_pong(wfile, data)
        if opcode == 0x1 (text):  on_message(data.decode())
        if idle > heartbeat_s:    ws_send_pong(wfile)  # 保活
```

#### 3.18.4 在 server.py 中的使用

`_handle_ws` 只使用了 `ws_handshake`（验证）和 `ws_send_text`（推送）。不读取客户端帧（仅服务端单向推送），依赖 OS 层的 `BrokenPipeError` 检测断线。

---

### 3.19 core/auth.py — 认证与鉴权

**文件**：[core/auth.py](../core/auth.py)（98行）

**职责**：API 访问控制，包含三层安全机制。

```python
class AuthManager:
    def authenticate(self, handler, client_ip, headers) -> (status, data, extra_headers):
```

**第一层：IP 白名单**
```python
_check_ip(client_ip):
    return client_ip in self.config.allowed_ips
# 默认: ("127.0.0.1", "::1", "localhost")
```

**第二层：API Key 校验**
```python
_check_api_key(provided_key):
    if not self.config.api_key:    # 未配置密钥 → 跳过检查
        return True
    return SHA256(provided_key) == self._api_key_hash  # 恒定时间比较
```
密钥使用 SHA256 哈希存储，不保存明文。

**第三层：速率限制**
```python
_check_rate_limit(client_ip):
    滑动窗口: 60秒内最多60个请求
    超限 → HTTP 429 + Retry-After 头
```

认证可以禁用（`AuthConfig(enabled=False)`），但默认启用。

---

### 3.20 core/services.py — 事件/状态/路由管理

**文件**：[core/services.py](../core/services.py)（174行）

包含三个核心服务类，管理任务运行时的"软状态"。

#### EventManager

```python
class EventManager:
    _events: [EventItem]     # 最近80条事件（倒序，最新的在前）
    _log_file: Path          # 持久化日志文件

    log(message, level)      # 记录事件，写入文件
    get_events()             # 返回事件列表（供API使用）
    clear()                  # 清空日志文件
```

#### MissionState

```python
class MissionState:
    _task_status: str         # idle|planning|running|completed|stopped|failed
    _current_task: str        # 当前任务标题
    _plan: [dict]             # 当前计划步骤

    set_mission(mission)      # 从 MissionPlan 设置状态
    set_status(status, msg)   # 更新状态和消息
    reset()                   # 重置为空闲
```

#### RouteManager

```python
class RouteManager:
    _trail: [dict]            # 飞行轨迹（最近240个点，采样间隔≥0.5m）
    _route: [dict]            # 计划航线（lawnmower等生成的航点）
    _search_area: dict|null   # 搜索区域边界
    _targets: [dict]          # 检测到的目标

    update_trail(x, y, z)     # 添加轨迹点（自动去重，距离<0.5m的点跳过）
    generate_search_route()   # 使用 lawnmower_path 生成航线
```

---

### 3.21 core/metrics.py — 指标采集与性能追踪

**文件**：[core/metrics.py](../core/metrics.py)（284行）

#### MetricsCollector

```python
class MetricsCollector:
    _metrics: {name: [MetricPoint]}  # 最多每种1000个数据点
    _counters: {name: int}           # 计数器
    _gauges: {name: float}           # 瞬时值
    _timers: {name: [float]}         # 计时器（最近100次）

    record_counter(name, value=1)    # 递增计数器
    record_gauge(name, value)        # 设置瞬时值
    record_timer(name, duration_ms)  # 记录耗时
    get_summary()                    # 返回统计摘要
```

#### StructuredLogger

结构化 JSON 日志，可选写入文件。支持 `debug/info/warn/error/critical` 五个级别。

#### TelemetryMonitor

关联遥测数据到指标：
```python
record_telemetry(telemetry):
    metrics.record_gauge("altitude", telemetry["altitude_m"])
    metrics.record_gauge("speed", telemetry["speed_mps"])
    metrics.record_gauge("position_x", telemetry["x"])
```

#### PerformanceTracker

组合以上三个组件，提供 `start_timer/stop_timer` 和上下文管理器 `track_execution`。

---

### 3.22 core/storage.py — SQLite 持久化

**文件**：[core/storage.py](../core/storage.py)（230行）

**职责**：任务和飞行记录的持久化存储。

#### 数据表

```sql
tasks (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    task_type TEXT,      -- takeoff/search/formation/...
    task_text TEXT,      -- 原始自然语言输入
    status TEXT,         -- pending/running/completed/failed/cancelled
    created_at TEXT,     -- ISO格式时间戳
    started_at TEXT,
    completed_at TEXT,
    plan_json TEXT,      -- MissionPlan 的 JSON 序列化
    result_json TEXT     -- 执行结果的 JSON
)

flight_logs (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    task_id INTEGER REFERENCES tasks(id),
    timestamp TEXT,
    x REAL, y REAL, z REAL,        -- 位置
    altitude_m REAL, speed_mps REAL, -- 高度和速度
    status TEXT
)
```

#### 主要 API

```python
create_task(type, text, plan_json) → task_id
update_task_status(task_id, status)
update_task_result(task_id, result_json)
get_task(task_id) → TaskRecord | None
list_tasks(status=None, limit=50) → [TaskRecord]
add_flight_log(task_id, x, y, z, alt, speed, status)
get_flight_logs(task_id) → [FlightLogRecord]
delete_task(task_id) → bool
get_stats() → {total, completed, failed, running}
```

---

### 3.23 core/exceptions.py — 异常体系

**文件**：[core/exceptions.py](../core/exceptions.py)（132行）

**职责**：统一的错误处理，将 Python 异常映射为 HTTP 状态码。

#### 异常层次

```
AeroMindError (HTTP 500)
├── AuthError (401)
├── RateLimitError (429)
├── ValidationError (400)
├── SafetyError (403)
├── AirSimError (503)
├── TaskError (500)
├── NotFoundError (404)
├── ConflictError (409)
└── PreconditionError (412)
```

每个异常自动携带 `error_code`（如 `AM_AUTH_ERROR`）和 `http_status`。

#### ErrorHandler

```python
handle_exception(exc) → (http_status, error_dict):
    if AeroMindError → exc.http_status + exc.to_api()
    if ValueError    → 400 + validation error
    if TypeError     → 400 + type error
    else             → 500 + traceback（截断为2000字符）
```

---

### 3.24 core/di.py — 依赖注入容器

**文件**：[core/di.py](../core/di.py)（69行）

轻量级 IoC 容器，避免硬编码依赖关系。

```python
class DependencyContainer:
    register_singleton(cls, instance)   # 注册已创建的实例
    register_factory(cls, factory)       # 注册延迟创建的工厂
    resolve(cls) → instance              # 获取实例（触发工厂）
```

`ServiceManager` 封装了 `DependencyContainer`，`Inject[T]` 描述符支持声明式注入：
```python
class SomeService:
    airsim: AirSimAdapter = Inject(AirSimAdapter)
```

---

### 3.25 core/async_tools.py — 异步工具运行时

**文件**：[core/async_tools.py](../core/async_tools.py)（173行）

**职责**：提供 asyncio 兼容的工具调用接口（为未来异步架构预留）。

`AsyncToolRuntime` 将所有同步工具方法包装为 `async def`，使用 `loop.run_in_executor(None, func)` 在线程池中执行。

`AsyncTaskRunner` 实现基于 asyncio 的任务流转（`run_task/cancel_task/stop`）。

---

## 4. 前端架构

### 4.1 index.html — UI 布局

**文件**：[static/index.html](../static/index.html)（219行）

整体布局分为三个区域：

```
┌─────────────────────────────────────────────┐
│  .topbar: 品牌 + 连接/视觉/任务状态指示器    │
├──────────────────────┬──────────────────────┤
│  .topDeck            │  .missionPanel       │
│  ┌─────────────────┐ │  ┌────────────────┐  │
│  │ .videoStage     │ │  │ .promptPanel   │  │
│  │ (相机画面+叠加) │ │  │ (任务输入+参数)│  │
│  └─────────────────┘ │  ├────────────────┤  │
│  ┌─────────────────┐ │  │ .planPanel     │  │
│  │ .mapStage       │ │  │ (大模型计划)   │  │
│  │ (Canvas地图)    │ │  ├────────────────┤  │
│  └─────────────────┘ │  │ .safePanel     │  │
│                       │  │ (飞控按钮)     │  │
│                       │  ├────────────────┤  │
│                       │  │ .logPanel      │  │
│                       │  │ (任务日志)     │  │
│                       │  └────────────────┘  │
├──────────────────────┴──────────────────────┤
│  .bottomDeck: 任务状态/位置/进度/事件流     │
└─────────────────────────────────────────────┘
```

通过 CSS Grid 实现响应式布局。`<script type="module">` 加载 ES 模块入口。

### 4.2 app.js — 入口与事件绑定

**文件**：[static/app.js](../static/app.js)（~189行）

**职责**：模块编排 + 事件绑定 + WebSocket 连接 + 初始化。

```javascript
// 从三个模块导入所需函数
import { state, MAP_ZOOM_STEP, templates, el, on, fmt, toast, withBusy } from "./js/common.js";
import { canvasToMap, resizeMissionMap, drawMissionMap, zoomMissionMap, setMapFollow, fitMissionMap } from "./js/map.js";
import { api, render, renderTelemetry, updateClock, previewTask, runTask, command, flightCommand, detectLatest, applyTemplateParams, buildTaskTextFromParams, renderMissionLog } from "./js/ui.js";
```

#### 事件绑定

| 交互元素 | 事件 | 处理 |
|---------|------|------|
| `.templateChip` | click | 应用模板参数 + 自动预览 |
| 参数输入控件 | change | 更新任务文本 + 自动预览 |
| `#previewBtn` | click | `previewTask()` — 预览不执行 |
| `#runBtn` | click | `runTask()` — 提交执行 |
| `#detectBtn` | click | `detectLatest()` — 触发检测 |
| `#takeoffBtn` | click | `flightCommand("/api/flight/takeoff")` |
| `#hoverBtn` | click | `flightCommand("/api/flight/hover")` |
| `#landBtn` | click | 确认弹窗 → `flightCommand("/api/flight/land")` |
| `#pauseBtn` | click | `command("/api/task/pause")` |
| `#stopBtn` | click | 确认弹窗 → `command("/api/task/stop")` |
| `#rtlBtn` | click | 确认弹窗 → `command("/api/task/rtl")` |
| `#voiceBtn` | click | 切换 `state.isListening`，预留语音入口 |
| `#missionMap` | mousemove | 拖拽平移 / 光标坐标显示 |
| `#missionMap` | mousedown | 开始拖拽 |
| window | mouseup | 停止拖拽 |
| `#missionMap` | wheel | 缩放（以鼠标位置为锚点） |
| `#mapFollowBtn` | click | 切换跟随模式 |
| `#mapFitBtn` | click | 自动缩放到所有数据可见 |
| `#mapZoomInBtn` | click | 放大 |
| `#mapZoomOutBtn` | click | 缩小 |
| `#connectionPill` | click | 触发 AirSim 重连 |
| `#visionPill` | click | 探测可用相机 |
| `.cameraChip` | click | 切换相机视角 |
| `#vehicleSelect` | change | 切换无人机 |
| `#clearLogBtn` | click | 清空任务日志 |

#### WebSocket 连接

```javascript
function connectWS() {
    const ws = new WebSocket(`${protocol}//${location.host}/ws`);
    ws.onmessage = (event) => {
        const msg = JSON.parse(event.data);
        if (msg.type === "state")      render(msg.data);       // 完整状态
        if (msg.type === "telemetry")  renderTelemetry(msg.data); // 遥测
    };
    ws.onclose = () => setTimeout(connectWS, 1200);  // 自动重连
}
```

#### 初始化

```javascript
api("/api/state").then(render);        // 首次HTTP获取（秒开）
api("/api/telemetry").then(renderTelemetry);
resizeMissionMap();
connectWS();                            // 后续WS推送
setInterval(updateClock, 1000);         // 时钟更新
window.addEventListener("resize", resizeMissionMap);
```

### 4.3 common.js — 全局状态与工具函数

**文件**：[static/js/common.js](../static/js/common.js)（172行）

#### 全局状态对象 `state`

```javascript
export const state = {
    map: null,              // 地图数据（uav, trail, route, obstacles...）
    mapPxPerMeter: 4,       // 地图缩放比例（像素/米）
    mapViewX: 25, mapViewY: 0,  // 地图视口中心（世界坐标）
    mapFollow: true,        // 是否跟随无人机
    mapDragging: false,     // 是否正在拖拽
    currentStatus: "idle",  // 当前任务状态
    safety: null,           // 安全状态
    agentProgress: null,    // Agent执行进度
    selectedVehicle: "",    // 当前选中的无人机
    selectedCamera: "",     // 当前相机
    previewMapActive: false,// 是否显示预览地图
    missionLogs: [],        // 任务日志（最近50条）
    lastPlan: [],           // 上一次任务计划
    pageStartedAt: Date.now()/1000,
};
```

#### 导出常量

```javascript
MAP_ZOOM_MIN = 0.6, MAP_ZOOM_MAX = 36, MAP_ZOOM_STEP = 1.18
text: { simConnected, simOffline, visionPending, ... }       // UI文本字典
riskLabels: { clear:"清空", low:"低", medium:"中", ... }      // 风险标签
directionLabels: { none, front, left, right, ... }            // 方向标签
templates: { search, scan, formation }                         // 任务模板
templateParams: { search:{...}, scan:{...}, formation:{...} } // 模板默认参数
```

#### 工具函数

```javascript
el(id)          // document.getElementById 快捷方式
on(id, event, handler)  // addEventListener 快捷方式
checked(id, fallback)   // 读取checkbox状态
valueOf(id, fallback)   // 读取input值
fmt(value, digits)      // 数值格式化（保留digits位小数）
clamp(value, min, max)  // 数值钳制
numInput(id, fallback)  // 读取数值输入
statusText/riskText/directionText/plannerText/statusLabel  // 文本映射
toast(title, detail)    // 弹出Toast通知（3.2秒自动消失）
withBusy(button, work)  // 按钮忙状态包装器
```

### 4.4 map.js — Canvas 2D 地图引擎

**文件**：[static/js/map.js](../static/js/map.js)（408行）

**职责**：在 HTML5 Canvas 上渲染 2D 俯视地图，包含坐标系、网格、航线、轨迹、障碍物、无人机图标等。

#### 坐标系统

```javascript
// 世界坐标 → Canvas像素坐标
mapToCanvas(x, y) → [px, py]:
    px = canvasWidth/2  + (y - mapViewY) * mapPxPerMeter
    py = canvasHeight/2 - (x - mapViewX) * mapPxPerMeter

// Canvas像素坐标 → 世界坐标
canvasToMap(cx, cy) → [x, y]:
    x = mapViewX + (canvasHeight/2 - cy) / mapPxPerMeter
    y = mapViewY + (cx - canvasWidth/2)  / mapPxPerMeter
```

注意：AirSim 使用 NED 坐标系（X=北/前，Y=东/右），Canvas 使用屏幕坐标系（X=右，Y=下）。`mapToCanvas` 巧妙地交换了 X/Y 轴并翻转了方向。

#### 缩放级别

```python
mapZoomLevel() → 1~10:
    对数映射: log(pxPerMeter/MIN) / log(MAX/MIN)
    线性映射到 1-10 的整数级别
```

#### 渲染层次

`drawMissionMap()` 按以下层次绘制（从下到上）：
1. 清空背景（`#091116` 深色）
2. 网格线（间距 = `max(24, pxPerMeter * 10)` 像素）
3. 搜索区域（半透明青色矩形）
4. 阻塞区（半透明红色矩形）
5. LiDAR 障碍点（红色小方块）
6. 任务航线（金色虚线）
7. A* 规划路径（蓝色实线 + 关键航点标记）
8. 飞行轨迹（青色实线）
9. 编队目标连线（橙色虚线）
10. 原点标记（蓝色圆 + "H" 标签）
11. 目标标记（红色圆 + "T" 标签）
12. 无人机图标（长机：青色菱形 + 信息标签；僚机：橙色圆形）

#### 碰撞重叠处理

```javascript
visibleUavs(uavs):
    当多架无人机位于同一坐标时:
      按 slot 序号分配圆周偏移位置
      偏移距离 = 13px
      角度 = slot/总数 * 2π
```

#### 地图交互

- **拖拽**：mousedown 开始 → mousemove 平移视口 → mouseup 结束
- **缩放**：滚轮事件以鼠标位置为锚点缩放
- **跟随**：开启后每帧将 mapViewX/Y 同步到无人机位置

### 4.5 ui.js — DOM 渲染与 API 调用

**文件**：[static/js/ui.js](../static/js/ui.js)（516行）

**职责**：所有 DOM 更新和 HTTP API 调用。

#### API 层

```javascript
async function api(path, body=null):
    POST (有body时) 或 GET
    Content-Type: application/json
    自动解析 JSON 响应
    非 2xx → throw Error
```

#### 主要渲染函数

| 函数 | 更新元素 | 数据来源 |
|------|---------|---------|
| `render(data)` | 全局状态、地图、计划、事件、日志 | `/api/state` |
| `renderTelemetry(data)` | 遥测、车辆选择器、地图轨迹 | `/api/telemetry` |
| `renderOpsOverview(data)` | 阶段/航点/距离/风险徽章、问题横幅 | `agent_progress` |
| `renderVideo(video)` | 相机画面、回退界面 | `video` 对象 |
| `renderCameraSwitcher(video)` | 相机按钮激活状态 | `selected_camera_name` |
| `renderVehicleSelector(vehicle)` | 无人机下拉框 | `vehicle.vehicles` |
| `renderPlan(plan, status, progress)` | 任务步骤列表、置信度条 | `plan` 数组 |
| `renderRoute(plan, status, progress)` | 进度点线 | 航点进度 |
| `renderToolPreview(preview)` | 工具预览面板（航点/距离/安全检查） | `/api/task/preview` |
| `renderEvents(events)` | 事件列表（最近8条，过滤历史） | `events` 数组 |
| `renderPerception(data)` | 传感器面板（13项指标） | `safety`, `vision`, `agent_progress` |
| `renderMissionLog()` | 任务日志列表 | `state.missionLogs` |

#### 任务操作函数

```javascript
collectTaskParams()           → 从表单控件收集任务参数
applyTemplateParams(name)     → 将模板参数写入表单控件
buildTaskTextFromParams()     → 从参数生成中文任务描述
previewTask(button)           → POST /api/task/preview → 渲染预览
runTask(button)               → POST /api/task/run → 提交执行
command(path, message)        → POST 命令 → 渲染结果 + toast
flightCommand(path, msg, ...) → POST 飞控命令
detectLatest(button)          → POST /api/detect/latest
updateMissionLog(data)        → 追加日志条目（去重）
```

---

## 5. 核心数据流

### 5.1 任务提交流

```
用户输入 "搜索前方60米区域，发现车辆后悬停"
  → app.js: runTask()
    → POST /api/task/run {text: "...", params: {...}}
      → server.py: console.run_task(text, params)
        → TaskPlanner.plan(text)
          → LLMClient.plan_mission(text)  [LLM路径]
          ──或──
          → _plan_rules(text)             [规则回退]
        → MissionPlan {intent:"area_search", area:{...}, plan:[...]}
        → SafetyGate.validate("search_area", args, connected)
          → ToolValidator.validate()      [Pydantic校验]
          → _run_preflight_checks()       [实时状态检查]
        → SafetyDecision(accepted=True, args={...})
        → AgentToolRuntime.call_tool("search_area", args)
          → _prepare_new_mission()        [停止旧任务]
          → 启动 _search_area 线程
            → preflight 飞行前扫描
            → lawnmower 路径生成
            → preflight 路线重规划（避障）
            → 沿航线飞行 + 视觉检测
            → _push_progress() 更新进度
  → WebSocket 推送进度到前端
  → 前端: render() 更新地图/状态/日志
```

### 5.2 实时遥测推流

```
AirSim 仿真 (200Hz)
  → AirSimAdapter.telemetry()        [读取多旋翼状态]
  → ConsoleState.telemetry_snapshot() [组装遥测包]
  → WebSocket: {"type": "telemetry", "data": {...}}   [每200ms]
  ──或──
  → ConsoleState.snapshot()          [完整状态快照]
  → WebSocket: {"type": "state", "data": {...}}        [每1000ms]
  → 前端: render(msg.data) / renderTelemetry(msg.data)
```

遥测包包含:
```json
{
  "connected": true,
  "uav": {"x": 12.3, "y": 5.1, "z": -8.0, "altitude_m": 8.0, "speed_mps": 1.2},
  "uavs": [...],  // 所有无人机遥测
  "map": {
    "uav": {...},
    "uavs": [...],
    "trail": [...]  // 最近240个轨迹点
  }
}
```

状态包在遥测基础上增加了:
```json
{
  "task": {"status": "running", "title": "...", "agent_progress": {...}},
  "safety": {"collision": {...}, "obstacle": {...}, "distance_sensors": {...}},
  "video": {"mode": "airsim_front", "camera_name": "front_center"},
  "events": [...],
  "map": {..., "route": [...], "search_area": {...}, "obstacles": [...]}
}
```

### 5.3 安全门校验流

```
工具调用请求 (tool="search_area", args={xxx})
  → SafetyGate.validate(tool, args, connected)
    ├─ 第一层: 工具白名单检查
    │   tool 在 allowed_tools() 中？
    │
    ├─ 第二层: 关键工具确认
    │   tool 是 land/return_home 且无 confirmed=true？
    │
    ├─ 第三层: Pydantic 参数校验
    │   ToolValidator.validate(tool, args)
    │   → SearchAreaParams(**args) 触发 Pydantic 校验
    │   → 范围/类型/必填项检查
    │
    ├─ 第四层: 实时状态飞行前检查
    │   _run_preflight_checks(tool, args, safety_state)
    │   → 碰撞状态 → 非恢复工具 blocked
    │   → 障碍物风险 → warn
    │   → 距离传感器紧急 → warn
    │
    └─ 第五层: 工具特定参数校验
        _validate_search_area(x_min, x_max, ...)
        → 边界检查（±120m）
        → 间距检查（2~30m）
        → 安全距离检查（3~20m）
  → SafetyDecision(accepted, args, warnings, preflight_checks)
```

---

## 6. 安全机制

### 6.1 网络安全

| 机制 | 实现 | 位置 |
|------|------|------|
| API Key 认证 | SHA256 哈希比较，恒定时间 | `auth.py:_check_api_key` |
| IP 白名单 | 仅允许 localhost 系列 IP | `auth.py:_check_ip` |
| 速率限制 | 滑动窗口 60req/60s | `auth.py:_check_rate_limit` |
| 请求体限制 | 1MB 上限 | `server.py:MAX_CONTENT_LENGTH` |
| 路径遍历防护 | resolve + relative_to | `server.py:send_static` |
| WebSocket 握手验证 | RFC 6455 Key 验证 | `websocket.py:ws_handshake` |

### 6.2 飞行安全

| 机制 | 实现 | 位置 |
|------|------|------|
| 高度边界 | 1.0~30.0m | `safety_gate.py:SafetyLimits` |
| 地理边界 | ±120m X/Y | `safety_gate.py:SafetyLimits` |
| 速度限制 | 0.5~4.0 m/s | `safety_gate.py:SafetyLimits` |
| 关键工具确认 | land/return_home 需 confirmed=true | `safety_gate.py:validate` |
| 碰撞检测 | 碰撞→阻止非恢复工具 | `safety_gate.py:_run_preflight_checks` |
| 碰撞恢复 | 爬升+后退+强制重定位 | `airsim_adapter.py:recover_from_collision` |
| 紧急气泡 | 距离传感器≤2.5m自动限制 | `airsim_adapter.py:distance_sensors_status` |
| 位置保持 | PD控制器防漂移 | `airsim_adapter.py:_position_hold_loop` |
| 任务中止 | threading.Event + 循环检查 | `agent_tools.py:_mission_stop` |

### 6.3 飞行前检查

| 检查项 | 实现 |
|--------|------|
| 障碍物密度分析 | 2m×2m单元网格计数 |
| 航线冲突检测 | 段-区域相交算法（padding=1.5m） |
| 航线重规划 | 阻塞区间减法国，最小段3m |
| 风险评估 | critical/high/medium/low 四级 |
| 区域覆盖率 | 阻塞面积/总搜索面积 |

---

## 7. 附录：模块依赖图

```
server.py
├── core/task_planner.py
│   ├── core/task_schema.py
│   └── core/llm_client.py
├── core/safety_gate.py
│   └── core/validation.py
│       └── core/task_schema.py
├── core/agent_tools.py
│   ├── core/airsim_adapter.py
│   ├── core/task_executor.py ─── core/airsim_adapter.py
│   ├── core/obstacle_avoidance.py
│   │   ├── core/occupancy_grid.py
│   │   └── core/path_planner.py ─── core/task_schema.py
│   ├── core/preflight.py
│   │   ├── core/path_planner.py
│   │   └── core/task_schema.py
│   ├── core/replay.py
│   ├── core/vision_detector.py
│   └── core/formation_manager.py
├── core/auth.py
├── core/services.py
│   ├── core/path_planner.py
│   └── core/task_schema.py
├── core/websocket.py
├── core/video_stream.py ─── core/airsim_adapter.py
├── core/metrics.py
├── core/storage.py ─── core/task_schema.py
├── core/exceptions.py
└── core/async_tools.py
    ├── core/agent_tools.py
    ├── core/airsim_adapter.py
    ├── core/safety_gate.py
    └── core/task_executor.py

前端
app.js
├── js/common.js (state, constants, utilities)
├── js/map.js    (Canvas 2D rendering)
│   └── js/common.js
└── js/ui.js     (DOM rendering, API calls)
    ├── js/common.js
    └── js/map.js
```

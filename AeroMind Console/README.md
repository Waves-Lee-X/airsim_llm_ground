# AeroMind Console

自然语言驱动的 AirSim 无人机任务控制台 — 用户用中文描述任务，大模型解析意图、拆解步骤、调用工具执行，实时三维建图避障。

## 架构

```
浏览器 Web UI (static/) 
    ↕ HTTP + MJPEG
server.py ── Tornado HTTP Server
    ├── core/task_planner.py    自然语言 → 意图解析 + 参数提取
    ├── core/agent_loop.py      LLM ReAct Agent 循环 (可选)
    ├── core/safety_gate.py     安全门：参数校验、边界限制、关键操作确认
    └── core/agent_tools.py     工具运行时：起飞/搜索/扫描/编队/返航
            ↕
        core/airsim_adapter.py   AirSim RPC 封装层
            ↕
        AirSim (UE 仿真) ── LiDAR / 深度相机 / 距离传感器
            ↕
        core/temporal_grid.py    3D 时间体素网格 (实时建图)
        core/obstacle_avoidance.py  A* 避障路径规划
```

## 快速启动

```powershell
cd D:\workspace\airsim_llm_ground
.\.venv\Scripts\python.exe "AeroMind Console\server.py" --host 127.0.0.1 --port 8010
```

浏览器打开 `http://127.0.0.1:8010`

**前置条件**：UE 中运行 AirSim，RPC 端口 41451，无人机名为 `Drone1`。

## 核心模块

| 模块 | 职责 |
|------|------|
| `core/airsim_adapter.py` | AirSim RPC 封装：连接管理、遥测、起飞/降落/速度控制、LiDAR 点云、碰撞检测、距离传感器、位置保持 |
| `core/agent_tools.py` | 工具运行时 (`AgentToolRuntime`)：`takeoff`, `land`, `hover`, `goto_local`, `search_area`, `scan_object`, `return_home`, `stop`, `recover_from_collision` |
| `core/agent_loop.py` | LLM ReAct Agent 循环：`MissionMemory` 历史记录、`StateSummarizer` 状态摘要、`AgentLoop` 迭代决策 |
| `core/task_planner.py` | 自然语言解析 (中/英)：意图分类、参数提取、生成 `MissionPlan` |
| `core/safety_gate.py` | 安全校验：高度 1-30m、速度 ≤4m/s、边界 ±120m、关键操作需确认 |
| `core/temporal_grid.py` | 3D 时间体素网格：LiDAR 点累积、衰减、膨胀、提取 2D 切片供 A* 规划 |
| `core/obstacle_avoidance.py` | A* 路径规划：在体素网格切片上搜索、避开障碍物 |
| `core/occupancy_grid.py` | 2D 占据网格：LiDAR 点云构建、障碍物膨胀 |
| `core/path_planner.py` | 路径生成：蛇形扫描 (lawnmower)、Waypoint/MissionArea 数据结构 |
| `core/preflight.py` | 飞行前评估：LiDAR 区域扫描、障碍物覆盖分析、风险评估 |
| `core/depth_camera.py` | 深度图 → 3D 点云：四元数旋转、NED 坐标转换 |
| `core/task_executor.py` | 任务执行器：封装 `AirSimAdapter` 提供任务级指令 |
| `core/formation_manager.py` | 多机编队：V 字/线形编队、位置计算 |
| `core/video_stream.py` | MJPEG 视频流：从 AirSim 摄像头拉取帧 |
| `core/vision_detector.py` | YOLO 目标检测：YOLOv5/v8 封装、图像采集 |
| `core/llm_client.py` | LLM API 客户端：Claude API 封装、Mission Prompt 模板 |
| `core/validation.py` | Pydantic 参数校验：航点、区域、工具参数 |
| `core/metrics.py` | 指标采集与结构化日志 |
| `core/storage.py` | SQLite 持久化：任务记录、飞行日志 |
| `core/services.py` | 事件管理、状态管理、任务编排 |
| `core/auth.py` | API 认证：密钥验证、IP 白名单、速率限制 |
| `core/di.py` | 依赖注入容器 |
| `core/replay.py` | 轨迹回放：加载 JSON 帧、插值、重建地图 |
| `core/async_tools.py` | 异步工具包装器 |
| `core/websocket.py` | 原始 WebSocket 协议实现 |
| `core/exceptions.py` | 异常体系：`AeroMindError` 基类 |

## 前端

| 文件 | 职责 |
|------|------|
| `static/index.html` | 主页面：视频画面、任务地图、任务输入面板、状态栏 |
| `static/app.js` | 入口：WebSocket 连接、事件分发 |
| `static/js/common.js` | 全局状态、文本标签、工具函数、模板参数 |
| `static/js/map.js` | Canvas 2D 地图：无人机位置、轨迹、航点、障碍物、A* 路径 |
| `static/js/ui.js` | DOM 渲染：状态更新、日志、事件流、API 调用 |
| `static/styles.css` | 深色主题样式 |

## API 端点

### 状态与遥测
| 端点 | 说明 |
|------|------|
| `GET /api/state` | 完整状态：连接、遥测、安全、任务进度、规划器 |
| `GET /api/telemetry` | 实时遥测 (位置、速度、高度) |

### 视频
| 端点 | 说明 |
|------|------|
| `GET /video/front` | 前摄像头 MJPEG 流 |
| `GET /camera/latest` | 最新单帧 JPEG |
| `GET /api/camera/probe` | 探测可用摄像头 |

### 工具调用 (Phase A)
| 端点 | 说明 |
|------|------|
| `GET /api/tools` | 列出所有工具及安全限制 |
| `POST /api/tools/call` | 调用工具：`{"tool":"takeoff","args":{"altitude_m":8}}` |

### 任务
| 端点 | 说明 |
|------|------|
| `POST /api/task/preview` | 解析自然语言，返回计划预览 |
| `POST /api/task/run` | 解析并执行任务 |

## 常用工具调用示例

```powershell
# 起飞
Invoke-WebRequest -Method POST -ContentType "application/json" `
  -Body '{"tool":"takeoff","args":{"altitude_m":8}}' `
  http://127.0.0.1:8010/api/tools/call

# 飞到指定位置
Invoke-WebRequest -Method POST -ContentType "application/json" `
  -Body '{"tool":"goto_local","args":{"x":10,"y":0,"z":-8,"speed_mps":2}}' `
  http://127.0.0.1:8010/api/tools/call

# 区域搜索
Invoke-WebRequest -Method POST -ContentType "application/json" `
  -Body '{"tool":"search_area","args":{"target":"vehicle","area":{"x_min":0,"x_max":60,"y_min":-20,"y_max":20},"altitude_m":8,"speed_mps":2,"avoidance":true}}' `
  http://127.0.0.1:8010/api/tools/call

# 悬停 / 降落 / 返航
Invoke-WebRequest -Method POST -ContentType "application/json" `
  -Body '{"tool":"hover","args":{}}' http://127.0.0.1:8010/api/tools/call
Invoke-WebRequest -Method POST -ContentType "application/json" `
  -Body '{"tool":"land","args":{"confirmed":true}}' http://127.0.0.1:8010/api/tools/call
Invoke-WebRequest -Method POST -ContentType "application/json" `
  -Body '{"tool":"return_home","args":{"safe_altitude_m":8,"confirmed":true}}' http://127.0.0.1:8010/api/tools/call
```

## 避障系统

### 数据流
```
LiDAR (360°, 32线) + DepthCamera (前向)
    → TemporalVoxelGrid (3D 时间体素, 70×70×24m, 1m 分辨率)
    → extract_2d_slice (飞行高度 ±8m 投影)
    → A* 搜索 (计划航点间局部路径)
    → P-controller 速度控制 (闭环执行)
    → 距离传感器 (最后防线, 紧急刹停)
```

### 安全机制 (多层)
1. **A* 规划层** — LiDAR 体素网格 + A* 路径规划 (avoidance=true)
2. **距离传感器** — 前/左/右红外距离传感器 (<2.6m 紧急刹停)
3. **碰撞检测** — AirSim 碰撞事件 → 自动恢复 (爬升+后退)
4. **安全门** — 高度/速度/边界硬限制

## 测试

```powershell
cd "AeroMind Console"
python -m pytest tests/ -v
```

8 个测试文件, 130 个测试用例:
- `test_obstacle_avoidance.py` — A* 核心算法、路径规划
- `test_temporal_grid.py` — 3D 体素网格操作
- `test_lidar_filtering.py` — LiDAR 点云降噪
- `test_depth_camera.py` — 深度相机坐标转换
- `test_agent_loop.py` — LLM 决策解析、状态摘要
- `test_task_planner.py` — 中英文任务解析
- `test_core_modules.py` — 认证、安全、存储、编队等
- `test_task_preview_api.py` — API 集成测试

## LLM Agent 模式 (可选)

```powershell
$env:AEROMIND_AGENTIC_MODE = "true"
```

启用后，任务会通过 ReAct Agent 循环执行：LLM 迭代式思考 → 选择工具 → 观察结果 → 决定下一步/完成。

## 配置

- `settings.json` — 单无人机 AirSim 配置 (摄像头、LiDAR、传感器)
- `settings_multidrone.json` — 多无人机 AirSim 配置
- AirSim 连接: `127.0.0.1:41451`, 无人机名: `Drone1`

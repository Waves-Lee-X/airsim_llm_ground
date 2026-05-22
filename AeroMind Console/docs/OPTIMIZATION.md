# AeroMind Console 优化文档

## 概述

AeroMind Console 是一个面向 AirSim/UE 仿真环境的任务驱动型无人机控制台，旨在实现大模型驱动的无人机任务控制。

本文档记录了项目的优化内容，按优先级分为 P0（安全）、P1（架构）、P2（功能）、P3（质量）四个级别。

---

## 目录结构

```
AeroMind Console/
├── core/                       # 核心模块
│   ├── __init__.py
│   ├── agent_tools.py          # LLM代理工具运行时
│   ├── airsim_adapter.py       # AirSim集成适配器
│   ├── async_tools.py          # 异步工具运行时
│   ├── auth.py                 # API认证管理器
│   ├── di.py                   # 依赖注入容器
│   ├── exceptions.py           # 统一异常处理
│   ├── formation_manager.py    # 多无人机编队管理
│   ├── metrics.py              # 指标收集和日志
│   ├── obstacle_avoidance.py   # 避障算法
│   ├── occupancy_grid.py       # 占用栅格地图
│   ├── path_planner.py         # 路径规划器
│   ├── safety_gate.py          # 安全门验证
│   ├── services.py             # 服务类
│   ├── storage.py              # SQLite任务存储
│   ├── task_executor.py        # 任务执行器
│   ├── task_planner.py         # 规则引擎任务规划器
│   ├── task_schema.py          # 任务数据类
│   ├── validation.py           # Pydantic参数验证
│   ├── video_stream.py         # 视频流处理
│   └── vision_detector.py      # 视觉检测器
├── static/                     # Web UI静态资源
├── tests/                      # 单元测试
│   └── test_core_modules.py
├── docs/                       # 文档
└── server.py                   # HTTP服务器入口
```

---

## P0 级别 - 安全优先

### P0-001: API认证机制

**文件**: `core/auth.py`

**功能**:
- API密钥验证
- IP白名单控制
- 请求频率限制（可配置窗口和最大请求数）

**使用方式**:

```python
from core.auth import AuthManager, AuthConfig

config = AuthConfig(
    enabled=True,
    api_key="your_secret_key",
    rate_limit_requests=30,
    rate_limit_window_s=60,
    allowed_ips=("127.0.0.1", "::1")
)
auth = AuthManager(config)
```

**API密钥生成**:

```python
new_key = auth.generate_api_key(prefix="AEROMIND_")
```

**响应头**:
- `X-RateLimit-Remaining`: 剩余请求次数
- `Retry-After`: 频率限制重置时间

---

### P0-002: 输入验证增强

**文件**: `core/validation.py`

**功能**:
- 使用 Pydantic 对所有工具参数进行严格校验
- 支持字段范围、类型、格式验证
- 统一的验证结果返回

**验证模型**:

| 模型 | 验证参数 |
|------|---------|
| `TakeoffParams` | altitude_m: 1.0-30.0m |
| `GotoLocalParams` | x/y: -120~120m, z: -30~0m, speed: 0.5-4.0m/s |
| `WaypointRouteParams` | waypoints: 1-40个, speed: 0.5-4.0m/s |
| `FormationFlightParams` | count: 2-6, spacing: 2-20m |
| `SearchAreaParams` | 区域边界、altitude、speed等 |
| `DetectObjectsParams` | camera: front_center/bottom_center/0/front_left/front_right |

**使用方式**:

```python
from core.validation import ToolValidator

result = ToolValidator.validate("takeoff", {"altitude_m": 10.0})
if result.ok:
    validated_data = result.data
else:
    errors = result.errors  # ["altitude_m: ensure value is less than or equal to 30"]
```

---

### P0-003: 安全门增强

**文件**: `core/safety_gate.py`

**功能**:
- 实时安全状态检查
- 飞行前预检机制
- 碰撞/障碍物/距离传感器检测
- 自动阻止危险操作

**安全状态模型**:

```python
@dataclass(frozen=True)
class SafetyState:
    has_collision: bool
    collision_object: str
    obstacle_risk_level: str  # clear/low/medium/high/critical
    obstacle_blocked: bool
    distance_emergency: bool
    distance_direction: str
    current_altitude_m: float
    current_speed_mps: float
```

**预检阻止条件**:
- 碰撞状态下禁止飞行命令（除 `recover_from_collision` 和 `stop`）
- 高风险障碍物时禁止起飞和导航
- 距离传感器紧急状态时禁止飞行

**注册状态检查器**:

```python
def get_safety_state() -> SafetyState:
    return SafetyState(
        has_collision=airsim.collision_status().has_collided,
        obstacle_risk_level=airsim.obstacle_status().risk_level,
        ...
    )

safety_gate.set_state_checker(get_safety_state)
```

---

## P1 级别 - 架构稳定性

### P1-001: 架构重构 - 依赖注入

**文件**: `core/di.py`, `core/services.py`

**依赖注入容器**:

```python
from core.di import ServiceManager

sm = ServiceManager()
sm.register(dict, {"key": "value"})
sm.register_lazy(MyClass, lambda: MyClass())

obj = sm.get(dict)
```

**服务类**:

| 类名 | 职责 |
|------|------|
| `EventManager` | 事件日志管理、文件持久化 |
| `MissionState` | 任务状态、计划、运行时间 |
| `RouteManager` | 轨迹、航点、搜索区域管理 |
| `MissionContext` | 整合所有服务的上下文 |

---

### P1-002: 错误处理完善

**文件**: `core/exceptions.py`

**异常类型**:

| 异常类 | HTTP状态 | 错误码 |
|--------|---------|--------|
| `AuthError` | 401 | AM_AUTH_ERROR |
| `RateLimitError` | 429 | AM_RATE_LIMIT |
| `ValidationError` | 400 | AM_VALIDATION_ERROR |
| `SafetyError` | 403 | AM_SAFETY_ERROR |
| `AirSimError` | 503 | AM_AIRSIM_ERROR |
| `TaskError` | 500 | AM_TASK_ERROR |
| `NotFoundError` | 404 | AM_NOT_FOUND |
| `ConflictError` | 409 | AM_CONFLICT |
| `PreconditionError` | 412 | AM_PRECONDITION_ERROR |

**错误响应格式**:

```python
{
    "error": "AM_VALIDATION_ERROR",
    "message": "Validation failed: altitude_m: ensure value is less than or equal to 30",
    "details": {},
    "timestamp": "2024-01-01T12:00:00"
}
```

**统一异常处理**:

```python
from core.exceptions import ErrorHandler

status, data = ErrorHandler.handle_exception(exc)
# 返回 (http_status, error_response_dict)
```

---

### P1-003: 异步支持

**文件**: `core/async_tools.py`

**异步工具运行时**:

```python
from core.async_tools import AsyncToolRuntime

tools = AsyncToolRuntime(airsim, executor, safety_gate)

# 所有操作都是异步的
result = await tools.takeoff(altitude_m=10.0)
result = await tools.search_area(area={...}, altitude_m=8.0)
result = await tools.telemetry()
```

**异步任务执行器**:

```python
from core.async_tools import AsyncTaskRunner

runner = AsyncTaskRunner(tools)
result = await runner.run_task([
    {"tool": "takeoff", "args": {"altitude_m": 10.0}},
    {"tool": "search_area", "args": {...}},
])
```

---

## P2 级别 - 功能完善

### P2-001: 任务持久化

**文件**: `core/storage.py`

**数据库表**:

```sql
-- 任务表
CREATE TABLE tasks (
    id INTEGER PRIMARY KEY,
    task_type TEXT,
    task_text TEXT,
    status TEXT,  -- pending/running/completed/failed/cancelled
    created_at TEXT,
    started_at TEXT,
    completed_at TEXT,
    plan_json TEXT,
    result_json TEXT
);

-- 飞行日志表
CREATE TABLE flight_logs (
    id INTEGER PRIMARY KEY,
    task_id INTEGER,
    timestamp TEXT,
    x REAL, y REAL, z REAL,
    altitude_m REAL,
    speed_mps REAL,
    status TEXT
);
```

**使用方式**:

```python
from core.storage import TaskStorage

storage = TaskStorage("data/tasks.db")

# 创建任务
task_id = storage.create_task("search_area", "搜索区域", '{"plan": []}')

# 更新状态
storage.update_task_status(task_id, "running")
storage.update_task_result(task_id, '{"detections": []}')

# 查询任务
task = storage.get_task(task_id)
tasks = storage.list_tasks(status="completed", limit=50)

# 统计
stats = storage.get_stats()
# {"total": 100, "completed": 80, "failed": 10, "running": 5}
```

---

### P2-002: 视觉检测集成

**文件**: `core/vision_detector.py`

**功能**:
- YOLO目标检测（支持YOLOv5/YOLOv8）
- 批量图像检测
- 检测结果可视化
- 目标过滤

**使用方式**:

```python
from core.vision_detector import VisionDetector

detector = VisionDetector(model_path="models/yolov8n.pt", confidence=0.3)

# 单张检测
result = detector.detect(image_bytes, target_filter="person")
# result.detections: [Detection(label="person", confidence=0.95, bbox=(x1,y1,x2,y2))]

# 带可视化
result, annotated_image = detector.detect_with_visualization(image_bytes, "car")

# 批量检测
results = detector.detect_batch([img1, img2, img3])
```

**检测结果格式**:

```python
{
    "ok": True,
    "detections": [
        {"label": "person", "confidence": 0.9521, "bbox": [100, 50, 200, 150]}
    ],
    "model_path": "models/yolov8n.pt",
    "backend": "ultralytics",
    "message": "检测完成，发现 1 个目标。",
    "elapsed_ms": 45,
    "image_shape": [480, 640]
}
```

---

### P2-003: 多无人机增强

**文件**: `core/formation_manager.py`

**编队模式**:

| 模式 | 描述 | 最小/最大无人机数 |
|------|------|-----------------|
| `v` | V形编队 | 2-6 |
| `line` | 一字横队 | 2-6 |
| `delta` | 三角翼编队 | 2-6 |
| `diamond` | 菱形编队 | 2-6 |
| `triangle` | 三角形编队 | 2-6 |
| `column` | 纵队 | 2-6 |

**使用方式**:

```python
from core.formation_manager import FormationManager, SwarmCoordinator

# 创建编队
manager = FormationManager()
pattern = manager.create_pattern("v", count=4, spacing_m=5.0)

# 蜂群协调
coordinator = SwarmCoordinator(airsim_adapter)
coordinator.set_formation("delta", count=4, spacing_m=5.0)

# 编队操作
coordinator.takeoff_formation(altitude_m=10.0)
coordinator.move_formation(x=0, y=0, z=-10, speed_mps=2.0)
status = coordinator.get_formation_status()
```

---

## P3 级别 - 持续改进

### P3-001: 可观测性增强

**文件**: `core/metrics.py`

**指标收集器**:

```python
from core.metrics import MetricsCollector, StructuredLogger

metrics = MetricsCollector()

# 计数器
metrics.record_counter("api_requests")
metrics.record_counter("task_completed", value=1)

# 仪表盘
metrics.record_gauge("altitude", 10.5)
metrics.record_gauge("speed", 2.0)

# 计时器
metrics.record_timer("detection_duration", 45.2)  # ms

# 获取统计
summary = metrics.get_summary()
# {"counters": {...}, "gauges": {...}, "timers": {...}}
```

**结构化日志**:

```python
logger = StructuredLogger(log_file=Path("logs/app.jsonl"))

logger.info("Task started", task_id=123, module="executor")
logger.error("Task failed", error="timeout", task_id=123)

entries = logger.get_entries(level="ERROR")
```

**遥测监控**:

```python
from core.metrics import TelemetryMonitor

telemetry_monitor = TelemetryMonitor(metrics, logger)
telemetry_monitor.record_telemetry({"altitude_m": 10.0, "speed_mps": 2.0})
status = telemetry_monitor.get_health_status()
```

---

### P3-002: 单元测试

**文件**: `tests/test_core_modules.py`

**测试覆盖**:

| 测试类 | 测试数量 | 覆盖模块 |
|--------|---------|---------|
| `TestAuthManager` | 4 | API认证 |
| `TestToolValidator` | 5 | 参数验证 |
| `TestSafetyGate` | 5 | 安全门 |
| `TestDI` | 2 | 依赖注入 |
| `TestMetrics` | 3 | 指标收集 |
| `TestStorage` | 2 | 任务存储 |
| `TestFormationManager` | 4 | 编队管理 |
| `TestExceptions` | 3 | 异常处理 |

**运行测试**:

```bash
python -m pytest tests/ -v
```

**测试结果**:
```
29 passed, 0 failed
```

---

## 配置说明

### 认证配置

```python
AuthConfig(
    enabled=False,              # 是否启用认证
    api_key="",                 # API密钥（启用时必填）
    rate_limit_requests=30,     # 窗口内最大请求数
    rate_limit_window_s=60,     # 统计窗口（秒）
    allowed_ips=("127.0.0.1",)  # 允许的IP列表
)
```

### 安全限制

```python
SafetyLimits(
    min_altitude_m=1.0,         # 最小飞行高度
    max_altitude_m=30.0,        # 最大飞行高度
    max_speed_mps=4.0,          # 最大飞行速度
    boundary_x_min=-120.0,      # X轴最小边界
    boundary_x_max=120.0,        # X轴最大边界
    boundary_y_min=-120.0,      # Y轴最小边界
    boundary_y_max=120.0,       # Y轴最大边界
    emergency_altitude_m=3.0,    # 紧急高度阈值
)
```

---

## 依赖

```
airsim>=1.8.0
numpy>=1.20.0
PySide6>=6.4.0
PyYAML>=6.0
pydantic>=2.0
opencv-python>=4.5.0
```

---

## 许可

本项目基于 MIT 许可证开源。

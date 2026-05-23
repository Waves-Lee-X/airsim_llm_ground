# AeroMind Console 系统详细说明文档

## 1. 项目概述

`AeroMind Console` 是一个面向 AirSim / Unreal Engine 仿真环境的无人机任务控制系统。它的核心目标不是做传统地面站那种“手动按钮控制”，而是做一个“任务驱动”的无人机控制台：用户用自然语言描述任务，系统把任务解析成结构化计划，再通过安全门控、飞行控制、感知检测、路径规划和前端可视化完成闭环演示。

一句话概括：

`AeroMind Console` 让无人机从“被人逐步手动操控”升级为“根据任务目标自动规划、执行、避障、记录和汇报”。

当前系统已经具备以下能力：

- Web 控制台界面。
- AirSim / UE 实时相机画面接入。
- 自然语言任务输入与规则化任务解析。
- 任务预览、结构化计划展示和安全检查展示。
- 基础飞控：起飞、悬停、降落、停止、返航、飞到指定坐标。
- 区域搜索任务：生成蛇形覆盖航线并执行。
- LiDAR 点云感知与障碍风险评估。
- 基于占据栅格和 A* 的局部避障路径规划。
- 距离传感器安全气泡与碰撞监测。
- 碰撞 / 卡住后的恢复动作。
- 多无人机编队任务。
- YOLO 目标检测接口。
- 图像采集任务，用于生成 YOLO 数据集。
- VLA 轨迹回放能力：读取历史飞行日志并在 AirSim 中重放轨迹，同时可重建目标物体。
- 状态、地图、轨迹、事件流和任务进度可视化。

## 2. 系统运行方式

项目主入口是：

```powershell
cd D:\workspace\airsim_llm_ground
.\.venv\Scripts\python.exe "AeroMind Console\server.py" --host 127.0.0.1 --port 8010
```

浏览器打开：

```text
http://127.0.0.1:8010
```

AirSim 侧默认连接信息：

- RPC 地址：`127.0.0.1`
- RPC 端口：`41451`
- 默认无人机：`Drone1`
- 默认前视相机：`front_center`
- 其他相机：`bottom_center`、`front_left`、`front_right`、`0`
- LiDAR：`LidarSensor1`
- 距离传感器：`DistanceFront`、`DistanceLeft`、`DistanceRight`

注意：系统默认 Web 端口是 `8010`。如果文档或旧命令里出现 `8020`，应以当前服务启动端口为准。

## 3. 总体架构

系统分为五层：

```text
用户 / 浏览器界面
  -> server.py HTTP API
  -> 任务理解与安全门控
  -> Agent 工具运行时
  -> AirSim 适配层
  -> AirSim / UE 仿真环境
```

核心运行流程：

```text
用户输入自然语言任务
  -> TaskPlanner 解析意图、目标、区域、高度、速度
  -> 生成 MissionPlan
  -> 预览路线、任务步骤和安全检查
  -> SafetyGate 校验工具参数和飞行边界
  -> AgentToolRuntime 调用具体任务工具
  -> TaskExecutor 调用 AirSimAdapter
  -> AirSimAdapter 下发 AirSim RPC 控制指令
  -> AirSim 返回相机、遥测、LiDAR、碰撞和距离传感器数据
  -> server.py 聚合状态
  -> Web UI 展示视频、地图、事件流、任务进度和安全信息
```

主要模块职责：

| 模块 | 作用 |
| --- | --- |
| `server.py` | HTTP 服务入口，提供静态页面、API、全局状态聚合 |
| `static/index.html` | Web 页面结构 |
| `static/app.js` | 前端交互、轮询状态、地图绘制、按钮命令 |
| `static/styles.css` | 控制台视觉样式 |
| `core/task_planner.py` | 自然语言任务解析，生成任务计划 |
| `core/task_schema.py` | 任务、区域、步骤等数据结构 |
| `core/safety_gate.py` | 工具调用安全门控 |
| `core/validation.py` | Pydantic 参数校验 |
| `core/agent_tools.py` | Agent 工具运行时，封装任务级能力 |
| `core/task_executor.py` | 飞行动作执行器，承接任务工具和 AirSimAdapter |
| `core/airsim_adapter.py` | AirSim RPC 适配层 |
| `core/video_stream.py` | MJPEG 视频流管理 |
| `core/path_planner.py` | 蛇形覆盖航线生成 |
| `core/occupancy_grid.py` | 局部占据栅格地图 |
| `core/obstacle_avoidance.py` | A* 避障路径规划 |
| `core/vision_detector.py` | YOLO 目标检测封装 |

## 4. 前端控制台功能

### 4.1 主视频画面

界面左侧是 AirSim 相机主画面。后端通过 AirSim Image API 获取相机图像，`VideoStream` 后台线程持续抓取最新帧，再由 `/video/front` 以 MJPEG 流方式推给浏览器。

相关实现：

- 后端视频流：`core/video_stream.py`
- AirSim 图像获取：`core/airsim_adapter.py` 中的 `camera_frame()` 和 `capture_camera_frame()`
- 前端显示：`static/app.js` 中的 `renderVideo()`
- API：`GET /video/front`、`GET /camera/latest`

支持的相机切换：

- `front_center`：前视相机。
- `bottom_center`：下视相机。
- `front_left`：左前视角。
- `front_right`：右前视角。
- `0`：AirSim 默认相机编号。

前端点击相机按钮后，会调用：

```text
POST /api/camera/select
```

### 4.2 任务输入区

用户可以在文本框中输入任务，例如：

```text
搜索前方 60 米区域，发现车辆后悬停并报告坐标
```

界面提供了三个任务模板：

- 区域搜索。
- 目标扫描。
- 编队飞行。

用户也可以通过参数面板调整：

- 目标类型。
- 前方搜索距离。
- 左右搜索宽度。
- 飞行高度。
- 飞行速度。
- 航线间距。
- 是否预扫描。
- 是否启用 A* 避障。

前端会将自然语言和参数一起提交给后端：

```text
POST /api/task/preview
POST /api/task/run
```

### 4.3 任务计划展示

当用户点击“预览计划”后，系统会显示：

- 识别出的任务意图。
- 目标类型。
- 计划步骤。
- 预计航点数量。
- 预计距离。
- 安全检查结果。
- 工具调用参数。
- 地图上的预览路线。

实现逻辑：

- `server.py` 的 `preview_task()` 接收前端请求。
- `TaskPlanner.plan()` 解析自然语言。
- `_mission_draft()` 将 MissionPlan 映射为可执行工具调用。
- `SafetyGate.validate()` 做预飞检查。
- 前端 `renderToolPreview()` 和 `renderPlan()` 展示结果。

### 4.4 任务地图

地图区域使用 HTML Canvas 绘制，不依赖外部地图服务。它显示的是 AirSim 本地坐标系：

- Home 起点。
- 当前无人机位置。
- 多无人机位置。
- 已飞轨迹。
- 任务航线。
- A* 当前规划子路径。
- LiDAR 障碍点。
- 预扫描阻塞区域。

前端实现：

- `static/app.js` 中的 `drawMissionMap()`。
- `GET /api/state` 返回 `map` 字段。
- `map.route` 显示任务航线。
- `map.trail` 显示飞行轨迹。
- `map.obstacles` 显示采样后的 LiDAR 障碍点。
- `map.blocked_zones` 显示预扫描识别的阻塞网格区域。

地图支持：

- 跟随无人机。
- 缩放。
- 拖拽平移。
- 自动适配任务范围。

### 4.5 任务日志和事件流

系统有两类日志：

- 前端任务日志：展示当前任务进展，例如“正在飞向第 3 个航点”。
- 后端事件流：记录工具调用、安全门拒绝、AirSim 连接、任务状态变化等。

后端日志写入：

```text
AeroMind Console/logs/aeromind.log
```

相关 API：

```text
GET /api/logs
DELETE /api/logs
```

## 5. 自然语言任务理解

当前任务理解由 `TaskPlanner` 实现，属于规则解析器。它不是完整大模型，但已经模拟了大模型任务规划的输入输出形态。

支持的任务意图：

| 意图 | 示例 | 输出策略 |
| --- | --- | --- |
| `area_search` | 搜索车辆、寻找行人 | 生成区域蛇形搜索任务 |
| `object_or_area_scan` | 扫描建筑、观察目标 | 生成扫描/观察计划 |
| `takeoff` | 起飞到 8 米 | 生成起飞悬停计划 |
| `formation_flight` | 三架无人机 V 字编队 | 生成编队飞行计划 |
| `path_planning` | 规划一条航点路线 | 生成航点飞行计划 |
| `obstacle_aware_navigation` | 避障导航到某点 | 生成闭环导航计划 |
| `image_collection` | 采集训练图片 | 生成数据采集计划 |
| `general_uav_task` | 无法分类的任务 | 生成通用引导计划 |

解析内容包括：

- 任务意图：通过关键词打分。
- 目标类型：车辆、行人、建筑、障碍物等。
- 飞行高度：从“高度 8 米”等表达中提取。
- 搜索区域：从“前方 60 米”“左右 20 米”等表达中提取。
- 速度：从“速度 2m/s”等表达中提取。
- 编队数量：从“三架无人机”等表达中提取。
- 编队形状：V 字、直线等。

输出数据结构是 `MissionPlan`，里面包含：

- `task`：原始任务文本。
- `intent`：任务意图。
- `target`：目标类型。
- `altitude_m`：高度。
- `strategy`：策略名称。
- `area`：任务区域。
- `plan`：分步骤计划。
- `raw`：额外解析信息。

## 6. 安全门控机制

无人机控制不能直接执行任意命令，所以系统加入了 `SafetyGate`。

安全门负责：

- 拒绝未知工具。
- 校验高度范围。
- 校验速度范围。
- 校验坐标边界。
- 校验搜索区域是否越界。
- 要求降落和返航等关键动作必须确认。
- 当前处于碰撞状态时，禁止普通飞行动作。
- 根据 LiDAR 和距离传感器状态给出预警。

默认安全限制：

| 项目 | 限制 |
| --- | --- |
| 最低高度 | 1 m |
| 最高高度 | 30 m |
| 最大速度 | 4 m/s |
| X 坐标边界 | -120 m 到 120 m |
| Y 坐标边界 | -120 m 到 120 m |
| 关键工具 | `land`、`return_home` |

关键动作示例：

```json
{
  "tool": "land",
  "args": {
    "confirmed": true
  }
}
```

如果没有 `confirmed=true`，安全门会拒绝执行。

安全门调用链：

```text
POST /api/tools/call
  -> server.py call_tool()
  -> SafetyGate.validate()
  -> ToolValidator.validate()
  -> AgentToolRuntime.call()
```

## 7. Agent 工具运行时

`AgentToolRuntime` 是系统最核心的任务能力层。它把底层飞控、感知、规划封装成“工具”。未来如果接入真正大模型，大模型不需要直接调用 AirSim API，而是调用这些工具。

当前工具清单：

| 工具 | 功能 |
| --- | --- |
| `takeoff` | 起飞到指定高度 |
| `hover` | 悬停 |
| `stop` | 停止任务并保持位置 |
| `land` | 降落 |
| `return_home` | 返航到本地原点 |
| `goto_local` | 飞到本地坐标 |
| `waypoint_route` | 按多个航点飞行 |
| `autonomous_nav` | 闭环自主导航 |
| `formation_flight` | 多无人机编队飞行 |
| `recover_from_collision` | 碰撞/卡住恢复 |
| `search_area` | 区域搜索 |
| `collect_images` | 图像数据采集 |
| `detect_objects` | 目标检测 |
| `report_target` | 目标报告占位工具 |
| `replay_trajectory` | VLA 历史轨迹回放 |

这些工具通过统一接口调用：

```text
POST /api/tools/call
```

示例：

```json
{
  "tool": "takeoff",
  "args": {
    "altitude_m": 8
  }
}
```

## 8. 基础飞控能力

基础飞控由 `TaskExecutor` 和 `AirSimAdapter` 实现。

### 8.1 起飞

功能：

- 连接 AirSim。
- 启用 API 控制。
- 解锁无人机。
- 调用 `takeoffAsync()`。
- 爬升到目标高度。
- 启动位置保持线程，减少起飞后漂移。

接口：

```text
POST /api/flight/takeoff
```

工具：

```text
takeoff
```

### 8.2 悬停

功能：

- 读取当前位置。
- 调用 AirSim 悬停。
- 启动位置保持控制，让无人机尽量稳定在当前位置。

接口：

```text
POST /api/flight/hover
```

工具：

```text
hover
```

### 8.3 降落

功能：

- 停止位置保持。
- 调用 `landAsync()`。
- 记录任务状态。

接口：

```text
POST /api/flight/land
```

工具：

```text
land
```

### 8.4 停止

功能：

- 取消 AirSim 上一个任务。
- 发送零速度。
- 保持当前位置。
- 尽量让无人机快速停住。

接口：

```text
POST /api/flight/stop
```

工具：

```text
stop
```

### 8.5 返航

功能：

- 先爬升到安全高度。
- 飞回本地原点 `(0, 0)`。
- 更新前端路线显示。

接口：

```text
POST /api/flight/rtl
POST /api/task/rtl
```

工具：

```text
return_home
```

### 8.6 飞到本地坐标

`goto_local` 使用 AirSim 本地 NED 坐标：

- `x`：前方方向，单位米。
- `y`：右侧方向，单位米。
- `z`：高度方向，NED 坐标中负数代表离地高度。

示例：

```json
{
  "tool": "goto_local",
  "args": {
    "x": 10,
    "y": 0,
    "z": -8,
    "speed_mps": 2
  }
}
```

实现特点：

- 不只是简单调用 `moveToPositionAsync()`。
- 系统实现了一个速度闭环控制。
- 根据当前位置和目标点计算速度。
- 限制加速度，避免过猛运动。
- 可选择机头朝向目标方向。
- 到点后执行位置保持。

## 9. 区域搜索功能

区域搜索是当前系统最重要的完整任务闭环。

用户输入：

```text
搜索前方 60 米区域，发现车辆后悬停并报告坐标
```

系统执行流程：

```text
解析任务
  -> 确定目标 vehicle
  -> 确定区域 x=0..60, y=-20..20
  -> 生成蛇形覆盖航线
  -> 安全门检查
  -> 起飞到任务高度
  -> 预扫描障碍物
  -> 根据预扫描结果调整航线
  -> 沿航点逐段飞行
  -> 每段前进行 LiDAR A* 避障
  -> 持续监测碰撞和距离传感器
  -> 完成后悬停
```

### 9.1 蛇形航线生成

航线由 `core/path_planner.py` 的 `lawnmower_path()` 生成。

例如区域：

```text
x = 0..60
y = -20..20
spacing = 10
altitude = 8
```

会生成类似：

```text
(0, -20, -8) -> (60, -20, -8)
(60, -10, -8) -> (0, -10, -8)
(0, 0, -8) -> (60, 0, -8)
(60, 10, -8) -> (0, 10, -8)
(0, 20, -8) -> (60, 20, -8)
```

这样可以覆盖整个矩形区域，适合搜索车辆、行人或目标物体。

### 9.2 区域内缩

搜索工具支持 `scan_margin_m`。它会把实际飞行区域向内收缩，避免贴着墙、树或建筑边缘飞行。

例如用户设置：

```text
x=0..40, y=-15..15, scan_margin_m=4
```

实际飞行区域变为：

```text
x=4..36, y=-11..11
```

### 9.3 预扫描

如果 `pre_scan=true`，系统会在正式搜索前对区域中心和四个角进行 LiDAR 采样。

预扫描会输出：

- 是否有障碍点。
- 障碍点数量。
- 阻塞网格区域。
- 航线是否与阻塞区域相交。
- 障碍覆盖率。
- 风险等级：`low`、`medium`、`high`、`critical`。
- 建议：继续执行、启用避障、缩小区域或停止。

如果发现原始路线穿过障碍区域，系统会尝试切分或重规划航线。

## 10. 避障功能

系统避障分为三层。

### 10.1 LiDAR 障碍感知

AirSimAdapter 从 `LidarSensor1` 读取点云：

```text
client.getLidarData(lidar_name="LidarSensor1")
```

系统会过滤：

- 距离太近的噪声点。
- 超出最大感知范围的点。
- 高度不在飞行层附近的点。
- 孤立点。

然后把局部点云转换到世界坐标，给前端地图显示，也给 A* 使用。

### 10.2 风险评估

系统将前方空间分成多个扇区：

- hard_left
- left
- front
- right
- hard_right

计算每个扇区：

- 点数。
- 最近距离。
- 是否阻塞前方走廊。
- 推荐从左侧还是右侧绕行。
- 风险等级。

风险等级：

| 等级 | 含义 |
| --- | --- |
| `clear` | 未发现明显障碍 |
| `low` | 有障碍但距离较远 |
| `medium` | 前方有障碍点，需要谨慎 |
| `high` | 前方走廊被阻塞 |
| `critical` | 障碍非常近，存在碰撞风险 |

### 10.3 A* 局部路径规划

当 `planned_avoidance=true` 时，系统会使用 `core/obstacle_avoidance.py` 进行局部路径规划。

实现流程：

```text
当前点 + 目标航点
  -> 以两点中间为中心建立局部占据栅格
  -> 将 LiDAR 点云投影到栅格
  -> 对障碍物进行安全膨胀
  -> 可选：限制路径不能逃出搜索区域
  -> 将起点和终点映射到栅格
  -> 如果起点或终点被占据，寻找最近空闲格
  -> 使用 A* 搜索路径
  -> 抽样生成少量子航点
  -> 逐个跟踪子航点
```

占据栅格信息会返回前端：

```json
{
  "grid": {
    "resolution_m": 1.5,
    "inflation_m": 4.0,
    "raw_obstacles": 12,
    "inflated_obstacles": 94
  }
}
```

前端地图会用蓝色线段显示 A* 规划结果。

### 10.4 距离传感器安全气泡

系统同时读取：

- `DistanceFront`
- `DistanceLeft`
- `DistanceRight`

如果前方或左右距离过近，系统会触发紧急安全判断。这样即使 LiDAR 点云不足，也可以通过距离传感器防止贴墙或撞树。

### 10.5 碰撞恢复

如果 AirSim 报告碰撞：

```text
simGetCollisionInfo()
```

系统可以调用：

```text
recover_from_collision
```

恢复流程：

```text
停止当前任务
  -> 爬升一定高度
  -> 向后退一定距离
  -> 如果仍然卡住，使用 AirSim pose relocation 强制移出碰撞
  -> 悬停
```

## 11. 自主导航功能

`autonomous_nav` 是点到点闭环导航工具。

输入：

```json
{
  "tool": "autonomous_nav",
  "args": {
    "x": 30,
    "y": 10,
    "z": -8,
    "speed_mps": 1.5
  }
}
```

能力：

- 自动起飞到目标高度。
- 持续读取当前位姿。
- 周期性基于 LiDAR 重新规划 A* 路径。
- 使用速度控制追踪局部目标点。
- 检测碰撞、卡住、超时和障碍。
- 到达后悬停。

这比普通 `goto_local` 更高级，因为它在飞行过程中会不断重新感知并调整路径。

## 12. 航点路线功能

`waypoint_route` 接收多个航点，让无人机按顺序飞行。

示例：

```json
{
  "tool": "waypoint_route",
  "args": {
    "waypoints": [
      {"x": 0, "y": 0, "z": -8},
      {"x": 20, "y": 0, "z": -8},
      {"x": 20, "y": 20, "z": -8}
    ],
    "speed_mps": 2,
    "avoidance": true,
    "hold_at_end": true
  }
}
```

执行时会：

- 后台线程运行，不阻塞 Web 服务。
- 更新当前航点编号。
- 监测碰撞。
- 可在每个航段前检查障碍风险。
- 任务结束后悬停。

## 13. 多无人机编队功能

系统支持多机编队演示。

当前支持：

- V 字队形。
- 直线队形。
- 多架无人机同步起飞。
- 根据长机轨迹生成僚机目标位置。
- 在地图上展示编队目标点和多机状态。

工具：

```text
formation_flight
```

示例：

```json
{
  "tool": "formation_flight",
  "args": {
    "count": 3,
    "shape": "v",
    "distance_m": 40,
    "altitude_m": 10,
    "spacing_m": 6,
    "speed_mps": 2
  }
}
```

实现逻辑：

- 读取 AirSim 中可用的车辆名。
- 选择前 `count` 架无人机。
- 计算编队 slot。
- 每架无人机分别建立控制 client。
- 同步解锁、起飞、爬升。
- 按长机航点计算每架无人机目标点。
- 分别调用 AirSim 移动接口。

## 14. 目标检测功能

目标检测由 `core/vision_detector.py` 实现。

它支持：

- 优先加载 Ultralytics YOLO。
- 兼容部分 YOLOv5 权重。
- 自动查找本地模型：
  - `Airsim_Yolov5_ODRT-master/my_best.pt`
  - `Airsim_Yolov5_ODRT-master/yolov8n.pt`
  - `Airsim_Yolov5_ODRT-master/yolov5s.pt`
  - `models/yolov8n.pt`
  - `models/yolov5s.pt`

检测流程：

```text
AirSim 相机帧
  -> OpenCV 解码
  -> YOLO 推理
  -> 解析类别、置信度、bbox
  -> 返回 JSON
  -> 前端展示检测结果
```

返回格式：

```json
{
  "label": "car",
  "confidence": 0.82,
  "bbox": [120, 80, 260, 210]
}
```

当前目标检测能力已经有接口和模型加载封装，但是否能稳定检测取决于本地模型文件和 AirSim 场景中的目标类别。

## 15. 图像采集功能

`collect_images` 用于飞行过程中自动保存相机图片，生成 YOLO 训练数据。

输入参数：

- 采集区域。
- 采集高度。
- 航线间距。
- 相机名。
- 数据集名称。
- 最大保存数量。
- 保存间隔。
- 是否启用预扫描。
- 是否启用 A* 避障。

系统会：

- 创建数据集目录。
- 沿覆盖航线飞行。
- 每隔指定时间保存一帧图像。
- 写入 `metadata.jsonl`。
- 记录每张图像对应的无人机位置、速度、航点编号和相机信息。

默认保存目录：

```text
AeroMind Console/datasets/yolo_collect/
```

这部分功能可以支撑后续“仿真数据自动采集 -> 标注 -> YOLO 训练 -> 回到系统检测”的闭环。

## 16. VLA 轨迹回放功能

系统已经集成了 `VLA` 子项目中的核心回放能力。

工具：

```text
replay_trajectory
```

它可以读取 VLA 数据中的历史飞行日志：

```text
VLA/data/NewYorkCity/<sequence_id>/log/*.json
```

每个 JSON 文件包含一帧无人机状态，包括：

- 位置。
- 姿态四元数。
- 速度。
- IMU。
- 时间戳。

回放流程：

```text
读取 log/*.json
  -> 按文件序号排序
  -> 根据距离去除过密航点
  -> 根据转角切分轨迹段
  -> 将无人机瞬移到起点
  -> 起飞
  -> 每段先旋转机头
  -> 调用 moveOnPathAsync 平滑飞行
  -> 可选绘制红色轨迹线
  -> 可选根据 mark.json 重建目标物体
  -> 结束后降落
```

示例：

```json
{
  "tool": "replay_trajectory",
  "args": {
    "log_folder": "D:/workspace/airsim_llm_ground/VLA/data/NewYorkCity/a42b0e07-34d0-40e0-87d5-506a67066f9a/log",
    "speed_mps": 3.5,
    "turn_threshold_deg": 30,
    "min_dist_m": 0.5,
    "spawn_target": true,
    "map_spawn_json": "D:/workspace/airsim_llm_ground/VLA/data/meta/map_spawnarea_info.json",
    "target_object_name": "ReplayTarget",
    "draw_trail": true,
    "trail_thickness": 8
  }
}
```

目标重建逻辑：

- 从 `mark.json` 读取目标真实位置。
- 根据地图名查找 `map_spawnarea_info.json`。
- 找到最接近目标位置的 spawn 区域。
- 读取资产名、位置、旋转和缩放。
- 调用 AirSim `simSpawnObject()` 在 UE 中生成目标。

这使系统不仅可以执行实时任务，也可以复现实验数据中的历史轨迹，适合做“模型回放、数据复盘、老师演示”。

## 17. API 说明

### 17.1 状态类 API

| API | 方法 | 功能 |
| --- | --- | --- |
| `/api/state` | GET | 获取完整状态，包含任务、地图、安全、视频、视觉结果 |
| `/api/telemetry` | GET | 获取轻量遥测，用于高频刷新地图和位置 |
| `/api/logs` | GET | 获取事件日志 |
| `/api/tools` | GET | 获取工具列表和安全限制 |

### 17.2 相机与 AirSim API

| API | 方法 | 功能 |
| --- | --- | --- |
| `/api/airsim/reconnect` | POST | 重新连接 AirSim |
| `/api/camera/probe` | GET/POST | 探测可用相机 |
| `/api/camera/options` | GET | 获取相机选项 |
| `/api/camera/select` | POST | 切换相机 |
| `/camera/latest` | GET | 获取单帧最新图像 |
| `/video/front` | GET | 获取 MJPEG 视频流 |

### 17.3 任务 API

| API | 方法 | 功能 |
| --- | --- | --- |
| `/api/task/preview` | POST | 预览任务计划 |
| `/api/task/run` | POST | 执行自然语言任务 |
| `/api/task/pause` | POST | 暂停任务 |
| `/api/task/stop` | POST | 停止任务 |
| `/api/task/rtl` | POST | 返航 |

### 17.4 飞控 API

| API | 方法 | 功能 |
| --- | --- | --- |
| `/api/flight/takeoff` | POST | 起飞 |
| `/api/flight/hover` | POST | 悬停 |
| `/api/flight/land` | POST | 降落 |
| `/api/flight/stop` | POST | 停止 |
| `/api/flight/rtl` | POST | 返航 |

### 17.5 Agent 工具 API

统一入口：

```text
POST /api/tools/call
```

请求体：

```json
{
  "tool": "search_area",
  "args": {
    "target": "vehicle",
    "area": {
      "x_min": 0,
      "x_max": 40,
      "y_min": -15,
      "y_max": 15
    },
    "altitude_m": 10
  }
}
```

返回内容：

- `accepted`：安全门是否接受。
- `safety`：安全检查详情。
- `result`：工具执行结果。
- `state`：执行后的系统状态。

## 18. 数据结构说明

### 18.1 MissionPlan

任务计划结构：

```json
{
  "task": "搜索前方60米区域",
  "intent": "area_search",
  "target": "vehicle",
  "altitude_m": 8,
  "strategy": "lawnmower",
  "area": {
    "x_min": 0,
    "x_max": 60,
    "y_min": -20,
    "y_max": 20
  },
  "plan": [
    {
      "name": "解析搜索任务",
      "detail": "提取目标类型、搜索区域和高度约束",
      "action": "parse"
    }
  ]
}
```

### 18.2 `/api/state` 关键字段

```json
{
  "connected": true,
  "video": {},
  "vehicle": {},
  "uav": {},
  "uavs": [],
  "task": {
    "status": "running",
    "title": "Agent search area: vehicle",
    "plan": [],
    "agent_progress": {}
  },
  "events": [],
  "map": {
    "uav": {},
    "trail": [],
    "route": [],
    "obstacles": [],
    "blocked_zones": []
  },
  "safety": {
    "collision": {},
    "obstacle": {},
    "distance_sensors": {},
    "lidar": {}
  },
  "vision": {}
}
```

## 19. 演示流程建议

### 19.1 基础演示

1. 启动 AirSim / UE。
2. 启动 AeroMind Console。
3. 浏览器打开控制台。
4. 点击连接状态，确认 AirSim 在线。
5. 查看前视相机画面。
6. 点击“起飞”。
7. 点击“悬停”。
8. 点击“返航”或“降落”。

### 19.2 自然语言区域搜索演示

输入：

```text
搜索前方 60 米区域，发现车辆后悬停并报告坐标
```

操作：

1. 点击“预览计划”。
2. 讲解系统识别出 `area_search`。
3. 展示地图上的蛇形航线。
4. 展示安全检查和工具参数。
5. 点击“执行任务”。
6. 观察无人机起飞、预扫描、避障、执行航线。
7. 观察事件流和任务日志。

### 19.3 避障演示

推荐讲解点：

- 红点是 LiDAR 采样点。
- 红色区域是预扫描得到的阻塞网格。
- 蓝色线是 A* 规划出的局部路径。
- 风险等级来自 LiDAR 扇区分析和距离传感器。
- 如果前方危险，系统会停止、绕行或恢复。

### 19.4 VLA 轨迹回放演示

调用：

```json
{
  "tool": "replay_trajectory",
  "args": {
    "log_folder": "D:/workspace/airsim_llm_ground/VLA/data/NewYorkCity/a42b0e07-34d0-40e0-87d5-506a67066f9a/log",
    "spawn_target": true,
    "draw_trail": true
  }
}
```

讲解点：

- 系统从历史日志中读取每一帧轨迹。
- 自动去除过密航点。
- 根据转弯角度切分飞行段。
- 在 UE 中重建目标物体。
- 按历史轨迹回放无人机飞行。

## 20. 当前优势

本系统的主要亮点：

- 任务驱动，而不是单纯遥控。
- 支持自然语言任务到结构化计划。
- 有完整前端可视化：视频、地图、轨迹、日志、状态。
- 工具调用经过安全门控，适合未来接大模型。
- 搜索任务不是静态航线，而是加入了 LiDAR、距离传感器、碰撞状态和 A* 规划。
- 支持数据采集和目标检测，为视觉闭环打基础。
- 集成了 VLA 轨迹回放，能够复现实验数据。
- 后端模块边界清楚，便于继续扩展。

## 21. 当前不足

需要客观说明的不足：

- 自然语言理解目前是规则解析，还没有真正接入大模型。
- YOLO 检测依赖本地模型文件和场景目标类别，稳定性取决于权重质量。
- 语音入口目前主要是前端预留，尚未完整接入 ASR。
- 编队功能已有执行框架，但还不是复杂队形控制算法。
- 系统目前主要面向本机演示，认证默认关闭，不适合直接暴露到公网。
- 部分中文历史文件存在编码问题，后续需要统一 UTF-8。
- AirSim 场景和传感器配置必须正确，否则视频、LiDAR、距离传感器会不可用。

## 22. 后续改进方向

推荐后续工作：

1. 接入真实大模型，让 `TaskPlanner` 从规则解析升级为 LLM 结构化规划。
2. 完善语音输入，接入浏览器 Web Speech API 或后端 ASR。
3. 强化 YOLO 检测闭环：检测到目标后自动悬停、截图、记录坐标。
4. 增加任务报告导出功能：航迹、截图、目标、耗时、风险记录。
5. 增加更复杂的路径规划算法，例如 RRT、DWA 或全局地图规划。
6. 完善多无人机任务：分区搜索、队形保持、任务协同。
7. 修复文档和界面中的历史编码乱码。
8. 增加用户认证和部署配置，提升工程化程度。

## 23. 汇报时可以使用的总结语

本项目实现了一个面向 AirSim 仿真的任务驱动无人机控制台。它将自然语言任务、飞行控制、视觉感知、LiDAR 避障、路径规划和 Web 可视化整合在一起。系统通过工具化架构为未来接入大模型预留了接口，大模型可以通过安全门控后的工具调用控制无人机，而不是直接操作底层 AirSim API。当前系统已经能够完成起飞、悬停、返航、区域搜索、A* 避障、图像采集、目标检测接口、多机编队和 VLA 轨迹回放等功能，具备较完整的演示闭环和继续扩展的基础。


# AeroMind Console 项目详细梳理（代码实现版）

更新时间：2026-05-12  
适用目录：`AeroMind Console/`

## 1. 项目目标

AeroMind Console 的目标是把“自然语言任务”变成“可执行、可观测、可回溯”的无人机任务流程：

1. 用户输入自然语言任务（如区域搜索）。
2. 后端规划任务意图并生成工具调用参数。
3. 安全门校验参数与当前安全状态。
4. 任务线程执行（起飞、预扫描、路径执行、避障、恢复）。
5. 前端实时展示阶段、航点、风险、日志与地图状态。

---

## 2. 目录与职责

## 2.1 服务入口

- `server.py`
  - HTTP API 路由与静态资源服务。
  - 维护全局 `ConsoleState`（状态聚合、日志、任务显示）。
  - 负责把 `agent_progress` 同步到事件流（MISSION 日志）。

## 2.2 核心模块（`core/`）

- `task_planner.py`
  - 自然语言规则解析（意图/目标/高度/速度/编队参数等）。
  - 输出 `MissionPlan`。
- `task_schema.py`
  - 统一任务数据结构（`MissionPlan`, `PlanStep`, `MissionArea`）。
- `task_executor.py`
  - 任务执行器外观层（调用 AirSimAdapter）。
- `agent_tools.py`
  - 工具运行时（takeoff/search_area/waypoint_route/...）。
  - 维护任务线程与 `mission_progress`。
  - 执行预扫描、A* 规划、避障、恢复逻辑。
- `airsim_adapter.py`
  - AirSim RPC 连接与飞控适配。
  - 多机支持、相机读取、传感器读取、持位控制、起飞降落等。
- `safety_gate.py`
  - 参数校验、安全预检、危险状态阻断。
- `path_planner.py`
  - 区域覆盖航线（lawnmower）等路径生成。
- `obstacle_avoidance.py` / `occupancy_grid.py`
  - 局部栅格化与 A* 规划支撑。
- `video_stream.py`
  - 视频流输出（`/video/front`）。
- `vision_detector.py`
  - 检测器状态与检测结果适配。

## 2.3 前端（`static/`）

- `index.html`：信息分区布局（视频、任务、计划、安全、日志、地图）。
- `styles.css`：控制台视觉样式与响应式布局。
- `app.js`：拉取状态、渲染计划进度、感知信息、地图和事件流。

---

## 3. 端到端运行链路

## 3.1 自然语言任务运行

1. `POST /api/task/run`
2. `ConsoleState.run_task()`
3. `TaskPlanner.plan(text)` 生成 `MissionPlan`
4. `ConsoleState._mission_tool_call()` 映射成具体工具（如 `search_area`）
5. `AgentToolRuntime.call(tool, args)` 启动后台任务线程
6. 线程通过 `TaskExecutor -> AirSimAdapter` 控制无人机
7. `mission_progress` 持续更新，`/api/state` 返回给前端渲染

## 3.2 状态刷新

- 前端周期请求 `GET /api/state`
- 返回内容包含：
  - `task.status/title/plan/agent_progress`
  - `uav` 与 `uavs` 遥测
  - `safety`（collision/obstacle/distance sensors）
  - `map`（route/trail/obstacles/blocked_zones）
  - `events`（包括 MISSION 心跳日志）

---

## 4. API 总览（当前实现）

## 4.1 GET

- `/api/state`
- `/api/telemetry`
- `/api/camera/probe`
- `/api/camera/options`
- `/api/vehicle/options`
- `/api/logs`
- `/api/tools`
- `/api/detect/latest`
- `/video/front`

## 4.2 POST

- `/api/airsim/reconnect`
- `/api/camera/probe`
- `/api/camera/select`
- `/api/vehicle/select`
- `/api/tools/call`
- `/api/detect/latest`
- `/api/flight/takeoff`
- `/api/flight/hover`
- `/api/flight/land`
- `/api/flight/stop`
- `/api/flight/rtl`
- `/api/task/preview`
- `/api/task/run`
- `/api/task/pause`
- `/api/task/stop`
- `/api/task/rtl`

---

## 5. 控制逻辑（重点）

## 5.1 起飞逻辑（`AirSimAdapter.takeoff`）

- `takeoffAsync` -> `moveToZAsync(target_z)`。
- 会做目标高度收敛确认，避免“异步提前返回导致未到高度就切状态”。
- 最终进入持位。

## 5.2 持位/悬停逻辑（已按 ground_station_qt_airsim 思路对齐）

- 采用 `moveByVelocityZAsync(vx, vy, z, duration)`。
- `vx/vy` 来自位置误差回拉（增益 0.35，限幅 ±0.45）。
- 固定刷新周期（约 0.35s）维持位姿与高度。

## 5.3 区域扫描（`search_area`）

主流程：

1. `_ensure_airborne(altitude_m)`（先到任务高度）
2. `pre_scan`（面向中心+四角采样 LiDAR）
3. 生成 `area_assessment`（风险、覆盖率、路线相交）
4. 必要时重规划 `replanned_route`
5. 航点循环执行：
   - `planned_avoidance=true`：先本地 A* 子航点
   - 再执行航点等待 `_wait_for_waypoint`
6. 过程中持续碰撞/距离传感器/障碍检查

---

## 6. 观测与日志机制

## 6.1 `mission_progress` 字段（核心诊断）

常用字段：

- `status`: `taking_off/pre_scanning/running/avoiding/blocked/failed/...`
- `message`
- `current_waypoint_index` / `total_waypoints`
- `distance_to_waypoint_m`
- `planner`
- `obstacle`
- `distance_sensors`
- `collision`
- `area_assessment`
- `telemetry`（每次 progress 附带同帧数据）

## 6.2 事件日志（`events`）

- `ConsoleState._sync_agent_progress_events()` 会将 progress 写入 MISSION 日志。
- 已加入心跳日志（活动阶段约每 3 秒一次），包含：
  - `alt/speed/d2wp/risk/obs/front/planner`

这让“卡在哪、为什么卡”可见，而不是只有“任务已派发”。

---

## 7. 前端可视化结构

- 顶部：连接、视觉、任务状态 pill。
- 中区：
  - 左：视频画面 + 覆层遥测。
  - 右：任务输入、参数、计划、执行按钮、任务日志。
- 底区：
  - 任务状态卡、位置卡、进度卡、感知卡、地图卡、事件流卡。
- 计划区域新增：
  - 阶段/航点/距离/风险概览
  - 问题定位横幅（Issue Banner）

---

## 8. 常见问题定位路径

## 8.1 起飞卡住或高度异常

看 `MISSION` 日志是否持续有：

- `status=taking_off`
- `alt` 是否增长
- `message` 中 `ALT x/target`

若 `alt` 不增长，优先排查：

1. AirSim 连接/车辆控制权
2. 是否被其它控制命令打断
3. 仿真场景碰撞或地面约束

## 8.2 预扫描卡住（pre_scanning 不前进）

看日志是否有：

- `Pre-scan: sampling i/5 ...`
- `sample i/5 done, lidar ...`

如果只出现开始不出现完成，优先看：

1. `face_local` 是否异常
2. LiDAR 返回是否超时/空
3. 线程是否被 stop/collision 分支终止

## 8.3 看起来“很蠢、卡一卡”

先看同一时间窗口的三组信息：

1. `MISSION` 心跳（数值趋势）
2. `planner` 字段（ok/blocked/phase/reason）
3. `distance_sensors` + `obstacle`（是否触发安全气泡或风险阻断）

---

## 9. 当前设计边界

1. `task_planner.py` 仍是规则解析，不是真正大模型推理代理。
2. 避障策略是“安全优先”，在弱感知场景会偏保守。
3. 复杂环境稳定性高度依赖 AirSim 传感器质量与场景设置。

---

## 10. 推荐后续文档拆分

建议把本文拆成 4 份长期维护文档：

1. `docs/CONTROL_LOGIC.md`：纯飞控与执行时序。
2. `docs/OBSERVABILITY.md`：日志、状态字段、排障流程。
3. `docs/API_REFERENCE.md`：接口清单与示例。
4. `docs/FRONTEND_LAYOUT.md`：界面信息架构与渲染逻辑。

这样后续改动可以按模块更新，不会一份文档越来越大。


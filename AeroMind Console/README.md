# AeroMind Console

AeroMind Console is a mission-driven UAV console for AirSim / UE demos. It is designed for large-model-driven UAV tasks, where the user describes a mission with natural language or voice instead of manually operating a button-heavy ground station.

## Current Focus

- Main video stage for AirSim / UE / UAV camera feed.
- Natural-language mission input.
- Mission templates for area search, scanning, and formation flight.
- Structured mission plan display.
- Event stream and mission status.
- Mission map with UAV position, route, search area, and trail.
- Safety actions: pause, return-to-launch, stop.
- Basic AirSim flight controls: takeoff, hover, land, stop, return-to-launch.
- Phase A agent tool runtime: safety-gated JSON tools for future LLM control.

## Start

```powershell
cd D:\workspace\airsim_llm_ground
.\.venv\Scripts\python.exe "AeroMind Console\server.py" --host 127.0.0.1 --port 8010
```

Open:

```text
http://127.0.0.1:8010
```

## Important Files

- `server.py`: lightweight HTTP server and API entry.
- `static/`: web UI.
- `core/task_schema.py`: mission dataclasses.
- `core/task_planner.py`: rule-based planner before the real LLM is connected.
- `core/path_planner.py`: path generation helpers.
- `core/airsim_adapter.py`: AirSim integration placeholder.
- `core/video_stream.py`: camera stream placeholder.
- `core/vision_detector.py`: target detector placeholder.
- `core/task_executor.py`: task execution placeholder.
- `IMPROVEMENT_PLAN.md`: full development plan.
- `docs/`: architecture, API, schema, and demo notes.

## Next Step

The AirSim front camera endpoint is now reserved at:

```text
http://127.0.0.1:8010/video/front
```

For a single latest frame, open:

```text
http://127.0.0.1:8010/camera/latest
```

It expects:

- AirSim RPC: `127.0.0.1:41451`
- Vehicle name: `Drone1`
- Default camera name: `front_center`
- Scan camera: `bottom_center`
- Side cameras: `front_left`, `front_right`

`settings.json` keeps AirSim `SubWindows` disabled by default, so UE will not show extra picture-in-picture camera boxes. The web console uses `/video/front` as the single main video surface.

Click the simulation status pill in the UI to retry AirSim connection.

For lower video latency, use the provided `settings.json` camera resolution (`640x360`) first. Higher PNG frame sizes can make AirSim's image API feel delayed in a browser MJPEG stream.

If AirSim is connected but the camera does not appear, open:

```text
http://127.0.0.1:8010/api/camera/probe
```

The probe tries these camera names:

- `front_center`
- `bottom_center`
- `0`
- `front_left`
- `front_right`

The provided AirSim `settings.json` also enables IMU, GPS, barometer, magnetometer, front/left/right distance sensors, and a 360-degree `LidarSensor1`. The obstacle planner primarily uses `LidarSensor1`; distance sensors provide a close-range safety bubble.

## Phase A: Agent Tool Runtime

The first LLM-facing control entrance is:

```text
POST /api/tools/call
```

List available tools and safety limits:

```text
http://127.0.0.1:8020/api/tools
```

PowerShell examples:

```powershell
Invoke-WebRequest -UseBasicParsing -Method POST -ContentType "application/json" -Body '{"tool":"takeoff","args":{"altitude_m":8}}' http://127.0.0.1:8020/api/tools/call
Invoke-WebRequest -UseBasicParsing -Method POST -ContentType "application/json" -Body '{"tool":"goto_local","args":{"x":10,"y":0,"z":-8,"speed_mps":2}}' http://127.0.0.1:8020/api/tools/call
Invoke-WebRequest -UseBasicParsing -Method POST -ContentType "application/json" -Body '{"tool":"hover","args":{}}' http://127.0.0.1:8020/api/tools/call
Invoke-WebRequest -UseBasicParsing -Method POST -ContentType "application/json" -Body '{"tool":"stop","args":{}}' http://127.0.0.1:8020/api/tools/call
Invoke-WebRequest -UseBasicParsing -Method POST -ContentType "application/json" -Body '{"tool":"recover_from_collision","args":{"climb_m":8,"backoff_m":10,"force_relocate":true}}' http://127.0.0.1:8020/api/tools/call
Invoke-WebRequest -UseBasicParsing -Method POST -ContentType "application/json" -Body '{"tool":"search_area","args":{"target":"vehicle","area":{"x_min":0,"x_max":40,"y_min":-15,"y_max":15},"altitude_m":15,"spacing_m":10,"speed_mps":1.5,"avoidance":true,"planned_avoidance":true,"obstacle_distance_m":12,"avoidance_offset_m":8,"scan_margin_m":4,"pre_scan":true,"pre_scan_stop_on_high_risk":true}}' http://127.0.0.1:8020/api/tools/call
```

Critical tools require confirmation:

```powershell
Invoke-WebRequest -UseBasicParsing -Method POST -ContentType "application/json" -Body '{"tool":"return_home","args":{"safe_altitude_m":8,"confirmed":true}}' http://127.0.0.1:8020/api/tools/call
Invoke-WebRequest -UseBasicParsing -Method POST -ContentType "application/json" -Body '{"tool":"land","args":{"confirmed":true}}' http://127.0.0.1:8020/api/tools/call
```

The safety gate currently rejects:

- Unknown tools.
- Altitudes below `1m` or above `30m`.
- `goto_local` points outside `x/y = -120m..120m`.
- `search_area` rectangles outside `x/y = -120m..120m`.
- Speeds above `4m/s`.
- Critical actions without `confirmed=true`.

During `search_area`, AirSim collision status is monitored. If `simGetCollisionInfo` reports a collision, the active search mission stops and the backend sends a hover command. The current collision status is available in `/api/state` under `safety.collision`.

Local avoidance is also enabled for `search_area` by default. With `planned_avoidance=true`, the backend builds an A* path from `LidarSensor1` and follows those sub-waypoints in a smooth scan mode that keeps yaw stable instead of rotating toward every small waypoint. With `planned_avoidance=false`, the older temporary left/right bypass behavior is still available.

`scan_margin_m` insets the effective scan route from the requested rectangle boundaries. This keeps the UAV away from walls or buildings at the edge of the area. For example, an area of `x=0..40, y=-15..15` with `scan_margin_m=4` flies the actual coverage path inside `x=4..36, y=-11..11`.

Before flying the route, `pre_scan=true` performs a LiDAR area assessment toward the effective area center and corners. It estimates blocked zones, route intersections, obstacle coverage, and a risk level. The map draws pre-scan blocked zones in red, and the perception panel shows the pre-scan risk. If the planned scan path intersects suspected obstacles, the worker first splits/replans the scan route around those zones. With `pre_scan_stop_on_high_risk=true`, the mission only stops if no safe route remains.

The red blocked zones are safety-expanded cells, not exact mesh outlines. They intentionally sit wider than the visible obstacle because the planner uses 2m grid cells, obstacle inflation, and route padding to keep the UAV away from walls and trees.

Front/left/right distance sensors are used as a safety bubble. If the front or side distance sensors report a very close obstacle, the mission hard-stops instead of trying to squeeze through or scrape along a wall. The latest obstacle and distance status is available in `/api/state` under `safety.obstacle` and `safety.distance_sensors`.

If the UAV is already stuck in a tree, wall, or mesh, call `recover_from_collision`. It cancels the active task, climbs, backs away, and hovers. With `force_relocate=true`, it uses AirSim pose relocation if normal motion does not move the UAV out of collision. `search_area` also calls this automatically on collision, timeout, critical obstacle risk, or stuck detection.

The LiDAR obstacle analysis is point-cloud based. `safety.obstacle` includes:

- `risk_level`: `clear`, `low`, `medium`, `high`, or `critical`.
- `sector_clearance`: nearest clearance in hard-left, left, front, right, and hard-right sectors.
- `sector_points`: point counts per sector.
- `clusters`: nearest obstacle clusters with centroid and point count.
- `decision`: left/right clearance scores and the reason for the bypass choice.

For better tree and wall avoidance, `search_area` supports `planned_avoidance=true`. It builds a local occupancy grid from LiDAR points, inflates obstacles by a safety margin, runs A* to the next search waypoint, and flies the generated sub-waypoints. Progress is visible in `/api/state` under `task.agent_progress.planner`.

`search_area` now performs its own takeoff/climb check. If the UAV is on the ground or below the requested altitude, it first climbs to `altitude_m`, then starts the search route. The current phase is visible in `/api/state` under `task.agent_progress.message`.

The A* planner is bounded to the effective scan area, so it should not escape outside the requested rectangle just to avoid an obstacle. If there are not enough LiDAR obstacle points but front/left/right distance sensors are safe, `search_area` uses a direct safe segment fallback. If both LiDAR and distance-sensor safety are unavailable or unsafe, it stops instead of flying blind.

The web UI visualizes avoidance data directly:

- Red dots on the mission map are sampled LiDAR obstacle points.
- Blue path segments are the current A* sub-waypoints.
- The perception card shows risk level, LiDAR point count, nearest obstacle distance, inflated grid cells, recommended side, and collision object.

# API Notes

## `GET /api/state`

Returns the current console state, including connection status, video status, UAV telemetry, task status, plan, events, and uptime.

The response also includes a `map` field:

```json
{
  "map": {
    "home": {"x": 0, "y": 0},
    "uav": {"x": 12.5, "y": -4.2, "z": -8.0},
    "trail": [],
    "route": [],
    "search_area": null,
    "targets": [],
    "obstacles": [],
    "blocked_zones": []
  }
}
```

## `POST /api/task/preview`

Request:

```json
{
  "text": "Search the front 60 meters and hover after finding a vehicle"
}
```

Response:

```json
{
  "task": "...",
  "intent": "area_search",
  "target": "vehicle",
  "altitude_m": 8,
  "strategy": "lawnmower",
  "plan": []
}
```

## `POST /api/task/run`

Creates a mission plan and moves the console into the `planning` state. AirSim execution will be connected later.

## Safety APIs

- `POST /api/task/pause`
- `POST /api/task/stop`
- `POST /api/task/rtl`

## Flight APIs

- `POST /api/flight/takeoff`
- `POST /api/flight/hover`
- `POST /api/flight/land`
- `POST /api/flight/stop`
- `POST /api/flight/rtl`

## Agent Tool APIs

These are the first APIs intended for the future LLM agent. The LLM should call these tools instead of directly calling low-level AirSim control methods.

- `GET /api/tools`: lists available tools and safety limits.
- `POST /api/tools/call`: validates a tool call through the safety gate, executes it if accepted, logs the result, and returns the updated console state.

Request:

```json
{
  "tool": "goto_local",
  "args": {
    "x": 10,
    "y": 0,
    "z": -8,
    "speed_mps": 2,
    "avoidance": true,
    "obstacle_distance_m": 8,
    "avoidance_offset_m": 6
  }
}
```

Response:

```json
{
  "accepted": true,
  "safety": {
    "accepted": true,
    "tool": "goto_local",
    "args": {"x": 10.0, "y": 0.0, "z": -8.0, "speed_mps": 2.0},
    "warnings": []
  },
  "result": {
    "ok": true,
    "tool": "goto_local",
    "message": "Goto local command sent: x=10.0, y=0.0, z=-8.0",
    "data": {"x": 10.0, "y": 0.0, "z": -8.0, "speed_mps": 2.0},
    "elapsed_ms": 12
  },
  "state": {}
}
```

Critical tools require `confirmed=true` in `args`:

```json
{
  "tool": "land",
  "args": {
    "confirmed": true
  }
}
```

Emergency recovery when stuck in a mesh or tree:

```json
{
  "tool": "recover_from_collision",
  "args": {
    "climb_m": 8,
    "backoff_m": 10,
    "force_relocate": true
  }
}
```

Replay a recorded trajectory log and optionally spawn a target from `mark.json`:

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

Start a rectangular search mission:

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
    "altitude_m": 15,
    "spacing_m": 10,
    "speed_mps": 1.5,
    "avoidance": true,
    "planned_avoidance": true,
    "obstacle_distance_m": 12,
    "avoidance_offset_m": 8,
    "scan_margin_m": 4,
    "pre_scan": true,
    "pre_scan_stop_on_high_risk": true
  }
}
```

`scan_margin_m` insets the actual scan route from the requested rectangle boundaries. With the example above, the effective flight path is inside `x=4..36, y=-11..11`.

When `pre_scan=true`, the worker first samples LiDAR toward the effective area center and corners, builds blocked zones, and writes the result to `task.agent_progress.area_assessment` and `map.blocked_zones`. If the original route crosses blocked cells, the worker first tries to split/replan the scan route around those zones. If no safe route remains and `pre_scan_stop_on_high_risk=true`, the mission stops before route execution.

`search_area` runs in a background worker. Progress is available in:

```json
{
  "task": {
    "agent_progress": {
      "status": "running",
      "tool": "search_area",
      "target": "vehicle",
      "current_waypoint_index": 2,
      "total_waypoints": 8,
      "distance_to_waypoint_m": 4.2,
      "message": "Flying to waypoint 2/8."
    }
  }
}
```

If the UAV is on the ground or below the requested search altitude, `search_area` first enters `taking_off` and climbs to `altitude_m`. Then it starts pre-scan and route execution.

Before each planned segment, the worker collects LiDAR points for A*. If too few obstacle points are available but front/left/right distance sensors are safe, it uses a `direct_safe_segment` fallback instead of stopping. If both LiDAR and distance safety are unavailable or unsafe, the worker reports `blocked`.

The same data is visible in the UI. The mission map draws sampled LiDAR points and the active A* path, while the perception card summarizes risk, planner state, point count, and collision object.

The backend monitors AirSim collision status while `search_area` is running. If a collision is detected, the search worker stops and reports:

```json
{
  "task": {
    "agent_progress": {
      "status": "collision",
      "message": "Collision detected with 'object'; search stopped.",
      "collision": {
        "has_collided": true,
        "object_name": "object"
      }
    }
  }
}
```

When `avoidance=true`, the search worker reads `LidarSensor1`. If the front corridor is blocked, it temporarily switches to:

```json
{
  "task": {
    "agent_progress": {
      "status": "avoiding",
      "message": "Obstacle ahead; bypassing left via temporary waypoint 1.",
      "obstacle": {
        "available": true,
        "blocked": true,
        "nearest_distance_m": 4.6,
        "recommended_side": "left",
        "risk_level": "high",
        "sector_clearance": {
          "left": 7.2,
          "front": 4.6,
          "right": 5.1
        },
        "clusters": [
          {
            "points": 18,
            "centroid": {"x": 4.8, "y": 0.7, "z": -0.3},
            "min_distance_m": 4.6
          }
        ],
        "decision": {
          "left_score": 16.2,
          "right_score": 12.4,
          "reason": "front corridor occupied"
        }
      }
    }
  }
}
```

When `planned_avoidance=true`, the worker builds a local occupancy grid from LiDAR points, inflates obstacles, runs A*, and flies generated sub-waypoints before each search waypoint:

```json
{
  "task": {
    "agent_progress": {
      "planner": {
        "ok": true,
        "reason": "A* path generated with 8 waypoints",
        "points": 142,
        "grid": {
          "resolution_m": 1.5,
          "inflation_m": 4.0,
          "raw_obstacles": 12,
          "inflated_obstacles": 94
        }
      }
    }
  }
}
```

## AirSim APIs

- `POST /api/airsim/reconnect`: tries to connect to AirSim RPC on `127.0.0.1:41451`.
- `GET /video/front`: MJPEG stream from the currently selected `Drone1` camera.
- `GET /api/camera/options`: returns the selected camera and supported camera candidates.
- `POST /api/camera/select`: switches the current camera. Body example: `{"camera":"bottom_center"}`.
- `GET /api/camera/probe` or `POST /api/camera/probe`: probes camera candidates and returns which camera can provide image bytes.

## AirSim Sensor Setup

The provided `settings.json` defines:

- `front_center`, `bottom_center`, `front_left`, `front_right`
- front/left/right distance sensors for safety-bubble stopping
- `LidarSensor1` as a 360-degree LiDAR with `HorizontalFOVStart=-180`, `HorizontalFOVEnd=180`, and `PointsPerSecond=80000`

After editing `settings.json`, copy it to AirSim's active settings location and restart AirSim/UE before testing.

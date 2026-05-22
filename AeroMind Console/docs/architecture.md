# AeroMind Console Architecture

## Purpose

AeroMind Console is designed as a mission-driven UAV control console. The user describes a mission in natural language or voice, the system converts it into a structured plan, and backend modules execute that plan through AirSim and perception components.

## Runtime Flow

```text
User prompt
  -> TaskPlanner
  -> MissionPlan
  -> TaskExecutor
  -> AirSimAdapter
  -> UAV / sensors
  -> VisionDetector
  -> events + report
  -> Web UI
```

## Main Modules

- `server.py`: HTTP API and static file server.
- `core/task_schema.py`: Shared dataclasses for mission plans.
- `core/task_planner.py`: Rule-based planner before the real LLM is connected.
- `core/path_planner.py`: Route generation utilities.
- `core/airsim_adapter.py`: AirSim integration placeholder.
- `core/video_stream.py`: AirSim camera streaming placeholder.
- `core/vision_detector.py`: YOLO or multimodal detector placeholder.
- `core/task_executor.py`: Mission execution placeholder.

## API Surface

- `GET /api/state`
- `GET /video/front`
- `POST /api/task/preview`
- `POST /api/task/run`
- `POST /api/task/pause`
- `POST /api/task/stop`
- `POST /api/task/rtl`


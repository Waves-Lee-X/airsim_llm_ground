# Agentic UAV Control Implementation Plan

## 1. Goal

This document describes how to turn AeroMind Console into a true LLM-driven UAV system.

The goal is not to let the LLM directly output low-level flight velocity commands. Instead, the LLM acts as a mission-level agent. It observes the UAV state, camera perception, and mission progress, then selects safe tools such as takeoff, search area, scan object, hover, or return home.

Core idea:

```text
LLM Agent = mission commander
Tools = safe UAV skills
AirSim = execution environment
Vision module = perception feedback
Safety layer = guardrail between LLM and UAV
```

## 2. Target Demonstration

User says:

```text
Search the front 60 meters. If you find a vehicle, hover above it and report its position.
```

Expected system behavior:

1. Agent reads current UAV state.
2. Agent calls `takeoff(altitude_m=8)`.
3. Agent calls `search_area(target="vehicle", area=...)`.
4. Executor generates a lawnmower path.
5. UAV flies through waypoints.
6. Vision detector checks camera frames.
7. Detector finds a vehicle.
8. Observation is sent back to the agent.
9. Agent calls `hover()`.
10. Agent calls `report_target()`.
11. UI shows target, map marker, snapshot, and mission report.

## 3. Why Not Direct LLM Flight Control

Do not let the LLM directly control low-level flight commands such as:

```json
{"vx": 1.2, "vy": -0.3, "vz": 0.1}
```

Reasons:

- LLM output is not deterministic enough for low-level control.
- It is unsafe to let language output directly drive motors.
- It is hard to verify, debug, and reproduce.
- Latency is too high for stable closed-loop control.

Correct approach:

```text
LLM chooses high-level tool -> backend validates -> controller executes safely
```

## 4. System Architecture

```text
User Prompt / Voice
      |
      v
LLM Agent
      |
      v
Tool Call JSON
      |
      v
Safety Gate
      |
      v
Tool Runtime
      |
      +--> AirSimAdapter
      +--> TaskExecutor
      +--> PathPlanner
      +--> VisionDetector
      +--> MissionMemory
      |
      v
Observation
      |
      v
LLM Agent loop continues
```

## 5. Agent Loop

The agent runs a loop:

```text
while mission not done:
    observation = collect_state()
    decision = llm_agent.decide(observation)
    checked_tool_call = safety_gate.validate(decision)
    result = tool_runtime.execute(checked_tool_call)
    memory.append(observation, decision, result)
```

Each loop should be slow and mission-level, not flight-control-level.

Recommended interval:

- Normal mission reasoning: 1-3 seconds.
- Vision detection: 2-10 FPS depending on hardware.
- Flight control: handled by AirSim / executor, not LLM.

## 6. Observation Format

The LLM should receive compact structured observations, not raw video streams.

Example:

```json
{
  "uav": {
    "name": "Drone1",
    "connected": true,
    "x": 12.5,
    "y": -3.1,
    "z": -8.0,
    "altitude_m": 8.0,
    "speed_mps": 2.1,
    "mode": "searching"
  },
  "mission": {
    "goal": "Find a vehicle in the front area",
    "status": "running",
    "current_step": "search_area",
    "elapsed_s": 42.0
  },
  "perception": {
    "detections": [
      {
        "label": "vehicle",
        "confidence": 0.82,
        "bbox": [120, 80, 260, 210],
        "estimated_local_position": {
          "x": 35.2,
          "y": -6.4
        }
      }
    ]
  },
  "safety": {
    "inside_boundary": true,
    "collision": false,
    "altitude_ok": true,
    "timeout_warning": false
  }
}
```

## 7. Tool Schema

All LLM actions must be tool calls with validated JSON.

### 7.1 `takeoff`

```json
{
  "tool": "takeoff",
  "args": {
    "altitude_m": 8
  }
}
```

### 7.2 `hover`

```json
{
  "tool": "hover",
  "args": {}
}
```

### 7.3 `land`

```json
{
  "tool": "land",
  "args": {}
}
```

### 7.4 `return_home`

```json
{
  "tool": "return_home",
  "args": {
    "safe_altitude_m": 8
  }
}
```

### 7.5 `goto_local`

```json
{
  "tool": "goto_local",
  "args": {
    "x": 20,
    "y": 5,
    "z": -8,
    "speed_mps": 3
  }
}
```

### 7.6 `search_area`

```json
{
  "tool": "search_area",
  "args": {
    "target": "vehicle",
    "area": {
      "x_min": 0,
      "x_max": 60,
      "y_min": -20,
      "y_max": 20
    },
    "altitude_m": 8,
    "strategy": "lawnmower"
  }
}
```

### 7.7 `scan_object`

```json
{
  "tool": "scan_object",
  "args": {
    "target": "building",
    "center": {"x": 30, "y": 10},
    "radius_m": 12,
    "altitude_m": 8
  }
}
```

### 7.8 `detect_objects`

```json
{
  "tool": "detect_objects",
  "args": {
    "target": "vehicle",
    "camera": "front_center"
  }
}
```

### 7.9 `report_target`

```json
{
  "tool": "report_target",
  "args": {
    "target_id": "vehicle-001"
  }
}
```

## 8. Safety Gate

Every tool call must pass through a safety gate before execution.

Safety checks:

- Maximum altitude.
- Minimum altitude.
- Maximum speed.
- Allowed mission boundary.
- Valid vehicle name.
- Valid camera name.
- Critical action confirmation.
- Collision state.
- Mission timeout.
- AirSim connection status.

Example validation output:

```json
{
  "accepted": true,
  "tool": "goto_local",
  "args": {
    "x": 20,
    "y": 5,
    "z": -8,
    "speed_mps": 3
  },
  "warnings": []
}
```

Rejected example:

```json
{
  "accepted": false,
  "reason": "Altitude 80m exceeds configured maximum 30m"
}
```

## 9. Required New Modules

Recommended directory:

```text
AeroMind Console/core/
├─ agent_loop.py
├─ agent_tools.py
├─ safety_gate.py
├─ mission_memory.py
├─ task_executor.py
├─ vision_detector.py
├─ report_builder.py
└─ llm_client.py
```

### 9.1 `agent_tools.py`

Defines all callable tools and their schemas.

Responsibilities:

- Tool registry.
- Tool schema metadata.
- Runtime tool dispatch.
- Uniform tool result format.

### 9.2 `safety_gate.py`

Validates every tool call.

Responsibilities:

- Clamp unsafe values.
- Reject dangerous requests.
- Require confirmation for critical commands.
- Produce validation result.

### 9.3 `agent_loop.py`

Runs the mission-level LLM loop.

Responsibilities:

- Build observations.
- Call LLM.
- Parse tool calls.
- Send tool calls to safety gate.
- Execute accepted tools.
- Store results in memory.
- Stop when mission completes.

### 9.4 `mission_memory.py`

Stores mission state.

Responsibilities:

- Current mission goal.
- Tool call history.
- Detection history.
- Last known target.
- Searched regions.
- Mission report assets.

### 9.5 `llm_client.py`

LLM provider abstraction.

Responsibilities:

- Call OpenAI / local model / other model.
- Enforce JSON output.
- Retry malformed output.
- Return structured decisions.

### 9.6 `vision_detector.py`

Detects objects in camera frames.

Responsibilities:

- Run YOLO / detector.
- Return labels, confidence, and bounding boxes.
- Optionally estimate local target position.

### 9.7 `report_builder.py`

Generates mission report.

Responsibilities:

- Summarize mission.
- Include target detections.
- Include coordinates.
- Include snapshots.
- Include route and event log.

## 10. Implementation Phases

## Phase A: Tool Runtime Foundation

Goal:

Create a reliable tool interface before adding LLM.

Tasks:

- Add `agent_tools.py`.
- Add tool result format.
- Wrap existing flight APIs as internal tools:
  - `takeoff`
  - `hover`
  - `land`
  - `return_home`
  - `goto_local`
- Add `safety_gate.py`.
- Add unit-style manual test endpoint:
  - `POST /api/tools/call`

Acceptance:

- Tools can be called through JSON.
- Unsafe values are rejected.
- Logs show tool execution result.

## Phase B: Area Search Tool

Goal:

Make `search_area` executable without LLM first.

Tasks:

- Implement `goto_local`.
- Implement waypoint execution.
- Use `lawnmower_path`.
- Add progress state:
  - current waypoint index
  - total waypoints
  - distance to waypoint
- Add tool:
  - `search_area`

Acceptance:

- Calling `search_area` makes UAV fly through a lawnmower path.
- Mission map shows route and trail.
- Event stream shows waypoint progress.

## Phase C: Vision Detection Tool

Goal:

Make the UAV able to perceive targets.

Tasks:

- Connect YOLOv5 / YOLOv8.
- Read latest camera frame.
- Detect target object.
- Return detection result.
- Draw detection boxes in UI.
- Add target markers to map.

Acceptance:

- Calling `detect_objects(target="vehicle")` returns detections.
- UI shows bounding boxes.
- Map shows target marker.

## Phase D: Agent Loop Without LLM

Goal:

Implement the control loop with a deterministic policy first.

Tasks:

- Add `agent_loop.py`.
- Hard-code policy:
  1. takeoff
  2. search_area
  3. detect_objects while searching
  4. hover when target found
  5. report_target
- Use observations and tool results.

Acceptance:

- The system completes the full search-find-hover-report loop without real LLM.
- This proves the architecture works.

## Phase E: LLM Decision Layer

Goal:

Replace deterministic policy with LLM tool selection.

Tasks:

- Add `llm_client.py`.
- Define system prompt.
- Provide tool schemas to LLM.
- Force JSON tool output.
- Validate with `safety_gate`.
- Add retry on invalid JSON.

Acceptance:

- LLM can choose tools based on observation.
- LLM can complete simple missions.
- Invalid tool calls are rejected safely.

## Phase F: Voice Input

Goal:

Allow voice to start an agent mission.

Tasks:

- Use browser Web Speech API first.
- Fill recognized text into task input.
- User confirms before execution.
- Later: backend ASR option.

Acceptance:

- User can speak a mission and start the agent loop.

## Phase G: Mission Report

Goal:

Generate a final report after mission completion.

Tasks:

- Add `report_builder.py`.
- Include:
  - task goal
  - route
  - detections
  - target coordinates
  - snapshots
  - event log
  - success/failure status

Acceptance:

- UI shows a complete report after mission.

## 11. First Real Milestone

The first milestone that proves real LLM-driven UAV control is:

```text
Natural language mission
-> structured agent goal
-> takeoff tool
-> search_area tool
-> detect_objects tool
-> hover tool
-> report_target tool
```

Even if the first version uses a rule-based planner before the real LLM, the architecture should already look like tool-driven agent control.

## 12. Recommended Next Code Task

Start with Phase A:

1. Create `core/agent_tools.py`.
2. Create `core/safety_gate.py`.
3. Add `/api/tools/call`.
4. Wrap existing basic flight controls as tools.
5. Use a consistent tool result format.

After Phase A, implement Phase B:

1. Add `goto_local`.
2. Add `search_area`.
3. Execute the lawnmower path.

This creates the foundation for the real LLM agent loop.


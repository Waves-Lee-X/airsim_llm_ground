from __future__ import annotations

import json
import logging
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Generator

logger = logging.getLogger(__name__)


@dataclass
class MissionMemory:
    user_goal: str
    max_iterations: int = 12
    max_total_time_s: float = 300.0
    iteration: int = 0
    started_at: float | None = None
    tool_history: list[dict[str, Any]] = field(default_factory=list)

    def record(self, tool: str, args: dict[str, Any], ok: bool, message: str, elapsed_ms: int) -> None:
        self.tool_history.append({
            "iteration": self.iteration,
            "tool": tool,
            "args": args,
            "ok": ok,
            "message": message,
            "elapsed_ms": elapsed_ms,
            "ts": time.time(),
        })

    def is_exhausted(self) -> bool:
        if self.iteration >= self.max_iterations:
            return True
        if self.started_at is not None and (time.time() - self.started_at) > self.max_total_time_s:
            return True
        return False

    def summary(self) -> str:
        if not self.tool_history:
            return "(no actions taken yet)"
        lines = []
        for entry in self.tool_history[-6:]:
            status = "OK" if entry["ok"] else "FAIL"
            lines.append(
                f"  [{entry['iteration']}] {entry['tool']} ({status}): {entry['message'][:120]}"
            )
        return "\n".join(lines)

    def to_api(self) -> dict[str, Any]:
        return {
            "user_goal": self.user_goal,
            "iteration": self.iteration,
            "max_iterations": self.max_iterations,
            "elapsed_s": round(time.time() - self.started_at, 1) if self.started_at else 0.0,
            "tool_history": self.tool_history,
        }


class StateSummarizer:
    @staticmethod
    def build_text(snapshot: dict[str, Any], memory: MissionMemory) -> str:
        uav = snapshot.get("uav") or {}
        safety = snapshot.get("safety") or {}
        vision = snapshot.get("vision") or {}
        task = snapshot.get("task") or {}
        agent_progress = task.get("agent_progress") or {}

        connected = "yes" if snapshot.get("connected") else "no"
        x = uav.get("x", 0.0)
        y = uav.get("y", 0.0)
        z = uav.get("z", 0.0)
        alt = uav.get("altitude_m", 0.0)
        speed = uav.get("speed_mps", 0.0)

        collision = safety.get("collision") or {}
        obstacle = safety.get("obstacle") or {}
        distance = safety.get("distance_sensors") or {}

        has_collision = collision.get("has_collided", False)
        obstacle_risk = obstacle.get("risk_level", "unknown")
        obstacle_blocked = obstacle.get("blocked", False)
        front_m = distance.get("front_m")
        left_m = distance.get("left_m")
        right_m = distance.get("right_m")
        dist_emergency = distance.get("emergency", False)

        detection = vision.get("latest_detection") or {}
        det_ok = detection.get("ok", False)
        detections = detection.get("detections") or []
        det_target = detection.get("target", "")
        det_camera = detection.get("camera", "")

        mission_status = agent_progress.get("status", task.get("status", "idle"))
        mission_tool = agent_progress.get("tool", "")
        wp_idx = agent_progress.get("current_waypoint_index", 0)
        wp_total = agent_progress.get("total_waypoints", 0)
        wp_dist = agent_progress.get("distance_to_waypoint_m")
        mission_msg = agent_progress.get("message", "")

        parts = [
            "--- STATE SUMMARY ---",
            f"POSITION: connected={connected}  x={x:.1f}  y={y:.1f}  z={z:.1f}  altitude={alt:.1f}m  speed={speed:.1f}m/s",
        ]

        safety_parts = [f"collision={'YES' if has_collision else 'none'}"]
        if obstacle_risk != "unknown":
            safety_parts.append(f"obstacle_risk={obstacle_risk}")
        if obstacle_blocked:
            safety_parts.append("obstacle_blocked=yes")
        if front_m is not None:
            safety_parts.append(f"distance_sensors: front={front_m:.1f}m left={left_m}m right={right_m}m")
        if dist_emergency:
            safety_parts.append("DISTANCE_EMERGENCY=yes")
        parts.append("SAFETY: " + "  ".join(safety_parts))

        if det_ok and detections:
            det_strs = []
            for d in detections[:3]:
                label = d.get("label", "?")
                conf = d.get("confidence", 0)
                det_strs.append(f"{label}({conf:.2f})")
            parts.append(f"PERCEPTION: camera={det_camera}  target={det_target}  detections=[{', '.join(det_strs)}]")
        elif det_ok:
            parts.append(f"PERCEPTION: camera={det_camera}  no objects detected")
        else:
            parts.append("PERCEPTION: no detection data available")

        mission_parts = [f"status={mission_status}"]
        if mission_tool:
            mission_parts.append(f"active_tool={mission_tool}")
        if wp_total > 0:
            mission_parts.append(f"waypoint={wp_idx}/{wp_total}")
            if wp_dist is not None:
                mission_parts.append(f"distance_to_waypoint={wp_dist:.1f}m")
        if mission_msg:
            mission_parts.append(f"message={mission_msg[:150]}")
        parts.append("MISSION: " + "  ".join(mission_parts))

        parts.append(f"\nACTIONS TAKEN:\n{memory.summary()}")
        parts.append(f"\nITERATION: {memory.iteration}/{memory.max_iterations}")
        parts.append(f"USER GOAL: {memory.user_goal}")
        parts.append("--- END STATE ---")

        return "\n".join(parts)


def build_system_prompt(tool_specs: list[Any], safety_limits: Any) -> str:
    tool_lines = []
    for spec in tool_specs:
        s = spec.to_api() if hasattr(spec, "to_api") else spec
        name = s.get("name", "")
        desc = s.get("description", "")
        required = s.get("required", [])
        props = s.get("properties", {})
        params_str = ", ".join(
            f"{k}: {props.get(k, 'any')}" for k in required
        ) if required else "none"
        tool_lines.append(f"  - {name}: {desc}  Required params: {params_str}")

    limits_dict = {}
    if hasattr(safety_limits, "__dataclass_fields__"):
        for fname in safety_limits.__dataclass_fields__:
            limits_dict[fname] = getattr(safety_limits, fname)
    else:
        limits_dict = dict(safety_limits)

    return f"""You are a UAV mission commander controlling an AirSim multirotor drone.
Your job is to understand the user's goal and decide, step by step, which tool to call
next to accomplish the mission safely.

You receive:
  1. A STATE SUMMARY with the drone's position, safety status, and detection results.
  2. A list of available tools with their required parameters.
  3. A history of actions already taken and their results.

You must respond with EXACTLY ONE of these JSON objects:

  A) To call a tool:
     {{"tool_call": {{"name": "<tool>", "args": {{<params>}}}}}}

  B) To finish the mission:
     {{"finish": {{"message": "<reason the mission is done>"}}}}

Rules:
  - Call only ONE tool per response. State updates after each call.
  - Check the SAFETY section before calling flight tools.
  - If a tool fails, consider a different approach or retry with adjusted parameters.
  - If the user's goal is achieved, use "finish".
  - If stuck after 3 failed attempts on the same tool, "finish" with explanation.
  - For "search_area" or "collect_images", the area must have x_min/x_max/y_min/y_max.
  - "land" and "return_home" require confirmed=true in args.
  - If unsure what to do, use "finish" with: "Cannot determine next action: <reason>".
  - Respond ONLY with the JSON object, no additional text.

SAFETY LIMITS (enforced by hardware safety gate):
  - Altitude: {limits_dict.get('min_altitude_m', 1.0)}m to {limits_dict.get('max_altitude_m', 30.0)}m
  - Speed: max {limits_dict.get('max_speed_mps', 4.0)} m/s
  - Boundary: x=[{limits_dict.get('boundary_x_min', -120)}, {limits_dict.get('boundary_x_max', 120)}],
    y=[{limits_dict.get('boundary_y_min', -120)}, {limits_dict.get('boundary_y_max', 120)}]

AVAILABLE TOOLS:
""" + "\n".join(tool_lines)


def parse_llm_decision(content: str) -> tuple[str | None, dict[str, Any] | None, str | None]:
    """Parse LLM response into (tool_name, tool_args, finish_message).
    Returns (None, None, None) if unparseable.
    """
    if not content or not content.strip():
        return None, None, None

    text = content.strip()
    # Remove markdown code fences
    if text.startswith("```"):
        lines = text.split("\n")
        if len(lines) > 1:
            text = "\n".join(lines[1:])
        if text.endswith("```"):
            text = text[:-3]

    # Try to find a JSON object
    brace_start = text.find("{")
    brace_end = text.rfind("}")
    if brace_start < 0 or brace_end <= brace_start:
        return None, None, None

    try:
        parsed = json.loads(text[brace_start:brace_end + 1])
    except json.JSONDecodeError:
        logger.debug("LLM response not valid JSON: %.200s", text)
        return None, None, None

    if not isinstance(parsed, dict):
        return None, None, None

    # Check for tool_call
    tool_call = parsed.get("tool_call")
    if isinstance(tool_call, dict):
        name = str(tool_call.get("name", "")).strip()
        args = tool_call.get("args") or tool_call.get("params") or tool_call.get("parameters") or {}
        if not isinstance(args, dict):
            args = {}
        if name:
            return name, args, None

    # Check for finish
    finish = parsed.get("finish")
    if isinstance(finish, dict):
        msg = str(finish.get("message", "Mission complete."))
        return None, None, msg
    if isinstance(finish, str):
        return None, None, finish

    # Check for action-based format (backward compat)
    action = parsed.get("action", "")
    if action == "finish" or action == "done" or action == "complete":
        return None, None, str(parsed.get("message", parsed.get("summary", "Done.")))

    return None, None, None


_TERMINAL_TOOLS = {"land", "stop", "return_home"}


class AgentLoop:
    def __init__(
        self,
        llm_client: Any,
        safety_gate: Any,
        tools: Any,
    ) -> None:
        self.llm = llm_client
        self.safety = safety_gate
        self.tools = tools
        self._stop_event = threading.Event()
        self._running = False

    def stop(self) -> None:
        self._stop_event.set()
        try:
            self.tools._stop_active_mission()
        except Exception:
            pass

    @property
    def running(self) -> bool:
        return self._running

    def execute(
        self,
        user_text: str,
        state_provider: Callable[[], dict[str, Any]],
    ) -> Generator[dict[str, Any], None, None]:
        memory = MissionMemory(user_goal=user_text.strip())
        tool_specs = self.tools.specs() if hasattr(self.tools, "specs") else []
        safety_limits = getattr(self.safety, "limits", {})
        system_prompt = build_system_prompt(tool_specs, safety_limits)

        memory.started_at = time.time()
        self._running = True

        yield {"type": "start", "goal": memory.user_goal, "memory": memory.to_api()}

        try:
            while not memory.is_exhausted() and not self._stop_event.is_set():
                memory.iteration += 1

                snapshot = state_provider()
                state_text = StateSummarizer.build_text(snapshot, memory)
                tool_list_text = self._short_tool_list()

                messages = [
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": tool_list_text + "\n\n" + state_text},
                ]
                self._append_history(messages, memory, max_rounds=3)

                yield {
                    "type": "think",
                    "iteration": memory.iteration,
                    "memory": memory.to_api(),
                }

                raw = self._call_llm(messages)
                if raw is None:
                    yield {
                        "type": "error",
                        "message": "LLM API call failed",
                        "iteration": memory.iteration,
                    }
                    memory.record("_llm_error", {}, False, "LLM API call failed", 0)
                    continue

                tool_name, tool_args, finish_msg = parse_llm_decision(raw)

                if finish_msg is not None:
                    yield {"type": "finish", "message": finish_msg, "memory": memory.to_api()}
                    break

                if tool_name is None:
                    yield {
                        "type": "error",
                        "message": f"Could not parse LLM response",
                        "raw": raw[:500],
                        "iteration": memory.iteration,
                    }
                    memory.record("_parse_error", {}, False, f"Unparseable: {raw[:200]}", 0)
                    continue

                tool_args = tool_args if isinstance(tool_args, dict) else {}
                yield {
                    "type": "tool_call",
                    "tool": tool_name,
                    "args": tool_args,
                    "iteration": memory.iteration,
                    "memory": memory.to_api(),
                }

                connected = snapshot.get("connected", False)
                decision = self.safety.validate(tool_name, tool_args, connected=connected)
                if not decision.accepted:
                    yield {
                        "type": "tool_rejected",
                        "tool": tool_name,
                        "safety": decision.to_api(),
                        "iteration": memory.iteration,
                    }
                    memory.record(tool_name, tool_args, False, decision.reason, 0)
                    continue

                t0 = time.time()
                try:
                    result = self.tools.call(decision.tool, decision.args)
                except Exception as exc:
                    elapsed = int((time.time() - t0) * 1000)
                    memory.record(decision.tool, decision.args, False, str(exc), elapsed)
                    yield {
                        "type": "tool_result",
                        "tool": decision.tool,
                        "result": {"ok": False, "message": str(exc)},
                        "iteration": memory.iteration,
                    }
                    continue

                elapsed = int((time.time() - t0) * 1000)
                memory.record(
                    decision.tool,
                    decision.args,
                    result.ok,
                    result.message if hasattr(result, "message") else str(result),
                    elapsed,
                )

                result_dict = result.to_api() if hasattr(result, "to_api") else {"ok": result.ok, "message": str(result)}
                yield {
                    "type": "tool_result",
                    "tool": decision.tool,
                    "result": result_dict,
                    "iteration": memory.iteration,
                    "memory": memory.to_api(),
                }

                if decision.tool in _TERMINAL_TOOLS and result.ok:
                    yield {
                        "type": "finish",
                        "message": f"Mission ended by {decision.tool}",
                        "memory": memory.to_api(),
                    }
                    break

            if memory.is_exhausted():
                yield {
                    "type": "finish",
                    "message": f"Agent loop exhausted: iteration={memory.iteration}, time={time.time() - memory.started_at:.0f}s",
                    "memory": memory.to_api(),
                }

        finally:
            self._running = False
            yield {"type": "done", "memory": memory.to_api()}

    def _call_llm(self, messages: list[dict[str, str]]) -> str | None:
        if hasattr(self.llm, "chat"):
            return self.llm.chat(messages)
        if hasattr(self.llm, "plan_mission"):
            user_content = ""
            for m in messages:
                if m.get("role") == "user":
                    user_content = m.get("content", "")
                    break
            result = self.llm.plan_mission(user_content)
            return json.dumps(result, ensure_ascii=False) if result else None
        return None

    @staticmethod
    def _short_tool_list() -> str:
        return "Respond with JSON: {\"tool_call\": {\"name\": \"...\", \"args\": {...}}} or {\"finish\": {\"message\": \"...\"}}"

    @staticmethod
    def _append_history(messages: list[dict], memory: MissionMemory, max_rounds: int) -> None:
        recent = memory.tool_history[-max_rounds * 2:]
        for entry in recent:
            tool = entry.get("tool", "")
            if tool.startswith("_"):
                continue
            args_str = json.dumps(entry.get("args", {}), ensure_ascii=False)
            status = "OK" if entry.get("ok") else "FAILED"
            messages.append({
                "role": "assistant",
                "content": f"I called {tool} with args: {args_str}",
            })
            messages.append({
                "role": "user",
                "content": f"Result ({status}): {entry.get('message', '')}",
            })

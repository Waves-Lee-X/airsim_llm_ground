from __future__ import annotations

import json
import logging
from dataclasses import dataclass
from typing import Any
from urllib import request, error as urllib_error

logger = logging.getLogger(__name__)

MISSION_PROMPT_TEMPLATE = """\
You are a UAV mission planner for an AirSim drone. Parse the user's natural language instruction into a structured mission plan JSON.

Available intents and their parameters:
- takeoff: altitude_m (float, default 8.0)
- land: no extra params
- return_to_launch: safe_altitude_m (float, default 8.0)
- autonomous_navigation: goal.x, goal.y (required), altitude_m, speed_mps, avoidance (bool)
- area_search: target (string), altitude_m, x_max, half_width
- image_collection: target, altitude_m, x_max, half_width, dataset_name
- formation_flight: count (int), shape ("line"|"v"), spacing_m, altitude_m
- object_or_area_scan: target, altitude_m
- path_planning: waypoints list of {x,y,z}, altitude_m, speed_mps
- waypoint_route: waypoints list of {x,y,z}, speed_mps, avoidance

Respond with ONLY a JSON object (no markdown, no explanation):
{
  "task": "<original user text>",
  "intent": "<one of the intents above>",
  "target": "<detected target or 'unknown'>",
  "altitude_m": <float, default 8.0>,
  "speed_mps": <float, default 2.0>,
  "strategy": "<brief strategy description>",
  "goal": {"x": <float>, "y": <float>} or null,
  "count": <int or null>,
  "shape": "<'line'|'v'|null>",
  "area": {"x_min": 0.0, "x_max": <float>, "y_min": <float>, "y_max": <float>} or null,
  "waypoints": [{"x": <float>, "y": <float>, "z": <float>}] or null,
  "plan": [
    {"name": "<step name>", "detail": "<step description>", "action": "<action verb>"}
  ]
}

Examples:
User: "起飞到15米高度"
Response: {"task": "起飞到15米高度", "intent": "takeoff", "target": "unknown", "altitude_m": 15.0, "speed_mps": 2.0, "strategy": "takeoff_and_hold", "goal": null, "count": null, "shape": null, "area": null, "waypoints": null, "plan": [{"name": "解析起飞高度", "detail": "提取目标高度并检查安全约束。", "action": "parse"}, {"name": "解锁并起飞", "detail": "连接当前选中的无人机并上升。", "action": "takeoff"}, {"name": "到达后悬停", "detail": "到达目标高度后进入悬停保持。", "action": "hover"}]}

User: "搜索汽车目标，高度10米，前方80米，左右30米"
Response: {"task": "搜索汽车目标", "intent": "area_search", "target": "vehicle", "altitude_m": 10.0, "speed_mps": 2.0, "strategy": "lawnmower", "goal": null, "count": null, "shape": null, "area": {"x_min": 0.0, "x_max": 80.0, "y_min": -30.0, "y_max": 30.0}, "waypoints": null, "plan": [{"name": "解析搜索任务", "detail": "提取目标类型、搜索区域和高度约束。", "action": "parse"}, {"name": "生成覆盖航线", "detail": "为指定区域生成往复扫描航点。", "action": "plan_path"}, {"name": "起飞并进入区域", "detail": "达到安全高度后飞向第一个搜索航点。", "action": "takeoff"}, {"name": "运行视觉检测", "detail": "读取相机画面并在飞行过程中检测目标。", "action": "detect"}, {"name": "确认目标并悬停", "detail": "发现目标后报告坐标、保存截图并保持悬停。", "action": "hover"}]}

User: "fly to x=50, y=30 at 12 meters, speed 4 m/s"
Response: {"task": "fly to x=50, y=30 at 12 meters, speed 4 m/s", "intent": "autonomous_navigation", "target": "unknown", "altitude_m": 12.0, "speed_mps": 4.0, "strategy": "closed_loop_autonomous_nav", "goal": {"x": 50.0, "y": 30.0}, "count": null, "shape": null, "area": null, "waypoints": null, "plan": [{"name": "解析导航目标", "detail": "提取目标点和安全约束。", "action": "parse"}, {"name": "检查障碍传感器", "detail": "使用 LiDAR、距离传感器和碰撞反馈评估风险。", "action": "sense"}, {"name": "执行安全导航", "detail": "使用局部避障和速度平滑飞行。", "action": "navigate"}, {"name": "到达后悬停", "detail": "在目标点停止并报告本地坐标。", "action": "hold"}]}

User: "{user_text}"
Response:"""


@dataclass
class LLMConfig:
    api_key: str = ""
    base_url: str = "https://api.openai.com/v1"
    model: str = "gpt-4o-mini"
    timeout_s: float = 15.0
    max_tokens: int = 1024
    temperature: float = 0.2


class LLMClient:
    def __init__(self, config: LLMConfig | None = None) -> None:
        self.config = config or LLMConfig()

    @property
    def enabled(self) -> bool:
        return bool(self.config.api_key)

    def plan_mission(self, text: str) -> dict[str, Any] | None:
        if not self.enabled:
            return None
        prompt = MISSION_PROMPT_TEMPLATE.format(user_text=text)
        payload = json.dumps({
            "model": self.config.model,
            "messages": [{"role": "user", "content": prompt}],
            "max_tokens": self.config.max_tokens,
            "temperature": self.config.temperature,
        }).encode("utf-8")

        req = request.Request(
            f"{self.config.base_url.rstrip('/')}/chat/completions",
            data=payload,
            headers={
                "Authorization": f"Bearer {self.config.api_key}",
                "Content-Type": "application/json",
            },
        )
        try:
            with request.urlopen(req, timeout=self.config.timeout_s) as resp:
                body = json.loads(resp.read().decode("utf-8"))
        except (urllib_error.URLError, OSError, ValueError) as exc:
            logger.warning("LLM API call failed: %s", exc)
            return None

        try:
            content = body["choices"][0]["message"]["content"]
        except (KeyError, IndexError, TypeError) as exc:
            logger.warning("LLM response missing expected fields: %s", exc)
            return None

        return self._extract_json(content)

    def chat(self, messages: list[dict[str, str]], max_tokens: int | None = None) -> str | None:
        """Send a multi-message conversation and return the model's text response."""
        if not self.enabled:
            return None
        payload = json.dumps({
            "model": self.config.model,
            "messages": messages,
            "max_tokens": max_tokens or self.config.max_tokens,
            "temperature": self.config.temperature,
        }).encode("utf-8")

        req = request.Request(
            f"{self.config.base_url.rstrip('/')}/chat/completions",
            data=payload,
            headers={
                "Authorization": f"Bearer {self.config.api_key}",
                "Content-Type": "application/json",
            },
        )
        try:
            with request.urlopen(req, timeout=self.config.timeout_s) as resp:
                body = json.loads(resp.read().decode("utf-8"))
        except (urllib_error.URLError, OSError, ValueError) as exc:
            logger.warning("LLM chat API call failed: %s", exc)
            return None

        try:
            return body["choices"][0]["message"]["content"]
        except (KeyError, IndexError, TypeError) as exc:
            logger.warning("LLM chat response missing expected fields: %s", exc)
            return None

    @staticmethod
    def _extract_json(content: str) -> dict[str, Any] | None:
        content = content.strip()
        if content.startswith("```"):
            lines = content.split("\n")
            content = "\n".join(lines[1:]) if len(lines) > 1 else content
            if content.endswith("```"):
                content = content[:-3]
        try:
            parsed = json.loads(content)
        except json.JSONDecodeError:
            # Try to extract JSON object from longer text
            brace_start = content.find("{")
            brace_end = content.rfind("}")
            if brace_start >= 0 and brace_end > brace_start:
                try:
                    parsed = json.loads(content[brace_start:brace_end + 1])
                except json.JSONDecodeError:
                    logger.warning("LLM response was not valid JSON: %.200s", content)
                    return None
            else:
                logger.warning("LLM response contained no JSON object: %.200s", content)
                return None
        return parsed if isinstance(parsed, dict) else None

from __future__ import annotations

import json
import re
from typing import Any

from core.task_schema import MissionArea, MissionPlan, PlanStep


class TaskPlanner:
    """Rule-based Chinese/English planner used before the real LLM planner is connected."""

    INTENT_KEYWORDS = {
        "autonomous_navigation": (
            "\u81ea\u4e3b\u5bfc\u822a",
            "\u95ed\u73af\u5bfc\u822a",
            "\u5b9e\u65f6\u5bfc\u822a",
            "autonomous navigation",
            "closed loop navigation",
        ),
        "image_collection": (
            "\u91c7\u96c6\u56fe\u50cf",
            "\u6536\u96c6\u56fe\u50cf",
            "\u62cd\u56fe",
            "\u62cd\u7167",
            "\u8bad\u7ec3\u6570\u636e",
            "\u6570\u636e\u96c6",
            "collect images",
            "image collection",
            "dataset",
            "training data",
        ),
        "takeoff": (
            "\u8d77\u98de",
            "\u5347\u7a7a",
            "\u722c\u5347",
            "\u8d77\u98de\u5230",
            "takeoff",
            "take off",
            "launch",
        ),
        "formation_flight": (
            "\u7f16\u961f",
            "\u961f\u5f62",
            "\u961f\u5217",
            "formation",
        ),
        "area_search": (
            "\u641c\u7d22",
            "\u641c\u5bfb",
            "\u641c\u67e5",
            "\u67e5\u627e",
            "\u5bfb\u627e",
            "\u5de1\u641c",
            "search",
            "find",
            "locate",
        ),
        "object_or_area_scan": (
            "\u626b\u63cf",
            "\u5de1\u68c0",
            "\u89c2\u5bdf",
            "\u73af\u7ed5",
            "\u62cd\u6444",
            "scan",
            "inspect",
            "observe",
        ),
        "obstacle_aware_navigation": (
            "\u907f\u969c",
            "\u7ed5\u969c",
            "\u7ed5\u5f00",
            "\u8eb2\u907f",
            "\u969c\u788d",
            "avoid",
            "obstacle",
        ),
        "path_planning": (
            "\u8def\u5f84",
            "\u822a\u7ebf",
            "\u822a\u8def",
            "\u822a\u70b9",
            "\u8def\u7ebf",
            "\u89c4\u5212",
            "path",
            "route",
            "waypoint",
        ),
    }
    TARGET_KEYWORDS = {
        "vehicle": (
            "\u8f66\u8f86",
            "\u6c7d\u8f66",
            "\u8f7f\u8f66",
            "\u5361\u8f66",
            "\u8f66",
            "car",
            "vehicle",
            "truck",
        ),
        "person": (
            "\u884c\u4eba",
            "\u4eba\u5458",
            "\u4eba",
            "\u76ee\u6807\u4eba",
            "person",
            "pedestrian",
            "human",
        ),
        "building": (
            "\u5efa\u7b51",
            "\u697c",
            "\u623f\u5c4b",
            "\u5efa\u7b51\u7269",
            "building",
            "house",
        ),
        "obstacle": (
            "\u969c\u788d\u7269",
            "\u969c\u788d",
            "\u6811",
            "\u5899",
            "\u8def\u969c",
            "obstacle",
            "tree",
            "wall",
        ),
    }

    def plan(self, text: str) -> MissionPlan:
        raw = text.strip()
        if not raw:
            raw = "No mission input"
        parsed_json = self._try_parse_json(raw)
        if parsed_json:
            return self._from_json(raw, parsed_json)

        intent, intent_scores = self.infer_intent(raw)
        target = self.infer_target(raw)
        altitude_m = self.extract_altitude(raw)
        speed_mps = self.extract_speed(raw)
        goal = self.extract_goal_local(raw)
        count = self.extract_count(raw)
        shape = self.extract_shape(raw)
        shared_raw = {
            "nlp": {
                "intent_scores": intent_scores,
                "speed_mps": speed_mps,
                "goal": goal,
                "count": count,
                "shape": shape,
            }
        }

        if intent == "image_collection":
            mission = self._image_collection_plan(raw, target, altitude_m)
            return MissionPlan(**{**mission.__dict__, "raw": {**mission.raw, **shared_raw}})
        if intent == "area_search":
            mission = self._area_search_plan(raw, target, altitude_m)
            return MissionPlan(**{**mission.__dict__, "raw": {**mission.raw, **shared_raw}})
        if intent == "takeoff":
            mission = self._takeoff_plan(raw, altitude_m)
            return MissionPlan(**{**mission.__dict__, "raw": {**mission.raw, **shared_raw}})
        if intent == "formation_flight":
            mission = self._formation_plan(raw, altitude_m)
            return MissionPlan(**{**mission.__dict__, "raw": {**mission.raw, **shared_raw}})
        if intent == "object_or_area_scan":
            mission = self._scan_plan(raw, target, altitude_m)
            return MissionPlan(**{**mission.__dict__, "raw": {**mission.raw, **shared_raw}})
        if intent in {"autonomous_navigation", "obstacle_aware_navigation"}:
            mission = self._obstacle_plan(raw, altitude_m)
            return MissionPlan(**{**mission.__dict__, "raw": {**mission.raw, **shared_raw}})
        if intent == "path_planning":
            mission = self._path_plan(raw, altitude_m)
            return MissionPlan(**{**mission.__dict__, "raw": {**mission.raw, **shared_raw}})
        mission = self._general_plan(raw, altitude_m)
        return MissionPlan(**{**mission.__dict__, "raw": {**mission.raw, **shared_raw}})

    def return_to_launch_plan(self) -> MissionPlan:
        return MissionPlan(
            task="\u8fd4\u56de\u8d77\u98de\u70b9",
            intent="return_to_launch",
            strategy="safe_rtl",
            plan=[
                PlanStep("\u505c\u6b62\u5f53\u524d\u4efb\u52a1", "\u53d6\u6d88\u6b63\u5728\u8fd0\u884c\u7684\u641c\u7d22\u3001\u626b\u63cf\u6216\u7f16\u961f\u4efb\u52a1\u3002", "cancel"),
                PlanStep("\u89c4\u5212\u8fd4\u822a\u8def\u5f84", "\u5728\u5b89\u5168\u9ad8\u5ea6\u8fd4\u56de\u672c\u5730\u539f\u70b9\u3002", "rtl"),
                PlanStep("\u60ac\u505c\u6216\u964d\u843d", "\u6839\u636e\u5b89\u5168\u7b56\u7565\u4fdd\u6301\u60ac\u505c\u6216\u6267\u884c\u964d\u843d\u3002", "hold"),
            ],
        )

    def infer_intent(self, text: str) -> tuple[str, dict[str, int]]:
        raw = text.lower()
        ordered = (
            "image_collection",
            "autonomous_navigation",
            "takeoff",
            "formation_flight",
            "area_search",
            "object_or_area_scan",
            "obstacle_aware_navigation",
            "path_planning",
        )
        scores: dict[str, int] = {key: 0 for key in ordered}
        for intent in ordered:
            for term in self.INTENT_KEYWORDS[intent]:
                if self._contains_term(text, raw, term):
                    scores[intent] += 1
        best_intent = max(scores, key=lambda key: scores[key]) if scores else "general_uav_task"
        if scores.get(best_intent, 0) <= 0:
            return "general_uav_task", scores
        return best_intent, scores

    def infer_target(self, text: str) -> str:
        raw = text.lower()
        for target, terms in self.TARGET_KEYWORDS.items():
            if self._contains_any(text, raw, terms):
                return target
        return "unknown"

    def _image_collection_plan(self, task: str, target: str, altitude_m: float) -> MissionPlan:
        area = self.extract_area(task)
        return MissionPlan(
            task=task,
            intent="image_collection",
            target=target,
            altitude_m=altitude_m,
            strategy="collect_images_with_avoidance",
            area=area,
            plan=[
                PlanStep("\u786e\u5b9a\u91c7\u96c6\u533a\u57df", "\u63d0\u53d6\u56fe\u50cf\u91c7\u96c6\u8303\u56f4\u3001\u9ad8\u5ea6\u548c\u76f8\u673a\u3002", "parse"),
                PlanStep("\u9884\u626b\u63cf\u969c\u788d", "\u8d77\u98de\u540e\u5148\u7528 LiDAR \u786e\u8ba4\u969c\u788d\u7269\u5206\u5e03\u3002", "pre_scan"),
                PlanStep("A* \u91cd\u65b0\u89c4\u5212", "\u6839\u636e\u969c\u788d\u7269\u5730\u56fe\u751f\u6210\u5b89\u5168\u91c7\u96c6\u8def\u5f84\u3002", "plan_path"),
                PlanStep("\u91c7\u96c6\u8bad\u7ec3\u56fe\u50cf", "\u6cbf\u8def\u5f84\u98de\u884c\u5e76\u6309\u95f4\u9694\u4fdd\u5b58\u539f\u59cb\u76f8\u673a\u56fe\u50cf\u3002", "capture"),
            ],
        )

    def _takeoff_plan(self, task: str, altitude_m: float) -> MissionPlan:
        return MissionPlan(
            task=task,
            intent="takeoff",
            altitude_m=altitude_m,
            strategy="takeoff_and_hold",
            plan=[
                PlanStep("\u89e3\u6790\u8d77\u98de\u9ad8\u5ea6", "\u63d0\u53d6\u76ee\u6807\u9ad8\u5ea6\u5e76\u68c0\u67e5\u5b89\u5168\u7ea6\u675f\u3002", "parse"),
                PlanStep("\u89e3\u9501\u5e76\u8d77\u98de", "\u8fde\u63a5\u5f53\u524d\u9009\u4e2d\u7684\u65e0\u4eba\u673a\u5e76\u4e0a\u5347\u3002", "takeoff"),
                PlanStep("\u5230\u8fbe\u540e\u60ac\u505c", "\u5230\u8fbe\u76ee\u6807\u9ad8\u5ea6\u540e\u8fdb\u5165\u60ac\u505c\u4fdd\u6301\u3002", "hover"),
            ],
        )

    def extract_altitude(self, text: str) -> float:
        patterns = (
            r"(?:\u9ad8\u5ea6|\u98de\u9ad8|\u4fdd\u6301|\u722c\u5347\u5230|\u4e0a\u5347\u5230|\u98de\u5230)\s*(\d+(?:\.\d+)?)\s*(?:\u7c73|m|meter|meters)?",
            r"(\d+(?:\.\d+)?)\s*(?:\u7c73|m|meter|meters)\s*(?:\u9ad8\u5ea6|\u9ad8\u7a7a|\u9ad8)",
            r"(?:altitude|height)\s*(\d+(?:\.\d+)?)\s*(?:m|meter|meters)?",
        )
        for pattern in patterns:
            match = re.search(pattern, text, re.IGNORECASE)
            if match:
                return max(1.0, float(match.group(1)))
        return 8.0

    def extract_area(self, text: str) -> MissionArea:
        x_max = self._extract_distance_after(
            text,
            ("\u524d\u65b9", "\u5411\u524d", "\u524d\u9762", "forward", "ahead"),
            default=60.0,
        )
        half_width = self._extract_distance_after(
            text,
            ("\u5de6\u53f3", "\u4e24\u4fa7", "\u6a2a\u5411", "\u4fa7\u5411", "\u5bbd\u5ea6", "side", "wide", "width"),
            default=20.0,
        )
        return MissionArea(0.0, max(5.0, x_max), -max(2.0, half_width), max(2.0, half_width))

    def extract_speed(self, text: str) -> float:
        patterns = (
            r"(?:速度|速率|飞行速度|巡航速度)\s*(\d+(?:\.\d+)?)\s*(?:米每秒|m/s|mps)?",
            r"(\d+(?:\.\d+)?)\s*(?:米每秒|m/s|mps)",
            r"(?:speed)\s*(\d+(?:\.\d+)?)\s*(?:m/s|mps)?",
        )
        for pattern in patterns:
            match = re.search(pattern, text, re.IGNORECASE)
            if match:
                return max(0.5, min(8.0, float(match.group(1))))
        return 2.0

    def extract_goal_local(self, text: str) -> dict[str, float] | None:
        patterns = (
            r"(?:到|飞到|前往|goto|go to)\s*[xX]\s*=?\s*(-?\d+(?:\.\d+)?)\s*[,，\s]+[yY]\s*=?\s*(-?\d+(?:\.\d+)?)",
            r"[xX]\s*=?\s*(-?\d+(?:\.\d+)?)\s*[,，\s]+[yY]\s*=?\s*(-?\d+(?:\.\d+)?)",
        )
        for pattern in patterns:
            match = re.search(pattern, text, re.IGNORECASE)
            if match:
                return {"x": float(match.group(1)), "y": float(match.group(2))}
        return None

    def extract_count(self, text: str) -> int:
        patterns = (
            r"(\d+)\s*(?:架|台|drone|drones|uav|uavs)",
            r"(?:编队|队形|formation)\s*(\d+)",
        )
        for pattern in patterns:
            match = re.search(pattern, text, re.IGNORECASE)
            if match:
                return max(2, min(8, int(match.group(1))))
        return 3

    def extract_shape(self, text: str) -> str:
        raw = text.lower()
        if any(token in text for token in ("一字", "横队", "直线")) or any(token in raw for token in ("line", "column")):
            return "line"
        return "v"

    def _area_search_plan(self, task: str, target: str, altitude_m: float) -> MissionPlan:
        area = self.extract_area(task)
        return MissionPlan(
            task=task,
            intent="area_search",
            target=target,
            altitude_m=altitude_m,
            strategy="lawnmower",
            area=area,
            raw={"on_detect": {"action": "hover_and_report", "report_coordinate": True, "save_snapshot": True}},
            plan=[
                PlanStep("\u89e3\u6790\u641c\u7d22\u4efb\u52a1", "\u63d0\u53d6\u76ee\u6807\u7c7b\u578b\u3001\u641c\u7d22\u533a\u57df\u548c\u9ad8\u5ea6\u7ea6\u675f\u3002", "parse"),
                PlanStep("\u751f\u6210\u8986\u76d6\u822a\u7ebf", "\u4e3a\u6307\u5b9a\u533a\u57df\u751f\u6210\u5f80\u590d\u626b\u63cf\u822a\u70b9\u3002", "plan_path"),
                PlanStep("\u8d77\u98de\u5e76\u8fdb\u5165\u533a\u57df", "\u8fbe\u5230\u5b89\u5168\u9ad8\u5ea6\u540e\u98de\u5411\u7b2c\u4e00\u4e2a\u641c\u7d22\u822a\u70b9\u3002", "takeoff"),
                PlanStep("\u8fd0\u884c\u89c6\u89c9\u68c0\u6d4b", "\u8bfb\u53d6\u76f8\u673a\u753b\u9762\u5e76\u5728\u98de\u884c\u8fc7\u7a0b\u4e2d\u68c0\u6d4b\u76ee\u6807\u3002", "detect"),
                PlanStep("\u786e\u8ba4\u76ee\u6807\u5e76\u60ac\u505c", "\u53d1\u73b0\u76ee\u6807\u540e\u62a5\u544a\u5750\u6807\u3001\u4fdd\u5b58\u622a\u56fe\u5e76\u4fdd\u6301\u60ac\u505c\u3002", "hover"),
            ],
        )

    def _formation_plan(self, task: str, altitude_m: float) -> MissionPlan:
        return MissionPlan(
            task=task,
            intent="formation_flight",
            altitude_m=altitude_m,
            strategy="leader_follower",
            plan=[
                PlanStep("\u89e3\u6790\u7f16\u961f\u9700\u6c42", "\u786e\u5b9a\u7f16\u961f\u5f62\u72b6\u3001\u95f4\u8ddd\u3001\u957f\u673a\u548c\u50da\u673a\u5173\u7cfb\u3002", "parse"),
                PlanStep("\u5206\u914d\u7f16\u961f\u4f4d", "\u751f\u6210\u5404\u65e0\u4eba\u673a\u7684\u76f8\u5bf9\u4f4d\u7f6e\u3002", "assign_slots"),
                PlanStep("\u540c\u6b65\u8d77\u98de", "\u5c06\u6240\u6709\u65e0\u4eba\u673a\u79fb\u52a8\u5230\u4efb\u52a1\u9ad8\u5ea6\u3002", "takeoff"),
                PlanStep("\u8ddf\u968f\u957f\u673a\u822a\u8ff9", "\u6839\u636e\u957f\u673a\u8def\u5f84\u66f4\u65b0\u50da\u673a\u4f4d\u7f6e\u3002", "formation"),
            ],
        )

    def _scan_plan(self, task: str, target: str, altitude_m: float) -> MissionPlan:
        return MissionPlan(
            task=task,
            intent="object_or_area_scan",
            target=target,
            altitude_m=altitude_m,
            strategy="orbit_or_sweep_scan",
            plan=[
                PlanStep("\u786e\u8ba4\u626b\u63cf\u5bf9\u8c61", "\u89e3\u6790\u76ee\u6807\u7269\u6216\u9700\u8981\u89c2\u5bdf\u7684\u533a\u57df\u3002", "parse"),
                PlanStep("\u751f\u6210\u89c2\u6d4b\u822a\u7ebf", "\u751f\u6210\u73af\u7ed5\u3001\u6a2a\u5411\u626b\u63cf\u6216\u5b9a\u70b9\u89c2\u5bdf\u8def\u5f84\u3002", "plan_path"),
                PlanStep("\u91c7\u96c6\u89c6\u89c9\u5e27", "\u8bfb\u53d6\u524d\u89c6\u76f8\u673a\u5e76\u4fdd\u5b58\u5173\u952e\u5e27\u3002", "capture"),
                PlanStep("\u751f\u6210\u626b\u63cf\u62a5\u544a", "\u6c47\u603b\u622a\u56fe\u3001\u5750\u6807\u548c\u68c0\u6d4b\u7ed3\u679c\u3002", "report"),
            ],
        )

    def _obstacle_plan(self, task: str, altitude_m: float) -> MissionPlan:
        return MissionPlan(
            task=task,
            intent="obstacle_aware_navigation",
            altitude_m=altitude_m,
            strategy="closed_loop_autonomous_nav",
            plan=[
                PlanStep("\u89e3\u6790\u5bfc\u822a\u76ee\u6807", "\u63d0\u53d6\u76ee\u6807\u70b9\u548c\u5b89\u5168\u7ea6\u675f\u3002", "parse"),
                PlanStep("\u68c0\u67e5\u969c\u788d\u4f20\u611f\u5668", "\u4f7f\u7528 LiDAR\u3001\u8ddd\u79bb\u4f20\u611f\u5668\u548c\u78b0\u649e\u53cd\u9988\u8bc4\u4f30\u98ce\u9669\u3002", "sense"),
                PlanStep("\u6267\u884c\u5b89\u5168\u5bfc\u822a", "\u4f7f\u7528\u5c40\u90e8\u907f\u969c\u548c\u901f\u5ea6\u5e73\u6ed1\u98de\u884c\u3002", "navigate"),
                PlanStep("\u5230\u8fbe\u540e\u60ac\u505c", "\u5728\u76ee\u6807\u70b9\u505c\u6b62\u5e76\u62a5\u544a\u672c\u5730\u5750\u6807\u3002", "hold"),
            ],
        )

    def _path_plan(self, task: str, altitude_m: float) -> MissionPlan:
        return MissionPlan(
            task=task,
            intent="path_planning",
            altitude_m=altitude_m,
            strategy="closed_loop_autonomous_nav",
            plan=[
                PlanStep("\u89e3\u6790\u8def\u5f84\u7ea6\u675f", "\u63d0\u53d6\u8d77\u70b9\u3001\u7ec8\u70b9\u3001\u9ad8\u5ea6\u548c\u7981\u98de\u7ea6\u675f\u3002", "parse"),
                PlanStep("\u751f\u6210\u822a\u70b9", "\u751f\u6210\u53ef\u5728 AirSim \u672c\u5730\u5750\u6807\u4e2d\u6267\u884c\u7684\u822a\u7ebf\u3002", "plan_path"),
                PlanStep("\u9a8c\u8bc1\u5b89\u5168\u6027", "\u68c0\u67e5\u9ad8\u5ea6\u3001\u8fb9\u754c\u548c\u969c\u788d\u5047\u8bbe\u3002", "validate"),
                PlanStep("\u4e0b\u53d1\u822a\u7ebf", "\u5c06\u822a\u70b9\u53d1\u9001\u7ed9\u4efb\u52a1\u6267\u884c\u5668\u3002", "execute"),
            ],
        )

    def _general_plan(self, task: str, altitude_m: float) -> MissionPlan:
        return MissionPlan(
            task=task,
            intent="general_uav_task",
            altitude_m=altitude_m,
            strategy="guided",
            plan=[
                PlanStep("\u89e3\u6790\u4efb\u52a1", "\u5c06\u81ea\u7136\u8bed\u8a00\u8f6c\u6210\u7ed3\u6784\u5316\u65e0\u4eba\u673a\u4efb\u52a1\u3002", "parse"),
                PlanStep("\u6267\u884c\u5b89\u5168\u68c0\u67e5", "\u68c0\u67e5\u9ad8\u5ea6\u3001\u8fb9\u754c\u3001\u969c\u788d\u548c\u8fde\u63a5\u72b6\u6001\u3002", "validate"),
                PlanStep("\u6267\u884c\u4efb\u52a1", "\u8c03\u7528 AirSim \u63a7\u5236\u548c\u611f\u77e5\u6a21\u5757\u3002", "execute"),
                PlanStep("\u603b\u7ed3\u7ed3\u679c", "\u751f\u6210\u822a\u8ff9\u3001\u4e8b\u4ef6\u548c\u76ee\u6807\u62a5\u544a\u3002", "report"),
            ],
        )

    def _try_parse_json(self, raw: str) -> dict[str, Any] | None:
        try:
            parsed = json.loads(raw)
        except json.JSONDecodeError:
            return None
        return parsed if isinstance(parsed, dict) else None

    def _from_json(self, task: str, parsed: dict[str, Any]) -> MissionPlan:
        intent = str(parsed.get("intent", "general_uav_task"))
        target = str(parsed.get("target", "unknown"))
        altitude_m = float(parsed.get("altitude_m", 8.0))
        strategy = str(parsed.get("strategy", "guided"))
        raw_steps = parsed.get("plan") or parsed.get("steps") or []
        steps = []
        if isinstance(raw_steps, list):
            for item in raw_steps:
                if isinstance(item, dict):
                    steps.append(
                        PlanStep(
                            name=str(item.get("name", item.get("command", "Step"))),
                            detail=str(item.get("detail", item.get("description", ""))),
                            action=str(item.get("action", item.get("command", "note"))),
                        )
                    )
        if not steps:
            steps = self._general_plan(task, altitude_m).plan
        return MissionPlan(str(parsed.get("task", task)), intent, target, altitude_m, strategy, plan=steps, raw=parsed)

    @staticmethod
    def _contains_any(text: str, raw: str, terms: tuple[str, ...]) -> bool:
        return any(TaskPlanner._contains_term(text, raw, term) for term in terms)

    @staticmethod
    def _contains_term(text: str, raw: str, term: str) -> bool:
        if term.isascii():
            return bool(re.search(rf"\b{re.escape(term.lower())}\b", raw))
        return term in text

    @staticmethod
    def _extract_distance_after(text: str, markers: tuple[str, ...], default: float) -> float:
        for marker in markers:
            pattern = rf"{re.escape(marker)}\s*(\d+(?:\.\d+)?)\s*(?:\u7c73|m|meter|meters)?"
            match = re.search(pattern, text, re.IGNORECASE)
            if match:
                return float(match.group(1))
        return default

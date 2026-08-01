"""State-aware conversational Mission Agent with operator-confirmed tools."""

from __future__ import annotations

import asyncio
import hashlib
import json
import math
import time
from dataclasses import dataclass, field
from datetime import datetime, timedelta, timezone
from typing import Any
from uuid import UUID, uuid4

from .semantic import SemanticService, extract_json_object


ATOMIC_AGENT_ACTIONS = frozenset(
    {"arm", "disarm", "takeoff", "hold", "land", "rtl", "goto", "formation"}
)


class MissionAgentError(RuntimeError):
    """Base error exposed by the Mission Agent API."""


class AgentSessionNotFound(MissionAgentError):
    pass


class AgentDraftNotFound(MissionAgentError):
    pass


class AgentDraftConflict(MissionAgentError):
    pass


class AgentDraftExpired(MissionAgentError):
    pass


class AgentDraftBlocked(MissionAgentError):
    pass


@dataclass(frozen=True)
class MissionAgentConfig:
    max_sessions: int = 16
    max_turns_per_session: int = 24
    draft_ttl_s: float = 120.0

    def __post_init__(self) -> None:
        if not 1 <= self.max_sessions <= 100:
            raise ValueError("max_sessions must be in [1, 100]")
        if not 2 <= self.max_turns_per_session <= 100:
            raise ValueError("max_turns_per_session must be in [2, 100]")
        if not 10.0 <= self.draft_ttl_s <= 600.0:
            raise ValueError("draft_ttl_s must be in [10, 600]")


@dataclass
class _AgentTurn:
    role: str
    content: str
    created_at_utc: datetime
    payload: dict[str, Any] | None = None

    def public_payload(self) -> dict[str, Any]:
        result = {
            "role": self.role,
            "content": self.content,
            "created_at_utc": self.created_at_utc.isoformat(),
        }
        if self.payload is not None:
            result["payload"] = self.payload
        return result


@dataclass
class _AgentSession:
    session_id: UUID
    created_at_utc: datetime
    updated_at_utc: datetime
    turns: list[_AgentTurn] = field(default_factory=list)
    latest_draft_id: UUID | None = None
    execution_history: list[dict[str, Any]] = field(default_factory=list)


@dataclass
class _AgentDraft:
    draft_id: UUID
    session_id: UUID
    created_at_utc: datetime
    expires_at_utc: datetime
    expires_monotonic_s: float
    action: str
    vehicle_id: int | None
    arguments: dict[str, Any]
    reason: str
    blockers: list[str]
    context_hash: str
    context_summary: dict[str, Any]
    status: str
    execution_result: dict[str, Any] | None = None

    @property
    def executable(self) -> bool:
        return self.status == "pending_confirmation" and not self.blockers


def normalize_agent_result(content: str) -> dict[str, Any]:
    """Normalize a model reply without treating model tool claims as authority."""
    try:
        parsed = extract_json_object(content)
        warning = None
    except ValueError as exc:
        parsed = {}
        warning = str(exc)

    reply = str(parsed.get("reply") or parsed.get("message") or "").strip()
    if not reply:
        reply = str(content or "").strip() or "模型未返回可显示内容"
    risk_level = str(parsed.get("risk_level") or "unknown").strip().lower()
    if risk_level not in {"low", "medium", "high", "unknown"}:
        risk_level = "unknown"
    proposal = parsed.get("proposed_action")
    if not isinstance(proposal, dict):
        proposal = None
    raw_plan = parsed.get("plan")
    plan = [item for item in (raw_plan or []) if isinstance(item, dict)]
    if not plan:
        plan = None
    questions = parsed.get("questions")
    if not isinstance(questions, list):
        questions = []
    return {
        "reply": reply,
        "state_summary": str(parsed.get("state_summary") or "").strip(),
        "risk_level": risk_level,
        "questions": [str(item).strip() for item in questions if str(item).strip()],
        "proposed_action": proposal,
        "plan": plan,
        "format_warning": warning,
    }


class MissionAgentService:
    """Own bounded chat history and deterministic one-shot action drafts."""

    _SYSTEM_PROMPT = (
        "你是 AeroMind APM Lite 的 Mission Agent。你可以解释当前飞机状态、回答问题并生成供操作员确认的草案，"
        "但你从不直接控制无人机。所有事实必须来自提供的状态 JSON，不得编造坐标、传感器、链路或执行结果。"
        "允许建议的单动作：arm、disarm、takeoff、hold、land、rtl；GOTO 与编队（formation）只允许在仿真"
        "（SIM/SITL/DEMO）模式下建议草案。复合任务（需要多个动作才能完成，例如“起飞→飞往→编队→降落”）"
        "应返回 plan 步骤数组，仅仿真模式可执行。搜索、路径规划和 analyze 只能文字说明，不得放入 "
        "proposed_action 或 plan。只返回合法 JSON，不输出 Markdown。字段为 reply、state_summary、"
        "risk_level、questions、proposed_action、plan。proposed_action 为 null 或包含 action、vehicle_id、"
        "arguments、reason；takeoff arguments 只含 altitude_m，其他单动作 arguments 必须为空对象。"
        "plan 为步骤数组，每步包含 action、vehicle_id 或 vehicle_ids、arguments、reason；"
        "支持动作 "
        "arm/disarm/takeoff/goto/formation/hold/land/analyze；goto arguments 含 "
        "target_position_ned_m（[北,东,下]）；"
        "formation arguments 含 formation（line/v/diamond）、leader_target_map_m、spacing_m、altitude_m；"
        "analyze arguments 可含 prompt 与 navigate_after（识别到目标后自动飞往目标，"
        "仅仿真可用），且只能指定一架飞机（到达目标后拍照识别）。"
        "当 analyze 已设置 navigate_after 时，不要再额外生成 goto 步骤。"
        "不要声称动作已经执行。"
    )

    def __init__(
        self,
        semantic: SemanticService,
        config: MissionAgentConfig | None = None,
        *,
        clock: Any = time.monotonic,
    ) -> None:
        self.semantic = semantic
        self.config = config or MissionAgentConfig()
        self._clock = clock
        self._sessions: dict[UUID, _AgentSession] = {}
        self._drafts: dict[UUID, _AgentDraft] = {}
        self._lock = asyncio.Lock()

    def status_payload(self) -> dict[str, Any]:
        return {
            "available": self.semantic.config.mission_available,
            "model": self.semantic.config.mission_model or None,
            "session_count": len(self._sessions),
            "draft_count": len(self._drafts),
            "allowed_tools": sorted(ATOMIC_AGENT_ACTIONS),
            "execution_policy": "operator_confirmation_required",
            "history_persistence": "memory_only",
            "draft_ttl_s": self.config.draft_ttl_s,
        }

    def create_session(self) -> dict[str, Any]:
        now = datetime.now(timezone.utc)
        if len(self._sessions) >= self.config.max_sessions:
            oldest = min(
                self._sessions.values(),
                key=lambda session: session.updated_at_utc,
            )
            self._remove_session(oldest.session_id)
        session = _AgentSession(
            session_id=uuid4(),
            created_at_utc=now,
            updated_at_utc=now,
        )
        self._sessions[session.session_id] = session
        return self._session_payload(session)

    def session_payload(self, session_id: UUID | str) -> dict[str, Any]:
        return self._session_payload(self._require_session(_uuid(session_id)))

    def close_session(self, session_id: UUID | str) -> dict[str, Any]:
        session_id = _uuid(session_id)
        session = self._require_session(session_id)
        payload = self._session_payload(session)
        self._remove_session(session_id)
        payload["closed"] = True
        return payload

    async def chat(
        self,
        session_id: UUID | str,
        message: str,
        vehicle_context: dict[str, Any],
    ) -> dict[str, Any]:
        message = str(message or "").strip()
        if not message:
            raise ValueError("对话内容不能为空")
        if len(message) > 4_000:
            raise ValueError("对话内容不能超过 4000 个字符")

        async with self._lock:
            session_id = _uuid(session_id)
            session = self._require_session(session_id)
            messages = self._model_messages(session, message, vehicle_context)
            completion = await self.semantic.complete_agent(messages)
            result = normalize_agent_result(completion["content"])
            now = datetime.now(timezone.utc)
            session.turns.append(_AgentTurn("user", message, now))
            proposal = result.get("proposed_action")
            if proposal is None and result.get("plan"):
                proposal = {
                    "action": "plan",
                    "plan": result["plan"],
                    "reason": (
                        str(result["reply"] or "Mission Agent 计划")[:128]
                    ),
                }
            draft = self._create_draft(
                session,
                proposal,
                vehicle_context,
            )
            assistant_payload = {
                "state_summary": result["state_summary"],
                "risk_level": result["risk_level"],
                "questions": result["questions"],
                "format_warning": result["format_warning"],
                "raw_response": completion["content"],
                "draft": self._draft_payload(draft) if draft is not None else None,
            }
            session.turns.append(
                _AgentTurn("assistant", result["reply"], now, assistant_payload)
            )
            session.turns = session.turns[-self.config.max_turns_per_session :]
            session.updated_at_utc = now
            session.latest_draft_id = draft.draft_id if draft is not None else None
            return {
                "kind": "mission_agent_reply",
                "session_id": str(session.session_id),
                "created_at_utc": now.isoformat(),
                "model": completion["model"],
                "latency_ms": completion["latency_ms"],
                "user_message": message,
                "reply": result["reply"],
                "state_summary": result["state_summary"],
                "risk_level": result["risk_level"],
                "questions": result["questions"],
                "format_warning": result["format_warning"],
                "raw_response": completion["content"],
                "draft": self._draft_payload(draft) if draft is not None else None,
                "flight_command_generated": False,
                "vehicle_context": self._context_summary(vehicle_context),
            }

    def claim_draft(
        self,
        draft_id: UUID | str,
        vehicle_context: dict[str, Any],
    ) -> dict[str, Any]:
        draft_id = _uuid(draft_id)
        draft = self._require_draft(draft_id)
        if draft.status != "pending_confirmation":
            raise AgentDraftConflict(
                f"草案状态为 {draft.status}，不能再次确认"
            )
        if self._clock() >= draft.expires_monotonic_s:
            draft.status = "expired"
            raise AgentDraftExpired("任务草案确认票据已过期")
        recheck_proposal = {
            "action": draft.action,
            "vehicle_id": draft.vehicle_id,
            "arguments": draft.arguments,
            "reason": draft.reason,
        }
        if draft.action == "plan":
            recheck_proposal["plan"] = draft.arguments.get("steps")
        blockers, arguments, vehicle_id = self._action_gate(
            recheck_proposal,
            vehicle_context,
        )
        if blockers:
            draft.blockers = blockers
            draft.status = "blocked"
            raise AgentDraftBlocked("；".join(blockers))
        draft.arguments = arguments
        draft.vehicle_id = vehicle_id
        draft.status = "executing"
        return {
            "draft": self._draft_payload(draft),
            "action": draft.action,
            "vehicle_id": vehicle_id,
            "arguments": arguments,
            "reason": draft.reason,
        }

    def cancel_draft(self, draft_id: UUID | str) -> dict[str, Any]:
        draft_id = _uuid(draft_id)
        draft = self._require_draft(draft_id)
        if draft.status != "pending_confirmation":
            raise AgentDraftConflict(f"草案状态为 {draft.status}，不能取消")
        draft.status = "cancelled"
        return self._draft_payload(draft)

    def record_execution(
        self,
        draft_id: UUID | str,
        *,
        succeeded: bool,
        result: dict[str, Any],
    ) -> dict[str, Any]:
        draft_id = _uuid(draft_id)
        draft = self._require_draft(draft_id)
        if draft.status != "executing":
            raise AgentDraftConflict(f"草案状态为 {draft.status}，不能记录执行结果")
        draft.status = "completed" if succeeded else "failed"
        draft.execution_result = result
        session = self._require_session(draft.session_id)
        summary = {
            "action": draft.action,
            "vehicle_id": draft.vehicle_id,
            "arguments": draft.arguments,
            "succeeded": succeeded,
            "result": {
                key: result.get(key)
                for key in ("phase", "stage", "status", "detail", "error")
            },
            "recorded_at_utc": datetime.now(timezone.utc).isoformat(),
        }
        session.execution_history.append(summary)
        session.execution_history = session.execution_history[-8:]
        return self._draft_payload(draft)

    def _validate_plan_steps(
        self,
        steps: list[dict[str, Any]],
        context: dict[str, Any],
    ) -> tuple[list[dict[str, Any]], list[str]]:
        """Validate and normalize a multi-step plan (simulation only)."""
        deployment_mode = str(context.get("deployment_mode") or "").lower()
        simulated = deployment_mode != "real"
        blockers: list[str] = []
        if not simulated:
            blockers.append("计划执行仅仿真（SIM/SITL）模式可用")
            return [], blockers
        if len(steps) > 20:
            blockers.append("计划步骤数不能超过 20")
            return [], blockers
        current_vehicle_id = _integer(context.get("vehicle_id"))
        normalized: list[dict[str, Any]] = []
        for index, step in enumerate(steps, start=1):
            action = str(step.get("action") or "").strip().lower()
            if action == "return_home":
                action = "rtl"
            if action not in {
                "arm",
                "disarm",
                "takeoff",
                "goto",
                "formation",
                "hold",
                "land",
                "analyze",
            }:
                blockers.append(f"第 {index} 步动作 {action!r} 不受支持")
                continue
            raw_ids = step.get("vehicle_ids")
            if raw_ids is None:
                single = _integer(step.get("vehicle_id"))
                raw_ids = (
                    [single]
                    if single is not None
                    else (
                        [current_vehicle_id]
                        if current_vehicle_id is not None
                        else []
                    )
                )
            ids: list[int] = []
            if isinstance(raw_ids, (list, tuple)):
                for item in raw_ids:
                    value = _integer(item)
                    if value is None or not 1 <= value <= 255:
                        blockers.append(f"第 {index} 步飞机编号无效")
                        break
                    ids.append(value)
            else:
                value = _integer(raw_ids)
                if value is None or not 1 <= value <= 255:
                    blockers.append(f"第 {index} 步飞机编号无效")
                else:
                    ids.append(value)
            if len(ids) != len(set(ids)):
                blockers.append(f"第 {index} 步飞机编号重复")
            if action == "formation" and len(ids) < 2:
                blockers.append(f"第 {index} 步编队至少需要 2 架飞机")
            if action == "analyze" and len(ids) != 1:
                blockers.append(f"第 {index} 步 analyze 只能指定一架飞机")

            raw_args = step.get("arguments")
            if not isinstance(raw_args, dict):
                raw_args = {}
            arguments: dict[str, Any] = {}
            if action == "takeoff":
                altitude = _finite_float(raw_args.get("altitude_m"))
                if altitude is None or not 0.5 <= altitude <= 20.0:
                    blockers.append(f"第 {index} 步起飞高度必须在 0.5-20 米")
                else:
                    arguments["altitude_m"] = altitude
            elif action == "goto":
                position = raw_args.get("target_position_ned_m")
                if not isinstance(position, (list, tuple)) or len(position) != 3:
                    blockers.append(f"第 {index} 步 GOTO 需要目标坐标")
                else:
                    try:
                        values = [float(value) for value in position]
                        if not all(math.isfinite(value) for value in values):
                            blockers.append(f"第 {index} 步 GOTO 坐标无效")
                        else:
                            arguments["target_position_ned_m"] = values
                    except (TypeError, ValueError):
                        blockers.append(f"第 {index} 步 GOTO 坐标无效")
            elif action == "analyze":
                prompt = raw_args.get("prompt")
                if prompt is not None and not isinstance(prompt, str):
                    blockers.append(f"第 {index} 步 analyze 提示词必须是字符串")
                elif prompt:
                    arguments["prompt"] = prompt
                navigate = raw_args.get("navigate_after")
                if navigate is not None and not isinstance(navigate, bool):
                    blockers.append(f"第 {index} 步 navigate_after 必须是布尔值")
                elif navigate:
                    arguments["navigate_after"] = True
            elif action == "formation":
                formation = str(raw_args.get("formation") or "").strip().lower()
                leader = raw_args.get("leader_target_map_m")
                spacing = _finite_float(raw_args.get("spacing_m"))
                altitude = _finite_float(raw_args.get("altitude_m"))
                if formation not in {"line", "v", "diamond"}:
                    blockers.append(f"第 {index} 步队形必须是 line/v/diamond")
                else:
                    arguments["formation"] = formation
                if not isinstance(leader, (list, tuple)) or len(leader) != 3:
                    blockers.append(f"第 {index} 步需要领机目标坐标")
                else:
                    try:
                        arguments["leader_target_map_m"] = [
                            float(value) for value in leader
                        ]
                    except (TypeError, ValueError):
                        blockers.append(f"第 {index} 步领机目标无效")
                if spacing is None or not 0.5 <= spacing <= 50.0:
                    blockers.append(f"第 {index} 步间距必须在 0.5-50 米")
                else:
                    arguments["spacing_m"] = spacing
                if altitude is None or not 0.5 <= altitude <= 20.0:
                    blockers.append(f"第 {index} 步编队高度必须在 0.5-20 米")
                else:
                    arguments["altitude_m"] = altitude
            if ids:
                normalized.append(
                    {
                        "action": action,
                        "vehicle_ids": ids,
                        "arguments": arguments,
                    }
                )
        return normalized, blockers

    def _create_draft(
        self,
        session: _AgentSession,
        proposal: dict[str, Any] | None,
        context: dict[str, Any],
    ) -> _AgentDraft | None:
        if proposal is None:
            return None
        blockers, arguments, vehicle_id = self._action_gate(proposal, context)
        action = str(proposal.get("action") or "").strip().lower()
        if action == "return_home":
            action = "rtl"
        reason = str(proposal.get("reason") or "Mission Agent 草案").strip()
        now = datetime.now(timezone.utc)
        draft = _AgentDraft(
            draft_id=uuid4(),
            session_id=session.session_id,
            created_at_utc=now,
            expires_at_utc=now + timedelta(seconds=self.config.draft_ttl_s),
            expires_monotonic_s=self._clock() + self.config.draft_ttl_s,
            action=action,
            vehicle_id=vehicle_id,
            arguments=arguments,
            reason=reason[:256],
            blockers=blockers,
            context_hash=self._context_hash(context),
            context_summary=self._context_summary(context),
            status="blocked" if blockers else "pending_confirmation",
        )
        self._drafts[draft.draft_id] = draft
        return draft

    def _action_gate(
        self,
        proposal: dict[str, Any],
        context: dict[str, Any],
    ) -> tuple[list[str], dict[str, Any], int | None]:
        blockers: list[str] = []
        action = str(proposal.get("action") or "").strip().lower()
        if action == "return_home":
            action = "rtl"
        if not action and isinstance(proposal.get("plan"), list):
            action = "plan"
        plan_steps: list[dict[str, Any]] | None = None
        if action == "plan":
            raw_plan = proposal.get("plan")
            if not isinstance(raw_plan, list):
                raw_plan = None
            plan_steps = [item for item in (raw_plan or []) if isinstance(item, dict)]
            if not plan_steps:
                blockers.append("计划草案缺少 plan 步骤列表")
        elif action not in ATOMIC_AGENT_ACTIONS:
            blockers.append("Agent 只允许建议受支持的原子动作（arm/disarm/takeoff/hold/land/rtl/goto/formation）")

        current_vehicle_id = _integer(context.get("vehicle_id"))
        raw_vehicle_id = proposal.get("vehicle_id")
        if raw_vehicle_id is None:
            vehicle_id = current_vehicle_id
        else:
            vehicle_id = _integer(raw_vehicle_id)
            if vehicle_id is None:
                blockers.append("草案飞机编号无效")
        if current_vehicle_id is None:
            blockers.append("当前飞机编号不可用")
        elif vehicle_id is not None and vehicle_id != current_vehicle_id:
            blockers.append("草案飞机编号与当前地面站飞机不一致")

        raw_arguments = proposal.get("arguments")
        if not isinstance(raw_arguments, dict):
            raw_arguments = {}
        arguments: dict[str, Any] = {}
        if action == "takeoff":
            altitude = _finite_float(raw_arguments.get("altitude_m"))
            if altitude is None or not 0.5 <= altitude <= 20.0:
                blockers.append("起飞高度必须在 0.5 到 20 米之间")
            else:
                arguments["altitude_m"] = altitude
        elif action == "goto":
            position = raw_arguments.get("target_position_ned_m")
            if not isinstance(position, (list, tuple)) or len(position) != 3:
                blockers.append("GOTO 需要 target_position_ned_m（北、东、下三个数值）")
            else:
                values = [float(value) for value in position]
                if not all(math.isfinite(value) for value in values) or any(
                    abs(value) > 100_000.0 for value in values
                ):
                    blockers.append("GOTO 目标坐标超出有效范围")
                else:
                    arguments["target_position_ned_m"] = values
        elif action == "formation":
            formation = str(raw_arguments.get("formation") or "").strip().lower()
            leader = raw_arguments.get("leader_target_map_m")
            spacing = _finite_float(raw_arguments.get("spacing_m"))
            altitude = _finite_float(raw_arguments.get("altitude_m"))
            hold = _finite_float(raw_arguments.get("hold_s"))
            ids = raw_arguments.get("vehicle_ids")
            if formation not in {"line", "v", "diamond"}:
                blockers.append("队形必须是 line、v 或 diamond")
            if not isinstance(leader, (list, tuple)) or len(leader) != 3:
                blockers.append("编队需要 leader_target_map_m（北、东、下三个数值）")
            else:
                leader_values = [float(value) for value in leader]
                if not all(math.isfinite(value) for value in leader_values):
                    blockers.append("编队领机目标坐标无效")
            if spacing is None or not 0.5 <= spacing <= 50.0:
                blockers.append("编队间距必须在 0.5 到 50 米之间")
            if altitude is None or not 0.5 <= altitude <= 20.0:
                blockers.append("编队高度必须在 0.5 到 20 米之间")
            if hold is None or not 0.0 <= hold <= 120.0:
                blockers.append("编队保持时间必须在 0 到 120 秒之间")
            if not isinstance(ids, (list, tuple)) or not 2 <= len(ids) <= 8:
                blockers.append("编队需要 2 到 8 架飞机的 vehicle_ids")
            else:
                id_values = []
                for item in ids:
                    value = _integer(item)
                    if value is None or not 1 <= value <= 255:
                        blockers.append("编队 vehicle_ids 必须是 1 到 255 的整数")
                        break
                    id_values.append(value)
                if len(id_values) != len(set(id_values)):
                    blockers.append("编队 vehicle_ids 不能重复")
                if id_values:
                    arguments["vehicle_ids"] = id_values
            if formation in {"line", "v", "diamond"}:
                arguments["formation"] = formation
            if leader is not None and len(leader) == 3:
                try:
                    arguments["leader_target_map_m"] = [float(value) for value in leader]
                except (TypeError, ValueError):
                    blockers.append("编队领机目标坐标无效")
            if spacing is not None:
                arguments["spacing_m"] = spacing
            if altitude is not None:
                arguments["altitude_m"] = altitude
            if hold is not None:
                arguments["hold_s"] = hold
        elif action == "plan":
            steps, step_blockers = self._validate_plan_steps(
                plan_steps or [],
                context,
            )
            blockers.extend(step_blockers)
            if steps:
                arguments["steps"] = steps
        elif raw_arguments:
            blockers.append("该原子动作不接受参数")

        deployment_mode = str(context.get("deployment_mode") or "").lower()
        simulated = deployment_mode != "real"
        if not simulated:
            if context.get("agent_connected") is not True:
                blockers.append("机载代理未连接")
            if context.get("fcu_link_ok") is not True:
                blockers.append("飞控链路未就绪")
            if context.get("command_output_enabled") is not True:
                blockers.append("机载命令输出未启用")
            allowed = {
                str(item).lower() for item in context.get("allowed_commands", [])
            }
            if action and action not in allowed:
                blockers.append(f"{action} 不在当前机载白名单")
        if action in {"goto", "formation"} and not simulated:
            blockers.append("GOTO 与编队执行仅在仿真（SIM/DEMO）模式下可用")

        telemetry = context.get("telemetry")
        if not isinstance(telemetry, dict):
            telemetry = {}
        health = telemetry.get("health")
        if not isinstance(health, dict):
            health = {}
        armed = telemetry.get("armed")
        if action == "arm":
            if armed is True:
                blockers.append("飞机已经解锁")
            if health.get("prearm_ok") is not True:
                blockers.append("飞控预解锁检查未通过")
        elif action == "disarm":
            if armed is not True:
                blockers.append("飞机当前未解锁")
        elif action in {"hold", "land", "rtl"} and armed is not True:
            blockers.append("飞机未解锁，无需执行飞行中安全动作")
        elif action == "takeoff" and not simulated:
            if health.get("prearm_ok") is not True:
                blockers.append("飞控预解锁检查未通过")
            blockers.extend(self._gps_navigation_blockers(telemetry, health))
            georeference = context.get("georeference")
            if deployment_mode == "real" and (
                not isinstance(georeference, dict)
                or georeference.get("status") != "surveyed"
            ):
                blockers.append("实机自动起飞要求已测量场地标定")
        if action == "rtl" and not simulated:
            blockers.extend(self._gps_navigation_blockers(telemetry, health))
            if telemetry.get("home_position_deg_m") is None:
                blockers.append("Home 位置不可用")
        return _unique(blockers), arguments, vehicle_id

    @staticmethod
    def _gps_navigation_blockers(
        telemetry: dict[str, Any],
        health: dict[str, Any],
    ) -> list[str]:
        blockers = []
        if health.get("gps_healthy") is not True:
            blockers.append("GPS 未达到健康状态")
        fix_type = _integer(health.get("gps_fix_type"))
        if fix_type is None or fix_type < 3:
            blockers.append("GPS 定位未达到 3D Fix")
        hdop = _finite_float(health.get("gps_hdop"))
        if hdop is None or hdop > 2.5:
            blockers.append("GPS HDOP 不满足 2.5 门限")
        if health.get("ekf_ok") is not True:
            blockers.append("EKF 不允许 GPS 导航")
        if telemetry.get("local_position_ned_m") is None:
            blockers.append("LOCAL_NED 位置不可用")
        return blockers

    def _model_messages(
        self,
        session: _AgentSession,
        message: str,
        context: dict[str, Any],
    ) -> list[dict[str, str]]:
        context_text = json.dumps(
            context,
            ensure_ascii=False,
            allow_nan=False,
            sort_keys=True,
            separators=(",", ":"),
        )
        messages = [
            {"role": "system", "content": self._SYSTEM_PROMPT},
            {
                "role": "system",
                "content": f"当前地面站状态 JSON：{context_text}",
            },
        ]
        if session.execution_history:
            history_text = json.dumps(
                session.execution_history,
                ensure_ascii=False,
                allow_nan=False,
                sort_keys=True,
                separators=(",", ":"),
            )
            messages.append(
                {
                    "role": "system",
                    "content": (
                        "最近已执行的命令/计划结果 JSON（供你总结与决策，"
                        f"不得虚构）：{history_text}"
                    ),
                }
            )
        for turn in session.turns[-self.config.max_turns_per_session :]:
            messages.append({"role": turn.role, "content": turn.content})
        messages.append({"role": "user", "content": message})
        return messages

    def _session_payload(self, session: _AgentSession) -> dict[str, Any]:
        return {
            "session_id": str(session.session_id),
            "created_at_utc": session.created_at_utc.isoformat(),
            "updated_at_utc": session.updated_at_utc.isoformat(),
            "turns": [turn.public_payload() for turn in session.turns],
            "latest_draft": (
                self._draft_payload(self._drafts[session.latest_draft_id])
                if session.latest_draft_id in self._drafts
                else None
            ),
            "execution_policy": "operator_confirmation_required",
        }

    @staticmethod
    def _draft_payload(draft: _AgentDraft) -> dict[str, Any]:
        return {
            "draft_id": str(draft.draft_id),
            "session_id": str(draft.session_id),
            "created_at_utc": draft.created_at_utc.isoformat(),
            "expires_at_utc": draft.expires_at_utc.isoformat(),
            "action": draft.action,
            "vehicle_id": draft.vehicle_id,
            "arguments": draft.arguments,
            "reason": draft.reason,
            "blockers": list(draft.blockers),
            "context_hash": draft.context_hash,
            "context_summary": draft.context_summary,
            "status": draft.status,
            "executable": draft.executable,
            "confirmation_required": draft.status == "pending_confirmation",
            "execution_result": draft.execution_result,
        }

    @staticmethod
    def _context_summary(context: dict[str, Any]) -> dict[str, Any]:
        telemetry = context.get("telemetry")
        if not isinstance(telemetry, dict):
            telemetry = {}
        health = telemetry.get("health")
        if not isinstance(health, dict):
            health = {}
        return {
            "captured_at_utc": context.get("captured_at_utc"),
            "vehicle_id": context.get("vehicle_id"),
            "deployment_mode": context.get("deployment_mode"),
            "agent_connected": context.get("agent_connected"),
            "fcu_link_ok": context.get("fcu_link_ok"),
            "allowed_commands": context.get("allowed_commands", []),
            "armed": telemetry.get("armed"),
            "mode": telemetry.get("mode"),
            "battery_remaining": telemetry.get("battery_remaining"),
            "local_position_ned_m": telemetry.get("local_position_ned_m"),
            "gps_fix_type": health.get("gps_fix_type"),
            "gps_hdop": health.get("gps_hdop"),
            "gps_healthy": health.get("gps_healthy"),
            "prearm_ok": health.get("prearm_ok"),
            "ekf_ok": health.get("ekf_ok"),
        }

    @staticmethod
    def _context_hash(context: dict[str, Any]) -> str:
        encoded = json.dumps(
            context,
            ensure_ascii=False,
            allow_nan=False,
            sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8")
        return hashlib.sha256(encoded).hexdigest()

    def _require_session(self, session_id: UUID) -> _AgentSession:
        try:
            return self._sessions[session_id]
        except KeyError as exc:
            raise AgentSessionNotFound("Mission Agent 会话不存在或已失效") from exc

    def _require_draft(self, draft_id: UUID) -> _AgentDraft:
        try:
            return self._drafts[draft_id]
        except KeyError as exc:
            raise AgentDraftNotFound("Mission Agent 草案不存在或已失效") from exc

    def _remove_session(self, session_id: UUID) -> None:
        self._sessions.pop(session_id, None)
        for draft_id, draft in tuple(self._drafts.items()):
            if draft.session_id == session_id:
                self._drafts.pop(draft_id, None)


def _integer(value: Any) -> int | None:
    if isinstance(value, bool):
        return None
    try:
        result = int(value)
    except (TypeError, ValueError):
        return None
    return result if 1 <= result <= 255 else None


def _finite_float(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _unique(values: list[str]) -> list[str]:
    return list(dict.fromkeys(values))


def _uuid(value: UUID | str) -> UUID:
    if isinstance(value, UUID):
        return value
    try:
        return UUID(str(value))
    except (TypeError, ValueError) as exc:
        raise ValueError("会话或草案 ID 不是合法 UUID") from exc

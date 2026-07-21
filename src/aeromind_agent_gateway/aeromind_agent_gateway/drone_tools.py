"""Read-only Drone MCP tools for the first Agent Gateway iteration."""

from __future__ import annotations

import json
from typing import Any, Awaitable, Callable

from claude_agent_sdk import create_sdk_mcp_server, tool

from .ros_state import RosStateBridge
from .capability_registry import capability_catalog
from .skill_registry import build_skill, skill_catalog
from .workflow import validate_workflow, workflow_json_schema


def _text_result(value: object) -> dict:
    return {
        "content": [
            {
                "type": "text",
                "text": json.dumps(value, ensure_ascii=False),
            }
        ]
    }


ControlRequester = Callable[[str, dict[str, Any]], Awaitable[dict[str, Any]]]


def _world_query_schema():
    return {
        "type": "object",
        "properties": {
            "query_type": {
                "type": "string",
                "enum": ["current", "nearest", "history", "health"],
            },
            "object_id": {"type": "string"},
            "class_name": {"type": "string"},
            "dynamic_only": {"type": "boolean"},
            "confirmed_only": {"type": "boolean"},
            "max_distance_m": {"type": "number", "minimum": 0.0},
            "fresh_within_sec": {"type": "number", "minimum": 0.0},
            "limit": {"type": "integer", "minimum": 1, "maximum": 100},
            "reference_position_m": {
                "type": "object",
                "properties": {
                    "x": {"type": "number"},
                    "y": {"type": "number"},
                    "z": {"type": "number"},
                },
                "required": ["x", "y", "z"],
                "additionalProperties": False,
            },
        },
        "required": ["query_type"],
        "additionalProperties": False,
    }


def create_drone_mcp_server(
    ros_state: RosStateBridge,
    request_control: ControlRequester | None = None,
):
    @tool(
        "get_drone_state",
        "读取无人机解锁、飞行模式、电池、GPS、EKF、位置和速度。该工具只读，不执行控制。",
        {},
    )
    async def get_drone_state(_args):
        return _text_result(ros_state.drone_state())

    @tool(
        "get_perception_summary",
        "读取当前目标检测和自主避障状态摘要。该工具只读，不执行控制。",
        {},
    )
    async def get_perception_summary(_args):
        return _text_result(ros_state.perception_summary())

    @tool(
        "query_world_model",
        "按类别、ID、距离、动态状态和时效查询当前或历史语义对象。该工具只读，不执行控制。目标类别使用检测器类别名，例如 person、car。",
        _world_query_schema(),
    )
    async def query_world_model(args):
        return _text_result(await ros_state.query_world_model(args))

    @tool(
        "analyze_current_image",
        "调用 ROS /perception/analyze_image，让已配置的 VLM 分析最新前视 RGB 图像。用户要求分析、描述或查看当前画面时必须使用。",
        {
            "type": "object",
            "properties": {"prompt": {"type": "string"}},
            "required": ["prompt"],
            "additionalProperties": False,
        },
    )
    async def analyze_current_image(args):
        return _text_result(await ros_state.analyze_current_image(args["prompt"]))

    @tool(
        "get_system_snapshot",
        "读取无人机状态、里程计、自主避障和检测结果的完整只读快照。",
        {},
    )
    async def get_system_snapshot(_args):
        return _text_result(ros_state.snapshot())

    @tool(
        "list_skills",
        "读取当前 Gateway 已注册、可执行的模块化技能目录。",
        {},
    )
    async def list_skills(_args):
        return _text_result({"skills": skill_catalog()})

    @tool(
        "list_capabilities",
        "读取当前真实接入 ROS 的基础能力、接口、风险和执行方式。",
        {},
    )
    async def list_capabilities(_args):
        return _text_result({"capabilities": capability_catalog()})

    async def request(action: str, args: dict[str, Any]):
        if request_control is None:
            return _text_result(
                {"success": False, "message": "控制请求功能未配置"}
            )
        return _text_result(await request_control(action, args))

    @tool(
        "request_arm",
        "创建无人机解锁确认请求。只创建请求，用户确认前不会解锁。",
        {},
    )
    async def request_arm(_args):
        return await request("arm", {})

    @tool(
        "request_disarm",
        "创建无人机加锁确认请求。只创建请求，用户确认前不会加锁。",
        {},
    )
    async def request_disarm(_args):
        return await request("disarm", {})

    @tool(
        "request_takeoff",
        "创建起飞确认请求。高度单位为米，允许范围 1 到 30 米。用户确认前不会起飞。",
        {
            "type": "object",
            "properties": {
                "altitude": {"type": "number", "minimum": 1.0, "maximum": 30.0}
            },
            "required": ["altitude"],
            "additionalProperties": False,
        },
    )
    async def request_takeoff(args):
        return await request("takeoff", {"altitude": args["altitude"]})

    @tool(
        "request_safe_takeoff",
        "创建一次确认的安全起飞工作流：先执行确定性安全检查，通过后起飞。",
        {
            "type": "object",
            "properties": {
                "altitude": {"type": "number", "minimum": 1.0, "maximum": 30.0},
                "minimum_obstacle_distance": {
                    "type": "number",
                    "minimum": 0.5,
                    "maximum": 20.0,
                },
                "require_gps": {"type": "boolean"},
            },
            "required": ["altitude"],
            "additionalProperties": False,
        },
    )
    async def request_safe_takeoff(args):
        return await request("workflow", _safe_takeoff_workflow(args))

    @tool(
        "request_land",
        "创建降落确认请求。用户确认前不会降落。",
        {},
    )
    async def request_land(_args):
        return await request("land", {})

    @tool(
        "request_move",
        "创建相对移动确认请求。方向使用 forward/backward/left/right/up/down，距离单位为米。",
        {
            "type": "object",
            "properties": {
                "direction": {
                    "type": "string",
                    "enum": ["forward", "backward", "left", "right", "up", "down"],
                },
                "distance": {"type": "number", "minimum": 0.5, "maximum": 100.0},
            },
            "required": ["direction", "distance"],
            "additionalProperties": False,
        },
    )
    async def request_move(args):
        return await request(
            "move",
            {"direction": args["direction"], "distance": args["distance"]},
        )

    @tool(
        "request_return_home",
        "创建 PX4 原生 RTL 返航确认请求。用户确认前不会返航。",
        {},
    )
    async def request_return_home(_args):
        return await request("return_home", {})

    @tool(
        "request_hover",
        "创建取消自主目标并悬停的确认请求。用户确认前不会改变飞行任务。",
        {},
    )
    async def request_hover(_args):
        return await request("hover", {})

    @tool(
        "request_capture_image",
        "创建保存当前前视相机图像的确认请求。",
        {},
    )
    async def request_capture_image(_args):
        return await request("capture_image", {})

    @tool(
        "request_square_mission",
        "创建正方形轨迹组合任务确认请求。整套轨迹只确认一次。",
        {
            "type": "object",
            "properties": {
                "side_length": {"type": "number", "minimum": 1, "maximum": 50},
                "altitude": {"type": "number", "minimum": 1, "maximum": 30},
                "takeoff_if_needed": {"type": "boolean"},
                "land_after": {"type": "boolean"},
            },
            "required": ["side_length"],
            "additionalProperties": False,
        },
    )
    async def request_square_mission(args):
        workflow = build_skill(
            "flight.square",
            {
                "side_length": args["side_length"],
                "altitude": args.get("altitude", 10.0),
                "takeoff_if_needed": args.get("takeoff_if_needed", True),
                "land_after": args.get("land_after", False),
            },
        )
        return await request("workflow", workflow)

    @tool(
        "request_skill",
        "按注册名称创建模块化技能确认请求。先用 list_skills 获取名称和参数说明。",
        {
            "type": "object",
            "properties": {
                "name": {"type": "string"},
                "args": {"type": "object"},
            },
            "required": ["name", "args"],
            "additionalProperties": False,
        },
    )
    async def request_skill(args):
        return await request(
            "workflow", build_skill(str(args["name"]), dict(args.get("args") or {}))
        )

    @tool(
        "request_workflow",
        "使用基础动作动态组合任务，支持条件、依赖、循环块和失败策略；创建一次人工确认请求。",
        workflow_json_schema(),
    )
    async def request_workflow(args):
        return await request("workflow", validate_workflow(args))

    return create_sdk_mcp_server(
        name="drone",
        version="0.1.0",
        tools=[
            get_drone_state,
            get_perception_summary,
            query_world_model,
            analyze_current_image,
            get_system_snapshot,
            list_skills,
            list_capabilities,
            request_arm,
            request_disarm,
            request_takeoff,
            request_safe_takeoff,
            request_land,
            request_move,
            request_return_home,
            request_hover,
            request_capture_image,
            request_square_mission,
            request_skill,
            request_workflow,
        ],
    )


READ_ONLY_TOOL_NAMES = [
    "mcp__drone__get_drone_state",
    "mcp__drone__get_perception_summary",
    "mcp__drone__query_world_model",
    "mcp__drone__analyze_current_image",
    "mcp__drone__get_system_snapshot",
    "mcp__drone__list_skills",
    "mcp__drone__list_capabilities",
]


CONTROL_REQUEST_TOOL_NAMES = [
    "mcp__drone__request_arm",
    "mcp__drone__request_disarm",
    "mcp__drone__request_takeoff",
    "mcp__drone__request_safe_takeoff",
    "mcp__drone__request_land",
    "mcp__drone__request_move",
    "mcp__drone__request_return_home",
    "mcp__drone__request_hover",
    "mcp__drone__request_capture_image",
    "mcp__drone__request_square_mission",
    "mcp__drone__request_skill",
    "mcp__drone__request_workflow",
]


OPENAI_DRONE_TOOLS = [
    {
        "type": "function",
        "function": {
            "name": "get_drone_state",
            "description": "读取无人机状态、位置和速度，只读。",
            "parameters": {"type": "object", "properties": {}},
        },
    },
    {
        "type": "function",
        "function": {
            "name": "list_skills",
            "description": "读取当前已注册的模块化技能目录。",
            "parameters": {"type": "object", "properties": {}},
        },
    },
    {
        "type": "function",
        "function": {
            "name": "list_capabilities",
            "description": "读取当前真实接入 ROS 的基础能力目录。",
            "parameters": {"type": "object", "properties": {}},
        },
    },
    {
        "type": "function",
        "function": {
            "name": "get_perception_summary",
            "description": "读取二维目标检测、三维语义世界对象和自主避障摘要，只读。",
            "parameters": {"type": "object", "properties": {}},
        },
    },
    {
        "type": "function",
        "function": {
            "name": "query_world_model",
            "description": "按类别、ID、距离、动态状态和时效查询当前、最近或历史语义对象，只读。类别使用 person、car 等检测器类别名。",
            "parameters": _world_query_schema(),
        },
    },
    {
        "type": "function",
        "function": {
            "name": "analyze_current_image",
            "description": "调用 ROS /perception/analyze_image，使用已配置的 VLM 分析最新前视 RGB 图像。用户要求分析、描述或查看当前画面时必须调用。",
            "parameters": {
                "type": "object",
                "properties": {"prompt": {"type": "string"}},
                "required": ["prompt"],
                "additionalProperties": False,
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "get_system_snapshot",
            "description": "读取无人机系统完整只读快照。",
            "parameters": {"type": "object", "properties": {}},
        },
    },
    *[
        {
            "type": "function",
            "function": {
                "name": name,
                "description": description,
                "parameters": parameters,
            },
        }
        for name, description, parameters in (
            (
                "request_arm",
                "创建解锁确认请求，不会立即控制无人机。",
                {"type": "object", "properties": {}},
            ),
            (
                "request_disarm",
                "创建加锁确认请求，不会立即控制无人机。",
                {"type": "object", "properties": {}},
            ),
            (
                "request_takeoff",
                "创建起飞确认请求，高度单位为米。",
                {
                    "type": "object",
                    "properties": {
                        "altitude": {"type": "number", "minimum": 1, "maximum": 30}
                    },
                    "required": ["altitude"],
                    "additionalProperties": False,
                },
            ),
            (
                "request_safe_takeoff",
                "创建先做确定性安全检查、通过后起飞的一次确认工作流。",
                {
                    "type": "object",
                    "properties": {
                        "altitude": {"type": "number", "minimum": 1, "maximum": 30},
                        "minimum_obstacle_distance": {"type": "number", "minimum": 0.5, "maximum": 20},
                        "require_gps": {"type": "boolean"},
                    },
                    "required": ["altitude"],
                    "additionalProperties": False,
                },
            ),
            (
                "request_land",
                "创建降落确认请求。",
                {"type": "object", "properties": {}},
            ),
            (
                "request_move",
                "创建相对移动确认请求。",
                {
                    "type": "object",
                    "properties": {
                        "direction": {
                            "type": "string",
                            "enum": [
                                "forward",
                                "backward",
                                "left",
                                "right",
                                "up",
                                "down",
                            ],
                        },
                        "distance": {
                            "type": "number",
                            "minimum": 0.5,
                            "maximum": 100,
                        },
                    },
                    "required": ["direction", "distance"],
                    "additionalProperties": False,
                },
            ),
            (
                "request_return_home",
                "创建 PX4 RTL 返航确认请求。",
                {"type": "object", "properties": {}},
            ),
            (
                "request_hover",
                "创建悬停确认请求。",
                {"type": "object", "properties": {}},
            ),
            (
                "request_capture_image",
                "创建保存当前相机图像的确认请求。",
                {"type": "object", "properties": {}},
            ),
        )
    ],
    {
        "type": "function",
        "function": {
            "name": "request_square_mission",
            "description": "创建正方形轨迹组合任务确认请求。",
            "parameters": {
                "type": "object",
                "properties": {
                    "side_length": {"type": "number", "minimum": 1, "maximum": 50},
                    "altitude": {"type": "number", "minimum": 1, "maximum": 30},
                    "takeoff_if_needed": {"type": "boolean"},
                    "land_after": {"type": "boolean"},
                },
                "required": ["side_length"],
                "additionalProperties": False,
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "request_skill",
            "description": "按注册名称和参数创建模块化技能确认请求；应先调用 list_skills。",
            "parameters": {
                "type": "object",
                "properties": {
                    "name": {"type": "string"},
                    "args": {"type": "object"},
                },
                "required": ["name", "args"],
                "additionalProperties": False,
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "request_workflow",
            "description": "使用基础动作动态组合任务，支持条件、依赖、循环块和失败策略，并创建一次确认请求。",
            "parameters": workflow_json_schema(),
        },
    },
]


async def execute_openai_drone_tool(
    name: str,
    args: dict[str, Any],
    ros_state: RosStateBridge,
    request_control: ControlRequester | None,
) -> dict[str, Any]:
    if name == "get_drone_state":
        return ros_state.drone_state()
    if name == "get_perception_summary":
        return ros_state.perception_summary()
    if name == "query_world_model":
        return await ros_state.query_world_model(args)
    if name == "analyze_current_image":
        return await ros_state.analyze_current_image(str(args.get("prompt") or ""))
    if name == "get_system_snapshot":
        return ros_state.snapshot()
    if name == "list_skills":
        return {"skills": skill_catalog()}
    if name == "list_capabilities":
        return {"capabilities": capability_catalog()}
    if name == "request_square_mission":
        if request_control is None:
            return {"success": False, "message": "控制请求功能未配置"}
        workflow = build_skill(
            "flight.square",
            {
                "side_length": args.get("side_length"),
                "altitude": args.get("altitude", 10.0),
                "takeoff_if_needed": args.get("takeoff_if_needed", True),
                "land_after": args.get("land_after", False),
            },
        )
        return await request_control("workflow", workflow)
    if name == "request_skill":
        if request_control is None:
            return {"success": False, "message": "控制请求功能未配置"}
        workflow = build_skill(
            str(args.get("name") or ""), dict(args.get("args") or {})
        )
        return await request_control("workflow", workflow)
    if name == "request_safe_takeoff":
        if request_control is None:
            return {"success": False, "message": "控制请求功能未配置"}
        return await request_control("workflow", _safe_takeoff_workflow(args))
    if name == "request_workflow":
        if request_control is None:
            return {"success": False, "message": "控制请求功能未配置"}
        return await request_control("workflow", validate_workflow(args))
    actions = {
        "request_arm": ("arm", {}),
        "request_disarm": ("disarm", {}),
        "request_takeoff": ("takeoff", {"altitude": args.get("altitude")}),
        "request_land": ("land", {}),
        "request_move": (
            "move",
            {"direction": args.get("direction"), "distance": args.get("distance")},
        ),
        "request_return_home": ("return_home", {}),
        "request_hover": ("hover", {}),
        "request_capture_image": ("capture_image", {}),
    }
    if name not in actions:
        raise ValueError(f"不允许的工具: {name}")
    if request_control is None:
        return {"success": False, "message": "控制请求功能未配置"}
    action, action_args = actions[name]
    return await request_control(action, action_args)


def _safe_takeoff_workflow(args: dict[str, Any]) -> dict[str, Any]:
    altitude = float(args.get("altitude", 10.0))
    minimum = float(args.get("minimum_obstacle_distance", 2.0))
    require_gps = bool(args.get("require_gps", True))
    return validate_workflow(
        {
            "name": f"安全检查后起飞到 {altitude:.1f} 米",
            "steps": [
                {
                    "id": "safety-check",
                    "action": "safety_check",
                    "args": {
                        "require_gps": require_gps,
                        "check_takeoff_zone": True,
                        "minimum_obstacle_distance": minimum,
                    },
                },
                {
                    "id": "takeoff",
                    "action": "takeoff",
                    "args": {"altitude": altitude},
                    "depends_on": ["safety-check"],
                },
            ],
        }
    )

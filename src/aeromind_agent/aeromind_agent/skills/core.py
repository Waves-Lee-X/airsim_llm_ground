"""Core flight-control skills currently backed by ROS services."""

from .base import SkillSpec


SKILLS = [
    SkillSpec(
        name="StatusSkill",
        label="状态检查",
        intent="status",
        type="hard",
        risk_level="low",
        description="读取 /control/drone_state，汇总解锁、模式、电池、GPS、EKF 状态。",
        example_task="查询状态",
        tools=["get_drone_state"],
        enabled=True,
    ),
    SkillSpec(
        name="ArmSkill",
        label="解锁/加锁",
        intent="arm",
        type="hard",
        risk_level="medium",
        description="调用 /control/arm 执行解锁或加锁。",
        example_task="解锁",
        tools=["get_drone_state", "arm_drone"],
        enabled=True,
    ),
    SkillSpec(
        name="TakeoffSkill",
        label="安全起飞",
        intent="takeoff",
        type="hard",
        risk_level="medium",
        description="解析目标高度，检查飞控状态，然后调用 /control/takeoff。",
        example_task="起飞到10米",
        tools=["get_drone_state", "safety_check", "takeoff"],
        enabled=True,
    ),
    SkillSpec(
        name="LandSkill",
        label="降落",
        intent="land",
        type="hard",
        risk_level="medium",
        description="读取当前状态并调用 /control/land。",
        example_task="降落",
        tools=["get_drone_state", "land"],
        enabled=True,
    ),
]

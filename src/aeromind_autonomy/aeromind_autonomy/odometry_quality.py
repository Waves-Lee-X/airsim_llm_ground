"""Quality gates for PX4, VIO and SLAM nav_msgs/Odometry inputs."""

from __future__ import annotations

import math


def odometry_quality_issue(
    msg,
    *,
    max_position_variance: float,
    max_orientation_variance: float,
) -> str | None:
    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation
    velocity = msg.twist.twist.linear
    values = (
        position.x,
        position.y,
        position.z,
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
        velocity.x,
        velocity.y,
        velocity.z,
    )
    if not all(math.isfinite(float(value)) for value in values):
        return "里程计包含 NaN 或无穷值"
    quaternion_norm = math.sqrt(
        orientation.x**2
        + orientation.y**2
        + orientation.z**2
        + orientation.w**2
    )
    if quaternion_norm < 0.5 or quaternion_norm > 1.5:
        return f"里程计四元数模长异常: {quaternion_norm:.3f}"

    position_variances = _positive_diagonal(msg.pose.covariance, (0, 7, 14))
    orientation_variances = _positive_diagonal(msg.pose.covariance, (21, 28, 35))
    if position_variances and max(position_variances) > max_position_variance:
        return f"位置协方差过大: {max(position_variances):.3f}"
    if orientation_variances and max(orientation_variances) > max_orientation_variance:
        return f"姿态协方差过大: {max(orientation_variances):.3f}"
    return None


def _positive_diagonal(covariance, indexes):
    values = [float(covariance[index]) for index in indexes]
    return [value for value in values if math.isfinite(value) and value > 0.0]

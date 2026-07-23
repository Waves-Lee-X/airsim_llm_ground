"""Run a non-invasive readiness check before a recorded demonstration."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import time
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.request import urlopen


REQUIRED_TOPICS = {
    "/control/drone_state",
    "/sensor/odometry",
    "/sensor/camera/rgb/front_center",
    "/sensor/camera/depth/front_center",
    "/sensor/lidar/points",
    "/autonomy/status",
}


def _check(name: str, status: str, detail: str, source: str = "") -> dict[str, str]:
    return {"name": name, "status": status, "detail": detail, "source": source}


def fetch_json(url: str, timeout: float) -> tuple[dict[str, Any] | None, str]:
    try:
        with urlopen(url, timeout=timeout) as response:
            payload = json.loads(response.read().decode("utf-8"))
        return payload if isinstance(payload, dict) else None, ""
    except (HTTPError, URLError, TimeoutError, json.JSONDecodeError, OSError) as exc:
        return None, str(exc)


def ros_topics(timeout: float) -> tuple[set[str], str]:
    try:
        result = subprocess.run(
            ["ros2", "topic", "list"],
            capture_output=True,
            text=True,
            timeout=timeout,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return set(), str(exc)
    if result.returncode != 0:
        error_lines = [line.strip() for line in result.stderr.splitlines() if line.strip()]
        return set(), error_lines[-1] if error_lines else f"ros2 exit={result.returncode}"
    return {line.strip() for line in result.stdout.splitlines() if line.strip()}, ""


def evaluate_status(
    web_status: dict[str, Any] | None,
    web_error: str,
    camera_status: dict[str, Any] | None,
    camera_error: str,
    gateway_health: dict[str, Any] | None,
    gateway_error: str,
    topics: set[str],
    topic_error: str,
    *,
    require_yolo: bool,
    require_vlm: bool,
) -> list[dict[str, str]]:
    checks: list[dict[str, str]] = []
    checks.append(_environment_check(
        "LLM 配置",
        ("AEROMIND_LLM_API_KEY", "DEEPSEEK_API_KEY", "OPENAI_API_KEY"),
        required=True,
    ))
    checks.append(_environment_check(
        "VLM 配置",
        ("AEROMIND_VLM_API_KEY", "DASHSCOPE_API_KEY"),
        required=require_vlm,
    ))
    checks.append(_environment_check(
        "VLM 模型参数",
        ("AEROMIND_VLM_API_URL", "AEROMIND_VLM_MODEL"),
        required=require_vlm,
        require_all=True,
    ))

    if web_status is None:
        checks.append(_check("Web 控制台", "FAIL", web_error or "无法读取状态", "/api/status"))
    else:
        checks.append(_check("Web 控制台", "PASS", "8080 状态接口可访问", "/api/status"))
        flight = web_status.get("telemetry_meta") or {}
        checks.append(_boolean_check("飞控遥测", flight.get("state_fresh"), "数据实时", "数据缺失或过期", "/control/drone_state"))
        checks.append(_boolean_check("里程计", flight.get("odom_fresh"), "数据实时", "数据缺失或过期", "/sensor/odometry"))
        camera_ready = bool(camera_status and camera_status.get("success"))
        camera_detail = (
            f"画面 {camera_status.get('width', '--')}x{camera_status.get('height', '--')}"
            if camera_ready else camera_error or "尚未收到画面"
        )
        checks.append(_boolean_check("RGB 相机", camera_ready, camera_detail, camera_detail, "/api/camera"))
        checks.append(_boolean_check("深度相机", bool(web_status.get("depth")), "深度摘要已收到", "深度数据缺失", "/sensor/camera/depth/front_center"))
        pointcloud = web_status.get("pointcloud") or {}
        point_count = pointcloud.get("sampled_points", pointcloud.get("points", 0))
        if isinstance(point_count, list):
            point_count = len(point_count)
        pointcloud_ready = bool(pointcloud) and bool(point_count)
        health = web_status.get("perception_health") or {}
        lidar_health = health.get("pointcloud") or {}
        lidar_detail = (
            f"状态 {lidar_health.get('status', 'MISSING')}，"
            f"频率 {float(lidar_health.get('rate_hz') or 0.0):.2f} Hz，"
            f"年龄 {_format_age(lidar_health.get('age_s'))}"
        )
        checks.append(_boolean_check(
            "点云",
            pointcloud_ready,
            f"采样点数 {point_count}；{lidar_detail}",
            f"未收到有效 PointCloud2；{lidar_detail}",
            "/sensor/lidar/points",
        ))
        services = web_status.get("services") or {}
        unavailable = [name for name in ("arm", "takeoff", "land", "agent") if not services.get(name)]
        checks.append(_check(
            "控制服务",
            "PASS" if not unavailable else "FAIL",
            "关键服务全部就绪" if not unavailable else "未就绪: " + ", ".join(unavailable),
            "ROS 2 services",
        ))

    if gateway_health is None:
        checks.append(_check("Agent Gateway", "FAIL", gateway_error or "无法读取健康状态", "/health"))
    else:
        checks.append(_check("Agent Gateway", "PASS", "8090 健康接口可访问", "/health"))
        checks.append(_boolean_check(
            "Gateway ROS 桥接",
            gateway_health.get("ros_state_available"),
            "实时飞控状态可用",
            "Gateway 未收到实时飞控状态",
            "/control/drone_state",
        ))
        checks.append(_check(
            "飞书图片 VLM",
            "PASS" if gateway_health.get("feishu_vision_enabled") else "WARN",
            (
                f"已启用，模型 {gateway_health.get('vlm_model') or '--'}"
                if gateway_health.get("feishu_vision_enabled")
                else "未启用；不影响 Web/ROS 感知节点 VLM"
            ),
            "/health",
        ))

    if topic_error:
        checks.append(_check("ROS 话题图", "FAIL", topic_error, "ros2 topic list"))
    else:
        missing = sorted(REQUIRED_TOPICS - topics)
        checks.append(_check(
            "关键 ROS 话题",
            "PASS" if not missing else "FAIL",
            "关键话题全部存在" if not missing else "缺少: " + ", ".join(missing),
            "ros2 topic list",
        ))
        if require_yolo:
            checks.append(_boolean_check(
                "YOLO 检测话题",
                "/perception/detections" in topics,
                "检测话题存在",
                "检测话题不存在",
                "/perception/detections",
            ))
    return checks


def _environment_check(
    name: str,
    keys: tuple[str, ...],
    required: bool,
    require_all: bool = False,
) -> dict[str, str]:
    configured = [key for key in keys if os.environ.get(key, "").strip()]
    ready = len(configured) == len(keys) if require_all else bool(configured)
    if ready:
        detail = "参数已加载" if require_all else "凭据已加载（值已隐藏）"
        return _check(name, "PASS", detail, "environment")
    return _check(
        name,
        "FAIL" if required else "WARN",
        "未检测到凭据" if required else "未启用，本次允许降级",
        "environment",
    )


def _boolean_check(name: str, value: Any, ok: str, bad: str, source: str) -> dict[str, str]:
    return _check(name, "PASS" if bool(value) else "FAIL", ok if value else bad, source)


def _format_age(value: Any) -> str:
    try:
        return f"{float(value):.2f}s"
    except (TypeError, ValueError):
        return "--"


def build_report(checks: list[dict[str, str]]) -> dict[str, Any]:
    counts = {status: sum(item["status"] == status for item in checks) for status in ("PASS", "WARN", "FAIL")}
    return {
        "schema_version": 1,
        "generated_at": time.time(),
        "ready": counts["FAIL"] == 0,
        "summary": counts,
        "checks": checks,
    }


def default_output() -> Path:
    stamp = time.strftime("%Y%m%d-%H%M%S")
    return Path("missions") / "contest" / f"preflight-{stamp}.json"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="演示前检查 ROS、Web、Gateway 与模型配置")
    parser.add_argument("--web-url", default="http://127.0.0.1:8080")
    parser.add_argument("--gateway-url", default="http://127.0.0.1:8090")
    parser.add_argument("--timeout", type=float, default=4.0)
    parser.add_argument("--require-yolo", action="store_true")
    parser.add_argument("--require-vlm", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)

    web, web_error = fetch_json(args.web_url.rstrip("/") + "/api/status", args.timeout)
    camera, camera_error = fetch_json(args.web_url.rstrip("/") + "/api/camera", args.timeout)
    gateway, gateway_error = fetch_json(args.gateway_url.rstrip("/") + "/health", args.timeout)
    topics, topic_error = ros_topics(args.timeout)
    checks = evaluate_status(
        web, web_error, camera, camera_error, gateway, gateway_error, topics, topic_error,
        require_yolo=args.require_yolo,
        require_vlm=args.require_vlm,
    )
    report = build_report(checks)
    output = args.output or default_output()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")

    print("\n演示预检")
    for item in checks:
        print(f"[{item['status']:<4}] {item['name']}: {item['detail']}")
    summary = report["summary"]
    print(f"\nPASS={summary['PASS']} WARN={summary['WARN']} FAIL={summary['FAIL']}")
    print(f"证据文件: {output.resolve()}")
    return 0 if report["ready"] else 2


if __name__ == "__main__":
    raise SystemExit(main())

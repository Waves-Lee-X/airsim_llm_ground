"""Process entrypoint: ROS state bridge plus async Agent Gateway."""

from __future__ import annotations

import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor
import uvicorn

from .api import create_app
from .config import GatewayConfig
from .events import EventBus
from .feishu_adapter import FeishuAdapter
from .ros_state import RosStateBridge
from .session_manager import SessionManager
from .store import SessionStore


def main(args=None):
    config = GatewayConfig.from_env()
    rclpy.init(args=args)
    ros_state = RosStateBridge()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(ros_state)
    ros_thread = threading.Thread(
        target=executor.spin,
        name="agent-gateway-ros",
        daemon=True,
    )
    ros_thread.start()

    store = SessionStore(config.database_path)
    events = EventBus(store)
    sessions = SessionManager(config, store, events, ros_state)
    adapters = []
    if config.feishu_enabled:
        adapters.append(FeishuAdapter(config, sessions, store))
    app = create_app(config, sessions, ros_state, adapters=adapters)

    ros_state.get_logger().info(
        f"Agent Gateway: http://localhost:{config.port} "
        f"provider={config.default_provider} model={config.default_model} "
        f"db={config.database_path}"
    )
    if not config.access_token:
        ros_state.get_logger().warning(
            "AEROMIND_AGENT_TOKEN 未配置，仅适合本机开发环境"
        )
    if config.feishu_enabled:
        ros_state.get_logger().info(
            f"飞书长连接已启用，白名单用户数: "
            f"{len(config.feishu_allowed_open_ids)}"
        )
        ros_state.get_logger().info(
            f"飞书图片 VLM: {'已启用' if config.vlm_api_url and config.vlm_model else '未配置'}"
        )

    try:
        uvicorn.run(
            app,
            host=config.host,
            port=config.port,
            log_level="info",
            access_log=False,
        )
    finally:
        executor.shutdown(timeout_sec=2.0)
        ros_state.destroy_node()
        store.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

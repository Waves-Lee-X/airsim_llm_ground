# 无人机任务控制台前端

这是独立运行的 Web 地面站前端，不依赖 ROS 2 Python package。当前界面与
`aeromind_web` 内置页面保持同一功能版本，通过 HTTP/WebSocket 分别连接 ROS
网关和 Agent Gateway：

```text
Browser Ground Station
  ├─ HTTP + WebSocket -> ROS Gateway :8080 -> ROS 2 topic/service
  └─ WebSocket        -> Agent Gateway :8090 -> LLM / MCP / Skills
```

支持流式对话、模型切换、会话历史、MCP 工具调用事件、人工确认、工作流控制、
飞行状态、相机、深度点云、目标检测和任务报告。Agent Gateway 不可用时，对话会
自动回退到 ROS 网关的 Legacy Agent HTTP 接口。

## 本地仿真运行

终端 1：启动 ROS 主系统。

```bash
cd ~/aeromind_ws
source install/setup.bash
ros2 launch aeromind_bringup aeromind_px4.launch.py
```

终端 2：启动 ROS 网关和 Agent Gateway。

```bash
cd ~/aeromind_ws
source install/setup.bash
export DEEPSEEK_API_KEY="你的 DeepSeek API Key"
ros2 launch aeromind_bringup aeromind_web.launch.py \
  host:=0.0.0.0 port:=8080 agent_port:=8090 \
  agent_provider:=deepseek agent_model:=deepseek-chat
```

终端 3：启动独立前端。

```bash
cd ~/aeromind_ws/web/ground_station
python3 -m http.server 5173
```

浏览器打开：

```text
http://localhost:5173
```

默认 ROS 网关地址是：

```text
http://localhost:8080
```

前端会根据 ROS 网关地址自动推导 Agent Gateway 地址：同一主机的 `8090` 端口。
成功连接后，对话区域会显示当前 Provider，并通过 `/ws/agent` 接收逐段输出。

检查两个后端：

```bash
curl http://localhost:8080/api/status
curl http://localhost:8090/health
```

如果 ROS 网关运行在另一台电脑或真机伴随计算机上，在页面顶部“ROS 网关”输入：

```text
http://<ROS_GATEWAY_IP>:8080
```

## 真机迁移思路

真机或伴随计算机上运行：

```bash
ros2 launch aeromind_bringup aeromind_web.launch.py \
  host:=0.0.0.0 port:=8080 agent_port:=8090
```

地面站电脑只需要运行这个静态前端，或直接把本目录部署到任意 Web 服务器。

注意：

- 地面站电脑必须能访问真机/伴随计算机的 `8080` 和 `8090` 端口。
- 真机网络中需要允许 HTTP 访问，必要时配置防火墙。
- 非本机部署必须配置强随机 `AEROMIND_AGENT_TOKEN`，并在浏览器
  `localStorage` 中设置相同的 `aeromind_agent_token`。
- Web 前端不直接使用 ROS，只通过网关的 HTTP/WebSocket 接口通信。

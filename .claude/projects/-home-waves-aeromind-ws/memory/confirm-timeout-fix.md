---
name: confirm-timeout-fix
description: Web 确认执行超时问题 — 服务调用超时的根因与修复
metadata:
  type: feedback
---

# 确认执行超时问题修复

## 问题
Web 前端点击"确认执行"后，虽然无人机实际执行了起飞命令，但 UI 显示：
> 确认执行失败：自然语言任务: __aeromind_confirm__:token: 服务调用超时

## 根因
全链路有 3 层超时，底层的 AirSim 阻塞操作导致总耗时超过 8s 阈值：

```
Web前端 → web_console_node (8s 超时) → agent_node (10s 超时) → control_node
                                                                  ↑ AirSim 模式下
                                                                    takeoffAsync().join() 最长 10s
                                                                    moveToPositionAsync().join() 再 5s
                                                                    合计最长 ~15s
```

- `web_console_node.py` 中 `_call_service` 默认超时 **8s**，通过 `execute_task` 调用 Agent 服务时未单独指定更长超时
- `agent_node.py` 中 `_call_service` 默认超时 **10s**，同样不足以覆盖 AirSim 模式的耗时

因此 Web 控制台在 8s 后收到了 503 超时错误，而 Agent 和 Control 仍在继续执行，无人机最终完成起飞。

## 修复
1. **web_console_node.py**: `execute_task()` 调用 Agent 服务时，timeout_sec 从默认 8s 提升至 **30s**
2. **web_console_node.py**: 直接控制调用（arm/takeoff/land）分别提升至 **15s/30s/30s**
3. **agent_node.py**: `_call_service` 默认 timeout_sec 从 10s 提升至 **30s**，并改为参数化

## Why
下游的 Control 节点在 AirSim 模式下会阻塞等待飞行动作完成（`.join()`），
PX4 模式的 `offboard_takeoff` 虽然只约 1.3s，但考虑到未来更多复杂操作，
足够长的超时可避免类似问题反复出现。

## How to apply
已在 `web_console_node.py` 和 `agent_node.py` 中修改完毕。重新编译即可生效：
```bash
cd ~/aeromind_ws && source /opt/ros/humble/setup.bash
colcon build --packages-select aeromind_web aeromind_agent --symlink-install
```

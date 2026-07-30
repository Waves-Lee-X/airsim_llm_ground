# 10 Mission Agent 设计与调试

## 1. 当前结论

2026-07-30 已完成 AeroMind-APM Lite Mission Agent 软件闭环。它运行在 Windows
地面站，不运行在树莓派，也不要求 ROS、MAVROS 或机载大模型。当前能力包括：

- 连续多轮对话，并在每一轮重新读取当前飞机状态；
- 向模型提供裁剪后的遥测、P9/FCU 状态、机载白名单、视觉结果和场地标定状态；
- 生成单个原子飞行动作的工具草案；
- 由代码侧确定性门禁验证草案，禁止把模型 JSON 当作权限依据；
- 只有操作员在 Web 中再次确认后，才进入原有命令 ACK 和物理完成证据链。

这不是自主飞行 Agent。GOTO、编队、搜索、路径规划和寻物任务当前只能对话说明，
不能生成可执行工具草案。实机白名单仍只有 ARM/DISARM，本功能没有扩大权限。

## 2. 处理链

```text
操作员消息
  -> 地面站采集当前飞机上下文
  -> LLM 返回对话回复和 proposed_action
  -> JSON 规范化
  -> 代码侧动作、机号、参数和实时状态门禁
  -> blocked 或 pending_confirmation 草案
  -> 操作员人工确认
  -> 再次采集实时上下文并重新执行全部门禁
  -> 原有 BrowserGateway.issue_command()
  -> 地面受理、MAVLink ACK、物理完成证据
  -> Web 命令证据列表
```

模型只负责建议，不负责授权，也不能直接调用 `issue_command()`。草案默认 120 秒
过期，只能确认一次；确认、取消、过期或执行失败后都不能复用。

## 3. 模型可见状态

每轮请求会注入以下裁剪信息：

- 当前机号、运行模式和采集时间；
- 机载 Agent、FCU、P9 和命令输出状态；
- 当前机载命令白名单；
- ARM、模式、电池、位置、速度、Home 和飞行状态；
- GPS Fix、HDOP、pre-arm、EKF 和 FCU 健康状态；
- 最新一次视觉识别的结构化结果；
- `GeoReference` 标定版本、状态和配置哈希。

API 密钥、P9 共享密钥、完整历史帧和未裁剪内部对象不会提供给模型。

## 4. 工具和门禁

模型只允许建议六种原子动作：

| 动作 | Agent 参数 | 关键门禁 |
|---|---|---|
| `arm` | 无 | 当前未解锁、pre-arm 通过、链路和白名单允许 |
| `disarm` | 无 | 当前已解锁、链路和白名单允许 |
| `takeoff` | `altitude_m`，0.5-20 m | pre-arm、GPS、EKF、LOCAL_NED、实机场地已实测、白名单允许 |
| `hold` | 无 | 当前已解锁、链路和白名单允许 |
| `land` | 无 | 当前已解锁、链路和白名单允许 |
| `rtl` | 无 | 当前已解锁、GPS、EKF、LOCAL_NED、Home、白名单允许 |

此外还会检查草案机号必须与地面站当前飞机一致、机载代理在线、FCU 链路正常、
命令输出开关启用。任一条件变化都会在确认时阻断旧草案。

DEMO 模式不产生 MAVLink 输出，但仍检查草案机号以及 ARM 状态，避免界面演示出
现明显不一致。SITL 和 REAL 使用相同的链路、健康状态和动作门禁。

## 5. Web 操作

1. 启动地面站并打开 `http://127.0.0.1:8000/`。
2. 在相机下方切换到 `Mission Agent`。
3. 先查看飞机、链路、GPS 和权限四项实时状态。
4. 输入“当前飞机状态怎么样”等问题，可进行连续对话。
5. 输入“解锁三号机”后，先检查回复和工具草案，不会立即发送命令。
6. 只有草案为“等待人工确认”时，“人工确认”按钮才可用。
7. 在第二个确认对话框中核对机号、模式和动作；确认后查看右侧三层命令证据。
8. 点击循环箭头可清除内存会话并新建会话。

浏览器使用 `sessionStorage` 记住当前会话 ID；地面站进程重启后内存会话会失效，
页面会自动创建新会话。每个会话最多保留 24 轮，最多同时保存 16 个会话。

## 6. 配置与部署

Mission Agent 复用地面站现有 OpenAI 兼容模型配置：

```text
AEROMIND_VLM_API_URL=https://your-openai-compatible-endpoint/v1
AEROMIND_VLM_API_KEY=<local-secret>
AEROMIND_VLM_MODEL=qwen3-vl-plus
AEROMIND_LLM_MODEL=qwen3-vl-plus
```

配置仍放在 `%USERPROFILE%\.aeromind\vlm.env`，不写入 Git。树莓派无需上传新的
Agent 对话程序；机载端继续只负责 P9 协议、MAVLink、遥测和相机推流。

## 7. API

| 方法 | 路径 | 用途 |
|---|---|---|
| `GET` | `/api/agent/status` | 模型可用性、工具集合和会话策略 |
| `POST` | `/api/agent/sessions` | 新建内存会话 |
| `GET` | `/api/agent/sessions/{session_id}` | 恢复会话和最新草案 |
| `DELETE` | `/api/agent/sessions/{session_id}` | 关闭会话并删除所属草案 |
| `POST` | `/api/agent/sessions/{session_id}/messages` | 发送一轮对话 |
| `POST` | `/api/agent/drafts/{draft_id}/confirm` | 明确确认并进入现有命令链 |
| `POST` | `/api/agent/drafts/{draft_id}/cancel` | 取消待确认草案 |

旧 `/api/semantic/mission/parse` 接口仍保留为只读任务预览，Web 对话区已经切换到
`/api/agent/...`。对话响应中的 `flight_command_generated=false` 表示模型回复本身
没有生成飞控命令；只有后续独立确认接口才能尝试执行草案。

## 8. 验证与未完成项

```bash
cd ~/aeromind_ws/apm_lite
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src \
python3 -m pytest -q tests/test_mission_agent.py tests/test_web_assets.py
```

当前完整自动测试基线为 `245 passed`。仍待完成：

1. 使用真实模型进行多轮状态问答质量测试和异常返回统计；
2. 外场条件具备后，只按现有白名单做人工监督测试；
3. 在第 22 项外场基础飞行验收完成前，不开放 TAKEOFF/HOLD/LAND/RTL 实机权限；
4. GOTO、编队、搜索和路径规划必须先形成独立任务 Schema、仿真验收和安全门禁，
   不能直接加入本版 Agent 原子工具集合。

# 05 Web 地面站设计与接口

## 1. 运行位置和入口

Web 地面站由一个 FastAPI 进程和静态前端组成。实机模式运行在 Windows，因为地面
P9 通过 USB 枚举为 Windows `COM3`；仿真模式可以运行在 WSL。

主要文件：

```text
src/aeromind_apm_lite/ground/browser/app.py       FastAPI 和 BrowserGateway
src/aeromind_apm_lite/ground/browser/runtime.py   单链路与 hybrid 组合运行时
src/aeromind_apm_lite/ground/browser/camera.py    AirSim/RTSP 相机桥
src/aeromind_apm_lite/ground/browser/semantic.py  VLM 和只读任务解析
src/aeromind_apm_lite/ground/browser/mission_agent.py  对话与工具草案
web/index.html
web/app.js
web/styles.css
```

启动后访问 `http://127.0.0.1:8000/`。默认只绑定本机回环地址，不应直接暴露到公共
网络。

## 2. BrowserGateway 的职责

`BrowserGateway` 把不同模式包装为同一套 Web 能力：

- 读取公开配置、运行状态和遥测；
- 管理地面 P9 串口和重连；
- 将六种 Web 原子动作转换为 Lite `VehicleCommand`；
- 等待应用 ACK、MAVLink ACK 和物理完成；
- 管理 AirSim 或 RTSP 相机桥；
- 调用 VLM/LLM；
- 管理 Mission Agent 内存会话和确认票据；
- 保存 GeoReference 和轨迹证据；
- 把状态、命令和语义结果发布到浏览器 WebSocket。

浏览器不能直接访问 `COM3`，也拿不到每机共享密钥。所有写操作都必须进入
FastAPI 代码侧门禁。

## 3. 页面布局

当前页面分为三列：

| 区域 | 内容 |
|---|---|
| 左侧 | 飞机、模式、电量、GPS、LOCAL_NED、链路和 EKF/Home 状态 |
| 中间 | 16:9 相机、视觉识别、Mission Agent 对话与工具草案 |
| 右侧 | 飞行控制、NED 轨迹和三层命令证据 |

顶部显示地面站、P9、机载 Agent、FCU 和 ARM 状态。实机模式还可在“串口设置”中
枚举端口并临时重连；修改只影响当前进程，下次启动仍使用 PowerShell 参数。

混合模式的飞机选择器同时列出 `[仿真] SITL UAV 1` 和 `[实机] real-uav3`。切换时
页面先锁定按钮，再按所选 `vehicle_id` 重新读取状态、遥测和相机。WebSocket 中属于
其他机号的遥测不会覆盖当前页面。

## 4. REST API 分组

### 4.1 服务和状态

```text
GET /api/health
GET /api/config
GET /api/status?vehicle_id={vehicle_id}
GET /api/vehicles/{vehicle_id}/telemetry
GET /api/vehicle/telemetry
```

`/api/config` 只返回公开配置，不包含密钥。`/api/status` 是 Web 判断控件是否可用的
主要来源，包括 runtime、P9 统计、Agent/FCU、白名单、相机和语义状态。

### 4.2 串口

```text
GET /api/serial/ports
PUT /api/serial/config
```

只在包含 `real_serial` 链路的模式有效。重配时后端只停止旧串口服务器，再按新端口
启动；`hybrid` 模式下不会停止 SITL。串口打开失败时实机写控件保持锁定。

### 4.3 相机和视觉语义

```text
GET  /api/camera/status?vehicle_id={vehicle_id}
GET  /api/camera/frame?vehicle_id={vehicle_id}
GET  /api/semantic/status
GET  /api/semantic/latest
POST /api/semantic/vision/analyze
POST /api/semantic/mission/parse
```

`/api/camera/frame` 返回所选机号的最新 JPEG，并设置 `Cache-Control: no-store`、帧
序号和采集时间头。混合模式下 UAV1 对应 AirSim RGB、UAV3 对应 RTSP RGB。旧任务
解析接口固定 `preview_only`，不会生成飞控命令。

### 4.4 Mission Agent

```text
GET    /api/agent/status
POST   /api/agent/sessions
GET    /api/agent/sessions/{session_id}
DELETE /api/agent/sessions/{session_id}
POST   /api/agent/sessions/{session_id}/messages
POST   /api/agent/drafts/{draft_id}/confirm
POST   /api/agent/drafts/{draft_id}/cancel
```

对话接口只产生回复和草案。确认是独立请求，且确认时重新读取飞机状态执行全部门禁。
页面会把当前 `vehicle_id` 连同消息传给 Agent，使模型上下文对应当前选择的仿真机或
实机；确认草案时再次使用草案机号检查，不能借切换页面改变目标。

### 4.5 场地和轨迹证据

```text
GET  /api/georeference
PUT  /api/georeference
GET  /api/trajectory/evidence
POST /api/trajectory/evidence
GET  /api/trajectory/evidence/{evidence_id}
POST /api/trajectory/reports
```

这些接口处理配置和文件，不直接发送飞行命令。

### 4.6 飞行命令

```text
POST /api/vehicles/{vehicle_id}/commands/{action}
POST /api/control/{control_action}   # 兼容入口
```

`action` 只接受 `arm/disarm/takeoff/hold/land/rtl`。实机模式会检查机号、连接、总开关
和机载心跳白名单；不能通过直接调用 API 绕过页面禁用状态。

## 5. WebSocket 事件

浏览器连接 `/ws` 后先收到 `snapshot`，随后接收：

| 事件 | 内容 |
|---|---|
| `status` / `vehicle_telemetry` | 服务与飞机状态 |
| `command_progress` | 应用、MAVLink、物理阶段进度 |
| `command_result` | 最终命令证据 |
| `visual_semantic_result` | VLM 结构化结果和模型原文 |
| `mission_parse_result` | 旧只读任务预览 |
| `mission_agent_reply` | 对话回复和工具草案 |
| `mission_agent_draft_updated` | 草案取消等状态变化 |
| `mission_agent_execution` | 人工确认后的执行结果 |
| `georeference_updated` | 场地标定版本变化 |

事件队列有界。最新状态可以覆盖旧状态，命令最终结果会保存在网关和右侧证据列表中。

## 6. 页面控制门禁

实机飞行按钮可用必须同时满足：

```text
API 在线
AND 机载 Agent 在线
AND FCU 就绪
AND command_output_enabled=true
AND action 位于 allowed_commands
```

页面禁用只是第一层用户体验保护，后端和树莓派仍会重复检查。页面状态闪烁时不要
连续点击，应先查看底部 P9 统计和 `/api/status`。

## 7. 只读调试命令

PowerShell 查看状态：

```powershell
Invoke-RestMethod http://127.0.0.1:8000/api/status |
  ConvertTo-Json -Depth 8
```

检查相机：

```powershell
Invoke-RestMethod http://127.0.0.1:8000/api/camera/status |
  ConvertTo-Json -Depth 5
Invoke-WebRequest http://127.0.0.1:8000/api/camera/frame `
  -OutFile "$env:TEMP\aeromind-frame.jpg"
```

检查 Agent 能力，不发送消息：

```powershell
Invoke-RestMethod http://127.0.0.1:8000/api/agent/status |
  ConvertTo-Json -Depth 5
```

## 8. 启动、停止和换端口

实机默认启动：

```powershell
Set-Location "\\wsl.localhost\Ubuntu-22.04\home\waves\aeromind_ws\apm_lite"
powershell -ExecutionPolicy Bypass -File .\deploy\windows\start-ground-uav3.ps1
```

端口已占用时先确认所属进程：

```powershell
Get-NetTCPConnection -LocalPort 8000 -State Listen |
  Select-Object LocalAddress,LocalPort,OwningProcess
```

只停止确认属于当前地面站的 PID：

```powershell
Stop-Process -Id <PID>
```

或者用另一个 Web 端口：

```powershell
.\deploy\windows\start-ground-uav3.ps1 -WebPort 8010
```

虚实同站启动：

```powershell
.\deploy\windows\start-ground-hybrid-uav3.ps1 `
  -SerialPort COM3 -SerialBaud 57600 -WebPort 8000
```

该脚本要求 SITL 已按 07 的说明把 UDP 发往 Windows 14550。不能同时再启动单独的
`start-ground-uav3.ps1`，否则 COM3 和 Web 端口会发生独占冲突。

## 9. 修改前端后的验证

```powershell
& 'C:\Users\16401\.cache\codex-runtimes\codex-primary-runtime\dependencies\node\bin\node.exe' `
  --check '\\wsl.localhost\Ubuntu-22.04\home\waves\aeromind_ws\apm_lite\web\app.js'
```

```bash
cd ~/aeromind_ws/apm_lite
PYTHONPATH=src python3 -m pytest -q tests/test_web_assets.py tests/test_browser_gateway.py
```

继续阅读：[06 树莓派机载端设计](06-树莓派机载端设计.md)。

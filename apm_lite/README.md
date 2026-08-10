# AeroMind-APM Lite

AeroMind-APM Lite 是一套面向 ArduPilot、树莓派机载电脑和 P9 数传的非 ROS
无人机系统。现有 `src/` 下的 ROS 2/PX4 工程继续作为原型和能力参考，Lite 不依赖
ROS 2、MAVROS、`px4_msgs` 或 PX4 Offboard。

完整开发路线见
[`doc/14-AeroMind-APM-Lite开发计划.md`](../doc/14-AeroMind-APM-Lite开发计划.md)，
专题资料见 [`docs/README.md`](docs/README.md)。

## 当前状态

| 阶段 | 状态 | 结果 |
|---|---|---|
| M0 协议与边界 | 已通过 | 严格 Schema、HMAC、TTL、会话、重放保护和非 ROS 依赖门禁 |
| M1 自动仿真 | 已通过 | AirSim 1.8.1 + ArduCopter 4.7.0，固定任务 10/10，独立 RTL 通过 |
| M1.5 手动仿真 | 已通过 | 非 Blocks 场景、手动启动 UE/SITL、Web 控制、相机、LAND/RTL 闭环 |
| M2 UAV3 基础链路 | 已打通 | V5+、树莓派、P9、D435i RGB、RTSP、Web 和 VLM 在线 |
| M2 真机命令台架 | 待人工完成 | 机载只允许 ARM/DISARM；TAKEOFF/HOLD/LAND/RTL 继续禁用 |
| Mission Agent | 任务闭环完成 | 连续对话、多步 plan 草案、MissionPlanner 按序执行、执行反馈回环、视觉 analyze 步骤（VLM 识别结果回写决策） |
| M5 编队（仿真） | 已验收 | SITL 10/10（一字/V/正方形）；地面站编队执行（序列/分层切换/取消）；态势地图 |
| M1 升级最小版 | 已实现 | 统一孪生契约与注册表；L0 256 对象/32 分支、四策略、Pareto、可校验 EvidencePackage |
| 深度避障与多机任务 | 真机待验收 | 编队仿真闭环完成；D435i Depth/IR、路径规划与真机编队尚未进入实机授权 |

2026-07-31 后真机已关机，当前工作全部在四机仿真（SITL + AirSim）中进行；
以下为 2026-07-30 记录的真机在线状态：

- 树莓派 `colony3`，当前网络地址 `192.168.1.110`；
- FCU `/dev/ttyAMA0 @ 921600`，固件标识 `4.4.1-255`；
- 机载 P9 `/dev/ttyAMA1 @ 57600`，地面 P9 `COM3 @ 57600`；
- D435i RGB 经 RTSP 到达 Web，Depth、双 IR 和 IMU 尚未接入 Lite；
- 机载心跳上报 `command_output_enabled=true` 和
  `allowed_commands=["arm", "disarm"]`；
- VLM/LLM 使用地面端兼容接口；Mission Agent 只能生成受控草案，实机权限仍由
  人工确认、地面门禁和机载白名单共同决定。

## 系统架构

```text
飞行控制与遥测

Web 浏览器
  -> Windows FastAPI 地面站
  -> COM3 / 地面 P9
  -> 机载 P9 /dev/ttyAMA1
  -> 树莓派 OnboardAgent
  -> /dev/ttyAMA0
  -> CUAV V5+ / ArduCopter

视频与视觉语义

D435i RGB
  -> 树莓派 FFmpeg + EasyDarwin RTSP
  -> WiFi
  -> Windows OpenCV 最新帧桥
  -> Web 画面 / VLM 分析 / 任务语义预览
```

P9 只承载认证、命令、心跳、遥测和命令证据，不能承载视频。树莓派只运行轻量
机载代理和视频推流，不运行 ROS、MAVROS 或大模型。

## 目录说明

```text
apm_lite/
  configs/       仿真与实机配置
  deploy/        Windows、树莓派、systemd 和离线安装脚本
  docs/          架构、验收、部署和台架记录
  schemas/       自动导出的协议 JSON Schema
  src/           Python 源码
  tests/         协议、MAVLink、P9、Web、相机和 VLM 测试
  web/           Lite Web 地面站静态资源
```

## 手动 AirSim/SITL

每次 WSL 重启后重新生成 `settings.json`：

```bash
cd ~/aeromind_ws/apm_lite
PYTHONPATH=src python3 -m \
  aeromind_apm_lite.ground.simulation.manual_settings \
  --output /mnt/c/Users/<Windows用户名>/Documents/AirSim/settings.json
```

生成器只写配置并打印匹配的 SITL 命令，不会启动或结束 UE、AirSim、SITL。
按照 [`docs/07-仿真环境启动与调试.md`](docs/07-仿真环境启动与调试.md) 的顺序启动后，运行：

```bash
PYTHONPATH=src python3 -m aeromind_apm_lite.ground.browser.app \
  --mode sitl --host 127.0.0.1 --port 8000
```

打开 `http://127.0.0.1:8000/`。仅检查 UI 时使用 `--mode demo`，DEMO 命令不会
进入 MAVLink。

## 仿真与 UAV3 同站

虚实同站时地面站运行在 Windows，以便同时访问 `COM3` 和 WSL 发来的 UDP。先在
WSL 生成面向 Windows 的 SITL 命令：

```bash
cd ~/aeromind_ws/apm_lite
PYTHONPATH=src python3 -m \
  aeromind_apm_lite.ground.simulation.manual_settings \
  --output /mnt/c/Users/16401/Documents/AirSim/settings.json \
  --ground-on-windows
```

启动 UE Play，并执行生成器输出的 `sitl_command`。然后在 Windows PowerShell：

```powershell
Set-Location "\\wsl.localhost\Ubuntu-22.04\home\waves\aeromind_ws\apm_lite"
powershell -ExecutionPolicy Bypass `
  -File .\deploy\windows\start-ground-hybrid-uav3.ps1
```

Web 车辆选择器会同时列出仿真 UAV1 和实机 UAV3。两条链按不同机号隔离；仿真命令
权限不会扩展实机白名单。完整流程见
[`docs/07-仿真环境启动与调试.md`](docs/07-仿真环境启动与调试.md)。

顶部“场地标定”用于维护 WGS84/map/LOCAL_NED/AirSim 的统一坐标配置、版本哈希
和往返报告。无室外测量数据时必须保持草稿状态，详见
[`docs/11-场地标定与坐标转换.md`](docs/11-场地标定与坐标转换.md)。

统一 `GUIDED -> ARM -> TAKEOFF -> GOTO -> HOLD -> LAND` 状态机和仿真故障注入
命令见 [`docs/12-导航状态机与故障注入.md`](docs/12-导航状态机与故障注入.md)。
该入口只接受 SIM/UDP 配置，不会连接实机串口。

`planned/predicted/observed` 版本化轨迹、SITL 自动记录、真机 GPS CSV 回放、虚实
误差指标和非飞控证据 API 见
[`docs/13-轨迹证据与虚实误差.md`](docs/13-轨迹证据与虚实误差.md)。轨迹证据接口不会发送
飞行命令。

## UAV3 实机地面站

Windows 启动命令：

```powershell
Set-Location "\\wsl.localhost\Ubuntu-22.04\home\waves\aeromind_ws\apm_lite"
powershell -ExecutionPolicy Bypass -File .\deploy\windows\start-ground-uav3.ps1
```

打开 `http://127.0.0.1:8000/`。启动脚本默认使用 `COM3 @ 57600` 和
`rtsp://192.168.1.110:15544/cam`；网络或串口变化时使用 `-SerialPort`、
`-SerialBaud`、`-RtspUrl` 覆盖。Web 顶部的串口设置只影响当前地面站进程。

停止当前地面站：

```powershell
Get-NetTCPConnection -LocalPort 8000 -State Listen |
  ForEach-Object { Stop-Process -Id $_.OwningProcess }
```

树莓派永久服务：

```bash
sudo systemctl status aeromind-apm-lite@3.service --no-pager
sudo journalctl -u aeromind-apm-lite@3.service -n 100 --no-pager
sudo systemctl restart aeromind-apm-lite@3.service
```

真机当前只能在拆桨、固定机体、现场清空并保留遥控器接管的条件下测试 ARM 和
DISARM。不得通过修改 Web、直接调用 API 或关闭 ArduPilot `ARMING_CHECK` 绕过
白名单和飞控预检。

## VLM 配置

模型只在 Windows 地面端调用。将以下变量写入
`%USERPROFILE%\.aeromind\vlm.env`，不得把真实密钥写入 Git、文档或命令记录：

```text
AEROMIND_VLM_API_URL=https://your-openai-compatible-endpoint/v1
AEROMIND_VLM_API_KEY=<local-secret>
AEROMIND_VLM_MODEL=qwen3-vl-plus
AEROMIND_LLM_MODEL=qwen3-vl-plus
```

处理链为：

```text
最新 RGB 帧 -> VLM -> 结构化结果与原始回复 -> Web 展示
自然语言消息 + 实时飞机状态 -> Mission Agent 回复 -> 受控工具草案
受控工具草案 -> 代码门禁 -> 人工确认 -> 原有命令证据链
```

视觉结果本身不会进入飞行命令队列。旧任务解析接口继续保持
`execution_policy=preview_only`；Mission Agent 仅允许六种原子动作，并在人工确认时
重新检查实时状态和机载白名单。详细说明见
[`docs/10-Mission-Agent设计与调试.md`](docs/10-Mission-Agent设计与调试.md)。

## 本地验证

```bash
cd ~/aeromind_ws/apm_lite
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q tests
PYTHONPATH=src python3 tools/export_schemas.py --check
python3 -m flake8 --max-line-length=101 --extend-ignore=E203,W503 src tests
```

当前 Lite 完整测试为 `334 passed`。提交前还应运行 JavaScript 语法检查和
`git diff --check`。

M1 多保真平行推演入口：

```bash
PYTHONPATH=src python3 -m aeromind_apm_lite.ground.deduction.cli demo \
  --vehicles 256 --branches 32 --workers 4 \
  --output /tmp/aeromind-m1-run.json \
  --evidence /tmp/aeromind-m1-evidence.zip
PYTHONPATH=src python3 tools/export_evidence.py verify /tmp/aeromind-m1-evidence.zip
```

设计、指标口径和当前边界见 [`docs/18-M1多保真平行推演与证据包.md`](docs/18-M1多保真平行推演与证据包.md)。

## 安全边界

- 实机命令必须同时通过地面 API 白名单和机载白名单。
- `COMMAND_ACK=ACCEPTED` 不代表动作物理完成，必须继续检查新鲜遥测。
- D435i RGB 在线不代表具备深度避障能力。
- 没有完成场地标定、定位、RC/failsafe 和多机最小间距验收前，不得实飞编队。
- VLM/LLM 只提供语义结果和工具草案，不直接控制飞控；Mission Agent 确认接口也
  不能绕过地面门禁、机载白名单或物理完成证据。

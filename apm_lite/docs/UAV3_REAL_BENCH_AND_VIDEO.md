# UAV3 实机基础链路、视频与语义联调记录

首次记录：2026-07-29

最近更新：2026-07-30

状态：FCU、P9、D435i RGB、RTSP、Web 地面站和 VLM 语义链路已打通；最新帧
低延迟桥与永久服务已启用。机载命令白名单仅允许 ARM/DISARM，软件门禁已验收，
人工 ARM/DISARM 动作尚未执行；深度/IR、其他飞行命令和实飞均未授权。

## 1. 本次完成范围

本次联调对象为三号机 `colony3`。已经现场确认：

- Raspberry Pi 4B 运行 Raspberry Pi OS 11 Bullseye 和 Python 3.9.2；
- 飞控 MAVLink `SYSID=3`，固件版本上报为 `4.4.1-255`；
- Pi `/dev/ttyAMA0 @ 921600` 连接飞控；
- Pi `/dev/ttyAMA1 @ 57600` 连接机载 P9 Radio；
- Windows 地面 P9 枚举为 CP210x `COM3 @ 57600`；
- Intel RealSense D435i 的 RGB 节点为 `/dev/video4`；
- RGB 当前使用 `424x240 @ 15 FPS`，编码输出为 H.264；
- 当前 D435i 通过 USB 2.0 `480 Mb/s` 接入；
- Web 地面站运行在 `http://127.0.0.1:8000/`。

联调先以 `command_output_enabled=false` 完成 60 秒只读验收。2026-07-30 在操作者
再次确认拆桨、固定机体和现场清空后，部署 ARM/DISARM 精确白名单：

```text
command_output_enabled=true
allowed_commands=["arm", "disarm"]
```

Web 只启用解锁和上锁；起飞、悬停、降落和返航继续禁用。部署与重启过程中没有
发送任何飞行命令。

## 2. 双链路架构

飞控消息和视频使用两条独立链路：

```text
飞控/任务链路

CUAV V5+
  -> /dev/ttyAMA0 @ 921600
  -> Raspberry Pi AeroMind onboard agent
  -> /dev/ttyAMA1 @ 57600
  -> 机载 P9 Radio
  -> 地面 P9 Radio
  -> Windows COM3 @ 57600
  -> AeroMind Web ground backend

视频链路

D435i RGB /dev/video4 @ 424x240, 15 FPS
  -> Raspberry Pi FFmpeg H.264 encoder
  -> EasyDarwin RTSP server
  -> WiFi rtsp://<pi-ip>:15544/cam
  -> Windows RTSP camera bridge
  -> GET /api/camera/frame
  -> Web camera panel
```

P9 只承载认证、命令、心跳、遥测和任务证据。不能通过 57600 波特率的 P9
传输视频。视频依赖 Pi 与地面电脑之间的 IP 网络。

## 3. 树莓派运行程序

Pi 当前需要两个独立服务。

### 3.1 Lite 机载代理

入口：

```text
python -m aeromind_apm_lite.onboard.runtime
```

职责：

- 独占 `/dev/ttyAMA0`，读取飞控 MAVLink；
- 独占 `/dev/ttyAMA1`，运行 Lite P9 协议；
- 上报 FCU、GPS、模式、电池和本地速度；
- 执行认证、会话、序列号、TTL 和重放保护；
- 默认拒绝所有实际飞控命令输出。

早期只读台架曾使用临时 `aeromind-apm-lite-bench.service`，现已停用。当前正式
服务为：

```text
aeromind-apm-lite@3.service: enabled, active
ed_rtsp.service: active
```

检查和重启：

```bash
sudo systemctl status aeromind-apm-lite@3.service --no-pager
sudo journalctl -u aeromind-apm-lite@3.service -n 100 --no-pager
sudo systemctl restart aeromind-apm-lite@3.service
```

### 3.2 D435i RGB 推流服务

仓库脚本：

```text
deploy/raspberry-pi/ed_rtsp.py
```

Pi 安装位置：

```text
/home/lenovo3/stream/ed_rtsp.py
```

脚本负责：

- 启动并监视 EasyDarwin；
- 等待 `/dev/video4` 出现；
- 启动 FFmpeg，将 RGB 编码为 H.264；
- 相机 USB 重枚举或 FFmpeg 退出后自动重新推流；
- 收到 `SIGTERM` 时结束 FFmpeg 和 EasyDarwin 子进程。

检查命令：

```bash
systemctl status ed_rtsp.service --no-pager
journalctl -u ed_rtsp.service -n 100 --no-pager
ffprobe -rtsp_transport tcp -v error \
  -select_streams v:0 \
  -show_entries stream=codec_name,width,height,r_frame_rate \
  -of default=noprint_wrappers=1 \
  rtsp://127.0.0.1:15544/cam
```

预期 FFprobe 结果：

```text
codec_name=h264
width=424
height=240
r_frame_rate=15/1
```

## 4. Windows Web 地面站

运行环境：

```text
C:\Users\<user>\.aeromind\venv
```

运行 RGB 视频桥需要 `opencv-python-headless`。开发测试还需要 `pytest` 和
`pymavlink`：

```powershell
& "$env:USERPROFILE\.aeromind\venv\Scripts\python.exe" -m pip install `
  "opencv-python-headless>=4.8,<5"
```

使用仓库脚本启动：

```powershell
Set-Location "\\wsl.localhost\Ubuntu-22.04\home\waves\aeromind_ws\apm_lite"
powershell -ExecutionPolicy Bypass -File .\deploy\windows\start-ground-uav3.ps1
```

脚本默认值：

```text
P9 serial: COM3 @ 57600
Web:       http://127.0.0.1:8000/
RTSP:      rtsp://192.168.1.109:15544/cam
```

Pi IP 改变时，通过参数覆盖：

```powershell
.\deploy\windows\start-ground-uav3.ps1 `
  -SerialPort COM3 `
  -SerialBaud 57600 `
  -RtspUrl "rtsp://<new-pi-ip>:15544/cam"
```

凭据只从本机文件和环境变量加载。不得把共享密钥写入命令记录、文档或 Git。

## 5. OpenCV 在当前链路中的作用

浏览器不能直接显示 `rtsp://`。当前 OpenCV 只运行在 Windows 地面电脑，作用
是：

1. 连接并解码 Pi 输出的 RTSP/H.264；
2. 将解码帧转换为 JPEG；
3. 缓存最新帧并通过 `/api/camera/frame` 提供给浏览器；
4. 为后续颜色检测、箱体编号识别、YOLO 和 VLM 提供像素矩阵。

Pi 不需要安装 OpenCV 才能完成当前 RGB 推流；Pi 使用 FFmpeg 编码。

OpenCV 不是唯一选择。后续如果只追求最低显示延迟，可以使用 FFmpeg 管道或
WebRTC；但颜色识别和 VLM 前处理仍需要某种解码后的图像帧接口。

## 6. FCU 在线状态闪烁

### 6.1 根因

飞控 UART 本身没有反复断开。P9 链路存在 CRC、重复帧和 ACK 重传，旧实现只
在发送队列完全装满时才淘汰遥测，因此旧心跳和旧遥测会在队列里排队。地面站
收到旧状态前，现有遥测先超过 TTL，Web 就会短暂显示 FCU 离线。

### 6.2 已实施修复

- `BoundedLatestQueue.put()` 增加 `replacement_key`；
- 心跳使用 `replacement_key="heartbeat"`；
- 遥测使用 `replacement_key="telemetry"`；
- 同一类旧状态立即删除，新状态追加到队尾；
- 追加到队尾保证协议序列号不会发生倒序；
- 遥测显示 TTL 调整为 15 秒；
- 握手、命令和任务证据保持可靠 ACK；
- 可替换心跳和遥测按周期刷新，不做 ACK 重传。

15 秒 TTL 是针对当前 P9 实测空窗的显示容忍值。它不等于飞控可以脱离本地
失控保护运行 15 秒。RC 接管、ArduPilot failsafe 和机载安全逻辑必须独立存在。

### 6.3 实测结果

最终连续 60 秒采样：

| 指标 | 结果 |
|---|---:|
| FCU 离线样本 | 0 / 60 |
| 视频离线样本 | 0 / 60 |
| 最大遥测时间差 | 10.51 s |
| 无效协议帧增量 | 0 |
| P9 重复帧增量 | 81 |
| CRC 错误增量 | 1 |
| 命令输出 | disabled |

上述样本发现每秒心跳虽已在应用队列中标记为可替换状态，串口层却仍要求可靠
ACK。持续上传遥测时，P9 反向 ACK 可能超过 0.4 秒，机载端便重发同一心跳。
这与“周期状态只保留最新值”的语义冲突，也是高重复率的主要原因。

2026-07-30 将心跳和遥测统一改为周期刷新，只对握手、命令和任务结果执行
ACK/重传。相同 60 秒只读回归结果：

| 指标 | 修复前 | 修复后 |
|---|---:|---:|
| 接收帧增量 | 164 | 152 |
| 重复帧增量 | 91 | 0 |
| CRC 错误增量 | 0 | 0 |
| 地面重传增量 | 0 | 0 |
| 无效负载增量 | 0 | 0 |
| Agent/FCU/视频/只读门可用率 | 100% | 100% |

周期状态丢失时会在下一周期自动刷新；命令和任务证据的可靠性没有降低。

## 7. 最新帧视频桥

2026-07-30 已完成原先的帧率不匹配修复：

- Windows OpenCV 不再按 5 FPS 休眠，而是由 RTSP 源节拍持续读取；
- 每次成功解码都覆盖内存中的单个最新 JPEG，不保留待显示旧帧队列；
- 浏览器在上一张 JPEG 完成加载后才请求下一张，失败时退避重连；
- `/api/camera/status` 增加 `source_fps`、`frames_received`、
  `consumer_skipped_frames` 和 `frame_age_s`；
- 机载 FFmpeg 使用零 B 帧、15 帧 GOP、小接收缓存和立即 flush。

真实 RTSP/Web API 短样本结果：

| 指标 | 结果 |
|---|---:|
| 视频可用率 | 100% |
| 解码源帧率中位数 | 15.0 FPS |
| 最大桥接帧龄 | 0.062 s |
| `/api/camera/frame` | HTTP 200, image/jpeg |

`frame_age_s` 从画面完成 Windows 解码时开始计算，因此不能代表 D435i 曝光到
浏览器显示的完整端到端延迟。后续仍需给采集端加入可验证时间戳，测量 P50/P95，
再决定是否需要 WebRTC。当前链路也不构成实时避障安全传感器。

## 8. 验收接口

```powershell
Invoke-RestMethod http://127.0.0.1:8000/api/status
Invoke-RestMethod http://127.0.0.1:8000/api/camera/status
curl.exe -o NUL -w "status=%{http_code} type=%{content_type}`n" `
  http://127.0.0.1:8000/api/camera/frame
```

只读阶段验收必须满足：

```text
runtime_mode=real_serial
vehicle_connected=true
fcu_link_ok=true
command_output_enabled=false
camera.state=online
camera.stream_available=true
/api/camera/frame -> HTTP 200 image/jpeg
```

页面验收还必须确认：

- 顶部显示“地面站在线”和“FCU 在线”；
- 摄像头区域显示真实 D435i RGB 画面；
- 六个飞行按钮和高度输入均为 disabled；
- 页面显示“只读验收模式：机载飞控命令输出已禁用”。

ARM/DISARM 白名单阶段改为：

```text
runtime_mode=real_serial
vehicle_connected=true
fcu_link_ok=true
command_output_enabled=true
allowed_commands=["arm", "disarm"]
armed=false
prearm_ok=true
```

页面只能启用“解锁”和“上锁”。其他四个飞行按钮及起飞高度输入必须保持
disabled。地面 API 和机载 Agent 都要拒绝白名单外的命令。

## 9. 地面视觉语义闭环

2026-07-30 增加非 ROS 的地面视觉语义闭环。Windows 地面站直接读取视频桥中的
最新 JPEG 帧，调用 OpenAI 兼容的 VLM，并在 Web 中显示：

- 箱体或目标名称、编号、颜色、标识面和置信度；
- 场景摘要、对象列表、风险等级、建议和识别证据；
- 经校验的结构化结果以及模型原始返回；
- 自然语言任务的结构化草案、步骤、安全检查和待确认歧义。

新增接口：

```text
GET  /api/semantic/status
GET  /api/semantic/latest
POST /api/semantic/vision/analyze
POST /api/semantic/mission/parse
```

该闭环是“感知和解析闭环”，不是自动飞行闭环。任务解析固定返回
`execution_policy=preview_only` 和 `flight_command_generated=false`。模型输出不会
进入 Lite 飞行命令队列，也不会绕过机载 `command_output_enabled` 门禁。

配置文件位于 Windows 地面电脑：

```text
%USERPROFILE%\.aeromind\vlm.env
```

内容使用本机模型服务实际值：

```text
AEROMIND_VLM_API_URL=https://your-openai-compatible-endpoint/v1
AEROMIND_VLM_API_KEY=<local-secret>
AEROMIND_VLM_MODEL=qwen3-vl-plus
AEROMIND_LLM_MODEL=qwen3-vl-plus
```

重新运行 `deploy/windows/start-ground-uav3.ps1` 后，Web 的 VLM 状态应显示模型名。
凭据不会进入 Web API、P9 链路或树莓派。树莓派端无需更新视觉模型程序，只需维持
现有 D435i RGB 推流。

同日补充地面站重启恢复：若地面进程收到机载旧会话的周期遥测，会发送可靠的
`session_reset` 控制帧，请求 Agent 重新建立认证会话。该帧不包含 MAVLink 或飞行
命令。现场重启后 FCU/P9 自动恢复在线，随后 10 秒样本接收 25 帧，重复、CRC、
无效负载和重试增量均为 0。

同日完成 Web 布局调整：左侧显示飞行器与链路状态，中间以 16:9 区域显示实时
相机，视觉识别和任务语义位于相机下方，右侧保留飞行控制和三层命令证据。视觉
分析与任务解析分别保留结果，结构化结果和模型原始回复可切换查看。

ARM/DISARM 白名单部署后的现场状态为：FCU/P9/RGB 在线，模式 `STABILIZE`，
`prearm_ok=true`、`armed=false`。5 秒只读采样收到 13 帧，重复、CRC、无效负载
增量均为 0，机载进程 `NRestarts=0`。该采样没有调用任何命令接口。

## 10. 当前边界和下一步

尚未完成：

1. D435i Depth 和 IR 驱动及 Web/VLM 数据接口；
2. 将 D435i 改接 USB 3.0 并验证供电和带宽；
3. P9 物理链路噪声排查和不少于 30 分钟稳定性测试；
4. 日志轮转、开机顺序和断电恢复验收；
5. 由操作者在拆桨条件下完成一次 ARM 后立即 DISARM，并保存三层证据；
6. RC 接管、失联保护和 ArduPilot failsafe 验收；
7. 任何带桨或低空实飞。

在以上安全门禁完成前，不得扩展 ARM/DISARM 之外的机载命令白名单，也不得把
Web 画面在线等同于具备实时避障能力。

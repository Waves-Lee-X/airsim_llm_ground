# AeroMind-APM Lite 中文文档总览

这套文档描述当前非 ROS 的 AeroMind-APM Lite。旧 ROS 2/PX4 工程是原型参考，
不能用旧工程的运行方法、MAVROS 话题或安全结论替代 Lite 文档。

## 1. 第一次阅读

如果希望从零理解项目，按以下顺序阅读：

1. [01 实机硬件与接线基线](01-实机硬件与接线基线.md)：先认识 V5+、树莓派、P9 和 D435i。
2. [02 总体架构与数据流](02-总体架构与数据流.md)：理解每个程序运行在哪里、三条链如何连接。
3. [03 Lite v1 通信协议详解](03-Lite-v1通信协议详解.md)：理解 P9 帧、认证、消息和 ACK。
4. [04 配置文件与运行模式](04-配置文件与运行模式.md)：理解 DEMO、SITL、实机和配置来源。
5. [05 Web 地面站设计与接口](05-Web地面站设计与接口.md)：理解页面、API、WebSocket 和门禁。
6. [06 树莓派机载端设计](06-树莓派机载端设计.md)：理解机载进程、UART、MAVLink 和 systemd。
7. 根据需要继续阅读仿真、实机、视觉、Agent、坐标、导航和证据手册。

读完 01-06 后，应当能够回答：

- 为什么 Lite 不需要 ROS/MAVROS；
- Web、地面 Python、P9、树莓派和 V5+ 分别做什么；
- 视频为什么不走 P9；
- 为什么协议中有命令不代表真机已经授权；
- `COMMAND_ACK` 和物理完成有什么区别；
- 哪些修改需要更新树莓派，哪些只修改地面站。

## 2. 当前设计手册

| 编号 | 文档 | 主要内容 | 当前状态 |
|---:|---|---|---|
| 01 | [实机硬件与接线基线](01-实机硬件与接线基线.md) | V5+、Pi、P9、D435i、UART 和供电 | UAV3 已确认，四机仍需逐机核对 |
| 02 | [总体架构与数据流](02-总体架构与数据流.md) | 组件职责、三条物理链、运行模式、虚实闭环 | 当前架构基准 |
| 03 | [Lite v1 通信协议详解](03-Lite-v1通信协议详解.md) | 二进制帧、CRC、压缩、认证、HMAC、TTL、Schema、ACK | 已实现并测试 |
| 04 | [配置文件与运行模式](04-配置文件与运行模式.md) | YAML、环境文件、AirSim、GeoReference、配置哈希 | 可用 |
| 05 | [Web 地面站设计与接口](05-Web地面站设计与接口.md) | 页面布局、REST、WebSocket、串口重配、控制门禁 | 可用 |
| 06 | [树莓派机载端设计](06-树莓派机载端设计.md) | 机载进程、MAVLink、白名单、systemd、更新范围 | UAV3 已部署 |
| 07 | [仿真环境启动与调试](07-仿真环境启动与调试.md) | settings.json、UE、AirSim、SITL、相机和启动顺序 | 已联调 |
| 08 | [实机部署与 P9 串口调试](08-实机部署与P9串口调试.md) | 安装、凭据、服务、Windows 地面站、60 秒验收、回滚 | UAV3 已部署 |
| 09 | [相机视频与视觉语义](09-相机视频与视觉语义.md) | D435i、RTSP、OpenCV、延迟、VLM 和当前边界 | RGB 已联调，Depth 待办 |
| 10 | [Mission Agent 设计与调试](10-Mission-Agent设计与调试.md) | 多轮对话、实时状态、工具草案、二次门禁 | 软件闭环已通过 |
| 11 | [场地标定与坐标转换](11-场地标定与坐标转换.md) | WGS84/map/LOCAL_NED/AirSim、draft/surveyed | 软件完成，外场实测待办 |
| 12 | [导航状态机与故障注入](12-导航状态机与故障注入.md) | GUIDED/TAKEOFF/GOTO/HOLD/LAND 和五类故障 | SITL 已验收 |
| 13 | [轨迹证据与虚实误差](13-轨迹证据与虚实误差.md) | planned/predicted/observed、GPS 回放和误差报告 | SITL 证据已生成，外场待办 |
| 14 | [常见故障定位与恢复](14-常见故障定位与恢复.md) | FCU/P9/ARM/视频/VLM/AirSim/systemd 排障 | 可用 |
| 15 | [测试验收与后续开发](15-测试验收与后续开发.md) | 274 项测试、L0-L6 分级调试和后续 M5-M8 | 当前验收入口 |
| 16 | [编队设计与验收](16-编队设计与验收.md) | 三种队形、平滑切换、槽位分配、时空碰撞与失联策略 | 逻辑已实现，SITL 回归待跑 |

## 3. 历史验收记录

下面是当时环境和结果的证据，不是当前操作手册。历史测试数量和未完成项应按记录
日期理解。

| 编号 | 文档 | 结果 |
|---:|---|---|
| 90 | [M0 协议与软件边界验收记录](90-M0协议与软件边界验收记录.md) | 初始 Schema、认证和非 ROS 边界通过 |
| 91 | [M1 单机自动仿真验收记录](91-M1单机自动仿真验收记录.md) | 固定任务 10/10 和独立 RTL 通过 |
| 92 | [M1.5 手动仿真与 Web 联调记录](92-M1.5手动仿真与Web联调记录.md) | 非 Blocks 场景、手动 UE/SITL/Web/相机通过 |
| 93 | [UAV3 实机链路与视频联调记录](93-UAV3实机链路与视频联调记录.md) | FCU/P9/RGB/VLM 和稳定性问题修复记录 |

## 4. 按任务选择阅读路线

### 4.1 我只想先把仿真跑起来

阅读 02、04、07，然后按 12 执行导航基线。遇到问题查 14。不要连接 `COM3` 或使用
`configs/real`。

### 4.2 我要连接三号真机

阅读 01、03、06、08、14、15。必须先做 L4 的 60 秒只读测试，再决定是否进行无桨
ARM/DISARM。当前不能测试实机起飞、GOTO、LAND 或 RTL。

### 4.3 我要在一个地面站同时看仿真和实机

先读 02 的混合链路边界，再按 07 第 6 节启动。混合地面站必须运行在 Windows；
生成 SITL 命令时必须加 `--ground-on-windows`。默认仿真 UAV1、实机 UAV3，两个
机号不能相同。

### 4.4 我要理解新协议

先读 02 的数据链，再完整读 03；结合源码：

```text
common/contracts/
common/communication/protocol.py
common/communication/serial_transport.py
ground/server.py
ground/serial_server.py
onboard/agent.py
```

### 4.5 我要调相机和大模型

阅读 09 和 10。相机画面先通过，再调用 VLM；VLM 结果正确后再测试 Mission Agent。
模型不可用时不能影响 FCU/P9 状态。

### 4.6 我要理解“虚控实、实应虚”

阅读 02、11、12、13。重点理解场地标定、同一任务 ID、三类轨迹和版本哈希。当前
真机实时镜像与 observed 外场证据尚未完成。

### 4.7 我遇到了报错

直接打开 14，先采集状态再定位层级。不要同时重启全部设备，也不要通过关闭飞控
预检让错误消失。

## 5. 五分钟启动入口

### DEMO（不连接飞控）

```bash
cd ~/aeromind_ws/apm_lite
PYTHONPATH=src python3 -m aeromind_apm_lite.ground.browser.app \
  --mode demo --host 127.0.0.1 --port 8010 --disable-camera
```

### SITL（UE、AirSim 和 ArduCopter 由操作者先启动）

```bash
PYTHONPATH=src python3 -m aeromind_apm_lite.ground.browser.app \
  --mode sitl --host 127.0.0.1 --port 8000
```

### UAV3 实机地面站

```powershell
Set-Location "\\wsl.localhost\Ubuntu-22.04\home\waves\aeromind_ws\apm_lite"
powershell -ExecutionPolicy Bypass -File .\deploy\windows\start-ground-uav3.ps1
```

这些只是入口，不替代 07/08 中的前置检查。

## 6. 当前不能误解的结论

- 当前实机白名单只有 ARM/DISARM，并且只允许无桨台架。
- TAKEOFF/HOLD/LAND/RTL/GOTO 在仿真可用，不代表真机已授权。
- D435i 当前只有 RGB；没有 Depth 安全层，不具备未知障碍自主绕行。
- Mission Agent 只能生成受控工具草案，不能直接控制飞控。
- 场地标定仍为 `draft`；没有外场实测时不能转换并执行真实航点。
- GPS 是首版真实定位方案，但普通 GPS 不能实现箱体表面附近的厘米级降落。
- 四机编队、路径规划和寻物项目仍需先完成多机仿真与外场分级验收。

## 7. 状态术语

| 术语 | 含义 |
|---|---|
| 已实现 | 代码存在且自动测试通过 |
| 已联调 | 在指定真实或仿真环境中观察到预期数据 |
| 已验收 | 按文档门禁执行并保存了可复查证据 |
| 已授权 | 当前安全范围明确允许执行；不会自动扩展到其他命令 |
| `preview_only` | 只能预览和分析，不能记为正式外场结果 |

## 8. 文档维护规则

1. 当前实现变化先更新 01-15 对应手册，再追加历史记录。
2. 命令必须注明运行位置：Windows PowerShell、WSL 或树莓派 Bash。
3. 不在文档写真实 API Key、P9 密钥或 WiFi 密码。
4. 仿真结果、实机台架和外场结果必须分开表述。
5. 文件重命名后运行 Markdown 链接检查。
6. 当前测试基线、提交和硬件状态变化时同步更新 15 和根 README。

项目长期路线见
[AeroMind-APM Lite 开发计划](../../doc/14-AeroMind-APM-Lite开发计划.md)。

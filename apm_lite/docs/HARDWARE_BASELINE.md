# V5+ 实机硬件基线

记录日期：2026-07-28

## 证据范围

本基线来自两类资料：

1. 用户提供的《郑州550无人机.docx》，用于确认硬件型号、规格和接线图。
2. 旧实机项目 `gps_real-main/swarmxian260413.py`，仅用于记录原系统曾使用的串口设备与波特率，不视为当前四架飞机已经完成台架验证。

原始硬件文档证据指纹：

```text
文件大小：1,385,935 bytes
修改时间：2025-11-25 19:00:31（源文件系统记录）
SHA-256：e4a249b57d4b112a4aaf6f418559680fa285fdc57f9494f444283c82d14b11ad
```

本文档确认的是硬件标签和接口资料。它没有给出 V5+ 当前安装的
ArduCopter 版本、板型构建目标或参数集，因此不能据此宣称真机软件兼容性已经冻结。

## 已确认硬件

| 部件 | 文档确认结果 | 对 APM Lite 的影响 |
|---|---|---|
| 机架 | 郑州 550 四旋翼 | `frame_class=QUAD`；X/Plus 类型仍需读参数确认 |
| 飞控 | 雷迅/CUAV V5+，STM32F765 主处理器 | 属于现代 Pixhawk 兼容类，不是旧 APM 2.x |
| 飞控接口 | 5 路 UART、2 路 CAN；TELEM1/TELEM2 为 GH1.25-6P | 树莓派计划连接 TELEM2 |
| 机载电脑 | Raspberry Pi 4B，4 GB，标称输入 5 V/3 A | 运行单一轻量机载服务，不运行 ROS/MAVROS/模型 |
| 定位 | C-RTK 9Ps，u-blox ZED-F9P | 具备 RTK 能力；差分源与每机实际状态仍需核对 |
| 磁力计 | C-COMPASS，RM3100，DroneCAN | 参数导出时检查 CAN 驱动与罗盘实例 |
| 数传 | P9Radio/MiniHomer，默认串口 57600；2026-07-29 地面 USB 端枚举为 CP210x `COM3` | 透明串口连接地面站与 Pi `/dev/ttyAMA1`；不能套用到 Pi-FCU 链路 |
| 遥控接收机 | RF209S | 保留人工接管与失控保护链路 |
| 电源模块 | HV-PM，3-14S、60 A，飞控输出标称 5.3 V/4 A | 该规格不能证明 TELEM2 可给树莓派提供 3 A |
| 深度相机 | 2026-07-29 实机枚举和视频节点确认是 Intel RealSense D435i，具有 RGB、Depth、双 IR 和 IMU | 当前只接入 `/dev/video4` RGB；深度、IR、IMU、尺度和安装方向仍未验收 |

## 树莓派到 V5+ 接线基线

V5+ TELEM2 图示引脚为：`5V+ / TX / RX / CTS / RTS / GND`。

APM Lite 首版使用无硬件流控的三线 UART：

```text
V5+ TELEM2 TX  -> Raspberry Pi UART RX
V5+ TELEM2 RX  -> Raspberry Pi UART TX
V5+ TELEM2 GND -> Raspberry Pi GND
CTS/RTS        -> 不连接
```

旧实机脚本使用：

```text
/dev/ttyAMA0 @ 921600 baud, 8N1, no flow control
```

旧实机脚本的独立地面数传链路使用：

```text
/dev/ttyAMA1 @ 57600 baud, 8N1 -> airborne P9 Radio
ground P9 Radio -> Windows CP210x COM3 @ 57600（本次枚举结果）
```

这组值可作为第一次只读心跳测试的候选值，但必须与飞控实际 `SERIALx_PROTOCOL`、`SERIALx_BAUD` 对照。不能仅因旧代码写了 921600 就直接解锁或发控制指令。

## 供电安全结论

资料写有“TELEM2 的 5V 连接树莓派 5V”，但没有给出 TELEM2 的 5V 额定电流；同一资料注明树莓派需要 5 V/3 A。因此当前冻结规则是：

- 树莓派使用独立、足额、稳压的 5 V 电源。
- 树莓派与 V5+ 必须共地。
- 厂家未书面确认 TELEM2 供电能力前，不连接 TELEM2 5V 到树莓派 5V。
- 连接前确认 UART 为 3.3 V 逻辑；若不是，必须使用电平转换。
- 所有首次测试均拆桨进行。

## 台架待确认项

以下项目不在所给文档中，当前不得猜测：

1. 已读取 UAV3 固件标识 `4.4.1-255`，仍需确认完整 Git 版本和板型名称。
2. `FRAME_CLASS`、`FRAME_TYPE`、电机顺序和 `SYSID_THISMAV`。
3. TELEM2 对应的 `SERIALx` 参数、MAVLink 版本、波特率和电平。
4. 四架飞机的完整参数文件及 SHA-256。
5. UAV3 已确认 Raspberry Pi OS 11、Python 3.9.2 和两路 UART；仍需补齐
   overlay、串口控制台、散热和独立供电的正式证据。
6. D435i 安装方向、USB 3 连接、深度尺度、IR/IMU 驱动和供电余量。
7. RTK 基站/改正数链路，以及四机是否都能稳定进入 RTK Fixed。

## UAV3 当前软件与链路状态

2026-07-30 已确认：

| 项目 | 结果 |
|---|---|
| Lite 机载服务 | `aeromind-apm-lite@3.service`，enabled/active |
| RGB 推流服务 | `ed_rtsp.service`，active |
| FCU/P9 | `/dev/ttyAMA0 @ 921600`、`/dev/ttyAMA1 @ 57600` |
| 当前网络 | `192.168.1.109`，仅用于 SSH/RTSP，不替代 P9 控制链路 |
| 命令权限 | 仅 ARM/DISARM；TAKEOFF/HOLD/LAND/RTL 禁用 |
| 视觉语义与 Agent | Windows 地面端 VLM/LLM；模型只生成语义结果或受控草案，确认后仍经过地面门禁和机载白名单 |

这些结论只适用于三号机当前台架，不能直接外推到其他三架飞机。

## 第一次无桨台架检查

1. 独立给树莓派和飞控供电，只连接交叉 TX/RX 与共地。
2. 在树莓派确认 `/dev/ttyAMA0` 或 `/dev/serial0` 的实际映射，并关闭 Linux 串口控制台。
3. 从飞控参数或地面站读取 TELEM2 对应的协议与波特率。
4. APM Lite 只读等待 HEARTBEAT，不发送解锁、模式或参数修改命令。
5. 读取 `AUTOPILOT_VERSION`、`SYSID_THISMAV`、机架参数和固件标识并写回兼容矩阵。
6. 导出完整参数，计算哈希后才进入无桨命令测试。

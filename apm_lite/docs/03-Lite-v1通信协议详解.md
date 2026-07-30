# 03 Lite v1 通信协议详解

## 1. 为什么不是直接透传 MAVLink

树莓派已经通过 `/dev/ttyAMA0` 和 V5+ 使用 MAVLink，但地面 P9 链路没有直接透传
这条 FCU 串口。Lite 在地面站和树莓派之间定义独立协议，原因是需要：

- 每架飞机独立认证和机号隔离；
- 命令 TTL、重放保护和会话重建；
- 命令白名单和应用级 ACK；
- 在 57600 波特率下控制重传和周期状态带宽；
- 向 Web 提供统一的仿真/实机接口。

协议分为四层：

```text
业务消息 Schema
  -> HMAC 签名 JSON 帧
  -> 双向挑战会话
  -> P9 二进制传输帧
  -> 透明串口字节流
```

MAVLink 位于树莓派到 FCU 的另一条链路，不是 Lite v1 帧的内层封装。

## 2. P9 二进制传输帧

源码：`common/communication/serial_transport.py`。

帧使用小端格式 `<2sBBBBBIH`，固定头 13 字节，尾部 CRC32 为 4 字节：

| 偏移 | 长度 | 字段 | 当前值/含义 |
|---:|---:|---|---|
| 0 | 2 | magic | `A9 4C` |
| 2 | 1 | version | `1` |
| 3 | 1 | kind | `1=DATA`，`2=ACK` |
| 4 | 1 | direction | `1=机载到地面`，`2=地面到机载` |
| 5 | 1 | vehicle_id | `1..255` |
| 6 | 1 | flags | bit0 可靠，bit1 zlib 压缩 |
| 7 | 4 | sequence | `uint32` 传输帧序号 |
| 11 | 2 | payload_length | 实际传输负载长度 |
| 13 | N | payload | UTF-8 JSON 或压缩后的 JSON |
| 13+N | 4 | crc32 | 覆盖 magic 之后的头部和实际传输负载 |

约束：

- 默认解压后最大负载 `8192` 字节；
- 原始负载达到 `256` 字节且压缩后更小时，使用 zlib level 1；
- 解压器限制输出大小并拒绝尾随数据，防止压缩炸弹；
- ACK 帧没有负载，也不能设置压缩标志；
- 解码器能从噪声和分片字节流中重新寻找 magic，并统计废弃字节和 CRC 错误。

### 2.1 可靠发送策略

默认可靠帧超时 `0.4 s`，最多重试 `4` 次。可靠帧接收方先回传输 ACK，再按
`(vehicle_id, sequence)` 做重复抑制。

不是所有消息都重传：

| 数据 | 串口层策略 | 原因 |
|---|---|---|
| 握手、命令、MissionAck、会话重置 | 可靠发送 | 丢失会改变流程结果 |
| Heartbeat、VehicleTelemetry | 非可靠、下一周期覆盖 | 避免旧状态重传挤占 P9 带宽 |

这项策略解决了旧版本中周期状态帧与 ACK 竞争导致的大量重复帧和 FCU 状态闪烁。

## 3. 双向挑战认证

源码：`common/communication/protocol.py`。每架飞机使用至少 32 字节的独立共享密钥。
密钥只存在于 Windows 本机配置和对应树莓派配置，不进入仓库或 Web。

握手顺序：

```text
Onboard -> Ground: client_hello(client_challenge, vehicle_id, session_id)
Ground  -> Onboard: server_challenge(server_challenge, server_proof)
Onboard 验证 server_proof
Onboard -> Ground: client_proof
Ground 验证 client_proof
Ground  -> Onboard: session_accepted
```

证明值使用 HMAC-SHA256，并绑定：

- `client` 或 `server` 角色；
- 一次性 challenge；
- `vehicle_id`；
- UUID `session_id`。

旧会话 ID、重复会话或身份在握手中变化都会被拒绝。地面站重启后若收到机载旧会话
数据，会可靠发送 `session_reset`，要求 Agent 建立新会话。

## 4. 签名消息帧

会话建立后，业务消息放入以下 JSON 外层：

```json
{
  "protocol_version": "1.0",
  "frame_type": "signed_message",
  "vehicle_id": 3,
  "session_id": "00000000-0000-0000-0000-000000000000",
  "algorithm": "hmac-sha256",
  "message": {},
  "signature": "64位十六进制HMAC"
}
```

HMAC 的输入不是任意 JSON 文本，而是业务对象按键排序、无多余空格的 UTF-8 规范
JSON。接收方还会检查外层的机号/会话与内层消息完全一致。

## 5. 业务消息公共字段

所有业务消息使用严格 Pydantic Schema，未知字段直接拒绝：

| 字段 | 作用 |
|---|---|
| `schema_version` | 固定 `1.0` |
| `message_type` | 判别具体消息类型 |
| `message_id` | 消息 UUID，用于去重和 ACK 关联 |
| `mission_id` | 可选任务 UUID |
| `vehicle_id` | 目标或来源飞机编号 |
| `sequence` | 会话内严格递增的业务序号 |
| `session_id` | 当前认证会话 |
| `created_at_utc` | 带时区的 UTC 时间 |
| `ttl_ms` | `100..300000` 毫秒 |
| `frame` | `none/map/local_ned/global_wgs84/body_frd/camera_optical` |
| `frame_calibration_id` | 带坐标消息必须提供的标定版本 |

接收门禁依次检查 HMAC、机号、会话、标定 ID、`message_id`、递增序号和接收端单调
时钟 TTL。墙上时钟只用于审计，过期判断不依赖可能跳变的系统时间。

## 6. 消息类型

| `message_type` | 方向 | 用途 | 当前实机状态 |
|---|---|---|---|
| `heartbeat` | 机载 -> 地面 | 版本、配置哈希、输出开关和白名单 | 已使用 |
| `vehicle_telemetry` | 机载 -> 地面 | ARM、模式、位置、速度、电池和健康状态 | 已使用 |
| `vehicle_command` | 地面 -> 机载 | 原子飞行命令 | 仅 ARM/DISARM 授权 |
| `mission_ack` | 机载 -> 地面 | 应用、运行、完成、失败、过期结果 | 已使用 |
| `trajectory_segment` | 地面 -> 机载 | 带时序短轨迹 | Schema 已定义，实机未授权 |
| `formation_command` | 地面 -> 机载 | 队形和切换参数 | Schema 已定义，尚未实现执行闭环 |
| `semantic_observation` | 机载/地面语义层 | 箱体、颜色、面和证据引用 | Schema 已定义，尚未进入实机任务 |

`VehicleCommand` 支持 `arm/disarm/takeoff/land/rtl/hold/cancel`。只有 `takeoff` 接受
`target_altitude_m`。协议支持某个命令不代表机载白名单允许它。

## 7. MissionAck 和物理完成

`MissionAck.status` 取值：

```text
accepted -> running -> completed
                   \-> failed/cancelled/expired
```

`completed` 必须同时设置 `physical_completion_confirmed=true`。机载 `ApmLink` 将
MAVLink 命令结果和新鲜遥测合并，例如：

- ARM：`COMMAND_ACK` 匹配，且 `armed=true` 持续满足；
- DISARM：`armed=false`；
- TAKEOFF：模式、ARM 和相对高度达到门限；
- HOLD：飞行模式和速度达到悬停条件；
- LAND/RTL：最终落地和加锁状态满足。

如果没有收到 `COMMAND_ACK`，但某些命令没有 ACK 语义或遥测已经证明物理完成，报告
会明确记录证据差异，不能伪造 ACK。

## 8. 队列和最新状态合并

命令和证据是不可替换事件，队列满时不得静默覆盖。Heartbeat 和 Telemetry 是周期
状态，可按 replacement key 只保留最新值。这样即使 Web 消费较慢，也不会让旧遥测
拖住认证链路。

## 9. 调试指标

Web 底部和 `/api/status` 会显示：

| 指标 | 正常解释 | 持续增长时检查 |
|---|---|---|
| `received_frames` | 收到的有效 Lite 帧 | 是否持续刷新 |
| `duplicate_frames` | 重传后重复到达 | ACK 丢失、链路拥塞 |
| `crc_errors` | 帧内容校验失败 | 波特率、供电、接地、P9 空中链路 |
| `discarded_bytes` | 为重新同步丢弃的字节 | 串口被其他协议/进程污染 |
| `retries` | 可靠帧超时重发 | 双向链路或 ACK 拥塞 |
| `invalid_payloads` | 帧有效但 JSON/身份非法 | 两端版本不一致、旧会话 |
| `session_reset_requests` | 地面请求机载重握手 | 地面站重启后少量出现正常 |

协议 JSON Schema 位于 `schemas/protocol-v1.schema.json`，由源码导出，不手工编辑：

```bash
cd ~/aeromind_ws/apm_lite
PYTHONPATH=src python3 tools/export_schemas.py --check
```

继续阅读：[04 配置文件与运行模式](04-配置文件与运行模式.md)。

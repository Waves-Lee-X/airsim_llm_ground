# M1 验收记录：单机 AirSim/ArduCopter SITL

验收日期：2026-07-28

## 结果

**M1 单机 AirSim/ArduPilot SITL 门禁通过。** 固定任务完成 10/10，独立 RTL
路径完成。每个飞行动作都保留应用接收、适用时的 MAVLink `COMMAND_ACK` 和
新鲜物理遥测证据。M1 没有向真机发送命令。

冻结环境：

- AirSim `1.8.1`，Blocks 场景；
- ArduPilot Copter `4.7.0`，提交
  `1511f27194f1dcc3728270883047bdf022b3fd53`；
- `pymavlink 2.4.49`；
- 单 AirSim 飞机、单 SITL、单 Lite Agent；
- GCS `SYSID=201`、飞行器 `SYSID=1`、接收端 `udpin:0.0.0.0:14550`。

该冻结只适用于 M1 仿真，不能证明实机 V5+ 的固件、参数、串口或 GPS 延迟。

## 固定任务

每轮都返回本轮起点，避免十轮任务不断向场景外累积位移：

```text
GUIDED
  -> ARM
  -> TAKEOFF 2 m
  -> LOCAL_NED 向北 3 m
  -> HOLD
  -> GUIDED
  -> 返回本轮起点
  -> LAND
```

| 动作 | MAVLink 证据 | 物理完成证据 |
|---|---|---|
| ARM/DISARM | command 400 ACK | 新鲜 HEARTBEAT armed 位 |
| TAKEOFF | command 22 ACK | 已解锁、达到相对高度、速度稳定 |
| SET_MODE | command 176 ACK | 新鲜 HEARTBEAT 模式 |
| GUIDED 目标 | 不适用 ACK | LOCAL_POSITION_NED 误差与速度稳定 |
| HOLD | command 176 ACK | 进入配置的保持模式且速度稳定 |
| LAND | command 21 ACK | ON_GROUND 且已上锁 |
| RTL | command 20 ACK | 回到 Home 容差、ON_GROUND 且已上锁 |

单独收到 `ACCEPTED` 不能判定动作完成。

## 正式证据

证据目录：

```text
D:\AirSim\aeromind-apm-lite-m1\acceptance4\run-20260728T114147Z-548787-d58b2f77
```

结果：

```text
gate: M1_SINGLE_VEHICLE_AIRSIM_SITL
fixed missions: 10/10 completed (threshold: 9/10)
fixed mission actions: 80/80 physically completed
independent RTL path: completed
overall passed: true
```

冻结哈希：

```text
ArduPilot binary SHA-256:
55f5f19fe4078ecdcf183b07a855831a1156002abfc054c7f0ccec82646dec4b

M1 parameter file SHA-256:
609657d295acdef35471de27800db222c4c5c307e1c559f02de8cbde5125f0c9

generated AirSim settings SHA-256:
c97a961d19a42bcdb0fe26cf156fa903cc5ed3a219cd9d876153b8a98b4ce5d3
```

证据目录未提交到 Git，因为包含约 31 MB DataFlash 和运行状态；`summary.json`、
`fixed-missions.jsonl`、`rtl.json`、FCU/AirSim 日志和 BIN 文件保留在本机审计目录。

## 失败批次

失败批次作为负面证据保留：

| 证据目录 | 结果 | 根因 | 修正 |
|---|---:|---|---|
| `D:\AirSim\aeromind-apm-lite-m1\acceptance\run-20260728T104727Z-532493-3da396e7` | 7/10 | AirSim GPS 实测延迟约 128-230 ms，但 `GPS1_DELAY_MS=20`，触发 GPS 不健康 | 只在 AirSim 参数中冻结 200 ms，并在 GUIDED 后重查 GPS/pre-arm |
| `D:\AirSim\aeromind-apm-lite-m1\acceptance2\run-20260728T110021Z-533669-9b8fc5e9` | 8/10 | 旧任务每轮累计向北 3 m，进入 Blocks 障碍区域并触发 EKF 故障 | 每轮 LAND 前返回本轮起点 |

`GPS1_DELAY_MS=200` 只属于 AirSim Connector 标定，未经实机日志证明不得复制到
F9P/V5+ 参数。

预检调用
`D:\AirSim\aeromind-apm-lite-m1\acceptance4\run-20260728T114133Z-548761-cfea746b`
因参数文件使用相对路径而在启动前被拒绝，不计为飞行尝试。

## M1 交付

- 可替换的 MAVLink 边界和确定性假飞控；
- 串口/UDP `PymavlinkTransport` 与 LOCAL_NED 掩码校验；
- 单一异步所有者 `ApmLink`；
- 目标 system/component 过滤、ArduPilot/四旋翼识别和版本探测；
- 位置、速度、姿态、电池、GPS、EKF、着陆、Home 和 STATUSTEXT 遥测；
- 带安全优先级的有界命令队列；
- ARM、DISARM、TAKEOFF、模式、LOCAL_NED、HOLD、LAND、RTL 服务；
- 应用、FCU ACK、物理完成三层证据；
- HMAC WebSocket 会话、TTL、序列和重放保护；
- AirSim/ArduPilot 自动生命周期与精确进程清理；
- 只读探测和可重复 M1 验收命令行。

## 真机边界

仿真命令全部通过不代表真机开放相同权限。UAV3 当前机载白名单仅为 ARM/DISARM；
TAKEOFF、HOLD、LAND、RTL 仍在地面与机载两层被拒绝。

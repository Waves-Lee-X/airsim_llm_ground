# planned/predicted/observed 轨迹证据与 GPS 回放

本模块把“以虚控实、以实应虚”需要比较的轨迹统一保存为场地 `map` 坐标证据：

| 角色 | 含义 | 当前来源 |
|---|---|---|
| `planned` | 任务在各执行阶段要求到达的位置 | `NavigationMissionPlan` |
| `predicted` | 同一任务在 AirSim/ArduCopter SITL 中的实际位置 | SITL `LOCAL_POSITION_NED` |
| `observed` | 真机执行后的实际位置 | APM GPS CSV 离线回放 |

三类证据必须使用相同 `mission_id`、`vehicle_id`、`frame_calibration_id` 和
GeoReference SHA-256。任一项不一致时，比较器直接拒绝，不能把不同场地、不同 Home
或不同任务的轨迹拼成一份报告。

## 1. 证据内容

`TrajectoryEvidence` Schema 版本为 `1.0`，主要包含：

- 证据 UUID、内容 SHA-256、任务和飞机编号；
- `planned/predicted/observed` 角色与数据来源；
- 场地标定 ID、标定 SHA-256 和 `draft/surveyed` 状态；
- 统一 `map` 坐标、相对任务开始时间和可选 UTC 时间；
- 原始 WGS84、GPS Fix、HDOP、卫星数和任务阶段；
- 任务、Lite 软件、ArduCopter、参数和 UE 场景版本信息。

证据按 UUID 原子写入，已存在的 UUID 不允许改写为其他内容。协议 Schema 位于
`schemas/protocol-v1.schema.json`。

## 2. SITL 自动记录 planned/predicted

先准备完整 GeoReference。未做外场测量时可以保持 `draft`，此时生成的证据只能用于
仿真预览，不能成为正式虚实验收结果。

操作者启动 UE 和 SITL 后执行：

```bash
cd ~/aeromind_ws/apm_lite
PYTHONPATH=src python3 -m \
  aeromind_apm_lite.ground.simulation.navigation_acceptance \
  --fleet-config configs/sim/fleet.m1.yaml \
  --north-m 3 --east-m 0 --altitude-m 2 \
  --georeference-config "$PWD/configs/calibration/venue.yaml" \
  --trajectory-evidence-dir /tmp/aeromind-m3-trajectory \
  --software-commit 8605422 \
  --arducopter-version 4.7.0 \
  --report /tmp/aeromind-m3-baseline.json
```

入口仍只接受 `mode: sim` 和 UDP FCU，不会连接实机串口。记录发生在软件故障注入
之前，因此 GPS/EKF/P9 观察层注入不会篡改原始 `predicted` 位置。验收 JSON 的
`trajectory_evidence` 会给出两个输出路径、证据 ID、哈希和采样数。

当前 `planned` 是与实际观察时刻对齐的阶段目标，适合计算位置跟踪误差；它还不是
固定采样周期的完整短轨迹，因此本阶段不据此评价时间调度误差。M5 编队和路径规划
接入 `TrajectorySegment` 后，再增加独立的时序误差指标。

## 3. 真机 GPS CSV 回放

首版回放接受 UTF-8 CSV，必需字段如下：

```csv
observed_at_utc,latitude_deg,longitude_deg,altitude_m,gps_fix_type,gps_hdop,satellites_visible
2026-08-01T02:00:00Z,34.7472000,113.6253000,112.4,3,0.9,14
2026-08-01T02:00:01Z,34.7472100,113.6253000,112.5,3,1.0,13
```

也接受 `timestamp/lat/lon/alt` 等短字段名。时间必须带时区且不能倒退；同一毫秒的
重复记录会合并并计数。高度必须和 GeoReference 的 `height_reference.datum` 一致。
二进制 `.tlog` 当前应先通过 MAVLink 工具导出为上述 CSV，不能直接改扩展名使用。

回放命令：

```bash
PYTHONPATH=src python3 -m aeromind_apm_lite.ground.trajectory.cli replay-gps \
  --input /absolute/path/uav1-gps.csv \
  --georeference /absolute/path/venue.yaml \
  --mission-id 11111111-2222-3333-4444-555555555555 \
  --vehicle-id 1 \
  --producer-version aeromind-apm-lite-v1 \
  --software-commit 8605422 \
  --arducopter-version 4.4.1-255 \
  --output /absolute/path/observed.json
```

正式回放要求 `surveyed` 标定。`--allow-draft` 只允许生成 `preview_only=true` 的离线
预览证据，不会把草稿伪装为实测结果。

## 4. 虚实误差报告

三份证据齐全后运行：

```bash
PYTHONPATH=src python3 -m aeromind_apm_lite.ground.trajectory.cli report \
  --evidence /absolute/path/planned.json \
  --evidence /absolute/path/predicted.json \
  --evidence /absolute/path/observed.json \
  --output /absolute/path/sim-real-report.json
```

报告分别计算 `planned/predicted`、`planned/observed` 和
`predicted/observed` 的水平 RMSE、垂直 RMSE、三维 RMSE、最大误差、P95 和终点
误差。默认阈值只是软件占位值，未做外场统计前
`thresholds.validated_for_venue=false`，因此 `acceptance_passed=null`。

只有同时满足以下条件，报告才允许输出正式布尔验收结果：

1. 三类证据齐全且任务、飞机、标定 ID/哈希完全一致；
2. 标定为 `surveyed`，所有证据均非 `preview_only`；
3. 任务、软件、ArduCopter、参数和场景版本元数据齐全；
4. 外场确定阈值后显式使用 `--thresholds-validated`。

## 5. 地面站 API

地面站默认把证据保存在 `~/.aeromind/trajectory-evidence`，可用
`--trajectory-evidence-dir` 修改。API 为：

```text
GET  /api/trajectory/evidence
POST /api/trajectory/evidence
GET  /api/trajectory/evidence/{evidence_id}
POST /api/trajectory/reports
```

保存时 API 会再次核对当前活动 GeoReference 的 ID、哈希和状态，不一致返回 HTTP
`409`。这些接口只处理文件和误差报告，不会发送 ARM、GOTO、TAKEOFF 或其他飞行
命令，也不会扩大 UAV3 当前 ARM/DISARM 白名单。

## 6. 当前边界

- 软件已具备证据、回放、比较、CLI、API 和确定性测试，当前全量基线为
  `237 passed`。
- 2026-07-30 已生成第 21 项的一份基线、五份故障报告和 12 份 planned/predicted
  证据，绑定提交 `e844420`，保存在
  `D:\AirSim\aeromind-apm-lite-m3\run-20260730-r3`。
- 基线 planned/predicted 终点三维误差为 `0.777 m`，但报告仍为
  `draft/preview_only`，`acceptance_passed=null`。
- `draft` 仿真结果不能替代室外场地实测。
- GPS 回放不能替代静态 GPS、低空航点、RC/failsafe 和编队安全间距验收。

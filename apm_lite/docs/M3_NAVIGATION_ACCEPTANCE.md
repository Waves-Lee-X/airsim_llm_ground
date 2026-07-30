# M3 统一导航状态机与 SITL 故障注入

本阶段验证同一套非 ROS 任务执行器能对 ArduCopter SITL 执行：

```text
任务校验
  -> GPS/HDOP/EKF/Home/P9/新鲜度门禁
  -> GUIDED
  -> ARM
  -> TAKEOFF
  -> GOTO LOCAL_NED
  -> HOLD
  -> LAND
  -> 物理完成证据
```

`NavigationMissionRunner` 位于机载公共执行层，不依赖 AirSim API、ROS 或 MAVROS。
AirSim 只提供仿真物理和传感器，所有飞行动作仍通过 `ApmLink -> MAVLink ->
ArduCopter`。

## 1. 已实现门禁

任务开始前和飞行过程中持续检查：

- FCU 心跳新鲜；
- GPS 健康且至少 3D Fix；
- `GPS_RAW_INT.eph` 转换后的 HDOP 不超过 `2.5`；
- EKF 具备姿态、水平/垂直速度、绝对水平位置和绝对高度标志；
- LOCAL_NED、Home、GPS 和 EKF 遥测未过期；
- P9/地面链路健康回调为真；
- 任务单调时钟期限和 GOTO 目标期限未过期。

飞行中出现故障时，执行器停止等待当前航点，先尝试 `HOLD`，再下发 `LAND`，并
分别保存应用接收、MAVLink ACK 和物理完成证据。任务过期进入 `expired`，其他健康
故障进入 `failed`；不能把恢复命令已发送误记为任务成功。

## 2. 前置条件

本工具只连接操作者已经启动的 UE、AirSim 和 ArduCopter SITL，不管理这些进程。
运行前确认：

1. 使用最新版 `settings.json` 启动目标 UE 场景并进入 Play。
2. SITL 已按手动仿真文档启动，并向 `127.0.0.1:14550` 输出 MAVLink。
3. 场景出生点和目标点之间没有碰撞体，飞行范围已清空。
4. 当前连接的是 `configs/sim/fleet.m1.yaml`，不能使用实机配置。

验收入口会拒绝 `mode: real` 和串口 FCU，因此不会误接 P9/真机。运行期间不要让
其他进程同时占用同一个 14550 输入端口。

## 3. 基线任务

先执行无故障基线：

```bash
cd ~/aeromind_ws/apm_lite
PYTHONPATH=src python3 -m \
  aeromind_apm_lite.ground.simulation.navigation_acceptance \
  --fleet-config configs/sim/fleet.m1.yaml \
  --north-m 3 --east-m 0 --altitude-m 2 \
  --report /tmp/aeromind-m3-baseline.json
```

通过条件：

- `passed=true`；
- `result.terminal_state=completed`；
- 六个步骤均为 `completed`；
- LAND 后有物理落地证据；
- `failure_code=null` 且没有恢复步骤。

需要同时自动保存 `planned/predicted` 轨迹时，增加完整 GeoReference 和绝对输出目录：

```bash
  --georeference-config "$PWD/configs/calibration/venue.yaml" \
  --trajectory-evidence-dir /tmp/aeromind-m3-trajectory \
  --software-commit 8605422 \
  --arducopter-version 4.7.0
```

证据记录发生在故障观察层之前，不会被软件故障注入修改。文件结构、GPS 真机日志
回放和误差报告命令见 [`TRAJECTORY_EVIDENCE.md`](TRAJECTORY_EVIDENCE.md)。

## 4. 软件故障注入

每次测试前确认 SITL 已落地并加锁，一次只运行一种故障：

```bash
for fault in gps_loss hdop_exceeded ekf_failure mission_expired ground_link_loss; do
  PYTHONPATH=src python3 -m \
    aeromind_apm_lite.ground.simulation.navigation_acceptance \
    --fleet-config configs/sim/fleet.m1.yaml \
    --north-m 3 --east-m 0 --altitude-m 2 \
    --fault "$fault" \
    --report "/tmp/aeromind-m3-${fault}.json" || break
done
```

注入在 GOTO 已提交后的第二次健康观察生效。报告必须出现对应 `failure_code`，并包含
`safety_hold` 和 `recovery_land`。工具只有收到预期故障结果才返回成功退出码。

当前注入层是 `navigation_health_observation`：它验证 Lite 状态机和恢复逻辑，不修改
ArduPilot 内部传感器或参数，报告固定写明
`ardupilot_sensor_state_modified=false`。后续若做 ArduPilot 传感器级故障实验，应
另存证据，不能用本结果替代。

## 5. 当前现场状态

2026-07-30 已在独立仿真端口完成实飞控隔离的 AirSim/SITL 验收，未连接三号真机、
P9 或实机串口。环境为 AirSim 1.8.1、UE 4.27、ArduCopter 4.7.0，Lite 软件提交为
`e844420`。最终证据目录为：

```text
D:\AirSim\aeromind-apm-lite-m3\run-20260730-r3
```

| 场景 | 终态/失败码 | 安全恢复 | 结果 |
|---|---|---|---|
| baseline | `completed` | 无 | 通过 |
| gps_loss | `failed / gps_unhealthy` | `HOLD -> LAND` | 通过 |
| hdop_exceeded | `failed / hdop_exceeded` | `HOLD -> LAND` | 通过 |
| ekf_failure | `failed / ekf_unhealthy` | `HOLD -> LAND` | 通过 |
| mission_expired | `expired / mission_expired` | `HOLD -> LAND` | 通过 |
| ground_link_loss | `failed / ground_link_lost` | `HOLD -> LAND` | 通过 |

六轮均生成 planned/predicted 证据。基线比较结果为：水平 RMSE `1.704 m`、垂直
RMSE `0.899 m`、最大三维误差 `3.189 m`、终点三维误差 `0.777 m`。这些指标在
当前软件占位阈值内，但标定状态仍为 `draft`，报告为 `preview_only`，不能作为外场
精度验收结论。

## 6. 安全边界

- 不得把该命令的 `--fleet-config` 改为实机 YAML。
- 不得在真机地面站或 P9 链路上试运行本阶段 GOTO。
- 当前 UAV3 实机白名单仍只有 ARM/DISARM。
- 软件注入通过不代表真实 GPS 失锁、EKF 异常或数传失联试验已经验收。
- SITL 基线和五份故障报告已齐全，开发计划第 21 项已完成；第 22 项外场验收仍未完成。

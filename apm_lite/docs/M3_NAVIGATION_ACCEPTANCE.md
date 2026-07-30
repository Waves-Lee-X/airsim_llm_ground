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

2026-07-30 收尾检查时，本机 `8000` 连接的是三号真机 `real_serial`，不是 SITL，
且室内 GPS 为 `FIX 1`。因此本轮没有执行任何飞行命令。自动化假飞控测试已经覆盖
正常六阶段和五类故障收敛；真实 AirSim/SITL 飞行证据仍需在操作者启动仿真后生成。

## 6. 安全边界

- 不得把该命令的 `--fleet-config` 改为实机 YAML。
- 不得在真机地面站或 P9 链路上试运行本阶段 GOTO。
- 当前 UAV3 实机白名单仍只有 ARM/DISARM。
- 软件注入通过不代表真实 GPS 失锁、EKF 异常或数传失联试验已经验收。
- SITL 基线和五份故障报告齐全后，才能在计划中将第 21 项标记为完全完成。

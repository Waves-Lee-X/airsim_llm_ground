# PX4 + AirSim 联合仿真配置说明

## 环境信息

| 组件 | 位置 | 说明 |
|------|------|------|
| AirSim | Windows 宿主机 | UE 引擎，物理/视觉/传感器仿真 |
| PX4 SITL | WSL Ubuntu 22.04 | 飞控固件仿真，通过 TCP:4560 接收 AirSim 传感器数据 |
| ROS 2 Humble | WSL Ubuntu 22.04 | 上层自主飞行，通过 uXRCE-DDS 与 PX4 通信 |

## 1. AirSim settings.json 配置要点

### 1.1 PX4 连接配置

```json
"Drone1": {
  "VehicleType": "PX4Multirotor",   // 使用 PX4 外部飞控，而非内置 SimpleFlight
  "UseSerial": false,                // 走 TCP，不走串口
  "SitlIp": "<WSL_IP>",             // WSL 的 IP 地址（hostname -I）
  "SitlPort": 4560,                  // PX4 SITL 默认端口
  "ControlIp": "<Windows_IP>",      // Windows 局域网 IP
  "LocalHostIp": "<Windows_IP>"     // AirSim 绑定的本地 IP
}
```

### 1.2 气压计配置（修复起飞高度异常）

修改前 PX4 读取的海拔约 121m（AirSim 默认模拟 Redmond, WA 海拔），导致 `ref_alt` 与实际地面高度不一致，触发 "Already higher than takeoff altitude" 拒绝起飞。

```json
"BarometerSensor1": {
  "SensorType": 1,
  "Enabled": true,
  "Pressure": 101325.0,    // 标准海平面气压 (Pa)
  "QNH": 101325.0          // 修正海平面气压 (Pa)
}
```

> `Pressure` 和 `QNH` 均设为 101325 Pa（标准大气压），PX4 计算出的参考高度接近 0m。

### 1.3 完整传感器列表

| 传感器 | 用途 | PX4 依赖 |
|--------|------|----------|
| ImuSensor1 | 陀螺仪 + 加速度计 | 必需，用于姿态估计 |
| GpsSensor1 | GPS 定位 | EKF 辅助 |
| BarometerSensor1 | 气压高度计 | 高度估计 |
| MagnetometerSensor1 | 磁罗盘 | 航向参考（仿真中关掉） |
| DistanceFront/Left/Right | 前方/左/右侧距离传感器 | 避障（可选） |
| LidarSensor1 | 32 线激光雷达 | 建图（可选） |

### 1.4 相机配置

4 路相机：front_center（前视 RGB + Depth + Segmentation）、bottom_center（下视 RGB）、front_left/front_right（前侧视 RGB）。

## 2. PX4 SITL 参数说明

以下参数仅用于解释 AirSim SITL 中可能遇到的预检问题，不是每次启动都必须设置的“一键答案”。应先使用 `commander check`、`listener vehicle_status` 和具体告警定位原因，再修改对应参数。

> 严禁把关闭电池、遥控器、数据链或 Offboard 保护的参数直接复制到真机。真机必须保留并验证遥控接管、失联、低电量和地理围栏策略。

### 2.1 传感器相关

| 参数 | 值 | 原因 |
|------|-----|------|
| `SYS_HAS_MAG` | 0 | AirSim 的磁罗盘数据不可靠，禁用以避免 "no heading reference" |
| `COM_ARM_WO_GPS` | 1 | 允许无 GPS 锁定时解锁（SITL 室内场景） |
| `SENS_BARO_QNH` | 1013.25 | 修正海平面气压 (hPa)，配合 AirSim 气压计配置 |

### 2.2 电池保护诊断

| 参数 | 值 | 原因 |
|------|-----|------|
| `BAT_CRIT_THR` | 版本相关 | 仅在 SITL 电池源确实缺失且阻塞测试时诊断 |
| `BAT_LOW_THR` | 版本相关 | 优先修复 battery_status 数据链，不应直接关闭 |
| `BAT_EMERGEN_THR` | 版本相关 | 仅用于隔离仿真问题，真机禁止关闭 |

### 2.3 失控保护诊断

| 参数 | 值 | 原因 |
|------|-----|------|
| `COM_OF_LOSS_T` | 0 | 关闭 Offboard 连接丢失超时 |
| `NAV_RCL_ACT` | 0 | 关闭遥控器信号丢失返航 |
| `NAV_DLL_ACT` | 0 | 关闭数据链丢失动作 |
| `COM_RCL_EXCEPT` | 4 | Offboard 模式下遥控器丢失不触发返航 |

### 2.4 最小化配置原则

优先只修改当前 SITL 场景确实缺失的硬件项。例如 AirSim 没有可靠磁罗盘且 PX4 明确因此拒绝初始化时，才考虑：

```
param set SYS_HAS_MAG 0
param set SENS_BARO_QNH 1013.25
param save
```

无 GPS 解锁、无遥控 Offboard 和失联动作应根据当前 PX4 版本文档与 `commander check` 单项配置。完成仿真排障后记录改动，不要长期保留来源不明的参数集合。

> `param save` 会将参数写入 `PX4-Autopilot/build/px4_sitl_default/tmp/rootfs/parameters.bson`，下次 `make px4_sitl_default` 时自动加载。

## 3. 启动顺序

推荐启动顺序如下。PX4 的 `none_iris` 会等待 AirSim 建立模拟器连接，因此先启动 PX4、再启动 AirSim 更便于观察连接日志；DDS Agent 在 ROS 节点之前就绪即可。

```
终端 1 (WSL):      启动 MicroXRCE-DDS Agent
                   micro-xrce-dds-agent udp4 -p 8888

终端 2 (WSL):      启动 PX4 SITL
                   cd ~/PX4-Autopilot
                   make px4_sitl_default none_iris

终端 3 (Windows):  启动 AirSim（UE 编辑器或打包 .exe）
                   等待无人机模型加载，并观察 PX4 出现 simulator connected

终端 4 (WSL):      启动 ROS 2 节点
                   cd ~/aeromind_ws
                   source install/setup.bash
                   ros2 launch aeromind_bringup aeromind_px4.launch.py
```

PX4 启动后确认日志中出现：
```
INFO  [simulator_mavlink] Simulator connected on TCP port 4560.
```

## 4. 网络配置

WSL2 网络隔离可能导致 IP 变化。配置前确认：

```bash
# WSL 中查询
hostname -I                          # WSL 自己 IP → AirSim settings.json 的 SitlIp
cat /etc/resolv.conf | grep nameserver  # Windows 宿主机 IP → AirSim 的 LocalHostIp
```

AirSim 的 settings.json 中 `SitlIp` 填 WSL IP，`LocalHostIp` 填 Windows IP。如果 AirSim 和 PX4 连不上，首先检查 IP 是否变化。

## 5. 常用调试命令

### 5.1 PX4 终端（pxh>）

| 命令 | 用途 |
|------|------|
| `listener vehicle_local_position` | 查看当前位姿和 ref_alt |
| `listener vehicle_status` | 查看解锁状态和 failsafe |
| `listener sensor_combined` | 查看传感器原始数据 |
| `commander arm -f` | 强制解锁（跳过所有检查） |
| `commander takeoff` | 内部起飞命令 |
| `param show` | 列出所有参数 |
| `param set <NAME> <VALUE>` | 修改参数 |

### 5.2 ROS 2

| 命令 | 用途 |
|------|------|
| `ros2 topic list \| grep fmu` | 列出 PX4 DDS 话题 |
| `ros2 topic echo /sensor/odometry --once` | 查看里程计数据 |
| `ros2 topic echo /control/drone_state --once` | 查看飞控状态 |
| `ros2 topic hz /fmu/out/sensor_combined` | 检查传感器数据频率 |
| `ros2 service call /control/arm ...` | 解锁 |
| `ros2 service call /control/takeoff ...` | 起飞 |

## 6. 常见问题

### AirSim 连不上 PX4（TCP:4560 不通）

1. 检查 Windows 防火墙：管理员 PowerShell 执行 `New-NetFirewallRule -DisplayName "AirSim PX4" -Direction Inbound -Protocol TCP -LocalPort 4560 -Action Allow`
2. 检查 IP：WSL 重启后 IP 可能变化，更新 settings.json 中 `SitlIp`
3. 检查 PX4 与 AirSim 两侧是否都在运行；必要时保持 PX4 运行并重启 AirSim

### ROS 2 收不到 PX4 数据

1. 确认 MicroXRCE-DDS Agent 在运行
2. 确认 `px4_msgs` 包已编译：`ros2 pkg list | grep px4_msgs`
3. 检查 bridge 日志无 QoS 警告

### 解锁后立即自动加锁

PX4 日志中查看原因（`Disarmed by ...`），通常是某个安全参数没关。

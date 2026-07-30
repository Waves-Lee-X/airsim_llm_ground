# 手动 AirSim/ArduCopter SITL 操作手册

本流程用于在任意启用了 AirSim 插件的 UE 4.27 场景中进行交互式仿真。它与
自动 M1 验收并存，只生成 `settings.json` 并打印 SITL 命令，不负责启动、监视或
关闭 UE、AirSim 和 SITL。

## 1. 前置条件

- UE 工程已启用 AirSim 1.8.1 插件；
- ArduPilot 默认位于 `/home/waves/ardupilot`，否则传入 `--ardupilot-root`；
- Windows 防火墙允许 UE/AirSim 与 WSL 交换 UDP；
- 场景出生点有足够净空，地形和碰撞体经过检查；
- Windows `Documents/AirSim/settings.json` 可写。

配置不依赖 Blocks。楼房、街道、球门和隧道应由当前 UE 场景提供，生成器不会
自动创建场景资产。

## 2. 生成 settings.json

WSL NAT 地址重启后可能变化，因此每次 WSL 重启后重新生成：

```bash
cd ~/aeromind_ws/apm_lite
PYTHONPATH=src python3 -m \
  aeromind_apm_lite.ground.simulation.manual_settings \
  --output /mnt/c/Users/<Windows用户名>/Documents/AirSim/settings.json
```

生成器自动选择：

- 当前 WSL IPv4 作为 AirSim `UdpIp`；
- Windows 默认网关作为 ArduCopter `--sim-address`；
- AirSim 传感器端口 `9003`、控制端口 `9002`；
- SITL MAVLink 输出 `127.0.0.1:14550`。

自动发现错误时显式覆盖：

```bash
PYTHONPATH=src python3 -m \
  aeromind_apm_lite.ground.simulation.manual_settings \
  --output /mnt/c/Users/<Windows用户名>/Documents/AirSim/settings.json \
  --wsl-ip 172.24.80.10 \
  --windows-host-ip 172.24.80.1
```

输出 JSON 中包含解析后的地址和 `sitl_command`，并固定报告
`processes_started=false`。

## 3. 启动顺序

1. 生成最新 `settings.json`。
2. 打开目标 UE 场景并进入 Play。
3. 在 WSL 新建独立 SITL 状态目录，进入该目录后执行生成器打印的
   `sitl_command`。
4. 等待 ArduCopter 与 AirSim 交换传感器数据。
5. 在第二个 WSL 终端启动 Lite 地面站：

```bash
cd ~/aeromind_ws/apm_lite
PYTHONPATH=src python3 -m aeromind_apm_lite.ground.browser.app \
  --mode sitl --host 127.0.0.1 --port 8000
```

6. 打开 `http://127.0.0.1:8000/`，确认 Agent、FCU 和相机状态。
7. 结束时由操作者依次停止地面站、SITL 和 UE。

地面站不拥有外部进程生命周期，不能用关闭 Web 代替停止 SITL 或 UE。

## 4. 相机基线

默认 `front_center` 是前视 Scene RGB 相机：

- `ImageType=0`；
- 1280 x 720；
- 水平视场角 90 度；
- 位姿可配置；
- 不声明 D435、Depth 或激光雷达。

修改参数示例：

```bash
PYTHONPATH=src python3 -m \
  aeromind_apm_lite.ground.simulation.manual_settings \
  --output /mnt/c/Users/<Windows用户名>/Documents/AirSim/settings.json \
  --camera-width 640 --camera-height 480 --camera-fov 78 \
  --camera-x 0.30 --camera-z -0.08 --camera-pitch -5
```

只有在实机相机分辨率、视场角和安装外参完成测量后，才能据此对齐虚实相机。

## 5. 图像与控制边界

地面站通过 AirSim RPC 读取 Scene 图像，隔离超时并缓存最新帧，接口为
`/api/camera/frame`。AirSim RPC 不用于飞行控制；所有飞行动作仍经过
Web -> OnboardAgent -> ApmLink -> MAVLink -> ArduCopter。

指定 RPC 地址或相机：

```bash
PYTHONPATH=src python3 -m aeromind_apm_lite.ground.browser.app \
  --mode sitl --host 127.0.0.1 --port 8000 \
  --airsim-host 172.24.80.1 --airsim-port 41451 \
  --airsim-vehicle Drone1 --airsim-camera front_center
```

FCU 单独测试可添加 `--disable-camera`。相机在线只表示能够取帧，不表示 VLM、
深度或避障已经可用。

## 6. 常见问题

| 现象 | 检查项 |
|---|---|
| FCU 离线 | SITL 是否输出到 14550，端口是否被其他进程占用 |
| AirSim 无响应 | UE 是否进入 Play，RPC 地址是否为 Windows 网关 |
| 无法解锁 | GPS/EKF/pre-arm 状态、出生点碰撞、参数文件是否匹配 |
| 相机黑屏 | 相机名、车辆名、RPC 41451、防火墙和 Scene 图像类型 |
| WSL 重启后失联 | 重新生成 `settings.json`，不要继续使用旧 NAT 地址 |

不得通过关闭预检参数掩盖连接、定位或场景碰撞问题。

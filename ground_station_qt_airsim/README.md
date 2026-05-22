# AirSim 基础地面站

这是一套独立的 AirSim Qt 地面站，当前主线直接通过 AirSim Python API / RPC 控制仿真无人机，不依赖串口、UDP 或 MAVLink 链路。

## 启动

请先回到仓库根目录：

```powershell
cd D:\workspace\airsim_llm_ground
```

安装依赖：

```powershell
pip install airsim PySide6 PySide6-Addons pyyaml numpy pywin32
```

`win32gui` 来自 `pywin32`，不要执行 `pip install win32gui`。

如果虚拟环境里的 `pip.exe` 报错并指向旧目录，请使用当前解释器安装：

```powershell
py -3.10 -m pip install pywin32
```

如果仍然失败，建议重建当前项目虚拟环境：

```powershell
cd D:\workspace\airsim_llm_ground
deactivate
Rename-Item .venv .venv_broken
py -3.10 -m venv .venv
.\.venv\Scripts\activate
python -m pip install --upgrade pip
python -m pip install airsim PySide6 PySide6-Addons pyyaml numpy pywin32
```

启动地面站，推荐使用模块方式：

```powershell
python -m ground_station_qt_airsim --config config_airsim.yaml
```

也可以使用兼容脚本：

```powershell
python ground_station_qt_airsim.py --config config_airsim.yaml
```

如果你已经进入了 `ground_station_qt_airsim` 子目录，请先执行：

```powershell
cd ..
```

再运行上面的启动命令。不要在子目录里执行 `python ground_station_qt_airsim.py`，那里没有这个脚本。

## 当前能力

- AirSim RPC 连接与车辆自动发现
- 遥测状态回传
- 解锁 / 上锁 / 起飞 / 降落 / 返航 / 悬停
- AirSim 本地坐标地图显示与单点导航
- 航点任务
- 基于 LiDAR 的专家避障指点导航
- 简单编队跟随
- UE/AirSim 场景窗口嵌入地面站显示区域

## 配置

可在根目录的 `config_airsim.yaml` 中调整：

- `airsim.host`
- `airsim.port`
- `airsim.vehicle_names`
- `airsim.takeoff_height_m`
- `airsim.default_speed_mps`
- `formation.enabled`
- `formation.default_spacing_m`

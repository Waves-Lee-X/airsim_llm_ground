# AirSim 基础地面站

这是一套独立的 AirSim Qt 地面站，不会影响原有实机和 SITL 主线。

## 启动

```powershell
pip install -r requirements-airsim.txt
python ground_station_qt_airsim.py --config config_airsim.yaml
```

## 当前能力

- 直接通过 AirSim Python API / RPC 控制飞机
- 不复用串口、UDP、MAVLink 那套链路
- 当前只保留基础能力：
- 遥测状态回传
- 解锁 / 上锁 / 起飞 / 降落 / 返航 / 悬停
- 地图单点导航
- 简单航点队列

## 配置

可在 `config_airsim.yaml` 中调整：

- `airsim.host`
- `airsim.port`
- `airsim.vehicle_names`
- `airsim.takeoff_height_m`
- `airsim.default_speed_mps`

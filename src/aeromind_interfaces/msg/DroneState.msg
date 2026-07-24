# 无人机飞控状态
bool armed              # 是否解锁
string mode             # 飞行模式
float32 battery         # 电池电压 (V)
int8 gps_fix            # GPS 定位类型 (0=无, 1=2D, 2=3D, 3=DGPS, 4=RTK)
bool ekf_healthy        # EKF 融合状态是否正常
bool preflight_ok       # PX4 是否通过全部解锁前健康检查
bool preflight_valid    # preflight_ok 是否来自有效的 VehicleStatus 遥测
bool landed             # 飞控是否确认已接地
bool landed_valid       # landed 是否来自有效的接地检测遥测

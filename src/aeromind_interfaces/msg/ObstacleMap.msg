# 障碍物占据栅格地图
builtin_interfaces/Time timestamp   # 地图时间戳
uint32 width                         # 地图宽度 (格数)
uint32 height                        # 地图高度 (格数)
float32 resolution                   # 分辨率 (米/格)
uint8[] data                         # 栅格数据 (0=空闲, 100=占据, -1=未知)

---
name: mission-safe-takeoff-capture
description: Perform a deterministic safety check, take off, and capture the forward image.
---

# 安全起飞拍照

适用于“检查安全后起飞并拍照”任务。

参数：`altitude`、`minimum_obstacle_distance`、`require_gps`。

流程：读取飞控与避障状态；检查通过后起飞到目标高度；到达后保存前视 RGB 图像。检查失败时停止，不会跳过检查继续起飞。

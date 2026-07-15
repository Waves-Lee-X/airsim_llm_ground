---
name: flight-square
description: Compose a closed square flight path from relative movement actions.
---

# 正方形轨迹

适用于“飞一个边长 N 米的正方形”任务。

参数：`side_length`、`altitude`、`takeoff_if_needed`、`land_after`。

流程：必要时起飞，依次执行前、右、后、左四个相对移动航段，最后按参数悬停或降落。整套任务只创建一次人工确认。

约束：边长 1 到 50 米，高度 1 到 30 米；移动由自主规划器执行，不绕过避障与飞控保护。

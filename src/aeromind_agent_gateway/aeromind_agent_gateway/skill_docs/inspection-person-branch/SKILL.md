---
name: inspection-person-branch
description: Inspect for people and branch to hover-and-capture or continue forward.
---

# 人员条件巡检

适用于“发现人就悬停拍照，否则继续前进”任务。

参数：`distance`、`minimum_confidence`。

流程：读取当前 YOLO 检测；若检测到 `person`，执行悬停并调用 `capture_image`；若明确未检测到，继续向前移动指定距离；若感知数据不可用，保持悬停。

分支依据来自 `perception_check` 的结构化 `matched` 结果，不允许模型凭文字推测。检测数据过期或服务失败时，工作流不会把“未知”当作“无人”。

约束：前进距离 0.5 到 100 米，置信度 0 到 1；整套任务只创建一次人工确认。

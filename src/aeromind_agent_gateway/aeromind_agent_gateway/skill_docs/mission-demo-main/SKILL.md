---
name: mission-demo-main
description: Run the fixed, repeatable contest demonstration mission with one real confirmation.
---

# 固定演示主链

适用于重复稳定性测试和正式演示。

参数：`altitude`、`distance`、`minimum_obstacle_distance`、`require_gps`。

流程：确定性检查飞控和起飞区域；通过后记录本轮任务起点并起飞到目标高度；向机头前方移动；悬停并保存图像；使用 VLM 分析到达位置的新画面；经自主规划返回本轮起点上方并校验水平误差；生成状态与感知报告；最后原生降落。

整个流程只创建一次真实确认。移动步骤不自动重试。拍照、VLM 或报告失败会被记录，但不会阻断安全回程。回程失败时 Workflow 停止并保持悬停，不会假装已经回到起点；确认回到起点容差内后才执行降落。

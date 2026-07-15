---
name: inspection-line-capture
description: Fly a straight inspection leg and capture images at both ends.
---

# 直线航线拍照巡检

适用于“沿前方巡检一段距离并在两端拍照”任务。

参数：`distance`、`altitude`、`takeoff_if_needed`、`return_after`、`land_after`。

流程：必要时起飞；保存起点图像；沿机头前方飞行指定距离；保存终点图像；按参数原路返回并降落。移动仍由自主规划器和避障控制器执行。

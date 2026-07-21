---
name: inspection-line-capture
description: Fly a straight inspection leg and capture images at both ends.
---

# 直线航线拍照巡检

适用于“沿前方巡检一段距离并在两端拍照”任务。

参数：`distance`、`altitude`、`takeoff_if_needed`、`return_after`、`land_after`。

流程：必要时起飞；保存起点图像；沿机头前方执行连续航点；保存终点图像；按参数原路返回并降落。航段使用 Minimum Snap 轨迹。确认状态的人员进入剩余航迹时，底层确定性语义保护会悬停，Workflow 保存现场图像；人员离开并稳定达到门限后，从当前位置重拼接剩余轨迹继续飞行。持续占用超过时限则终止航段并保持悬停。

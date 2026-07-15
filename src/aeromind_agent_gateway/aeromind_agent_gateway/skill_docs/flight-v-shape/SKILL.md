---
name: flight-v-shape
description: Compose a heading-relative V-shaped path and optionally capture an image at its vertex.
---

# V 字轨迹

适用于“飞 V 字形”“在 V 字顶点拍照”等任务。

参数：`width`、`depth`、`altitude`、`takeoff_if_needed`、`capture_at_vertex`、`land_after`。

流程：必要时起飞；第一个向量航段飞向 V 字顶点；可选执行 `capture_image`；第二个向量航段飞向另一端；最后按参数悬停或降落。

坐标：`forward_m`、`right_m`、`up_m` 均为机头朝向相对坐标，单位为米。轨迹由两个真实对角向量航段组成。

约束：宽度和深度 1 到 50 米，高度 1 到 30 米；执行前必须由用户确认。

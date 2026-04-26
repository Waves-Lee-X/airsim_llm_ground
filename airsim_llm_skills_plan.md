# 单 AirSim 场景下基于大模型的无人机自主操控方案

## 1. 项目目标

本方案面向单 AirSim 仿真场景，目标是构建一套“自然语言驱动无人机执行任务”的系统。系统不让大模型直接生成底层飞行控制代码，而是让大模型作为任务规划与调度中心，根据用户自然语言选择合适的无人机技能，并由技能模块调用 AirSim API、强化学习策略或目标识别模型完成具体动作。

核心目标包括：

- 支持用户通过自然语言下达无人机任务。
- 将无人机能力封装为可复用的 skills。
- 基础飞行动作直接调用 AirSim API。
- 复杂路径规划与避障能力接入已有强化学习模型。
- 使用预训练目标识别模型增强无人机感知能力。
- 建立可解释、可调试、可逐步扩展的自主任务执行框架。

## 2. 总体思路

系统采用“LLM 规划 + Skill 调用 + AirSim 执行 + 感知反馈”的分层结构。

```text
用户自然语言
  -> 大模型解析任务意图
  -> 生成结构化任务计划
  -> Skill Registry 匹配技能
  -> 调用 AirSim / RL / 目标识别模块
  -> 获取状态反馈
  -> 继续执行、调整或向用户汇报
```

大模型的角色不是直接控制无人机的速度、电机或姿态，而是负责任务理解、步骤拆解、技能选择和多步任务编排。底层执行由确定性的 skill 完成，这样系统更容易调试，也更适合课程展示和论文/报告描述。

## 3. 系统架构

建议新增独立模块，不污染现有实机和 SITL 代码。

```text
ground_station_qt_airsim/
  llm/
    agent.py                 # 大模型任务调度器
    command_parser.py        # 自然语言 -> 结构化 JSON 计划
    prompts.py               # 系统提示词与 skill 描述

  skills/
    registry.py              # skill 注册表
    base_flight.py           # 起飞、降落、悬停、返航、Z 坐标移动
    navigation.py            # 点到点导航、航点任务
    rl_navigation.py         # 强化学习路径规划与避障 skill
    perception.py            # 目标检测、图像识别、目标定位
    mission.py               # 搜索、巡检、靠近目标等复合任务

  safety/
    guard.py                 # 限高、限速、超时、碰撞保护
    validators.py            # LLM 输出参数校验

  runtime/
    context.py               # 当前无人机状态、目标缓存、任务上下文
    executor.py              # 执行结构化计划
```

现有 `ground_station_qt_airsim/backend.py` 可以继续作为 AirSim 执行后端，后续 skills 只需要调用 backend 暴露出的控制方法。

## 4. Skill 设计

### 4.1 基础飞行 Skills

基础飞行 skill 直接调用 AirSim API，适合作为第一阶段实现内容。

| Skill 名称 | 功能 | 推荐 AirSim API |
|---|---|---|
| `takeoff` | 解锁并起飞到默认高度 | `armDisarm`, `takeoffAsync`, `moveToZAsync` |
| `land` | 降落 | `landAsync` |
| `hover` | 悬停 | `hoverAsync` |
| `go_home` | 返航 | `goHomeAsync` |
| `arm` | 解锁 | `armDisarm(True)` |
| `disarm` | 上锁 | `armDisarm(False)` |
| `move_to_position` | 飞到 AirSim 本地坐标点 | `moveToPositionAsync` |
| `move_to_z` | 改变 AirSim Z 坐标 | `moveToZAsync` |
| `get_state` | 获取无人机状态 | `getMultirotorState`, `getGpsData` |

AirSim 使用 NED 坐标系，`Z` 为负数时表示向上。例如 `z=-8` 通常表示离初始平面约 8 米高度。后续所有自然语言中的“高度”都应在解析阶段转换为明确的 AirSim `Z` 坐标，避免正负方向混乱。

### 4.2 导航 Skills

导航 skill 负责将目标点转换为 AirSim 可执行动作。

建议包含：

- `goto_local(x, y, z)`：飞到 AirSim 本地坐标。
- `goto_map_point(lat, lon, z)`：从地图点击点换算为本地坐标后飞行。
- `run_waypoints(points)`：执行航点序列。
- `stop_motion()`：中止当前任务并悬停。

第一版可以只实现点到点直线飞行，第二版再加入 RL 避障或视觉导航。

## 5. 强化学习路径规划与避障接入

你之前训练的强化学习模型非常适合封装为一个独立 skill，而不是替代所有飞行控制。

推荐封装为：

```text
navigate_with_rl(target_x, target_y, target_z, max_time_s, safety_radius_m)
```

执行流程如下：

```text
读取当前状态
  -> 构造 RL observation
  -> RL policy 输出动作
  -> 安全检查
  -> 调用 AirSim 速度或位置控制 API
  -> 检查是否到达目标
  -> 循环直到成功、超时或触发安全停止
```

RL skill 的输入建议包括：

- 当前无人机本地坐标：`x, y, z`
- 当前速度：`vx, vy, vz`
- 目标点坐标：`target_x, target_y, target_z`
- 障碍物信息：深度图、激光雷达、距离传感器或环境状态
- 任务约束：最大速度、最小高度、最大执行时间

RL skill 的输出建议限制为：

- `vx, vy, vz, duration`
- 或 `next_x, next_y, next_z`

建议不要让 RL 模型直接无限制控制无人机。外层必须加安全保护：

- 最大速度限制
- 最大/最小 Z 坐标限制
- 碰撞检测
- 超时停止
- 距离目标收敛判断
- 异常时自动悬停

这样可以在展示时说明：大模型负责任务规划，RL 负责复杂运动策略，安全层负责约束执行。

## 6. 目标识别能力接入

目标识别可以作为 perception skills 接入，用于增强无人机“看见目标、理解目标、围绕目标执行任务”的能力。

推荐第一阶段使用预训练模型，不必重新训练。

可选模型：

| 模型 | 适合任务 |
|---|---|
| YOLOv8 / YOLOv11 | 常见目标检测，速度快，适合实时演示 |
| GroundingDINO | 文本提示目标检测，例如“红色车辆”“建筑入口” |
| CLIP | 图文匹配、开放词汇分类 |
| SAM / FastSAM | 目标分割、区域提取 |

AirSim 图像获取方式：

```text
AirSim 相机图像
  -> simGetImages
  -> OpenCV / PIL 解码
  -> 目标检测模型
  -> 输出目标类别、置信度、图像坐标
  -> 估计目标方向或位置
  -> 交给导航 skill
```

建议封装的 perception skills：

```text
capture_image(camera_name)
detect_objects(labels)
find_target(text_prompt)
track_target(target_id)
estimate_target_direction(target)
```

后续可组合成任务：

```text
“起飞，搜索车辆，找到后靠近并悬停观察”
```

对应计划：

```json
{
  "steps": [
    {"skill": "takeoff", "args": {"z": -8}},
    {"skill": "find_target", "args": {"text": "vehicle"}},
    {"skill": "navigate_to_detected_target", "args": {"z": -8}},
    {"skill": "hover", "args": {"duration_s": 5}}
  ]
}
```

## 7. 大模型调度方式

大模型输出应当是结构化 JSON，而不是自由文本或 Python 代码。

示例：

```json
{
  "task": "让无人机起飞到 8 米高度，然后飞到前方目标点并悬停",
  "steps": [
    {"skill": "takeoff", "args": {"z": -8}},
    {"skill": "goto_local", "args": {"x": 20, "y": 0, "z": -8}},
    {"skill": "hover", "args": {"duration_s": 5}}
  ]
}
```

执行前需要做参数校验：

- skill 是否存在
- 参数是否完整
- Z 坐标是否在安全范围内
- 速度是否超过限制
- 任务是否需要用户确认
- 是否正在执行其他任务

建议设置确认机制：

- `takeoff`、`land`、`go_home`、`navigate_with_rl` 需要确认
- `get_state`、`detect_objects`、`hover` 可以直接执行

## 8. 推荐开发阶段

### 第一阶段：基础 skill 闭环

目标是完成自然语言到基础 AirSim 控制的闭环。

任务：

- 建立 `SkillRegistry`
- 实现基础飞行 skills
- 实现自然语言解析到 JSON plan
- 在 Qt 地面站中加入自然语言输入框
- 展示执行日志与状态回传

验收例子：

```text
“让 UAV1 起飞到 Z=-8，然后悬停”
“让 UAV1 返航”
“让 UAV1 飞到 X=20 Y=5 Z=-8”
```

### 第二阶段：RL 避障 skill

目标是把已有强化学习模型作为能力模块接入。

任务：

- 梳理 RL 模型输入 observation
- 封装模型加载和推理接口
- 实现 `navigate_with_rl`
- 加入安全层
- 在 AirSim 中构建障碍场景测试

验收例子：

```text
“避开障碍飞到前方目标点”
“使用自主避障飞到 X=30 Y=10 Z=-8”
```

### 第三阶段：目标识别 skill

目标是让无人机能通过视觉寻找目标。

任务：

- 接入 AirSim 相机图像
- 接入预训练目标检测模型
- 实现 `detect_objects` 和 `find_target`
- 将检测结果显示在地面站日志或图像窗口中
- 将目标位置传给导航模块

验收例子：

```text
“寻找车辆”
“找到目标后靠近并悬停”
```

### 第四阶段：复合自主任务

目标是让大模型组合多个 skills 完成任务。

任务：

- 实现多步任务 executor
- 支持任务中断、失败回退和状态汇报
- 增加任务上下文 memory
- 支持“搜索-识别-靠近-悬停-返航”流程

验收例子：

```text
“起飞，搜索场景中的车辆，靠近它并保持 5 秒悬停，然后返航”
```

## 9. 演示路线

推荐最终演示分三段。

第一段展示基础自然语言控制：

```text
用户：让一号机起飞到 Z=-8
系统：解析为 takeoff(z=-8)，调用 AirSim API，状态回传更新
```

第二段展示强化学习避障：

```text
用户：避开障碍飞到目标点
系统：调用 navigate_with_rl，RL 模型输出动作，安全层限制速度和高度
```

第三段展示目标识别与复合任务：

```text
用户：搜索车辆，找到后靠近并悬停
系统：调用 detect_objects，找到目标后调用导航 skill，最后 hover
```

## 10. 风险与控制措施

| 风险 | 解决方式 |
|---|---|
| 大模型输出不稳定 | 强制 JSON schema，执行前校验 |
| AirSim Z 坐标正负混乱 | 全系统统一使用 AirSim NED 坐标，界面显示目标 Z |
| RL 模型动作过激 | 增加速度限制、边界限制、超时和自动悬停 |
| 目标识别误检 | 使用置信度阈值，多帧确认 |
| 任务执行中断 | executor 支持取消、暂停和失败回退 |
| 大模型误触发危险动作 | 起飞、降落、返航、RL 自主飞行前要求确认 |

## 11. 最小可行版本

最小可行版本建议包含：

- Qt AirSim 地面站
- AirSim 基础控制 backend
- `SkillRegistry`
- 基础飞行 skills
- 简单自然语言解析
- JSON plan 执行器
- 状态回传显示

最小 demo 示例：

```text
输入：让 UAV1 起飞到 Z=-8，然后悬停

输出计划：
[
  {"skill": "takeoff", "args": {"z": -8}},
  {"skill": "hover", "args": {"duration_s": 5}}
]

执行结果：
无人机起飞，状态回传显示 Z 坐标、速度、位置，任务完成后悬停。
```

## 12. 后续扩展

后续可以逐步扩展：

- 多无人机协同任务
- 大模型根据状态回传动态调整任务
- 目标检测结果可视化
- RL 与传统路径规划混合
- 任务执行记录保存
- 失败案例回放
- 对不同场景训练或微调专用策略

本方案的核心价值在于把大模型、AirSim API、强化学习避障和视觉感知分工明确地组合起来。大模型负责理解和规划，skills 负责执行，RL 负责复杂运动策略，目标识别负责环境感知，安全层负责边界约束。这种结构既适合工程实现，也适合课程汇报和论文式阐述。

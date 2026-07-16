# Agent、Skills 与 Workflow

本文解释项目如何让大模型调用无人机能力，以及基础动作、Skill 文档、Workflow 和 ROS 执行层之间的关系。

## 1. 核心原则

大模型擅长理解开放语言、组合任务和解释结果，但不适合直接输出高频电机控制量。项目采用分层设计：

```text
自然语言
→ LLM 生成结构化任务或 Workflow
→ 能力注册表校验
→ 确认中心处理风险动作
→ ROS Service/Action/Topic
→ PX4/Autonomy 执行
→ 遥测验证
```

这样既体现模型能力，又保留确定性、安全性和可测试性。

## 2. 三层能力模型

### 2.1 基础能力 Capability

基础能力是代码真实实现的原子动作，例如：

- `get_drone_state`
- `get_odometry`
- `get_camera_summary`
- `get_detections`
- `arm`
- `takeoff`
- `land`
- `return_home`
- `move_to`
- `follow_waypoints`
- `hover`
- `capture_image`
- `analyze_image`

每个能力必须定义输入、输出、风险等级、超时、取消方式和物理完成条件。没有底层代码的能力不能仅靠 Markdown 变成真实动作。

### 2.2 Skill

Skill 描述“怎样正确使用一组基础能力”。它可以包含：

- 适用意图和示例任务。
- 前置条件和安全约束。
- 参数提取规则。
- 推荐执行步骤。
- 失败恢复和完成标准。
- 允许使用的工具列表。

Skill 目录位于：

```text
src/aeromind_agent_gateway/aeromind_agent_gateway/skill_docs/
```

当前组合技能包括：

| Skill | 作用 |
|---|---|
| `flight-square` | 连续正方形多航点飞行 |
| `flight-v-shape` | V 字轨迹与顶点任务 |
| `inspection-line-capture` | 直线巡检和拍照 |
| `inspection-person-branch` | 发现人员则悬停拍照，否则继续 |
| `mission-safe-takeoff-capture` | 安全检查、起飞和取证组合 |

### 2.3 Workflow

Workflow 是一次任务实例的结构化执行图。它将 Skill 具体化为步骤、参数、依赖和条件。

```json
{
  "name": "人员条件巡检",
  "steps": [
    {"id": "check", "action": "safety_check"},
    {"id": "takeoff", "action": "takeoff", "args": {"altitude": 10}},
    {"id": "detect", "action": "perception_check", "args": {"target": "person"}},
    {
      "id": "capture",
      "action": "capture_image",
      "condition": "detect.matched == true"
    },
    {
      "id": "move",
      "action": "move_to",
      "args": {"forward_m": 20},
      "condition": "detect.matched == false"
    }
  ]
}
```

实际字段以 `workflow.py` 和能力注册表为准；模型输出仍需严格 Schema 校验。

## 3. 原来的规则解析器做什么

规则解析器用于高频、明确、可预测的基础命令，例如“起飞到 10 米”“降落”“返航”。优点是速度快、无网络依赖、结果确定；缺点是难以覆盖自由表达和复杂条件。

当前推荐采用混合路由：

| 输入 | 处理方式 |
|---|---|
| 明确白名单命令 | 确定性路由，直接创建真实确认请求 |
| 已注册组合 Skill | Skill 模板 + 参数校验 + Workflow |
| 开放复杂任务 | LLM 规划，再由能力注册表验证 |
| 无法映射到底层能力 | 解释缺失能力，不执行 |

完全删除所有规则并不能自动提升智能，反而会让起飞、降落和确认变得不稳定。更合理的做法是让规则负责安全底座，让 Skill/LLM 负责组合和长尾表达。

## 4. 大模型可以组合出的任务

只要基础能力完整，少量原子动作可以组合出大量任务：

- 起飞后飞正方形、V 字形、折线或自定义多航点。
- 到每个航点拍照，最后生成任务报告。
- 检查前方，发现人则悬停拍照，否则继续前进。
- 搜索车辆，若未发现则改变观察点并再次检测。
- 电池低于阈值则中止巡检并 RTL。
- 某个航点阻塞时局部重规划，重试失败则悬停。
- 先巡检建筑立面，再按异常位置生成复查任务。

模型的价值在于根据目标和状态动态选择步骤，而不是把每一种任务都硬编码成单独函数。

## 5. 正方形和 V 字形是否写死

轨迹几何可以由代码参数化生成，也可以由模型产生航点。推荐分工：

- Skill 定义几何语义、参数范围和任务流程。
- 确定性代码根据宽、深、高度计算航点。
- `FollowWaypoints` Action 执行连续轨迹。
- Autonomy 负责局部避障和剩余轨迹拼接。
- 模型决定何时使用这个 Skill、参数是多少、是否附加拍照/返航。

这比让模型直接输出几十个 setpoint 更稳定，也比为每种尺寸写一个函数更通用。

## 6. Skill 文档建议格式

一个组合技能目录包含 `SKILL.md` 和 `skill.yaml`：

```text
skill_docs/my-skill/
├── SKILL.md
└── skill.yaml
```

`skill.yaml` 建议包含：

```yaml
id: inspection.example
name: 示例巡检
version: 1
risk_level: high
enabled: true
tools:
  - safety_check
  - takeoff
  - move_to
  - capture_image
parameters:
  distance_m:
    type: number
    minimum: 1
    maximum: 50
```

`SKILL.md` 应回答：

1. 这个技能解决什么任务。
2. 什么时候允许调用。
3. 参数如何解释，坐标系是什么。
4. 需要哪些前置状态。
5. 按什么顺序执行。
6. 每一步如何判断完成。
7. 失败时重试、悬停、降落还是 RTL。
8. 最终向用户返回哪些证据。

## 7. 新增 Skill 的流程

1. 确认所需基础能力已真实存在。
2. 在能力注册表中定义参数 Schema、风险和执行适配器。
3. 编写 `skill.yaml` 和 `SKILL.md`。
4. 注册 Skill，使 `/api/skills`、模型工具和 Web 能力库共用同一目录。
5. 编写正常、参数越界、取消、超时和恢复测试。
6. 在仿真中验证物理完成条件，而不只检查函数返回。

团队可通过 `AEROMIND_GATEWAY_SKILL_MODULES` 注册独立技能模块，避免多人同时修改核心路由器。

## 8. 确认机制

高风险动作采用两阶段执行：

```text
请求阶段：生成计划和确认记录，不控制无人机
确认阶段：校验用户、会话、有效期、意图和参数，再执行真实动作
```

确认中心应满足：

- 持久化，刷新页面后仍能恢复。
- 与用户和会话绑定，防止跨用户确认。
- 幂等，同一个 ID 不能重复执行。
- 过期后拒绝执行。
- 显示完整任务摘要和风险。
- 确认后不重新让 LLM 猜测原始意图。

## 9. Workflow 状态与恢复

建议状态：

```text
pending → awaiting_confirmation → running
running → paused → running
running → succeeded / failed / cancelled / interrupted
```

每一步应记录：

- `step_id`、能力、参数和依赖。
- 开始/结束时间。
- ROS 请求是否受理。
- 物理完成证据。
- 重试次数和失败原因。
- 截图、检测和报告等产物路径。

进程重启后，不应自动恢复飞行中的高风险 Workflow；应标记为 `interrupted`，由操作员检查现场后重新规划。

## 10. LLM、VLM、YOLO 和 VLA 的边界

| 模型 | 当前作用 | 不应承担 |
|---|---|---|
| LLM | 意图理解、任务规划、工具选择、报告解释 | 高频姿态和电机控制 |
| VLM | 单帧语义理解、开放类别描述、风险建议 | 精确测距和唯一避障依据 |
| YOLO | 低延迟固定类别检测 | 开放世界推理和复杂任务规划 |
| VLA | 当前仅作为后续研究方向 | 未经 Shadow Mode 验证直接控真机 |

大模型真正的提升方向不是替代 PX4，而是建立“证据化任务规划、语义世界模型、失败恢复和经验记忆”。详见 [限制与升级路线](07-限制与升级路线.md)。

## 11. 调试入口

```bash
curl http://localhost:8090/api/skills
curl http://localhost:8080/api/status
```

具体 API 以 Gateway 路由实现为准。调试确认问题时，同时观察浏览器 WebSocket、Agent Gateway 日志、ROS Agent 日志和 `/autonomy/status`，不要只看模型回复文本。

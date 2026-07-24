# Agent 参赛评测数据

本目录保存自然语言任务规划评测集和可提交的基线摘要，不包含 API Key、ROS 遥测或真实飞行控制。

## 文件

- `agent_tasks_v1.jsonl`：第一版 80 条人工标注中文任务。
- `rules-baseline.md`：当前确定性规则路由的可复现基线结果。

## 标签字段

| 字段 | 含义 |
|---|---|
| `id` | 全局唯一案例编号 |
| `category` | 状态感知、原子飞行、Workflow、边界安全或对抗任务 |
| `input` | 用户自然语言输入 |
| `expected_intent` | 期望任务意图 |
| `expected_args` | 需要精确评分的参数子集 |
| `expected_confirm` | 是否必须经过确认 |
| `expected_actions` | 原子任务的有序动作 |
| `expected_workflow_actions` | 组合任务的有序步骤 |
| `expected_rejected` | 是否应拒绝生成执行计划 |

仅在案例显式提供参数或动作标签时计算对应准确率，避免大量空标签虚高指标。

## 运行

```bash
cd ~/aeromind_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run aeromind_agent_gateway agent_evaluate \
  --dataset evaluation/agent_tasks_v1.jsonl \
  --mode rules \
  --output /tmp/rules.json \
  --csv /tmp/rules.csv \
  --markdown /tmp/rules.md
```

模型模式为 `llm_free`、`llm_schema` 和 `hybrid`。它们会把测试文本发送给配置的模型 Provider，并产生 Token 消耗，但不会连接 ROS 或执行飞行动作。建议先使用 `--limit 5` 验证模型输出格式。

四组报告生成后，可汇总为对比表、中文结论和三张图：

```bash
python3 evaluation/generate_comparison.py \
  --input-dir missions/contest/agent-eval-20260724-v2
```

汇总器会校验四组样本数和 Git commit 是否一致，并在输入目录生成
`comparison.json`、`comparison.csv`、`comparison.md` 以及准确率、效率和任务分类
三张 PNG 图表。

## 评测规范

1. 四种模式必须使用同一数据集。
2. 固定模型版本、温度、Prompt 和能力目录版本。
3. 每次报告记录 Git commit、运行时间和 Provider/模型。
4. 修改标签必须经过人工复核并说明原因。
5. 不因某一模型的输出习惯临时修改标签。
6. 对外材料同时报告准确率、样本数、失败案例和延迟。

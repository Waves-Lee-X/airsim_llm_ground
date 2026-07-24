# 参赛实验结果

本目录只归档已经确定用于参赛材料的最终实验结果。ROS 运行期间生成的
截图、临时报告、数据库和中间测试仍保存在 `missions/`，不纳入 Git。

## 目录

- `agent-eval-20260724-v2/`：同一版本、同一 80 条数据集上的 Rules、
  LLM Free、LLM + Schema 和 Hybrid 四组离线规划实验。
- `physical-flight-20260724/`：10 次 ROS/PX4 完整任务链稳定性实验。

## 结论边界

- Agent 离线实验评估自然语言意图、参数、Workflow、确认与安全拒绝，
  不连接 ROS，也不执行飞行动作。
- 物理闭环实验评估固定演示任务在 ROS/PX4 链路中的执行可靠性。
- 两组结果回答不同问题，不应把离线规划准确率表述为飞行成功率。

## 重新生成 Agent 汇总

```bash
python3 evaluation/generate_comparison.py \
  --input-dir evaluation/results/agent-eval-20260724-v2
```

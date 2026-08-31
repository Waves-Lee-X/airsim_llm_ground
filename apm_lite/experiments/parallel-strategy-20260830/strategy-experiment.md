# L0四策略多种子实验报告

- 实验矩阵：4种策略 × 5类场景 × 10个种子 = 200次有效运行
- 重复性复核：另执行200次；哈希全部一致：True
- 对象数量：32；单次仿真时长：120 s
- 内容SHA-256：`f268ed696bdf95c1a4c4f2bda07a8b70771ac8b3b955c459c9105c0df9543708`
- 证据边界：仅代表L0事件级模型，不代表SITL轨迹验收或真实飞行性能。

| 场景 | 策略 | 完成率（均值±标准差） | 用时s | 能耗 | 通信负载 | 恢复率 | 硬约束通过 |
|---|---|---:|---:|---:|---:|---:|---:|
| link_degradation | ai_assisted_hybrid | 0.6985±0.0000 | 115.63 | 0.0067 | 0.6291 | 0.8000 | 10/10 |
| link_degradation | centralized_optimization | 0.6707±0.0000 | 116.06 | 0.0074 | 0.8044 | 0.6200 | 10/10 |
| link_degradation | distributed_collaboration | 0.6368±0.0000 | 116.93 | 0.0063 | 0.5363 | 0.8800 | 10/10 |
| link_degradation | fixed_partition | 0.5695±0.0000 | 118.37 | 0.0060 | 0.3609 | 0.3500 | 10/10 |
| low_battery | ai_assisted_hybrid | 0.7135±0.0000 | 115.63 | 0.0129 | 0.6291 | 1.0000 | 10/10 |
| low_battery | centralized_optimization | 0.6851±0.0000 | 116.06 | 0.0136 | 0.8044 | 1.0000 | 10/10 |
| low_battery | distributed_collaboration | 0.6505±0.0000 | 116.93 | 0.0126 | 0.5363 | 1.0000 | 10/10 |
| low_battery | fixed_partition | 0.5819±0.0000 | 118.37 | 0.0122 | 0.3609 | 1.0000 | 10/10 |
| nominal | ai_assisted_hybrid | 0.7135±0.0000 | 115.63 | 0.0067 | 0.6100 | 1.0000 | 10/10 |
| nominal | centralized_optimization | 0.6851±0.0000 | 116.06 | 0.0073 | 0.7800 | 1.0000 | 10/10 |
| nominal | distributed_collaboration | 0.6505±0.0000 | 116.93 | 0.0063 | 0.5200 | 1.0000 | 10/10 |
| nominal | fixed_partition | 0.5819±0.0000 | 118.37 | 0.0060 | 0.3500 | 1.0000 | 10/10 |
| target_change | ai_assisted_hybrid | 0.6343±0.0000 | 115.63 | 0.0067 | 0.6291 | 1.0000 | 10/10 |
| target_change | centralized_optimization | 0.6090±0.0000 | 116.06 | 0.0073 | 0.8044 | 1.0000 | 10/10 |
| target_change | distributed_collaboration | 0.5782±0.0000 | 116.93 | 0.0063 | 0.5363 | 1.0000 | 10/10 |
| target_change | fixed_partition | 0.5172±0.0000 | 118.37 | 0.0060 | 0.3609 | 1.0000 | 10/10 |
| vehicle_failure | ai_assisted_hybrid | 0.6912±0.0000 | 115.63 | 0.0067 | 0.6291 | 0.7750 | 10/10 |
| vehicle_failure | centralized_optimization | 0.6637±0.0000 | 116.06 | 0.0073 | 0.8044 | 0.6006 | 10/10 |
| vehicle_failure | distributed_collaboration | 0.6302±0.0000 | 116.93 | 0.0063 | 0.5363 | 0.8525 | 10/10 |
| vehicle_failure | fixed_partition | 0.5637±0.0000 | 118.37 | 0.0060 | 0.3609 | 0.3391 | 10/10 |

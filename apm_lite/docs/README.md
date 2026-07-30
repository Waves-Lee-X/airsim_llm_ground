# AeroMind-APM Lite 文档索引

本文档目录只描述非 ROS 的 AeroMind-APM Lite。旧 ROS 2/PX4 原型资料不等同于
Lite 的实机验收结果。

| 文档 | 用途 | 当前状态 |
|---|---|---|
| [`HARDWARE_BASELINE.md`](HARDWARE_BASELINE.md) | V5+、树莓派、P9、D435i 接线与供电基线 | 持续维护 |
| [`M0_ACCEPTANCE.md`](M0_ACCEPTANCE.md) | 协议、Schema、安全门禁和依赖边界验收 | 已通过 |
| [`M1_PROGRESS.md`](M1_PROGRESS.md) | AirSim/ArduCopter 自动任务 10/10 与 RTL 证据 | 已通过 |
| [`M1_5_ACCEPTANCE.md`](M1_5_ACCEPTANCE.md) | 非 Blocks 场景手动 SITL、Web 和相机联调 | 已通过 |
| [`MANUAL_SIMULATION.md`](MANUAL_SIMULATION.md) | 手动启动 UE、SITL、地面站的操作手册 | 可用 |
| [`GEOREFERENCE.md`](GEOREFERENCE.md) | WGS84/map/NED/AirSim 场地标定、哈希和 Web 操作 | 软件链完成，外场实测待办 |
| [`REAL_SERIAL_DEPLOYMENT.md`](REAL_SERIAL_DEPLOYMENT.md) | P9 协议、树莓派安装、systemd、回滚和验收 | UAV3 已部署 |
| [`UAV3_REAL_BENCH_AND_VIDEO.md`](UAV3_REAL_BENCH_AND_VIDEO.md) | UAV3 现场状态、视频、VLM、白名单和遗留项 | ARM/DISARM 待人工测试 |

## 阅读顺序

1. 新成员先读根目录 [`README.md`](../README.md) 和硬件基线。
2. 仿真开发读 M0、M1、M1.5、手动仿真和场地标定手册。
3. 实机维护先读实机部署，再读 UAV3 台架记录。
4. 路线、阶段门禁和三个表演项目读
   [`doc/14-AeroMind-APM-Lite开发计划.md`](../../doc/14-AeroMind-APM-Lite开发计划.md)。

## 状态术语

- **已实现**：代码存在且自动测试通过。
- **已联调**：真实设备链路已观察到预期数据。
- **已验收**：按照文档门禁执行并留有可复查证据。
- **已授权**：仅指明确批准的当前测试范围，不自动扩展到其他命令或实飞。

仿真支持完整飞行命令集，不代表真机已授权相同命令。UAV3 当前实机白名单只有
ARM/DISARM，其他动作即使代码存在也必须保持禁用。

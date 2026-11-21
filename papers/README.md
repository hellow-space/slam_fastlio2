# 论文与项目证据索引

此索引建立于 2026-11-21。只整理项目已有资料与引用，不下载、复制或声称读过未取得的正文。Agent 可直接读取下列源码树内的已有 PDF，不要求把文件搬入本目录。

## 论文

| 资料 | 原始位置或引用 | 核验范围 |
|---|---|---|
| Supplementary Material for FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry | [本地 PDF](../ws_livox/src/FAST-LIVO2-master/Supplementary/LIVO2_supplementary.pdf) | 10 页，可提取文本。含模块消融与 ESIKF 更新比较；不是主论文全文。PDF 创建日期不能替代发表年份。 |
| FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry | [项目 README](../ws_livox/src/FAST-LIVO2-master/README.md)，原引用 `https://arxiv.org/pdf/2408.14035` | 本地 README 提供引用；主论文正文未核验。不得补写公式、硬件或实验细节。 |
| FAST-LIVO: Fast and Tightly-coupled Sparse-Direct LiDAR-Inertial-Visual Odometry | 同一 README，原引用 `https://arxiv.org/pdf/2203.00893` | 仅引用，正文及发表年份未核验。 |
| OpenCalib: A Multi-sensor Calibration Toolbox for Autonomous Driving | [标定工具 README](../ws_livox/src/TFlab/SensorsCalibration/README.md)，原引用 `https://arxiv.org/abs/2205.14087` | README 的 BibTeX 标记 2022；正文未核验。属于辅助标定资料，不默认视为延迟优化依据。 |

论文作者报告的指标必须标注论文页码/表格及实验条件，不能并入本机基线或候选测量。涉及视觉模块的方法不能直接推定适用于当前关闭图像的 LIO 配置。

## 本项目历史证据

- [2026-11-15 室外时延分析](../室外录包时延分析_20261115.md)：历史录包、队列积压、分段计时和状态年龄分析。含 TF 广播器复用及 Path 限长降频的已有改进；不是本轮新测量。
- [诊断字段与边界](../ws_livox/src/FAST-LIVO2-master/docs/timing-diagnostics.md)：理解 `events.csv` 必读。该文档的旧版本描述需与当前源码核对。
- [历史统计及分析资料](../ws_livox/src/FAST-LIVO2-master/docs/outdoor-timing-20261115_202612/)：已归档结果，不能替代实际运行版本/输入/时钟的核验。

已核实存在的历史候选输入：

```text
/home/nvidia/fastlivo_timing/outdoor_bags/handheld_20261115_202612/metadata.yaml
/home/nvidia/fastlivo_timing/session_20261115_202458.erzjsp/run_1794745502735169375_9436/events.csv
```

本轮只检查文件存在和 CSV 表头，未重算性能、验证整个 bag 的完整性或证明历史二进制等于当前源码。下一步可先只读分析，不需要启动 ROS。

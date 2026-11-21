# FAST-LIVO2 里程计更新延迟研究框架

本目录的 Codex 配置专用于 `ws_livox/src/FAST-LIVO2-master`（ROS 包 `fast_livo`），当前 `nav_test.yaml` 为 LiDAR–IMU 模式。目标是减少位姿实际状态年龄及持续积压，并保持精度、TF 与输出连续性。

框架设立阶段仅修改配置与研究资料。2026-11-21 用户随后明确授权方案001（PR #465）最小源码移植、构建及原室内/室外bag隔离回放，目标是室外每帧处理从100多ms降至80ms；进度和实测结论见 [用户方案记录](plans/latency_schemes.md)。这不授权实机操作或其他无关改动。

## 入口

- [闭环经验与自动化操作](plans/lio_benchmark_playbook.md)：后续方案优先使用 `scripts/lio_benchmark.py`，失败可断点续跑，结果自动归档。
- [主控契约](AGENTS.md)：权限、四角色调度、实验前提与验收。
- [论文及历史证据索引](papers/README.md)：已有 PDF、主论文链接和历史分析。
- [延迟实验合同](experiments/FASTLIVO_LATENCY_CONTRACT.md)：指标来源、单位、环境核验及停止条件。
- [冻结清单模板](experiments/fastlivo_latency_manifest.template.json)：默认 `blocked`，未填完整不能运行实验。
- [FAST-LIVO2 指标表头](experiments/fastlivo_metrics_template.csv)：长表格式，未填写任何测量数据。

## 使用

从 `/home/nvidia/wsnew` 打开新的 Codex 会话或重载项目，使新配置与角色文件生效。默认父会话为 `read-only`；子 Agent 继承父权限。角色中的“只读”约束不能代替沙箱。本次会话已启动的旧角色不能证明新配置已加载。

后续直接给方案或PR链接，例如：

> 这是方案002，请按已完成的闭环经验分析、生成最小补丁，并用同一组室内/室外bag测试80ms目标，记录结果。

Codex会填写 `experiments/recipes/002.json`，然后调用统一入口，不再手拼每次不同的测试命令：

```bash
python3 scripts/lio_benchmark.py run --config experiments/recipes/002.json
python3 scripts/lio_benchmark.py resume experiments/lio_runs/002
```

默认模板 `experiments/recipes/lio_default.json` 的方案/patch未填完整，不能直接运行。只读预检使用 `preflight --config ...`；`run ... --prepare-only` 只准备隔离源码，不编译或回放。物理机器人操作和部署仍需单独批准的计划。

离线读取已有文件不需要启动 ROS。回放会启动节点，属于另一阶段，不能把“已有 bag”当成回放授权。

## 已处理的框架缺口

论文索引已建立；指标改为状态年龄、成功帧耗时、发布周期、队列、同步等待和拒绝计数，并明确未测项。环境与地图冲突纳入前置核验，不自动改写原始运行文件。原始包与 CSV 已找到，但其时间域、版本和输入配对仍须在具体实验前冻结。

旧 `experiments/metrics_template.csv` 与 `scripts/compare_metrics.py` 保留为 CPP 历史接口，不能用于 SLAM 延迟验收。`scripts/run_trial.sh` 只保存运行证据。新的统一入口复用既有观察器/字段解析和nearest-rank统计，新增审计、判定、报告与指标导出，版本/哈希需要冻结，旧001原始证据不覆盖。

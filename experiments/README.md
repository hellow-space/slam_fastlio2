# FAST-LIVO2 延迟实验资料

先读 [专用合同](FASTLIVO_LATENCY_CONTRACT.md)，再为每个实验填写 [冻结清单](fastlivo_latency_manifest.template.json)。模板默认 `blocked`；它是人工审核的清单，不是会自动执行或阻止命令的程序。

[专用 CSV](fastlivo_metrics_template.csv) 只有表头，不含模拟结果。各行必须指向真实原始证据。缺失指标的数值为空，带原因，不能填零或替代为其他指标。

基线与候选使用相同数据、参数、运行窗口、硬件、时间域、二进制构建条件及评估逻辑，唯一差异须在实验合同中说明。冻结源码 SHA 还不够：必须保存 dirty diff、未跟踪源码和实际二进制身份。

## 已有接口

- `../ws_livox/src/FAST-LIVO2-master/include/timing_diagnostics.h`：原始 `events.csv` 字段。
- `../ws_livox/src/FAST-LIVO2-master/scripts/summarize_timing.py`：已有离线汇总器，输出 JSON；本轮未修改或执行它。
- `../ws_livox/src/FAST-LIVO2-master/docs/timing-diagnostics.md`：事件语义、开关和测量边界。
- `../scripts/run_trial.sh`：包装主控批准的真实命令，保存 stdout/stderr 合并日志、SHA/status 和秒级总时长。总时长不等于每帧算法耗时，status 也不能代替 dirty 文件快照。

现有汇总器不会生成本框架长表 CSV。转换必须按合同明确映射、单位和证据路径，并审核或手动可追溯记录；没有适配器时保留 JSON 与原始日志，不假装自动比较已就绪。

## 旧 CPP 接口

`metrics_template.csv` 的 `region_count/runtime_ms/path_length_m/coverage_ratio` 和 `../scripts/compare_metrics.py` 保持原样，专用于旧 CPP 工作流。不得将 SLAM 状态年龄塞入 `runtime_ms`，不得用该脚本验收 FAST-LIVO2。

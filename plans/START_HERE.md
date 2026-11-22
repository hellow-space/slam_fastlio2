# 新话题先读：FAST-LIVO2 当前状态与固定测试流程

更新：2026-11-22。先读根目录 `AGENTS.md`，再读本文。本文用于接续项目；当前文件与Git状态变化时，以核对后的实际状态为准。

## 当前项目与进度

- 工作区：`/home/nvidia/wsnew`。
- 主要源码：`ws_livox/src/FAST-LIVO2-master`；ROS 2 Humble，包 `fast_livo`，程序 `fastlivo_mapping`。当前图像关闭，优先处理LIO。
- 用户目标：降低室外每帧处理耗时到80ms，减少积压；不以改时间戳、丢帧或降低精度制造改善。
- **当前实验源码已经累计包含001＋002**：001为PR #465的残差去锁性能部分；002为关闭IMU逐采样与LIO逐迭代等高频调试输出。下一项编号为 **003**。
- 核对时HEAD为 `e1187af5c67544603905285f0274cb1d136545a6`，上述运行源码修改仍在未提交工作区。**只看HEAD不能还原当前累计版本**；保留现有dirty和未跟踪资料。
- 正常 `ws_livox/install` 未更新；正常安装与当前实验源码不是同一个已验证版本。离线测试使用独立worktree和构建目录。
- 默认单Agent，配置见 `.codex/config.toml`（当前 `agents.enabled=false`）。优先节省费用，上限 `gpt-6.1-sol/high`；不因接续项目而启动多Agent烟测。

当前累计源码身份：[cumulative_source_state.json](../experiments/002_quiet_logs/cumulative_source_state.json)；完整001＋002补丁：[cumulative_combined.patch](../experiments/002_quiet_logs/cumulative_combined.patch)。不要依赖可能被清理的 `/tmp` 目录保存唯一证据。

## 固定输入与比较方法

**以后仅使用这一份指定室外原始bag，不再默认测试室内：**

```text
/home/nvidia/fastlivo_timing/outdoor_bags/handheld_20261115_202612
```

时长约123.84秒，包含原始 `/livox/lidar`、`/livox/imu`。完整1倍速回放，前5秒暖机；主要比较源时间+40至+100秒的高负载同扫描窗口，同时报告全包暖机后的结果。

每个新方案默认一组A/B，共两次室外回放：**A=上一轮保留的累计源码；B=A＋本次修改**。例如003必须比较“001＋002”与“001＋002＋003”，不能退回原代码。默认不追加重复回放；一次配对是快速筛选，重复性不足如实注明。

## 用户已经确定的操作流程

1. 用户提供方案或链接；先分析最小改动，并在 `plans/latency_schemes.md` 追加编号、来源和假设。
2. 保留A，制作B的增量补丁及新配方，复用 `experiments/recipes/lio_default.json`。**运行脚本时当前主工作区应保留A**，由脚本只对候选应用增量补丁；不能先将新补丁写入A，导致重复应用或比较失效。
3. 使用现成工具隔离构建、跑已有诊断测试；节点先通过启动检查，再回放固定室外bag。
4. 对比同一扫描的mean/p50/p95/max、>80/>100ms次数及比例；同时核对状态年龄、队列、输入完整性和位姿/TF。
5. 写出真实结果和结论，更新方案记录、本文当前进度及保留源码的身份。保留实验源码与通过严格验收分开说明；正常安装不自动覆盖。

已获用户授权的新方案按上述步骤连续执行，不为机械步骤反复确认。仅交接或讨论方案时先只读；新修改/实验须处于用户给定范围与实际会话权限内。`.codex`默认只读，权限不足时如实说明，不能绕过；实机操作和部署不属于本流程。

## 统一工具入口

先生成并填好新配方，例如003的来源、假设、增量patch、allowed_files、baseline_ref及唯一输出编号。模板不是可直接执行的成品；旧002配方和已完成结果不能复用为003。

```bash
python3 -B scripts/lio_benchmark.py preflight --config experiments/recipes/003.json
python3 -B scripts/lio_benchmark.py run --config experiments/recipes/003.json
# 仅当该次运行中断/失败且冻结条件未变时继续：
python3 -B scripts/lio_benchmark.py resume experiments/lio_runs/003
```

流程细节：[lio_benchmark_playbook.md](lio_benchmark_playbook.md)。每次使用唯一编号并保留真实日志，沿用现有解析器及指标，不重新手拼评估流程或事后放宽阈值。

## 最近真实结果与限制

最新累计比较：`experiments/lio_runs/002-cumulative/`。室外40–100秒同扫描600帧：

| 指标 | 001 | 001＋002 |
|---|---:|---:|
| 平均处理耗时 | 67.15ms | 65.65ms |
| P95 | 87.13ms | 85.48ms |
| 最大耗时 | 147.83ms | 120.40ms |
| >100ms | 8/600 | 5/600 |

平均达到80ms预算，P95未达到。两侧初始化IMU接收不同，配对位置最大差约3.08cm，每场景仅一对、无真值；严格结论 **RETEST**，不能声称精度等价或日志的稳定增益已确认。10Hz输入正常约100ms输出一次，处理预算80ms不等于12.5Hz发布率。

上述累计实验当时包含室内、室外两包；此后用户改为仅室外。**室外单包自动调度与报告已通过单元测试（最近32项通过），尚未为这次范围调整额外回放bag**。固定测试流程已确定，不能把单测写成新的ROS实测。

详细证据：[累计补测报告](../experiments/002_quiet_logs/CUMULATIVE_REPORT.md)、[全部方案记录](latency_schemes.md)。原002曾错误地未叠加001，那份单独测试不能替代最新累计比较。

## 新话题第一次回复

先只读核对 `git status`、当前源码身份与 `.codex/config.toml`，简短确认“累计版本、固定bag、最近结果、本次任务”。没有新方案时，不自动重跑旧实验；用户给出下一项方案后，沿用本流程继续。

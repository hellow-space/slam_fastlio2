# FAST-LIVO2 里程计延迟：方案001经验与后续标准流程

## 用户只需要给什么

给出方案描述或 PR 链接，并要求按这套流程验证。Codex负责核对原始改动、选最小机制、生成补丁及本方案JSON、登记方案；之后交给统一脚本执行。无需每次重新设计框架或重复确认已经授权的离线步骤。实机与部署不属于这套自动化。

## 一键入口

在项目根目录运行，父会话须允许写工作区及临时目录。`.codex/config.toml` 仍按项目要求默认只读；脚本不能绕过父会话权限。仅只读分析时不运行写入/回放命令。

```bash
# 新方案：Codex先复制模板并填入实际编号/来源/假设/patch/允许文件
python3 scripts/lio_benchmark.py preflight --config experiments/recipes/002.json

# 自动隔离准备、构建、诊断单测、室内外A/B、审计、报告
python3 scripts/lio_benchmark.py run --config experiments/recipes/002.json

# 中断或失败后继续，同一目录保持原证据，不能改配置后借用旧测量
python3 scripts/lio_benchmark.py resume experiments/lio_runs/002
```

默认模板是 `experiments/recipes/lio_default.json`，未填方案与patch时必须报缺项，不能执行。每个方案输出目录唯一；重新实验使用新编号/修订输出根目录，不删除失败现场。只验证准备、不启动ROS时：

```bash
python3 scripts/lio_benchmark.py run --config experiments/recipes/001_reproduction.json --prepare-only
```

`001_reproduction.json` 仅用于复现已知补丁的工具准备验证，`record_ledger=false`，不是新用户方案或新性能结果。

## 自动步骤与产物

| 步骤 | 机械工作 | 必须保留 |
|---|---|---|
| prepare | 校验配置与精确patch范围；核对bag/环境；保存当前包源码、dirty和untracked源；创建两棵独立worktree | SHA、源文件哈希、实际补丁、参数、bag身份及state |
| common setup | 两边统一应用已验证的关停异常处理，已存在则确认，不能匹配则停止 | common patch独立记录，不能冒充方案收益 |
| build | ROS Humble、诊断ON、相同编译条件、独立build/install前缀 | 准确argv、输出、退出码、实际二进制身份 |
| tests | 运行既有诊断单测 | 测试日志；未运行不能称通过 |
| replay | 串行室内/室外基线与候选；domain176、localhost、仅原始LiDAR/IMU、1倍速、200Hz clock、30秒排空 | 原始events.csv、outputs.jsonl、node/player/observer日志、状态与硬件采样 |
| report | 同源扫描配对、完整性审计、固定口径统计、独立判定 | comparison.json、REPORT.md、verdict.json、metrics.csv及追加方案记录 |

状态持久化；已通过步骤只有冻结身份一致才能复用。失败步骤写新attempt，不改旧文件。配置、补丁、输入、源码或评估器变动导致不可比时，不能把旧结果拼进去。仅清理本流程创建的子进程，不杀用户节点、不停止实机。

## 必须记住的经验

1. **80ms是成功帧完整处理耗时。** 10Hz输入正常约100ms一帧，不能称处理65ms就发布15Hz。分别给mean、p50、p95、max、>80和>100次数/比例。
2. **同条件新基线优先。** 历史现场197.23ms峰值与本次134.92ms基线不是同一次实验，不能直接归因补丁。历史100多ms均值没有在本次配对复现，报告必须说明。
3. **单个方案只改一个机制。** 方案001仅测试PR465的25a774f去锁部分；视觉关闭，VIO修复不进入热路径；IMU去畸变修复改变结果，不混入该次对照。
4. **平均达标不等于稳定达标。** 001室外高负载600帧，mean81.04→65.90ms，但p95 107.68→87.09ms，仍121/600超过80ms、5/600超过100ms；max134.92→145.70ms，长尾不能隐去。
5. **状态年龄与耗时分开。** odom header使用当前时间，不能据此判位姿新鲜。001高负载状态年龄p95约1398→108ms、完整周期队列峰值13→0。EOF后clock冻结，尾部延迟不能被虚假变小；保留原始年龄和已验证steady映射口径。
6. **关停异常是测试设施问题。** 首次室外基线播完后SIGINT遇RCLError，CSV尾行截断；不能剔除坏尾行就算成功。已将特定shutdown-only兼容层独立保存、两边同加，正常析构刷新后才使用V2。
7. **按源扫描匹配。** 不能用输出到达时间选样本；不能把bag记录结束时间当扫描末尾上界；时间键重复、单边丢帧、两边共同漏处理都要审计。计数分开列回调、同步周期、成功周期、odom与observer。
8. **初始化也会不同。** 001室外首个LiDAR前IMU拒收7→15，同扫描轨迹最大相差4.12cm。输入文件相同不代表实际初始化序列相同；没有对照与真值，不能据此说PR降低精度，也不能宣称逐位等价。自动报告应提示RETEST，绝不事后放宽阈值。
9. **原源码和安装隔离。** ROOT_DIR决定日志写入源位置，必须从独立worktree编译；直接换二进制输出目录不够。保留正常安装，不用reset/clean丢用户改动。
10. **失败也是证据。** 每个用户方案在 `plans/latency_schemes.md` 留编号、来源、假设、范围、真实结果和结论。失败/中断保留原日志；resume只重试未完成部分，不能抹掉第一次失败。

## 判定口径

- `performance_mean_pass`：室外指定高负载窗口实际mean是否≤80ms。
- `performance_p95_pass`：实际p95是否≤80ms，独立报告，不能当作均值结论。
- `ACCEPT`：预先确定的性能、输入可比性、回归和重复性检查均有证据通过；它仍不触发自动部署。
- `REJECT`：可比条件下，明确未达到性能或已冻结回归条件；保留实际数值。
- `RETEST`：数据/日志/版本/初始化可比性/重复性或必要门槛不充分；可以同时报告真实的平均性能达标，但不得冒充可采用。

默认一次配对用于快速初筛；需要正式采用时，在方案配置中设置至少两次配对并冻结条件。脚本不自主追加算法、不自动调整阈值、不自动合并或部署。方案是否适合当前LIO路径、源码差异的原因及最终取舍仍由主控负责。

## 复用旧证据验证新工具

```bash
python3 scripts/lio_benchmark_report.py experiments/pr465/paired_runs_v2 --output /tmp/lio_report_recheck
python3 -m unittest discover -s scripts/tests -v
```

导出目录必须是新的，不能写回旧实验目录。归档复算应保留81.037273615→65.90308844ms、>100ms的114→5/600，初始化与重复性结论仍为RETEST。它只验证工具与真实证据的兼容性，不等于重新运行了ROS或新的性能实验。

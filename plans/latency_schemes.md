# 用户提供的里程计延迟方案记录

每个新方案按编号追加，保留失败与未完成记录，不将论文/PR作者结果当本机测量。

**当前实验源码包含方案001＋002**，正常安装尚未更新。方案002按用户纠正完成累计补测，最新结果见 [累计报告](../experiments/002_quiet_logs/CUMULATIVE_REPORT.md)。后续默认以上一轮保留的实验源码为基线，不默认退回原代码。

**后续仅测试指定的最新室外bag**：`/home/nvidia/fastlivo_timing/outdoor_bags/handheld_20261115_202612`，约123.84秒。2026-11-21用户明确不再默认跑室内；每次一对A/B只需室外基线、室外候选各一次。新方案沿用40–100秒高负载统计窗口，同时报告全包跳过暖机后的结果；历史配方和数据不改写。

## 001 — FAST-LIVO2 PR #465（2026-11-21）

- 用户原始方案：https://github.com/hku-mars/FAST-LIVO2/pull/465
- 用户目标：室外原先100多ms降低到80ms；流程为“分析改法 → 修改、用之前室内/室外bag测试 → 判断是否达标”，无需额外配置讨论。
- 授权：本方案的最小源码移植、构建和隔离离线回放；不含实机操作，不覆盖正常安装。
- 上游快照：head `b3dd11fc4aa4a0514b9c170d8d2c4d5b5c43e4c8`，base `0d2c0346107b75b59934975adec9a6eeeb913c64`，获取时 PR open、未合并。原始元数据/完整diff保存在 `experiments/pr465/`。
- 原始主张：PR包含残差匹配逐点互斥锁去除、VIO内存/边界修复、IMU首点重复去畸变修复。作者报告残差对比相同，但没有给出本仓库室外端到端80ms结果。
- 本轮最小移植：先测试其中独立性能提交 `25a774f787d2b8922ba4d216c175eb29258f7ada`：`BuildResidualListOMP` 的位压缩 `vector<bool>` 改为按点字节标志，去除逐点互斥锁，值初始化 `PointToPlane`，保留残差计算与原顺序收集。VIO当前关闭；IMU修复会改变数值行为，均不混入这次单因素性能测试。结果仅代表PR的这一性能部分，不代表已测试完整PR。
- 假设：同样原始扫描、配置、4个OpenMP线程和诊断条件下，去除逐点锁能使室外高负载窗口每帧完整处理均值降至≤80ms，并减少/不增加积压。
- 评价：优先使用已知长室外包源时间+40至+100秒相同扫描，另报全包（跳过前5秒）和室内。mean≤80ms只称平均达标；p95≤80ms才称95%帧达到该预算。同步报告输出间隔与状态年龄，不以积压追赶伪造更新率。
- 回归：输入扫描/IMU接收完整性、发布连续性、同扫描残差/位姿差异、TF一致性。没有真值不宣称精度已认证。
- 停止：编译失败、输入丢失、节点异常或比较条件错配则RETEST；未达到80ms如实记录，不叠加第二个方案。先完成一次配对；临界/噪声大时最多增加一次反序配对。
- 基线：当前 Git SHA `5a506e73827609c6814969648ac94f274c39c167`；保留已有 `localization.md` 改动及未跟踪资料，源码变体在独立worktree测试。
- 原始输入：室内 `imu_debug_20261112_210002`（26.125秒）；主室外 `outdoor_bags/handheld_20261115_202612`（123.838秒），均在 `/home/nvidia/fastlivo_timing/`。目前没有发现单个包含全部室内/室外的其他bag；暂按既有室内与室外两包分别比较，若用户指定其他路径则更新。
- 状态：精确移植的候选补丁为 `experiments/pr465/candidate.patch`（仅voxel_map.cpp，+4/-10）。基线与候选独立构建成功，已有诊断单测3/3通过。首次室内A/B成功，但室外基线在回放结束后SIGINT关停时抛出ROS context/guard condition异常（node=-6），CSV尾行截断，未运行室外候选，不计为有效室外比较；原始证据完整保留在 `experiments/pr465/paired_runs/`。
- 测试入口修复：只在两个隔离worktree的main.cpp加入相同的、严格限定ROS已关闭且特定guard condition异常的关停catch，其他异常原样抛出；让正常析构刷新CSV，不改估计循环/参数/TF。共同补丁单独保存为 `experiments/pr465/offline_shutdown_common.patch`，它不是PR性能改动，也未写入原工作区。两侧重新构建并重新冻结源diff及二进制身份后，在 `experiments/pr465/paired_runs_v2/` 重跑全部四次。比较两侧唯一算法差异仍是voxel_map.cpp。V2四次回放与汇总均成功，实测结果如下；原始失败不计入配对结果。
- 结果：室外40–100秒600个同扫描成功帧，平均81.04→65.90ms（降低18.7%，平均≤80ms达标）；P95 107.68→87.09ms（95%帧80ms预算未达），>80ms比例69.50%→20.17%，最大134.92→145.70ms。室内平均39.95→31.62ms。室外状态年龄P95 1397.98→108.05ms，完整周期LiDAR队列峰值13→0。当前配对没有复现历史100多ms均值。
- 采用判断：**RETEST**。严格输出等价门槛未通过（室外同扫描轨迹最大差4.12cm），且初始化IMU拒收7→15、每场景仅一对；不声称精度退化或等价，不自动部署，候选补丁留在隔离分支。性能平均目标已达到，95%帧稳定80ms未达到。
- 证据与完整命令：[报告](../experiments/pr465/REPORT.md)、[实测JSON](../experiments/pr465/paired_runs_v2/comparison.json)、[真实指标CSV](../experiments/pr465/measured_metrics.csv)、[原始运行总日志](../experiments/runs/20261121T060957Z_pr465-paired-replay_51696/output.log)。后续用户方案追加新编号，不覆盖本条。

## 002 — 关闭高频调试日志（2026-11-21）

- 用户原始方案：移除或禁用IMU 200Hz日志及逐次迭代输出，减少I/O和线程阻塞；按“分析→修改→确认可启动→同bag测试”执行，复用现成工具。
- 本轮授权：上述最小日志修改、隔离构建和已有室内/室外原始bag离线A/B；不含实机或覆盖正常安装。
- 最小范围：LIVMapper.cpp的IMU/LiDAR逐消息INFO降为DEBUG，去掉持锁IMU cout、每扫描地图提示与耗时表；voxel_map.cpp去掉逐次迭代统计输出；IMU_Processing.cpp去掉逐采样imu.txt写入/刷新及其文件打开。告警、初始化信息、诊断CSV、轨迹输出及估计计算保留。
- 假设：减少同步输出及持锁刷新可降低室外40–100秒相同扫描完整周期耗时和积压，目标均值≤80ms；同时报告P95/max和>80/>100ms计数。TF树、时间戳和话题语义无改动。
- 基线：当前HEAD `e1187af5c67544603905285f0274cb1d136545a6`；本次单独测试日志方案，未叠加方案001。补丁 `experiments/002_quiet_logs/candidate.patch`，配方 `experiments/recipes/002.json`。
- 执行：复用 `python3 -B scripts/lio_benchmark.py run --config experiments/recipes/002.json`；各节点先通过现有启动检查再回放；室内与室外各一对，沿用5秒暖机、室外40–100秒窗口及既定回归阈值。不因观测结果修改阈值；单对结果用于快速性能筛选，重复性和无真值精度证据保留限制。
- 状态：两个版本隔离构建成功，已有诊断测试各3/3通过；室内/室外四次回放全部完成，节点、回放器及观察器退出码均为0。目标控制台输出在候选中计数为0，imu.txt不再产生。
- 实测：室外40–100秒600个同扫描成功帧，mean 83.6582→83.6514ms，P95 108.6273→109.6157ms，max 133.0569→128.3948ms，>100ms 180/600→177/600，>80ms 433/600→424/600。室内mean 39.6105→40.5841ms。均值与95%帧80ms目标均未达到，未观察到有效平均耗时改善。
- 回归：室外原始ROS状态年龄P95 2414.54→2698.68ms（各600样本），steady映射年龄P95 2413.62→2697.66ms，LiDAR队列峰值24→27；两项超过冻结回归阈值。同扫描位置和姿态输出差异为0，TF/odom一致，完整输入与初始化指纹检查通过；无真值精度认证。每场景仅一对，不能认定这些时间波动的因果。
- 判断：工具判定RETEST；本轮不采用为80ms性能优化，不叠加方案001、不追加重复测试、不覆盖正常安装。真实改动保留在 `/tmp/lio-benchmark-dexffe53/candidate` 及归档补丁中。说明与日志关闭核验见 [方案002报告](../experiments/002_quiet_logs/REPORT.md)，完整数据见 [自动报告](../experiments/lio_runs/002/REPORT.md)。


<!-- lio-benchmark:/home/nvidia/wsnew/experiments/lio_runs/002:complete -->
### 方案 002 自动化记录

- 来源：用户方案002：关闭高频调试日志
- 假设：关闭IMU逐采样控制台/文件刷新和LIO逐迭代/逐帧表格输出，降低I/O与持锁阻塞；相同输入下比较室外40–100秒完整处理均值是否达到80ms，并报告P95、最大值和超过100ms帧数。
- 运行目录：`experiments/lio_runs/002`
- 阶段：report
- 性能：mean≤80 ms=False, p95≤80 ms=False
- 判定：RETEST。[报告](../experiments/lio_runs/002/REPORT.md)

### 方案002累计测试修正（2026-11-21）

- 用户纠正：第二次修改应叠加第一次修改。此前002是“原代码 vs 原代码+日志关闭”的单独测试，不能替代累计比较；保留原结果供追溯。
- 当前主工作区源码先应用方案001作为实验基线，正常安装未覆盖；其完整diff保存在 `experiments/002_quiet_logs/cumulative_baseline.patch`。
- 正确A/B：A=方案001，B=方案001+方案002；唯一增量仍是日志关闭。沿用两个原始bag、同参数与旧指标、每场景一对；补跑目录 `experiments/lio_runs/002-cumulative`，配方 `experiments/recipes/002_cumulative.json`。
- 后续按用户要求累计改动；若需要单独测试某个机制，应明确说明其比较基线，不能默认回到原代码。
- 状态：两版本构建成功，已有诊断测试各3/3通过，四次回放的node/player/observer退出码全部为0，完成累计比较。
- 累计实测：室外40–100秒同扫描600帧，方案001→方案001＋002：mean 67.1506→65.6531ms，P50 75.8722→74.5087ms，P95 87.1318→85.4757ms，max 147.8329→120.3993ms，>80ms 157/600→112/600，>100ms 8/600→5/600。室内mean 30.3148→30.7530ms。
- 积压：室外状态年龄P95 110.63→106.60ms、有效样本各600；steady映射年龄P95 110.96→107.37ms，LiDAR队列峰值两侧均为0。TF/odom一致，完整输入与成功扫描配对检查通过。
- 限制：室外初始化IMU拒收6→7、初始化指纹不同，同扫描位置最大差3.08cm；每场景仅一对且无真值，严格判定RETEST。累计版本平均80ms目标已达，P95目标未达；不能将本轮1.50ms小幅改善全部归因为日志，也不宣称精度等价。
- 源码保留：将已测试候选的001＋002三份实现源码保留到当前主工作区，并逐文件核验与候选内容完全一致。正常安装未覆盖，仍是实验源码，不等于部署/严格验收通过。累计补丁 `experiments/002_quiet_logs/cumulative_combined.patch`；身份记录 `experiments/002_quiet_logs/cumulative_source_state.json`；完整冻结与真实日志 `experiments/lio_runs/002-cumulative/`。


<!-- lio-benchmark:/home/nvidia/wsnew/experiments/lio_runs/002-cumulative:complete -->
### 方案 002-cumulative 自动化记录

- 来源：用户纠正：方案002必须叠加已测试的方案001
- 假设：累计比较：基线为当前HEAD加方案001的PR465残差去锁补丁，候选为该基线再关闭方案002高频日志。使用相同室内/室外原始bag，判断日志关闭在方案001基础上的增量收益及室外80ms目标。
- 运行目录：`experiments/lio_runs/002-cumulative`
- 阶段：report
- 性能：mean≤80 ms=True, p95≤80 ms=False
- 判定：RETEST。[报告](../experiments/lio_runs/002-cumulative/REPORT.md)


<!-- lio-benchmark:/home/nvidia/wsnew/experiments/lio_runs/003:complete -->
### 方案 003 自动化记录

- 来源：OpenMP thread count comparison: 4 vs 8, cumulative source 001+002 unchanged
- 假设：With identical cumulative FAST-LIVO2 source, runtime parameters, outdoor input and evaluation, MP_PROC_NUM=8 will reduce high-load full_cycle p95 relative to MP_PROC_NUM=4 by up to the targeted 5.5 ms without regression gates failing.
- 运行目录：`experiments/lio_runs/003`
- 阶段：report
- 性能：mean≤80 ms=True, p95≤80 ms=True
- 判定：RETEST。[报告](../experiments/lio_runs/003/REPORT.md)

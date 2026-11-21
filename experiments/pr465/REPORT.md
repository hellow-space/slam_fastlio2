# 方案001：PR #465 性能部分的室内/室外 A/B

状态：V2四次回放和固定汇总均退出0。**室外高负载平均处理耗时目标达成；采用建议 RETEST。** 预设输出等价门槛未通过，不自动采用或部署。

## 假设与改动

去除 `BuildResidualListOMP` 的逐点互斥锁，保持残差算法/顺序、输入及参数一致，能将室外高负载窗口成功帧完整处理均值降至80ms以内。

来源：https://github.com/hku-mars/FAST-LIVO2/pull/465 ，仅移植独立性能提交 `25a774f787d2b8922ba4d216c175eb29258f7ada`。PR作者的745条残差一致性结果不是本机耗时证据。本轮不混入视觉和IMU去畸变行为修复，也不声称测试了整个PR。

`candidate.patch` 为算法补丁（voxel_map.cpp，+4/-10）。`offline_shutdown_common.patch` 是两边相同的测试入口关停兼容层，解决首次室外基线SIGINT后的RCLError/CSV未刷新问题，不改变估计热循环。正常安装、原工作区运行代码和原Log不变。

## 输入与口径

基线 HEAD：`5a506e73827609c6814969648ac94f274c39c167`；工作状态、原始diff、源码哈希、实际二进制/依赖、参数与评估器身份见 `contract.json` 及配对目录manifest。两侧在独立worktree构建，诊断ON、OpenMP四线程、filter_size_surf=0.15。

室内：26.125秒原始bag，262帧LiDAR、5224条IMU。室外：123.838秒原始bag，1238帧LiDAR、24771条IMU。源文件路径/完整哈希在contract中；不回放历史输出odom/TF。

主指标为 `full_cycle.status=1.duration_ms`，不是odom header年龄或10Hz输入的发布间隔。按相同源扫描配对：全包跳过前5秒；室外高负载窗口为源时间+40至+100秒。mean≤80ms仅称平均达标，p95≤80ms才称95%帧达到预算。状态年龄、输出周期、队列、TF和位姿差异分别审计，无真值不认证绝对精度。

## 准确执行入口

构建、测试和每次回放均由 `scripts/run_trial.sh` 保存命令、合并stdout/stderr、退出码及时间信息到 `experiments/runs/`。V2准确argv保存于contract的 `approved_command`；每轮实际命令保存于 `paired_runs_v2/manifest.json` 和各运行 `commands.json`。

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/wsnew/ws_livox/install/setup.bash
cmake --build /tmp/fastlivo_pr465_build_baseline --target fastlivo_mapping -j2
cmake --build /tmp/fastlivo_pr465_build_candidate --target fastlivo_mapping -j2
python3 -m unittest discover -s /tmp/fastlivo_pr465_candidate/ws_livox/src/FAST-LIVO2-master/tests -v
python3 scripts/run_pr465_ab.py --baseline /tmp/fastlivo_pr465_build_baseline/fastlivo_mapping --candidate /tmp/fastlivo_pr465_build_candidate/fastlivo_mapping --out /home/nvidia/wsnew/experiments/pr465/paired_runs_v2
python3 scripts/summarize_pr465_ab.py experiments/pr465/paired_runs_v2
```

runner会拒绝已有输出目录，不能覆盖重跑。重新运行需明确新路径并冻结清单。回放为domain176、localhost-only、1倍速、200Hz /clock、use_sim_time=true，仅原始IMU/LiDAR；每次排空30秒。不启动实机脚本，不发送运动命令。

## 当前验证与边界

两套构建成功；既有诊断单测3/3通过。新增评估入口明确复用现有load/stats/summarize，未改旧解析器；四个合成审计样例验证正常记录、重复扫描、NaN header和乱序header检查，它们不是性能数据。

首次室外基线失败证据保留于 `paired_runs/`，没有用于以下测量。以下仅使用完整的V2证据。

## 实测结果（基线 → 候选）

| 范围 | 配对帧数 | Mean ms | P95 ms | Max ms | >80ms比例 |
|---|---:|---:|---:|---:|---:|
| 室内（跳过前5秒） | 211 | 39.95 → 31.62 | 50.88 → 41.53 | 73.03 → 66.92 | 0.00% → 0.00% |
| 室外全包（跳过前5秒） | 1188 | 62.17 → 49.56 | 104.03 → 83.65 | 134.92 → 145.70 | 37.04% → 10.19% |
| 室外高负载（源时间40–100秒） | 600 | 81.04 → 65.90 | 107.68 → 87.09 | 134.92 → 145.70 | 69.50% → 20.17% |

室外高负载均值降低 18.7%。本轮基线均值81.04ms，**没有复现历史100多ms的均值，不能写成实测115→66ms**。P95仍87.09ms、20.17%帧超过80ms，因此只称平均达标，不称每帧或95%帧稳定80ms。候选单帧最大145.70ms高于基线134.92ms，长尾仍存在。10Hz原始输入下，两侧输出间隔均约100ms，处理预算80ms不等同12.5Hz发布。

## 回归、重复性与主控判断

- 室外高负载状态年龄P95：1397.98→108.05ms；steady映射口径1398.44→107.46ms，主窗口发布发生在回放EOF前，没有冻结clock污染。所有V2原始odom年龄负值/非有限计数均为0。
- 完整周期队列峰值13→0；同步前队列峰值全程14→1，口径不同，不混用。输出间隔P95为116.36→115.24ms。
- 两侧输入回调数量完整：室内5224 IMU/262 LiDAR，室外24771 IMU/1238 LiDAR；同场景两侧odom分别258/1235，成功周期/诊断/observer计数一致；暖机后独有扫描均为0。
- TF frame均为camera_init→aft_mapped，位置差为0，四元数sign等价分量差最大6.66e-16；TF门槛通过。
- **输出等价门槛未通过**：室外相同扫描位置差最大0.041167m，四元数分量差最大0.00174480（全包）；室内最大位置差3.9849e-9m、四元数分量差1.0599e-9，也超过冻结的1e-9m/1e-12严格阈值。阈值没有事后放宽。
- 室外首次LiDAR前IMU拒收数为7→15（status=-2），不是DDS丢包；两次初始化消费序列不同。该事实可能解释部分轨迹差异，但尚未证明，不能把4.12cm输出差写成PR导致的精度退化或正常噪声。
- 室外CPU温度均值62.24/62.26°C，频率样本均值1570/1539MHz，均采用同一调频策略；未锁频，记录见原始tegrastats。每场景只有一组成功配对，尚未验证重复性；当前平均结果离80ms有余量，未追加无关方案或反复调参。
- 独立实验员按严格等价门槛给出REJECT（不可采用）；主控最终采用建议为 **RETEST**：性能平均目标已达到，但初始化条件不一致、等价性和重复性不足，不据此接受或否定算法精度。不自动合入原运行源码/安装。若后续需要采用，先做一致初始化的反序重复对照。

详细证据：`paired_runs_v2/comparison.json`、`matched_frames.csv`、各运行原始events.csv/outputs.jsonl/summary_timing.json；实际命令及版本为`paired_runs_v2/manifest.json`，长表为`measured_metrics.csv`，判定为`verdict.json`。

最终完整性复核：四次运行exit0、原始CSV无截断行、102条实测/缺失指标记录合法；二进制、参数及评估器与冻结哈希一致；原工作区106个已跟踪包文件未变，两测试树之间唯一源差异为voxel_map.cpp。复核哈希见`verified_artifact_sha256.json`，源范围见`verified_source_scope.json`。

# Codex 主控契约：FAST-LIVO2 里程计更新延迟

## 目标与当前授权

- 主要对象：`ws_livox/src/FAST-LIVO2-master`，ROS 包名 `fast_livo`，可执行文件 `fastlivo_mapping`。当前 `config/nav_test.yaml` 关闭图像、启用 IMU，优先分析 LIO 路径。
- 目标：减少实际位姿状态年龄和持续积压，同时保持精度、TF/时间戳语义、数据完整性和输出连续性。不得通过仅修改 header、丢弃旧帧或未经批准降低精度制造改善。
- 默认只读诊断。**2026-11-21 用户新增授权：将 PR #465 作为方案001，分析后做最小源码移植、构建，并使用已有室内/室外原始 bag 做隔离离线 A/B 测试，判断室外原先100多ms的每帧处理是否降至80ms；把每个用户方案持续记录在 `plans/latency_schemes.md`。本授权不包含实机操作、运动命令、覆盖正常安装或无关运行参数修改。**
- 后续源码修改或实验必须有用户明确授权的具体范围；不能把“建立框架”视为授权。用户授权后还需确认父会话权限允许相应操作。物理机器人操作始终需要单独批准的计划。
- `.codex/config.toml` 默认 `sandbox_mode="read-only"`，新会话/重载后核验。子 Agent 继承父沙箱与审批设置；角色中的只读指令不是独立权限边界，角色文件不能覆盖父权限。

## 四角色与主控调度

主控是 orchestrator，负责证据取舍、唯一假设、范围、实验合同、整合及最终判断。只派有界任务，不为小问题拆角色；禁止子 Agent 派生子 Agent，最多三个活跃子 Agent。

| 角色 | 输入 | 输出与边界 |
|---|---|---|
| `paper_researcher` | `papers/README.md` 索引的现有论文、认可的参考链接和历史报告 | 原文方法/假设/页码、适用于当前 LIO 的条件、局限与可证伪建议；严格区分原文、历史测量和推断；不改文件 |
| `code_analyst` | 实际源码、配置、已有诊断/日志及主控研究问题 | 输入到发布的数据流、时间域/队列/耗时边界、具体函数路径、最小改动点和现有评估接口；只读，不先认定根因 |
| `implementer` | 主控确定且用户授权的一个假设、明确文件范围和验证条件 | 先列计划改动，再给最小 diff、验证记录与风险；不扩大范围，不改评估器或运行输入，不声称性能提升 |
| `experimenter` | 已批准的基线/候选、完整冻结清单、准确命令与验收阈值 | 实际日志、配对测量、重复性、失败与回归；不改实现源码，不发明命令/数值，不自主操作机器人 |

研究流程：需要独立证据时并行 `paper_researcher` 与 `code_analyst` → 等待两者完成 → 主控定义**一个可证伪假设**、依据、最小范围、基线/候选、指标、阈值及停止条件 → 取得相应操作授权后使用 `implementer` → 随后使用 `experimenter` → 主控 ACCEPT / REJECT / RETEST。默认最多两轮改进后人工审查。

纯源码/日志性能诊断可依据可复查的源码与历史数据，不必假装存在论文。若声称复现论文方法，须取得相应正文并记录原文依据。不可读、缺失或只有链接的论文必须明确标记，不能编造公式、引用或作者实验。

## 延迟指标与证据合同

- 本轮80ms目标：优先比较相同室外高负载扫描窗口内成功帧 `full_cycle.status=1` 的处理耗时，报告 mean/p50/p95/max、超过80/100ms的比例。10Hz输入的正常输出间隔约100ms，处理预算80ms不等同12.5Hz发布率；若用户进一步明确口径，以最新说明为准。
- 同时检查状态年龄：`1000 * (odom_publish.ros_s - odom_publish.scan_end_s)`，单位 ms，报告 p50/p95/max 及有效样本数。仅在时间域已确认、扫描状态有效时成立；它不等于 DDS/下游接收/控制延迟。
- odom/TF header 当前使用节点当前时间，不能据此推断状态新鲜。steady clock 只用于耗时；禁止当前墙钟减历史 stamp。
- 分别报告成功帧 `full_cycle.status=1` 耗时、`frame_period`、IMU/LIO/地图/发布分段、回调原始/修正时间年龄、队列及同步等待原因。外层/内层耗时不可重复相加，窗口累计不可冒充单次调用。
- 回调拒绝和同步失败不是 DDS 丢包率；未测 stale/drop、下游延迟和 APE/RPE 时记录 missing，不能填零。没有真值不得宣称精度不退化。
- 详细字段、单位、有效样本与缺失规则见 `experiments/FASTLIVO_LATENCY_CONTRACT.md`。冻结清单用 `experiments/fastlivo_latency_manifest.template.json`；指标用 `experiments/fastlivo_metrics_template.csv`。
- 清单默认 `blocked`，没有完整合同、可比输入、批准命令和回归阈值不得运行。模板是人工检查资料，不是程序化执行拦截器。旧 CPP 模板和 `scripts/compare_metrics.py` 不得用于 SLAM 延迟验收。
- 基线保留 Git SHA、dirty diff、未跟踪源码快照、源码与实际运行二进制/依赖身份。未来比较变体时用独立分支/worktree，保留用户现有工作，不覆盖正常安装。
- 基线与候选保持录包、参数、外参、时间偏移、硬件/功耗/频率、线程、诊断开关、回放时钟、暖机/窗口、重复次数和评估逻辑相同，唯一差异是批准的机制。原始包应含实际 LiDAR/IMU 输入；不能只回放输出 odom/TF。
- 不静默新增或更换指标解析器。已有 JSON 汇总器不自动生成本框架长表；缺适配器时保留真实日志/JSON及可追溯人工记录，不能声称自动比较已就绪。

## 本项目已有证据与环境前置检查

- 先读 `papers/README.md`、`室外录包时延分析_20261115.md` 和 FAST-LIVO2 的 `docs/timing-diagnostics.md`。历史资料不等于本轮测量，也不自动证明当前二进制/配置一致。
- 当前源码已经复用 TF 广播器，并对 Path 限长/降频。不得把这些已实现改动重复当作新方案；诊断文档中的旧描述需与源码核对。
- 历史室外 bag 和 `events.csv` 已发现，可先只读分析；完整性、时间域、版本、输入配对仍须核验，不把“文件存在”写成“实验已运行”。
- FAST-LIVO2 使用根 `ws_livox` 布局。定位包 `lidar_localization_ros2` 的嵌套 `AGENTS.md` 针对该子树生效，预期另一 overlay 布局；不得把其 setup 默认路径套到 `fast_livo`，不得自动改包内脚本。
- 定位 launch 与 YAML 默认指向不同且均存在的地图。若任务涉及下游定位，主控须显式选定地图、核验有效参数并冻结哈希；纯 LIO 分析不依赖先验地图时标记不适用。具体路径见专用合同，不静默修改原默认值。
- 环境、时间域、数据、命令、关键指标或回归证据缺失时 fail closed，列出缺项并给 RETEST；绝不伪造实验成功。优先保证可比性，不归因于尚未测量的算法或调度阶段。

## 产物与报告

### 后续方案统一入口

2026-11-21 用户要求总结001闭环经验，并将机械步骤自动化。先读 `plans/lio_benchmark_playbook.md`。后续用户给出方案并要求按本流程验证时，主控登记方案编号/来源/一个假设，生成最小patch及 `experiments/recipes/<编号>.json`；已授权的代码/离线测试步骤不重复索要确认。仅讨论方案时不擅自执行，实机与部署仍需单独授权。

优先使用 `python3 scripts/lio_benchmark.py run --config <方案JSON>`；失败使用 `resume <运行目录>`。该入口负责隔离源码、保存dirty与未跟踪源、同条件构建/测试/回放、完整性审计、统计和报告。禁止重新手工拼一套每次不同的评估流程，禁止覆盖旧日志、绕过冻结校验或自动放宽容限。配置未填完整时必须停止并列缺项。

自动报告独立列出平均80ms/P95 80ms、最大值、>80/>100次数及分母、状态年龄、输出周期、队列、初始化和位姿/TF。初始化指纹不一致、日志不全或重复性未验证时给RETEST，不能仅因平均变快就采用。分析原理、确定最小变更和最终判断仍由主控负责；脚本不自动合并或部署。当前父权限不足时如实报告，不修改或绕过权限。

`papers/` 保存索引/获准资料，`experiments/` 保存冻结合同与真实指标，`experiments/runs/` 保存批准命令通过 `scripts/run_trial.sh` 捕获的合并日志、SHA/status 和时间信息。包装器不代替算法 benchmark、dirty 快照或指标适配器。

算法实验报告须包含假设、最小补丁、准确命令、基线/候选实测表、重复性、回归和 ACCEPT / REJECT / RETEST。纯配置或只读检查报告列出文件、静态检查和缺失证据，实验项写“未执行/不适用”，不造测量表，不声称降低了延迟。

新会话烟测提示：真正并行启动 `paper_researcher` 和 `code_analyst`，只读查看论文索引、现有历史证据和主要算法入口；报告实际任务名与结果，不修改任何文件、不编译、不跑 Benchmark 或 ROS。启动失败则报告具体错误并查版本/信任/角色/模型访问，不模拟角色。

---

# AGENTS.md — wsnew 工作区

## 仓库结构

```
/home/nvidia/wsnew
├── ws_livox/                 # ROS 2 (Humble) colcon 工作区 —— 主要代码在这里
│   ├── src/                  # 源码包（唯一需要修改的地方）
│   ├── build/ install/ log/  # colcon 生成物，禁止手改
│   ├── fix_scan_timestamp.py # 雷达时间戳修复脚本
│   └── odom_to_tf.py         # odom → TF 转换节点
├── start&close_script/       # 启动/关闭脚本（start_livo.sh 等）
├── localization.md           # 定位模块笔记 + 已知问题/时延记录
├── pcd.md                    # 点云相关笔记
└── 室外*.md                   # 外场录包与时延分析
```

`ws_livox/src/` 下的包：

| 包 | 作用 |
|---|---|
| `FAST-LIVO2-master` | LiDAR-惯性-视觉里程计（主 SLAM） |
| `lidar_localization_ros2` | 基于先验地图的激光定位 |
| `livox_ros_driver2` | Livox 雷达驱动 |
| `livox_pointcloud_transformer` | 点云坐标变换 |
| `livox_3d_detector` | 3D 检测 |
| `ndt_omp_ros2` | NDT 配准（OMP 并行） |
| `rpg_vikit` (`vikit_common`, `vikit_ros`) | 视觉工具库 |
| `TFlab` / `radar_imu_tf` | TF / 标定相关 |

## 构建与运行

```bash
# 编译（在 ws_livox 下）
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# 只编译改动过的包
colcon build --symlink-install --packages-select <package_name>

# 运行前必须 source
source /opt/ros/humble/setup.bash
source /home/nvidia/wsnew/ws_livox/install/setup.bash

# 外场启动（约定了启动顺序，不要随意改）
bash "/home/nvidia/wsnew/start&close_script/start_livo.sh"
bash "/home/nvidia/wsnew/start&close_script/stop_livo.sh"
```

## 硬性约定

1. **运行代码修改限于 `ws_livox/src/` 和获准的根目录脚本，且须有本轮任务授权**。Codex 配置、根 `AGENTS.md` / `README.md` 及 `papers/`、`experiments/`、`plans/` 的研究管理资料可在相应任务授权范围内维护；这不授权改运行代码。`build/ install/ log/` 是生成物，禁止手改。
2. C++ 遵循包内现有风格（ROS 2 Humble、`rclcpp`、`ament_cmake`）；Python 节点用 `rclpy`。
3. 改参数默认值前先看 `*_params.yaml` / `config/`，不要在代码里写死魔数。
4. 涉及时间同步、TF 树、坐标系（`map/odom/base_link/livox`）的改动，必须在报告里说明对 TF 树的影响。
5. 没有测试框架的包，用 `colcon build` 通过 + 相关节点能启动作为最低验证；不要声称"测试通过"。
6. 不要升级 ROS 版本、不要全局替换第三方库、不要动 `.git` 里已有的提交历史。

## 验证顺序

下列步骤仅适用于已授权的运行代码变更；方案001已获准在隔离环境构建、测试及原始bag回放，不启动实机启动脚本。

```
colcon build --packages-select <pkg>   →  launch 对应节点看是否有报错   →  检查 topic/TF
```

时延相关的改动，参照 `室外录包时延分析_20261115.md` 里的方法做前后对比，并记录数据。

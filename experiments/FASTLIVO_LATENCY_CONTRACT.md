# FAST-LIVO2 里程计延迟实验合同

## 当前边界

本框架默认只读。当前授权仅配置和研究资料，不包含运行源码/参数/launch/脚本修改、编译、包测试、回放或实机操作。模板、说明和人工检查不构成自动权限控制器。

主要对象是 `ws_livox/src/FAST-LIVO2-master`（`fast_livo` / `fastlivo_mapping`）。`config/nav_test.yaml` 当前 `img_en=0`、`imu_en=true`；不能把视觉优化收益直接迁移为当前 LIO 收益。读取并记录实际 launch、安装包前缀及有效参数后，才可认定某次运行使用了该配置。

## 主控流程与停止条件

1. 按需要并行收集论文/历史证据和源码证据。标注原始论文声明、历史测量、本轮观察与假设，不重复研究已实现的 TF 广播器复用、Path 限长降频。
2. 主控选择一个可证伪假设，写明依据、最小修改范围、相同条件的基线/候选、指标、阈值、有效数据窗口、重复次数、准确命令和停止条件。纯性能分析可依据源码与历史日志，不强求不存在的论文；声称复现论文则必须取得相应正文。
3. 填完整 manifest 并经主控验收前保持 `blocked`。用户明确授权具体后续操作，父会话具有相应权限后，才允许最小修改与实验。物理机器人操作仍需独立批准计划。
4. 修改与实验顺序为 implementer → experimenter；禁止子 Agent 派生 Agent。最多两轮改进后人工审查。只改一个机制，不能顺带改输入、预处理、阈值、指标、线程或记录方式。
5. 时间域不明、原始数据/命令/评估器不可用、基线与候选不可比、日志缺失、TF 冲突、缺关键回归证据时停止，结论为 RETEST。超出预先冻结的回归容限为 REJECT；所有门槛满足才可 ACCEPT。

本轮没有基线/候选测量，也没有设置验收阈值。空阈值不是无限容限。没有真值时可以报告性能观察，但不能宣称精度不退化或接受涉及精度的算法改动。

## 状态时间与耗时

主指标是发布调用前的状态年龄：

`state_age_ms = 1000 * (odom_publish.ros_s - odom_publish.scan_end_s)`。

现有 `publish_odometry()` 将 odom header 填为节点当前时间，不能用 header 的新鲜程度替代状态年龄。此指标不是发布完成、DDS 传输、下游接收或控制执行延迟。若研究下游延迟，须有独立时间对应及测量证据。

传感器时间和 ROS 时间必须处于已确认的同一时间域。历史回放需明确 `/clock`、`use_sim_time`、回放速率及起止窗口，不能用当前墙钟减历史 stamp。耗时使用 `steady_clock`，只作时长，不能与 ROS/传感器绝对时间混减。负年龄、时钟跳变、非有限值和无状态帧应单列为无效，并记录排除规则及数量，不能悄悄截为零或移除。

## 与现有诊断输出的映射

来源：`include/timing_diagnostics.h`、`scripts/summarize_timing.py`、`docs/timing-diagnostics.md`，均位于 FAST-LIVO2 包。沿用已有 nearest-rank 分位数；不在本轮修改汇总或评估逻辑。

| 指标 | 已有 JSON 路径或原始证据 | 单位与边界 |
|---|---|---|
| 状态年龄 p50/p95/max | `events.odom_publish.age_s.{p50,p95,max}` | 秒乘 1000 为 ms；存在无效样本时，现有汇总不足以直接当有效样本结论，先按冻结规则审核原始行 |
| 状态年龄端点趋势 | `scan_age_trend.growth_s_per_wall_s` | 乘 1000 为 ms/s；只是首末有限样本差除 steady 时间差，不是回归斜率，不能代替中段窗口分析 |
| 成功帧完整周期 | `events.full_cycle.by_status["1"].duration_ms` | ms；与 status=0 初始化/提前返回分开，不能混合均值 |
| 相邻处理周期结束间隔 | `events.frame_period.duration_ms` | ms；包含中间等待等，不等于 ROS 订阅者收到 odom 的间隔 |
| 分阶段墙钟耗时 | `events.<event>.duration_ms`，如 `state_estimation`、`voxel_map_update`、`process_imu`、`odom_publish` | ms；包含调度等开销，不是纯 CPU 时间。外层包含内层，不能全部相加 |
| 同步等待 | `events.sync_wait.duration_ms`、`events.wait_reason.statuses` | ms / 原因计数；`wait_spin/wait_sync/wait_sleep` 是窗口累计，不是单次调用耗时 |
| 应用层队列深度 | `events.sync_before.<imu_q|lidar_q|prop_imu_q>` 及原始快照 | 条数；不覆盖 DDS history，不能由短 IMU 队列推断没有回调前积压 |
| 队列时间跨度 | 原始 `sync_before` 的 `lidar_back_s-lidar_front_s` / `imu_back_s-imu_front_s` | 秒乘 1000 为 ms；汇总器不直接输出，只有核验过原始行和转换过程才可填写 |
| 回调消息年龄 | `events.<imu_callback|lidar_callback>.by_status["1"].<header_age_s|corrected_header_age_s>` | 秒乘 1000 为 ms；原始与修正时间分别报告 |
| 回调接纳/拒绝 | `events.<imu_callback|lidar_callback>.statuses`，保留具体 status | 由原始 `count` 累加；不是 DDS 丢包率，不能合并成未测的 stale/drop 数 |
| 点数、迭代、地图与路径规模 | 相应事件的 `input_points/down_points/effective_points/iterations/map_voxels/path_poses` | 依字段语义，比较时关联同一有效帧/窗口，不能直接用相关性断言因果 |

现有汇总器仅过滤非有限数，并不自动校验时间域、剔除负年龄或解决所有帧关联问题。回调 `frame_id=0` 时不能直接归入正在消费的扫描；必须依据 loop 与原始时间建立关系。缺少帧关联或条件分组时保留原始证据并停止相应结论。

尚不能从上述字段直接得到的指标：实际 DDS 丢失/陈旧丢弃、下游接收延迟、ATE/APE/RPE 和完整定位失败率。没有独立测量时标为 missing；不能填零，不能把同步失败次数当丢帧。输出周期改善也不能替代状态年龄改善。

诊断默认关闭，运行前必须核实实际二进制是否含诊断、日志是否完整。ON/OFF 差异本身可能扰动性能；基线与候选须具有相同诊断条件，必要时另行批准配对评估插桩扰动。

## 长表 CSV 约定

使用 `fastlivo_metrics_template.csv`。`dataset_id/pair_id` 标识相同输入与重复次序，`variant` 为 baseline 或 candidate，`event_status` 保留原事件状态，无条件分组则为空。`statistic` 标记 count/mean/p50/p95/max 或已批准的定义。

`availability` 使用 `measured`、`missing`、`invalid`、`not_applicable`。只有 measured 填有限数值、真实 sample_count、单位和证据路径；其余数值留空并填 missing_reason。sample_count 是该指标实际有效样本数，不一定等于事件 represented_count。键 `(dataset_id,pair_id,variant,event,event_status,metric,statistic)` 必须唯一。

保留原 CSV、汇总 JSON、转换规则与版本及文件哈希。现有 JSON 汇总器不输出本框架长表，没有已审核适配器时不声称自动对比可用；可先保留原始结果及人工可追溯表。旧 CPP CSV/比较器不得用于本合同。

## 冻结基线与同条件比较

manifest 至少记录 bag 全部文件与哈希、topic/type、参数与外参/时间偏移、launch、实际可执行路径/哈希与动态依赖身份、编译与诊断开关、Git SHA、dirty diff、未跟踪源码、硬件与CPU/线程/功耗/频率/温度、RMW/QoS、后台负载、回放时钟/速率/区间、暖机/有效窗口、重复次数、评估器/适配器版本、命令与阈值。

源码存在未提交内容时，不能只用 HEAD 重建基线。未来比较源码变体应在独立分支/worktree 保留当前完整工作状态，禁止覆盖现有安装或用 reset/clean 丢弃用户改动。基线和候选除批准的一个机制外条件一致；按配对运行与窗口报告，不能只报全程平均值。

原始录包应包含配置所需的 LiDAR 与 IMU 输入（当前为 `/livox/lidar`、`/livox/imu`），核验类型和逐点时间。不能仅依靠输出 odom/TF 重放算法，也不能把历史输出 TF/odom 混入候选结果。回放需隔离 ROS domain，并记录隔离方式；不能默认启动现有实机启动脚本。

## 环境与地图冲突的处理

- FAST-LIVO2 使用根工作区规则。已只读确认 `/opt/ros/humble/setup.bash`、`/home/nvidia/wsnew/ws_livox/install/setup.bash` 和 `install/fast_livo` 存在；这不证明已安装程序等于当前源码。实际运行前核实所选前缀和二进制身份，不在纯分析阶段执行 ROS 命令。
- 定位包嵌套 `AGENTS.md` 预期另一布局，其 setup 脚本默认指向 `ws_livox/src/build_ws/install/setup.bash`，该路径不存在。禁止把此 overlay 自动用于 fast_livo。若任务明确扩展到定位包，先遵守其嵌套规则，再由主控记录实际布局和显式环境覆盖，不能默默修改包内脚本。
- 定位 launch 默认地图为 `/home/nvidia/ws_lkytest/ws_livox/src/FAST-LIVO2-master/Log/PCD/all_raw_points1.pcd`，YAML 为 `/home/nvidia/wsnew/ws_livox/src/FAST-LIVO2-master/Log/PCD/all_raw_points.pcd`；两个文件均存在但不能假定相同。下游定位实验必须显式选地图、冻结哈希及有效参数，不能静默依赖冲突默认值。纯 LIO 无先验地图依赖时记录 not_applicable，不因该冲突阻断日志分析。
- 不改时间同步、TF、帧名、QoS、点云过滤或输出开关以“准备实验”。任何后续此类差异均需属于明确批准的候选，并说明影响。

这些措施解决框架误用环境与默认地图的问题，原运行文件中的默认值保持不变。默认 read-only 需新会话/重载生效；未来运行会写日志，必须先由用户明确授权并在父会话设置适当权限。任何角色文件都不能覆盖父权限。

# FAST-LIVO2 离线闭环自动化实施计划

用户授权：总结方案001经验，并把能自动化的准备、构建、测试、回放、检查和报告自动化。保持机器人运行源码/参数、正常安装和既有实验原始证据不变；本轮不增加算法方案，不启动实机。此前“只配置/只读”的限制不用于阻止本次已授权的工具开发和离线验证。

## 设计

入口 `python3 scripts/lio_benchmark.py run --config <方案JSON>`。Codex负责理解用户方案、给出最小补丁并写方案配置；入口负责后续机械工作。配置含编号、原始链接、假设、候选patch和允许文件、baseline ref、两包、80ms/窗口/回归阈值。采用JSON+标准库，不引入服务、依赖升级或自动部署。

准备两棵独立worktree，保存当前运行包tracked dirty diff及必要untracked源文件，候选只加允许范围patch。两侧统一应用已验证的shutdown-only common patch（已存在则验证，不可应用则失败），保存精确源范围。用source ROOT_DIR隔离Log；只建独立build/install前缀，不覆盖正常安装。

阶段为prepare/build/tests/replay/report；持久state和每步日志。命令使用argv，测试环境source固定ROS setup，仅ROS_DOMAIN_ID176、localhost、原始输入，1倍速、200Hz clock、30秒排空。复用原观察器/数值解析，不静默更换nearest-rank或字段语义。新入口/审计/判定版本记录哈希。一个流程加运行锁，失败保留原目录并写RETEST报告；resume验证冻结输入/源码/工具/二进制，不覆盖失败attempt，只重做未完成步骤。

自动结果分开输出 mean≤80、p95≤80、max和>80/>100次数/比例、状态年龄、输出周期、队列、TF、对应扫描位姿差、启动IMU接纳指纹及重复性。初始化不一致时不把轨迹差异归因补丁，整体RETEST；不放宽已冻结容限。无真值标missing，不填0。报告与指标CSV、verdict JSON自动生成，方案记录只追加结果，原始用户条目不覆盖。平均达标不等于采用或部署。

## 文件责任与接口

- implementer/pipeline：`scripts/lio_benchmark.py`、`scripts/tests/test_lio_benchmark.py`。CLI run/resume/preflight，工作树/构建/状态/执行/失败报告；不编辑report模块或其他文件。
- implementer/report：`scripts/lio_benchmark_report.py`、`scripts/tests/test_lio_benchmark_report.py`。纯读取/汇总/判定/渲染与CSV输出，不启动节点、不编辑pipeline。
- root：默认recipe `experiments/recipes/lio_default.json`、经验文档 `plans/lio_benchmark_playbook.md`、AGENTS/README及Codex角色入口、整合与验证。

共享接口：report模块提供 `analyze(run_dir: Path, *, write: bool = True) -> dict`。run_dir下 `manifest.json` 含 `contract`（`bags`沿用方案001的path/start_s/duration_s/input_counts/sha256；`warmup_seconds=5`、`high_load_start_s=40`、`high_load_end_s=100`、`target_ms=80`、`regression_gates`沿用已有字段），`runs`包含每个实际run的 `variant`、`dataset`、`repeat`、`path`、`exit_code`、`binary_sha256`；允许兼容旧001的run名字与相对目录。生成comparison.json、REPORT.md、verdict.json、metrics.csv；返回包含 `verdict`、`performance_mean_pass`、`performance_p95_pass`、`reasons`、`datasets` 的结果。重复运行先按pair分别匹配，再检查同一pair源扫描；不跨重复混合掩盖失败。

## 验证与停止

- [ ] 两个实现者先写有意义的失败测试再实现；仅用临时git仓库/小文件、合成日志验证错误场景，不能把它们当benchmark。
- [ ] 配置路径/越界patch、dirty baseline保留、已有输出拒绝覆盖、输入哈希变动拒绝resume、失败步骤不跳过、退出码传播及报告失败处理。
- [ ] 报告测试涵盖NaN/重复/缺扫描、边界80/100、启动差异→RETEST、平均通过P95未通过、无真值missing。
- [ ] 用已有真实V2日志重新汇总到新的导出目录，不改旧comparison或原始CSV。复现81.037273615→65.90308844、114→5，不声称重新跑了ROS。
- [ ] 整个工具单测、CLI preflight、真实prepare-only验证以及一次独立只读审查。无新必要证据不重跑已完成7分钟的性能实验。
- [ ] 主控记录实际验证范围；完整新pipeline的ROS执行若未重跑，最终明确，不宣称验证过。

## 进度

已定义接口与范围。正常运行源码保持不变，历史方案001产物只读保留。

# FAST-LIVO2 里程计延迟研究框架

本次授权：修改 Codex 项目配置、角色指令和研究管理资料；保持所有机器人运行代码、参数、launch、启动脚本及生成物不变。不编译、不运行测试/benchmark、不启动 ROS、不下载论文、不提交 Git。

## 设计与取舍

采用现有四角色框架，专门服务于 `ws_livox/src/FAST-LIVO2-master`（ROS 包 `fast_livo`）。不新建算法实现或评估解析器，不将 CPP 指标套到里程计。定位包只作为下游接口参考，环境或地图配置冲突必须在实验前显式解决，不能自动修改运行文件。

默认父会话使用 `sandbox_mode = "read-only"`；保留 `[agents]` 的启用及三子线程上限，不指定未经验证的模型。四角色继承父权限，角色文件不能设置独立沙箱。未来源码修改需用户明确授权，并在父会话明确切换权限；本轮仍禁止源码修改。配置更新后在新会话核验生效，不能把本会话旧角色的行为当作新配置验证。

根 `AGENTS.md` 是主控契约：默认只读诊断 → 两类证据收集（需要时才并行）→ 主控确定一个假设及可执行实验合同 → 明确授权的最小修改 → 同条件基线/候选实验 → ACCEPT/REJECT/RETEST。最多两轮。保存 dirty baseline、SHA、diff/未跟踪文件和运行二进制身份，不仅记录 HEAD。

主要目标是 odom 发布调用前的状态年龄：`1000 * (odom_publish.ros_s - odom_publish.scan_end_s)`；它不等于下游接收/控制延迟。只有时间域对齐且数据有效才能使用。成功帧计算耗时、输出周期、队列、回调原始/修正时间年龄、同步等待和拒绝计数分别报告。跟踪精度、TF/时间戳语义、丢失数据和发布连续性是回归约束，不能靠改 header、丢帧或降精度制造提升。

## 修改范围

- 修改：`AGENTS.md`、`README.md`、`.codex/config.toml`、四个 `.codex/agents/*.toml`、`experiments/README.md`。
- 新增：`papers/README.md`（索引现有 PDF 和引用，不复制或下载）、`experiments/fastlivo_metrics_template.csv`、`experiments/fastlivo_latency_manifest.template.json`、`experiments/FASTLIVO_LATENCY_CONTRACT.md`。
- 本文件由主控维护。保留既有 CPP CSV 与 `scripts/compare_metrics.py`，明确仅供旧 CPP 工作流，不能误用到 FAST-LIVO2。

## 验证计划

- [x] 主控完成上述配置/文档改动。两次有界 implementer 调度均被中断，未采纳其产物，不计为实现成功；主控接管整合。用户随后误删 AGENTS.md 研究契约，已保留其工作区说明并恢复专用合同。
- [x] 主控核对 TOML/JSON/CSV 语法、角色职责、路径、指标单位与现有诊断输出映射；14 个本地 Markdown 链接目标存在。
- [x] `codex app-server --strict-config --print-startup-config` 返回 0（CLI 0.167.1，仅 PATH alias 创建有只读警告）；未执行包测试、编译、benchmark 或 ROS。
- [x] 对照 `/tmp/fastlivo_codex_framework_before.json` 验证 2614 个运行源码/配置/脚本的 SHA256 未变。
- [x] 已派发有界 experimenter 静态验收任务 `/root/verify_framework`；连续等待后未返回结果，已中断，不计为独立验收成功。主控的语法、链接、指标来源和运行文件哈希核验已通过；新会话加载及算法性能未验证。
- [x] 交付说明区分框架改动与仍缺的实验时间域/版本配对、真值和阈值。历史录包与CSV已找到，但没有重跑或声称降低延迟。

## 验收与停止条件

框架必须能区分“缺数据”和“指标为零”，能阻止使用错配环境、时间域、解析器或 CPP 指标得出优化结论。任何越界写入、配置语法错误或指标映射不实均不接受。当前没有基线/候选实验，不构造测量表。

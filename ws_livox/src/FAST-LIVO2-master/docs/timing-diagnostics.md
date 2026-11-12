# FAST-LIVO2 诊断版使用说明

本补丁用于定位室外累计延迟。它只增加计时、队列快照及迭代计数，不修复算法。数学逻辑、原始调用顺序、TF 结构/消息时间戳、订阅深度、算法缓冲和线程模型保持原样。编译开关 `ENABLE_LIO_DIAGNOSTICS` 默认 OFF。

## 版本与构建

源码基线：`ac47d06`；回退标签：`fastlivo-before-timing-20261112`。

改动位于 `LIVMapper.cpp/.h`、`voxel_map.cpp/.h`、`CMakeLists.txt`，以及本包新增的记录器、汇总脚本、测试和文档。`voxel_map` 只增加实际迭代次数计数。用户已有 `localization.md` 修改不属于补丁。

当前安装程序及其工作区内的动态依赖已备份到：

```text
/home/nvidia/.local/state/fastlivo/backups/before-timing-20261112/
```

备份目录的 `manifest.json` 含原路径和 SHA256。Git 恢复源码不能自动恢复已经安装的可执行程序。本轮独立构建/安装，不覆盖旧安装目录。

插桩后的代码位置（相对定位包）：

| 入口 | 位置 |
|---|---|
| 完整循环、spin、sync及等待统计 | `src/LIVMapper.cpp:685`，`run()` |
| 原始点云/CustomMsg回调 | `src/LIVMapper.cpp:961` / `:1002` |
| IMU回调 | `src/LIVMapper.cpp:1070` |
| 同步返回原因 | `src/LIVMapper.cpp:1216`，`sync_packages()` |
| IMU处理及前后状态时间 | `src/LIVMapper.cpp:366`，`processImu()` |
| 状态估计分发 | `src/LIVMapper.cpp:392` |
| 降采样、估计、地图、发布、I/O分段 | `src/LIVMapper.cpp:462`，`handleLIO()` |
| odom/TF发布、扫描年龄 | `src/LIVMapper.cpp:1652`，`publish_odometry()` |
| 实际迭代数 | `src/voxel_map.cpp:367`，`StateEstimation()` |

参考实现仅用于选择观察点，未移植其算法/同步改动：

- [Intel 每帧计时补丁](https://github.com/open-edge-platform/robotics-ai-suite/blob/main/src/pipelines/fast-livo2-demo/patches/0002-Port-to-ROS2-and-bring-up-Mid-360-D415-on-A2W.patch)
- [hku-mars #225](https://github.com/hku-mars/FAST-LIVO2/issues/225)、[#306](https://github.com/hku-mars/FAST-LIVO2/issues/306)
- [DeepRoboticsLab 实现](https://github.com/DeepRoboticsLab/fast-livo2-deep-robotics)

重新构建诊断版：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/wsnew/ws_livox/install/setup.bash
cmake -S /home/nvidia/wsnew/ws_livox/src/FAST-LIVO2-master \
  -B /home/nvidia/.cache/fastlivo_timing_build/on \
  -DENABLE_LIO_DIAGNOSTICS=ON \
  -DCMAKE_INSTALL_PREFIX=/home/nvidia/.cache/fastlivo_timing_install/on
cmake --build /home/nvidia/.cache/fastlivo_timing_build/on --target fastlivo_mapping -j2
cmake --install /home/nvidia/.cache/fastlivo_timing_build/on
```

验证关闭版本时，用独立的 `off` build/install 目录及 `-DENABLE_LIO_DIAGNOSTICS=OFF`。关闭开关后新记录器不参与编译。当前环境链接存在 OpenCV 4.5/4.10 混合版本警告；编译成功不等于已验证实机运行。

## 采样启动

以下命令供操作者在结束旧定位进程后执行；不要同时启动两个定位实例。它使用相同 `nav_test.py` 及配置、TF 发布方式，只切换包前缀到独立安装的诊断版：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/wsnew/ws_livox/install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export AMENT_PREFIX_PATH="/home/nvidia/.cache/fastlivo_timing_install/on:${AMENT_PREFIX_PATH}"
export FASTLIVO_TIMING_DIR=/home/nvidia/fastlivo_timing
ros2 pkg prefix fast_livo
# 上一条必须显示 /home/nvidia/.cache/fastlivo_timing_install/on
ros2 launch fast_livo nav_test.py
```

每次启动在目录下新建唯一 `run_<time>_<pid>/events.csv`。未设置环境变量则写 `/tmp/fastlivo_timing/`。保留默认日志级别、RViz/PCD/路径发布设置，避免同时改变负载。正常 Ctrl+C 结束后离线汇总；不要用 kill -9，未写出的诊断缓冲可能丢失。原算法仍可能覆盖其 `Log/mat_out.txt` 等旧日志，重要历史资料需提前保留。

```bash
python3 /home/nvidia/wsnew/ws_livox/src/FAST-LIVO2-master/scripts/summarize_timing.py \
  /home/nvidia/fastlivo_timing/run_实际目录/events.csv --skip-seconds 30 > summary.json
```

## 字段和计时口径

- 耗时、回调间隔使用 `steady_clock`。`ros_s` 是采样时的节点时钟，只有确认传感器与节点时钟处于相同时间域，header/scan 年龄才有物理意义。历史回放使用一致的 `/clock` 和 `use_sim_time`；主机墙钟不能直接减历史 stamp。
- `header_age_s = ros_s - raw_stamp_s`，`corrected_header_age_s = ros_s - corrected_stamp_s`。IMU 同时保留驱动原始 stamp 和原代码修正后的 stamp，不引入新的修正。
- `scan_end_s` 是状态对应的 `last_lio_update_time`；`target_end_s` 是本次同步选中的 `lio_time`，初始化时两者可能不同。`process_imu.state_before_s` 单独保存处理前的状态时间，`scan_end_s` 在该阶段结束后填写，避免把上一状态时间标为本帧处理结果。
- `odom_publish` 的 `ros_s` 取自原 odom 发布调用前，`raw_stamp_s` 是原代码实际填写的 odom header；`age_s = ros_s - scan_end_s`。它表示状态年龄，不是底盘执行延迟。该事件耗时覆盖整个 publish_odometry，包括 TF。
- 队列 `imu_q/lidar_q/lidar_stamp_q/prop_imu_q` 以及首尾时间，均为已有容器的只读快照。LiDAR 首尾字段是帧头时间。快照不测量 DDS history；回调消息年龄增长而内部队列短，仍可能存在回调之前的排队。
- 回调 `status=1` 表示入队；负值分别表示禁用(-1)、尚未接到 LiDAR(-2)、IMU 时间倒退(-3)、IMU 跳变拒绝(-4)、空点云(-5)。`callback_gap_ms` 是入口间隔，不是传感器采样间隔。
- `frame_id` 对成功同步分配，初始化/空点云仍有 frame_id；`full_cycle.status=1` 表示本轮实际调用了 odom 发布，0 表示没有。`spin_some` 内的回调、点云预处理、传播 timer 均使用 `frame_id=0`，scan/target 字段缺失，因为一次 spin 可能收到多帧。用 `loop_id` 关联接收轮次，并按原始/修正消息时间与 `sync_before.lidar_front_s` 对照实际消费扫描，不能把全部回调直接归到本轮消费帧。

汇总同时保留总计与 `by_status` 分组。判断正常定位负担优先看 `events.full_cycle.by_status["1"].duration_ms`；初始化/提前返回的 status=0 单列。回调接受/拒绝也分组。等待窗口的 duration_unit 标注为窗口累计，不能把其 P95 当作单次调用 P95。

主要事件：

| 事件 | 范围 |
|---|---|
| `full_cycle` | 成功同步所在这一轮，从 spin_some 前到估计/建图/发布/原有 I/O 返回；包含这一轮诊断开销，不包含之前同步失败的循环 |
| `frame_period` | 相邻成功处理周期结束之间的墙钟时间，含中间所有等待、睡眠及上轮诊断输出 |
| `spin` / `sync` | 成功同步这一轮的执行器与同步耗时 |
| `wait_window` | 同步失败期间的窗口，最多每秒一组，成功同步时补齐不足一秒的窗口 |
| `wait_spin/wait_sync/wait_sleep` | 上述窗口内失败轮次的阶段耗时之和，`count` 为轮次数；不是单次调用耗时分布 |
| `wait_reason` | `count` 为该原因的失败次数：1无LiDAR、2无图像、3无IMU、4点云≤1点、5IMU未覆盖扫描末尾、6其他/LIVO分支 |
| `sync_wait` | 连续同步失败到成功同步的墙钟跨度，含中间回调/睡眠；不是 sync 函数 CPU 时间 |
| `sync_before/sync_after` | 同步前后的算法队列快照 |
| `imu_callback/lidar_callback` | 回调本体，包含原有日志及预处理；不覆盖 DDS 反序列化之前的工作 |
| `lidar_message_copy/lidar_preprocess` | CustomMsg 复制、点云预处理 |
| `imu_prop_callback` | spin_some 内执行的原有高频传播 timer |
| `process_imu` | IMU 传播、去畸变、重力对齐与状态赋值 |
| `estimation_mapping/handle_lio` | 分发及完整 LIO 处理的外层计时 |
| `downsample/downcloud_transform/initial_map_build` | 降采样、降采样点云变换、首次建图 |
| `state_estimation` | StateEstimation 调用；实际迭代数在完整周期记录中 |
| `state_copy_pose_io_and_odom` | 状态复制、原位姿文件输出、姿态转换与 odom/TF 发布 |
| `tf_broadcaster_create/tf_send/odom_send` | 原每帧 TF 发布器创建、TF 发送、odom 发送调用 |
| `map_covariance_and_update/voxel_map_update/map_slide` | 协方差与更新总体、单独地图更新、滑窗条件段 |
| `fullcloud_transform/post_map_publications` | 稠密点云变换，以及地图更新后各项发布 |
| `cloud_publish_and_save/effect_publish/plane_publish/path_publish/mavros_publish` | 对应原发布函数/条件段；点云函数包含原 PCD 保存 |
| `lio_preamble_io/lio_tail_console_and_file_io` | LIO 前置状态输出、末尾终端与状态文件输出 |
| `diagnostic_flush` | CSV 格式化及缓冲文件写出耗时；不等于 fsync 的持久化时延 |

外层计时包含内层，不能将表中全部事件相加。比如 full_cycle 已包含 spin、process_imu、estimation_mapping，而 handle_lio 又包含各项子阶段。未解释时间应按同一层级相减。诊断刷写在当前调用记录之后发生，该耗时会落入相应外层/下一段或下一帧周期，额外用 diagnostic_flush 显示。

CSV 使用512条固定记录缓冲和64KiB stdio缓冲，不新增后台线程。写出失败只关闭诊断；缓冲格式化、时钟读取和读取队列仍有开销。应以同源码 OFF/ON 配对实验评价总扰动，不应将 diagnostic_flush 当作全部诊断成本。

## 室内/室外对照

1. 同一诊断二进制、参数、功耗模式、RViz/录包/导航后台进程设置；记录版本、参数快照、CPU频率/温度与负载。
2. 室内、室外分别重新启动定位，初始化后静止30秒，再按相似速度遥控直行和转弯；每次至少10～15分钟，覆盖历史故障运行时长。先测定位，不需要发导航目标。
3. 记录原始 `/livox/imu`、`/livox/lidar`、odom/TF 与诊断CSV。旧故障包没有原始IMU/LiDAR，不能只靠它回放出本次诊断数据。
4. 每种场景至少两轮；比较相同运行时长窗口的完整周期、frame_period、回调年龄、同步原因、队列时间跨度、降采样点数、有效点数、迭代数、地图规模，以及扫描年龄增长斜率。
5. 若需回放对照，用新原始输入包在独立ROS域以1倍速运行，保持相同输入/参数，分别跑OFF与ON。不要把旧输出odom/TF回放混入新定位结果。

判断路径：输入回调前已旧→继续区分驱动、DDS、执行器调度；回调新而同步后旧且队列时间跨度增长→内部积压；成功处理总体超过实际扫描周期→看哪项随点数/迭代/地图增长；大量等待但计算短→按 wait_reason 检查覆盖关系与时间域。多个机制可同时存在。没有实际计时结果前，不认定ICP、IMU、地图或TF发布为根因。

## 回退

当前正常安装目录未覆盖。结束诊断进程后，在新终端只 source 原工作区，再确认 `ros2 pkg prefix fast_livo` 返回 `/home/nvidia/wsnew/ws_livox/install/fast_livo`，即可使用原安装程序。

源码补丁提交后，可用 `git revert <诊断提交>` 撤销这一个补丁。请勿用全仓库 `reset --hard`，它会丢弃与诊断无关的未提交工作。回退标签用于比对本包原文件；不需要覆盖用户其他修改。

## 验证与边界

单元验证：`python3 -m unittest discover -s tests -v`（在本包目录）。包含缓冲完整性、提前返回记录、不可写路径、模拟写盘失败、P95以及空/非有限样本。另核对移除诊断宏/条件块后，原C++计算语句的token序列与基线一致（仅归一化单条return的新增花括号）。

本次未启动定位/导航节点，未发送运动命令，尚无室内/室外实测计时数据。所有根因判断以采样结果为准。

# ws_livox 工作区架构说明

> 依据：对 `ws_livox/src/` 与根目录脚本的静态阅读与检索，未做实际运行验证。
> 所有行号对应本文写作时的仓库状态。

---

## 1. 包职责

| 目录 | ROS 包名 | 职责 |
|---|---|---|
| `FAST-LIVO2-master` | **`fast_livo`**（目录名 ≠ 包名） | LiDAR-IMU 里程计与建图，当前 `img_en: 0` 视觉不参与估计；发布 `camera_init → aft_mapped` 动态 TF |
| `livox_ros_driver2` | `livox_ros_driver2` | MID-360 驱动，发布 `/livox/lidar` 与 `/livox/imu`，点云消息类型由 `xfer_format` 决定 |
| `lidar_localization_ros2` | `lidar_localization_ros2` | 基于先验 PCD 地图的 NDT/GICP 激光定位，生命周期节点，输出位姿、路径与健康诊断 |
| `ndt_omp_ros2` | `ndt_omp_ros2` | OpenMP 并行 NDT/GICP **纯库**，无节点，被定位包以 `find_package(ndt_omp_ros2 REQUIRED)` 链接 |
| `livox_pointcloud_transformer` | 同名 | 用静态外参把 `/livox/lidar` 转到 `base_link` 系输出 `/livox/lidar_transformed`，同时发布 `base_link → livox_frame` 静态 TF |
| `livox_3d_detector` | 同名 | 订阅 `/livox/lidar`，做 ROI 裁剪 / 地面分离 / 聚类，输出障碍点云、状态与障碍位姿 |
| `rpg_vikit`（`vikit_common`、`vikit_ros`） | 两个包 | 视觉工具库，`fast_livo` 强依赖，无独立节点 |
| `TFlab` | `radar_imu_tf` 等 | 标定工具集（`SensorsCalibration/lidar2imu` 手动 / 自动标定）与静态 TF 发布节点；内部 vendored 了完整 ceres 与 eigen3 |
| 根目录脚本 | — | `fix_scan_timestamp.py`：`/scan → /scan_fixed`，用当前系统时间重打时间戳；`odom_to_tf.py`：IMU 角速度积分 yaw，发布 `odom → base_link` |

依据：
- `src/FAST-LIVO2-master/package.xml:3`（包名 `fast_livo`）
- `src/FAST-LIVO2-master/config/nav_test.yaml:8`（`img_en: 0`）
- `src/FAST-LIVO2-master/src/LIVMapper.cpp:69,1672`（TF broadcaster 与 `camera_init → aft_mapped`）
- `src/livox_ros_driver2/src/lddc.cpp:580,659,630,684,532-548`（话题与类型选择）
- `src/lidar_localization_ros2/package.xml:6`、`src/lidar_localization_ros2/src/lidar_localization_component.cpp:916-988`
- `src/ndt_omp_ros2/package.xml:5`、`src/lidar_localization_ros2/CMakeLists.txt:39`
- `src/livox_pointcloud_transformer/src/pointcloud_transformer.cpp:15-24`、`src/livox_pointcloud_transformer/src/livox_static_tf_publisher.cpp:14,61-62`
- `src/livox_3d_detector/package.xml:6`、`src/livox_3d_detector/src/obstacle_avoidance_node.cpp:56-69`
- `src/FAST-LIVO2-master/CMakeLists.txt:100-101`（`vikit_common` / `vikit_ros` 依赖）
- `src/TFlab/radar_imu_tf/package.xml:6`、`src/TFlab/SensorsCalibration/lidar2imu/auto_calib/ceres/package.xml:34`
- `ws_livox/fix_scan_timestamp.py:9-16`、`ws_livox/odom_to_tf.py:15-72`

---

## 2. 数据流

### 2.1 发布端

| 发布者 | 话题 / TF | 类型 | 依据 |
|---|---|---|---|
| livox_ros_driver2 | `/livox/lidar` | `xfer_format=1` → `livox_ros_driver2/CustomMsg`；`xfer_format=0` 才是 `sensor_msgs/PointCloud2` | `src/livox_ros_driver2/src/lddc.cpp:580,659,532-536`；`src/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py:8` |
| livox_ros_driver2 | `/livox/imu` | `sensor_msgs/Imu` | `src/livox_ros_driver2/src/lddc.cpp:630,684,548` |
| fast_livo | `/aft_mapped_to_init` | `nav_msgs/Odometry` | `src/FAST-LIVO2-master/src/LIVMapper.cpp:326` |
| fast_livo | `/cloud_registered`、`/cloud_effected`、`/Laser_map`、`/path`、`/LIVO2/imu_propagate`、`/visualization_marker`、`/planes`、`/dyn_obj*`、`/mavros/vision_pose/pose` | 点云 / 路径 / 标记 | `src/FAST-LIVO2-master/src/LIVMapper.cpp:321-337` |
| fast_livo | TF `camera_init → aft_mapped` | `tf2_ros::TransformBroadcaster` | `src/FAST-LIVO2-master/src/LIVMapper.cpp:69,1672` |
| lidar_localization_ros2 | `pcl_pose`（remap 为 `/localization/pose_with_covariance`）、`path`、`alignment_status`、`reinitialization_requested`、`initial_map` | PoseWithCovarianceStamped / Path / DiagnosticArray / Bool / PointCloud2 | `src/lidar_localization_ros2/src/lidar_localization_component.cpp:920-938`；`src/lidar_localization_ros2/launch/mid360_legged_localization.launch.py:185` |
| lidar_localization_ros2 | TF `map → odom`（`enable_map_odom_tf:=true`） | 动态 | `src/lidar_localization_ros2/launch/mid360_legged_localization.launch.py:46,162,238` |
| launch 静态 TF | `odom → base_link`(全 0)、`base_link → livox_frame`、`base_link → livox_imu_frame`（默认关闭） | static | `src/lidar_localization_ros2/launch/mid360_legged_localization.launch.py:96-144,256-271` |
| livox_pointcloud_transformer | `/livox/lidar_transformed`（`base_link` 系）、静态 TF `base_link → livox_frame`（读 yaml 外参） | PointCloud2 | `src/livox_pointcloud_transformer/src/pointcloud_transformer.cpp:19-20`；`src/livox_pointcloud_transformer/src/livox_static_tf_publisher.cpp:14,61-62` |
| livox_3d_detector | `/obstacle_avoidance/{cloud_roi,cloud_obstacles,cloud_ground,markers,status,obstacle_poses}`、`/livox/processed_final` | 点云 / MarkerArray / String / PoseArray | `src/livox_3d_detector/src/obstacle_avoidance_node.cpp:61-69` |
| `odom_to_tf.py` | TF `odom → base_link`（仅 yaw，平移恒 0） | 动态 | `ws_livox/odom_to_tf.py:54-72` |
| `fix_scan_timestamp.py` | `/scan_fixed` | LaserScan（时间戳换为当前系统时间） | `ws_livox/fix_scan_timestamp.py:10,13-16` |

### 2.2 订阅端

| 订阅者 | 话题（remap 后） | 依据 |
|---|---|---|
| fast_livo | `/livox/lidar`（`lidar_type: 1` → `CustomMsg` 分支）、`/livox/imu`、`/left_camera/image_raw`（未启用） | `src/FAST-LIVO2-master/config/nav_test.yaml:10-12,64`；`src/FAST-LIVO2-master/src/LIVMapper.cpp:314-319`；`src/FAST-LIVO2-master/launch/nav_test.py:83-86` |
| lidar_localization_ros2 | `cloud`←`/livox/lidar`（**PointCloud2**）、`twist`←`/twist`、`imu`←`/livox/imu`，另有 `map`、`initialpose`、`odom` | `src/lidar_localization_ros2/launch/mid360_legged_localization.launch.py:181-186`；`src/lidar_localization_ros2/src/lidar_localization_component.cpp:944-979` |
| nav2 相关配置 | `/localization/pose_with_covariance`、`/cmd_vel`、`/reinitialization_requested`、`/initialpose`、`/odom` | `src/lidar_localization_ros2/launch/nav2_navigation.launch.py:216,511,538,590,608-610` |
| livox_pointcloud_transformer | `/livox/lidar`（PointCloud2） | `src/livox_pointcloud_transformer/src/pointcloud_transformer.cpp:15-17` |
| livox_3d_detector | `/livox/lidar`（PointCloud2） | `src/livox_3d_detector/src/obstacle_avoidance_node.cpp:56-58` |
| 外部包 `~/MPPI_3dnavi/fishbot_navigation2` | `pcl_to_scan.launch.py`，点云转 `/scan`（工作区外，未核查其内部） | `start&close_script/start_livo.sh:55-57` |
| `fix_scan_timestamp.py` | `/scan` | `ws_livox/fix_scan_timestamp.py:9` |

### 2.3 关键结构性问题：`/livox/lidar` 的消息类型是分裂点

- `fast_livo` 在 `lidar_type: 1` 下订阅 `CustomMsg`（`src/FAST-LIVO2-master/src/LIVMapper.cpp:314`，配置见 `config/nav_test.yaml:64`）。
- 定位包、`livox_pointcloud_transformer`、`livox_3d_detector` 订阅的都是 `sensor_msgs/PointCloud2`（`launch:181-186`；`pointcloud_transformer.cpp:15-17`；`obstacle_avoidance_node.cpp:56-58`）。
- 驱动发布的类型由 `xfer_format` 单一决定（`src/livox_ros_driver2/src/lddc.cpp:532-536`），当前 launch 为 `xfer_format = 1`（`launch_ROS2/msg_MID360_launch.py:8`）。

结论：在当前参数下，`fast_livo` 链路与"定位 / 变换 / 检测"三条链路**不可能同时收到数据**。

---

## 3. 启动顺序（`start&close_script/start_livo.sh`）

| 步 | 动作 | 依赖 | 依据 |
|---|---|---|---|
| 0 | `tmux kill-session -t livo_system`（存在则杀） | — | `start_livo.sh:6-8` |
| 1 | 新建 session `main`，开启鼠标操作 | — | `start_livo.sh:13,17` |
| 2 | **pane0 · Livox Driver**：`cd ~/wsnew/ws_livox && source install/setup.bash && ros2 launch livox_ros_driver2 msg_MID360_launch.py` | `config/MID360_config.json`、网卡与雷达 IP；**未 source `/opt/ros/humble/setup.bash`**，依赖 tmux 默认 shell 的 `~/.bashrc` 已 source ROS | `start_livo.sh:24-27`；`msg_MID360_launch.py:19,22-33` |
| 3 | `sleep 2` | 固定等待，非就绪检测 | `start_livo.sh:29` |
| 4 | **pane1 · FAST-LIVO2**（`split-window -h`）：`ros2 launch fast_livo nav_test.py` | 依赖 pane0 的 `/livox/lidar`(CustomMsg) 与 `/livox/imu`；启动后自带 3 个 `static_transform_publisher`；IMU 初始化需累计 `imu_int_frame: 30` 帧 | `start_livo.sh:35-42`；`launch/nav_test.py:150-239`；`config/nav_test.yaml:109` |
| 5 | `sleep 2` | 同上 | `start_livo.sh:44` |
| 6 | **pane2 · PCL→Scan**（在 pane1 上 `split-window -v`）：`cd /home/nvidia/MPPI_3dnavi && source install/setup.bash && ros2 launch fishbot_navigation2 pcl_to_scan.launch.py` | 依赖**工作区外**目录 `/home/nvidia/MPPI_3dnavi`；其输入必须是 `PointCloud2` | `start_livo.sh:50-57` |
| 7 | 设置 pane 边框标题，默认选中 pane0 | 界面 | `start_livo.sh:63-69` |
| 8 | `gnome-terminal` 附加到该 tmux session | 需要图形会话 | `start_livo.sh:76-78` |

补充事实：仓库中存在**第二套不一致的启动路径**——`livox_manager.sh` 使用 `source install/setup.sh`（`.sh` 应为 `.bash`）且启动 `fast_livo test1.py use_rviz:=true`，与 `start_livo.sh` 的 `nav_test.py` 不同，两套产出的 TF 链与参数不一致。依据：`start&close_script/livox_manager.sh:14,24`。

---

## 4. 易改坏的地方

### ① `/livox/lidar` 的消息类型 —— `src/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py:8`

`xfer_format` 是模块级硬编码常量，**不是 launch argument**，命令行无法覆盖，只能改文件。

- 改成 `0`：`fast_livo`（`LIVMapper.cpp:314` 的 `CustomMsg` 分支）立刻收不到数据。
- 保持 `1`：定位包 / transformer / detector 三条 `PointCloud2` 链路全部收不到数据。
- 后果：`ros2 topic hz /livox/lidar` 显示正常，订阅端零回调且不报错——典型的静默失效。

### ② 同一 child frame 被多方重复发布，TF 树多父

- `launch/nav_test.py:150-239` 用三段静态 TF 拼出 `aft_mapped → base_link → line_frame → livox_frame`，成立前提是 `aft_mapped` 与 `livox_frame` 完全重合（推导见 `nav_test.py:128-147`）。
- `mid360_legged_localization.launch.py:96-110` 又发布 `base_link → livox_frame`（全 0）。
- `livox_static_tf_publisher.cpp:61-62` 依据 yaml 再发布一次 `base_link → livox_frame`。
- `odom → base_link` 同时由 launch 静态 TF（`launch:130-144`）与 `odom_to_tf.py:56-57` 发布。

后果：多父警告 / `TF_OLD_DATA` / 定位与里程计互相拉扯导致跳变；只改动其中一处数值（例如 `pitch = 0.593`）就会破坏 `aft_mapped == livox_frame` 的前提，而平移与旋转必须成逆对出现。

### ③ 定位地图路径指向另一个工作区 —— `src/lidar_localization_ros2/launch/mid360_legged_localization.launch.py:38,231`

默认 `map_path = /home/nvidia/ws_lkytest/ws_livox/src/FAST-LIVO2-master/Log/PCD/all_raw_points1.pcd`，指向 `ws_lkytest` 而非本仓库 `wsnew`。同目录还存在近似克隆文件 `mid360_legged_localization.launch（复件）.py`。

后果：节点能启动但 `initial_map` 为空、生命周期无法进入 ACTIVE、`pcl_pose` 不发布；改在复件上则表现为"改了没生效"。

### ④ `nav_test.yaml` 的 publish 段与 `LIVMapper.cpp` 的发布块 —— `config/nav_test.yaml:175-216`；`src/LIVMapper.cpp:321-337`

该段参数被标注为"改版参数"，注释本身即声明语义不确定（`nav_test.yaml:183,191,197,216`）。

- 关闭 `scan_lidarframe_pub_en`：下游 `pcl_to_scan` 失去点云源。
- 关闭 `pub_odom_en`：`/aft_mapped_to_init` 断流，Nav2 无 odom。
- `imu_rate_odom` 与高频 `/LIVO2/imu_propagate` 耦合。

后果：单个开关造成下游整链静默断流，且当 yaml 开关与 `LIVMapper.cpp` 硬编码话题名不一致时排查困难。

### ⑤ 时间基准被三处各自改写

- 驱动 `use_system_time: 1`（`msg_MID360_launch.py:32`），使用主机时间而非雷达硬件时间。
- `fix_scan_timestamp.py:13-16` 用当前系统时间重打 `/scan`。
- `odom_to_tf.py:55` 用 `now()` 而非 `msg.header.stamp` 打 TF 时间戳。
- 点云 TF 查询严格使用消息时间戳（`pointcloud_transformer.cpp:32,39`）。

后果：点云按雷达时间、scan 按系统时间、TF 按节点当前时间，三者错位 → `TF extrapolation into the future/past`、`pcl_to_scan` 丢帧、定位跳变。改动任一时钟源必须同步核对其余几处。

### 附加地雷（构建期）

1. `TFlab/SensorsCalibration/lidar2imu/auto_calib/` 下 vendored 了完整 ceres 与 eigen3，且带 `package.xml`（`.../auto_calib/ceres/package.xml:34`）。`colcon build` 全量扫描会将其视为独立包，与系统 `libceres-dev` / eigen 冲突或直接构建失败。
2. `start&close_script/livox_manager.sh:14,24` 的 `setup.sh` 拼写错误；`start_livo.sh` 全程不 source `/opt/ros/humble/setup.bash`，与 AGENTS.md 的显式 source 约定不一致。
3. 定位包含 40+ 个 `*_policy.hpp` 策略头文件（`src/lidar_localization_ros2/include/lidar_localization/`），任一策略默认值改动会同时影响 `param/mid360_legged.yaml` 与实验基线；该包约定新行为须先进入 `experiments/`（`src/lidar_localization_ros2/AGENTS.md`）。

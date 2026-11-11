用于3D点云重定位

可能没colconbuild 这个包idar_localization_ros2，及其依赖ndt_omp_ros2

1.先启动rviz，订阅初始地图，实时点云，以及路径等等。用于查看收敛情况
2.启动重定位节点
ros2 launch lidar_localization_ros2 mid360_legged_localization.launch.py（先source）
3.启动livox_ros_driver2用于发布点云livox_lidar供lidar_localization_ros2用于重定位使用
ros2 launch livox_ros_driver2 rviz_MID360_launch.py

主要通过 /tf 话题 和 /localization/pose_with_covariance 话题 来广播位姿。
    消息类型：geometry_msgs/PoseWithCovarianceStamped
    发布频率：由 pose_publish_frequency 参数控制（你的配置里是 20Hz）。
    内容：包含了当前位姿的 位置 (x, y, z) 和朝向 (四元数)，以及表示不确定性的协方差矩阵
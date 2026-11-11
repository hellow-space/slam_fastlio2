#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
3D 点云重定位一键启动（不启动 FAST-LIVO2）

流程：
  livox 驱动(xfer_format=0 -> PointCloud2 /livox/lidar)
  + 静态 TF base_link -> livox_frame
  + lidar_localization(NDT_OMP/GICP_OMP scan-to-map，加载 FAST-LIVO2 的 PCD 地图)

使用：
  ros2 launch lidar_localization_ros2 relocalization.launch.py
  ros2 launch lidar_localization_ros2 relocalization.launch.py use_rviz:=True

初值：
  在 param/localization.yaml 里 set_initial_pose=true 并填写粗位姿；
  或 set_initial_pose=false 后用 RViz 的 2D Pose Estimate 发 /initialpose。
"""

import os

import launch
import launch.actions
import launch.events
import launch_ros
import launch_ros.actions
import launch_ros.events
import lifecycle_msgs.msg

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ---------------- 路径 ----------------
    livox_config = os.path.join(
        get_package_share_directory("livox_ros_driver2"), "config", "MID360_config.json")
    localization_param = os.path.join(
        get_package_share_directory("lidar_localization_ros2"), "param", "localization.yaml")
    rviz_cfg = os.path.join(
        get_package_share_directory("lidar_localization_ros2"), "rviz", "localization.rviz")

    # ---------------- launch 参数 ----------------
    use_rviz = LaunchConfiguration("use_rviz")

    # ---------------- livox 驱动：输出标准 PointCloud2 ----------------
    # xfer_format=0 -> sensor_msgs/PointCloud2，topic /livox/lidar，frame livox_frame
    livox_driver = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_lidar_publisher",
        output="screen",
        parameters=[{
            "xfer_format": 0,          # 0=PointCloud2, 1=CustomMsg
            "multi_topic": 0,
            "data_src": 0,             # 0=在线雷达
            "publish_freq": 10.0,
            "output_data_type": 0,
            "frame_id": "livox_frame",
            "user_config_path": livox_config,
            "cmdline_input_bd_code": "livox0000000001",
        }],
    )

    # ---------------- 静态 TF：base_link -> livox_frame ----------------
    # 默认单位；如有标定外参请替换前 6 个参数 x y z roll pitch yaw
    lidar_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_livox",
        arguments=["0", "0", "0", "0", "0", "0", "1", "base_link", "livox_frame"],
        output="screen",
    )

    # ---------------- 重定位节点（LifecycleNode） ----------------
    lidar_localization = LifecycleNode(
        name="lidar_localization",
        namespace="",
        package="lidar_localization_ros2",
        executable="lidar_localization_node",
        parameters=[localization_param],
        remappings=[("/cloud", "/livox/lidar")],
        output="screen",
    )

    # ---------------- lifecycle 状态机 ----------------
    to_inactive = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(lidar_localization),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    from_unconfigured_to_inactive = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=lidar_localization,
            goal_state="unconfigured",
            entities=[
                launch.actions.LogInfo(msg="-- Unconfigured --"),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(lidar_localization),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                )),
            ],
        )
    )

    from_inactive_to_active = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=lidar_localization,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                launch.actions.LogInfo(msg="-- Inactive --"),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(lidar_localization),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    # ---------------- rviz（可选） ----------------
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        arguments=["--display-config", rviz_cfg],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_rviz", default_value="False",
            description="是否启动 RViz2 可视化"),
        from_unconfigured_to_inactive,
        from_inactive_to_active,
        livox_driver,
        lidar_tf,
        lidar_localization,
        to_inactive,
        rviz_node,
    ])

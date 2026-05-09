#!/usr/bin/python3
# -- coding: utf-8 --

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():

    config_file_dir = os.path.join(get_package_share_directory("fast_livo"), "config")
    avia_config_cmd = os.path.join(config_file_dir, "nav_test.yaml")
    camera_config_cmd = os.path.join(config_file_dir, "camera_pinhole.yaml")

    use_laserscan_arg = DeclareLaunchArgument("use_laserscan", default_value="True")
    avia_config_arg = DeclareLaunchArgument('avia_params_file', default_value=avia_config_cmd)
    camera_config_arg = DeclareLaunchArgument('camera_params_file', default_value=camera_config_cmd)
    use_respawn_arg = DeclareLaunchArgument('use_respawn', default_value='True')

    avia_params_file = LaunchConfiguration('avia_params_file')
    camera_params_file = LaunchConfiguration('camera_params_file')
    use_respawn = LaunchConfiguration('use_respawn')
    use_laserscan = LaunchConfiguration('use_laserscan')

    ld = LaunchDescription([
        use_laserscan_arg,
        avia_config_arg,
        camera_config_arg,
        use_respawn_arg,
    ])

    # 【终极方案】让 camera_init 和 livox_frame 完全重合
    # 这样 /cloud_effected 虽然写 camera_init，但等价于雷达坐标系！
    # tf_camera_to_livox = Node(
    # package="tf2_ros",
    # executable="static_transform_publisher",
    # name="tf_camera_livox_fix",
    # arguments=["0", "0", "0", "0", "0", "0", "livox_frame","camera_init" ],
    # output="screen"
    # )
    # ld.add_action(tf_camera_to_livox)


    # ==============================================
    # 1. FAST-LIVO2 只做去畸变 + 点云输出
    # 绝对不参与 map / odom 坐标系！
    # ==============================================
    fast_livo_node = Node(
        package="fast_livo",
        executable="fastlivo_mapping",
        name="laserMapping",
        parameters=[avia_params_file, camera_params_file],
        output="screen",
        respawn=use_respawn,
        remappings=[
            ('/livox/lidar', '/livox/lidar'),
            ('/livox/imu', '/livox/imu'),
            # ❌ 删掉了错误的 /aft_mapped_to_init → /odom
        ]
    )
    ld.add_action(fast_livo_node)

    # ==============================================
    #  静态 TF：机器人本体 → 雷达
    # 正确方向：base_link → livox_frame
    # ==============================================
    tf_aft_to_livox = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_aft_to_livox",
        arguments=["-0.47", "0", "-0.1", "0", "-0.593", "0", "aft_mapped", "base_link"],
        output="screen"
    )
    ld.add_action(tf_aft_to_livox)

    tf_aft_to_livox = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_aft_to_livox",
        arguments=["0.47", "0", "0.1", "0", "0", "0", "base_link", "line_frame"],
        output="screen"
    )
    ld.add_action(tf_aft_to_livox)

    # ==============================================
    #  静态 TF：机器人本体 → 雷达
    # 正确方向：base_link → livox_frame
    # ==============================================
    tf_base_to_livox = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_to_livox",
        arguments=["0", "0", "0", "0", "0.593", "0", "line_frame", "livox_frame"],
        output="screen"
    )
    ld.add_action(tf_base_to_livox)
    # ==============================================
    # 3. Python 节点：发布正确的 odom → base_link
    # 纯局部里程计，不依赖 SLAM 世界系
    # ==============================================
    # odom_to_tf_node = Node(
    #     executable="/usr/bin/python3",
    #     name="odom_to_tf_node",
    #     arguments=["/home/nvidia/wsnew/ws_livox/odom_to_tf.py"],
    #     output="screen"
    # )
    # ld.add_action(odom_to_tf_node)

    # ==============================================
    # 4. 点云转 scan（坐标系正确：livox_frame）
    # ==============================================
    # laserscan_node = Node(
    #     condition=IfCondition(use_laserscan),
    #     package='pointcloud_to_laserscan',
    #     executable='pointcloud_to_laserscan_node',
    #     name='pointcloud_to_laserscan',
    #     parameters=[{
    #         'target_frame': 'livox_frame',
    #         'transform_tolerance': 0.1,
    #         'min_height': -0.30,
    #         'max_height': 0.30,
    #         'angle_min': -3.14159,
    #         'angle_max': 3.14159,
    #         'angle_increment': 0.0174533,
    #         'scan_time': 0.1,
    #         'range_min': 0.1,
    #         'range_max': 12.0,
    #         'use_inf': False,
    #     }],
    #     remappings=[
    #         ('cloud_in', '/cloud_effected'),
    #         ('scan', '/scan'),
    #     ],
    #     output='screen'
    # )
    #ld.add_action(laserscan_node)

    return ld

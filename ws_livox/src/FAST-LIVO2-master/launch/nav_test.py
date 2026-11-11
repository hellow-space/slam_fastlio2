#!/usr/bin/python3
# -- coding: utf-8 --

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():

    config_file_dir = os.path.join(
        get_package_share_directory("fast_livo"),
        "config"
    )

    avia_config_cmd = os.path.join(
        config_file_dir,
        "nav_test.yaml"
    )

    camera_config_cmd = os.path.join(
        config_file_dir,
        "camera_pinhole.yaml"
    )

    use_laserscan_arg = DeclareLaunchArgument(
        "use_laserscan",
        default_value="True"
    )

    avia_config_arg = DeclareLaunchArgument(
        "avia_params_file",
        default_value=avia_config_cmd
    )

    camera_config_arg = DeclareLaunchArgument(
        "camera_params_file",
        default_value=camera_config_cmd
    )

    use_respawn_arg = DeclareLaunchArgument(
        "use_respawn",
        default_value="True"
    )

    avia_params_file = LaunchConfiguration("avia_params_file")
    camera_params_file = LaunchConfiguration("camera_params_file")
    use_respawn = LaunchConfiguration("use_respawn")

    ld = LaunchDescription([
        use_laserscan_arg,
        avia_config_arg,
        camera_config_arg,
        use_respawn_arg,
    ])

    # ============================================================
    # 1. FAST-LIVO2
    #
    # FAST-LIVO2 负责：
    #   camera_init -> aft_mapped
    #
    # aft_mapped 代表 FAST-LIVO2 当前雷达/机体位姿。
    # 我们希望最终：
    #
    #   aft_mapped 与 livox_frame 完全重合
    #
    # ============================================================

    fast_livo_node = Node(
        package="fast_livo",
        executable="fastlivo_mapping",
        name="laserMapping",
        parameters=[
            avia_params_file,
            camera_params_file
        ],
        output="screen",
        respawn=use_respawn,
        remappings=[
            ("/livox/lidar", "/livox/lidar"),
            ("/livox/imu", "/livox/imu"),
        ]
    )

    ld.add_action(fast_livo_node)

    # ============================================================
    # TF 总体结构
    #
    # camera_init
    #      |
    #      | FAST-LIVO2 动态 TF
    #      v
    # aft_mapped
    #      |
    #      | 雷达外参逆变换
    #      v
    # base_link
    #      |
    #      | 纯平移
    #      | x = +0.535 m
    #      | z = +0.310 m
    #      v
    # line_frame
    #      |
    #      | 纯旋转
    #      | pitch = +0.593 rad
    #      v
    # livox_frame
    #
    #
    # 最终满足：
    #
    #     aft_mapped == livox_frame
    #
    # ============================================================


    # ============================================================
    # 2. aft_mapped -> base_link
    #
    # 这是 base_link -> livox_frame 的完整逆变换。
    #
    # base_link -> livox:
    #
    #   Translation:
    #       x = +0.535
    #       y =  0
    #       z = +0.310
    #
    #   Rotation:
    #       pitch = +0.593
    #
    # 求逆后：
    #
    #   aft_mapped -> base_link:
    #
    #       x = -0.27041439
    #       y =  0
    #       z = -0.55605850
    #
    #       pitch = -0.593
    #
    # ============================================================

    tf_aft_to_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_aft_to_base",
        arguments=[
            "-0.27041439",
            "0",
            "-0.55605850",
            "0",
            "-0.593",
            "0",
            "aft_mapped",
            "base_link"
        ],
        output="screen"
    )

    ld.add_action(tf_aft_to_base)


    # ============================================================
    # 3. base_link -> line_frame
    #
    # 只做位移！
    #
    # 雷达安装位置：
    #
    #   前方：0.535 m
    #   高度：0.310 m
    #
    # 不允许在这里旋转。
    #
    # ============================================================

    tf_base_to_line = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_to_line",
        arguments=[
            "0.535",
            "0",
            "0.31",

            "0",
            "0",
            "0",

            "base_link",
            "line_frame"
        ],
        output="screen"
    )

    ld.add_action(tf_base_to_line)


    # ============================================================
    # 4. line_frame -> livox_frame
    #
    # 只做旋转！
    #
    # 保留原来的雷达倾斜角：
    #
    #   pitch = +0.593 rad
    #         ≈ +34°
    #
    # Translation 必须为 0。
    #
    # ============================================================

    tf_line_to_livox = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_line_to_livox",
        arguments=[
            "0",
            "0",
            "0",

            "0",
            "0.593",
            "0",

            "line_frame",
            "livox_frame"
        ],
        output="screen"
    )

    ld.add_action(tf_line_to_livox)

    return ld
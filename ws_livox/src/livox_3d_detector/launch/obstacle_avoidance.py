import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # ============== 1. 加载避障参数 ==============
    pkg_path = get_package_share_directory('livox_3d_detector')
    yaml_path = os.path.join(pkg_path, 'config', 'obstacle_params.yaml')

    # ============== 2. 启动 Mid360 驱动（ROS2 版） ==============
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[
            {"xfer_format": 0},
            {"multi_topic": 0},
            {"data_src": 0},
            {"publish_freq": 10.0},
            {"output_data_type": 0},
            {"frame_id": "livox_frame"},
            {"user_config_path": os.path.join(
                get_package_share_directory('livox_ros_driver2'),
                'config', 'MID360_config.json'
            )}
        ]
    )

    # ============== 3. 启动 RViz2 ==============
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['--display-config', os.path.join(
            get_package_share_directory('livox_ros_driver2'),
            'config', 'display_point_cloud_ROS2.rviz'
        )]
    )

    # ============== 4. 启动避障节点 ==============
    obstacle_node = Node(
        package='livox_3d_detector',
        executable='obstacle_avoidance_node',
        name='obstacle_avoidance_node',
        output='screen',
        parameters=[yaml_path],  # 加载你的参数文件
        remappings=[
            # 订阅 Mid360 正确话题
            ("/input_pointcloud", "/livox/lidar"),
            # 输出话题保持不变
            ("/livox/processed_final", "/livox/processed_final"),
        ]
    )

    return LaunchDescription([
        livox_driver,
        rviz2,
        obstacle_node,
    ])
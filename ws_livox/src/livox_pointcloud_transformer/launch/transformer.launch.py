from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('livox_pointcloud_transformer')
    yaml_path = os.path.join(pkg, 'config', 'livox_extrinsics.yaml')

    return LaunchDescription([
        # 1. 发布 Livox -> base_link 静态TF
        Node(
            package='livox_pointcloud_transformer',
            executable='livox_static_tf_publisher',
            name='livox_static_tf_node',
            parameters=[{'yaml_path': yaml_path}],
            output='screen'
        ),
        # 2. 点云变换节点（倾斜 -> 平）
        Node(
            package='livox_pointcloud_transformer',
            executable='pointcloud_transformer',
            name='pointcloud_transformer_node',
            output='screen'
        )
    ])


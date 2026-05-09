from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('radar_imu_tf')
    yaml_path = os.path.join(pkg, 'config', 'livox_body_tf.yaml')

    return LaunchDescription([
        Node(
            package='radar_imu_tf',
            executable='livox_body_publisher',
            name='livox_body_tf_node',
            parameters=[{'yaml_path': yaml_path}],
            output='screen'
        )
    ])


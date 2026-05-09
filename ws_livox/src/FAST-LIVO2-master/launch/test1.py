
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
    
    # Find path - 保持原有路径不变
    config_file_dir = os.path.join(get_package_share_directory("fast_livo"), "config")
    rviz_config_file = os.path.join(get_package_share_directory("fast_livo"), "rviz_cfg", "fast_livo2.rviz")

    # Load parameters - 保持原有参数文件不变
    avia_config_cmd = os.path.join(config_file_dir, "test.yaml")
    camera_config_cmd = os.path.join(config_file_dir, "camera_pinhole.yaml")

    # Param use_rviz - 保持原有参数
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="False",
        description="Whether to launch Rviz2",
    )

    # 新增：是否启用点云转激光的参数
    use_laserscan_arg = DeclareLaunchArgument(
        "use_laserscan",
        default_value="True",
        description="Whether to convert pointcloud to laserscan for navigation",
    )
    
    # 新增：是否启用发布pcd地图转换的参数
    use_map_pub_arg = DeclareLaunchArgument(
        "use_map_pub", default_value="False",
        description="Whether to launch the global_map_publisher node"
    )
    
    # 新增：是否启用全局定位节点的参数
    use_global_localization_arg = DeclareLaunchArgument(
        "use_global_localization", default_value="False",
        description="Whether to launch the global_localization node"
    )
    
    # 新增：是否启用pcd_to_occupancy_grid节点的参数
    use_pcd_to_occupancy_grid_arg = DeclareLaunchArgument(
        "use_pcd_to_occupancy_grid", default_value="False",
        description="Whether to launch the pcd_to_occupancy_grid node"
    )
    
    pcd_file_arg = DeclareLaunchArgument(
        'pcd_file',
        default_value='/home/nvidia/wsnew/ws_livox/src/FAST-LIVO2-master/Log/PCD/all_raw_points.pcd',  # 默认文件
        description='Full path to the .pcd file for the occupancy grid'
    )
    pcd_file = LaunchConfiguration('pcd_file')

    avia_config_arg = DeclareLaunchArgument(
        'avia_params_file',
        default_value=avia_config_cmd,
        description='Full path to the ROS2 parameters file to use for fast_livo2 nodes',
    )

    camera_config_arg = DeclareLaunchArgument(
        'camera_params_file',
        default_value=camera_config_cmd,
        description='Full path to the ROS2 parameters file to use for vikit_ros nodes',
    )

    use_respawn_arg = DeclareLaunchArgument(
        'use_respawn', 
        default_value='True',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.'
    )

    # 获取参数值
    avia_params_file = LaunchConfiguration('avia_params_file')
    camera_params_file = LaunchConfiguration('camera_params_file')
    use_respawn = LaunchConfiguration('use_respawn')
    use_laserscan = LaunchConfiguration('use_laserscan')  
    use_map_pub = LaunchConfiguration('use_map_pub')   
    use_global_localization = LaunchConfiguration('use_global_localization')  
    use_pcd_to_occupancy_grid = LaunchConfiguration('use_pcd_to_occupancy_grid')  

    # 创建启动描述
    ld = LaunchDescription([
        use_rviz_arg,
        use_laserscan_arg,  
        use_map_pub_arg,          
        use_global_localization_arg,  
        use_pcd_to_occupancy_grid_arg,  
        pcd_file_arg,      
        avia_config_arg,
        camera_config_arg,
        use_respawn_arg,
    ])

    # ==================== 原有节点 ====================
    # 1. Fast-LIVO2主节点 - 完全不变
    fast_livo_node = Node(
        package="fast_livo",
        executable="fastlivo_mapping",
        name="laserMapping",
        parameters=[
            avia_params_file,
            camera_params_file,
        ],
        output="screen",
        respawn=use_respawn
    )
    ld.add_action(fast_livo_node)

    # ==================== 新增节点（可选） ====================
    # 2. 点云转LaserScan节点 - 只在启用时添加
    laserscan_node = Node(
        condition=IfCondition(use_laserscan),
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        parameters=[{
            # 基础参数
            'target_frame': 'camera_init',
            'transform_tolerance': 0.01,
            
            # 高度过滤
            'min_height': -0.30,      # 过滤地面
            'max_height': 0.30,       # 保留有效高度
            
            # 扫描角度
            'angle_min': -3.14159,   # -π (-180度)
            'angle_max': 3.14159,    # +π (+180度)
            'angle_increment': 0.0174533,  # 1度 (π/180)
            
            # 扫描参数
            'scan_time': 0.1,
            'range_min': 0.1,
            'range_max': 12.0,
            
            # 其他参数
            'use_inf': False,
            'inf_epsilon': 1.0,
        }],
        remappings=[
            # Fast-LIVO2的点云话题 -> 转换器 -> 标准scan话题
            ('cloud_in', '/cloud_registered'),  # Fast-LIVO2的原始输出
            ('scan', '/scan'),                  # 导航需要的标准话题
        ],
        output='screen'
    )
    ld.add_action(laserscan_node)

    # --------------- 新增：地图发布节点 ---------------
    #map_pub_node = Node(
    #    condition=IfCondition(use_map_pub),
    #    package='fast_livo',        
    #    executable='global_map_publisher.py',  
    #    name='global_map_publisher',
    #    parameters=[{
   #         'pcd_path': pcd_file,          
    #    }],
    #    output='screen')
    #ld.add_action(map_pub_node)

    # --------------- 新增：全局定位节点 ---------------
   # global_localization_node = Node(
    #    condition=IfCondition(use_global_localization),
   #     package='fast_livo',  
   #     executable='global_localization.py',  
   #     name='global_localization',
   #     output='screen')
   # ld.add_action(global_localization_node)

    # --------------- 新增：pcd_to_occupancy_grid节点 ---------------
    pcd_to_occupancy_grid_node = Node(
        condition=IfCondition(use_pcd_to_occupancy_grid),
        package='fast_livo',        
        executable='pcd_to_occupancy_grid.py',  
        name='pcd_to_occupancy_grid',
        parameters=[{
            'pcd_path': pcd_file,          
        }],
        output='screen')
    ld.add_action(pcd_to_occupancy_grid_node)

    # 1. camera_init 重命名为 map
    tf_map_to_camera = Node(
        condition=IfCondition(use_laserscan),
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_map_to_camera",
        arguments=["0", "0", "0", "0", "0", "0", "camera_init", "map"],
        output="screen"
    )
    ld.add_action(tf_map_to_camera)

    # 2. aft_mapped 重命名为 odom  
    tf_odom_to_aft = Node(
        condition=IfCondition(use_laserscan),
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_odom_to_aft",
        arguments=["0", "0", "0", "0", "0", "0", "aft_mapped", "odom"],
        output="screen"
    )
    ld.add_action(tf_odom_to_aft)

    # 3. 新增：odom -> base_link（关键连接！）
    tf_odom_to_base = Node(
        condition=IfCondition(use_laserscan),
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_odom_to_base",
        arguments=["-0.47", "0", "-0.1", "0", "34", "0", "odom", "base_link"],
        output="screen"
    )
    ld.add_action(tf_odom_to_base)

    # 4. base_link -> base_laser
    tf_base_to_laser = Node(
        condition=IfCondition(use_laserscan),
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_base_to_laser",
        arguments=["0.1", "0", "0.2", "0", "0", "0", "base_link", "base_laser"],
        output="screen"
    )
    ld.add_action(tf_base_to_laser)

    # ==================== RViz节点 ====================
    # 4. RViz - 保持原有不变
    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen"
    )
    ld.add_action(rviz_node)

    return ld

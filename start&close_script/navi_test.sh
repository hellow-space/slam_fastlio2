#!/bin/bash

start() {
 
    echo "启动 Livox 驱动..."
    gnome-terminal --title="Livox_Driver" -- bash -c '
        cd ~/wsnew/ws_livox
        source install/setup.sh
        ros2 launch livox_ros_driver2 msg_MID360_launch.py
        exec bash
    ' &

    sleep 2

    echo "启动 FAST-LIVO2..."
    gnome-terminal --title="FAST-LIVO2" -- bash -c '
        cd ~/wsnew/ws_livox
        source install/setup.sh
        source install/setup.bash
        ros2 launch fast_livo nav_test.py
        exec bash
    ' &
    
    sleep 2
    echo "启动 pcl"
    gnome-terminal --title="pcl" -- bash -c '
        cd /home/nvidia/MPPI_3dnavi
        source install/setup.bash
        ros2 launch fishbot_navigation2 pcl_to_scan.launch.py
        exec bash
    ' &
}

start

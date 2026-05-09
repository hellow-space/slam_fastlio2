#!/bin/bash

echo "正在关闭 Livox 驱动..."
pkill -f "ros2 launch livox_ros_driver2 msg_MID360_launch.py" 2>/dev/null || true

echo "正在关闭 FAST-LIVO2..."
pkill -f "ros2 launch fast_livo nav_test.py" 2>/dev/null || true

echo "正在关闭 pcl 节点..."
pkill -f "ros2 launch fishbot_navigation2 pcl_to_scan.launch.py" 2>/dev/null || true

# 可选：通过 wmctrl 关闭特定标题的 gnome-terminal 窗口（需安装 wmctrl）
if command -v wmctrl &>/dev/null; then
    echo "通过窗口管理器关闭终端窗口..."
    wmctrl -c "Livox_Driver" 2>/dev/null || true
    wmctrl -c "FAST-LIVO2" 2>/dev/null || true
    wmctrl -c "pcl" 2>/dev/null || true
else
    echo "提示: 未安装 wmctrl，无法自动关闭终端窗口。"
fi

echo "所有进程已关闭。"

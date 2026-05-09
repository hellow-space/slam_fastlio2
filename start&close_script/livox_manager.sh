#!/bin/bash

# 使用临时文件记录子shell的PID（不是gnome-terminal的PID）
PID_FILE_LIVOX="/tmp/livox_bash_pid"
PID_FILE_FAST="/tmp/fast_bash_pid"

start() {
    echo "启动 Livox 终端..."
    
    # 终端一：Livox Driver
    # 使用$$获取bash的真实PID，写入文件
    gnome-terminal --title="Livox_Driver" -- bash -c '
        echo $$ > '"$PID_FILE_LIVOX"'
        cd ~/wsnew/ws_livox && source install/setup.sh && ros2 launch livox_ros_driver2 msg_MID360_launch.py
        rm -f '"$PID_FILE_LIVOX"'
        exit
    ' &
    
    sleep 2
    
    # 终端二：FAST_LIVO
    gnome-terminal --title="FAST_LIVO" -- bash -c '
        echo $$ > '"$PID_FILE_FAST"'
        cd ~/wsnew/ws_livox && source install/setup.sh && ros2 launch fast_livo test1.py use_rviz:=true
        rm -f '"$PID_FILE_FAST"'
        exit
    ' &
    
    sleep 1
    echo "两个终端已启动"
    echo "再次运行此脚本查看关闭选项"
}

check_running() {
    local pid_file=$1
    if [ -f "$pid_file" ]; then
        local pid=$(cat "$pid_file")
        # 检查进程是否存在且是bash或ros2
        if ps -p "$pid" > /dev/null 2>&1; then
            # 检查命令行是否包含相关关键字
            if ps -p "$pid" -o args= 2>/dev/null | grep -qE "(bash|ros2|python)"; then
                return 0  # 运行中
            fi
        fi
    fi
    return 1  # 未运行
}

stop_menu() {
    echo ""
    echo "========== 关闭选项 =========="
    
    if check_running "$PID_FILE_LIVOX"; then
        echo "1 : 关闭 Livox_Driver 终端 [运行中]"
    else
        echo "1 : 关闭 Livox_Driver 终端 [已停止]"
        rm -f "$PID_FILE_LIVOX"
    fi
    
    if check_running "$PID_FILE_FAST"; then
        echo "2 : 关闭 FAST_LIVO 终端 [运行中]"
    else
        echo "2 : 关闭 FAST_LIVO 终端 [已停止]"
        rm -f "$PID_FILE_FAST"
    fi
    
    echo "0 : 退出"
    echo ""
    echo -n "请选择: "
    read choice
    
    case $choice in
        1)
            if check_running "$PID_FILE_LIVOX"; then
                local pid=$(cat "$PID_FILE_LIVOX")
                echo "正在关闭 Livox_Driver..."
                # 获取进程组ID并发送SIGINT
                local pgid=$(ps -o pgid= "$pid" 2>/dev/null | tr -d ' ')
                if [ -n "$pgid" ]; then
                    kill -SIGINT -- "-$pgid" 2>/dev/null || kill -SIGINT "$pid" 2>/dev/null
                else
                    kill -SIGINT "$pid" 2>/dev/null
                fi
                # 等待退出
                local count=0
                while check_running "$PID_FILE_LIVOX" && [ $count -lt 30 ]; do
                    sleep 1
                    ((count++))
                done
                if check_running "$PID_FILE_LIVOX"; then
                    # 强制关闭
                    pgid=$(ps -o pgid= "$pid" 2>/dev/null | tr -d ' ')
                    [ -n "$pgid" ] && kill -9 -- "-$pgid" 2>/dev/null || kill -9 "$pid" 2>/dev/null
                    echo "Livox_Driver 已强制关闭"
                else
                    echo "Livox_Driver 已关闭"
                fi
                rm -f "$PID_FILE_LIVOX"
            else
                echo "Livox_Driver 未运行"
                rm -f "$PID_FILE_LIVOX"
            fi
            ;;
        2)
            if check_running "$PID_FILE_FAST"; then
                local pid=$(cat "$PID_FILE_FAST")
                echo "正在关闭 FAST_LIVO..."
                local pgid=$(ps -o pgid= "$pid" 2>/dev/null | tr -d ' ')
                if [ -n "$pgid" ]; then
                    kill -SIGINT -- "-$pgid" 2>/dev/null || kill -SIGINT "$pid" 2>/dev/null
                else
                    kill -SIGINT "$pid" 2>/dev/null
                fi
                local count=0
                while check_running "$PID_FILE_FAST" && [ $count -lt 30 ]; do
                    sleep 1
                    ((count++))
                done
                if check_running "$PID_FILE_FAST"; then
                    pgid=$(ps -o pgid= "$pid" 2>/dev/null | tr -d ' ')
                    [ -n "$pgid" ] && kill -9 -- "-$pgid" 2>/dev/null || kill -9 "$pid" 2>/dev/null
                    echo "FAST_LIVO 已强制关闭"
                else
                    echo "FAST_LIVO 已关闭"
                fi
                rm -f "$PID_FILE_FAST"
            else
                echo "FAST_LIVO 未运行"
                rm -f "$PID_FILE_FAST"
            fi
            ;;
        0)
            exit 0
            ;;
        *)
            echo "无效选择"
            ;;
    esac
}

# 主逻辑
if check_running "$PID_FILE_LIVOX" || check_running "$PID_FILE_FAST"; then
    stop_menu
else
    # 清理残留文件
    rm -f "$PID_FILE_LIVOX" "$PID_FILE_FAST"
    start
fi

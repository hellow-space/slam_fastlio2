#!/bin/bash

SESSION="livo_system"

# 如果旧 session 存在，先关掉
if tmux has-session -t "$SESSION" 2>/dev/null; then
    tmux kill-session -t "$SESSION"
fi

# ============================================================
# 创建 tmux，会先得到 pane 0
# ============================================================
tmux new-session -d -s "$SESSION" -n "main"

# 开启鼠标操作
# 之后可以直接鼠标点击不同 pane
tmux set-option -t "$SESSION" mouse on

# ============================================================
# Pane 0：Livox Driver
# ============================================================
tmux select-pane -t "$SESSION:0.0" -T "Livox Driver"

tmux send-keys -t "$SESSION:0.0" \
"cd ~/wsnew/ws_livox && \
source install/setup.bash && \
ros2 launch livox_ros_driver2 msg_MID360_launch.py" C-m

sleep 2

# ============================================================
# Pane 1：FAST-LIVO2
# 右侧分屏
# ============================================================
tmux split-window -h -t "$SESSION:0.0"

tmux select-pane -t "$SESSION:0.1" -T "FAST-LIVO2"

tmux send-keys -t "$SESSION:0.1" \
"cd ~/wsnew/ws_livox && \
source install/setup.bash && \
ros2 launch fast_livo nav_test.py" C-m

sleep 2

# ============================================================
# Pane 2：PCL
# 把右侧再上下切开
# ============================================================
tmux split-window -v -t "$SESSION:0.1"

tmux select-pane -t "$SESSION:0.2" -T "PCL"

tmux send-keys -t "$SESSION:0.2" \
"cd /home/nvidia/MPPI_3dnavi && \
source install/setup.bash && \
ros2 launch fishbot_navigation2 pcl_to_scan.launch.py" C-m


# ============================================================
# 显示每个 pane 的标题
# ============================================================
tmux set-option -t "$SESSION" pane-border-status top
tmux set-option -t "$SESSION" pane-border-format \
" #{pane_index} | #{pane_title} "


# 默认选中 Livox
tmux select-pane -t "$SESSION:0.0"


# ============================================================
# 弹出一个新的 GNOME Terminal
# 并在里面进入这个 tmux
# ============================================================
gnome-terminal \
    --title="LIVO SYSTEM" \
    -- bash -c "tmux attach-session -t $SESSION"
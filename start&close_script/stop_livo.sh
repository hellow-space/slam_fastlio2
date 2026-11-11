#!/bin/bash

SESSION="livo_system"

if ! tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "$SESSION 没有运行。"
    exit 0
fi

echo "正在停止 Livox / FAST-LIVO2 / PCL ..."

# 给所有 pane 发送 Ctrl+C
for pane in $(tmux list-panes -t "$SESSION" -F '#{pane_id}'); do
    tmux send-keys -t "$pane" C-c
done

# 给 ROS 节点一点退出时间
sleep 3

# 关闭整个 tmux session
tmux kill-session -t "$SESSION"

echo "全部关闭完成。"
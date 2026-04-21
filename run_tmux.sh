#!/bin/bash

# 检查是否安装了 tmux
if ! command -v tmux &> /dev/null; then
    echo "[ERROR] tmux is not installed. Please run: sudo apt-get install tmux"
    exit 1
fi

echo "[SYSTEM] Checking and Initializing CAN FD interfaces..."
if ! ip link show vcan0 > /dev/null 2>&1; then
    sudo modprobe vcan
    sudo ip link add dev vcan0 type vcan
    sudo ip link set up vcan0 mtu 72
    sudo ip link add dev vcan1 type vcan
    sudo ip link set up vcan1 mtu 72
fi

# 清理旧进程
pkill -f world_server
pkill -f vcu_node
pkill -f socat

# 启动虚拟硬件
socat -d -d PTY,link=/tmp/v-bus-m,raw,echo=0 PTY,link=/tmp/v-bus-v,raw,echo=0 2> /dev/null &
sleep 1 

# 启动车厢节点
./vcu_node 1 > vcu_node_1.log 2>&1 &
./vcu_node 2 > vcu_node_2.log 2>&1 &
./vcu_node 3 > vcu_node_3.log 2>&1 &
./vcu_node 4 > vcu_node_4.log 2>&1 &

# ==========================================
# TMUX 自动化分屏魔法开始
# ==========================================
SESSION_NAME="SDVCU_Sim"

# 杀死已经存在的同名 tmux 会话
tmux kill-session -t $SESSION_NAME 2>/dev/null

# 创建一个后台运行的新 tmux 会话
tmux new-session -d -s $SESSION_NAME

# 此时我们有一个大窗口 (编号 0)。让它运行 world_server
tmux send-keys -t $SESSION_NAME:0 "./world_server" C-m

# 垂直切割屏幕（右边分出一半，编号 1）
tmux split-window -h -t $SESSION_NAME:0

# 在右边的窗口运行 CAN 总线监控 (带颜色)
tmux send-keys -t $SESSION_NAME:0.1 "candump -c vcan0" C-m

# 将右边的窗口再水平切割一次（右下角变出编号 2）
tmux split-window -v -t $SESSION_NAME:0.1

# 在右下角实时滚动查看 3号车 的死亡日志
tmux send-keys -t $SESSION_NAME:0.2 "tail -f vcu_node_3.log" C-m

# 调整一下左边主窗口的大小 (让 server 表格有足够的空间)
tmux resize-pane -R -t $SESSION_NAME:0.0 20

# 绑定所有窗格，附加到终端展示给用户！
echo "[SYSTEM] Launching TMUX Dashboard..."
sleep 1
tmux attach-session -t $SESSION_NAME
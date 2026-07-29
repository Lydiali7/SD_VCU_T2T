#!/bin/bash

# Cleanup function to kill all background processes
cleanup() {
    echo -e "\n[SYSTEM] Caught Ctrl+C! Initiating emergency shutdown..."
    pkill -f vcu_node
    pkill -f world_server
    pkill -f socat
    echo "[SYSTEM] All SD-VCU nodes and Virtual Bus terminated."
    echo "[SYSTEM] Check vcu_node_X.log for individual train logs."
    echo "[SYSTEM] Check fleet_log.csv for physics simulation data."
    exit 0
}

# Trap the SIGINT (Ctrl+C) signal
trap cleanup SIGINT

# Ensure clean state before starting
pkill -f world_server
pkill -f vcu_node
pkill -f socat

echo "[SYSTEM] Initializing Virtual Hardware Bus (socat)..."
# Start socat in background and suppress its startup logs
socat -d -d PTY,link=/tmp/v-bus-m,raw,echo=0 PTY,link=/tmp/v-bus-v,raw,echo=0 2> /dev/null &

# Give socat 1 second to create the virtual port files
sleep 1 

echo "[SYSTEM] Starting 4 distributed VCU nodes in background..."
# Redirect VCU node outputs to individual log files to prevent terminal tearing
./vcu_node 1 > vcu_node_1.log 2>&1 &
./vcu_node 2 > vcu_node_2.log 2>&1 &
./vcu_node 3 > vcu_node_3.log 2>&1 &
./vcu_node 4 > vcu_node_4.log 2>&1 &

sleep 0.5
echo "[SYSTEM] Starting background CAN bus recorder..."
# 使用 -l 参数，让它在后台静默运行，自动把总线数据存入当前目录的日志文件中
candump -l vcan0 &
candump_pid=$!
echo "[SYSTEM] Starting World Server Physics Engine..."
echo "[SYSTEM] UI Monitor will appear shortly. Press [Ctrl+C] to exit."
sleep 1

# Run world_server in the foreground so its table renders cleanly on the terminal
./world_server

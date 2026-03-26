#!/bin/bash

cleanup() {
    echo -e "\n🛑 Caught Ctrl+C! Shutting down all nodes..."
    pkill -f vcu_node
    pkill -f world_server
    echo "Simulation ended. Run python3 plot_fleet.py to view results."
    exit 0
}

trap cleanup SIGINT

# 运行前先清理可能存在的僵尸进程
pkill -f world_server
pkill -f vcu_node

echo "Starting world_server..."
./world_server &
sleep 1

echo "Starting 4 distributed VCU nodes..."
./vcu_node 1 &
./vcu_node 2 &
./vcu_node 3 &
./vcu_node 4 &

echo "All systems online. Simulation running..."
echo "⚠️ Press [Ctrl+C] to safely terminate the simulation."

# 无限循环挂起主进程，等待 Ctrl+C 打断
while true; do
    sleep 1
done
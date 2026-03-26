#!/bin/bash

cleanup() {
    echo -e "\nCaught Ctrl+C! Shutting down all nodes..."
    pkill -f vcu_node
    pkill -f world_server
    echo "Simulation ended. Run python3 plot_fleet.py to view results."
    exit 0
}

trap cleanup SIGINT

pkill -f world_server
pkill -f vcu_node

echo "Starting world_server..."
./world_server &
sleep 0.1

echo "Starting 4 distributed VCU nodes..."
./vcu_node 1 &
./vcu_node 2 &
./vcu_node 3 &
./vcu_node 4 &

echo "All systems online. Simulation running in 3-phase mode..."
echo "Press [Ctrl+C] to safely terminate the simulation."

while true; do
    sleep 1
done
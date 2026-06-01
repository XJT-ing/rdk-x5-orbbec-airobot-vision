#!/usr/bin/env bash
set -e

echo "Prepare can0..."

sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

echo "Current can0 status:"
ip -details link show can0

echo "Start airbot_server on can0..."
sudo airbot_server -i can0 -p 50001

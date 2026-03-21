#!/bin/bash
# Antenna Tracker CAN Interface Setup Script
# Configures the can0 interface for ESP32 LoRa communication
set -e

echo "=== CAN Interface Setup ==="

if ! command -v ip &> /dev/null; then
    echo "iproute2 is required but not installed. Installing..."
    sudo apt-get update && sudo apt-get install -y iproute2 can-utils
fi

# Check if can0 exists
if ! ip link show can0 &> /dev/null; then
    echo "ERROR: can0 interface not found!"
    echo "Make sure the CAN hat/module is connected and overlay is enabled in /boot/firmware/config.txt"
    echo "Example for MCP2515: dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=25"
    exit 1
fi

# Bring down interface if it's already up to reconfigure
if ip link show can0 | grep -q "UP"; then
    sudo ip link set can0 down
fi

echo "Setting can0 bitrate to 500000 bps..."
sudo ip link set can0 type can bitrate 500000

echo "Bringing can0 up..."
sudo ip link set can0 up

# Verify
if ip link show can0 | grep -q "UP"; then
    echo "SUCCESS: can0 is UP and running at 500kbps"
    echo "You can test reception using: candump can0"
else
    echo "FAILED to bring up can0"
    exit 1
fi

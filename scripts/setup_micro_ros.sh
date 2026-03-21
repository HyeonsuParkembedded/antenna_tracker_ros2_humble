#!/bin/bash
# micro-ROS Agent and Workspace Setup Script
set -e

echo "=== micro-ROS Setup ==="

# Ensure ROS2 is sourced
source /opt/ros/humble/setup.bash

WORKSPACE_DIR="$HOME/microros_ws"

# Step 1: Create micro-ROS agent workspace
echo "--- Creating micro-ROS agent workspace ---"
mkdir -p "${WORKSPACE_DIR}/src"
cd "${WORKSPACE_DIR}"

# Step 2: Clone micro-ROS agent
echo "--- Cloning micro-ROS agent ---"
if [ ! -d "src/micro_ros_setup" ]; then
    git clone -b humble \
        https://github.com/micro-ROS/micro_ros_setup.git \
        src/micro_ros_setup
fi

# Step 3: Build micro-ROS setup tool
echo "--- Building micro-ROS setup ---"
rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install

source install/setup.bash

# Step 4: Create and build micro-ROS agent
echo "--- Building micro-ROS agent ---"
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh

echo ""
echo "=== micro-ROS Agent Setup Complete ==="
echo ""
echo "Add to ~/.bashrc:"
echo "  source ${WORKSPACE_DIR}/install/setup.bash"
echo ""
echo "Run micro-ROS agent (USB CDC):"
echo "  ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200"
echo ""
echo "For Zephyr firmware build, install west:"
echo "  pip3 install west"
echo "  cd antenna_tracker_ros/src/antenna_tracker_hardware/micro_ros_firmware"
echo "  west init -l ."
echo "  west update"
echo "  west build -b nucleo_h7a3zi_q"
echo "  west flash"

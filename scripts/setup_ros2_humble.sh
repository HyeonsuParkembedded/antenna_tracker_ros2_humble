#!/bin/bash
# ROS2 Humble Installation Script for Ubuntu 22.04 (RPi4B)
set -e

echo "=== ROS2 Humble Installation ==="

# Check Ubuntu version
. /etc/os-release
if [ "$VERSION_ID" != "22.04" ]; then
    echo "WARNING: This script is designed for Ubuntu 22.04"
    echo "Current version: $VERSION_ID"
fi

# Step 1: Set locale
echo "--- Setting locale ---"
sudo apt-get update && sudo apt-get install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Step 2: Add ROS2 repository
echo "--- Adding ROS2 repository ---"
sudo apt-get install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Step 3: Install ROS2 Humble
echo "--- Installing ROS2 Humble ---"
sudo apt-get update
sudo apt-get install -y \
    ros-humble-ros-base \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rosbridge-suite \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-xacro \
    ros-humble-rviz2 \
    ros-humble-rclcpp-action \
    python3-colcon-common-extensions \
    python3-rosdep

# Step 4: Initialize rosdep
echo "--- Initializing rosdep ---"
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

# Step 5: Add to bashrc
echo "--- Configuring shell ---"
SETUP_LINE="source /opt/ros/humble/setup.bash"
if ! grep -q "$SETUP_LINE" ~/.bashrc; then
    echo "$SETUP_LINE" >> ~/.bashrc
fi

# Step 6: Install Acados (Required for NMPC Controller)
echo "--- Installing Acados NMPC Solver ---"
sudo apt-get install -y cmake make gcc g++ python3-pip
if [ ! -d "/opt/acados" ]; then
    echo "Cloning and building Acados..."
    sudo mkdir -p /opt/acados
    sudo chown $USER:$USER /opt/acados
    git clone https://github.com/acados/acados.git /opt/acados
    cd /opt/acados
    git submodule update --recursive --init
    mkdir -p build && cd build
    cmake -DACADOS_WITH_QPOASES=ON -DACADOS_WITH_OSQP=ON ..
    make install -j$(nproc)
    
    # Install Python interface
    pip3 install -e /opt/acados/interfaces/acados_template
else
    echo "Acados already installed in /opt/acados"
fi

# Step 7: Set Acados environment variables
echo "--- Configuring shell for Acados ---"
ACADOS_SRC="export ACADOS_SOURCE_DIR=/opt/acados"
LD_LIB="export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/opt/acados/lib"

if ! grep -q "$ACADOS_SRC" ~/.bashrc; then
    echo "$ACADOS_SRC" >> ~/.bashrc
fi
if ! grep -q "/opt/acados/lib" ~/.bashrc; then
    echo "$LD_LIB" >> ~/.bashrc
fi

# Step 8: Set DDS middleware
DDS_LINE="export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
if ! grep -q "$DDS_LINE" ~/.bashrc; then
    echo "$DDS_LINE" >> ~/.bashrc
fi

echo ""
echo "=== ROS2 Humble Installation Complete ==="
echo "Run: source ~/.bashrc"
echo "Verify: ros2 topic list"

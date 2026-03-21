#!/bin/bash
# RPi4B PREEMPT_RT Kernel Installation Script
# Target: Ubuntu 22.04 (RPi4B)
set -e

echo "=== PREEMPT_RT Kernel Installation for RPi4B ==="
echo ""

# Check architecture
ARCH=$(uname -m)
if [ "$ARCH" != "aarch64" ]; then
    echo "WARNING: This script is designed for aarch64 (RPi4B)"
    echo "Current architecture: $ARCH"
fi

CURRENT_KERNEL=$(uname -r)
echo "Current kernel: $CURRENT_KERNEL"

# Step 1: Install build dependencies
echo ""
echo "--- Step 1: Installing build dependencies ---"
sudo apt-get update
sudo apt-get install -y \
    build-essential bc bison flex libssl-dev libncurses-dev \
    git wget kmod cpio

# Step 2: Get kernel source with PREEMPT_RT patch
KERNEL_VERSION="5.15"
RT_PATCH_VERSION="5.15.148-rt74"
echo ""
echo "--- Step 2: Downloading kernel ${KERNEL_VERSION} with RT patch ---"

cd /tmp
if [ ! -d "linux-${KERNEL_VERSION}" ]; then
    wget -q "https://cdn.kernel.org/pub/linux/kernel/v5.x/linux-${KERNEL_VERSION}.tar.xz"
    tar xf "linux-${KERNEL_VERSION}.tar.xz"
fi

cd "linux-${KERNEL_VERSION}"

# Download and apply RT patch
if [ ! -f ".rt_patched" ]; then
    wget -q "https://cdn.kernel.org/pub/linux/kernel/projects/rt/5.15/patch-${RT_PATCH_VERSION}.patch.xz"
    xzcat "patch-${RT_PATCH_VERSION}.patch.xz" | patch -p1
    touch .rt_patched
fi

# Step 3: Configure kernel
echo ""
echo "--- Step 3: Configuring kernel ---"
cp /boot/config-$(uname -r) .config 2>/dev/null || make bcm2711_defconfig

# Enable PREEMPT_RT
scripts/config --enable PREEMPT_RT
scripts/config --disable PREEMPT_VOLUNTARY
scripts/config --disable PREEMPT_NONE

# Enable high-resolution timers
scripts/config --enable HIGH_RES_TIMERS
scripts/config --enable NO_HZ_FULL

make olddefconfig

# Step 4: Build kernel
echo ""
echo "--- Step 4: Building kernel (this takes ~1-2 hours on RPi4B) ---"
NCPU=$(nproc)
make -j${NCPU} Image modules dtbs

# Step 5: Install
echo ""
echo "--- Step 5: Installing kernel ---"
sudo make modules_install
sudo cp arch/arm64/boot/Image /boot/vmlinuz-${KERNEL_VERSION}-rt
sudo cp arch/arm64/boot/dts/broadcom/*.dtb /boot/dtbs/

# Step 6: Update boot configuration
echo ""
echo "--- Step 6: Configuring boot ---"

# Update cmdline.txt for CPU isolation
CMDLINE_FILE="/boot/firmware/cmdline.txt"
if [ -f "$CMDLINE_FILE" ]; then
    sudo cp "$CMDLINE_FILE" "${CMDLINE_FILE}.bak"
    # Add isolcpus for real-time controller node (cores 2,3)
    if ! grep -q "isolcpus" "$CMDLINE_FILE"; then
        sudo sed -i 's/$/ isolcpus=2,3 nohz_full=2,3 rcu_nocbs=2,3/' "$CMDLINE_FILE"
    fi
fi

echo ""
echo "=== Installation Complete ==="
echo ""
echo "Next steps:"
echo "  1. sudo reboot"
echo "  2. Verify with: uname -a (should show PREEMPT_RT)"
echo "  3. Test latency with: sudo cyclictest -l100000 -m -Sp90 -i200 -h400 -q"
echo ""
echo "CPU Isolation configured:"
echo "  - Cores 0,1: System processes, ROS2 nodes"
echo "  - Cores 2,3: Real-time controller_node (use taskset -c 2,3)"
echo ""
echo "Run controller with RT priority:"
echo "  sudo chrt -f 80 taskset -c 2,3 ros2 run antenna_tracker_controller controller_node"

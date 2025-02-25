#!/bin/bash
set -e

# Set ROS 2 distribution as a variable
ROS_DISTRO="humble"

# Source ROS 2 setup
source /opt/ros/$ROS_DISTRO/setup.bash

# Install system dependencies for MongoDB and PCL
apt-get update && apt-get install -y \
    gnupg \
    curl \
    libpcap-dev


# Navigate to the workspace
cd /root/ros2_ws/src

# Navigate back to the workspace root
cd /root/ros2_ws

# Install ROS2 dependencies for all packages
echo "Installing ROS 2 dependencies..."
rosdep update
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y

# Now apply all the fixes after dependencies are installed


# Build the packages
echo "Building packages..."
# # First build without the problematic package
# colcon build --packages-skip demo_nova_sanctum
# source install/setup.bash

# # Then build the problematic package with warning suppression
# colcon build --packages-select demo_nova_sanctum--cmake-args -Wno-dev
# source install/setup.bash

# # Final build of everything
colcon build
source install/setup.bash

echo "Workspace setup completed!"
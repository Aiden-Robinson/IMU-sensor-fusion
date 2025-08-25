#!/bin/bash
# Install script for ROS2 IMU Package dependencies

echo "Installing ROS2 IMU Package dependencies..."

# Check if ROS2 is installed
if ! command -v ros2 &> /dev/null; then
    echo "Error: ROS2 is not installed or not in PATH"
    echo "Please install ROS2 first: https://docs.ros.org/en/humble/Installation.html"
    exit 1
fi

# Detect ROS2 distribution
ROS_DISTRO=$(ros2 --version | grep -oP 'ros2 \K\w+' || echo "humble")
echo "Detected ROS2 distribution: $ROS_DISTRO"

# Install required ROS2 packages
echo "Installing ROS2 packages..."
sudo apt update
sudo apt install -y \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-tf2-ros \
    ros-$ROS_DISTRO-tf2-tools \
    ros-$ROS_DISTRO-rviz-imu-plugin \
    ros-$ROS_DISTRO-sensor-msgs \
    ros-$ROS_DISTRO-geometry-msgs \
    ros-$ROS_DISTRO-std-msgs

# Install Python dependencies
echo "Installing Python dependencies..."
pip3 install pyserial

# Add user to dialout group for serial port access
echo "Adding user to dialout group for serial port access..."
sudo usermod -aG dialout $USER

echo "Installation complete!"
echo ""
echo "IMPORTANT: You need to log out and log back in for the dialout group changes to take effect."
echo ""
echo "To build and run the ROS2 package:"
echo "1. cd ~/ros2_ws"
echo "2. colcon build --packages-select ros2_imu_package"
echo "3. source install/setup.bash"
echo "4. ros2 launch ros2_imu_package imu_visualization.launch.py"

#!/bin/bash

# ROS2 Installation Script for Ubuntu
# This script installs ROS2 Humble (LTS) on Ubuntu 20.04/22.04

set -e  # Exit on any error

echo "=========================================="
echo "ROS2 Humble Installation Script"
echo "=========================================="
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check Ubuntu version
print_status "Checking Ubuntu version..."
UBUNTU_VERSION=$(lsb_release -rs)
UBUNTU_CODENAME=$(lsb_release -cs)

print_status "Detected Ubuntu $UBUNTU_VERSION ($UBUNTU_CODENAME)"

# Verify supported Ubuntu version
if [[ "$UBUNTU_VERSION" != "20.04" && "$UBUNTU_VERSION" != "22.04" && "$UBUNTU_VERSION" != "24.04" ]]; then
    print_warning "This script is tested on Ubuntu 20.04, 22.04, and 24.04"
    print_warning "Your version: $UBUNTU_VERSION"
    read -p "Continue anyway? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 0
    fi
fi

# Step 1: Set locale
print_status "Setting up locale..."
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Step 2: Setup sources
print_status "Setting up ROS2 repository..."

# Install curl if not already installed
sudo apt install -y curl gnupg lsb-release

# Add the ROS 2 GPG key
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add the repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Step 3: Update package index
print_status "Updating package index..."
sudo apt update

# Step 4: Install ROS2
print_status "Installing ROS2 Humble Desktop (this may take a while)..."
sudo apt install -y ros-humble-desktop

# Step 5: Install development tools
print_status "Installing ROS2 development tools..."
sudo apt install -y \
    ros-dev-tools \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool

# Step 6: Initialize rosdep
print_status "Initializing rosdep..."
sudo rosdep init || print_warning "rosdep already initialized"
rosdep update

# Step 7: Setup environment
print_status "Setting up ROS2 environment..."

# Add ROS2 sourcing to bashrc
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# ROS2 Humble" >> ~/.bashrc
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    print_success "Added ROS2 sourcing to ~/.bashrc"
else
    print_status "ROS2 sourcing already in ~/.bashrc"
fi

# Source ROS2 for current session
source /opt/ros/humble/setup.bash

# Step 8: Test installation
print_status "Testing ROS2 installation..."
if command -v ros2 &> /dev/null; then
    ROS_DISTRO=$(ros2 --version | grep -oP 'ros2 \K\w+' || echo "humble")
    print_success "ROS2 $ROS_DISTRO installed successfully!"
else
    print_error "ROS2 installation may have failed"
    exit 1
fi

# Step 9: Install additional packages commonly needed
print_status "Installing additional useful packages..."
sudo apt install -y \
    ros-humble-rviz2 \
    ros-humble-robot-state-publisher \
    ros-humble-tf2-ros \
    ros-humble-tf2-tools \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-std-msgs \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins

print_success "ROS2 installation completed!"
echo ""
echo "============================================="
echo "Installation Summary:"
echo "============================================="
echo "✓ ROS2 Humble Desktop installed"
echo "✓ Development tools installed"
echo "✓ Environment configured"
echo "✓ Additional packages installed"
echo ""
echo "Next steps:"
echo "1. Open a new terminal (or run: source ~/.bashrc)"
echo "2. Test ROS2: ros2 --version"
echo "3. Run the IMU setup: ./setup_ros2_imu.sh"
echo ""
echo "Quick test commands:"
echo "  ros2 --version"
echo "  ros2 pkg list"
echo "  rviz2 &"
echo ""
print_warning "Please open a new terminal or run 'source ~/.bashrc' to use ROS2 commands"

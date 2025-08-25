#!/bin/bash

# Docker entrypoint script for ROS2 IMU container

set -e

# Source ROS2 environment
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

# Set up X11 forwarding for GUI applications
export QT_X11_NO_MITSHM=1
export XDG_RUNTIME_DIR=/tmp/runtime-root
mkdir -p $XDG_RUNTIME_DIR
chmod 700 $XDG_RUNTIME_DIR

# Add dialout group for serial port access and add root user to it
groupadd -f dialout
usermod -a -G dialout root

# Set serial port permissions
if [ -e "${SERIAL_PORT:-/dev/ttyACM0}" ]; then
    chmod 666 "${SERIAL_PORT:-/dev/ttyACM0}"
    echo "Serial port ${SERIAL_PORT:-/dev/ttyACM0} configured"
else
    echo "Warning: Serial port ${SERIAL_PORT:-/dev/ttyACM0} not found"
    echo "Available serial ports:"
    ls /dev/tty* 2>/dev/null | grep -E "(ACM|USB)" || echo "No USB/ACM ports found"
fi

# Execute the command
exec "$@"

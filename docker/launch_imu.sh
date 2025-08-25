#!/bin/bash

# Default serial port (can be overridden with environment variable)
SERIAL_PORT=${SERIAL_PORT:-/dev/ttyACM0}

echo "=========================================="
echo "ROS2 IMU Docker Container"
echo "=========================================="
echo "Serial Port: $SERIAL_PORT"
echo "Starting IMU visualization..."
echo ""

# Check if serial port exists
if [ ! -e "$SERIAL_PORT" ]; then
    echo "WARNING: Serial port $SERIAL_PORT not found!"
    echo "Available serial ports:"
    ls /dev/tty* 2>/dev/null | grep -E "(ACM|USB)" || echo "No USB/ACM ports found"
    echo ""
    echo "You can specify a different port with:"
    echo "  docker run -e SERIAL_PORT=/dev/ttyUSB0 ..."
    echo ""
    echo "Continuing anyway (for demo mode)..."
fi

# Launch the IMU visualization
ros2 launch ros2_imu_package imu_visualization.launch.py serial_port:=$SERIAL_PORT

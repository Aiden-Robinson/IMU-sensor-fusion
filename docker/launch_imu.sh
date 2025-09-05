#!/bin/bash

# Launch ROS2 nodes

# Wait a moment for ROS2 to initialize
sleep 2

# Launch the IMU serial reader node and RViz
ros2 launch ros2_imu_package imu_visualization.launch.py serial_port:=$SERIAL_PORT

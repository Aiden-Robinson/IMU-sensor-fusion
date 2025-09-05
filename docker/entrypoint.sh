#!/bin/bash

# Source ROS2 setup
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

# Execute the command passed to docker run
exec "$@"

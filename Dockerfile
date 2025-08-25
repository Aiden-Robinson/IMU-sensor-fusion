# Use ROS2 Humble base image
FROM osrf/ros:humble-desktop-full

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Install additional dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-humble-rviz2 \
    ros-humble-robot-state-publisher \
    ros-humble-tf2-ros \
    ros-humble-tf2-tools \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-std-msgs \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    udev \
    && rm -rf /var/lib/apt/lists/*

# Try to install IMU plugin (may not be available)
RUN apt-get update && apt-get install -y ros-humble-rviz-imu-plugin || echo "IMU plugin not available"

# Install Python dependencies
RUN pip3 install pyserial

# Install ROS2 transforms package (replaces tf-transformations in ROS2)
RUN apt-get update && apt-get install -y \
    ros-humble-tf-transformations \
    python3-transforms3d \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /ros2_ws

# Copy the ROS2 package
COPY ros2_imu_package /ros2_ws/src/ros2_imu_package

# Build the workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# Setup entrypoint
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Create a script to launch the IMU visualization
COPY docker/launch_imu.sh /launch_imu.sh
RUN chmod +x /launch_imu.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/launch_imu.sh"]

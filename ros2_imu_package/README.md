# ROS2 IMU Package

This ROS2 package provides real-time visualization of MPU9250 IMU sensor data in RVIZ. It reads data from an Arduino via serial communication and publishes standard sensor_msgs/Imu messages.

## Features

- **Real-time IMU data streaming** from Arduino to ROS2
- **RVIZ visualization** with 3D orientation display
- **TF broadcasting** for coordinate frame visualization
- **Configurable serial port** and parameters
- **Standard ROS2 sensor_msgs/Imu** message format

## Package Structure

```
ros2_imu_package/
├── ros2_imu_package/
│   ├── __init__.py
│   ├── imu_serial_reader.py      # Main node for reading serial data
│   └── imu_tf_broadcaster.py     # TF broadcaster for visualization
├── launch/
│   └── imu_visualization.launch.py  # Launch file for complete setup
├── config/
│   ├── imu_params.yaml           # Configuration parameters
│   └── imu_visualization.rviz    # RVIZ configuration
├── scripts/
│   └── list_serial_ports.py      # Utility to find serial ports
├── package.xml
├── setup.py
└── README.md
```

## Prerequisites

### ROS2 Installation
Make sure you have ROS2 (Humble, Iron, or Rolling) installed:
```bash
# Source ROS2 environment
source /opt/ros/humble/setup.bash  # Replace 'humble' with your ROS2 distro
```

### Python Dependencies
```bash
pip install pyserial
```

### Additional ROS2 Packages
```bash
sudo apt install ros-humble-rviz2 ros-humble-robot-state-publisher ros-humble-tf2-ros ros-humble-rviz-imu-plugin
```

## Installation

1. **Create a ROS2 workspace** (if you don't have one):
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. **Copy this package** to your workspace:
```bash
cp -r /path/to/ros2_imu_package ~/ros2_ws/src/
```

3. **Build the workspace**:
```bash
cd ~/ros2_ws
colcon build --packages-select ros2_imu_package
source install/setup.bash
```

## Usage

### 1. Hardware Setup
- Connect your Arduino with MPU9250 IMU sensor
- Upload the Arduino sketch (`imu/imu.ino`) to your Arduino
- Note the serial port (usually `/dev/ttyACM0` or `/dev/ttyUSB0` on Linux)

### 2. Find Serial Port
```bash
python3 src/ros2_imu_package/scripts/list_serial_ports.py
```

### 3. Launch the Complete System
```bash
# Launch with default serial port (/dev/ttyACM0)
ros2 launch ros2_imu_package imu_visualization.launch.py

# Launch with custom serial port
ros2 launch ros2_imu_package imu_visualization.launch.py serial_port:=/dev/ttyUSB0

# Launch without RVIZ
ros2 launch ros2_imu_package imu_visualization.launch.py use_rviz:=false
```

### 4. Run Individual Nodes

**IMU Serial Reader** (publishes sensor_msgs/Imu):
```bash
ros2 run ros2_imu_package imu_serial_reader --ros-args -p serial_port:=/dev/ttyACM0
```

**TF Broadcaster** (for RVIZ visualization):
```bash
ros2 run ros2_imu_package imu_tf_broadcaster
```

**RVIZ Visualization**:
```bash
ros2 run rviz2 rviz2 -d src/ros2_imu_package/config/imu_visualization.rviz
```

## Topics and Frames

### Published Topics
- `/imu/data` (sensor_msgs/Imu) - IMU sensor data with orientation, angular velocity, and linear acceleration

### TF Frames
- `world` - Fixed world frame
- `base_link` - Robot base frame
- `imu_link` - IMU sensor frame (follows IMU orientation)

## Configuration

Edit `config/imu_params.yaml` to customize:
- Serial port and baud rate
- Frame IDs
- Publishing rate

## Troubleshooting

### Serial Port Issues
1. **Permission denied**: Add user to dialout group
   ```bash
   sudo usermod -aG dialout $USER
   # Log out and log back in
   ```

2. **Port not found**: Check available ports
   ```bash
   ls /dev/tty*
   dmesg | grep tty
   ```

3. **Arduino not recognized**: Check USB connection and drivers

### ROS2 Issues
1. **Package not found**: Make sure workspace is built and sourced
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select ros2_imu_package
   source install/setup.bash
   ```

2. **TF issues**: Check that all nodes are running
   ```bash
   ros2 node list
   ros2 topic list
   ros2 run tf2_tools view_frames
   ```

## Data Format

The package publishes standard `sensor_msgs/Imu` messages containing:
- **Header**: Timestamp and frame_id
- **Orientation**: Quaternion (from roll, pitch, yaw)
- **Angular Velocity**: rad/s (from gyroscope)
- **Linear Acceleration**: m/s² (from accelerometer)
- **Covariance matrices**: For uncertainty estimation

## RVIZ Visualization

The RVIZ configuration includes:
- **Grid**: Reference grid
- **RobotModel**: Simple 3D representation of the IMU
- **IMU Plugin**: Real-time orientation visualization
- **TF**: Coordinate frame display

## License

MIT License - See LICENSE file for details.

# MPU9250 IMU Sensor Project

A project for reading and visualizing data from the MPU9250 9-axis IMU sensor using Arduino and Python.

## Overview

This project demonstrates how to interface with the MPU9250 sensor to read:

- **3-axis Gyroscope** - Angular velocity measurements (°/s)
- **3-axis Accelerometer** - Linear acceleration measurements (m/s²)
- **3-axis Magnetometer** - Magnetic field measurements (μT)
- **Temperature** - Onboard temperature sensor (°C)

## Hardware Requirements

- Arduino (Nano, Uno, or compatible)
- MPU9250 IMU sensor module
- Jumper wires
- Breadboard (optional)

## Wiring

Connect the MPU9250 to your Arduino as follows:

| MPU9250 Pin | Arduino Pin |
| ----------- | ----------- |
| VCC         | 3.3V        |
| GND         | GND         |
| SDA         | A4 (SDA)    |
| SCL         | A5 (SCL)    |

## Dependencies

### Arduino Libraries

- **I2C Library** - Custom library for MPU9250 communication
  - Download from: [HiBit MPU9250 Repository](https://github.com/hibit-dev/mpu9250/raw/master/lib/I2C.zip)
  - Install via Arduino IDE: Sketch → Include Library → Add .ZIP Library

### Python Dependencies

Install the required Python packages:

```bash
pip install -r requirements.txt
```

## File Structure

```
├── imu/
│   └── imu.ino           # Arduino sketch for MPU9250 sensor reading
├── ros2_imu_package/     # ROS2 package for RVIZ visualization
│   ├── ros2_imu_package/
│   │   ├── imu_serial_reader.py    # ROS2 node for serial communication
│   │   └── imu_tf_broadcaster.py   # TF broadcaster for RVIZ
│   ├── launch/
│   │   └── imu_visualization.launch.py  # Launch file
│   ├── config/
│   │   ├── imu_params.yaml         # Configuration parameters
│   │   └── imu_visualization.rviz  # RVIZ setup
│   └── README.md         # ROS2 package documentation
├── test_serial.py        # Python script for testing serial communication
├── visualize_3d.py       # Python script for 3D visualization (legacy)
├── requirements.txt      # Python dependencies
└── README.md            # This file
```

## Usage

### 1. Arduino Setup

1. Install the I2C library in Arduino IDE
2. Upload `imu/imu.ino` to your Arduino
3. Open Serial Monitor (115200 baud) to view sensor readings
   - **Windows**: Serial port will look like `COM3`, `COM4`, etc.
   - **Linux**: Serial port will look like `/dev/ttyACM0` or `/dev/ttyUSB0` (use `ls /dev/tty*` to find it)

### 2. Expected Output

When the sensor is stationary, you should see readings similar to:

```
TEMP:    26.11°C
GYR (°/s):    0.183     -1.372     -0.274
ACC (m/s^2):  0.750     -0.564     -9.829
MAG (μT):     307.950   3417.000   -3081.750
```

### 3. Data Visualization

**Option A: Docker + ROS2 + RVIZ (🔥 Recommended - Works on Linux & Windows)**

**🚀 Ultra-Simple Setup:**
```bash
# Linux
./docker_setup.sh

# Windows
docker_setup.bat

# With custom serial port
./docker_setup.sh -p /dev/ttyUSB0    # Linux
docker_setup.bat -p COM4             # Windows
```

See [DOCKER_README.md](DOCKER_README.md) for complete Docker setup guide.

**Option B: Native ROS2 + RVIZ (Linux only)**

**🚀 One-Command Setup:**
```bash
# Complete automated setup and launch
./setup_ros2_imu.sh

# Or with custom serial port
./setup_ros2_imu.sh -p /dev/ttyUSB0

# Or just setup without launching
./setup_ros2_imu.sh --no-launch
```

**Manual Setup:**
See the [ROS2 package README](ros2_imu_package/README.md) for complete setup instructions.

```bash
# Build the ROS2 package
cd ros2_imu_package
colcon build
source install/setup.bash

# Launch RVIZ visualization
ros2 launch ros2_imu_package imu_visualization.launch.py serial_port:=/dev/ttyACM0
```

**Option C: Python Matplotlib (Legacy)**

Run the Python plotting script to visualize sensor data:

```bash
python visualize_3d.py
```

## Serial Port Notes (Linux vs Windows)

- **Windows**: Serial ports are named like `COM3`, `COM4`, etc.
- **Linux**: Serial ports are named like `/dev/ttyACM0` or `/dev/ttyUSB0`. Use `ls /dev/tty*` to list available ports. Make sure you have permission to access the port (you may need to add your user to the `dialout` group: `sudo usermod -aG dialout $USER` and then log out/in).

**For ROS2 users**: The ROS2 package includes a utility script to list available serial ports:
```bash
python3 ros2_imu_package/scripts/list_serial_ports.py
```

## Understanding the Data

- **Temperature**: Ambient temperature around the sensor
- **Gyroscope**: Small values (~±2°/s) when stationary are normal due to sensor drift
- **Accelerometer**: Should read approximately gravity (9.8 m/s²) on one axis when stationary
- **Magnetometer**: Detects Earth's magnetic field plus local magnetic interference

## Calibration Notes

- **Gyroscope**: May require offset calibration for precise applications
- **Accelerometer**: Generally well-calibrated out of the box
- **Magnetometer**: Sensitive to magnetic interference; calibrate away from metal objects and electronics

## References

- [Reading MPU9250 sensors with Arduino - HiBit](https://www.hibit.dev/posts/36/reading-mpu9250-sensors-with-arduino)
- [MPU9250 Datasheet](https://invensense.tdk.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf)

## License

This project is open source and available under the MIT License.

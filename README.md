# MPU9250 IMU Sensor Fusion

A portfolio project demonstrating real-time sensor fusion and 3D visualization using the MPU9250 IMU (gyroscope + accelerometer), Arduino, and ROS2. Magnetometer support has been removed for simplicity and reliability.

## Demo Video

https://github.com/user-attachments/assets/320ea084-6b7b-444e-82cf-7f2acc84bc24

## What It Does

- **Reads** gyroscope, accelerometer, and temperature data from the MPU9250 via Arduino.
- **Fuses** sensor data using a custom Kalman filter for accurate roll, pitch, and yaw estimation.
- **Streams** orientation data to ROS2 and visualizes it in RVIZ (3D).
- **Runs** fully containerized via Docker for easy setup and reproducibility.

## How It Works

1. **Arduino** runs [`imu/imu.ino`](imu/imu.ino), which reads raw IMU data and applies a Kalman filter for sensor fusion. It outputs filtered roll, pitch, yaw, and raw acceleration over serial.
2. **ROS2 Python Node** ([`ros2_imu_package/ros2_imu_package/imu_serial_reader.py`](ros2_imu_package/ros2_imu_package/imu_serial_reader.py)) reads serial data, converts it to ROS2 `sensor_msgs/Imu` messages, and publishes to `/imu/data`.
3. **TF Broadcaster** ([`ros2_imu_package/ros2_imu_package/imu_tf_broadcaster.py`](ros2_imu_package/ros2_imu_package/imu_tf_broadcaster.py)) broadcasts orientation for RVIZ visualization.
4. **Docker** ([`docker_setup.sh`](docker_setup.sh)) builds and runs the entire ROS2 stack, including RVIZ, in a container.

## Quick Start

1. **Flash Arduino:**

   - Connect your MPU9250 to Arduino (VCC-3.3V, GND-GND, SDA-A4, SCL-A5).
   - Upload [`imu/imu.ino`](imu/imu.ino) using Arduino IDE.
   - Confirm serial output at 115200 baud.

2. **Run Visualization (Linux/Windows with Docker):**

   - Plug in Arduino via USB.
   - Open a terminal in this folder.
   - Run:
     ```bash
     ./docker_setup.sh
     ```
   - For a custom serial port (e.g., `/dev/ttyUSB0` or `COM4`):
     ```bash
     ./docker_setup.sh -p /dev/ttyUSB0
     ```
   - RVIZ will launch automatically and show the IMU orientation in 3D.

3. **Native ROS2 (Linux only):**
   - See [`ros2_imu_package/README.md`](ros2_imu_package/README.md) for manual ROS2 setup and launch instructions.

## Kalman Filtering

The Arduino firmware implements a Kalman filter to combine noisy accelerometer and gyroscope data, producing stable roll and pitch estimates. Yaw is integrated from the gyroscope. This approach greatly reduces drift and noise, enabling reliable real-time orientation tracking.

## Example Serial Output

```
Roll,Pitch,Yaw,AccelX,AccelY,AccelZ
-2.13,0.87,15.42,0.012,-0.003,-0.998
```

## Project Structure

```
imu/imu.ino                # Arduino firmware (Kalman filter, serial output)
ros2_imu_package/          # ROS2 Python package (serial reader, TF broadcaster, RVIZ launch)
docker_setup.sh            # One-command Docker setup and launch
Dockerfile, docker-compose.yml, docker/  # Containerization scripts
requirements.txt           # Python dependencies
```

## Troubleshooting

- **Serial port not found:** Use [`ros2_imu_package/scripts/list_serial_ports.py`](ros2_imu_package/scripts/list_serial_ports.py) or `ls /dev/tty*` (Linux).
- **Permission denied:** Add your user to the `dialout` group (`sudo usermod -aG dialout $USER`), then log out/in.
- **RVIZ not displaying:** Make sure Arduino is streaming data and the correct serial port is set.

## References

- [MPU9250 Datasheet](https://invensense.tdk.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf)
- [HiBit MPU9250 Arduino Guide](https://www.hibit.dev/posts/36/reading-mpu9250-sensors-with-arduino)

## License

This project is open source and available under the MIT License.

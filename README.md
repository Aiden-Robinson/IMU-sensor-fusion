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
├── plot_imu.py           # Python script for data visualization
├── requirements.txt      # Python dependencies
└── README.md            # This file
```

## Usage

### 1. Arduino Setup

1. Install the I2C library in Arduino IDE
2. Upload `imu/imu.ino` to your Arduino
3. Open Serial Monitor (115200 baud) to view sensor readings

### 2. Expected Output

When the sensor is stationary, you should see readings similar to:

```
TEMP:    26.11°C
GYR (°/s):    0.183     -1.372     -0.274
ACC (m/s^2):  0.750     -0.564     -9.829
MAG (μT):     307.950   3417.000   -3081.750
```

### 3. Data Visualization (Optional)

Run the Python plotting script to visualize sensor data:

```bash
python plot_imu.py
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

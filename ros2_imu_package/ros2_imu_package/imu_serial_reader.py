#!/usr/bin/env python3
"""
ROS2 IMU Serial Reader Node
Reads IMU data from Arduino serial port and publishes sensor_msgs/Imu messages
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion
from std_msgs.msg import Header
import serial
import time
import threading
import queue
import math
from tf_transformations import quaternion_from_euler


class IMUSerialReader(Node):
    def __init__(self):
        super().__init__('imu_serial_reader')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 50.0)
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        
        # Initialize serial connection
        self.serial_conn = None
        self.data_queue = queue.Queue()
        self.running = False
        
        # IMU data storage
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.gyro_x = 0.0
        self.gyro_y = 0.0
        self.gyro_z = 0.0
        self.accel_x = 0.0
        self.accel_y = 0.0
        self.accel_z = 0.0
        self.temperature = 0.0
        
        # Create publisher
        self.imu_publisher = self.create_publisher(Imu, 'imu/data', 10)
        
        # Create timer for publishing
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_imu_data)
        
        # Start serial connection
        self.connect_serial()
        
        self.get_logger().info(f'IMU Serial Reader started on {self.serial_port}')
        
    def connect_serial(self):
        """Connect to Arduino serial port"""
        try:
            # Close existing connection if any
            if self.serial_conn:
                self.serial_conn.close()
                
            self.serial_conn = serial.Serial(
                port=self.serial_port,
                baudrate=self.baudrate,
                timeout=1,
                write_timeout=1
            )
            
            # Wait for Arduino to reset and initialize
            time.sleep(2)
            
            # Clear any stale data
            if self.serial_conn.in_waiting:
                self.serial_conn.reset_input_buffer()
                
            self.get_logger().info(f'Connected to {self.serial_port} at {self.baudrate} baud')
            
            # Start serial reading thread
            self.running = True
            self.serial_thread = threading.Thread(target=self.read_serial_data)
            self.serial_thread.daemon = True
            self.serial_thread.start()
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to connect to {self.serial_port}: {e}')
            return False
            
    def read_serial_data(self):
        """Read data from serial port in a separate thread"""
        while self.running:
            try:
                if self.serial_conn and self.serial_conn.is_open:
                    if self.serial_conn.in_waiting:
                        try:
                            line = self.serial_conn.readline().decode('utf-8', errors='replace').strip()
                            if line:  # Only process non-empty lines
                                self.parse_imu_data(line)
                        except UnicodeDecodeError:
                            # Handle corrupt data by flushing the buffer
                            self.serial_conn.reset_input_buffer()
                            continue
                    else:
                        time.sleep(0.001)  # Short sleep when no data
                else:
                    # Try to reconnect if connection is lost
                    self.get_logger().warn('Serial connection lost, attempting to reconnect...')
                    self.connect_serial()
                    time.sleep(1)  # Wait before retry
                    
            except Exception as e:
                self.get_logger().error(f'Serial read error: {e}')
                # Try to recover from errors
                try:
                    self.serial_conn.reset_input_buffer()
                except:
                    pass
                time.sleep(0.1)
            
    def parse_imu_data(self, line):
        """Parse IMU data from Arduino output"""
        # Skip header lines and debug info
        if not line or line.startswith("Kalman") or line.startswith("Format") or line.startswith("DEBUG"):
            return
            
        # Parse comma-separated values
        try:
            parts = line.strip().split(',')
            if len(parts) != 6:
                if line.strip():  # Only log for non-empty lines
                    self.get_logger().warn(f'Invalid data format: expected 6 values, got {len(parts)} from line "{line}"')
                return

            # Parse IMU data
            self.roll = math.radians(float(parts[0]))   # Convert to radians
            self.pitch = math.radians(float(parts[1]))  # Convert to radians
            self.yaw = math.radians(float(parts[2]))    # Convert to radians

            # Raw accelerometer data
            raw_ax = float(parts[3])
            raw_ay = float(parts[4])
            raw_az = float(parts[5])
            
            # Convert accelerometer data to m/s^2
            self.accel_x = raw_ax * 9.81
            self.accel_y = raw_ay * 9.81
            self.accel_z = raw_az * 9.81
            
            # Log data periodically
            if not hasattr(self, '_msg_count'):
                self._msg_count = 0
            self._msg_count += 1
            
            if self._msg_count % 50 == 0:  # Log every 50 messages
                self.get_logger().info(
                    f'Raw IMU Data - Roll: {float(parts[0]):.2f}°, '
                    f'Pitch: {float(parts[1]):.2f}°, '
                    f'Yaw: {float(parts[2]):.2f}°, '
                    f'Accel: ({raw_ax:.3f}, {raw_ay:.3f}, {raw_az:.3f})g'
                )
                
        except ValueError as e:
            self.get_logger().error(f'Failed to parse values from line "{line}": {e}')
        except Exception as e:
            self.get_logger().warn(f'Data parsing error: {e}')
            
    def publish_imu_data(self):
        """Publish IMU data as sensor_msgs/Imu message"""
        try:
            # Create IMU message
            imu_msg = Imu()
            
            # Header
            imu_msg.header = Header()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = self.frame_id
            
            # Debug output of raw angles
            self.get_logger().info(
                f'Publishing - Roll: {math.degrees(self.roll):.2f}°, '
                f'Pitch: {math.degrees(self.pitch):.2f}°, '
                f'Yaw: {math.degrees(self.yaw):.2f}°'
            )
            
            # Convert roll, pitch, yaw to quaternion
            # Note: The order and signs might need adjustment based on your IMU orientation
            quaternion = quaternion_from_euler(
                self.roll,   # Roll (rotation around X)
                self.pitch,  # Pitch (rotation around Y)
                self.yaw     # Yaw (rotation around Z)
            )
            
            imu_msg.orientation.x = quaternion[0]
            imu_msg.orientation.y = quaternion[1]
            imu_msg.orientation.z = quaternion[2]
            imu_msg.orientation.w = quaternion[3]
            
            # Log quaternion for debugging
            self.get_logger().info(
                f'Quaternion - X: {quaternion[0]:.4f}, Y: {quaternion[1]:.4f}, '
                f'Z: {quaternion[2]:.4f}, W: {quaternion[3]:.4f}'
            )
            
            # Debug logging every 50 messages (once per second at 50Hz)
            if hasattr(self, '_msg_count'):
                self._msg_count += 1
            else:
                self._msg_count = 0
                
            if self._msg_count % 25 == 0:  # More frequent logging
                self.get_logger().info(
                    f'EULER INPUT - Roll: {math.degrees(self.roll):.2f}°, '
                    f'Pitch: {math.degrees(self.pitch):.2f}°, '
                    f'Yaw: {math.degrees(self.yaw):.2f}°'
                )
                self.get_logger().info(
                    f'QUATERNION OUTPUT - X: {quaternion[0]:.4f}, Y: {quaternion[1]:.4f}, '
                    f'Z: {quaternion[2]:.4f}, W: {quaternion[3]:.4f}'
                )
                
                # Check if euler angles are actually changing
                roll_changing = abs(math.degrees(self.roll)) > 0.5
                pitch_changing = abs(math.degrees(self.pitch)) > 0.5
                yaw_changing = abs(math.degrees(self.yaw)) > 180  # Yaw can be large
                self.get_logger().info(
                    f'MOVEMENT CHECK - Roll changing: {roll_changing}, '
                    f'Pitch changing: {pitch_changing}, Yaw changing: {yaw_changing}'
                )
            
            # Set orientation covariance (diagonal matrix)
            # Lower values = more confident in the measurement
            imu_msg.orientation_covariance = [
                0.01, 0.0, 0.0,
                0.0, 0.01, 0.0,
                0.0, 0.0, 0.01
            ]
            
            # Angular velocity (already in rad/s)
            imu_msg.angular_velocity.x = self.gyro_x
            imu_msg.angular_velocity.y = self.gyro_y
            imu_msg.angular_velocity.z = self.gyro_z
            
            # Set angular velocity covariance
            imu_msg.angular_velocity_covariance = [
                0.01, 0.0, 0.0,
                0.0, 0.01, 0.0,
                0.0, 0.0, 0.01
            ]
            
            # Linear acceleration (already in m/s^2)
            imu_msg.linear_acceleration.x = self.accel_x
            imu_msg.linear_acceleration.y = self.accel_y
            imu_msg.linear_acceleration.z = self.accel_z
            
            # Set linear acceleration covariance
            imu_msg.linear_acceleration_covariance = [
                0.01, 0.0, 0.0,
                0.0, 0.01, 0.0,
                0.0, 0.0, 0.01
            ]
            
            # Publish the message
            self.imu_publisher.publish(imu_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing IMU data: {e}')
            
    def destroy_node(self):
        """Clean up when node is destroyed"""
        self.running = False
        if self.serial_conn:
            self.serial_conn.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        imu_reader = IMUSerialReader()
        rclpy.spin(imu_reader)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if 'imu_reader' in locals():
            imu_reader.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

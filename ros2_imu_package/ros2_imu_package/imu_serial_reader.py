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
            self.serial_conn = serial.Serial(self.serial_port, self.baudrate, timeout=1)
            time.sleep(2)  # Wait for Arduino to reset
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
                if self.serial_conn and self.serial_conn.in_waiting:
                    line = self.serial_conn.readline().decode('utf-8', errors='replace').strip()
                    self.parse_imu_data(line)
            except Exception as e:
                self.get_logger().warn(f'Serial read error: {e}')
            time.sleep(0.01)  # Small delay to prevent excessive CPU usage
            
    def parse_imu_data(self, line):
        """Parse IMU data from Arduino output"""
        try:
            # Parse temperature data
            if line.startswith("TEMP:"):
                temp_str = line.split('\t')[0].replace("TEMP:", "").replace("°C", "").strip()
                self.temperature = float(temp_str)
                
            # Parse gyroscope data
            elif line.startswith("GYR"):
                parts = line.split('\t')
                if len(parts) >= 4:
                    self.gyro_x = math.radians(float(parts[1]))  # Convert to rad/s
                    self.gyro_y = math.radians(float(parts[2]))
                    self.gyro_z = math.radians(float(parts[3]))
                    
            # Parse accelerometer data
            elif line.startswith("ACC"):
                parts = line.split('\t')
                if len(parts) >= 4:
                    self.accel_x = float(parts[1])
                    self.accel_y = float(parts[2])
                    self.accel_z = float(parts[3])
                    
            # Parse orientation data
            elif line.startswith("ORIENTATION:"):
                parts = line.split('\t')
                if len(parts) >= 4:
                    self.roll = math.radians(float(parts[1]))   # Convert to radians
                    self.pitch = math.radians(float(parts[2]))
                    self.yaw = math.radians(float(parts[3]))
                    
        except (ValueError, IndexError) as e:
            # Silently ignore parsing errors to avoid spam
            pass
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
            
            # Convert roll, pitch, yaw to quaternion
            quaternion = quaternion_from_euler(self.roll, self.pitch, self.yaw)
            imu_msg.orientation.x = quaternion[0]
            imu_msg.orientation.y = quaternion[1]
            imu_msg.orientation.z = quaternion[2]
            imu_msg.orientation.w = quaternion[3]
            
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

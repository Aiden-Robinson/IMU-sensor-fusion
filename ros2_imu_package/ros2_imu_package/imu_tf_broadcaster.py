#!/usr/bin/env python3
"""
ROS2 IMU TF Broadcaster Node
Subscribes to IMU data and broadcasts TF transforms for visualization in RVIZ
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import tf_transformations


class IMUTFBroadcaster(Node):
    def __init__(self):
        super().__init__('imu_tf_broadcaster')
        
        # Declare parameters
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('imu_frame', 'imu_link')
        
        # Get parameters
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.imu_frame = self.get_parameter('imu_frame').get_parameter_value().string_value
        
        # Create TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribe to IMU data
        self.imu_subscription = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            10
        )
        
        self.get_logger().info('IMU TF Broadcaster started')
        
    def imu_callback(self, msg):
        """Callback for IMU data - broadcast TF transform"""
        try:
            # Create transform message
            transform = TransformStamped()
            
            # Header
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = self.base_frame
            transform.child_frame_id = self.imu_frame
            
            # Translation (IMU position relative to base_link)
            transform.transform.translation.x = 0.0
            transform.transform.translation.y = 0.0
            transform.transform.translation.z = 0.1  # 10cm above base
            
            # Rotation (from IMU orientation)
            transform.transform.rotation = msg.orientation
            
            # Broadcast the transform
            self.tf_broadcaster.sendTransform(transform)
            
        except Exception as e:
            self.get_logger().error(f'Error broadcasting TF: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        tf_broadcaster = IMUTFBroadcaster()
        rclpy.spin(tf_broadcaster)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if 'tf_broadcaster' in locals():
            tf_broadcaster.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

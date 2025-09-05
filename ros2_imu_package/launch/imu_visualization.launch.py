#!/usr/bin/env python3
"""
Launch file for MPU9250 IMU visualization in RVIZ
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    from launch.actions import TimerAction
    from launch.event_handlers import OnProcessStart
    from launch.events import matches_action
    
    # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for Arduino connection'
    )
    
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Baud rate for serial communication'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RVIZ for visualization'
    )
    
    # Get package directory
    pkg_dir = get_package_share_directory('ros2_imu_package')
    
    # IMU Serial Reader Node with respawn
    imu_reader_node = Node(
        package='ros2_imu_package',
        executable='imu_serial_reader',
        name='imu_serial_reader',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'frame_id': 'imu_link',
            'publish_rate': 50.0
        }],
        output='screen',
        respawn=True,
        respawn_delay=1
    )
    
    # Static TF publisher for base_link to world
    static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link']
    )
    
    # Robot State Publisher (for visualization)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': get_robot_description(),
            'publish_frequency': 50.0
        }]
    )
    
    # IMU TF Broadcaster Node
    imu_tf_broadcaster_node = Node(
        package='ros2_imu_package',
        executable='imu_tf_broadcaster',
        name='imu_tf_broadcaster',
        parameters=[{
            'base_frame': 'base_link',
            'imu_frame': 'imu_link'
        }],
        output='screen'
    )
    
    # RVIZ Node - start after delay to ensure other nodes are ready
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('ros2_imu_package'),
        'config',
        'imu_visualization.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen'
    )
    
    # Delay RViz start
    delayed_rviz = TimerAction(
        period=3.0,  # 3 second delay
        actions=[rviz_node]
    )
    
    # Return launch description with ordered node startup
    return LaunchDescription([
        serial_port_arg,
        baudrate_arg,
        use_rviz_arg,
        # Start static transforms first
        static_tf_publisher,
        # Then robot state publisher
        robot_state_publisher,
        # Start IMU nodes
        imu_reader_node,
        imu_tf_broadcaster_node,
        # Finally start RViz with delay
        delayed_rviz
    ])


def get_robot_description():
    """Generate a simple URDF for IMU visualization"""
    return '''<?xml version="1.0"?>
<robot name="imu_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.02"/>
      </geometry>
      <material name="blue">
        <color rgba="0.0 0.0 1.0 1.0"/>
      </material>
    </visual>
  </link>
  
  <link name="imu_link">
    <visual>
      <geometry>
        <box size="0.02 0.02 0.005"/>
      </geometry>
      <material name="red">
        <color rgba="1.0 0.0 0.0 1.0"/>
      </material>
    </visual>
  </link>
  
  <joint name="base_to_imu" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>
</robot>'''

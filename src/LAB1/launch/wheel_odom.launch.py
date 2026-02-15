#!/usr/bin/env python3
"""
FRA532 LAB1 - Phase 1: Wheel Odometry Launch File
Launches wheel odometry node and static TF publisher
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    # Wheel Odometry Node
    wheel_odom_node = Node(
        package='LAB1',
        executable='wheel_odom_node.py',
        name='wheel_odometry_node',
        output='screen',
        emulate_tty=True,
    )
    
    # Static TF Publisher (base_footprint -> base_link, imu_link, base_scan)
    static_tf_node = Node(
        package='LAB1',
        executable='static_tf_publisher.py',
        name='static_tf_publisher',
        output='screen',
    )
    
    return LaunchDescription([
        wheel_odom_node,
        static_tf_node,
    ])
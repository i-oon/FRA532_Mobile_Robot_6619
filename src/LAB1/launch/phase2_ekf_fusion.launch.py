#!/usr/bin/env python3
"""
FRA532 LAB1 - Phase 2: EKF Fusion Launch File
Runs both wheel odometry and EKF fusion for comparison
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    
    # Declare arguments for tuning
    process_noise_x_arg = DeclareLaunchArgument(
        'process_noise_x',
        default_value='0.001',
        description='Process noise for x position'
    )
    
    process_noise_y_arg = DeclareLaunchArgument(
        'process_noise_y',
        default_value='0.001',
        description='Process noise for y position'
    )
    
    process_noise_theta_arg = DeclareLaunchArgument(
        'process_noise_theta',
        default_value='0.001',
        description='Process noise for theta (heading)'
    )
    
    measurement_noise_theta_arg = DeclareLaunchArgument(
        'measurement_noise_theta',
        default_value='0.01',
        description='Measurement noise for IMU theta'
    )
    
    use_imu_update_arg = DeclareLaunchArgument(
        'use_imu_update',
        default_value='true',
        description='Enable/disable IMU orientation update'
    )
    
    # Wheel Odometry Node (Phase 1)
    wheel_odom_node = Node(
        package='LAB1',
        executable='wheel_odom_node.py',
        name='wheel_odometry_node',
        output='screen',
        emulate_tty=True,
    )
    
    # EKF Fusion Node (Phase 2)
    ekf_fusion_node = Node(
        package='LAB1',
        executable='ekf_fusion_node.py',
        name='ekf_fusion_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'process_noise_x': LaunchConfiguration('process_noise_x'),
            'process_noise_y': LaunchConfiguration('process_noise_y'),
            'process_noise_theta': LaunchConfiguration('process_noise_theta'),
            'measurement_noise_theta': LaunchConfiguration('measurement_noise_theta'),
            'use_imu_update': LaunchConfiguration('use_imu_update'),
        }]
    )
    
    # Static TF Publisher
    static_tf_node = Node(
        package='LAB1',
        executable='static_tf_publisher.py',
        name='static_tf_publisher',
        output='screen',
    )
    
    return LaunchDescription([
        process_noise_x_arg,
        process_noise_y_arg,
        process_noise_theta_arg,
        measurement_noise_theta_arg,
        use_imu_update_arg,
        wheel_odom_node,
        ekf_fusion_node,
        static_tf_node,
    ])
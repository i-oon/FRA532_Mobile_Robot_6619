#!/usr/bin/env python3
"""
FRA532 LAB1 - Phase 3: ICP Localization Launch
Runs wheel + EKF + ICP for comparison
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    
    # ICP parameters
    max_iterations_arg = DeclareLaunchArgument(
        'max_iterations',
        default_value='50',
        description='Maximum ICP iterations'
    )
    
    correspondence_threshold_arg = DeclareLaunchArgument(
        'correspondence_threshold',
        default_value='0.5',
        description='Max distance for point correspondences (meters)'
    )
    
    min_correspondences_arg = DeclareLaunchArgument(
        'min_correspondences',
        default_value='30',
        description='Minimum number of correspondences'
    )
    
    downsample_ratio_arg = DeclareLaunchArgument(
        'downsample_ratio',
        default_value='2',
        description='Downsample scan points (take every Nth point)'
    )
    
    # Wheel Odometry Node
    wheel_odom_node = Node(
        package='LAB1',
        executable='wheel_odom_node.py',
        name='wheel_odometry_node',
        output='screen',
        emulate_tty=True,
    )
    
    # EKF Fusion Node
    ekf_fusion_node = Node(
        package='LAB1',
        executable='ekf_fusion_node.py',
        name='ekf_fusion_node',
        output='screen',
        emulate_tty=True,
    )
    
    # ICP Localization Node
    icp_node = Node(
        package='LAB1',
        executable='icp_localization_node.py',
        name='icp_localization_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'max_iterations': LaunchConfiguration('max_iterations'),
            'correspondence_threshold': LaunchConfiguration('correspondence_threshold'),
            'min_correspondences': LaunchConfiguration('min_correspondences'),
            'downsample_ratio': LaunchConfiguration('downsample_ratio'),
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
        max_iterations_arg,
        correspondence_threshold_arg,
        min_correspondences_arg,
        downsample_ratio_arg,
        wheel_odom_node,
        ekf_fusion_node,
        icp_node,
        static_tf_node,
    ])
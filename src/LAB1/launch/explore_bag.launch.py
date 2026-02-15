#!/usr/bin/env python3
"""
Launch file for rosbag exploration with visualization
Usage: ros2 launch LAB1 explore_bag.launch.py
Then in another terminal: ros2 bag play <path_to_bag>
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('LAB1')
    
    # Declare arguments
    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value='results/plots',
        description='Directory to save exploration plots'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz'
    )
    
    bag_file_arg = DeclareLaunchArgument(
        'bag_file',
        default_value='',
        description='Path to rosbag file (optional, can play manually)'
    )
    
    # Static TF Publisher
    static_tf_node = Node(
        package='LAB1',
        executable='static_tf_publisher.py',
        name='static_tf_publisher',
        output='screen'
    )
    
    # Bag Explorer Node
    explorer_node = Node(
        package='LAB1',
        executable='explore_bag.py',
        name='bag_explorer',
        output='screen',
        parameters=[{
            'output_dir': LaunchConfiguration('output_dir')
        }]
    )
    
    # RViz (optional)
    rviz_config = os.path.join(pkg_dir, 'config', 'rviz', 'lab1.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=lambda context: context.launch_configurations['use_rviz'] == 'true',
        output='screen'
    )
    
    return LaunchDescription([
        output_dir_arg,
        use_rviz_arg,
        bag_file_arg,
        static_tf_node,
        explorer_node,
        # rviz_node,  # Uncomment when rviz config is ready
    ])
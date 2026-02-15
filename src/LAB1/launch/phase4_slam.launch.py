#!/usr/bin/env python3
"""
FRA532 LAB1 - Phase 4: SLAM All-in-One
Launches everything except rosbag playback
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    
    # SLAM config file path
    slam_config = os.path.join(
        os.path.expanduser('~'),
        'FRA532_Mobile_Robot_6619',
        'src',
        'LAB1',
        'config',
        'slam_toolbox_mapping.yaml'
    )
    
    # RViz config file path
    rviz_config = os.path.join(
        os.path.expanduser('~'),
        'FRA532_Mobile_Robot_6619',
        'src',
        'LAB1',
        'config',
        'rviz',
        'phase4_slam.rviz'
    )
    
    # Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (bag) time'
    )
    
    # 1. Wheel Odometry Node
    wheel_odom_node = Node(
        package='LAB1',
        executable='wheel_odom_node.py',
        name='wheel_odometry_node',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # 2. EKF Fusion Node
    ekf_fusion_node = Node(
        package='LAB1',
        executable='ekf_fusion_node.py',
        name='ekf_fusion_node',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # 3. ICP Localization Node
    icp_node = Node(
        package='LAB1',
        executable='icp_localization_node.py',
        name='icp_localization_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'keyframe_distance': 0.2},
            {'keyframe_rotation': 0.087},
            {'max_iterations': 50},
            {'local_map_size': 15}
        ]
    )
    
    # 4. SLAM Toolbox Node with REMAPPING
    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        remappings=[
            ('/odom', '/ekf_odom'),  # ✅ REMAP ODOMETRY TOPIC!
        ],
    )
    
    # 5. Static TF Publisher
    static_tf_node = Node(
        package='LAB1',
        executable='static_tf_publisher.py',
        name='static_tf_publisher',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # 6. Odometry to Path Converter (for RViz visualization)
    odom_to_path_node = Node(
        package='LAB1',
        executable='odom_to_path_multi.py',
        name='multi_odom_to_path',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # 7. RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        wheel_odom_node,
        ekf_fusion_node,
        icp_node,
        slam_node,
        static_tf_node,
        odom_to_path_node,
        rviz_node,
    ])
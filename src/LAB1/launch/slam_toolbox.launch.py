#!/usr/bin/env python3
"""
Phase 3: SLAM Toolbox Launch (Corrected)
Combines best practices from reference launch file
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # Get package share directory
    pkg_share = FindPackageShare('LAB1')
    
    # SLAM Toolbox config file - use sync mode config
    slam_params_file = PathJoinSubstitution([
        pkg_share, 'config', 'slam_toolbox_sync.yaml'
    ])
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time from rosbag'
    )
    
    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value='~/FRA532_Mobile_Robot_6619/src/LAB1/data/rosbags/fibo_floor3_seq00',
        description='Path to rosbag file'
    )
    
    rate_arg = DeclareLaunchArgument(
        'rate',
        default_value='2.0',
        description='Rosbag playback rate'
    )
    
    # Wheel Odometry Node
    wheel_odom_node = Node(
        package='LAB1',
        executable='wheel_odom_node.py',
        name='wheel_odometry_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'wheel_separation': 0.265,
            'wheel_radius': 0.0325,
        }]
    )
    
    # EKF Fusion Node
    ekf_fusion_node = Node(
        package='LAB1',
        executable='ekf_fusion_node.py',
        name='ekf_fusion_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'process_noise_v': 0.01,
            'process_noise_omega': 0.05,
            'measurement_noise_v': 0.05,
            'measurement_noise_omega': 0.1,
        }]
    )
    
    # ICP Localization (Part 2)
    icp_localization_node = Node(
        package='LAB1',
        executable='icp_localization_node.py',
        name='icp_localization_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_icp': True,
            'icp_weight': 0.3,
        }]
    )
    
    # Static TF: base_link -> base_scan (LiDAR frame)
    static_tf_base_scan = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_scan',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_scan']
    )
    
    # Static TF Publisher (other frames)
    static_tf_node = Node(
        package='LAB1',
        executable='static_tf_publisher.py',
        name='static_tf_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # SLAM Toolbox - Sync SLAM (more stable for offline bags)
    slam_toolbox_node = TimerAction(
        period=3.0,  # Wait for odometry to stabilize
        actions=[
            Node(
                package='slam_toolbox',
                executable='sync_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[
                    slam_params_file,
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}
                ],
                remappings=[
                    ('/scan', '/scan'),
                    ('/odom', '/wheel_odom'),  # Use wheel odometry
                ]
            )
        ]
    )
    
    # Plotter (delayed start)
    plotter_node = TimerAction(
        period=4.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'python3',
                    PathJoinSubstitution([pkg_share, '..', '..', '..', 'src', 'LAB1', 'scripts', 'plot_slam_comparison.py'])
                ],
                output='screen'
            )
        ]
    )
    
    # Rosbag Play (delayed start to let everything initialize)
    bag_play = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'bag', 'play',
                    LaunchConfiguration('bag_path'),
                    '--rate', LaunchConfiguration('rate'),
                    '--clock'
                ],
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        bag_path_arg,
        rate_arg,
        wheel_odom_node,
        ekf_fusion_node,
        icp_localization_node,
        static_tf_base_scan,
        static_tf_node,
        slam_toolbox_node,
        # plotter_node,  # Comment out if running plotter separately
        # bag_play,      # Comment out if playing bag separately
    ])
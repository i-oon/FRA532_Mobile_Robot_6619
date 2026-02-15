#!/usr/bin/env python3
"""
Static TF Publisher for Turtlebot3 Burger
Publishes static transforms between robot frames
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
import yaml
import os
from ament_index_python.packages import get_package_share_directory


class StaticTFPublisher(Node):
    def __init__(self):
        super().__init__('static_tf_publisher')
        
        # Load robot parameters
        self.load_parameters()
        
        # Create static transform broadcaster
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        
        # Publish all static transforms
        self.publish_static_transforms()
        
        self.get_logger().info('Static TF Publisher initialized')
        self.get_logger().info(f'Published transforms: base_footprint->base_link, base_link->imu_link, base_link->base_scan')
    
    def load_parameters(self):
        """Load robot parameters from YAML file"""
        try:
            pkg_dir = get_package_share_directory('LAB1')
            config_file = os.path.join(pkg_dir, 'config', 'robot_params.yaml')
            
            with open(config_file, 'r') as f:
                params = yaml.safe_load(f)
            
            self.frames = params['frames']
            self.sensors = params['sensors']
            
        except Exception as e:
            self.get_logger().warn(f'Could not load params file: {e}')
            self.get_logger().warn('Using default parameters')
            
            # Default values
            self.frames = {
                'odom': 'odom',
                'base_footprint': 'base_footprint',
                'base_link': 'base_link',
                'imu_link': 'imu_link',
                'laser_link': 'base_scan'
            }
            
            self.sensors = {
                'imu': {'x': 0.0, 'y': 0.0, 'z': 0.068, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
                'lidar': {'x': -0.032, 'y': 0.0, 'z': 0.172, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
            }
    
    def publish_static_transforms(self):
        """Publish all static transforms"""
        transforms = []
        
        # Transform 1: base_footprint -> base_link
        t1 = TransformStamped()
        t1.header.stamp = self.get_clock().now().to_msg()
        t1.header.frame_id = self.frames['base_footprint']
        t1.child_frame_id = self.frames['base_link']
        t1.transform.translation.x = 0.0
        t1.transform.translation.y = 0.0
        t1.transform.translation.z = 0.010  # 1cm above ground
        t1.transform.rotation.x = 0.0
        t1.transform.rotation.y = 0.0
        t1.transform.rotation.z = 0.0
        t1.transform.rotation.w = 1.0
        transforms.append(t1)
        
        # Transform 2: base_link -> imu_link
        imu_params = self.sensors['imu']
        t2 = TransformStamped()
        t2.header.stamp = self.get_clock().now().to_msg()
        t2.header.frame_id = self.frames['base_link']
        t2.child_frame_id = self.frames['imu_link']
        t2.transform.translation.x = imu_params['x']
        t2.transform.translation.y = imu_params['y']
        t2.transform.translation.z = imu_params['z']
        t2.transform.rotation.x = 0.0
        t2.transform.rotation.y = 0.0
        t2.transform.rotation.z = 0.0
        t2.transform.rotation.w = 1.0
        transforms.append(t2)
        
        # Transform 3: base_link -> base_scan (LiDAR)
        lidar_params = self.sensors['lidar']
        t3 = TransformStamped()
        t3.header.stamp = self.get_clock().now().to_msg()
        t3.header.frame_id = self.frames['base_link']
        t3.child_frame_id = self.frames['laser_link']
        t3.transform.translation.x = lidar_params['x']
        t3.transform.translation.y = lidar_params['y']
        t3.transform.translation.z = lidar_params['z']
        t3.transform.rotation.x = 0.0
        t3.transform.rotation.y = 0.0
        t3.transform.rotation.z = 0.0
        t3.transform.rotation.w = 1.0
        transforms.append(t3)
        
        # Broadcast all transforms
        self.tf_broadcaster.sendTransform(transforms)


def main(args=None):
    rclpy.init(args=args)
    node = StaticTFPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
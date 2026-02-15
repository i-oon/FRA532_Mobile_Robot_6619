#!/usr/bin/env python3
"""
Odom to Path converter for RViz visualization
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class OdomToPath(Node):
    def __init__(self):
        super().__init__('odom_to_path')
        
        # Path message
        self.path = Path()
        self.path.header.frame_id = 'odom'
        
        # Subscriber
        self.odom_sub = self.create_subscription(
            Odometry,
            '/wheel_odom',
            self.odom_callback,
            10
        )
        
        # Publisher
        self.path_pub = self.create_publisher(Path, '/wheel_path', 10)
        
        self.get_logger().info('Odom to Path converter started')
    
    def odom_callback(self, msg):
        # Create pose stamped
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        
        # Add to path
        self.path.poses.append(pose)
        self.path.header.stamp = msg.header.stamp
        
        # Publish path
        self.path_pub.publish(self.path)


def main():
    rclpy.init()
    node = OdomToPath()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
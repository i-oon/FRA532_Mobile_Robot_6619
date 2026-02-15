#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class MultiOdomToPath(Node):
    def __init__(self):
        super().__init__('multi_odom_to_path')
        
        # Wheel path
        self.wheel_path = Path()
        self.wheel_path.header.frame_id = 'odom'
        self.wheel_sub = self.create_subscription(
            Odometry, '/wheel_odom', self.wheel_callback, 10)
        self.wheel_path_pub = self.create_publisher(Path, '/wheel_path', 10)
        
        # EKF path
        self.ekf_path = Path()
        self.ekf_path.header.frame_id = 'odom'
        self.ekf_sub = self.create_subscription(
            Odometry, '/ekf_odom', self.ekf_callback, 10)
        self.ekf_path_pub = self.create_publisher(Path, '/ekf_path', 10)
        
        # ICP path
        self.icp_path = Path()
        self.icp_path.header.frame_id = 'odom'
        self.icp_sub = self.create_subscription(
            Odometry, '/icp_odom', self.icp_callback, 10)
        self.icp_path_pub = self.create_publisher(Path, '/icp_path', 10)
        
        # SLAM path (from /pose)
        from geometry_msgs.msg import PoseWithCovarianceStamped
        self.slam_path = Path()
        self.slam_path.header.frame_id = 'map'  # SLAM in map frame!
        self.slam_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/pose', self.slam_callback, 10)
        self.slam_path_pub = self.create_publisher(Path, '/slam_path', 10)
        
        self.get_logger().info('Multi Odometry to Path converter started')
    
    def wheel_callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.wheel_path.poses.append(pose)
        self.wheel_path.header.stamp = msg.header.stamp
        self.wheel_path_pub.publish(self.wheel_path)
    
    def ekf_callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.ekf_path.poses.append(pose)
        self.ekf_path.header.stamp = msg.header.stamp
        self.ekf_path_pub.publish(self.ekf_path)
    
    def icp_callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.icp_path.poses.append(pose)
        self.icp_path.header.stamp = msg.header.stamp
        self.icp_path_pub.publish(self.icp_path)
    
    def slam_callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.slam_path.poses.append(pose)
        self.slam_path.header.stamp = msg.header.stamp
        self.slam_path_pub.publish(self.slam_path)


def main():
    rclpy.init()
    node = MultiOdomToPath()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
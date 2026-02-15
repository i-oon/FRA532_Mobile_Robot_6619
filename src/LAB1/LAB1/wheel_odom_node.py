#!/usr/bin/env python3
"""
FRA532 LAB1 - Phase 1: Wheel Odometry
Clean implementation using ICC (Instantaneous Center of Curvature) method
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
import math


class WheelOdometryNode(Node):
    def __init__(self):
        super().__init__('wheel_odometry_node')
        
        # Robot parameters (Turtlebot3 Burger)
        self.wheel_radius = 0.033  # meters
        self.track_width = 0.160   # meters (wheel separation)
        
        # State variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Velocities
        self.v = 0.0
        self.omega = 0.0
        
        # Previous wheel positions
        self.last_left_pos = None
        self.last_right_pos = None
        self.last_time = None
        
        # Message counter
        self.msg_count = 0
        
        # Subscriber
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )
        
        # Publisher
        self.odom_pub = self.create_publisher(Odometry, '/wheel_odom', 10)
        
        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.get_logger().info('='*60)
        self.get_logger().info('PHASE 1: Wheel Odometry Node Started')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Wheel radius: {self.wheel_radius} m')
        self.get_logger().info(f'Track width: {self.track_width} m')
        self.get_logger().info('Method: ICC (Instantaneous Center of Curvature)')
        self.get_logger().info('='*60)
    
    def joint_callback(self, msg):
        """Process joint states and update odometry"""
        
        # Extract wheel positions (radians)
        if len(msg.position) < 2:
            self.get_logger().warn('Not enough joint positions')
            return
        
        left_pos = msg.position[0]   # wheel_left_joint
        right_pos = msg.position[1]  # wheel_right_joint
        
        # Get timestamp
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # Initialize on first message
        if self.last_left_pos is None:
            self.last_left_pos = left_pos
            self.last_right_pos = right_pos
            self.last_time = current_time
            self.get_logger().info('Initialized wheel positions')
            return
        
        # Compute time step
        dt = current_time - self.last_time
        
        if dt <= 0 or dt > 1.0:
            self.get_logger().warn(f'Invalid dt: {dt:.3f}s')
            self.last_time = current_time
            return
        
        # Calculate wheel displacements (radians)
        delta_left = left_pos - self.last_left_pos
        delta_right = right_pos - self.last_right_pos
        
        # Convert to linear displacements (meters)
        d_left = delta_left * self.wheel_radius
        d_right = delta_right * self.wheel_radius
        
        # Calculate angular displacement
        d_theta = (d_right - d_left) / self.track_width
        
        # Update pose using ICC method (exact circular arc)
        if abs(d_right - d_left) < 1e-6:
            # Straight line motion
            d_center = (d_left + d_right) / 2.0
            self.x += d_center * math.cos(self.theta)
            self.y += d_center * math.sin(self.theta)
        else:
            # Circular arc motion
            R = (self.track_width / 2.0) * (d_left + d_right) / (d_right - d_left)
            
            # ICC coordinates
            icc_x = self.x - R * math.sin(self.theta)
            icc_y = self.y + R * math.cos(self.theta)
            
            # Rotation around ICC
            cos_dtheta = math.cos(d_theta)
            sin_dtheta = math.sin(d_theta)
            
            new_x = cos_dtheta * (self.x - icc_x) - sin_dtheta * (self.y - icc_y) + icc_x
            new_y = sin_dtheta * (self.x - icc_x) + cos_dtheta * (self.y - icc_y) + icc_y
            
            self.x = new_x
            self.y = new_y
        
        # Update heading
        self.theta += d_theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Calculate velocities
        self.v = (d_left + d_right) / (2.0 * dt)
        self.omega = (d_right - d_left) / (self.track_width * dt)
        
        # Publish odometry
        self.publish_odometry(msg.header.stamp)
        
        # Update previous values
        self.last_left_pos = left_pos
        self.last_right_pos = right_pos
        self.last_time = current_time
        
        # Log progress
        self.msg_count += 1
        if self.msg_count % 200 == 0:
            self.get_logger().info(
                f'[{self.msg_count:5d}] '
                f'x={self.x:6.2f}m, y={self.y:6.2f}m, θ={math.degrees(self.theta):6.1f}° | '
                f'v={self.v:.3f}m/s, ω={math.degrees(self.omega):5.1f}°/s'
            )
    
    def publish_odometry(self, stamp):
        """Publish odometry message and TF"""
        
        # Create odometry message
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Pose
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Orientation (quaternion)
        quat = self.euler_to_quaternion(self.theta)
        odom.pose.pose.orientation = quat
        
        # Twist
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.omega
        
        # Covariance
        odom.pose.covariance[0] = 0.001   # x
        odom.pose.covariance[7] = 0.001   # y
        odom.pose.covariance[35] = 0.01   # theta
        
        # Publish
        self.odom_pub.publish(odom)
        
        # Publish TF
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = quat
        
        self.tf_broadcaster.sendTransform(t)
    
    def euler_to_quaternion(self, yaw):
        """Convert yaw to quaternion"""
        q = Quaternion()
        q.w = math.cos(yaw / 2.0)
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        return q


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometryNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print()
        node.get_logger().info('='*60)
        node.get_logger().info('PHASE 1: Wheel Odometry Results')
        node.get_logger().info('='*60)
        node.get_logger().info(f'Messages processed: {node.msg_count}')
        node.get_logger().info(f'Final position: x={node.x:.3f}m, y={node.y:.3f}m')
        node.get_logger().info(f'Final heading: {math.degrees(node.theta):.1f}°')
        
        # Calculate path length
        if hasattr(node, 'x') and node.msg_count > 0:
            dist = math.sqrt(node.x**2 + node.y**2)
            node.get_logger().info(f'Distance from origin: {dist:.3f}m')
        
        node.get_logger().info('='*60)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
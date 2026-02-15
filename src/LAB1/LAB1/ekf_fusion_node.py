#!/usr/bin/env python3
"""
FRA532 LAB1 - Phase 2: EKF Sensor Fusion
Fuses wheel odometry (linear velocity) with IMU (angular velocity)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
import numpy as np
import math


class EKFFusionNode(Node):
    def __init__(self):
        super().__init__('ekf_fusion_node')
        
        # Declare tunable parameters
        self.declare_parameter('process_noise_x', 0.001)
        self.declare_parameter('process_noise_y', 0.001)
        self.declare_parameter('process_noise_theta', 0.001)
        self.declare_parameter('measurement_noise_theta', 0.01)
        self.declare_parameter('use_imu_update', True)
        
        # Get parameters
        q_x = self.get_parameter('process_noise_x').value
        q_y = self.get_parameter('process_noise_y').value
        q_theta = self.get_parameter('process_noise_theta').value
        r_theta = self.get_parameter('measurement_noise_theta').value
        self.use_imu_update = self.get_parameter('use_imu_update').value
        
        # Robot parameters
        self.wheel_radius = 0.033
        self.wheel_separation = 0.160
        
        # EKF State: [x, y, theta]
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Covariance matrix
        self.P = np.eye(3) * 0.1
        
        # Process noise (tunable)
        self.Q = np.diag([q_x, q_y, q_theta])
        
        # Measurement noise (IMU orientation)
        self.R = np.array([[r_theta]])
        
        # Current measurements
        self.v = 0.0          # Linear velocity from wheels
        self.omega_imu = 0.0  # Angular velocity from IMU
        
        # Previous wheel positions
        self.prev_left_pos = None
        self.prev_right_pos = None
        self.prev_time = None
        
        # IMU bias compensation
        self.imu_theta_offset = None
        
        # Statistics
        self.msg_count = 0
        
        # Subscribers
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )
        
        # Publisher
        self.odom_pub = self.create_publisher(Odometry, '/ekf_odom', 10)
        
        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.get_logger().info('='*60)
        self.get_logger().info('PHASE 2: EKF Sensor Fusion Node Started')
        self.get_logger().info('='*60)
        self.get_logger().info('Fusing: Wheel Odom (v) + IMU (ω, θ)')
        self.get_logger().info(f'Process Noise Q: diag([{q_x}, {q_y}, {q_theta}])')
        self.get_logger().info(f'Measurement Noise R: {r_theta}')
        self.get_logger().info(f'IMU Update: {"Enabled" if self.use_imu_update else "Disabled"}')
        self.get_logger().info('='*60)
    
    def joint_callback(self, msg):
        """Get linear velocity from wheel encoders"""
        
        if len(msg.position) < 2:
            return
        
        left_pos = msg.position[0]
        right_pos = msg.position[1]
        
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # Initialize
        if self.prev_left_pos is None:
            self.prev_left_pos = left_pos
            self.prev_right_pos = right_pos
            self.prev_time = current_time
            self.get_logger().info('Initialized wheel positions')
            return
        
        # Compute dt
        dt = current_time - self.prev_time
        
        if dt <= 0 or dt > 1.0:
            self.prev_time = current_time
            return
        
        # Calculate displacements
        d_left = (left_pos - self.prev_left_pos) * self.wheel_radius
        d_right = (right_pos - self.prev_right_pos) * self.wheel_radius
        
        # Linear velocity (from wheels)
        d_center = (d_left + d_right) / 2.0
        self.v = d_center / dt
        
        # Update previous
        self.prev_left_pos = left_pos
        self.prev_right_pos = right_pos
        self.prev_time = current_time
        
        # EKF Prediction (using IMU omega!)
        self.predict(self.v, self.omega_imu, dt)
        
        # Publish
        self.publish_odometry(msg.header.stamp)
        
        # Statistics
        self.msg_count += 1
        if self.msg_count % 200 == 0:
            self.get_logger().info(
                f'[{self.msg_count:5d}] '
                f'x={self.x:6.2f}m, y={self.y:6.2f}m, θ={math.degrees(self.theta):6.1f}° | '
                f'v={self.v:.3f}m/s, ω={math.degrees(self.omega_imu):5.1f}°/s'
            )
    
    def imu_callback(self, msg):
        """Get angular velocity and orientation from IMU"""
        
        # Angular velocity (for prediction)
        self.omega_imu = msg.angular_velocity.z
        
        # Orientation (for correction)
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w
        
        # Convert quaternion to yaw
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        theta_imu_raw = math.atan2(siny_cosp, cosy_cosp)
        
        # Initialize offset
        if self.imu_theta_offset is None:
            self.imu_theta_offset = theta_imu_raw
            self.get_logger().info(f'IMU offset initialized: {math.degrees(theta_imu_raw):.1f}°')
            return
        
        # Apply offset
        theta_imu = theta_imu_raw - self.imu_theta_offset
        theta_imu = math.atan2(math.sin(theta_imu), math.cos(theta_imu))
        
        # EKF Update (correct heading using IMU)
        self.update_theta(theta_imu)
    
    def predict(self, v, omega, dt):
        """EKF Prediction step"""
        
        # State prediction using IMU omega
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += omega * dt  # Using IMU omega, not wheel-based!
        
        # Normalize theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Jacobian matrix
        F = np.array([
            [1, 0, -v * math.sin(self.theta) * dt],
            [0, 1,  v * math.cos(self.theta) * dt],
            [0, 0,  1]
        ])
        
        # Covariance prediction
        self.P = F @ self.P @ F.T + self.Q
    
    def update_theta(self, theta_measured):
        """EKF Update step - correct theta using IMU orientation"""
        
        # Skip update if disabled
        if not self.use_imu_update:
            return
        
        # Measurement matrix (measuring theta directly)
        H = np.array([[0, 0, 1]])
        
        # Innovation (angle difference)
        y = theta_measured - self.theta
        y = math.atan2(math.sin(y), math.cos(y))  # Wrap to [-pi, pi]
        
        # Innovation covariance
        S = H @ self.P @ H.T + self.R
        
        # Kalman gain
        K = self.P @ H.T / S
        
        # State update
        self.x += K[0, 0] * y
        self.y += K[1, 0] * y
        self.theta += K[2, 0] * y
        
        # Normalize theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Covariance update
        I = np.eye(3)
        self.P = (I - K @ H) @ self.P
    
    def publish_odometry(self, stamp):
        """Publish fused odometry"""
        
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        
        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Orientation
        quat = self.euler_to_quaternion(self.theta)
        odom.pose.pose.orientation = quat
        
        # Velocity
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.omega_imu
        
        # Covariance
        odom.pose.covariance[0] = self.P[0, 0]   # x
        odom.pose.covariance[7] = self.P[1, 1]   # y
        odom.pose.covariance[35] = self.P[2, 2]  # theta
        
        # Publish
        self.odom_pub.publish(odom)
        
        # TF
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
    node = EKFFusionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print()
        node.get_logger().info('='*60)
        node.get_logger().info('PHASE 2: EKF Fusion Results')
        node.get_logger().info('='*60)
        node.get_logger().info(f'Messages processed: {node.msg_count}')
        node.get_logger().info(f'Final position: x={node.x:.3f}m, y={node.y:.3f}m')
        node.get_logger().info(f'Final heading: {math.degrees(node.theta):.1f}°')
        node.get_logger().info('='*60)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import csv
import os
from pathlib import Path
import math


class Phase4DataRecorder(Node):
    def __init__(self):
        super().__init__('phase4_data_recorder')
        
        # Get results directory
        home = str(Path.home())
        self.results_dir = os.path.join(home, 'FRA532_Mobile_Robot_6619', 'results', 'phase4')
        os.makedirs(self.results_dir, exist_ok=True)
        
        # CSV file paths
        self.wheel_csv = os.path.join(self.results_dir, 'phase4_wheel_odom.csv')
        self.ekf_csv = os.path.join(self.results_dir, 'phase4_ekf_odom.csv')
        self.icp_csv = os.path.join(self.results_dir, 'phase4_icp_odom.csv')
        self.slam_csv = os.path.join(self.results_dir, 'phase4_slam_pose.csv')
        
        # Open CSV files
        self.wheel_file = open(self.wheel_csv, 'w', newline='')
        self.ekf_file = open(self.ekf_csv, 'w', newline='')
        self.icp_file = open(self.icp_csv, 'w', newline='')
        self.slam_file = open(self.slam_csv, 'w', newline='')
        
        # CSV writers
        fieldnames = ['timestamp', 'x', 'y', 'theta', 'qw', 'qz']
        self.wheel_writer = csv.DictWriter(self.wheel_file, fieldnames=fieldnames)
        self.ekf_writer = csv.DictWriter(self.ekf_file, fieldnames=fieldnames)
        self.icp_writer = csv.DictWriter(self.icp_file, fieldnames=fieldnames)
        self.slam_writer = csv.DictWriter(self.slam_file, fieldnames=fieldnames)
        
        # Write headers
        self.wheel_writer.writeheader()
        self.ekf_writer.writeheader()
        self.icp_writer.writeheader()
        self.slam_writer.writeheader()
        
        # Subscribers
        self.wheel_sub = self.create_subscription(
            Odometry, '/wheel_odom', self.wheel_callback, 10
        )
        
        self.ekf_sub = self.create_subscription(
            Odometry, '/ekf_odom', self.ekf_callback, 10
        )
        
        self.icp_sub = self.create_subscription(
            Odometry, '/icp_odom', self.icp_callback, 10
        )
        
        # ✅ SLAM Toolbox publishes to /pose (PoseWithCovarianceStamped)
        self.slam_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/pose', self.slam_callback, 10
        )
        
        self.wheel_count = 0
        self.ekf_count = 0
        self.icp_count = 0
        self.slam_count = 0
        
        self.get_logger().info('='*60)
        self.get_logger().info('Phase 4 Data Recorder Started')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Saving to: {self.results_dir}')
        self.get_logger().info('Recording: Wheel + EKF + ICP + SLAM')
        self.get_logger().info('Press Ctrl+C to stop recording')
        self.get_logger().info('='*60)
    
    def wheel_callback(self, msg):
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        self.wheel_writer.writerow({
            'timestamp': timestamp,
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'theta': self.quat_to_yaw(msg.pose.pose.orientation),
            'qw': msg.pose.pose.orientation.w,
            'qz': msg.pose.pose.orientation.z
        })
        self.wheel_file.flush()
        
        self.wheel_count += 1
        if self.wheel_count % 200 == 0:
            self.get_logger().info(
                f'Recorded - Wheel: {self.wheel_count}, EKF: {self.ekf_count}, '
                f'ICP: {self.icp_count}, SLAM: {self.slam_count}'
            )
    
    def ekf_callback(self, msg):
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        self.ekf_writer.writerow({
            'timestamp': timestamp,
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'theta': self.quat_to_yaw(msg.pose.pose.orientation),
            'qw': msg.pose.pose.orientation.w,
            'qz': msg.pose.pose.orientation.z
        })
        self.ekf_file.flush()
        
        self.ekf_count += 1
    
    def icp_callback(self, msg):
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        self.icp_writer.writerow({
            'timestamp': timestamp,
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'theta': self.quat_to_yaw(msg.pose.pose.orientation),
            'qw': msg.pose.pose.orientation.w,
            'qz': msg.pose.pose.orientation.z
        })
        self.icp_file.flush()
        
        self.icp_count += 1
    
    def slam_callback(self, msg):
        # ✅ SLAM publishes PoseWithCovarianceStamped to /pose
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        self.slam_writer.writerow({
            'timestamp': timestamp,
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'theta': self.quat_to_yaw(msg.pose.pose.orientation),
            'qw': msg.pose.pose.orientation.w,
            'qz': msg.pose.pose.orientation.z
        })
        self.slam_file.flush()
        
        self.slam_count += 1
    
    def quat_to_yaw(self, quat):
        """Convert quaternion to yaw angle (CORRECTED formula)"""
        qw = quat.w
        qz = quat.z
        return math.atan2(2.0 * (qw * qz), 1.0 - 2.0 * (qz * qz))
    
    def cleanup(self):
        self.wheel_file.close()
        self.ekf_file.close()
        self.icp_file.close()
        self.slam_file.close()
        
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('Recording Complete!')
        self.get_logger().info(f'Wheel odom: {self.wheel_count}')
        self.get_logger().info(f'EKF odom: {self.ekf_count}')
        self.get_logger().info(f'ICP odom: {self.icp_count}')
        self.get_logger().info(f'SLAM pose: {self.slam_count}')
        self.get_logger().info(f'Files saved to: {self.results_dir}')
        self.get_logger().info('='*60)


def main():
    rclpy.init()
    recorder = Phase4DataRecorder()
    
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        print('\nStopping recording...')
    finally:
        recorder.cleanup()
        recorder.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
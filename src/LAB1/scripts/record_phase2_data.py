#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import csv
import os
from pathlib import Path
import math


class Phase2DataRecorder(Node):
    def __init__(self):
        super().__init__('phase2_data_recorder')
        
        # Get results directory
        home = str(Path.home())
        self.results_dir = os.path.join(home, 'FRA532_Mobile_Robot_6619/results', 'phase2')
        os.makedirs(self.results_dir, exist_ok=True)
        
        # CSV file paths
        self.wheel_csv = os.path.join(self.results_dir, 'phase2_wheel_odom.csv')
        self.ekf_csv = os.path.join(self.results_dir, 'phase2_ekf_odom.csv')
        
        # Open CSV files
        self.wheel_file = open(self.wheel_csv, 'w', newline='')
        self.ekf_file = open(self.ekf_csv, 'w', newline='')
        
        # CSV writers
        fieldnames = ['timestamp', 'x', 'y', 'theta', 'qw', 'qz']
        self.wheel_writer = csv.DictWriter(self.wheel_file, fieldnames=fieldnames)
        self.ekf_writer = csv.DictWriter(self.ekf_file, fieldnames=fieldnames)
        
        # Write headers
        self.wheel_writer.writeheader()
        self.ekf_writer.writeheader()
        
        # Subscribers
        self.wheel_sub = self.create_subscription(
            Odometry, '/wheel_odom', self.wheel_callback, 10
        )
        
        self.ekf_sub = self.create_subscription(
            Odometry, '/ekf_odom', self.ekf_callback, 10
        )
        
        self.wheel_count = 0
        self.ekf_count = 0
        
        self.get_logger().info('Phase 2 Data Recorder Started')
        self.get_logger().info(f'Saving to: {self.results_dir}')
        self.get_logger().info('Press Ctrl+C to stop recording')
    
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
        
        self.wheel_count += 1
        if self.wheel_count % 100 == 0:
            self.get_logger().info(f'Recorded {self.wheel_count} wheel odom msgs')
    
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
        
        self.ekf_count += 1
    
    def quat_to_yaw(self, quat):
        """Convert quaternion to yaw angle (CORRECTED formula)"""
        qw = quat.w
        qz = quat.z
        return math.atan2(2.0 * (qw * qz), 1.0 - 2.0 * (qz * qz))
    
    def cleanup(self):
        self.wheel_file.close()
        self.ekf_file.close()
        
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info(f'Recording Complete!')
        self.get_logger().info(f'Wheel odom messages: {self.wheel_count}')
        self.get_logger().info(f'EKF odom messages: {self.ekf_count}')
        self.get_logger().info(f'Files saved to: {self.results_dir}')
        self.get_logger().info('='*60)


def main():
    rclpy.init()
    recorder = Phase2DataRecorder()
    
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
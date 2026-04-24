#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import math
import signal
import sys


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
        
        # SLAM path
        self.slam_path = Path()
        self.slam_path.header.frame_id = 'map'
        self.slam_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/pose', self.slam_callback, 10)
        self.slam_path_pub = self.create_publisher(Path, '/slam_path', 10)
        
        self.get_logger().info('Multi Odometry to Path converter started')
        self.get_logger().info('Press Ctrl+C to see final analysis')
    
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
    
    def calculate_loop_error(self, path):
        """Calculate loop closure error"""
        if len(path.poses) == 0:
            return None, None, None
        
        final_pose = path.poses[-1].pose.position
        x = final_pose.x
        y = final_pose.y
        
        loop_error = math.sqrt(x**2 + y**2)
        
        path_length = 0.0
        for i in range(1, len(path.poses)):
            p1 = path.poses[i-1].pose.position
            p2 = path.poses[i].pose.position
            dx = p2.x - p1.x
            dy = p2.y - p1.y
            path_length += math.sqrt(dx**2 + dy**2)
        
        drift_pct = (loop_error / path_length * 100) if path_length > 0 else 0
        
        return loop_error, path_length, drift_pct
    
    def calculate_heading_error(self, path):
        """Calculate heading error"""
        if len(path.poses) < 2:
            return None
        
        q0 = path.poses[0].pose.orientation
        theta_0 = math.atan2(2.0 * (q0.w * q0.z), 1.0 - 2.0 * (q0.z * q0.z))
        
        qf = path.poses[-1].pose.orientation
        theta_f = math.atan2(2.0 * (qf.w * qf.z), 1.0 - 2.0 * (qf.z * qf.z))
        
        heading_error = theta_f - theta_0
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))
        
        return math.degrees(abs(heading_error))
    
    def print_final_analysis(self):
        """Print comprehensive final analysis"""
        
        print()  # Newline after ^C
        print('='*80)
        print('FINAL ANALYSIS - LOOP CLOSURE PERFORMANCE')
        print('='*80)
        
        results = {}
        
        for name, path in [('Wheel', self.wheel_path), 
                          ('EKF', self.ekf_path),
                          ('ICP', self.icp_path),
                          ('SLAM', self.slam_path)]:
            
            if len(path.poses) == 0:
                print(f'{name}: No data')
                continue
            
            loop_err, path_len, drift = self.calculate_loop_error(path)
            heading_err = self.calculate_heading_error(path)
            
            results[name] = {
                'poses': len(path.poses),
                'loop_error': loop_err,
                'path_length': path_len,
                'drift_pct': drift,
                'heading_error': heading_err,
                'final_x': path.poses[-1].pose.position.x,
                'final_y': path.poses[-1].pose.position.y
            }
        
        print()
        print('INDIVIDUAL RESULTS:')
        print('-'*80)
        
        for name, data in results.items():
            print(f'{name:10s} | Poses: {data["poses"]:5d} | '
                  f'Loop Error: {data["loop_error"]:6.3f}m | '
                  f'Path: {data["path_length"]:6.2f}m | '
                  f'Drift: {data["drift_pct"]:5.2f}%')
            print(f'{"":10s} | Final: ({data["final_x"]:6.2f}, {data["final_y"]:6.2f}) | '
                  f'Heading Error: {data["heading_error"]:5.1f}°')
        
        print()
        print('PERFORMANCE COMPARISON:')
        print('-'*80)
        print(f'{"Method":10s} | {"Loop Error":>12s} | {"Drift %":>10s} | {"Heading°":>10s}')
        print('-'*80)
        
        for name, data in sorted(results.items(), key=lambda x: x[1]['loop_error']):
            print(f'{name:10s} | {data["loop_error"]:10.3f}m | '
                  f'{data["drift_pct"]:8.2f}% | {data["heading_error"]:8.1f}°')
        
        if len(results) > 0:
            best = min(results.items(), key=lambda x: x[1]['loop_error'])
            worst = max(results.items(), key=lambda x: x[1]['loop_error'])
            
            print()
            print('SUMMARY:')
            print('-'*80)
            print(f'Best:  {best[0]:10s} - {best[1]["loop_error"]:.3f}m')
            print(f'Worst: {worst[0]:10s} - {worst[1]["loop_error"]:.3f}m')
            
            if 'Wheel' in results and len(results) > 1:
                print()
                print('IMPROVEMENT OVER BASELINE (Wheel):')
                print('-'*80)
                baseline = results['Wheel']['loop_error']
                
                for name, data in results.items():
                    if name == 'Wheel':
                        continue
                    improvement = (baseline - data['loop_error']) / baseline * 100
                    sign = '+' if improvement > 0 else ''
                    print(f'{name:10s}: {sign}{improvement:6.1f}% '
                          f'({baseline:.3f}m → {data["loop_error"]:.3f}m)')
        
        print('='*80)


def main():
    rclpy.init()
    node = MultiOdomToPath()
    
    # Signal handler
    def signal_handler(sig, frame):
        print('\nShutdown signal received...')
        node.print_final_analysis()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
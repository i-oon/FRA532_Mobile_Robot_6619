#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import numpy as np


class ComparisonPlotter(Node):
    def __init__(self):
        super().__init__('comparison_plotter')
        
        # Wheel odometry data
        self.wheel_x = []
        self.wheel_y = []
        self.wheel_theta = []
        self.wheel_time = []
        
        # EKF fusion data
        self.ekf_x = []
        self.ekf_y = []
        self.ekf_theta = []
        self.ekf_time = []
        
        self.start_time = None
        
        # Subscribers
        self.wheel_sub = self.create_subscription(
            Odometry, '/wheel_odom', self.wheel_callback, 10
        )
        
        self.ekf_sub = self.create_subscription(
            Odometry, '/ekf_odom', self.ekf_callback, 10
        )
        
        # Setup plot
        plt.ion()
        self.fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        self.axes = axes.flatten()
        
        self.get_logger().info('Comparison Plotter Started')
        self.get_logger().info('Blue = Wheel Odom | Red = EKF Fusion')
    
    def wheel_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        theta = 2.0 * np.arctan2(qz, qw)
        
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.start_time is None:
            self.start_time = t
        
        self.wheel_x.append(x)
        self.wheel_y.append(y)
        self.wheel_theta.append(np.rad2deg(theta))
        self.wheel_time.append(t - self.start_time)
        
        if len(self.wheel_x) % 50 == 0:
            self.update_plot()
    
    def ekf_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        theta = 2.0 * np.arctan2(qz, qw)
        
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.start_time is None:
            self.start_time = t
        
        self.ekf_x.append(x)
        self.ekf_y.append(y)
        self.ekf_theta.append(np.rad2deg(theta))
        self.ekf_time.append(t - self.start_time)
    
    def update_plot(self):
        for ax in self.axes:
            ax.clear()
        
        # Plot 1: XY Trajectories
        if self.wheel_x:
            self.axes[0].plot(self.wheel_x, self.wheel_y, 'b-', 
                            linewidth=2, alpha=0.7, label='Wheel Odom')
        if self.ekf_x:
            self.axes[0].plot(self.ekf_x, self.ekf_y, 'r-', 
                            linewidth=2, alpha=0.7, label='EKF Fusion')
        
        self.axes[0].plot(0, 0, 'go', markersize=10, label='Start', zorder=5)
        self.axes[0].set_xlabel('X (m)', fontsize=11)
        self.axes[0].set_ylabel('Y (m)', fontsize=11)
        self.axes[0].set_title('Trajectory Comparison', fontsize=12, fontweight='bold')
        self.axes[0].legend(fontsize=10)
        self.axes[0].grid(True, alpha=0.3)
        self.axes[0].axis('equal')
        
        # Plot 2: Heading Comparison
        if self.wheel_time:
            self.axes[1].plot(self.wheel_time, self.wheel_theta, 'b-',
                            linewidth=2, alpha=0.7, label='Wheel Odom')
        if self.ekf_time:
            self.axes[1].plot(self.ekf_time, self.ekf_theta, 'r-',
                            linewidth=2, alpha=0.7, label='EKF Fusion')
        
        self.axes[1].set_xlabel('Time (s)', fontsize=11)
        self.axes[1].set_ylabel('Heading (deg)', fontsize=11)
        self.axes[1].set_title('Heading Comparison', fontsize=12, fontweight='bold')
        self.axes[1].legend(fontsize=10)
        self.axes[1].grid(True, alpha=0.3)
        
        # Plot 3: Position Error
        if len(self.wheel_x) > 0 and len(self.ekf_x) > 0:
            min_len = min(len(self.wheel_x), len(self.ekf_x))
            if min_len > 10:
                pos_err = np.sqrt(
                    (np.array(self.wheel_x[:min_len]) - np.array(self.ekf_x[:min_len]))**2 +
                    (np.array(self.wheel_y[:min_len]) - np.array(self.ekf_y[:min_len]))**2
                )
                time_axis = self.wheel_time[:min_len]
                
                self.axes[2].plot(time_axis, pos_err, 'purple', linewidth=2)
                self.axes[2].set_xlabel('Time (s)', fontsize=11)
                self.axes[2].set_ylabel('Position Difference (m)', fontsize=11)
                self.axes[2].set_title('Wheel vs EKF Position Error', fontsize=12, fontweight='bold')
                self.axes[2].grid(True, alpha=0.3)
        
        # Plot 4: Heading Error
        if len(self.wheel_theta) > 0 and len(self.ekf_theta) > 0:
            min_len = min(len(self.wheel_theta), len(self.ekf_theta))
            if min_len > 10:
                heading_err = np.abs(
                    np.array(self.wheel_theta[:min_len]) - np.array(self.ekf_theta[:min_len])
                )
                time_axis = self.wheel_time[:min_len]
                
                self.axes[3].plot(time_axis, heading_err, 'orange', linewidth=2)
                self.axes[3].set_xlabel('Time (s)', fontsize=11)
                self.axes[3].set_ylabel('Heading Difference (deg)', fontsize=11)
                self.axes[3].set_title('Wheel vs EKF Heading Error', fontsize=12, fontweight='bold')
                self.axes[3].grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.pause(0.01)
    
    def save_plot(self):
        filename = f'phase2_comparison_{len(self.wheel_x)}pts.png'
        self.fig.savefig(filename, dpi=150, bbox_inches='tight')
        
        print('\n' + '='*60)
        print('PHASE 2: COMPARISON RESULTS')
        print('='*60)
        
        if self.wheel_x and self.ekf_x:
            print(f'\nWheel Odometry:')
            print(f'  End: ({self.wheel_x[-1]:.3f}, {self.wheel_y[-1]:.3f})')
            print(f'  Heading: {self.wheel_theta[-1]:.1f}°')
            wheel_err = np.sqrt(self.wheel_x[-1]**2 + self.wheel_y[-1]**2)
            print(f'  Loop error: {wheel_err:.3f}m')
            
            print(f'\nEKF Fusion:')
            print(f'  End: ({self.ekf_x[-1]:.3f}, {self.ekf_y[-1]:.3f})')
            print(f'  Heading: {self.ekf_theta[-1]:.1f}°')
            ekf_err = np.sqrt(self.ekf_x[-1]**2 + self.ekf_y[-1]**2)
            print(f'  Loop error: {ekf_err:.3f}m')
            
            print(f'\nImprovement:')
            improvement = ((wheel_err - ekf_err) / wheel_err) * 100
            print(f'  Loop closure: {improvement:+.1f}%')
            
            heading_diff = abs(self.wheel_theta[-1] - self.ekf_theta[-1])
            print(f'  Heading difference: {heading_diff:.1f}°')
        
        print(f'\nPlot saved: {filename}')
        print('='*60)


def main():
    rclpy.init()
    plotter = ComparisonPlotter()
    
    try:
        rclpy.spin(plotter)
    except KeyboardInterrupt:
        print('\nStopping...')
    finally:
        if len(plotter.wheel_x) > 0:
            plotter.update_plot()
            plotter.save_plot()
            plt.show(block=True)
        
        plotter.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
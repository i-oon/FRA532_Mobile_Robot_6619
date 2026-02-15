#!/usr/bin/env python3
"""
FRA532 LAB1 - Phase 1: Wheel Odometry Plotter
Real-time visualization
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import numpy as np


class WheelOdomPlotter(Node):
    def __init__(self):
        super().__init__('wheel_odom_plotter')
        
        # Data storage
        self.x = []
        self.y = []
        self.theta = []
        self.time = []
        
        self.start_time = None
        
        # Subscribe
        self.odom_sub = self.create_subscription(
            Odometry,
            '/wheel_odom',
            self.odom_callback,
            10
        )
        
        # Setup plot
        plt.ion()
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(14, 6))
        
        self.get_logger().info('Wheel Odometry Plotter Started')
        self.get_logger().info('Subscribing to: /wheel_odom')
    
    def odom_callback(self, msg):
        # Extract position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Extract heading from quaternion
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        theta = 2.0 * np.arctan2(qz, qw)
        
        # Timestamp
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.start_time is None:
            self.start_time = t
        
        # Store
        self.x.append(x)
        self.y.append(y)
        self.theta.append(np.rad2deg(theta))
        self.time.append(t - self.start_time)
        
        # Update plot
        if len(self.x) % 50 == 0:
            self.update_plot()
    
    def update_plot(self):
        # Clear axes
        self.ax1.clear()
        self.ax2.clear()
        
        # Plot 1: XY Trajectory
        self.ax1.plot(self.x, self.y, 'b-', linewidth=2, label='Wheel Odometry')
        self.ax1.plot(self.x[0], self.y[0], 'go', markersize=12, label='Start', zorder=5)
        self.ax1.plot(self.x[-1], self.y[-1], 'ro', markersize=12, label='Current', zorder=5)
        
        self.ax1.set_xlabel('X (m)', fontsize=12)
        self.ax1.set_ylabel('Y (m)', fontsize=12)
        self.ax1.set_title(f'Trajectory ({len(self.x)} points)', fontsize=14, fontweight='bold')
        self.ax1.grid(True, alpha=0.3)
        self.ax1.axis('equal')
        self.ax1.legend(fontsize=10)
        
        # Plot 2: Heading
        self.ax2.plot(self.time, self.theta, 'r-', linewidth=2)
        self.ax2.set_xlabel('Time (s)', fontsize=12)
        self.ax2.set_ylabel('Heading (deg)', fontsize=12)
        self.ax2.set_title('Heading Angle', fontsize=14, fontweight='bold')
        self.ax2.grid(True, alpha=0.3)
        
        # Add reference lines for turns
        for angle in [-360, -270, -180, -90, 0, 90, 180, 270, 360]:
            self.ax2.axhline(y=angle, color='gray', linestyle='--', alpha=0.2)
        
        plt.tight_layout()
        plt.pause(0.01)
    
    def save_plot(self):
        filename = f'phase1_wheel_odom_{len(self.x)}pts.png'
        self.fig.savefig(filename, dpi=150, bbox_inches='tight')
        
        print('\n' + '='*60)
        print('PHASE 1: WHEEL ODOMETRY RESULTS')
        print('='*60)
        print(f'Data points: {len(self.x)}')
        print(f'Duration: {self.time[-1]:.1f} seconds')
        print(f'Start: ({self.x[0]:.3f}, {self.y[0]:.3f})')
        print(f'End: ({self.x[-1]:.3f}, {self.y[-1]:.3f})')
        print(f'Final heading: {self.theta[-1]:.1f}°')
        
        # Path length
        path_len = sum(np.sqrt((self.x[i]-self.x[i-1])**2 + (self.y[i]-self.y[i-1])**2)
                       for i in range(1, len(self.x)))
        print(f'Path length: {path_len:.2f} m')
        
        # Displacement
        displacement = np.sqrt(self.x[-1]**2 + self.y[-1]**2)
        print(f'Displacement from origin: {displacement:.2f} m')
        
        print(f'\nPlot saved: {filename}')
        print('='*60)


def main():
    rclpy.init()
    plotter = WheelOdomPlotter()
    
    try:
        rclpy.spin(plotter)
    except KeyboardInterrupt:
        print('\nStopping plotter...')
    finally:
        if len(plotter.x) > 0:
            plotter.update_plot()
            plotter.save_plot()
            plt.show(block=True)
        
        plotter.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
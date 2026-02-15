#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import numpy as np


class ICPComparisonPlotter(Node):
    def __init__(self):
        super().__init__('icp_comparison_plotter')
        
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
        
        # ICP localization data
        self.icp_x = []
        self.icp_y = []
        self.icp_theta = []
        self.icp_time = []
        
        self.start_time = None
        
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
        
        # Setup plot - 2x3 layout
        plt.ion()
        self.fig, axes = plt.subplots(2, 3, figsize=(18, 10))
        self.axes = axes.flatten()
        
        self.get_logger().info('ICP Comparison Plotter Started')
        self.get_logger().info('Blue = Wheel | Red = EKF | Green = ICP')
    
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
    
    def icp_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        theta = 2.0 * np.arctan2(qz, qw)
        
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.start_time is None:
            self.start_time = t
        
        self.icp_x.append(x)
        self.icp_y.append(y)
        self.icp_theta.append(np.rad2deg(theta))
        self.icp_time.append(t - self.start_time)
    
    def update_plot(self):
        for ax in self.axes:
            ax.clear()
        
        # Plot 1: XY Trajectories - All 3 methods
        if self.wheel_x:
            self.axes[0].plot(self.wheel_x, self.wheel_y, 'b-', 
                            linewidth=2, alpha=0.6, label='Wheel')
        if self.ekf_x:
            self.axes[0].plot(self.ekf_x, self.ekf_y, 'r-', 
                            linewidth=2, alpha=0.6, label='EKF')
        if self.icp_x:
            self.axes[0].plot(self.icp_x, self.icp_y, 'g-', 
                            linewidth=2.5, alpha=0.8, label='ICP')
        
        self.axes[0].plot(0, 0, 'ko', markersize=12, label='Start', zorder=5)
        self.axes[0].set_xlabel('X (m)', fontsize=11)
        self.axes[0].set_ylabel('Y (m)', fontsize=11)
        self.axes[0].set_title('Trajectory Comparison (3 Methods)', fontsize=12, fontweight='bold')
        self.axes[0].legend(fontsize=10, loc='best')
        self.axes[0].grid(True, alpha=0.3)
        self.axes[0].axis('equal')
        
        # Plot 2: Heading Comparison
        if self.wheel_time:
            self.axes[1].plot(self.wheel_time, self.wheel_theta, 'b-',
                            linewidth=1.5, alpha=0.6, label='Wheel')
        if self.ekf_time:
            self.axes[1].plot(self.ekf_time, self.ekf_theta, 'r-',
                            linewidth=1.5, alpha=0.6, label='EKF')
        if self.icp_time:
            self.axes[1].plot(self.icp_time, self.icp_theta, 'g-',
                            linewidth=2, alpha=0.8, label='ICP')
        
        self.axes[1].set_xlabel('Time (s)', fontsize=11)
        self.axes[1].set_ylabel('Heading (deg)', fontsize=11)
        self.axes[1].set_title('Heading Comparison', fontsize=12, fontweight='bold')
        self.axes[1].legend(fontsize=10)
        self.axes[1].grid(True, alpha=0.3)
        
        # Plot 3: Loop Closure Error Over Time
        if self.wheel_time:
            wheel_loop_err = [np.sqrt(x**2 + y**2) for x, y in zip(self.wheel_x, self.wheel_y)]
            self.axes[2].plot(self.wheel_time, wheel_loop_err, 'b-',
                            linewidth=1.5, alpha=0.6, label='Wheel')
        if self.ekf_time:
            ekf_loop_err = [np.sqrt(x**2 + y**2) for x, y in zip(self.ekf_x, self.ekf_y)]
            self.axes[2].plot(self.ekf_time, ekf_loop_err, 'r-',
                            linewidth=1.5, alpha=0.6, label='EKF')
        if self.icp_time:
            icp_loop_err = [np.sqrt(x**2 + y**2) for x, y in zip(self.icp_x, self.icp_y)]
            self.axes[2].plot(self.icp_time, icp_loop_err, 'g-',
                            linewidth=2, alpha=0.8, label='ICP')
        
        self.axes[2].set_xlabel('Time (s)', fontsize=11)
        self.axes[2].set_ylabel('Distance from Start (m)', fontsize=11)
        self.axes[2].set_title('Loop Closure Error Evolution', fontsize=12, fontweight='bold')
        self.axes[2].legend(fontsize=10)
        self.axes[2].grid(True, alpha=0.3)
        
        # Plot 4: ICP vs Wheel Position Difference
        if len(self.wheel_x) > 0 and len(self.icp_x) > 0:
            min_len = min(len(self.wheel_x), len(self.icp_x))
            if min_len > 10:
                pos_diff = np.sqrt(
                    (np.array(self.wheel_x[:min_len]) - np.array(self.icp_x[:min_len]))**2 +
                    (np.array(self.wheel_y[:min_len]) - np.array(self.icp_y[:min_len]))**2
                )
                time_axis = self.wheel_time[:min_len]
                
                self.axes[3].plot(time_axis, pos_diff, 'purple', linewidth=2)
                self.axes[3].set_xlabel('Time (s)', fontsize=11)
                self.axes[3].set_ylabel('Position Difference (m)', fontsize=11)
                self.axes[3].set_title('ICP vs Wheel Position Divergence', fontsize=12, fontweight='bold')
                self.axes[3].grid(True, alpha=0.3)
        
        # Plot 5: ICP vs EKF Position Difference
        if len(self.ekf_x) > 0 and len(self.icp_x) > 0:
            min_len = min(len(self.ekf_x), len(self.icp_x))
            if min_len > 10:
                pos_diff = np.sqrt(
                    (np.array(self.ekf_x[:min_len]) - np.array(self.icp_x[:min_len]))**2 +
                    (np.array(self.ekf_y[:min_len]) - np.array(self.icp_y[:min_len]))**2
                )
                time_axis = self.ekf_time[:min_len]
                
                self.axes[4].plot(time_axis, pos_diff, 'orange', linewidth=2)
                self.axes[4].set_xlabel('Time (s)', fontsize=11)
                self.axes[4].set_ylabel('Position Difference (m)', fontsize=11)
                self.axes[4].set_title('ICP vs EKF Position Divergence', fontsize=12, fontweight='bold')
                self.axes[4].grid(True, alpha=0.3)
        
        # Plot 6: Heading Error Summary
        if len(self.wheel_theta) > 0 and len(self.icp_theta) > 0:
            min_len_wheel = min(len(self.wheel_theta), len(self.icp_theta))
            min_len_ekf = min(len(self.ekf_theta), len(self.icp_theta))
            
            if min_len_wheel > 10:
                heading_diff_wheel = np.abs(
                    np.array(self.wheel_theta[:min_len_wheel]) - np.array(self.icp_theta[:min_len_wheel])
                )
                self.axes[5].plot(self.wheel_time[:min_len_wheel], heading_diff_wheel, 
                                'b-', linewidth=1.5, alpha=0.6, label='ICP vs Wheel')
            
            if min_len_ekf > 10:
                heading_diff_ekf = np.abs(
                    np.array(self.ekf_theta[:min_len_ekf]) - np.array(self.icp_theta[:min_len_ekf])
                )
                self.axes[5].plot(self.ekf_time[:min_len_ekf], heading_diff_ekf,
                                'r-', linewidth=1.5, alpha=0.6, label='ICP vs EKF')
            
            self.axes[5].set_xlabel('Time (s)', fontsize=11)
            self.axes[5].set_ylabel('Heading Difference (deg)', fontsize=11)
            self.axes[5].set_title('Heading Error Comparison', fontsize=12, fontweight='bold')
            self.axes[5].legend(fontsize=10)
            self.axes[5].grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.pause(0.01)
    
    def save_plot(self):
        filename = f'phase3_comparison_{len(self.wheel_x)}pts.png'
        self.fig.savefig(filename, dpi=150, bbox_inches='tight')
        
        print('\n' + '='*70)
        print('PHASE 3: ICP COMPARISON RESULTS')
        print('='*70)
        
        if self.wheel_x and self.ekf_x and self.icp_x:
            # Wheel Odometry
            print(f'\nWheel Odometry:')
            print(f'  End: ({self.wheel_x[-1]:.3f}, {self.wheel_y[-1]:.3f})')
            print(f'  Heading: {self.wheel_theta[-1]:.1f}°')
            wheel_err = np.sqrt(self.wheel_x[-1]**2 + self.wheel_y[-1]**2)
            print(f'  Loop error: {wheel_err:.3f}m')
            
            # EKF Fusion
            print(f'\nEKF Fusion:')
            print(f'  End: ({self.ekf_x[-1]:.3f}, {self.ekf_y[-1]:.3f})')
            print(f'  Heading: {self.ekf_theta[-1]:.1f}°')
            ekf_err = np.sqrt(self.ekf_x[-1]**2 + self.ekf_y[-1]**2)
            print(f'  Loop error: {ekf_err:.3f}m')
            
            # ICP Localization
            print(f'\nICP Localization:')
            print(f'  End: ({self.icp_x[-1]:.3f}, {self.icp_y[-1]:.3f})')
            print(f'  Heading: {self.icp_theta[-1]:.1f}°')
            icp_err = np.sqrt(self.icp_x[-1]**2 + self.icp_y[-1]**2)
            print(f'  Loop error: {icp_err:.3f}m')
            
            # Improvements
            print(f'\nICP vs Wheel:')
            improvement_wheel = ((wheel_err - icp_err) / wheel_err) * 100
            print(f'  Loop closure: {improvement_wheel:+.1f}%')
            heading_diff_wheel = abs(self.wheel_theta[-1] - self.icp_theta[-1])
            print(f'  Heading difference: {heading_diff_wheel:.1f}°')
            
            print(f'\nICP vs EKF:')
            improvement_ekf = ((ekf_err - icp_err) / ekf_err) * 100
            print(f'  Loop closure: {improvement_ekf:+.1f}%')
            heading_diff_ekf = abs(self.ekf_theta[-1] - self.icp_theta[-1])
            print(f'  Heading difference: {heading_diff_ekf:.1f}°')
            
            # Best method
            print(f'\n{"WINNER":^70}')
            errors = {
                'Wheel': wheel_err,
                'EKF': ekf_err,
                'ICP': icp_err
            }
            best = min(errors, key=errors.get)
            print(f'  🏆 Best: {best} ({errors[best]:.3f}m)')
        
        print(f'\nPlot saved: {filename}')
        print('='*70)


def main():
    rclpy.init()
    plotter = ICPComparisonPlotter()
    
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
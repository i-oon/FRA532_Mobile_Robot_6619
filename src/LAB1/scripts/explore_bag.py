#!/usr/bin/env python3
"""
Rosbag Explorer for FRA532 LAB1
Analyzes sensor data from recorded bags and generates statistics
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu, JointState
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
import os


class BagExplorer(Node):
    def __init__(self):
        super().__init__('bag_explorer')
        
        # Declare parameter for output directory
        self.declare_parameter('output_dir', 'results/plots')
        self.output_dir = self.get_parameter('output_dir').value
        
        # Create output directory if not exists
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Statistics storage
        self.stats = {
            'scan': {
                'count': 0,
                'timestamps': [],
                'ranges_count': [],
                'angle_min': [],
                'angle_max': []
            },
            'imu': {
                'count': 0,
                'timestamps': [],
                'gyro_z': [],
                'accel_x': [],
                'accel_y': []
            },
            'joints': {
                'count': 0,
                'timestamps': [],
                'positions': [],
                'velocities': [],
                'names': None
            }
        }
        
        # QoS profiles to match rosbag
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        
        # Scan uses Best Effort (from bag info)
        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # IMU and Joints use Reliable (from bag info)
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, scan_qos)
        self.imu_sub = self.create_subscription(
            Imu, '/imu', self.imu_callback, reliable_qos)
        self.joints_sub = self.create_subscription(
            JointState, '/joint_states', self.joints_callback, reliable_qos)
        
        self.get_logger().info('='*60)
        self.get_logger().info('Bag Explorer Started')
        self.get_logger().info('Play your rosbag file now...')
        self.get_logger().info('Press Ctrl+C when done to see summary and plots')
        self.get_logger().info('='*60)
    
    def stamp_to_sec(self, stamp):
        """Convert ROS timestamp to seconds"""
        return stamp.sec + stamp.nanosec * 1e-9
    
    def scan_callback(self, msg):
        """Callback for LaserScan messages"""
        self.stats['scan']['count'] += 1
        t = self.stamp_to_sec(msg.header.stamp)
        self.stats['scan']['timestamps'].append(t)
        self.stats['scan']['ranges_count'].append(len(msg.ranges))
        self.stats['scan']['angle_min'].append(msg.angle_min)
        self.stats['scan']['angle_max'].append(msg.angle_max)
        
        if self.stats['scan']['count'] % 10 == 0:
            self.get_logger().info(f"[SCAN] Messages: {self.stats['scan']['count']}")
    
    def imu_callback(self, msg):
        """Callback for IMU messages"""
        self.stats['imu']['count'] += 1
        t = self.stamp_to_sec(msg.header.stamp)
        self.stats['imu']['timestamps'].append(t)
        self.stats['imu']['gyro_z'].append(msg.angular_velocity.z)
        self.stats['imu']['accel_x'].append(msg.linear_acceleration.x)
        self.stats['imu']['accel_y'].append(msg.linear_acceleration.y)
        
        if self.stats['imu']['count'] % 50 == 0:
            self.get_logger().info(f"[IMU] Messages: {self.stats['imu']['count']}")
    
    def joints_callback(self, msg):
        """Callback for JointState messages"""
        self.stats['joints']['count'] += 1
        t = self.stamp_to_sec(msg.header.stamp)
        self.stats['joints']['timestamps'].append(t)
        
        # Store joint names on first message
        if self.stats['joints']['names'] is None:
            self.stats['joints']['names'] = msg.name
            self.get_logger().info(f"Joint names: {msg.name}")
        
        self.stats['joints']['positions'].append(list(msg.position))
        self.stats['joints']['velocities'].append(list(msg.velocity))
        
        if self.stats['joints']['count'] % 50 == 0:
            self.get_logger().info(f"[JOINTS] Messages: {self.stats['joints']['count']}")
    
    def print_summary(self):
        """Print comprehensive statistics summary"""
        print("\n" + "="*70)
        print(" "*20 + "ROSBAG EXPLORATION SUMMARY")
        print("="*70 + "\n")
        
        for topic, data in self.stats.items():
            print(f"{topic.upper()} Topic:")
            print("-" * 50)
            print(f"  Total messages: {data['count']}")
            
            if len(data['timestamps']) > 1:
                timestamps = np.array(data['timestamps'])
                dt = np.diff(timestamps)
                
                print(f"  Duration: {timestamps[-1] - timestamps[0]:.2f} seconds")
                print(f"  Average frequency: {1.0/np.mean(dt):.2f} Hz")
                print(f"  Std frequency: {np.std(1.0/dt):.2f} Hz")
                print(f"  Min dt: {np.min(dt)*1000:.2f} ms")
                print(f"  Max dt: {np.max(dt)*1000:.2f} ms")
                
                # Topic-specific info
                if topic == 'scan':
                    print(f"  Scan points per message: {np.mean(data['ranges_count']):.0f}")
                    print(f"  Angle range: {np.rad2deg(data['angle_min'][0]):.1f}° to {np.rad2deg(data['angle_max'][0]):.1f}°")
                
                elif topic == 'imu':
                    gyro_z = np.array(data['gyro_z'])
                    print(f"  Gyro Z range: [{np.min(gyro_z):.3f}, {np.max(gyro_z):.3f}] rad/s")
                    print(f"  Gyro Z mean: {np.mean(gyro_z):.3f} rad/s (bias)")
                
                elif topic == 'joints':
                    if data['names']:
                        print(f"  Joint names: {', '.join(data['names'])}")
                    vels = np.array(data['velocities'])
                    if vels.shape[1] >= 2:
                        print(f"  Left wheel vel range: [{np.min(vels[:,0]):.2f}, {np.max(vels[:,0]):.2f}] rad/s")
                        print(f"  Right wheel vel range: [{np.min(vels[:,1]):.2f}, {np.max(vels[:,1]):.2f}] rad/s")
            
            print()
        
        print("="*70)
    
    def plot_statistics(self):
        """Generate comprehensive visualization plots"""
        fig = plt.figure(figsize=(16, 10))
        
        # Create grid: 3 rows x 2 columns
        gs = fig.add_gridspec(3, 2, hspace=0.3, wspace=0.3)
        
        # Plot 1: Message arrival timeline
        ax1 = fig.add_subplot(gs[0, :])
        colors = {'scan': 'red', 'imu': 'blue', 'joints': 'green'}
        
        for i, (topic, data) in enumerate(self.stats.items()):
            if data['timestamps']:
                t = np.array(data['timestamps'])
                t_rel = t - t[0]  # Relative time
                ax1.scatter(t_rel, [i]*len(t), s=1, alpha=0.5, 
                           label=f"{topic} ({data['count']} msgs)", 
                           color=colors[topic])
        
        ax1.set_yticks(range(len(self.stats)))
        ax1.set_yticklabels(list(self.stats.keys()))
        ax1.set_xlabel('Time (s)')
        ax1.set_title('Message Arrival Timeline', fontweight='bold')
        ax1.legend(loc='upper right')
        ax1.grid(True, alpha=0.3)
        
        # Plot 2: IMU Gyro Z
        ax2 = fig.add_subplot(gs[1, 0])
        if self.stats['imu']['timestamps']:
            t_imu = np.array(self.stats['imu']['timestamps'])
            t_imu = t_imu - t_imu[0]
            gyro_z = np.array(self.stats['imu']['gyro_z'])
            
            ax2.plot(t_imu, gyro_z, linewidth=0.5, color='blue', alpha=0.7)
            ax2.axhline(y=np.mean(gyro_z), color='r', linestyle='--', 
                       label=f'Mean: {np.mean(gyro_z):.4f} rad/s', linewidth=1)
            ax2.set_xlabel('Time (s)')
            ax2.set_ylabel('Angular Velocity (rad/s)')
            ax2.set_title('IMU Gyroscope Z-axis', fontweight='bold')
            ax2.legend()
            ax2.grid(True, alpha=0.3)
        
        # Plot 3: IMU Acceleration
        ax3 = fig.add_subplot(gs[1, 1])
        if self.stats['imu']['timestamps']:
            t_imu = np.array(self.stats['imu']['timestamps'])
            t_imu = t_imu - t_imu[0]
            accel_x = np.array(self.stats['imu']['accel_x'])
            accel_y = np.array(self.stats['imu']['accel_y'])
            
            ax3.plot(t_imu, accel_x, linewidth=0.5, label='Accel X', alpha=0.7)
            ax3.plot(t_imu, accel_y, linewidth=0.5, label='Accel Y', alpha=0.7)
            ax3.set_xlabel('Time (s)')
            ax3.set_ylabel('Acceleration (m/s²)')
            ax3.set_title('IMU Linear Acceleration', fontweight='bold')
            ax3.legend()
            ax3.grid(True, alpha=0.3)
        
        # Plot 4: Wheel Velocities
        ax4 = fig.add_subplot(gs[2, 0])
        if self.stats['joints']['velocities']:
            t_joints = np.array(self.stats['joints']['timestamps'])
            t_joints = t_joints - t_joints[0]
            vels = np.array(self.stats['joints']['velocities'])
            
            if vels.shape[1] >= 2:
                ax4.plot(t_joints, vels[:, 0], linewidth=0.5, 
                        label='Left Wheel', alpha=0.7)
                ax4.plot(t_joints, vels[:, 1], linewidth=0.5, 
                        label='Right Wheel', alpha=0.7)
                ax4.set_xlabel('Time (s)')
                ax4.set_ylabel('Velocity (rad/s)')
                ax4.set_title('Wheel Joint Velocities', fontweight='bold')
                ax4.legend()
                ax4.grid(True, alpha=0.3)
        
        # Plot 5: Message Frequency over Time
        ax5 = fig.add_subplot(gs[2, 1])
        window_size = 10  # Moving average window
        
        for topic, data in self.stats.items():
            if len(data['timestamps']) > window_size:
                t = np.array(data['timestamps'])
                t_rel = t - t[0]
                dt = np.diff(t)
                freq = 1.0 / dt
                
                # Moving average
                freq_smooth = np.convolve(freq, np.ones(window_size)/window_size, mode='valid')
                t_smooth = t_rel[window_size//2:-(window_size//2)]
                
                ax5.plot(t_smooth, freq_smooth, linewidth=1, 
                        label=f"{topic}", alpha=0.7)
        
        ax5.set_xlabel('Time (s)')
        ax5.set_ylabel('Frequency (Hz)')
        ax5.set_title('Message Frequency (Moving Avg)', fontweight='bold')
        ax5.legend()
        ax5.grid(True, alpha=0.3)
        
        # Save figure
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(self.output_dir, f'bag_exploration_{timestamp}.png')
        plt.savefig(filename, dpi=150, bbox_inches='tight')
        
        print(f"\n📊 Plot saved to: {filename}\n")
        
        # Show plot (comment out if running headless)
        try:
            plt.show()
        except:
            print("(Plot display skipped - running in headless mode)")
            plt.close()


def main(args=None):
    rclpy.init(args=args)
    node = BagExplorer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nCtrl+C detected. Generating summary...\n")
    finally:
        node.print_summary()
        node.plot_statistics()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
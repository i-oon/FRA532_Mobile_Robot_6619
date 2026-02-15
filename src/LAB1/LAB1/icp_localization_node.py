#!/usr/bin/env python3
"""
FRA532 LAB1 - Phase 3: ICP Localization (Scan-to-Map)
Educational implementation with clear frame handling
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
import numpy as np
import math


class ICPLocalizationNode(Node):
    def __init__(self):
        super().__init__('icp_localization_node')
        
        # ICP Parameters
        self.declare_parameter('max_iterations', 30)
        self.declare_parameter('correspondence_threshold', 0.5)  # meters
        self.declare_parameter('min_correspondences', 30)
        self.declare_parameter('convergence_threshold', 1e-5)
        
        # Keyframe Parameters  
        self.declare_parameter('keyframe_distance', 0.3)  # meters
        self.declare_parameter('keyframe_rotation', 0.174)  # ~10 degrees
        
        # Map Parameters
        self.declare_parameter('local_map_size', 10)  # number of keyframes
        self.declare_parameter('downsample_ratio', 2)  # take every Nth scan point
        
        # Get parameters
        self.max_iter = self.get_parameter('max_iterations').value
        self.corr_threshold = self.get_parameter('correspondence_threshold').value
        self.min_corr = self.get_parameter('min_correspondences').value
        self.conv_threshold = self.get_parameter('convergence_threshold').value
        
        self.kf_dist_threshold = self.get_parameter('keyframe_distance').value
        self.kf_rot_threshold = self.get_parameter('keyframe_rotation').value
        
        self.map_size = self.get_parameter('local_map_size').value
        self.downsample = self.get_parameter('downsample_ratio').value
        
        # QoS for LiDAR
        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # State - ICP pose (corrected at keyframes)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Last keyframe pose
        self.last_kf_x = 0.0
        self.last_kf_y = 0.0
        self.last_kf_theta = 0.0
        
        # Accumulated motion since last keyframe (from EKF)
        self.accumulated_dx = 0.0
        self.accumulated_dy = 0.0
        self.accumulated_dtheta = 0.0
        
        # EKF tracking
        self.ekf_x = 0.0
        self.ekf_y = 0.0
        self.ekf_theta = 0.0
        self.prev_ekf_x = None
        self.prev_ekf_y = None
        self.prev_ekf_theta = None
        self.ekf_initialized = False
        
        # Local map (stores keyframe scans in world frame)
        self.local_map_scans = []  # List of numpy arrays
        self.local_map_points = None  # Combined map
        
        # Statistics
        self.scan_count = 0
        self.keyframe_count = 0
        self.icp_success = 0
        self.icp_fail = 0
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, scan_qos
        )
        
        self.ekf_sub = self.create_subscription(
            Odometry, '/ekf_odom', self.ekf_callback, 10
        )
        
        # Publisher
        self.odom_pub = self.create_publisher(Odometry, '/icp_odom', 10)
        
        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.get_logger().info('='*60)
        self.get_logger().info('PHASE 3: ICP Localization Node Started')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Method: Scan-to-Map with keyframing')
        self.get_logger().info(f'Max iterations: {self.max_iter}')
        self.get_logger().info(f'Correspondence threshold: {self.corr_threshold}m')
        self.get_logger().info(f'Keyframe: {self.kf_dist_threshold}m or {math.degrees(self.kf_rot_threshold):.1f}°')
        self.get_logger().info(f'Local map size: {self.map_size} keyframes')
        self.get_logger().info('='*60)
    
    def ekf_callback(self, msg):
        """Track EKF odometry"""
        self.ekf_x = msg.pose.pose.position.x
        self.ekf_y = msg.pose.pose.position.y
        
        qw = msg.pose.pose.orientation.w
        qz = msg.pose.pose.orientation.z
        self.ekf_theta = math.atan2(2.0 * (qw * qz), 1.0 - 2.0 * (qz * qz))
        
        if not self.ekf_initialized:
            self.prev_ekf_x = self.ekf_x
            self.prev_ekf_y = self.ekf_y
            self.prev_ekf_theta = self.ekf_theta
            self.ekf_initialized = True
            self.get_logger().info('EKF odometry initialized')
    
    def scan_callback(self, msg):
        """Main ICP pipeline"""
        
        # Wait for EKF
        if not self.ekf_initialized:
            return
        
        # Convert scan to points (in body frame)
        scan_points = self.scan_to_points(msg)
        
        if scan_points is None or len(scan_points) < self.min_corr:
            return
        
        # Get EKF delta since last update
        ekf_dx = self.ekf_x - self.prev_ekf_x
        ekf_dy = self.ekf_y - self.prev_ekf_y
        ekf_dtheta = self.ekf_theta - self.prev_ekf_theta
        ekf_dtheta = math.atan2(math.sin(ekf_dtheta), math.cos(ekf_dtheta))
        
        # Accumulate motion since last keyframe
        self.accumulated_dx += ekf_dx
        self.accumulated_dy += ekf_dy
        self.accumulated_dtheta += ekf_dtheta
        self.accumulated_dtheta = math.atan2(math.sin(self.accumulated_dtheta), 
                                            math.cos(self.accumulated_dtheta))
        
        # Predict current pose (last keyframe + accumulated motion)
        self.x = self.last_kf_x + self.accumulated_dx
        self.y = self.last_kf_y + self.accumulated_dy
        self.theta = self.last_kf_theta + self.accumulated_dtheta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Check if keyframe
        dist_from_kf = math.sqrt(self.accumulated_dx**2 + self.accumulated_dy**2)
        angle_from_kf = abs(self.accumulated_dtheta)
        is_keyframe = (dist_from_kf > self.kf_dist_threshold or 
                      angle_from_kf > self.kf_rot_threshold)
        
        # Run ICP at keyframes (if we have a map)
        if is_keyframe and len(self.local_map_scans) >= 2:
            # Rebuild map if needed
            if self.local_map_points is None:
                self.rebuild_local_map()
            
            # Run ICP to correct pose
            if self.local_map_points is not None and len(self.local_map_points) > 50:
                success, corrected_x, corrected_y, corrected_theta = self.run_icp(
                    scan_points,
                    self.local_map_points,
                    self.x,  # Initial guess from EKF prediction
                    self.y,
                    self.theta
                )
                
                if success:
                    # Accept ICP correction
                    self.x = corrected_x
                    self.y = corrected_y
                    self.theta = corrected_theta
                    self.icp_success += 1
                else:
                    # Keep EKF prediction
                    self.icp_fail += 1
        
        # Add to map at keyframes
        if is_keyframe:
            # Transform scan to world frame and add to map
            scan_in_world = self.transform_to_world(scan_points, self.x, self.y, self.theta)
            self.local_map_scans.append(scan_in_world)
            
            # Limit map size
            if len(self.local_map_scans) > self.map_size:
                self.local_map_scans.pop(0)
            
            # Mark map for rebuild
            self.local_map_points = None
            
            # Update keyframe pose
            self.last_kf_x = self.x
            self.last_kf_y = self.y
            self.last_kf_theta = self.theta
            
            # Reset accumulator
            self.accumulated_dx = 0.0
            self.accumulated_dy = 0.0
            self.accumulated_dtheta = 0.0
            
            self.keyframe_count += 1
        
        # Publish
        self.publish_odometry(msg.header.stamp)
        
        # Update previous EKF
        self.prev_ekf_x = self.ekf_x
        self.prev_ekf_y = self.ekf_y
        self.prev_ekf_theta = self.ekf_theta
        
        # Statistics
        self.scan_count += 1
        if self.scan_count % 50 == 0:
            if self.keyframe_count > 0:
                success_rate = 100.0 * self.icp_success / (self.icp_success + self.icp_fail) if (self.icp_success + self.icp_fail) > 0 else 0
                self.get_logger().info(
                    f'[{self.scan_count:4d}] '
                    f'x={self.x:6.2f}m, y={self.y:6.2f}m, θ={math.degrees(self.theta):6.1f}° | '
                    f'KF: {self.keyframe_count}, ICP ok: {success_rate:.0f}%, Map: {len(self.local_map_scans)}'
                )
    
    def scan_to_points(self, msg):
        """Convert LaserScan to 2D points in body frame"""
        points = []
        
        angle = msg.angle_min
        for i in range(0, len(msg.ranges), self.downsample):
            r = msg.ranges[i]
            
            # Filter valid points
            if r >= msg.range_min and r <= msg.range_max and r > 0.1:
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                points.append([x, y])
            
            angle += msg.angle_increment * self.downsample
        
        return np.array(points) if len(points) > 0 else None
    
    def transform_to_world(self, points, x, y, theta):
        """Transform points from body frame to world frame"""
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        
        # Rotation + Translation
        world_points = np.zeros_like(points)
        world_points[:, 0] = points[:, 0] * cos_t - points[:, 1] * sin_t + x
        world_points[:, 1] = points[:, 0] * sin_t + points[:, 1] * cos_t + y
        
        return world_points
    
    def rebuild_local_map(self):
        """Combine all keyframe scans into one map"""
        if len(self.local_map_scans) == 0:
            self.local_map_points = None
            return
        
        # Stack all scans
        self.local_map_points = np.vstack(self.local_map_scans)
        
        self.get_logger().info(
            f'Map rebuilt: {len(self.local_map_scans)} keyframes, '
            f'{len(self.local_map_points)} points'
        )
    
    def run_icp(self, scan_body, map_world, init_x, init_y, init_theta):
        
        x, y, theta = init_x, init_y, init_theta
        prev_error = float('inf')
        
        for iteration in range(self.max_iter):
            # Transform scan to world frame with current pose estimate
            scan_world = self.transform_to_world(scan_body, x, y, theta)
            
            # Find nearest neighbors (scan points → map points)
            correspondences = []
            correspondence_indices = []  # Track which scan points matched
            distances = []
            
            for idx, scan_pt in enumerate(scan_world):
                # Find closest map point
                dists = np.linalg.norm(map_world - scan_pt, axis=1)
                min_idx = np.argmin(dists)
                min_dist = dists[min_idx]
                
                if min_dist < self.corr_threshold:
                    correspondences.append((scan_pt, map_world[min_idx]))
                    correspondence_indices.append(idx)  # ✅ Track original scan index
                    distances.append(min_dist)
            
            # Check sufficient correspondences
            if len(correspondences) < self.min_corr:
                return False, init_x, init_y, init_theta
            
            # Check convergence
            mean_error = np.mean(distances)
            if abs(prev_error - mean_error) < self.conv_threshold:
                break
            prev_error = mean_error
            
            # ✅ CRITICAL FIX: Use ORIGINAL body-frame scan, not transformed!
            scan_matched_body = scan_body[correspondence_indices]  # Body frame!
            map_matched_world = np.array([c[1] for c in correspondences])  # World frame
            
            # Compute centroids
            scan_centroid = np.mean(scan_matched_body, axis=0)
            map_centroid = np.mean(map_matched_world, axis=0)
            
            # Center points
            scan_centered = scan_matched_body - scan_centroid
            map_centered = map_matched_world - map_centroid
            
            # Cross-covariance matrix (body frame → world frame)
            H = scan_centered.T @ map_centered
            
            # SVD
            U, _, Vt = np.linalg.svd(H)
            R = Vt.T @ U.T
            
            # Ensure proper rotation
            if np.linalg.det(R) < 0:
                Vt[-1, :] *= -1
                R = Vt.T @ U.T
            
            # Translation (world frame)
            t = map_centroid - R @ scan_centroid
            
            # ✅ Extract global pose directly
            x = t[0]
            y = t[1]
            theta = math.atan2(R[1, 0], R[0, 0])
        
        # Final error check
        final_scan_world = self.transform_to_world(scan_body, x, y, theta)
        final_dists = []
        for scan_pt in final_scan_world:
            dists = np.linalg.norm(map_world - scan_pt, axis=1)
            final_dists.append(np.min(dists))
        
        final_error = np.mean(final_dists)
        
        # Success if error is reasonable
        success = final_error < self.corr_threshold
        
        return success, x, y, theta
    
    def publish_odometry(self, stamp):
        """Publish ICP odometry"""
        
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Orientation
        quat = self.euler_to_quaternion(self.theta)
        odom.pose.pose.orientation = quat
        
        # Covariance
        odom.pose.covariance[0] = 0.0005   # x
        odom.pose.covariance[7] = 0.0005   # y
        odom.pose.covariance[35] = 0.005   # theta
        
        self.odom_pub.publish(odom)
        
        # TF
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link_icp'
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
    node = ICPLocalizationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print()
        node.get_logger().info('='*60)
        node.get_logger().info('PHASE 3: ICP Localization Results')
        node.get_logger().info('='*60)
        node.get_logger().info(f'Scans processed: {node.scan_count}')
        node.get_logger().info(f'Keyframes created: {node.keyframe_count}')
        node.get_logger().info(f'ICP success: {node.icp_success}')
        node.get_logger().info(f'ICP failures: {node.icp_fail}')
        if node.keyframe_count > 0:
            success_rate = 100.0 * node.icp_success / (node.icp_success + node.icp_fail) if (node.icp_success + node.icp_fail) > 0 else 0
            node.get_logger().info(f'Success rate: {success_rate:.1f}%')
        node.get_logger().info(f'Final position: x={node.x:.3f}m, y={node.y:.3f}m')
        node.get_logger().info(f'Final heading: {math.degrees(node.theta):.1f}°')
        node.get_logger().info('='*60)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
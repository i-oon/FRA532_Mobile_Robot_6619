#!/usr/bin/env python3
"""
ICP (Iterative Closest Point) Scan Matching Utility
2D implementation for LiDAR scan matching
"""

import numpy as np
from scipy.spatial import KDTree


class ICP2D:
    """
    2D ICP for LiDAR scan matching
    """
    
    def __init__(self, max_iterations=50, tolerance=1e-6, max_correspondence_distance=0.5):
        """
        Initialize ICP
        
        Args:
            max_iterations: Maximum number of iterations
            tolerance: Convergence threshold
            max_correspondence_distance: Maximum distance for point matching (meters)
        """
        self.max_iterations = max_iterations
        self.tolerance = tolerance
        self.max_correspondence_distance = max_correspondence_distance
    
    def align(self, source, target, initial_transform=None):
        """
        Align source scan to target scan using ICP
        
        Args:
            source: Source point cloud (N x 2)
            target: Target point cloud (M x 2)
            initial_transform: Initial [dx, dy, dtheta] guess
            
        Returns:
            transform: [dx, dy, dtheta] final transformation
            fitness: Alignment quality score (0-1)
            converged: Whether algorithm converged
        """
        
        if initial_transform is None:
            transform = np.array([0.0, 0.0, 0.0])
        else:
            transform = np.array(initial_transform)
        
        # Transform source
        transformed_source = self._apply_transform(source, transform)
        
        prev_error = float('inf')
        
        for iteration in range(self.max_iterations):
            
            # Find correspondences
            indices, distances = self._find_correspondences(transformed_source, target)
            
            # Filter by distance
            valid = distances < self.max_correspondence_distance
            
            if np.sum(valid) < 3:  # Need at least 3 points
                return transform, 0.0, False
            
            # Compute transformation
            delta = self._compute_transformation(
                transformed_source[valid],
                target[indices[valid]]
            )
            
            # Update transform
            transform = self._compose_transforms(transform, delta)
            
            # Apply to source
            transformed_source = self._apply_transform(source, transform)
            
            # Check convergence
            error = np.mean(distances[valid])
            
            if abs(prev_error - error) < self.tolerance:
                fitness = self._compute_fitness(distances, valid)
                return transform, fitness, True
            
            prev_error = error
        
        # Max iterations reached
        fitness = self._compute_fitness(distances, valid)
        return transform, fitness, False
    
    def _find_correspondences(self, source, target):
        """
        Find nearest neighbors in target for each source point
        
        Returns:
            indices: Target indices for each source point
            distances: Distance to nearest neighbor
        """
        tree = KDTree(target)
        distances, indices = tree.query(source)
        return indices, distances
    
    def _compute_transformation(self, source, target):
        """
        Compute transformation from source to target
        Uses SVD for optimal rotation + translation
        
        Returns:
            [dx, dy, dtheta]
        """
        
        # Centroids
        source_centroid = np.mean(source, axis=0)
        target_centroid = np.mean(target, axis=0)
        
        # Centered
        source_centered = source - source_centroid
        target_centered = target - target_centroid
        
        # Compute rotation using SVD
        # CRITICAL: Correct order is target.T @ source
        H = target_centered.T @ source_centered
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        
        # Handle reflection
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T
        
        # Extract angle
        theta = np.arctan2(R[1, 0], R[0, 0])
        
        # Compute translation
        t = target_centroid - R @ source_centroid
        
        return np.array([t[0], t[1], theta])
    
    def _apply_transform(self, points, transform):
        """
        Apply transformation to points
        
        Args:
            points: N x 2 array
            transform: [dx, dy, dtheta]
            
        Returns:
            transformed: N x 2 array
        """
        dx, dy, theta = transform
        
        # Rotation matrix
        c = np.cos(theta)
        s = np.sin(theta)
        R = np.array([[c, -s],
                      [s,  c]])
        
        # Apply
        transformed = points @ R.T
        transformed[:, 0] += dx
        transformed[:, 1] += dy
        
        return transformed
    
    def _compose_transforms(self, t1, t2):
        """
        Compose two transformations
        
        Args:
            t1: [dx1, dy1, dtheta1]
            t2: [dx2, dy2, dtheta2]
            
        Returns:
            composed: [dx, dy, dtheta]
        """
        dx1, dy1, theta1 = t1
        dx2, dy2, theta2 = t2
        
        # Rotate translation by theta1
        c = np.cos(theta1)
        s = np.sin(theta1)
        
        dx = dx1 + dx2 * c - dy2 * s
        dy = dy1 + dx2 * s + dy2 * c
        theta = theta1 + theta2
        
        # Normalize angle
        theta = np.arctan2(np.sin(theta), np.cos(theta))
        
        return np.array([dx, dy, theta])
    
    def _compute_fitness(self, distances, valid):
        """
        Compute alignment quality score
        
        Returns:
            fitness: 0-1, higher is better
        """
        if np.sum(valid) == 0:
            return 0.0
        
        # Percentage of inliers
        inlier_ratio = np.sum(valid) / len(distances)
        
        # Average inlier distance (normalized)
        avg_distance = np.mean(distances[valid])
        distance_score = max(0, 1.0 - avg_distance / self.max_correspondence_distance)
        
        # Combined score
        fitness = 0.5 * inlier_ratio + 0.5 * distance_score
        
        return fitness


def scan_to_points(scan_msg, max_range=10.0, min_range=0.1):

    ranges = np.array(scan_msg.ranges)
    angles = scan_msg.angle_min + np.arange(len(ranges)) * scan_msg.angle_increment
    
    # Filter valid ranges
    valid = (ranges >= min_range) & (ranges <= max_range) & np.isfinite(ranges)
    
    ranges = ranges[valid]
    angles = angles[valid]
    
    # Convert to Cartesian
    x = ranges * np.cos(angles)
    y = ranges * np.sin(angles)
    
    points = np.column_stack([x, y])
    
    return points


def downsample_points(points, voxel_size=0.05):
    if len(points) == 0:
        return points
    
    # Voxel grid indices
    voxel_indices = np.floor(points / voxel_size).astype(int)
    
    # Unique voxels
    _, unique_indices = np.unique(voxel_indices, axis=0, return_index=True)
    
    return points[unique_indices]
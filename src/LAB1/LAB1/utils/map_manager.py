#!/usr/bin/env python3
"""
Global Map Manager for Scan-to-Map ICP
Builds and maintains accumulated point cloud map
"""

import numpy as np
from scipy.spatial import KDTree


class GlobalMapManager:
    """
    Manages global point cloud map built from LiDAR scans
    """
    
    def __init__(self, voxel_size=0.05, max_points=100000):
        """
        Initialize map manager
        
        Args:
            voxel_size: Voxel grid size for downsampling
            max_points: Maximum points in map (for memory management)
        """
        self.voxel_size = voxel_size
        self.max_points = max_points
        
        # Global map storage (x, y coordinates)
        self.map_points = np.empty((0, 2), dtype=np.float32)
        
        # Voxel grid for fast duplicate checking
        self.voxel_grid = {}
        
        # Statistics
        self.num_scans_added = 0
        self.total_points_added = 0
    
    def add_scan(self, points, pose):
        """
        Add scan to global map
        
        Args:
            points: N x 2 scan points in sensor frame
            pose: [x, y, theta] robot pose in world frame
        """
        
        if len(points) == 0:
            return
        
        # Transform points to world frame
        world_points = self._transform_to_world(points, pose)
        
        # Add to map with voxel filtering
        new_points = self._filter_and_add(world_points)
        
        # Update statistics
        self.num_scans_added += 1
        self.total_points_added += len(new_points)
        
        # Downsample if map too large
        if len(self.map_points) > self.max_points:
            self._downsample_map()
    
    def get_map(self):
        """
        Get current global map
        
        Returns:
            map_points: M x 2 array of world coordinates
        """
        return self.map_points.copy()
    
    def get_local_map(self, center, radius):
        """
        Get local submap around a point
        
        Args:
            center: [x, y] query point
            radius: Search radius (meters)
            
        Returns:
            local_points: Points within radius
        """
        
        if len(self.map_points) == 0:
            return np.empty((0, 2))
        
        # Compute distances
        dx = self.map_points[:, 0] - center[0]
        dy = self.map_points[:, 1] - center[1]
        distances = np.sqrt(dx**2 + dy**2)
        
        # Filter by radius
        mask = distances <= radius
        
        return self.map_points[mask]
    
    def clear(self):
        """Clear the map"""
        self.map_points = np.empty((0, 2), dtype=np.float32)
        self.voxel_grid.clear()
        self.num_scans_added = 0
        self.total_points_added = 0
    
    def get_stats(self):
        """
        Get map statistics
        
        Returns:
            dict with statistics
        """
        return {
            'num_points': len(self.map_points),
            'num_scans': self.num_scans_added,
            'total_added': self.total_points_added,
            'voxel_size': self.voxel_size
        }
    
    def _transform_to_world(self, points, pose):
        """
        Transform points from sensor frame to world frame
        
        Args:
            points: N x 2 in sensor frame
            pose: [x, y, theta] robot pose
            
        Returns:
            world_points: N x 2 in world frame
        """
        x, y, theta = pose
        
        # Rotation matrix
        c = np.cos(theta)
        s = np.sin(theta)
        R = np.array([[c, -s],
                      [s,  c]])
        
        # Rotate and translate
        world_points = points @ R.T
        world_points[:, 0] += x
        world_points[:, 1] += y
        
        return world_points
    
    def _filter_and_add(self, points):
        """
        Add points to map using voxel grid filtering
        Only adds points in new voxels
        
        Returns:
            new_points: Points actually added
        """
        
        new_points = []
        
        for point in points:
            # Compute voxel index
            voxel_idx = self._point_to_voxel(point)
            
            # Add if voxel is new
            if voxel_idx not in self.voxel_grid:
                self.voxel_grid[voxel_idx] = point
                new_points.append(point)
        
        # Add to map
        if new_points:
            new_points = np.array(new_points, dtype=np.float32)
            self.map_points = np.vstack([self.map_points, new_points])
        
        return new_points
    
    def _point_to_voxel(self, point):
        """
        Convert point to voxel grid index
        
        Returns:
            (vx, vy) voxel indices
        """
        vx = int(np.floor(point[0] / self.voxel_size))
        vy = int(np.floor(point[1] / self.voxel_size))
        return (vx, vy)
    
    def _downsample_map(self):
        """
        Downsample map when it gets too large
        Keeps most recent points (assumes later scans are better)
        """
        
        # Keep last max_points
        keep_count = int(self.max_points * 0.8)  # Keep 80%
        self.map_points = self.map_points[-keep_count:]
        
        # Rebuild voxel grid
        self.voxel_grid.clear()
        for point in self.map_points:
            voxel_idx = self._point_to_voxel(point)
            self.voxel_grid[voxel_idx] = point


class SubmapManager:
    """
    Alternative: Manages multiple local submaps instead of one global map
    More efficient for large environments
    """
    
    def __init__(self, submap_size=10.0, voxel_size=0.05):
        """
        Initialize submap manager
        
        Args:
            submap_size: Size of each submap (meters)
            voxel_size: Voxel grid size for downsampling
        """
        self.submap_size = submap_size
        self.voxel_size = voxel_size
        
        # Dictionary of submaps {(grid_x, grid_y): points}
        self.submaps = {}
    
    def add_scan(self, points, pose):
        """Add scan to relevant submaps"""
        
        if len(points) == 0:
            return
        
        # Transform to world
        x, y, theta = pose
        c = np.cos(theta)
        s = np.sin(theta)
        R = np.array([[c, -s], [s, c]])
        
        world_points = points @ R.T
        world_points[:, 0] += x
        world_points[:, 1] += y
        
        # Determine which submaps these points belong to
        for point in world_points:
            submap_idx = self._point_to_submap(point)
            
            if submap_idx not in self.submaps:
                self.submaps[submap_idx] = []
            
            self.submaps[submap_idx].append(point)
    
    def get_local_map(self, center, radius):
        """
        Get points from submaps around center
        
        Args:
            center: [x, y]
            radius: Search radius
            
        Returns:
            Combined points from relevant submaps
        """
        
        # Find relevant submaps
        cx, cy = center
        grid_radius = int(np.ceil(radius / self.submap_size))
        center_grid = self._point_to_submap(center)
        
        relevant_points = []
        
        for dx in range(-grid_radius, grid_radius + 1):
            for dy in range(-grid_radius, grid_radius + 1):
                submap_idx = (center_grid[0] + dx, center_grid[1] + dy)
                
                if submap_idx in self.submaps:
                    relevant_points.extend(self.submaps[submap_idx])
        
        if not relevant_points:
            return np.empty((0, 2))
        
        points = np.array(relevant_points)
        
        # Filter by actual radius
        dx = points[:, 0] - cx
        dy = points[:, 1] - cy
        distances = np.sqrt(dx**2 + dy**2)
        
        return points[distances <= radius]
    
    def _point_to_submap(self, point):
        """Get submap index for point"""
        sx = int(np.floor(point[0] / self.submap_size))
        sy = int(np.floor(point[1] / self.submap_size))
        return (sx, sy)
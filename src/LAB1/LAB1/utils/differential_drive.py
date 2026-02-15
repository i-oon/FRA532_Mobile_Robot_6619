#!/usr/bin/env python3
import numpy as np
from dataclasses import dataclass


@dataclass
class DifferentialDriveParams:

    wheel_separation: float  # L (m) - distance between wheels
    wheel_radius: float      # R (m) - wheel radius
    @classmethod
    def turtlebot3_burger(cls):
        return cls(
            wheel_separation=0.160,  # 16 cm
            wheel_radius=0.033       # 3.3 cm
        )

class DifferentialDrive:
    
    def __init__(self, params: DifferentialDriveParams):
        self.params = params
        self.L = params.wheel_separation
        self.R = params.wheel_radius
    
    def forward_kinematics(self, omega_left: float, omega_right: float) -> tuple:

        # Convert to wheel linear velocities
        v_left = self.R * omega_left
        v_right = self.R * omega_right
        # Robot velocities
        v = (v_left + v_right) / 2.0       # Linear velocity
        omega = (v_right - v_left) / self.L  # Angular velocity
        
        return v, omega
    
    def inverse_kinematics(self, v: float, omega: float) -> tuple:

        # Wheel linear velocities
        v_left = v - (self.L / 2.0) * omega
        v_right = v + (self.L / 2.0) * omega
        # Wheel angular velocities
        omega_left = v_left / self.R
        omega_right = v_right / self.R
        
        return omega_left, omega_right
    
    def integrate_pose(self, x: float, y: float, theta: float,
                      v: float, omega: float, dt: float) -> tuple:

        # Update heading first
        theta_new = theta + omega * dt
        # Normalize theta to [-pi, pi]
        theta_new = np.arctan2(np.sin(theta_new), np.cos(theta_new))
        # Use midpoint method for position (more accurate than simple Euler)
        theta_mid = theta + omega * dt / 2.0
        x_new = x + v * np.cos(theta_mid) * dt
        y_new = y + v * np.sin(theta_mid) * dt
        
        return x_new, y_new, theta_new
    
    def integrate_pose_exact(self, x: float, y: float, theta: float,
                        v: float, omega: float, dt: float) -> tuple:
        """Exact integration using midpoint method"""
        
        # Update heading FIRST
        theta_new = theta + omega * dt
        theta_new = np.arctan2(np.sin(theta_new), np.cos(theta_new))
        
        # Use AVERAGE heading for position (critical fix!)
        theta_avg = theta + omega * dt / 2.0
        
        x_new = x + v * np.cos(theta_avg) * dt
        y_new = y + v * np.sin(theta_avg) * dt
        
        return x_new, y_new, theta_new


if __name__ == '__main__':
    # Create kinematics calculator
    params = DifferentialDriveParams.turtlebot3_burger()
    kinematics = DifferentialDrive(params)
    
    print("Turtlebot3 Burger Differential Drive Kinematics")
    print(f"Wheel separation: {params.wheel_separation} m")
    print(f"Wheel radius: {params.wheel_radius} m")
    print()
    
    # Test 1: Forward motion
    print("Test 1: Forward motion (both wheels same speed)")
    omega_l = omega_r = 1.0  # rad/s
    v, omega = kinematics.forward_kinematics(omega_l, omega_r)
    print(f"  Wheel velocities: {omega_l:.3f}, {omega_r:.3f} rad/s")
    print(f"  Robot velocity: v={v:.4f} m/s, ω={omega:.4f} rad/s")
    print()
    
    # Test 2: Left turn
    print("Test 2: Left turn (right wheel faster)")
    omega_l = 0.5
    omega_r = 1.5
    v, omega = kinematics.forward_kinematics(omega_l, omega_r)
    print(f"  Wheel velocities: {omega_l:.3f}, {omega_r:.3f} rad/s")
    print(f"  Robot velocity: v={v:.4f} m/s, ω={omega:.4f} rad/s")
    print(f"  Turning radius: {v/omega:.3f} m" if abs(omega) > 1e-6 else "  Straight line")
    print()
    
    # Test 3: In-place rotation
    print("Test 3: In-place rotation")
    omega_l = -1.0
    omega_r = 1.0
    v, omega = kinematics.forward_kinematics(omega_l, omega_r)
    print(f"  Wheel velocities: {omega_l:.3f}, {omega_r:.3f} rad/s")
    print(f"  Robot velocity: v={v:.4f} m/s, ω={omega:.4f} rad/s")
    print()
    
    # Test 4: Inverse kinematics
    print("Test 4: Inverse kinematics")
    v_target = 0.1  # m/s
    omega_target = 0.5  # rad/s
    omega_l, omega_r = kinematics.inverse_kinematics(v_target, omega_target)
    print(f"  Target: v={v_target:.3f} m/s, ω={omega_target:.3f} rad/s")
    print(f"  Required wheel velocities: {omega_l:.3f}, {omega_r:.3f} rad/s")
    # Verify
    v_check, omega_check = kinematics.forward_kinematics(omega_l, omega_r)
    print(f"  Verification: v={v_check:.3f} m/s, ω={omega_check:.3f} rad/s")
    print()
    
    # Test 5: Pose integration
    print("Test 5: Pose integration (1 second forward)")
    x, y, theta = 0.0, 0.0, 0.0
    v, omega = 0.1, 0.0  # Straight forward
    dt = 1.0
    x_new, y_new, theta_new = kinematics.integrate_pose(x, y, theta, v, omega, dt)
    print(f"  Initial pose: ({x:.3f}, {y:.3f}, {np.rad2deg(theta):.1f}°)")
    print(f"  Velocity: v={v:.3f} m/s, ω={omega:.3f} rad/s")
    print(f"  Final pose: ({x_new:.3f}, {y_new:.3f}, {np.rad2deg(theta_new):.1f}°)")
    print(f"  Distance traveled: {np.sqrt(x_new**2 + y_new**2):.3f} m")


""" result:

Turtlebot3 Burger Differential Drive Kinematics
Wheel separation: 0.16 m
Wheel radius: 0.033 m

Test 1: Forward motion (both wheels same speed)
  Wheel velocities: 1.000, 1.000 rad/s
  Robot velocity: v=0.0330 m/s, ω=0.0000 rad/s

Test 2: Left turn (right wheel faster)
  Wheel velocities: 0.500, 1.500 rad/s
  Robot velocity: v=0.0330 m/s, ω=0.2063 rad/s
  Turning radius: 0.160 m

Test 3: In-place rotation
  Wheel velocities: -1.000, 1.000 rad/s
  Robot velocity: v=0.0000 m/s, ω=0.4125 rad/s

Test 4: Inverse kinematics
  Target: v=0.100 m/s, ω=0.500 rad/s
  Required wheel velocities: 1.818, 4.242 rad/s
  Verification: v=0.100 m/s, ω=0.500 rad/s

Test 5: Pose integration (1 second forward)
  Initial pose: (0.000, 0.000, 0.0°)
  Velocity: v=0.100 m/s, ω=0.000 rad/s
  Final pose: (0.100, 0.000, 0.0°)
  Distance traveled: 0.100 m

"""
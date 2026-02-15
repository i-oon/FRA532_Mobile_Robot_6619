#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
from pathlib import Path


def main():
    # Get results directory
    home = str(Path.home())
    results_dir = os.path.join(home, 'FRA532_Mobile_Robot_6619/results', 'phase2')
    
    # CSV file paths
    wheel_csv = os.path.join(results_dir, 'phase2_wheel_odom.csv')
    ekf_csv = os.path.join(results_dir, 'phase2_ekf_odom.csv')
    
    # Check files exist
    if not os.path.exists(wheel_csv):
        print(f'ERROR: {wheel_csv} not found!')
        print('Run record_phase2_data.py first to collect data.')
        return
    
    if not os.path.exists(ekf_csv):
        print(f'ERROR: {ekf_csv} not found!')
        print('Run record_phase2_data.py first to collect data.')
        return
    
    # Load data
    print('Loading data...')
    wheel = pd.read_csv(wheel_csv)
    ekf = pd.read_csv(ekf_csv)
    
    print(f'Wheel odom: {len(wheel)} points')
    print(f'EKF odom: {len(ekf)} points')
    
    # Convert theta to degrees
    wheel['theta_deg'] = np.rad2deg(wheel['theta'])
    ekf['theta_deg'] = np.rad2deg(ekf['theta'])
    
    # Normalize time to start from 0
    wheel['time'] = wheel['timestamp'] - wheel['timestamp'].iloc[0]
    ekf['time'] = ekf['timestamp'] - ekf['timestamp'].iloc[0]
    
    # Create plots
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    axes = axes.flatten()
    
    # Plot 1: XY Trajectories
    axes[0].plot(wheel['x'].values, wheel['y'].values, 'b-', 
                linewidth=2, alpha=0.7, label='Wheel Odom')
    axes[0].plot(ekf['x'].values, ekf['y'].values, 'r-', 
                linewidth=2, alpha=0.7, label='EKF Fusion')
    axes[0].plot(0, 0, 'go', markersize=10, label='Start', zorder=5)
    
    axes[0].set_xlabel('X (m)', fontsize=11)
    axes[0].set_ylabel('Y (m)', fontsize=11)
    axes[0].set_title('Trajectory Comparison', fontsize=12, fontweight='bold')
    axes[0].legend(fontsize=10)
    axes[0].grid(True, alpha=0.3)
    axes[0].axis('equal')
    
    # Plot 2: Heading Comparison
    axes[1].plot(wheel['time'].values, wheel['theta_deg'].values, 'b-',
                linewidth=2, alpha=0.7, label='Wheel Odom')
    axes[1].plot(ekf['time'].values, ekf['theta_deg'].values, 'r-',
                linewidth=2, alpha=0.7, label='EKF Fusion')
    
    axes[1].set_xlabel('Time (s)', fontsize=11)
    axes[1].set_ylabel('Heading (deg)', fontsize=11)
    axes[1].set_title('Heading Comparison', fontsize=12, fontweight='bold')
    axes[1].legend(fontsize=10)
    axes[1].grid(True, alpha=0.3)
    
    # Plot 3: Position Error
    min_len = min(len(wheel), len(ekf))
    if min_len > 10:
        pos_err = np.sqrt(
            (wheel['x'].values[:min_len] - ekf['x'].values[:min_len])**2 +
            (wheel['y'].values[:min_len] - ekf['y'].values[:min_len])**2
        )
        time_axis = wheel['time'].values[:min_len]
        
        axes[2].plot(time_axis, pos_err, 'purple', linewidth=2)
        axes[2].set_xlabel('Time (s)', fontsize=11)
        axes[2].set_ylabel('Position Difference (m)', fontsize=11)
        axes[2].set_title('Wheel vs EKF Position Error', fontsize=12, fontweight='bold')
        axes[2].grid(True, alpha=0.3)
    
    # Plot 4: Heading Error
    if min_len > 10:
        heading_err = np.abs(
            wheel['theta_deg'].values[:min_len] - ekf['theta_deg'].values[:min_len]
        )
        time_axis = wheel['time'].values[:min_len]
        
        axes[3].plot(time_axis, heading_err, 'orange', linewidth=2)
        axes[3].set_xlabel('Time (s)', fontsize=11)
        axes[3].set_ylabel('Heading Difference (deg)', fontsize=11)
        axes[3].set_title('Wheel vs EKF Heading Error', fontsize=12, fontweight='bold')
        axes[3].grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    # Save plots
    png_path = os.path.join(results_dir, 'phase2_comparison.png')
    svg_path = os.path.join(results_dir, 'phase2_comparison.svg')
    
    fig.savefig(png_path, dpi=150, bbox_inches='tight')
    fig.savefig(svg_path, bbox_inches='tight')
    
    # Print results
    print('\n' + '='*60)
    print('PHASE 2: COMPARISON RESULTS')
    print('='*60)
    
    print(f'\nWheel Odometry:')
    print(f'  End: ({wheel["x"].iloc[-1]:.3f}, {wheel["y"].iloc[-1]:.3f})')
    print(f'  Heading: {wheel["theta_deg"].iloc[-1]:.1f}°')
    wheel_err = np.sqrt(wheel['x'].iloc[-1]**2 + wheel['y'].iloc[-1]**2)
    print(f'  Loop error: {wheel_err:.3f}m')
    
    print(f'\nEKF Fusion:')
    print(f'  End: ({ekf["x"].iloc[-1]:.3f}, {ekf["y"].iloc[-1]:.3f})')
    print(f'  Heading: {ekf["theta_deg"].iloc[-1]:.1f}°')
    ekf_err = np.sqrt(ekf['x'].iloc[-1]**2 + ekf['y'].iloc[-1]**2)
    print(f'  Loop error: {ekf_err:.3f}m')
    
    print(f'\nImprovement:')
    improvement = ((wheel_err - ekf_err) / wheel_err) * 100
    print(f'  Loop closure: {improvement:+.1f}%')
    
    heading_diff = abs(wheel['theta_deg'].iloc[-1] - ekf['theta_deg'].iloc[-1])
    print(f'  Heading difference: {heading_diff:.1f}°')
    
    print(f'\nPlots saved:')
    print(f'  {png_path}')
    print(f'  {svg_path}')
    print('='*60)
    
    plt.show()


if __name__ == '__main__':
    main()
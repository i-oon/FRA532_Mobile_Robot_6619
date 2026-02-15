#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
from pathlib import Path


def main():
    # Get results directory
    home = str(Path.home())
    results_dir = os.path.join(home, 'FRA532_Mobile_Robot_6619', 'results', 'phase4')
    
    # CSV file paths
    wheel_csv = os.path.join(results_dir, 'phase4_wheel_odom.csv')
    ekf_csv = os.path.join(results_dir, 'phase4_ekf_odom.csv')
    icp_csv = os.path.join(results_dir, 'phase4_icp_odom.csv')
    slam_csv = os.path.join(results_dir, 'phase4_slam_pose.csv')
    
    # Check files exist
    for fname, fpath in [('Wheel', wheel_csv), ('EKF', ekf_csv), 
                         ('ICP', icp_csv), ('SLAM', slam_csv)]:
        if not os.path.exists(fpath):
            print(f'ERROR: {fname} file not found: {fpath}')
            print('Run record_phase4_data.py first to collect data.')
            return
    
    # Load data
    print('Loading data...')
    wheel = pd.read_csv(wheel_csv)
    ekf = pd.read_csv(ekf_csv)
    icp = pd.read_csv(icp_csv)
    slam = pd.read_csv(slam_csv)
    
    print(f'Wheel odom: {len(wheel)} points')
    print(f'EKF odom: {len(ekf)} points')
    print(f'ICP odom: {len(icp)} points')
    print(f'SLAM pose: {len(slam)} points')
    
    # Convert theta to degrees
    wheel['theta_deg'] = np.rad2deg(wheel['theta'])
    ekf['theta_deg'] = np.rad2deg(ekf['theta'])
    icp['theta_deg'] = np.rad2deg(icp['theta'])
    slam['theta_deg'] = np.rad2deg(slam['theta'])
    
    # Normalize time to start from 0
    wheel['time'] = wheel['timestamp'] - wheel['timestamp'].iloc[0]
    ekf['time'] = ekf['timestamp'] - ekf['timestamp'].iloc[0]
    icp['time'] = icp['timestamp'] - icp['timestamp'].iloc[0]
    slam['time'] = slam['timestamp'] - slam['timestamp'].iloc[0]
    
    # Create plots - 2 rows, 3 columns
    fig = plt.figure(figsize=(20, 11))
    
    # Main trajectory plot (larger, spans 2 columns)
    ax_traj = plt.subplot2grid((2, 3), (0, 0), colspan=2, rowspan=2)
    
    # Other plots
    ax_heading = plt.subplot2grid((2, 3), (0, 2))
    ax_loop = plt.subplot2grid((2, 3), (1, 2))
    
    # Plot 1: XY Trajectories (MAIN PLOT)
    ax_traj.plot(wheel['x'].values, wheel['y'].values, 'b-', 
                 linewidth=2.5, alpha=0.6, label='Wheel Odom')
    ax_traj.plot(ekf['x'].values, ekf['y'].values, 'r-', 
                 linewidth=2.5, alpha=0.6, label='EKF Fusion')
    ax_traj.plot(icp['x'].values, icp['y'].values, 'g-', 
                 linewidth=2.5, alpha=0.7, label='ICP Odometry')
    ax_traj.plot(slam['x'].values, slam['y'].values, 'm-', 
                 linewidth=3, alpha=0.8, label='SLAM (Loop Closure)')
    ax_traj.plot(0, 0, 'ko', markersize=15, label='Start', zorder=10)
    
    # Mark end points
    ax_traj.plot(wheel['x'].iloc[-1], wheel['y'].iloc[-1], 'bs', 
                 markersize=10, alpha=0.6)
    ax_traj.plot(ekf['x'].iloc[-1], ekf['y'].iloc[-1], 'rs', 
                 markersize=10, alpha=0.6)
    ax_traj.plot(icp['x'].iloc[-1], icp['y'].iloc[-1], 'gs', 
                 markersize=10, alpha=0.7)
    ax_traj.plot(slam['x'].iloc[-1], slam['y'].iloc[-1], 'ms', 
                 markersize=12, alpha=0.8, label='End Points')
    
    ax_traj.set_xlabel('X (m)', fontsize=13, fontweight='bold')
    ax_traj.set_ylabel('Y (m)', fontsize=13, fontweight='bold')
    ax_traj.set_title('Trajectory Comparison - All Methods', 
                      fontsize=14, fontweight='bold')
    ax_traj.legend(fontsize=11, loc='best')
    ax_traj.grid(True, alpha=0.3)
    ax_traj.axis('equal')
    
    # Plot 2: Heading Comparison
    ax_heading.plot(wheel['time'].values, wheel['theta_deg'].values, 'b-',
                    linewidth=2, alpha=0.6, label='Wheel')
    ax_heading.plot(ekf['time'].values, ekf['theta_deg'].values, 'r-',
                    linewidth=2, alpha=0.6, label='EKF')
    ax_heading.plot(icp['time'].values, icp['theta_deg'].values, 'g-',
                    linewidth=2, alpha=0.7, label='ICP')
    ax_heading.plot(slam['time'].values, slam['theta_deg'].values, 'm-',
                    linewidth=2.5, alpha=0.8, label='SLAM')
    
    ax_heading.set_xlabel('Time (s)', fontsize=11)
    ax_heading.set_ylabel('Heading (deg)', fontsize=11)
    ax_heading.set_title('Heading Comparison', fontsize=12, fontweight='bold')
    ax_heading.legend(fontsize=9)
    ax_heading.grid(True, alpha=0.3)
    
    # Plot 3: Loop Closure Error Over Time
    wheel['loop_error'] = np.sqrt(wheel['x']**2 + wheel['y']**2)
    ekf['loop_error'] = np.sqrt(ekf['x']**2 + ekf['y']**2)
    icp['loop_error'] = np.sqrt(icp['x']**2 + icp['y']**2)
    slam['loop_error'] = np.sqrt(slam['x']**2 + slam['y']**2)
    
    ax_loop.plot(wheel['time'].values, wheel['loop_error'].values, 'b-',
                 linewidth=2, alpha=0.6, label='Wheel')
    ax_loop.plot(ekf['time'].values, ekf['loop_error'].values, 'r-',
                 linewidth=2, alpha=0.6, label='EKF')
    ax_loop.plot(icp['time'].values, icp['loop_error'].values, 'g-',
                 linewidth=2, alpha=0.7, label='ICP')
    ax_loop.plot(slam['time'].values, slam['loop_error'].values, 'm-',
                 linewidth=2.5, alpha=0.8, label='SLAM')
    
    ax_loop.set_xlabel('Time (s)', fontsize=11)
    ax_loop.set_ylabel('Distance from Origin (m)', fontsize=11)
    ax_loop.set_title('Loop Closure Error', fontsize=12, fontweight='bold')
    ax_loop.legend(fontsize=9)
    ax_loop.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    # Save plots
    png_path = os.path.join(results_dir, 'phase4_comparison.png')
    svg_path = os.path.join(results_dir, 'phase4_comparison.svg')
    
    fig.savefig(png_path, dpi=150, bbox_inches='tight')
    fig.savefig(svg_path, bbox_inches='tight')
    
    print(f'\n📊 Plots saved:')
    print(f'  PNG: {png_path}')
    print(f'  SVG: {svg_path}')
    print('\n📈 Close the plot window to see results...\n')
    
    # Show plot (blocks until window closed)
    try:
        plt.show()
    except KeyboardInterrupt:
        print('\n\n⚠️  Plot interrupted')
    
    # ✅ Print results AFTER plot closes
    print('\n' + '='*70)
    print(' '*20 + 'PHASE 4: FULL COMPARISON RESULTS')
    print('='*70)
    
    # Calculate errors
    wheel_err = np.sqrt(wheel['x'].iloc[-1]**2 + wheel['y'].iloc[-1]**2)
    ekf_err = np.sqrt(ekf['x'].iloc[-1]**2 + ekf['y'].iloc[-1]**2)
    icp_err = np.sqrt(icp['x'].iloc[-1]**2 + icp['y'].iloc[-1]**2)
    slam_err = np.sqrt(slam['x'].iloc[-1]**2 + slam['y'].iloc[-1]**2)
    
    print(f'\n{"Method":<20} {"End Position":<25} {"Heading":<12} {"Loop Error":<12}')
    print('-'*70)
    
    print(f'{"Wheel Odometry":<20} '
          f'({wheel["x"].iloc[-1]:6.3f}, {wheel["y"].iloc[-1]:6.3f})m    '
          f'{wheel["theta_deg"].iloc[-1]:6.1f}°     '
          f'{wheel_err:6.3f}m')
    
    print(f'{"EKF Fusion":<20} '
          f'({ekf["x"].iloc[-1]:6.3f}, {ekf["y"].iloc[-1]:6.3f})m    '
          f'{ekf["theta_deg"].iloc[-1]:6.1f}°     '
          f'{ekf_err:6.3f}m')
    
    print(f'{"ICP Odometry":<20} '
          f'({icp["x"].iloc[-1]:6.3f}, {icp["y"].iloc[-1]:6.3f})m    '
          f'{icp["theta_deg"].iloc[-1]:6.1f}°     '
          f'{icp_err:6.3f}m')
    
    print(f'{"SLAM Toolbox":<20} '
          f'({slam["x"].iloc[-1]:6.3f}, {slam["y"].iloc[-1]:6.3f})m    '
          f'{slam["theta_deg"].iloc[-1]:6.1f}°     '
          f'{slam_err:6.3f}m')
    
    print('\n' + '='*70)
    print('📊 IMPROVEMENT ANALYSIS (vs Wheel Odometry)')
    print('='*70)
    
    ekf_improvement = ((wheel_err - ekf_err) / wheel_err) * 100
    icp_improvement = ((wheel_err - icp_err) / wheel_err) * 100
    slam_improvement = ((wheel_err - slam_err) / wheel_err) * 100
    
    print(f'  EKF Fusion:      {ekf_improvement:+6.1f}%')
    print(f'  ICP Odometry:    {icp_improvement:+6.1f}%')
    print(f'  SLAM Toolbox:    {slam_improvement:+6.1f}%  🏆')
    
    print('\n' + '='*70)
    print('🎯 METHOD COMPARISON')
    print('='*70)
    
    icp_vs_ekf = ((ekf_err - icp_err) / ekf_err) * 100
    slam_vs_icp = ((icp_err - slam_err) / icp_err) * 100
    slam_vs_ekf = ((ekf_err - slam_err) / ekf_err) * 100
    
    print(f'  ICP vs EKF:      {icp_vs_ekf:+6.1f}%')
    print(f'  SLAM vs ICP:     {slam_vs_icp:+6.1f}%')
    print(f'  SLAM vs EKF:     {slam_vs_ekf:+6.1f}%')
    
    print('\n' + '='*70)
    print('📈 RANKING (Best to Worst Loop Closure)')
    print('='*70)
    
    results = [
        ('SLAM Toolbox', slam_err),
        ('ICP Odometry', icp_err),
        ('EKF Fusion', ekf_err),
        ('Wheel Odometry', wheel_err)
    ]
    results.sort(key=lambda x: x[1])
    
    for i, (method, error) in enumerate(results, 1):
        medal = '🥇' if i == 1 else '🥈' if i == 2 else '🥉' if i == 3 else '  '
        print(f'  {medal} {i}. {method:<20} {error:6.3f}m')
    
    print('='*70)


if __name__ == '__main__':
    main()
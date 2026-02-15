#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
from pathlib import Path


def main():
    # Get results directory
    home = str(Path.home())
    results_dir = os.path.join(home, 'FRA532_Mobile_Robot_6619', 'results', 'phase3')
    
    # CSV file paths
    wheel_csv = os.path.join(results_dir, 'phase3_wheel_odom.csv')
    ekf_csv = os.path.join(results_dir, 'phase3_ekf_odom.csv')
    icp_csv = os.path.join(results_dir, 'phase3_icp_odom.csv')
    
    # Check files exist
    for fname, fpath in [('Wheel', wheel_csv), ('EKF', ekf_csv), ('ICP', icp_csv)]:
        if not os.path.exists(fpath):
            print(f'ERROR: {fname} file not found: {fpath}')
            print('Run record_phase3_data.py first to collect data.')
            return
    
    # Load data
    print('Loading data...')
    wheel = pd.read_csv(wheel_csv)
    ekf = pd.read_csv(ekf_csv)
    icp = pd.read_csv(icp_csv)
    
    print(f'Wheel odom: {len(wheel)} points')
    print(f'EKF odom: {len(ekf)} points')
    print(f'ICP odom: {len(icp)} points')
    
    # Convert theta to degrees
    wheel['theta_deg'] = np.rad2deg(wheel['theta'])
    ekf['theta_deg'] = np.rad2deg(ekf['theta'])
    icp['theta_deg'] = np.rad2deg(icp['theta'])
    
    # Normalize time to start from 0
    wheel['time'] = wheel['timestamp'] - wheel['timestamp'].iloc[0]
    ekf['time'] = ekf['timestamp'] - ekf['timestamp'].iloc[0]
    icp['time'] = icp['timestamp'] - icp['timestamp'].iloc[0]
    
    # Create plots
    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    
    # Plot 1: XY Trajectories
    axes[0, 0].plot(wheel['x'].values, wheel['y'].values, 'b-', 
                    linewidth=2, alpha=0.7, label='Wheel Odom')
    axes[0, 0].plot(ekf['x'].values, ekf['y'].values, 'r-', 
                    linewidth=2, alpha=0.7, label='EKF Fusion')
    axes[0, 0].plot(icp['x'].values, icp['y'].values, 'g-', 
                    linewidth=2, alpha=0.7, label='ICP')
    axes[0, 0].plot(0, 0, 'ko', markersize=10, label='Start', zorder=5)
    
    axes[0, 0].set_xlabel('X (m)', fontsize=11)
    axes[0, 0].set_ylabel('Y (m)', fontsize=11)
    axes[0, 0].set_title('Trajectory Comparison', fontsize=12, fontweight='bold')
    axes[0, 0].legend(fontsize=10)
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].axis('equal')
    
    # Plot 2: Heading Comparison
    axes[0, 1].plot(wheel['time'].values, wheel['theta_deg'].values, 'b-',
                    linewidth=2, alpha=0.7, label='Wheel Odom')
    axes[0, 1].plot(ekf['time'].values, ekf['theta_deg'].values, 'r-',
                    linewidth=2, alpha=0.7, label='EKF Fusion')
    axes[0, 1].plot(icp['time'].values, icp['theta_deg'].values, 'g-',
                    linewidth=2, alpha=0.7, label='ICP')
    
    axes[0, 1].set_xlabel('Time (s)', fontsize=11)
    axes[0, 1].set_ylabel('Heading (deg)', fontsize=11)
    axes[0, 1].set_title('Heading Comparison', fontsize=12, fontweight='bold')
    axes[0, 1].legend(fontsize=10)
    axes[0, 1].grid(True, alpha=0.3)
    
    # Plot 3: Loop Closure Error (distance from origin)
    wheel['loop_error'] = np.sqrt(wheel['x']**2 + wheel['y']**2)
    ekf['loop_error'] = np.sqrt(ekf['x']**2 + ekf['y']**2)
    icp['loop_error'] = np.sqrt(icp['x']**2 + icp['y']**2)
    
    axes[0, 2].plot(wheel['time'].values, wheel['loop_error'].values, 'b-',
                    linewidth=2, alpha=0.7, label='Wheel')
    axes[0, 2].plot(ekf['time'].values, ekf['loop_error'].values, 'r-',
                    linewidth=2, alpha=0.7, label='EKF')
    axes[0, 2].plot(icp['time'].values, icp['loop_error'].values, 'g-',
                    linewidth=2, alpha=0.7, label='ICP')
    
    axes[0, 2].set_xlabel('Time (s)', fontsize=11)
    axes[0, 2].set_ylabel('Distance from Origin (m)', fontsize=11)
    axes[0, 2].set_title('Loop Closure Error Over Time', fontsize=12, fontweight='bold')
    axes[0, 2].legend(fontsize=10)
    axes[0, 2].grid(True, alpha=0.3)
    
    # Plot 4: Wheel vs EKF Position Error
    min_len = min(len(wheel), len(ekf))
    if min_len > 10:
        pos_err_we = np.sqrt(
            (wheel['x'].values[:min_len] - ekf['x'].values[:min_len])**2 +
            (wheel['y'].values[:min_len] - ekf['y'].values[:min_len])**2
        )
        axes[1, 0].plot(wheel['time'].values[:min_len], pos_err_we, 'purple', linewidth=2)
        axes[1, 0].set_xlabel('Time (s)', fontsize=11)
        axes[1, 0].set_ylabel('Position Difference (m)', fontsize=11)
        axes[1, 0].set_title('Wheel vs EKF Position Error', fontsize=12, fontweight='bold')
        axes[1, 0].grid(True, alpha=0.3)
    
    # Plot 5: Wheel vs ICP Position Error
    min_len = min(len(wheel), len(icp))
    if min_len > 10:
        pos_err_wi = np.sqrt(
            (wheel['x'].values[:min_len] - icp['x'].values[:min_len])**2 +
            (wheel['y'].values[:min_len] - icp['y'].values[:min_len])**2
        )
        axes[1, 1].plot(wheel['time'].values[:min_len], pos_err_wi, 'orange', linewidth=2)
        axes[1, 1].set_xlabel('Time (s)', fontsize=11)
        axes[1, 1].set_ylabel('Position Difference (m)', fontsize=11)
        axes[1, 1].set_title('Wheel vs ICP Position Error', fontsize=12, fontweight='bold')
        axes[1, 1].grid(True, alpha=0.3)
    
    # Plot 6: EKF vs ICP Position Error
    min_len = min(len(ekf), len(icp))
    if min_len > 10:
        pos_err_ei = np.sqrt(
            (ekf['x'].values[:min_len] - icp['x'].values[:min_len])**2 +
            (ekf['y'].values[:min_len] - icp['y'].values[:min_len])**2
        )
        axes[1, 2].plot(ekf['time'].values[:min_len], pos_err_ei, 'cyan', linewidth=2)
        axes[1, 2].set_xlabel('Time (s)', fontsize=11)
        axes[1, 2].set_ylabel('Position Difference (m)', fontsize=11)
        axes[1, 2].set_title('EKF vs ICP Position Error', fontsize=12, fontweight='bold')
        axes[1, 2].grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    # Save plots
    png_path = os.path.join(results_dir, 'phase3_comparison.png')
    svg_path = os.path.join(results_dir, 'phase3_comparison.svg')
    
    fig.savefig(png_path, dpi=150, bbox_inches='tight')
    fig.savefig(svg_path, bbox_inches='tight')
    
    print(f'\nPlots saved:')
    print(f'  PNG: {png_path}')
    print(f'  SVG: {svg_path}')
    print('\n📊 Close the plot window to see results...\n')
    
    # Show plot (blocks until window closed)
    try:
        plt.show()
    except KeyboardInterrupt:
        print('\n\n⚠️  Plot interrupted')
    
    # ✅ Print results AFTER plot closes
    print('\n' + '='*60)
    print('PHASE 3: COMPARISON RESULTS')
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
    
    print(f'\nICP Localization:')
    print(f'  End: ({icp["x"].iloc[-1]:.3f}, {icp["y"].iloc[-1]:.3f})')
    print(f'  Heading: {icp["theta_deg"].iloc[-1]:.1f}°')
    icp_err = np.sqrt(icp['x'].iloc[-1]**2 + icp['y'].iloc[-1]**2)
    print(f'  Loop error: {icp_err:.3f}m')
    
    print(f'\n📈 Improvement vs Wheel:')
    ekf_improvement = ((wheel_err - ekf_err) / wheel_err) * 100
    icp_improvement = ((wheel_err - icp_err) / wheel_err) * 100
    print(f'  EKF: {ekf_improvement:+.1f}%')
    print(f'  ICP: {icp_improvement:+.1f}%')
    
    print(f'\n🎯 ICP vs EKF:')
    icp_vs_ekf = ((ekf_err - icp_err) / ekf_err) * 100
    print(f'  Improvement: {icp_vs_ekf:+.1f}%')
    
    print('='*60)


if __name__ == '__main__':
    main()
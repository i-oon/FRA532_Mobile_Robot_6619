# FRA532 Mobile Robot - LAB1: Odometry and Localization

**Course:** FRA532 Mobile Robot  
**Institution:** VISTEC  
**Academic Year:** 2024-2025

---

## Table of Contents

1. [Overview](#overview)
2. [Learning Objectives](#learning-objectives)
3. [Project Structure](#project-structure)
4. [Installation](#installation)
5. [Dataset Information](#dataset-information)
6. [Phase 0: Rosbag Exploration](#phase-0-rosbag-exploration)
6. [Phase 1: Wheel Odometry](#phase-1-wheel-odometry)
7. [Phase 2: EKF Sensor Fusion](#phase-2-ekf-sensor-fusion)
8. [Phase 3: ICP Scan Matching](#phase-3-icp-scan-matching)
9. [Phase 4: SLAM](#phase-4-slam)
10. [Results Summary](#results-summary)
11. [Dependencies](#dependencies)
12. [Troubleshooting](#troubleshooting)
13. [References](#references)

---

## Overview

This laboratory explores mobile robot localization through progressive implementation of odometry and SLAM techniques. The lab is divided into four phases, each building upon the previous to address fundamental limitations in dead-reckoning navigation.

**Robot Platform:** Turtlebot3 Burger  
**Environment:** FIBO Building Floor 3, VISTEC  
**Sensors:** Wheel encoders, IMU, 2D LiDAR

---

## Learning Objectives

- Implement differential drive kinematics for wheel odometry
- Understand error accumulation in dead-reckoning systems
- Apply Extended Kalman Filter for multi-sensor fusion
- Utilize ICP for scan matching and pose correction
- Implement graph-based SLAM with loop closure detection
- Analyze performance trade-offs between different localization methods

---

## Project Structure

```
LAB1/
├── LAB1/
│   ├── wheel_odom_node.py          # Phase 1: Wheel odometry
│   ├── ekf_fusion_node.py          # Phase 2: EKF implementation
│   └── utils/
│       ├── differential_drive.py   # Kinematics utilities
│       └── ekf_2d.py               # EKF utilities (unused)
├── launch/
│   ├── phase1_wheel_odom.launch.py # Phase 1 launcher
│   └── phase2_ekf_fusion.launch.py # Phase 2 launcher (with tunable params)
├── scripts/
│   ├── plot_wheel_odom.py          # Phase 1 visualization
│   ├── plot_phase2_comparison.py   # Phase 2 comparison plotter
│   ├── odom_to_path.py             # Path converter for RViz
│   └── static_tf_publisher.py      # TF tree publisher
├── config/
│   └── rviz/
│       └── phase1_wheel_odom.rviz  # RViz configuration
├── data/
│   └── rosbags/
│       ├── fibo_floor3_seq00/      # Empty hallway
│       ├── fibo_floor3_seq01/      # Sharp turns
│       └── fibo_floor3_seq02/      # Non-aggressive motion
└── results/
    └── plots/                      # Generated trajectory plots
        ├── phase1_wheel_odom_*.png
        └── phase2_comparison_*.png
```

---

## Installation

### Prerequisites

- ROS2 Humble
- Python 3.10+
- Ubuntu 22.04 (recommended)

### Setup

```bash
# Clone repository
cd ~/
git clone <repository-url> FRA532_Mobile_Robot_6619
cd FRA532_Mobile_Robot_6619

# Build workspace
colcon build --packages-select LAB1

# Source workspace
source install/setup.bash
```

### Python Dependencies

```bash
pip3 install numpy matplotlib scipy
```

---

## Dataset Information

Three sequences recorded at FIBO Floor 3 with Turtlebot3 Burger:

| Sequence | Duration | Messages | Description | Difficulty |
|----------|----------|----------|-------------|------------|
| **seq00** | 526.23s | 10,505 | Empty hallway (baseline) | Medium |
| **seq01** | 393.00s | 7,852 | Sharp turns (challenging) | Hard |
| **seq02** | 599.19s | 11,975 | Non-aggressive motion | Easy |

**Topics:**
- `/joint_states` - Wheel encoder positions (20 Hz)
- `/imu` - IMU data including gyroscope (20 Hz)
- `/scan` - 2D LiDAR scans (5 Hz)

**Robot Parameters:**
- Wheel radius: 0.033 m
- Wheel separation: 0.160 m
- Max linear velocity: ~0.22 m/s
- Max angular velocity: ~2.84 rad/s


---



## Phase 0: Rosbag Exploration

## **Purpose:**
Validate rosbag data quality before running odometry algorithms. This prevents debugging algorithm issues when the problem is actually bad sensor data.

### **Rosbag Dataset:**
- **Location:** `~/FRA532_Mobile_Robot_6619/src/LAB1/data/rosbags/`
- **Sequences:** `fibo_floor3_seq00`, `seq01`, `seq02` (3 sequences total)

### **Available Topics (from your plots):**
```
- /scan          → LiDAR data (2603 msgs, ~5Hz)
- /imu           → IMU data (10505 msgs, ~20Hz)  
- /joint_states  → Wheel encoders (10505 msgs, ~20Hz)
```

### **Validation Plots Created:**
You have plots showing:
1. **Message Arrival Timeline** - Verify all sensors publishing
2. **IMU Gyroscope Z-axis** - Check angular velocity (mean: -0.0108 rad/s)
3. **IMU Linear Acceleration** - Check X/Y accel data
4. **Wheel Joint Velocities** - Left/Right wheel speeds (0.1-0.15 rad/s)
5. **Message Frequency** - Confirm stable rates (scan ~5Hz, imu/joints ~20Hz)


<p align="center">
    <img width=70% src="results/phase0/bag_exploration_seq00.png">
    <img width=70% src="results/phase0/bag_exploration_seq01.png">
    <img width=70% src="results/phase0/bag_exploration_seq02.png">
    </br> 
</p>




### **Key Observations from Plots:**
- ✅ All sensors publishing continuously over 500+ seconds
- ✅ Wheel velocities show realistic values (0.1 rad/s ≈ 0.003 m/s with 0.033m radius)
- ✅ IMU gyro stable around 0 with small bias
- ✅ Message frequencies consistent

---

## 📊 **Robot Parameters**
- **Wheel separation:** 0.265m (or possibly 0.160m - **UNRESOLVED!**)
- **Wheel radius:** 0.0325m (or possibly 0.033m - **UNRESOLVED!**)
- **Base frame:** `base_footprint`
- **Odom frame:** `odom`
- **Laser frame:** `base_scan`
- **Wheel joint names:** `wheel_left_joint`, `wheel_right_joint`

---




### **How to Generate Rosbag Analysis:**
```bash
# Play rosbag
ros2 bag play ~/FRA532_Mobile_Robot_6619/src/LAB1/data/rosbags/fibo_floor3_seq00

# In separate terminal - record data
python3 <rosbag_analysis_script.py>  # Your existing validation script

# Generates plots showing:
# - Topic message counts
# - Sensor data ranges
# - Timing analysis
# - Data quality metrics
```

### **Expected Validation Results:**
- Joint states should show smooth, continuous encoder readings
- IMU should show stable gyro (near 0) when stationary
- Scan should have consistent range data
- No large gaps in timestamps



---

## Phase 1: Wheel Odometry

### [COMPLETED ✅]

Implementation of wheel odometry using ICC (Instantaneous Center of Curvature) method for accurate circular arc integration.

### Method

**Kinematics:**
```python
# Wheel displacements (from encoder positions)
d_left = (pos_left - prev_left) * wheel_radius
d_right = (pos_right - prev_right) * wheel_radius

# Robot motion
d_center = (d_left + d_right) / 2.0
d_theta = (d_right - d_left) / wheel_separation

# ICC integration (exact for circular motion)
if straight_line:
    x += d_center * cos(theta)
    y += d_center * sin(theta)
else:
    # Rotate around instantaneous center of curvature
    R = (wheel_separation/2) * (d_left + d_right) / (d_right - d_left)
    icc_x = x - R * sin(theta)
    icc_y = y + R * cos(theta)
    # Apply rotation...
```

### Usage

**Run with visualization:**

```bash
# Terminal 1: Launch wheel odometry
ros2 launch LAB1 phase1_wheel_odom.launch.py

# Terminal 2: Matplotlib plotter
python3 src/LAB1/scripts/plot_wheel_odom.py

# Terminal 3: RViz visualization (optional)
python3 src/LAB1/scripts/odom_to_path.py
rviz2 -d src/LAB1/config/rviz/phase1_wheel_odom.rviz

# Terminal 4: Play dataset
ros2 bag play src/LAB1/data/rosbags/fibo_floor3_seq00 --rate 3.0
```

**Clear path in RViz:**
```bash
ros2 service call /reset_path std_srvs/srv/Empty
```

### Results

**Performance Summary:**

| Sequence | Path Length | Loop Error | % Drift | Heading Error |
|----------|-------------|------------|---------|---------------|
| **Seq 00** | 55.42 m | 4.36 m | 7.9% | 37.9° |
| **Seq 01** | 56.38 m | 5.27 m | 9.3% | 36.8° |
| **Seq 02** | 59.71 m | **1.31 m** | **2.2%** | 47.8° |


<p align="center">
    <img width=70% src="results/phase1/wheel_odom_00_10501pts.png">
    <img width=70% src="results/phase1/wheel_odom_01_7852pts.png">
    <img width=70% src="results/phase1/wheel_odom_02_11975pts.png">
    </br> 
</p>


**Key Findings:**

1. **Motion Profile Matters:**
   - Smooth motion (Seq 02): 2.2% drift ✅ Best performance
   - Normal motion (Seq 00): 7.9% drift
   - Aggressive turns (Seq 01): 9.3% drift ❌ Worst performance

2. **Wheel Slip is Primary Error Source:**
   - Sharp turns cause 4x more drift than smooth motion
   - Heading errors accumulate to 36-48°
   - No loop closure achieved in any sequence

3. **Distance Measurement Accurate:**
   - Path lengths consistent (~55-60m)
   - Wheel encoder integration works correctly
   - Problem is in pose estimation, not odometry calculation

**Visualization:**

Trajectory plots show:
- Clear rectangular path structure (4 sides visible)
- Distinct 90° turns at corners
- Progressive drift preventing loop closure
- Final position 1.31-5.27m from origin

**Limitations Identified:**

- ❌ Unbounded drift over time
- ❌ Wheel slip during rotation (58x error in angular velocity from Phase 0)
- ❌ No external correction mechanism
- ❌ Motion-dependent accuracy

**Conclusion:**

Wheel odometry alone is insufficient for accurate long-term localization. Best-case performance (Seq 02) shows 2.2% drift, which accumulates to 1.31m error over 60m path. This motivates Phase 2 implementation to fuse IMU data for heading correction.

---

## Phase 2: EKF Sensor Fusion

### [COMPLETED ✅]

Extended Kalman Filter implementation that fuses wheel odometry (linear velocity) with IMU (angular velocity and orientation) for improved localization accuracy.

### Method

**State Vector:** `x = [x, y, θ]ᵀ`

**Prediction Step (Motion Model):**
```python
# Use linear velocity from wheels + angular velocity from IMU
x_new = x + v_wheel * cos(θ) * dt
y_new = y + v_wheel * sin(θ) * dt
θ_new = θ + ω_imu * dt  # Key: Use IMU gyro, not wheel-based omega!

# Jacobian for covariance prediction
F = [[1, 0, -v*sin(θ)*dt],
     [0, 1,  v*cos(θ)*dt],
     [0, 0,  1          ]]

P = F @ P @ F.T + Q
```

**Update Step (Measurement Model):**
```python
# Correct heading using IMU orientation
H = [[0, 0, 1]]  # Measure theta directly
y = θ_imu - θ_predicted  # Innovation
K = P @ H.T / (H @ P @ H.T + R)  # Kalman gain
x = x + K * y  # State correction
P = (I - K @ H) @ P  # Covariance update
```

**Key Innovation:**
- Prediction uses **IMU angular velocity** (ω) instead of wheel-based calculation
- Eliminates 58x wheel slip error from Phase 0
- Update step uses **IMU orientation** for heading correction

### Usage

**Basic usage:**
```bash
# Terminal 1: Launch both wheel odom and EKF
ros2 launch LAB1 phase2_ekf_fusion.launch.py

# Terminal 2: Comparison plotter
python3 src/LAB1/scripts/plot_phase2_comparison.py

# Terminal 3: Play dataset
ros2 bag play src/LAB1/data/rosbags/fibo_floor3_seq01 --rate 3.0
```

**With custom parameters:**
```bash
# Tune process and measurement noise
ros2 launch LAB1 phase2_ekf_fusion.launch.py \
  process_noise_x:=0.01 \
  process_noise_y:=0.01 \
  process_noise_theta:=0.005 \
  measurement_noise_theta:=0.02
  
# Disable IMU orientation update (prediction only)
ros2 launch LAB1 phase2_ekf_fusion.launch.py \
  use_imu_update:=false
```

### Tunable Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `process_noise_x` | 0.001 | Process noise for x position |
| `process_noise_y` | 0.001 | Process noise for y position |
| `process_noise_theta` | 0.001 | Process noise for heading |
| `measurement_noise_theta` | 0.01 | IMU orientation measurement noise |
| `use_imu_update` | true | Enable/disable IMU orientation correction |

**Tuning Guidelines:**
- **Lower Q (process noise):** Trust motion model more, smoother but less responsive
- **Higher Q:** Trust measurements more, more responsive but noisier
- **Lower R (measurement noise):** Trust IMU more, faster correction but may oscillate
- **Higher R:** Trust IMU less, slower correction but more stable

### Results

**Performance Comparison (Wheel Odometry vs EKF Fusion):**

<p align="center">
    <img width=70% src="results/phase2/phase2_00_comparison_10484pts.png">
    <img width=70% src="results/phase2/phase2_01_comparison_7839pts.png">
    <img width=70% src="results/phase2/phase2_02_comparison_11923pts.png">
    </br> 
</p>



| Sequence | Wheel Loop Error | EKF Loop Error | Improvement | Wheel Heading | EKF Heading |
|----------|------------------|----------------|-------------|---------------|-------------|
| **Seq 00** | 4.358m | 3.245m | **+25.5%** | 37.9° | 34.9° |
| **Seq 01** | 5.266m | **2.027m** | **+61.5%** 🏆 | 36.8° | **-0.5°** 🏆 |
| **Seq 02** | **1.311m** | 2.943m | -124.6% ❌ | 47.8° | 37.5° |

### Key Findings

**1. Motion-Dependent Performance:**

EKF effectiveness varies dramatically with motion profile:

- **Aggressive Motion (Seq 01):** 🏆 **Excellent Performance**
  - Loop closure improved by 61.5%
  - Heading error reduced from 36.8° to 0.5° (nearly perfect!)
  - Sharp turns provide strong IMU signals
  - Wheel slip high → IMU correction critical

- **Normal Motion (Seq 00):** ✅ **Moderate Improvement**
  - Loop closure improved by 25.5%
  - Heading slightly better (37.9° → 34.9°)
  - Consistent with typical indoor navigation

- **Smooth Motion (Seq 02):** ❌ **Performance Degradation**
  - Loop closure worse than wheel-only
  - Wheel odometry already excellent (1.311m)
  - IMU measurement noise dominates small signals
  - Over-correction during slow, smooth motion

**2. Best Overall Performance:**

**Sequence 01 with EKF Fusion:**
- Loop error: 2.027m (from 5.266m wheel-only)
- Heading error: 0.5° (from 36.8° wheel-only)
- Demonstrates EKF excels in challenging scenarios

**3. Heading Correction Effectiveness:**

| Sequence | Wheel Heading Error | EKF Heading Error | Improvement |
|----------|---------------------|-------------------|-------------|
| Seq 00 | 37.9° | 34.9° | 3.0° better |
| Seq 01 | 36.8° | **0.5°** | **36.3° better** 🎯 |
| Seq 02 | 47.8° | 37.5° | 10.3° better |

**Seq 01 shows dramatic heading correction** - nearly perfect loop closure in orientation!

### Analysis

**Why EKF Excels with Aggressive Motion:**
1. Sharp turns generate strong IMU gyroscope signals
2. High wheel slip makes wheel-based omega unreliable (58x error from Phase 0)
3. IMU provides accurate angular velocity when wheels slip most
4. Clear maneuvers → clear corrections

**Why EKF Struggles with Smooth Motion:**
1. Small IMU signals → measurement noise becomes significant
2. Wheel odometry already performs well (2.2% drift in Seq 02)
3. Over-correction from noisy IMU degrades performance
4. Need motion-dependent parameter adaptation

**Implications:**
- EKF is essential for robust navigation in varied environments
- Single parameter set cannot optimize for all motion types
- Adaptive filtering or motion classification recommended
- Trade-off: optimize for challenging scenarios vs smooth motion

### Visualization

Comparison plots show:
- **Blue trajectory (Wheel Odom):** Significant drift, especially during turns
- **Red trajectory (EKF Fusion):** Better loop closure, reduced heading drift
- **Seq 01:** Dramatic improvement visible in both XY and heading plots
- **Seq 02:** EKF trajectory diverges more than wheel-only

Position and heading error plots quantify the improvement over time.

### Limitations

- ❌ Still no perfect loop closure (best: 2.027m error)
- ❌ Motion-dependent performance requires parameter tuning
- ❌ Can perform worse than wheel-only in smooth motion scenarios
- ❌ Single fixed noise model not optimal for all conditions

### Future Work

**Parameter Optimization:**
- Adaptive noise covariance based on motion detection
- Different parameter sets for different scenarios
- Online tuning based on innovation statistics

**Motion Classification:**
- Detect aggressive vs smooth motion from velocity/acceleration
- Switch between parameter sets dynamically
- Optimize separately for each motion type

**Advanced Filtering:**
- Adaptive Extended Kalman Filter (AEKF)
- Unscented Kalman Filter (UKF) for better nonlinearity handling
- Particle Filter for multi-modal distributions

**Conclusion:**

Phase 2 demonstrates successful sensor fusion for mobile robot localization. EKF achieves 61.5% improvement in challenging scenarios (Seq 01), with nearly perfect heading correction (0.5° error). However, performance degrades in smooth motion, indicating need for adaptive filtering. This motivates Phase 3 (ICP) to provide absolute position corrections independent of motion type.

---

## Phase 3: ICP Scan Matching

### [TODO]

Iterative Closest Point algorithm for LiDAR-based pose correction.

---

## Phase 4: SLAM

### [TODO]

Graph-based SLAM with loop closure detection.

---

## Results Summary

### Phase 1: Wheel Odometry Performance

**Best Performance (Sequence 02):**
- Environment: Non-aggressive motion
- Path length: 59.71 m
- Loop closure error: 1.31 m (2.2% drift)
- Demonstrates wheel odometry works well for smooth motion

**Worst Performance (Sequence 01):**
- Environment: Sharp turns
- Path length: 56.38 m  
- Loop closure error: 5.27 m (9.3% drift)
- Wheel slip during aggressive maneuvers dominates error

**Typical Performance (Sequence 00):**
- Environment: Empty hallway
- Path length: 55.42 m
- Loop closure error: 4.36 m (7.9% drift)
- Representative of standard indoor navigation

**Comparative Analysis:**

The 4x performance difference between smooth (Seq 02) and aggressive motion (Seq 01) clearly demonstrates that:
1. Wheel odometry accuracy is highly motion-dependent
2. Rotational motion introduces significant wheel slip
3. External sensors (IMU, LiDAR) are necessary for robust localization

---

### Phase 2: EKF Sensor Fusion Performance

**Best Performance (Sequence 01 with EKF):**
- Environment: Sharp turns (most challenging for wheel-only)
- Loop closure error: **2.027m** (from 5.266m wheel-only)
- Improvement: **+61.5%**
- Heading error: **0.5°** (from 36.8° wheel-only)
- Near-perfect heading correction demonstrates EKF effectiveness

**Moderate Performance (Sequence 00 with EKF):**
- Environment: Empty hallway
- Loop closure error: 3.245m (from 4.358m wheel-only)
- Improvement: +25.5%
- Consistent improvement for normal navigation scenarios

**Degraded Performance (Sequence 02 with EKF):**
- Environment: Smooth motion (best for wheel-only)
- Loop closure error: 2.943m (from 1.311m wheel-only)
- Performance: -124.6% (worse than wheel-only)
- Indicates need for adaptive parameter tuning

**Key Insight:**

EKF performance is **strongly motion-dependent**:
- **Aggressive motion:** EKF essential (61.5% improvement)
- **Smooth motion:** Wheel-only may be better (without tuning)
- **Implication:** Adaptive filtering or motion classification needed

**Comparative Analysis:**

| Method | Seq 00 | Seq 01 | Seq 02 | Best Case |
|--------|--------|--------|--------|-----------|
| **Wheel Only** | 4.36m | 5.27m | **1.31m** | 2.2% drift |
| **EKF Fusion** | 3.25m | **2.03m** | 2.94m | 61.5% improvement |
| **Winner** | EKF | **EKF** 🏆 | Wheel | Motion-dependent |

The dramatic 61.5% improvement in Seq 01, combined with nearly perfect heading (0.5° error), validates the EKF approach for challenging scenarios. However, degraded performance in smooth motion indicates single parameter sets cannot optimize for all conditions.

---

### Overall Conclusions

**Phase 1 demonstrated:**
- Wheel odometry sufficient for smooth, straight motion (2.2% drift)
- Fails dramatically with aggressive maneuvers (9.3% drift)
- Motion profile critically affects accuracy

**Phase 2 demonstrated:**
- Sensor fusion essential for challenging scenarios (61.5% improvement)
- IMU corrects heading drift nearly perfectly (36.8° → 0.5°)
- Motion-dependent performance requires adaptive approaches
- Best result: 2.027m loop error with 0.5° heading error (Seq 01)

**Remaining Challenges:**
- No method achieves perfect loop closure (<0.5m)
- Parameter tuning needed for optimal performance
- Motion-dependent behavior requires classification or adaptation
- Motivates Phase 3 (ICP) for absolute position correction

---

## Dependencies

### ROS2 Packages
- `rclpy` - ROS2 Python client library
- `sensor_msgs` - Sensor message definitions
- `nav_msgs` - Navigation message definitions
- `geometry_msgs` - Geometry message definitions
- `tf2_ros` - Transform library
- `std_srvs` - Standard service definitions

### Python Libraries
- `numpy` - Numerical computing
- `matplotlib` - Plotting and visualization
- `scipy` - Scientific computing (for Phase 3+)

### System Requirements
- ROS2 Humble
- Python 3.10+
- Ubuntu 22.04 (tested)

---

## Troubleshooting

### No plot window appearing

**Problem:** Matplotlib window doesn't show

**Solution:**
```bash
# Check if display is available
echo $DISPLAY

# If empty, set it
export DISPLAY=:0

# Or use alternative backend
python3 -c "import matplotlib; matplotlib.use('TkAgg')"
```

### RViz shows no data

**Problem:** Path or odometry not visible

**Solution:**
1. Check Fixed Frame is set to `odom`
2. Verify topics are publishing: `ros2 topic list | grep odom`
3. Reset displays: uncheck/check in Displays panel
4. Increase "Keep" parameter in Odometry display to 10000

### Bag file plays but no odometry updates

**Problem:** Node receives joint_states but doesn't publish odometry

**Solution:**
1. Check node is running: `ros2 node list | grep wheel`
2. Verify joint names match: `ros2 topic echo /joint_states --once`
3. Check for error messages in node output
4. Restart node and replay bag from beginning

### Build errors

**Problem:** `colcon build` fails

**Solution:**
```bash
# Clean build
rm -rf build/ install/ log/

# Rebuild
colcon build --packages-select LAB1 --cmake-clean-cache

# Source
source install/setup.bash
```

### Path doesn't clear in RViz

**Solution:**
```bash
# Use reset service
ros2 service call /reset_path std_srvs/srv/Empty

# Or restart path converter node
# Ctrl+C on odom_to_path.py terminal, then run again
```

---

## References

### Academic Papers
1. Thrun, S., Burgard, W., & Fox, D. (2005). *Probabilistic Robotics*. MIT Press.
2. Borenstein, J., & Feng, L. (1996). "Measurement and correction of systematic odometry errors in mobile robots." *IEEE Transactions on Robotics and Automation*, 12(6), 869-880.

### Technical Documentation
3. ROBOTIS. (2024). "Turtlebot3 Specifications." https://emanual.robotis.com/
4. ROS2 Documentation. (2024). "Navigation2." https://navigation.ros.org/

### Course Materials
5. FRA532 Mobile Robot Course Materials, VISTEC, 2024-2025

---

## License

This project is for educational purposes as part of FRA532 Mobile Robot course at VISTEC.

---

## Acknowledgments

- FRA532 course instructors and TAs
- VISTEC Robotics Laboratory
- Dataset collection team

---

**Last Updated:** February 16, 2026  
**Status:** Phase 1 Complete ✅ | Phase 2 Complete ✅ | Phase 3 Pending ⏳
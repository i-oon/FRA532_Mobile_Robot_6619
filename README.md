# FRA532 Mobile Robot - LAB1: Odometry and Localization

**Course:** FRA532 Mobile Robot  
**Student:** Disthorn Suttwet 66340500019

---

## Table of Contents

1. [Overview](#overview)
2. [Learning Objectives](#learning-objectives)
3. [Project Structure](#project-structure)
4. [Installation](#installation)
5. [Dataset Information](#dataset-information)
6. [Phase 0: Rosbag Exploration](#phase-0-rosbag-exploration)
7. [Phase 1: Wheel Odometry](#phase-1-wheel-odometry)
8. [Phase 2: EKF Sensor Fusion](#phase-2-ekf-sensor-fusion)
9. [Phase 3: ICP Scan Matching](#phase-3-icp-scan-matching)
10. [Phase 4: SLAM Toolbox](#phase-4-slam-toolbox)
11. [Results Summary](#results-summary)
12. [Dependencies](#dependencies)
13. [Troubleshooting](#troubleshooting)
14. [References](#references)

---

## Overview

This LAB1 explores mobile robot localization through progressive implementation of odometry and SLAM techniques. The lab is divided into four phases, each building upon the previous to address fundamental limitations in dead-reckoning navigation.

**Robot Platform:** Turtlebot3 Burger  
**Environment:** FIBO Building Floor 3  
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
│   ├── ekf_fusion_node.py          # Phase 2: EKF sensor fusion
│   ├── icp_localization_node.py    # Phase 3: ICP scan matching
│   └── static_tf_publisher.py      # TF tree publisher
├── launch/
│   ├── phase1_wheel_odom.launch.py
│   ├── phase2_ekf_fusion.launch.py
│   ├── phase3_icp.launch.py
│   └── phase4_slam.launch.py
├── scripts/
│   ├── record_phase2_data.py       # Phase 2 data recorder
│   ├── record_phase3_data.py       # Phase 3 data recorder
│   ├── record_phase4_data.py       # Phase 4 data recorder
│   ├── plot_phase2_offline.py      # Phase 2 offline plotter
│   ├── plot_phase3_offline.py      # Phase 3 offline plotter
│   └── plot_phase4_offline.py      # Phase 4 offline plotter
├── config/
│   └── slam_toolbox_mapping.yaml   # SLAM Toolbox configuration
├── data/
│   └── rosbags/
│       ├── fibo_floor3_seq00/      # Empty hallway
│       ├── fibo_floor3_seq01/      # Sharp turns
│       └── fibo_floor3_seq02/      # Non-aggressive motion
└── results/
    ├── phase0/                     # Rosbag analysis
    ├── phase2/                     # EKF comparison
    ├── phase3/                     # ICP comparison
    └── phase4/                     # SLAM comparison
```

---

## Installation

### Prerequisites

- ROS2 Humble
- Python 3.10+
- Ubuntu 22.04 (recommended)
- SLAM Toolbox (`sudo apt install ros-humble-slam-toolbox`)

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
pip3 install numpy matplotlib scipy pandas --break-system-packages
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

### **Purpose**

Validate rosbag data quality before running odometry algorithms. This prevents debugging algorithm issues when the problem is actually bad sensor data.

### **Validation Checks**

**Data Quality Analysis:**
- Message arrival timeline (verify all sensors publishing)
- IMU gyroscope Z-axis (angular velocity, mean: -0.0108 rad/s)
- IMU linear acceleration (X/Y acceleration data)
- Wheel joint velocities (left/right wheel speeds, 0.1-0.15 rad/s)
- Message frequency stability (scan ~5Hz, IMU/joints ~20Hz)

### **Results**

<p align="center">
    <img width="70%" src="results/phase0/bag_exploration_seq00.png">
    <img width="70%" src="results/phase0/bag_exploration_seq01.png">
    <img width="70%" src="results/phase0/bag_exploration_seq02.png">
</p>

**Key Observations:**
- ✓ All sensors publishing continuously over 500+ seconds
- ✓ Wheel velocities show realistic values (0.1 rad/s ≈ 0.003 m/s)
- ✓ IMU gyro stable around 0 with small bias
- ✓ Message frequencies consistent (no dropouts)

---

## Phase 1: Wheel Odometry

### **Overview** [COMPLETED ✓]

Implementation of wheel odometry using ICC (Instantaneous Center of Curvature) method for accurate circular arc integration.

### **Method**

**Differential Drive Kinematics:**
```python
# Wheel displacements (from encoder positions)
d_left = (pos_left - prev_left) * wheel_radius
d_right = (pos_right - prev_right) * wheel_radius

# Robot motion
d_center = (d_left + d_right) / 2.0
d_theta = (d_right - d_left) / wheel_separation

# ICC integration (exact for circular motion)
if abs(d_theta) < 1e-6:  # Straight line
    x += d_center * cos(theta)
    y += d_center * sin(theta)
else:  # Circular arc
    R = (wheel_separation/2) * (d_left + d_right) / (d_right - d_left)
    icc_x = x - R * sin(theta)
    icc_y = y + R * cos(theta)
    # Rotation around ICC...
```

### **Usage**
```bash
# Terminal 1: Launch wheel odometry
ros2 launch LAB1 phase1_wheel_odom.launch.py

# Terminal 2: Play dataset
ros2 bag play src/LAB1/data/rosbags/fibo_floor3_seq01 --rate 2.0 --clock
```

### **Results**

**Performance Summary:**

| Sequence | Path Length | Loop Error | % Drift | Heading Error |
|----------|-------------|------------|---------|---------------|
| **Seq 00** | 55.42 m | 4.36 m | 7.9% | 37.9° |
| **Seq 01** | 56.38 m | 5.27 m | 9.3% | 36.8° |
| **Seq 02** | 59.71 m | **1.31 m** | **2.2%** | 47.8° |

<p align="center">
    <img width="70%" src="results/phase1/wheel_odom_00_10501pts.png">
    <img width="70%" src="results/phase1/wheel_odom_01_7852pts.png">
    <img width="70%" src="results/phase1/wheel_odom_02_11975pts.png">
</p>

**Key Findings:**

- **Motion Dependency:** Smooth motion (Seq 02) achieves 2.2% drift vs 9.3% for aggressive turns (Seq 01)
- **Error Source:** Wheel slip during rotation dominates error accumulation
- **Distance Accuracy:** Path length consistent (~55-60m), problem is in pose estimation
- **Limitation:** Unbounded drift prevents loop closure in all sequences

**Conclusion:** Wheel odometry sufficient for short-distance smooth motion, but requires external correction for long-term accuracy.

---

## Phase 2: EKF Sensor Fusion

### **Overview** [COMPLETED ✅]

Extended Kalman Filter implementation fusing wheel odometry (linear velocity) with IMU (angular velocity and orientation) for improved heading estimation.

### **Method**

**State Vector:** `x = [x, y, θ]ᵀ`

**Prediction Step (Motion Model):**
```python
# Use linear velocity from wheels + angular velocity from IMU
x_new = x + v_wheel * cos(θ) * dt
y_new = y + v_wheel * sin(θ) * dt
θ_new = θ + ω_imu * dt  # Key: Use IMU gyro, not wheel-based omega!

# Jacobian matrix
F = [[1, 0, -v*sin(θ)*dt],
     [0, 1,  v*cos(θ)*dt],
     [0, 0,  1          ]]

P = F @ P @ F.T + Q  # Covariance prediction
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

### **Usage**
```bash
# Terminal 1: Launch EKF fusion
ros2 launch LAB1 phase2_ekf_fusion.launch.py

# Terminal 2: Record data
python3 src/LAB1/scripts/record_phase2_data.py

# Terminal 3: Play dataset
ros2 bag play src/LAB1/data/rosbags/fibo_floor3_seq01 --rate 2.0 --clock

# After recording, plot offline
python3 src/LAB1/scripts/plot_phase2_offline.py
```

**Tunable Parameters:**
```bash
ros2 launch LAB1 phase2_ekf_fusion.launch.py \
  process_noise_x:=0.001 \
  process_noise_y:=0.001 \
  process_noise_theta:=0.001 \
  measurement_noise_theta:=0.01
```

### **Results**

**Performance Comparison:**

| Sequence | Wheel Error | EKF Error | Improvement | Wheel Heading | EKF Heading |
|----------|-------------|-----------|-------------|---------------|-------------|
| **Seq 00** | 4.358m | 3.245m | **+25.5%** | 37.9° | 34.9° |
| **Seq 01** | 5.266m | **2.027m** | **+61.5%** ✓ | 36.8° | **0.5°** ✓ |
| **Seq 02** | 1.311m | 2.943m | -124.6% | 47.8° | 37.5° |

<p align="center">
    <img width="70%" src="results/phase2/phase2_00_comparison_10484pts.png">
    </br> Sequence 00 - Normal Motion (EKF Normal)
    <img width="70%" src="results/phase2/phase2_01_comparison_7839pts.png">
    </br> Sequence 01 - Normal Motion (EKF Optimal)
    <img width="70%" src="results/phase2/phase2_02_comparison_11923pts.png">
    </br> Sequence 02 - Normal Motion (EKF Overshoot)
</p>

**Key Findings:**

- **Best Case (Seq 01):** 61.5% improvement with near-perfect heading (0.5° error)
- **Motion Dependency:** EKF excels with aggressive motion, struggles with smooth motion
- **Heading Correction:** Dramatic improvement in challenging scenarios (36.8° → 0.5°)
- **Trade-off:** Single parameter set cannot optimize for all motion types

**Conclusion:** EKF fusion essential for challenging scenarios with significant wheel slip. Demonstrates need for adaptive filtering based on motion profile.

---

## Phase 3: ICP Scan Matching

### **Overview** [COMPLETED ✅]

Scan-to-map ICP implementation revealing critical motion-dependency limitations despite excellent performance in favorable conditions.

### **Results - All Sequences**

**Performance Summary:**

| Sequence | Motion | Wheel | EKF | ICP | Best |
|----------|--------|-------|-----|-----|------|
| **00** | Normal | 4.358m | 3.245m | **1.020m** (+76.6%) | ICP 🏆 |
| **01** | Aggressive | 5.266m | 2.028m | **1.986m** (+62.3%) | ICP 🏆 |
| **02** | Smooth | **1.311m** | 2.943m | 5.589m (-326.4%) | Wheel 🏆 |

<p align="center">
    <img width="70%" src="results/phase3/00/phase3_comparison.png">
    <br><em>Sequence 00 - Normal Motion (ICP Optimal)</em>
</p>

<p align="center">
    <img width="70%" src="results/phase3/01/phase3_comparison.png">
    <br><em>Sequence 01 - Aggressive Turns (ICP and EKF Tied)</em>
</p>

<p align="center">
    <img width="70%" src="results/phase3/02/phase3_comparison.png">
    <br><em>Sequence 02 - Smooth Motion (ICP Catastrophic Failure)</em>
</p>

### **Key Findings**

**1. Motion-Dependent Performance (CRITICAL):**
- **Best case:** 1.020m (Seq 00, 76.6% improvement)
- **Worst case:** 5.589m (Seq 02, 326% degradation)
- **Performance variance:** 5.5x (worse than EKF's 1.4x)

**2. ICP Failure Modes:**
- Smooth motion causes catastrophic divergence
- Insufficient keyframes in gradual movement
- Repetitive geometry triggers local minima
- Cannot distinguish similar hallway sections

**3. Method Reliability Comparison:**

| Method | Mean Error | Std Dev | Best Case | Worst Case | Consistency |
|--------|-----------|---------|-----------|------------|-------------|
| **EKF** | 2.739m | ±0.48m | 2.028m | 3.245m | ✅ Excellent |
| **ICP** | 2.865m | ±2.09m | 1.020m | 5.589m | ❌ Poor |
| **Wheel** | 3.645m | ±1.73m | 1.311m | 5.266m | ⚠️ Moderate |

**4. Comparative Analysis:**
- **Seq 00:** ICP dominates (76.6% improvement)
- **Seq 01:** ICP and EKF tied (~2m, both excellent)
- **Seq 02:** ICP fails completely (5.6m, worst of all)

### **Technical Analysis**

**ICP Failure Root Causes (Seq 02):**

1. **Sparse Keyframes:**
   - Smooth motion triggers fewer keyframes
   - 0.2m threshold rarely met
   - Local map has insufficient coverage

2. **Feature Scarcity:**
   - Long straight hallways lack distinctive geometry
   - Similar-looking sections cause mismatches
   - ICP trapped in wrong correspondences

3. **Small Delta Problem:**
   - Gradual movements too small for reliable SVD
   - Numerical precision issues
   - Fails to converge correctly

4. **No Recovery Mechanism:**
   - Once diverged, no global correction
   - Local map forgets origin
   - Errors compound throughout sequence

### **Conclusions**

**ICP Scan Matching:**
- ✅ Excellent when motion is varied (1.0-2.0m)
- ✅ Best single-case performance (1.020m)
- ❌ Catastrophic failure on smooth motion (5.589m)
- ❌ Least consistent method (±2.09m variance)

**EKF Sensor Fusion:**
- ✅ Most consistent across all scenarios (±0.48m)
- ✅ Reliable 2.0-3.2m performance
- ✅ Best choice for unknown environments
- ❌ Cannot achieve <2m loop closure

**Critical Insight:** Neither ICP nor EKF provides reliable, motion-independent localization. ICP achieves best peak performance but worst reliability. EKF is most predictable but cannot close the loop. This strongly motivates Phase 4 SLAM with global optimization.

### **Limitations Motivating SLAM**

1. **No Global Loop Closure:** All methods drift (1.0-5.6m)
2. **Motion Dependency:** Performance unpredictable
3. **Local-Only Correction:** Cannot optimize full trajectory
4. **Map Forgetting:** ICP local map insufficient

**Phase 4 Goal:** <0.5m loop closure across all motion types with global pose graph optimization.

## Phase 4: SLAM Toolbox

### **Overview** [IN PROGRESS ⏳]

Full graph-based SLAM with loop closure detection using SLAM Toolbox for global pose optimization.

### **Method**

**Approach:** Graph-based SLAM with automatic loop closure detection

**Configuration:**
- Mode: Mapping (synchronous)
- Odometry input: EKF fusion (`/ekf_odom`)
- Loop closure: Enabled (search radius 4.0m)
- Keyframe threshold: 0.15m or 5°

### **Usage**
```bash
# Terminal 1: Launch SLAM Toolbox
ros2 launch LAB1 phase4_slam.launch.py

# Terminal 2: Record data
python3 src/LAB1/scripts/record_phase4_data.py

# Terminal 3: Play dataset
ros2 bag play src/LAB1/data/rosbags/fibo_floor3_seq01 --clock

# After recording, plot offline
python3 src/LAB1/scripts/plot_phase4_offline.py
```

### **Results**

**[RESULTS PENDING - Data collection in progress]**

Expected performance:
- Loop closure error: <0.5m (best performance with global optimization)
- Comparison with Wheel, EKF, ICP, and SLAM methods
- Map quality and loop closure effectiveness analysis

---

## Results Summary

### **Completed Phases**

**Phase 1: Wheel Odometry**
- Best: 2.2% drift (Seq 02, smooth motion)
- Worst: 9.3% drift (Seq 01, aggressive turns)
- Conclusion: Motion-dependent accuracy, unbounded error accumulation

**Phase 2: EKF Sensor Fusion**
- Best: 2.027m loop error (Seq 01, 61.5% improvement)
- Heading: 0.5° error (near-perfect correction)
- Conclusion: Essential for challenging scenarios, requires adaptive tuning

### **In Progress**

**Phase 3: ICP Scan Matching**
- Expected: ~1-2m loop closure error
- Status: Data collection and analysis in progress

**Phase 4: SLAM Toolbox**
- Expected: <0.5m loop closure error with global optimization
- Status: Implementation complete, testing in progress

---

## Dependencies

### ROS2 Packages
- `rclpy` - ROS2 Python client library
- `sensor_msgs` - Sensor message definitions
- `nav_msgs` - Navigation message definitions
- `geometry_msgs` - Geometry message definitions
- `tf2_ros` - Transform library
- `slam_toolbox` - Graph-based SLAM

### Python Libraries
- `numpy` - Numerical computing
- `matplotlib` - Plotting and visualization
- `pandas` - Data analysis
- `scipy` - Scientific computing (optional)

### System Requirements
- ROS2 Humble
- Python 3.10+
- Ubuntu 22.04 (tested)

---

## Troubleshooting

### Common Issues

**1. NumPy compatibility errors:**
```bash
pip install 'numpy<2' --break-system-packages
```

**2. QoS mismatch for LiDAR:**
- LiDAR uses `BEST_EFFORT` reliability
- Subscribers must match: `QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT)`

**3. CSV files not saving:**
- Add `.flush()` after each `writerow()` call
- Ensures data written to disk immediately

---

## References

### Academic Papers
1. Thrun, S., Burgard, W., & Fox, D. (2005). *Probabilistic Robotics*. MIT Press.
2. Borenstein, J., & Feng, L. (1996). "Measurement and correction of systematic odometry errors." IEEE Transactions on Robotics.

### Technical Documentation
3. ROBOTIS. (2024). "Turtlebot3 Specifications." https://emanual.robotis.com/
4. ROS2 Documentation. (2024). "SLAM Toolbox." https://github.com/SteveMacenski/slam_toolbox

---

## License

This project is for educational purposes as part of FRA532 Mobile Robot course at VISTEC.

---

**Last Updated:** February 16, 2026  
**Status:** Phase 1 ✅ | Phase 2 ✅ | Phase 3 ⏳ | Phase 4 ⏳
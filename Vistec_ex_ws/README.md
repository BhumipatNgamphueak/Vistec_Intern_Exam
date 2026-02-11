# Vistec_ex_ws: ROS 2 Gazebo Deployment Module

**Part of**: [Vistec_Intern_Exam](../README.md) Sim2Sim Research Project

---

## 📍 Module Overview

This module contains the **ROS 2 Gazebo deployment workspace** for transferring trained Isaac Lab policies to Gazebo Ignition.

**Full Repository Location**: `/home/drl-68/vistec_ex_ws/`

---

## 📦 Contents (Extracted Files)

This directory contains **key documentation** from the full workspace:

### Documentation Files

- **QUICKSTART.md** - Gazebo deployment guide
- **README_VISUALIZATION.md** - Visualization tools (14 KB)
- **DELIVERABLES.md** - Project deliverables checklist
- **collision_mesh_comparison.md** - Mesh validation
- **sim2sim_mismatch_analysis.md** - Isaac ↔ Gazebo comparison
- **TASK_COMPARISON.md** - Multi-task analysis
- **FK_VERIFICATION_REPORT.md** - Forward kinematics validation
- **HYBRID_ANALYSIS_SUMMARY.md** - Comprehensive analysis
- **FLOATING_GO2_GUIDE.md** - Floating base configuration
- **SKY_ATTACHMENT_GUIDE.md** - Sky attachment for hanging tests
- And more...

---

## 🚀 Quick Start

### 1. Navigate to Full Workspace

```bash
cd /home/drl-68/vistec_ex_ws/
source install/setup.bash
```

### 2. Read Deployment Guide

```bash
cat QUICKSTART.md
```

### 3. Launch Gazebo Simulation

```bash
# Launch Gazebo with GO2
ros2 launch go2_gazebo_simulation go2_empty_world.launch.py

# Or with policy
ros2 launch go2_bringup go2_rl_policy.launch.py \
    policy_path:=/path/to/policy.onnx \
    actuator_type:=mlp
```

### 4. Send Velocity Commands

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "linear: {x: 0.5, y: 0.0, z: 0.0}
     angular: {x: 0.0, y: 0.0, z: 0.0}" --rate 10
```

---

## 📁 Full Workspace Structure

Located at: `/home/drl-68/vistec_ex_ws/`

```
vistec_ex_ws/
├── src/
│   ├── go2_gazebo_simulation/      # Gazebo models, launch files
│   │   ├── models/                 # GO2 robot model
│   │   ├── worlds/                 # Gazebo worlds
│   │   └── launch/                 # Launch files
│   │
│   └── deploy_policy/              # ROS 2 policy deployment
│       ├── deploy_policy/          # Policy inference node
│       ├── launch/                 # Launch files
│       └── config/                 # Configuration files
│
├── scripts/                        # 100+ analysis scripts
│   ├── 1_start_gazebo.sh
│   ├── 2_deploy_policy.sh
│   ├── 3_collect_data.sh
│   ├── analysis/
│   ├── visualization/
│   └── comparison/
│
├── build/                          # Colcon build output
├── install/                        # Installed packages
└── log/                            # Build logs
```

---

## 🎯 Key Features

### 1. ROS 2 Integration
- **Middleware**: ROS 2 Humble
- **Control**: ros2_control framework
- **Bridge**: ros_gz_bridge for Gazebo communication
- **Frequency**: 50 Hz policy inference

### 2. Gazebo Simulation
- **Simulator**: Gazebo Ignition (Garden/Harmonic)
- **Physics**: DART or ODE solvers
- **Robot**: Unitree GO2 with accurate URDF
- **Sensors**: IMU, joint states, contact forces

### 3. Policy Deployment
- **Format**: ONNX or TorchScript (JIT)
- **Inference**: Real-time at 50 Hz
- **Actuators**: MLP, LSTM, or Implicit models
- **Commands**: Velocity commands (linear + angular)

### 4. Analysis & Validation
- **100+ Scripts**: Comprehensive sim2sim analysis
- **Metrics**: Position tracking, velocity errors, gait analysis
- **Visualization**: Generate figures for papers
- **Comparison**: Isaac Lab vs Gazebo performance

---

## 🔬 Validation Tools

### Chirp Tests (Motor Validation)

```bash
# Run chirp test in Gazebo
cd /home/drl-68/vistec_ex_ws/
ros2 launch go2_gazebo_simulation chirp_test.launch.py

# In another terminal
ros2 run go2_control chirp_test_node \
    --joint FR_hip_joint \
    --duration 10.0 \
    --f0 0.1 \
    --f1 20.0
```

### Data Collection

```bash
# Collect trajectory data
cd scripts/
./3_collect_data.sh

# Output: experiment_data/*.csv
```

### Visualization

```bash
# Generate comparison plots
python scripts/visualization/generate_figures.py

# Output: figures/*.png
```

---

## 📊 Performance Metrics

### Sim2Sim Transfer Results

| Metric | Target | Achieved |
|--------|--------|----------|
| Velocity Tracking | <0.1 m/s RMSE | ✅ 0.08 m/s |
| Gait Stability | Maintain trot | ✅ Stable |
| Recovery from Push | Robust | ✅ Good |
| Actuator Matching | <0.05 rad RMSE | ✅ 0.03 rad |

---

## 🛠️ Build Instructions

### Prerequisites

```bash
# ROS 2 Humble
source /opt/ros/humble/setup.bash

# Gazebo Ignition
sudo apt install ros-humble-ros-gz
```

### Build Workspace

```bash
cd /home/drl-68/vistec_ex_ws/

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source
source install/setup.bash
```

### Verify Build

```bash
# Check packages
ros2 pkg list | grep go2

# Launch RViz
ros2 launch go2_description display.launch.py
```

---

## 📚 Documentation

**In this directory**:
- **QUICKSTART.md** - Start here for deployment
- **README_VISUALIZATION.md** - Visualization tools
- **DELIVERABLES.md** - Project deliverables

**Related**:
- [Main README](../README.md) - Project overview
- [README_SIM2SIM.md](../README_SIM2SIM.md) - Complete Sim2Sim guide
- [unitree_rl_lab/](../unitree_rl_lab/) - Training module
- [Actuator_net/](../Actuator_net/) - Actuator models

---

## 🔗 Integration Points

### From Isaac Lab

1. Export trained policy to ONNX:
   ```bash
   cd /home/drl-68/unitree_rl_lab/
   ./play_any_policy.sh  # Automatically exports
   ```

2. Export parameters:
   ```bash
   python export_isaaclab_params.py --output gazebo_params.yaml
   ```

3. Copy to Gazebo workspace:
   ```bash
   cp logs/.../exported/policy.onnx /home/drl-68/vistec_ex_ws/policies/
   ```

### To Actuator Models

- Actuator type specified in launch file: `actuator_type:=mlp|lstm|implicit`
- Pre-trained models located in `Actuator_net/app/resources/`
- Chirp comparison validates actuator matching

---

## 🚨 Troubleshooting

### Gazebo Won't Launch

```bash
# Check Gazebo installation
gz sim --version

# Rebuild workspace
cd /home/drl-68/vistec_ex_ws/
colcon build --packages-select go2_gazebo_simulation
```

### Policy Not Loading

```bash
# Check ONNX file exists
ls /home/drl-68/vistec_ex_ws/policies/*.onnx

# Check policy node
ros2 node list | grep policy
```

### Robot Falls Immediately

1. Check PD gains match Isaac Lab (Kp=25.0, Kd=0.5)
2. Verify control frequency is 50 Hz
3. Run chirp test to validate actuators

**Full Troubleshooting**: [README_SIM2SIM.md](../README_SIM2SIM.md#-troubleshooting)

---

## 📧 Contact

**For Gazebo-specific issues**:
1. Check QUICKSTART.md
2. Review ROS 2 launch files in full workspace
3. Check analysis scripts for validation

**Related Documentation**:
- [Main Project README](../README.md)
- [Sim2Sim Guide](../README_SIM2SIM.md)

---

**Module**: Vistec_ex_ws (ROS 2 Gazebo Deployment)
**Part of**: Vistec Intern Exam Sim2Sim Research Project
**Last Updated**: 2026-02-11

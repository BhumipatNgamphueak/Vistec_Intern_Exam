# Vistec Intern Exam: Sim2Sim Quadruped Locomotion

**Unitree Go2 Locomotion: IsaacLab → Gazebo Transfer**

[![IsaacSim](https://img.shields.io/badge/IsaacSim-5.1.0-silver.svg)](https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html)
[![Isaac Lab](https://img.shields.io/badge/IsaacLab-2.3.0-silver)](https://isaac-sim.github.io/IsaacLab)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/index.html)
[![Gazebo](https://img.shields.io/badge/Gazebo-Ignition-orange)](https://gazebosim.org/)
[![Python](https://img.shields.io/badge/Python-3.10-blue)](https://www.python.org/)

This repository provides **essential configuration files, pre-trained models, and deployment scripts** for reproducing Sim2Sim transfer learning research on quadruped robots.

---

## 📋 Table of Contents

- [Overview](#-overview)
- [Repository Structure](#-repository-structure)
- [Quick Start](#-quick-start)
- [Training Pipeline](#-training-pipeline)
- [Deployment](#-deployment)
- [4 Locomotion Tasks](#-4-locomotion-tasks)
- [Key Features](#-key-features)
- [Troubleshooting](#-troubleshooting)
- [Environment Versions](#-environment-versions)

---

## 🎯 Overview

This repository enables you to:
- ✅ **Train** locomotion policies in Isaac Lab (PhysX simulator)
- ✅ **Deploy** policies to Gazebo (DART/ODE simulator) via ROS 2
- ✅ **Test** 4 fundamental locomotion tasks (standing, walking, turning, combined)
- ✅ **Compare** 3 actuator models (MLP, LSTM, Implicit)
- ✅ **Validate** Sim2Sim transfer with comprehensive domain randomization

**Key Research Contribution**: Successfully transfer policies trained in Isaac Lab to Gazebo with minimal performance degradation using 15 domain randomization strategies.

---

## 📁 Repository Structure

```
Vistec_Intern_Exam/
├── README.md                          # This comprehensive guide
├── verify_setup.sh                    # Setup verification script
│
├── send_velocity_commands_gazebo.sh   # Gazebo velocity control (14 options)
├── send_velocity_commands_isaac.py    # Isaac velocity presets (13 tasks)
│
├── trained_models/                    # Pre-trained policies (22 MB)
│   ├── mlp_with_dr_24999.pt          # ⭐ RECOMMENDED (MLP + DR)
│   ├── implicit_dr_latest.pt          # Implicit actuator + DR
│   ├── implicit_no_dr_latest.pt       # Implicit without DR
│   └── lstm_dr_25000.pt               # LSTM + DR
│
├── unitree_rl_lab/                    # ⭐ COMPLETE Isaac Lab framework
│   ├── source/                        # Framework source code
│   │   └── unitree_rl_lab/
│   │       └── unitree_rl_lab/
│   │           ├── assets/            # Robot definitions, actuator models
│   │           ├── tasks/             # Environment implementations (Go2, H1, G1)
│   │           └── utils/             # Core utilities
│   ├── scripts/                       # Training & testing scripts
│   │   ├── rsl_rl/                    # train.py, play.py
│   │   ├── actuator_comparison/       # Comparison tools
│   │   ├── data_collection/           # Data loggers
│   │   └── motor_testing/             # Motor tests
│   ├── Configs/                       # 12 custom task configs
│   ├── Utils/                         # Episode generators
│   ├── Testing_Scripts/               # Chirp tests
│   ├── Policy_Playback/               # Custom playback
│   ├── deploy/                        # Real robot deployment (C++)
│   ├── docker/                        # Docker setup
│   └── unitree_rl_lab.sh              # Setup script
│
├── Actuator_net/                      # Pre-trained actuator models
│   ├── train.py, train_lstm.py        # Training scripts
│   ├── test.py                        # Validation
│   └── app/resources/                 # 3 pre-trained models (598 KB)
│       ├── actuator_lstm.pth          # LSTM (R²=0.999)
│       ├── actuator.pth               # MLP (R²=0.998)
│       └── actuator_lstm_6input.pth   # 6-input variant
│
└── Vistec_ex_ws/                      # ROS 2 deployment workspace
    └── src/
        ├── deploy_policy/             # Policy inference node
        │   └── config/README_CONFIG.md
        └── go2_gazebo_simulation/     # Gazebo simulation setup
```

> **📌 IMPORTANT**: This repository is **SELF-CONTAINED** with everything you need:
> - ✅ Complete Isaac Lab framework (source/, scripts/)
> - ✅ Pre-trained policies (22 MB)
> - ✅ Actuator models (LSTM, MLP, Implicit)
> - ✅ 12 custom training configurations
> - ✅ ROS 2 deployment workspace
>
> **No additional cloning required!** Just clone and run.

---

## 🚀 Quick Start

### Step 1: Clone Repository

```bash
git clone https://github.com/BhumipatNgamphueak/Vistec_Intern_Exam.git
cd Vistec_Intern_Exam
```

### Step 2: Set Environment Variables

This repository uses environment variables for flexible paths:

```bash
# Set workspace paths (adjust to your clone location)
export VISTEC_REPO=~/Vistec_Intern_Exam                      # This repo
export UNITREE_LAB=~/Vistec_Intern_Exam/unitree_rl_lab       # Isaac Lab framework (inside repo)
export ACTUATOR_NET=~/Vistec_Intern_Exam/Actuator_net        # Actuator models (inside repo)
export VISTEC_WS=~/Vistec_Intern_Exam/Vistec_ex_ws           # ROS 2 workspace (inside repo)

# Make permanent
echo "export VISTEC_REPO=~/Vistec_Intern_Exam" >> ~/.bashrc
echo "export UNITREE_LAB=~/Vistec_Intern_Exam/unitree_rl_lab" >> ~/.bashrc
echo "export ACTUATOR_NET=~/Vistec_Intern_Exam/Actuator_net" >> ~/.bashrc
echo "export VISTEC_WS=~/Vistec_Intern_Exam/Vistec_ex_ws" >> ~/.bashrc
source ~/.bashrc
```

### Step 3: Install Prerequisites

```bash
# Isaac Lab 2.3.0 (for training)
cd ~
git clone https://github.com/isaac-sim/IsaacLab.git
cd IsaacLab && ./isaaclab.sh --install

# ROS 2 Humble (for Gazebo deployment)
sudo apt update && sudo apt install ros-humble-desktop
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Verify CUDA
nvidia-smi  # Should show CUDA 11.8+ or 12.1+
```

### Step 4: Verify Setup

```bash
cd $VISTEC_REPO
./verify_setup.sh
```

**Expected**: ✅ All checks passed (environment vars, directories, software)

---

## 🎓 Training Pipeline

### Stage 1: Install Isaac Lab Extension

```bash
# Navigate to unitree_rl_lab (already in the repo)
cd $UNITREE_LAB

# Install the extension into Isaac Lab
~/IsaacLab/isaaclab.sh -p -m pip install -e source/unitree_rl_lab
```

✅ **All configurations and models are already in place!**
- Custom configs: `source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/go2/`
- Actuator models: `source/unitree_rl_lab/unitree_rl_lab/assets/actuator_models/`

### Stage 2: Train Policy

```bash
cd $UNITREE_LAB

# Train MLP policy with domain randomization (RECOMMENDED)
# Time: 6-8 hours on RTX 3090
python scripts/rsl_rl/train.py \
    --task Unitree-Go2-Velocity-MLP-Custom \
    --num_envs 4096 \
    --headless

# Training produces: logs/rsl_rl/unitree_go2_velocity_mlp_custom/{timestamp}/model_*.pt
```

**Key Training Parameters**:
- **PD Gains**: Kp=25.0, Kd=0.5 (unified across all actuators)
- **Environments**: 4096 parallel
- **Iterations**: ~25,000 (6-8 hours)
- **Domain Randomization**: 15 strategies (mass, friction, forces, delays, etc.)

### Stage 3: Test & Export Policy

```bash
cd $UNITREE_LAB

# Test policy (automatically exports to ONNX/JIT)
python scripts/rsl_rl/play.py \
    --task Unitree-Go2-Velocity-MLP-Custom \
    --num_envs 32 \
    --load_run {timestamp}

# Exported to: logs/rsl_rl/.../exported/policy.onnx
```

**Camera Controls** (Isaac Sim):
- Mouse drag: Rotate view
- Mouse wheel: Zoom
- Middle click + drag: Pan

---

## 🎮 Deployment

### Option A: Isaac Lab (Visualization & Testing)

**Use pre-trained models from this repo**:

```bash
cd $UNITREE_LAB

# Create directory structure
mkdir -p logs/rsl_rl/unitree_go2_velocity_mlp_custom/pretrained

# Copy pre-trained model
cp $VISTEC_REPO/trained_models/mlp_with_dr_24999.pt \
   logs/rsl_rl/unitree_go2_velocity_mlp_custom/pretrained/

# Play policy
python scripts/rsl_rl/play.py \
    --task Unitree-Go2-Velocity-MLP-Custom \
    --num_envs 32 \
    --load_run pretrained \
    --checkpoint logs/rsl_rl/unitree_go2_velocity_mlp_custom/pretrained/mlp_with_dr_24999.pt
```

### Option B: Gazebo (Realistic Simulation with ROS 2)

#### Setup ROS 2 Workspace (First Time)

```bash
# Create workspace
mkdir -p $VISTEC_WS/src
cd $VISTEC_WS

# Copy ROS 2 packages from this repo
cp -r $VISTEC_REPO/Vistec_ex_ws/src/* src/

# Install dependencies and build
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash

# Add to bashrc
echo "source $VISTEC_WS/install/setup.bash" >> ~/.bashrc
```

#### Launch Gazebo with Policy

```bash
cd $VISTEC_WS
source install/setup.bash

# Launch with pre-trained MLP policy (RECOMMENDED)
ros2 launch deploy_policy go2_rl_policy.launch.py \
    policy_path:=$VISTEC_REPO/trained_models/mlp_with_dr_24999.pt \
    actuator_type:=mlp \
    use_gpu:=true
```

**Launch Arguments**:
- `policy_path`: Path to .pt or .onnx model
- `actuator_type`: `mlp`, `lstm`, or `implicit`
- `use_gpu`: `true` (CUDA) or `false` (CPU)

#### Send Velocity Commands

**Interactive Menu (14 options matching 4 training tasks)**:
```bash
cd $VISTEC_REPO
./send_velocity_commands_gazebo.sh
```

**Manual Command**:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "linear: {x: 1.0, y: 0.0, z: 0.0}
     angular: {x: 0.0, y: 0.0, z: 0.0}" --rate 10
```

---

## 🎯 4 Locomotion Tasks

The robot is trained and tested on **4 fundamental locomotion primitives**:

### Task 1: Standing
**Goal**: Maintain stable standing position

| Variant | lin_x | lin_y | ang_z | Gazebo Option | Isaac Preset |
|---------|-------|-------|-------|---------------|--------------|
| Stand still | 0.0 | 0.0 | 0.0 | Option 1 | `--task 1` |

### Task 2: Walking
**Goal**: Walk forward at varying speeds

| Variant | lin_x (m/s) | lin_y | ang_z | Gazebo Option | Isaac Preset |
|---------|-------------|-------|-------|---------------|--------------|
| Slow | 0.5 | 0.0 | 0.0 | Option 2 | `--task 2.1` |
| Normal | 1.0 | 0.0 | 0.0 | Option 3 | `--task 2.2` ⭐ |
| Fast | 1.5 | 0.0 | 0.0 | Option 4 | `--task 2.3` |
| Moderate | 0.8 | 0.0 | 0.0 | Option 5 | `--task 2.4` |

### Task 3: Turn in Place
**Goal**: Rotate without forward motion

| Variant | lin_x | lin_y | ang_z (rad/s) | Gazebo Option | Isaac Preset |
|---------|-------|-------|---------------|---------------|--------------|
| Slow CCW | 0.0 | 0.0 | +0.5 | Option 6 | `--task 3.1` |
| Normal CCW | 0.0 | 0.0 | +1.0 | Option 7 | `--task 3.2` |
| Normal CW | 0.0 | 0.0 | -1.0 | Option 8 | `--task 3.3` |
| Fast CCW | 0.0 | 0.0 | +1.5 | Option 9 | `--task 3.4` |

### Task 4: Walk + Turn (Combined Maneuvers)
**Goal**: Execute combined forward and rotational motion

| Variant | Description | lin_x (m/s) | ang_z (rad/s) | Gazebo Option | Isaac Preset |
|---------|-------------|-------------|---------------|---------------|--------------|
| Right arc | Arc to right | 0.8 | +0.6 | Option 10 | `--task 4.1` |
| Straight fast | Fast forward | 1.2 | 0.0 | Option 11 | `--task 4.2` |
| Left arc | Arc to left | 0.8 | -0.6 | Option 12 | `--task 4.3` |
| Tight turn | Circle walk | 0.5 | +1.0 | Option 13 | `--task 4.4` |

### Using Task Presets

**Gazebo**:
```bash
./send_velocity_commands_gazebo.sh
# Select option 3 for Task 2.2 (Walk normal - 1.0 m/s)
# Select option 10 for Task 4.1 (Right arc)
```

**Isaac Lab**:
```bash
# List all tasks
python send_velocity_commands_isaac.py --list

# Test specific task
python send_velocity_commands_isaac.py --task 2.2  # Walk normal
python send_velocity_commands_isaac.py --task 4.1  # Right arc

# Custom command
python send_velocity_commands_isaac.py --linear_x 0.6 --angular_z 0.4
```

**Note**: Isaac Lab requires modifying config file to fix velocities. See script output for instructions.

---

## ✨ Key Features

### 1. Three Actuator Models

| Actuator | Type | Parameters | R² Score | Use Case |
|----------|------|------------|----------|----------|
| **MLP** | Feedforward NN | 137 KB | 0.998 | **Best for Gazebo transfer** |
| **LSTM** | Recurrent NN | 226 KB | 0.999 | Highest accuracy, temporal dynamics |
| **Implicit** | Physics-based | N/A | N/A | Baseline comparison |

### 2. Comprehensive Domain Randomization (15 Strategies)

Applied in MLP-Custom config:

1. **Physics Material**: Friction (0.4-1.4), restitution (0-0.2)
2. **Base Mass**: -1 to +3 kg variation
3. **COM Position**: ±2cm per link (critical for Gazebo URDF matching)
4. **Joint Reset Position**: ±60° randomization
5. **Motor Strength**: Kp ±25%, Kd ±50%
6. **Joint Friction**: 0-0.15 Nm
7. **Joint Armature**: 0-0.015 kg·m²
8. **Velocity Push**: ±1.0 m/s every 5-10s
9. **Force Impulses**: ±10N every 3-8s
10. **Torque Impulses**: ±3Nm every 3-8s
11. **Action Latency**: 0-2 steps (simulates ROS 2 delays)
12. **Observation Noise**: IMU + encoder noise
13. **Velocity Limits**: 85% of nominal (Gazebo safety margin)
14. **Spawn Position**: 0-0.3m height variation
15. **Terrain Variation**: Ground irregularities

### 3. Unified PD Gains

**All actuators use identical gains**:
- Kp = 25.0 (Stiffness)
- Kd = 0.5 (Damping)

This simplifies deployment and ensures fair actuator comparison.

### 4. Pre-trained Models Included

| Model | Iterations | Size | Actuator | DR | Status |
|-------|------------|------|----------|----|---------|
| **mlp_with_dr_24999.pt** | 24,999 | 4.4 MB | MLP | ✅ | ⭐ **RECOMMENDED** |
| implicit_dr_latest.pt | ~18,400 | 4.4 MB | Implicit | ✅ | Ready |
| implicit_no_dr_latest.pt | ~12,100 | 4.4 MB | Implicit | ❌ | Ablation study |
| lstm_dr_25000.pt | 25,000 | 4.4 MB | LSTM | ✅ | Research (obs issues) |

---

## 🛠️ Troubleshooting

### Issue 1: Environment Variables Not Set

```bash
# Check if set
echo $VISTEC_REPO $UNITREE_LAB $VISTEC_WS

# If empty, set them
export VISTEC_REPO=~/Vistec_Intern_Exam
export UNITREE_LAB=~/unitree_rl_lab
export VISTEC_WS=~/vistec_ex_ws
```

### Issue 2: verify_setup.sh Fails

```bash
# Make executable
chmod +x verify_setup.sh

# Run with verbose output
bash -x verify_setup.sh

# Check specific components
ls $UNITREE_LAB  # Should show: source/, scripts/, logs/
ls $VISTEC_REPO/trained_models  # Should show: 4 .pt files
```

### Issue 3: Config File Not Found (Training)

```bash
# Copy config to training repo
cp $VISTEC_REPO/unitree_rl_lab/Configs/velocity_env_cfg_mlp_custom.py \
   $UNITREE_LAB/source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/go2/
```

### Issue 4: Hardcoded Paths in Configs

**Problem**: Some configs have `/home/drl-68/` hardcoded paths

**Solution 1** (Recommended): Use configs without hardcoded paths
- ✅ `velocity_env_cfg_mlp_custom.py` - Safe
- ✅ `velocity_env_cfg_implicit_with_dr.py` - Safe
- ✅ `velocity_env_cfg_lstm_with_dr.py` - Safe
- ⚠️ `velocity_env_cfg_lstm_my_model.py` - Has hardcoded paths

**Solution 2**: Fix paths with sed
```bash
cd $VISTEC_REPO/unitree_rl_lab/Configs
sed -i "s|/home/drl-68|$HOME|g" velocity_env_cfg_lstm_my_model.py
```

**Solution 3**: See `unitree_rl_lab/Configs/README_PATHS.md` for details

### Issue 5: ROS 2 Workspace Build Fails

```bash
cd $VISTEC_WS

# Clean and rebuild
rm -rf build install log
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install

# Source
source install/setup.bash
```

### Issue 6: Gazebo Robot Falls

**Cause**: PD gains mismatch or wrong actuator type

**Solution**:
1. Verify using MLP actuator: `actuator_type:=mlp`
2. Check PD gains in URDF (should be Kp=25.0, Kd=0.5)
3. Use MLP-Custom policy (best DR for Gazebo)

```bash
# Check controller parameters
ros2 param list /controller_manager
ros2 param get /controller_manager use_sim_time
```

### Issue 7: CUDA Not Found

```bash
# Check CUDA
nvidia-smi
nvcc --version

# If missing, install CUDA 11.8 or 12.1
# https://developer.nvidia.com/cuda-downloads
```

---

## 📊 Expected Results

### Training Metrics (MLP @ 25K iterations)

- Mean Episode Reward: 180-220
- Success Rate: >95%
- Velocity Tracking RMSE: <0.1 m/s
- Training Time: 6-8 hours (RTX 3090, 4096 envs)

### Sim2Sim Transfer (Isaac Lab → Gazebo)

| Metric | Isaac Lab (PhysX) | Gazebo (DART/ODE) | Transfer Gap |
|--------|-------------------|-------------------|--------------|
| Velocity Tracking | <0.1 m/s RMSE | <0.15 m/s RMSE | +50% |
| Gait Stability | ✅ Smooth trot | ✅ Maintained | Similar |
| Recovery from Push | ✅ Robust | ✅ Good with DR | Comparable |
| Actuator RMSE | 0.03 rad (MLP) | N/A | - |

---

## 📚 Environment Versions

| Software | Version | Notes |
|----------|---------|-------|
| **Python** | 3.10.12 | Required for Isaac Lab |
| **CUDA** | 11.8 / 12.1 | Match PyTorch version |
| **Isaac Sim** | 5.1.0 | Physics simulator |
| **Isaac Lab** | 2.3.0 | RL training framework |
| **ROS 2** | Humble | Middleware for Gazebo |
| **Gazebo** | Ignition | Deployment simulator |
| **Ubuntu** | 22.04 | Operating system |
| **GPU** | RTX 3090 | Recommended (any RTX works) |

---

## 🔗 Related Resources

### Full Repositories

This repo contains **essential files only**. Full source code:

| Module | This Repo | Full Repository | Env Variable |
|--------|-----------|-----------------|--------------|
| **unitree_rl_lab** | Configs, utils | Clone from source | `$UNITREE_LAB` |
| **Actuator_net** | Pre-trained models | Clone from source | `$ACTUATOR_NET` |
| **Vistec_ex_ws** | ROS 2 packages | Build from this repo | `$VISTEC_WS` |

### Helper Documentation

- `unitree_rl_lab/Configs/README_PATHS.md` - Config path fixes
- `Vistec_ex_ws/src/deploy_policy/config/README_CONFIG.md` - ROS 2 config guide
- `verify_setup.sh` - Automated setup verification

---

## 📧 Support

**For issues or questions**:

1. ✅ Run `./verify_setup.sh` to check setup
2. ✅ Check environment variables: `echo $VISTEC_REPO`
3. ✅ Review [Troubleshooting](#-troubleshooting) section
4. ✅ Verify system requirements (Ubuntu 22.04, CUDA 11.8+, RTX GPU)

**GitHub Repository**: https://github.com/BhumipatNgamphueak/Vistec_Intern_Exam

---

## 📄 License

BSD-3-Clause License

---

<div align="center">

**Sim2Sim Quadruped Locomotion Research**

**Last Updated**: 2026-02-11
**Isaac Lab**: 2.3.0 | **ROS 2**: Humble | **Tested**: Ubuntu 22.04, RTX 3090

**Clone & Get Started**: `git clone https://github.com/BhumipatNgamphueak/Vistec_Intern_Exam.git`

</div>

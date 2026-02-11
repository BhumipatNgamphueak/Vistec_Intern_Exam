# Vistec Intern Exam: Sim2Sim Quadruped Locomotion

**Unitree Go2 Locomotion: IsaacLab → Gazebo Transfer**

[![IsaacSim](https://img.shields.io/badge/IsaacSim-5.1.0-silver.svg)](https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html)
[![Isaac Lab](https://img.shields.io/badge/IsaacLab-2.3.0-silver)](https://isaac-sim.github.io/IsaacLab)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/index.html)
[![Gazebo](https://img.shields.io/badge/Gazebo-Ignition-orange)](https://gazebosim.org/)
[![Python](https://img.shields.io/badge/Python-3.10-blue)](https://www.python.org/)

**Complete, self-contained Sim2Sim transfer learning package**. Train in Isaac Lab, deploy to Gazebo. No additional repos needed!

---

## 📋 Quick Navigation

| Section | Description |
|---------|-------------|
| [Quick Start](#-quick-start) | Clone to running policies in 30 min |
| [Pre-trained Models](#-pre-trained-models) | 6 ready-to-use policies (27 MB) |
| [Gazebo Deployment](#-deployment) | 3-terminal workflow |
| [Training Pipeline](#-training-pipeline) | Train your own policies |
| [4 Locomotion Tasks](#-4-locomotion-tasks) | Stand, Walk, Turn, Combined |
| [Velocity Commands](#-velocity-commands-guide) | Simplified & detailed testing |
| [Troubleshooting](#-troubleshooting) | Common issues & solutions |
| [Task Names](#-registered-task-names) | All registered Isaac Lab tasks |

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
├── README.md                          # This complete guide
│
├── verify_setup.sh                    # Setup verification script
├── send_velocity_commands_gazebo.sh   # Gazebo velocity control (14 options)
├── send_velocity_commands_isaac.py    # Isaac velocity presets (4 tasks + variants)
│
├── trained_models/                    # Pre-trained policies (27 MB)
│   ├── mlp_dr.pt                      # ⭐ RECOMMENDED (MLP + DR)
│   ├── mlp.pt                         # MLP without DR
│   ├── lstm_dr.pt                     # LSTM + DR
│   ├── lstm.pt                        # LSTM without DR
│   ├── Implicit_dr.pt                 # Implicit actuator + DR
│   └── implicit.pt                    # Implicit without DR
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
> - ✅ Pre-trained policies (27 MB - 6 models)
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

## 🎯 Pre-trained Models

All models are in `trained_models/` directory (27 MB total):

| Model File | Size | Actuator | Domain Randomization | Status |
|------------|------|----------|---------------------|---------|
| **mlp_dr.pt** | 4.4 MB | MLP | ✅ Yes | ⭐ **RECOMMENDED** |
| mlp.pt | 4.4 MB | MLP | ❌ No | Baseline comparison |
| lstm_dr.pt | 4.4 MB | LSTM | ✅ Yes | Research use |
| lstm.pt | 4.4 MB | LSTM | ❌ No | Ablation study |
| Implicit_dr.pt | 4.4 MB | Implicit | ✅ Yes | Physics-based |
| implicit.pt | 4.4 MB | Implicit | ❌ No | Baseline |

### Quick Test Commands

```bash
# Activate Isaac Lab environment (if using conda)
conda activate env_isaaclab  # Skip if not using conda

cd $UNITREE_LAB

# 1. MLP with DR (RECOMMENDED - Best Sim2Sim transfer)
~/IsaacLab/isaaclab.sh -p scripts/rsl_rl/play.py \
  --task Unitree-Go2-Velocity-MLP-Custom \
  --checkpoint $VISTEC_REPO/trained_models/mlp_dr.pt \
  --num_envs 4

# 2. LSTM with DR
~/IsaacLab/isaaclab.sh -p scripts/rsl_rl/play.py \
  --task Unitree-Go2-Velocity-LSTM-DR \
  --checkpoint $VISTEC_REPO/trained_models/lstm_dr.pt \
  --num_envs 4

# 3. Implicit with DR
~/IsaacLab/isaaclab.sh -p scripts/rsl_rl/play.py \
  --task Unitree-Go2-Velocity-Implicit-DR \
  --checkpoint $VISTEC_REPO/trained_models/Implicit_dr.pt \
  --num_envs 4
```

---

## ✅ Registered Task Names

Based on `__init__.py`, here are the **CORRECT** task names:

### MLP Actuator Tasks
1. `Unitree-Go2-Velocity-MLP-Custom` - MLP with Domain Randomization ⭐ **RECOMMENDED**
2. `Unitree-Go2-Velocity-MLP-No-DR` - MLP without Domain Randomization

### LSTM Actuator Tasks
3. `Unitree-Go2-Velocity-LSTM-DR` - LSTM with Domain Randomization
4. `Unitree-Go2-Velocity-LSTM-No-DR` - LSTM without Domain Randomization

### Implicit Actuator Tasks
5. `Unitree-Go2-Velocity-Implicit-DR` - Implicit with Domain Randomization
6. `Unitree-Go2-Velocity-Implicit` - Implicit without Domain Randomization

### Base Task
7. `Unitree-Go2-Velocity` - Base configuration

### Task Name Pattern

```
Unitree-Go2-Velocity-[ACTUATOR]-[DR_STATUS]
```

Where:
- `[ACTUATOR]` = MLP-Custom, MLP, LSTM, Implicit
- `[DR_STATUS]` = DR, No-DR, Custom, or omitted for base

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
# Activate Isaac Lab environment (if using conda)
conda activate env_isaaclab # Skip if not using conda

cd $UNITREE_LAB

# Train MLP policy with domain randomization (RECOMMENDED)
# Time: 6-8 hours on RTX 3090
~/IsaacLab/isaaclab.sh -p scripts/rsl_rl/train.py \
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
# Activate Isaac Lab environment (if using conda)
conda activate env_isaaclab   # Skip if not using conda

cd $UNITREE_LAB

# Test policy (automatically exports to ONNX/JIT)
~/IsaacLab/isaaclab.sh -p scripts/rsl_rl/play.py \
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
# Activate Isaac Lab environment (if using conda)
conda activate env_isaaclab   # Skip if not using conda

cd $UNITREE_LAB

# Create directory structure
mkdir -p logs/rsl_rl/unitree_go2_velocity_mlp_custom/pretrained

# Copy pre-trained model
cp $VISTEC_REPO/trained_models/mlp_dr.pt \
   logs/rsl_rl/unitree_go2_velocity_mlp_custom/pretrained/

# Play policy
~/IsaacLab/isaaclab.sh -p scripts/rsl_rl/play.py \
  --task Unitree-Go2-Velocity-MLP-Custom \
  --num_envs 32 \
  --load_run pretrained \
  --checkpoint logs/rsl_rl/unitree_go2_velocity_mlp_custom/pretrained/mlp_dr.pt
```

### Option B: Gazebo (Realistic Simulation with ROS 2)

#### Setup ROS 2 Workspace (First Time)

```bash
cd $VISTEC_WS

# Install dependencies and build
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash

# Add to bashrc
echo "source $VISTEC_WS/install/setup.bash" >> ~/.bashrc
```

#### Launch Gazebo with Policy (3 Terminals)

**Step 1: Deploy Policy (Terminal 2)**

```bash
# Deactivate conda if active
conda deactivate

cd $VISTEC_WS
source install/setup.bash

# Deploy pre-trained MLP policy (RECOMMENDED)
ros2 launch deploy_policy go2_deploy.launch.py \
  policy_path:=$VISTEC_REPO/trained_models/mlp_dr.pt \
  device:=cpu
```

**Step 2: Launch Gazebo Fortress (Terminal 1)**

```bash
# Deactivate conda if active
conda deactivate

cd $VISTEC_WS
source install/setup.bash

# Launch Gazebo with Go2 robot
ros2 launch go2_gazebo_simulation go2_fortress.launch.py
```

**Expected**: Gazebo opens with Go2 robot spawned

**Launch Arguments**:
- `policy_path`: Path to .pt or .onnx model
- `device`: `cpu` or `cuda` (default: cpu, use `cuda` if GPU available)

**Step 3: Send Velocity Commands (Terminal 3)**

**Interactive Menu (14 options for 4 training tasks)**:
```bash
cd $VISTEC_REPO
./send_velocity_commands_gazebo.sh

# Select an option:
# Option 1  - Task 1: Standing
# Option 3  - Task 2: Walking (1.0 m/s)
# Option 7  - Task 3: Turn in Place (1.0 rad/s CCW)
# Option 10 - Task 4: Walk + Turn (arc)
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

| Task | Description | lin_x (m/s) | lin_y (m/s) | ang_z (rad/s) | Isaac Preset | Gazebo Menu |
|------|-------------|-------------|-------------|---------------|--------------|-------------|
| **1** | Standing | 0.0 | 0.0 | 0.0 | `--task 1` | Option 1 |
| **2** | Walking (forward) | 1.0 | 0.0 | 0.0 | `--task 2` | Option 3 |
| **3** | Turn in Place (CCW) | 0.0 | 0.0 | 1.0 | `--task 3` | Option 7 |
| **4** | Walk + Turn (arc) | 0.8 | 0.0 | 0.6 | `--task 4` | Option 10 |

**Notes**:
- Task 2 supports varying speeds (0.5-1.5 m/s) - adjust with `--linear_x` parameter
- Task 3 supports different turn rates and directions - adjust with `--angular_z` parameter
- Task 4 supports various arcs and combined maneuvers - adjust with both parameters
- For detailed variants, use `--task 2.1`, `--task 2.2`, etc. (see `--list` for all options)

---

## 📊 Velocity Commands Guide

You have **TWO ways** to test the 4 locomotion tasks:

### 🎯 SIMPLIFIED (4 Commands - Quick Testing)

Perfect for quick testing of each main task category.

**Isaac Lab**:
```bash
python send_velocity_commands_isaac.py --task 1   # Task 1: Standing
python send_velocity_commands_isaac.py --task 2   # Task 2: Walking (1.0 m/s)
python send_velocity_commands_isaac.py --task 3   # Task 3: Turning (1.0 rad/s)
python send_velocity_commands_isaac.py --task 4   # Task 4: Walk+Turn (arc)
```

**Gazebo**:
```bash
./send_velocity_commands_gazebo.sh
# Quick select: 1, 3, 7, or 10
```

| Option | Task | Command |
|--------|------|---------|
| 1 | Standing | (0.0, 0.0, 0.0) |
| 3 | Walk normal | (1.0, 0.0, 0.0) |
| 7 | Turn normal | (0.0, 0.0, 1.0) |
| 10 | Walk+Turn | (0.8, 0.0, 0.6) |

### 📋 DETAILED (17 Variants - Comprehensive Testing)

Test all speed/rate variations from training episodes.

**Isaac Lab - All Variants**:

```bash
# TASK 1: Standing (1 variant)
python send_velocity_commands_isaac.py --task 1

# TASK 2: Walking (5 variants)
python send_velocity_commands_isaac.py --task 2      # Alias: normal (1.0 m/s)
python send_velocity_commands_isaac.py --task 2.1    # Slow (0.5 m/s)
python send_velocity_commands_isaac.py --task 2.2    # Normal (1.0 m/s)
python send_velocity_commands_isaac.py --task 2.3    # Fast (1.5 m/s)
python send_velocity_commands_isaac.py --task 2.4    # Moderate (0.8 m/s)

# TASK 3: Turn in Place (5 variants)
python send_velocity_commands_isaac.py --task 3      # Alias: normal CCW (1.0 rad/s)
python send_velocity_commands_isaac.py --task 3.1    # Slow CCW (0.5 rad/s)
python send_velocity_commands_isaac.py --task 3.2    # Normal CCW (1.0 rad/s)
python send_velocity_commands_isaac.py --task 3.3    # Normal CW (-1.0 rad/s)
python send_velocity_commands_isaac.py --task 3.4    # Fast CCW (1.5 rad/s)

# TASK 4: Walk + Turn (5 variants)
python send_velocity_commands_isaac.py --task 4      # Alias: right arc
python send_velocity_commands_isaac.py --task 4.1    # Right arc (0.8, 0.6)
python send_velocity_commands_isaac.py --task 4.2    # Straight fast (1.2, 0.0)
python send_velocity_commands_isaac.py --task 4.3    # Left arc (0.8, -0.6)
python send_velocity_commands_isaac.py --task 4.4    # Tight turn (0.5, 1.0)
```

**Gazebo - All Variants**:

```bash
./send_velocity_commands_gazebo.sh
```

| Option | Task | Description | (vx, vy, wz) |
|--------|------|-------------|--------------|
| **1** | **1** | **Standing** ⭐ | (0.0, 0.0, 0.0) |
| 2 | 2.1 | Walk slow | (0.5, 0.0, 0.0) |
| **3** | **2.2** | **Walk normal** ⭐ | (1.0, 0.0, 0.0) |
| 4 | 2.3 | Walk fast | (1.5, 0.0, 0.0) |
| 5 | 2.4 | Walk moderate | (0.8, 0.0, 0.0) |
| 6 | 3.1 | Turn slow CCW | (0.0, 0.0, 0.5) |
| **7** | **3.2** | **Turn normal CCW** ⭐ | (0.0, 0.0, 1.0) |
| 8 | 3.3 | Turn normal CW | (0.0, 0.0, -1.0) |
| 9 | 3.4 | Turn fast CCW | (0.0, 0.0, 1.5) |
| **10** | **4.1** | **Right arc** ⭐ | (0.8, 0.0, 0.6) |
| 11 | 4.2 | Straight fast | (1.2, 0.0, 0.0) |
| 12 | 4.3 | Left arc | (0.8, 0.0, -0.6) |
| 13 | 4.4 | Tight turn | (0.5, 0.0, 1.0) |
| 14 | - | Custom | (enter values) |

**⭐ = Simplified aliases (options 1, 3, 7, 10)**

### 🔍 Which Should You Use?

**Use SIMPLIFIED (1, 2, 3, 4) when:**
- ✅ Quick sanity check
- ✅ Demonstrating the 4 main task categories
- ✅ Teaching/showing someone
- ✅ Initial policy validation

**Use DETAILED (2.1, 2.2, 3.3, etc.) when:**
- ✅ Comprehensive policy testing
- ✅ Matching exact training episode conditions
- ✅ Testing performance across speed variations
- ✅ Research/data collection

### 💡 Pro Tips

**1. List All Available Tasks**:
```bash
python send_velocity_commands_isaac.py --list
```

**2. Gazebo Quick Select**:
In the menu, just remember:
- **1** = Standing
- **3** = Walking
- **7** = Turning
- **10** = Walk+Turn

**3. Custom Commands (Advanced)**:
```bash
# Isaac Lab - custom velocity
python send_velocity_commands_isaac.py --linear_x 0.7 --angular_z 0.4

# Gazebo - option 14 for custom input
./send_velocity_commands_gazebo.sh
# Select 14, then enter custom values
```

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

---

## 🛠️ Troubleshooting

### Issue 1: Environment Variables Not Set

```bash
# Check if set
echo $VISTEC_REPO $UNITREE_LAB $VISTEC_WS

# If empty, set them
export VISTEC_REPO=~/Vistec_Intern_Exam
export UNITREE_LAB=~/Vistec_Intern_Exam/unitree_rl_lab
export VISTEC_WS=~/Vistec_Intern_Exam/Vistec_ex_ws
```

### Issue 2: verify_setup.sh Fails

```bash
# Make executable
chmod +x verify_setup.sh

# Run with verbose output
bash -x verify_setup.sh

# Check specific components
ls $UNITREE_LAB  # Should show: source/, scripts/, logs/
ls $VISTEC_REPO/trained_models  # Should show: 6 .pt files (mlp_dr.pt, mlp.pt, lstm_dr.pt, lstm.pt, Implicit_dr.pt, implicit.pt)
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
1. Verify using MLP actuator
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

### Issue 8: ModuleNotFoundError: No module named 'rclpy'

**Cause**: Using conda Python instead of system Python for ROS 2

**Solution**:
```bash
# Deactivate conda before running ROS 2 commands
conda deactivate

# Then run ROS 2 commands
ros2 launch deploy_policy go2_deploy.launch.py
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

**Last Updated**: 2026-02-12
**Isaac Lab**: 2.3.0 | **ROS 2**: Humble | **Tested**: Ubuntu 22.04, RTX 3090

**Clone & Get Started**: `git clone https://github.com/BhumipatNgamphueak/Vistec_Intern_Exam.git`

</div>

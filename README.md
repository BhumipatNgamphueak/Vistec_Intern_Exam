# Vistec Intern Exam: Sim2Sim Quadruped Locomotion

**Unitree Go2 Locomotion: IsaacLab → Gazebo Transfer**

[![IsaacSim](https://img.shields.io/badge/IsaacSim-5.1.0-silver.svg)](https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html)
[![Isaac Lab](https://img.shields.io/badge/IsaacLab-2.3.0-silver)](https://isaac-sim.github.io/IsaacLab)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/index.html)
[![Gazebo](https://img.shields.io/badge/Gazebo-Ignition-orange)](https://gazebosim.org/)
[![Python](https://img.shields.io/badge/Python-3.10-blue)](https://www.python.org/)


**Last Updated**: February 12, 2026

---

## 📋 Quick Navigation

| Section | Description |
|---------|-------------|
| [Quick Start](#-quick-start) | Clone to running policies in 30 min |
| [Pre-trained Models](#-pre-trained-models) | 6 ready-to-use policies (27 MB) |
| [Gazebo Deployment](#-deployment) | 3-terminal workflow |
| [Training Pipeline](#-training-pipeline) | Train your own policies |
| [4 Locomotion Tasks](#-4-locomotion-tasks) | Stand, Walk, Turn, Combined |
| [Velocity Commands](#-velocity-commands-guide) | Testing the 4 tasks |
| [Task Names](#-registered-task-names) | All registered Isaac Lab tasks |

## 📁 Repository Structure

```
Vistec_Intern_Exam/
├── README.md                          # This complete guide
│
├── verify_setup.sh                    # Setup verification script
├── send_velocity_commands_gazebo.sh   # Gazebo velocity control (4 tasks + custom)
├── send_velocity_commands_isaac.py    # Isaac velocity presets (4 tasks)
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

### Task Name Pattern

```
Unitree-Go2-Velocity-[ACTUATOR]-[DR_STATUS]
```

Where:
- `[ACTUATOR]` = MLP-Custom, MLP, LSTM, LSTM-Custom, LSTM-MyModel, Implicit
- `[DR_STATUS]` = DR, No-DR, Custom, Enhanced, or omitted for base

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
conda activate env_isaaclab  # Skip if not using conda

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
conda activate env_isaaclab  # Skip if not using conda

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
conda activate env_isaaclab  # Skip if not using conda

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

**Step 1: Deploy Policy (Terminal 1)**

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

**Launch Arguments**:
- `policy_path`: Path to .pt or .onnx model
- `device`: `cpu` or `cuda` (default: cpu, use `cuda` if GPU available)

**Step 2: Launch Gazebo Fortress (Terminal 2)**

```bash
# Deactivate conda if active
conda deactivate

cd $VISTEC_WS
source install/setup.bash

# Launch Gazebo with Go2 robot
ros2 launch go2_gazebo_simulation go2_fortress.launch.py
```

**Expected**: Gazebo opens with Go2 robot spawned

**Step 3: Send Velocity Commands (Terminal 3)**

**Interactive Menu (4 main tasks + custom)**:
```bash
cd $VISTEC_REPO
./send_velocity_commands_gazebo.sh

# Select an option:
# 1 - Task 1: Standing
# 2 - Task 2: Walking (1.0 m/s)
# 3 - Task 3: Turn in Place (1.0 rad/s)
# 4 - Task 4: Walk + Turn (arc)
# 5 - Custom command
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
| **2** | Walking (forward) | 1.0 | 0.0 | 0.0 | `--task 2` | Option 2 |
| **3** | Turn in Place (CCW) | 0.0 | 0.0 | 1.0 | `--task 3` | Option 3 |
| **4** | Walk + Turn (arc) | 0.8 | 0.0 | 0.6 | `--task 4` | Option 4 |

**Notes**:
- Use custom velocities with `--linear_x` and `--angular_z` parameters for fine-tuning
- Gazebo: Select option 5 for custom command
- Isaac Lab: Modify config file ranges for specific velocities (see script output)

---

## 📊 Velocity Commands Guide

Testing the 4 locomotion tasks is simple:

### 🎯 Using Isaac Lab

**4 Main Tasks**:
```bash
python send_velocity_commands_isaac.py --task 1   # Task 1: Standing
python send_velocity_commands_isaac.py --task 2   # Task 2: Walking (1.0 m/s)
python send_velocity_commands_isaac.py --task 3   # Task 3: Turn in Place (1.0 rad/s)
python send_velocity_commands_isaac.py --task 4   # Task 4: Walk+Turn (arc)
```

**Custom Velocities**:
```bash
# Custom velocity command
python send_velocity_commands_isaac.py --linear_x 0.7 --angular_z 0.4

# List all tasks
python send_velocity_commands_isaac.py --list
```

### 🎯 Using Gazebo

**4 Main Tasks**:
```bash
./send_velocity_commands_gazebo.sh

# Then select:
# 1 - Task 1: Standing
# 2 - Task 2: Walking
# 3 - Task 3: Turn in Place
# 4 - Task 4: Walk + Turn
# 5 - Custom command
```

| Option | Task | Velocities (vx, vy, wz) |
|--------|------|-------------------------|
| **1** | Standing | (0.0, 0.0, 0.0) |
| **2** | Walking | (1.0, 0.0, 0.0) |
| **3** | Turn in Place | (0.0, 0.0, 1.0) |
| **4** | Walk + Turn | (0.8, 0.0, 0.6) |
| **5** | Custom | (enter values) |

**Manual ROS 2 Command**:
```bash
# Publish velocity directly
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "linear: {x: 1.0, y: 0.0, z: 0.0}
   angular: {x: 0.0, y: 0.0, z: 0.0}" --rate 10
```
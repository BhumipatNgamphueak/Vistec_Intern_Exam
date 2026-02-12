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
├── test_isaac_task.py                 # Isaac Lab testing (exact training sequences)
├── test_gazebo_task.sh                # Gazebo testing (auto-running sequences)
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

**Expected**: ✅ All checks passed (environment vars, directories, software) expect Ros2 folder because we didn't build yet.

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

cd ~/IsaacLab

# 1. MLP with DR (RECOMMENDED - Best Sim2Sim transfer)
~/IsaacLab/isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py \
  --task Unitree-Go2-Velocity-MLP-Custom \
  --checkpoint $VISTEC_REPO/trained_models/mlp_dr.pt \
  --num_envs 4

# 2. LSTM with DR
~/IsaacLab/isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py \
  --task Unitree-Go2-Velocity-LSTM-DR \
  --checkpoint $VISTEC_REPO/trained_models/lstm_dr.pt \
  --num_envs 4

# 3. Implicit with DR
~/IsaacLab/isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py \
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

cd ~/IsaacLab

# Train MLP policy with domain randomization (RECOMMENDED)
# Time: 6-8 hours on RTX 3090
~/IsaacLab/isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
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

cd ~/IsaacLab

# Test policy (automatically exports to ONNX/JIT)
~/IsaacLab/isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py \
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

---

## 🎮 Testing & Deployment

Two testing environments available:
- **Isaac Lab**: Physics simulation with GPU acceleration (visualization & training)
- **Gazebo**: Realistic simulation with ROS 2 (deployment testing)

---

## 🔬 ISAAC LAB: Testing Policies

### Prerequisites

**⚠️ IMPORTANT: Activate Isaac Lab Environment First!**

```bash
# 1. Activate Isaac Lab conda environment (REQUIRED!)
conda activate env_isaaclab

# If you don't have this environment, install Isaac Lab:
# cd ~/IsaacLab && ./isaaclab.sh --install

# 2. Set environment variables
export VISTEC_REPO=~/Vistec_Intern_Exam
export UNITREE_LAB=$VISTEC_REPO/unitree_rl_lab
export ISAACLAB_PATH=~/IsaacLab

# 3. Verify Isaac Lab is installed
python -c "from importlib.metadata import version; print('Isaac Lab:', version('rsl-rl-lib'))"
```

### Method 1: Direct Testing with Test Script (RECOMMENDED)

**Step-by-step for all 4 tasks:**

```bash
cd $VISTEC_REPO

# Task 0: Standing (20s) - default: mlp_dr
python test_isaac_task.py --task 0

# Task 1: Walking (4 speeds, auto-switching)
python test_isaac_task.py --task 1

# Task 2: Turn in Place (4 rates, auto-switching)
python test_isaac_task.py --task 2

# Task 3: Walk + Turn (5 maneuvers, auto-switching)
python test_isaac_task.py --task 3

# Test with different policies
python test_isaac_task.py --task 1 --policy lstm_dr
python test_isaac_task.py --task 1 --policy implicit_dr

# List all tasks and sequences
python test_isaac_task.py --list
```

**Command Pattern:**
```bash
python test_isaac_task.py --task <TASK_ID> [--policy <MODEL>] [--envs <NUM>]

# Parameters:
#   --task <0-3>       : Task ID (0=Standing, 1=Walking, 2=Turn, 3=Walk+Turn)
#   --policy <MODEL>   : Policy/model to use (default: mlp_dr)
#   --envs <NUM>       : Number of parallel environments (default: 1)
#   --list             : List all tasks

# Available models (--policy):
#   mlp_dr       (default) → trained_models/mlp_dr.pt       (Task: Unitree-Go2-Velocity-MLP-Custom)
#   mlp                    → trained_models/mlp.pt           (Task: Unitree-Go2-Velocity-MLP-No-DR)
#   lstm_dr                → trained_models/lstm_dr.pt       (Task: Unitree-Go2-Velocity-LSTM-DR)
#   lstm                   → trained_models/lstm.pt          (Task: Unitree-Go2-Velocity-LSTM-No-DR)
#   implicit_dr            → trained_models/Implicit_dr.pt   (Task: Unitree-Go2-Velocity-Implicit-DR)
#   implicit               → trained_models/implicit.pt      (Task: Unitree-Go2-Velocity-Implicit)
```

**Examples:**
```bash
# Test Task 1 with 4 parallel environments
python test_isaac_task.py --task 1 --envs 4

# Test all tasks sequentially
for task in 0 1 2 3; do
    echo "Testing Task $task..."
    python test_isaac_task.py --task $task
done

# Test with all 6 policies
for policy in mlp_dr mlp lstm_dr lstm implicit_dr implicit; do
    python test_isaac_task.py --task 1 --policy $policy
done
```

### Method 2: Direct Isaac Lab Play Script

```bash
cd ~/IsaacLab

# MLP Policy with DR (RECOMMENDED)
~/IsaacLab/isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py \
  --task Unitree-Go2-Velocity-MLP-Custom \
  --checkpoint $VISTEC_REPO/trained_models/mlp_dr.pt \
  --num_envs 4

# LSTM Policy with DR
~/IsaacLab/isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py \
  --task Unitree-Go2-Velocity-LSTM-DR \
  --checkpoint $VISTEC_REPO/trained_models/lstm_dr.pt \
  --num_envs 4

# Implicit Actuator with DR
~/IsaacLab/isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py \
  --task Unitree-Go2-Velocity-Implicit-DR \
  --checkpoint $VISTEC_REPO/trained_models/Implicit_dr.pt \
  --num_envs 4
```

**Command Pattern:**
```bash
~/IsaacLab/isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py \
  --task <TASK_NAME> \
  --checkpoint <PATH_TO_MODEL> \
  --num_envs <NUM>

# Parameters:
#   --task          : Task name (see "Registered Task Names" section)
#   --checkpoint    : Path to .pt model file
#   --num_envs      : Number of parallel environments (default: 1)
```

---

## 🤖 GAZEBO: ROS 2 Deployment Testing

### Prerequisites

```bash
# Deactivate conda if active
conda deactivate

# Set environment variables
export VISTEC_REPO=~/Vistec_Intern_Exam
export VISTEC_WS=$VISTEC_REPO/Vistec_ex_ws

# Source ROS 2
source /opt/ros/humble/setup.bash
```

### Setup ROS 2 Workspace (First Time Only)

```bash
cd $VISTEC_WS

# Install dependencies and build
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash

# Add to bashrc
echo "source $VISTEC_WS/install/setup.bash" >> ~/.bashrc
```

### Launch & Test (3 Terminals Required)

#### Terminal 1: Deploy Policy Node

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

**Command Pattern:**
```bash
ros2 launch deploy_policy go2_deploy.launch.py \
  policy_path:=<PATH_TO_MODEL> \
  device:=<cpu|cuda>

# Parameters:
#   policy_path: Path to .pt or .onnx model file
#   device:      cpu or cuda (default: cpu)
```

**Available Models:**
```bash
# MLP with DR (RECOMMENDED)
policy_path:=$VISTEC_REPO/trained_models/mlp_dr.pt

# LSTM with DR
policy_path:=$VISTEC_REPO/trained_models/lstm_dr.pt

# Implicit with DR
policy_path:=$VISTEC_REPO/trained_models/Implicit_dr.pt
```

#### Terminal 2: Launch Gazebo Simulation

```bash
# Deactivate conda if active
conda deactivate

cd $VISTEC_WS
source install/setup.bash

# Launch Gazebo Fortress with Go2 robot
ros2 launch go2_gazebo_simulation go2_fortress.launch.py
```

**Expected Output**: Gazebo window opens with Go2 robot spawned

#### Terminal 3: Send Velocity Commands

**Method 1: Automatic Test Script (RECOMMENDED)**

Test with exact training sequences:

```bash
cd $VISTEC_REPO
./test_gazebo_task.sh

# Interactive menu appears:
# Select task 0, 1, 2, or 3
# Script automatically runs the entire 20-second sequence!
```

**Task Options:**
- **Task 0**: Standing (20s) - vx=0.0
- **Task 1**: Walking (20s, 4 speeds) - vx: 0.5→1.0→1.5→0.8
- **Task 2**: Turn (20s, 4 rates) - wz: +0.5→+1.0→-1.0→+1.5
- **Task 3**: Walk+Turn (20s, 5 maneuvers) - Combined motions

**Method 2: Manual ROS 2 Command**

For static velocity commands:

```bash
# Forward walking at 1.0 m/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "linear: {x: 1.0, y: 0.0, z: 0.0}
   angular: {x: 0.0, y: 0.0, z: 0.0}" --rate 10

# Turn in place at 1.0 rad/s (CCW)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "linear: {x: 0.0, y: 0.0, z: 0.0}
   angular: {x: 0.0, y: 0.0, z: 1.0}" --rate 10

# Walk + Turn (arc motion)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "linear: {x: 0.8, y: 0.0, z: 0.0}
   angular: {x: 0.0, y: 0.0, z: 0.6}" --rate 10
```

**Command Pattern:**
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "linear: {x: <VX>, y: <VY>, z: 0.0}
   angular: {x: 0.0, y: 0.0, z: <WZ>}" --rate 10

# Parameters:
#   VX:  Forward/backward velocity (m/s), range: -1.0 to 1.0
#   VY:  Left/right velocity (m/s), range: -0.4 to 0.4
#   WZ:  Yaw rotation rate (rad/s), range: -1.0 to 1.0
```

---

---

## 🎯 4 Locomotion Tasks (Training Sequences)

The robot is trained and tested on **4 fundamental locomotion primitives** with time-varying sequences:

### Task Definitions

| Task | Name | Duration | Sequence Details |
|------|------|----------|------------------|
| **0** | Standing | 20s | vx=0.0, vy=0.0, wz=0.0 (constant) |
| **1** | Walking | 20s | vx: 0.5→1.0→1.5→0.8 m/s (5s each, 4 speeds) |
| **2** | Turn in Place | 20s | wz: +0.5→+1.0→-1.0→+1.5 rad/s (5s each, 4 rates) |
| **3** | Walk + Turn | 20s | 5 combined maneuvers (see below) |

### Detailed Sequences

**Task 0: Standing (20s)**
- Constant: vx=0.0, vy=0.0, wz=0.0 for entire 20 seconds

**Task 1: Walking (20s, 4 segments)**
1. Slow: vx=0.5 m/s for 5s
2. Normal: vx=1.0 m/s for 5s
3. Fast: vx=1.5 m/s for 5s
4. Moderate: vx=0.8 m/s for 5s

**Task 2: Turn in Place (20s, 4 segments)**
1. Slow CCW: wz=+0.5 rad/s for 5s
2. Normal CCW: wz=+1.0 rad/s for 5s
3. Normal CW: wz=-1.0 rad/s for 5s (direction change!)
4. Fast CCW: wz=+1.5 rad/s for 5s

**Task 3: Walk + Turn (20s, 5 segments)**
1. Right arc: (vx=0.8, wz=+0.6) for 5s
2. Straight: (vx=1.0, wz=0.0) for 2s
3. Left arc: (vx=0.8, wz=-0.6) for 5s
4. Fast straight: (vx=1.2, wz=0.0) for 3s
5. Tight turn: (vx=0.5, wz=+1.0) for 5s

---

## 📊 Quick Testing Guide

### Isaac Lab - Quick Commands

```bash
# First: Activate Isaac Lab environment!
conda activate env_isaaclab

cd $VISTEC_REPO

# Test Task 0 (Standing)
python test_isaac_task.py --task 0

# Test Task 1 (Walking - 4 speeds)
python test_isaac_task.py --task 1

# Test Task 2 (Turn - 4 rates)
python test_isaac_task.py --task 2

# Test Task 3 (Walk+Turn - 5 maneuvers)
python test_isaac_task.py --task 3

# List all tasks
python test_isaac_task.py --list

# Test all tasks in sequence
for task in 0 1 2 3; do
    python test_isaac_task.py --task $task
done
```

### Gazebo - Quick Commands

```bash
cd $VISTEC_REPO

# Launch test script (interactive menu)
./test_gazebo_task.sh

# Then select: 0, 1, 2, or 3
# Script automatically runs the entire 20-second sequence!
```

### Comparison Table

| Feature | Isaac Lab | Gazebo |
|---------|-----------|--------|
| **Command** | `python test_isaac_task.py --task <0-3>` | `./test_gazebo_task.sh` → select task |
| **Task 0** | ✅ Supported | ✅ Supported (auto-runs) |
| **Task 1** | ✅ Supported | ✅ Auto-runs 4 speeds |
| **Task 2** | ✅ Supported | ✅ Auto-runs 4 rates |
| **Task 3** | ✅ Supported | ✅ Auto-runs 5 maneuvers |
| **Sequences** | Exact training sequences | Exact training sequences |
| **Policy** | MLP/LSTM (--lstm flag) | MLP/LSTM (launch param) |
| **Visualization** | Isaac Sim GUI | Gazebo GUI |

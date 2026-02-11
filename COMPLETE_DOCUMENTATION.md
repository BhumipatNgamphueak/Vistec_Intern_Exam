# Complete Documentation: Vistec Intern Exam

**Unitree Go2 Sim2Sim Locomotion: IsaacLab → Gazebo Transfer**

[![IsaacSim](https://img.shields.io/badge/IsaacSim-5.1.0-silver.svg)](https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html) [![Isaac Lab](https://img.shields.io/badge/IsaacLab-2.3.0-silver)](https://isaac-sim.github.io/IsaacLab) [![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/index.html) [![Gazebo](https://img.shields.io/badge/Gazebo-Ignition-orange)](https://gazebosim.org/) [![Python](https://img.shields.io/badge/Python-3.10-blue)](https://www.python.org/)

**📘 This is the COMPLETE all-in-one documentation** - Everything you need in one place!

**Version**: 2.0 (Self-Contained)
**Last Updated**: February 12, 2026
**Repository**: https://github.com/BhumipatNgamphueak/Vistec_Intern_Exam.git

---

## ⚡ Quick Links

| Section | Description |
|---------|-------------|
| [Quick Start](#-quick-start-30-minutes) | Clone to running policies in 30 min |
| [Pre-trained Models](#-pre-trained-models) | 6 ready-to-use policies (27 MB) |
| [Gazebo Deployment](#-deployment-to-gazebo) | 3-terminal workflow |
| [Training Pipeline](#-training-pipeline) | Train your own policies |
| [4 Locomotion Tasks](#-4-locomotion-tasks) | Stand, Walk, Turn, Combined |
| [Troubleshooting](#-troubleshooting) | Common issues & solutions |
| [Task Names](#-registered-task-names) | All registered Isaac Lab tasks |

---

## 📋 Table of Contents

1. [Quick Start](#-quick-start-30-minutes)
2. [What's Included](#-whats-included)
3. [Pre-trained Models](#-pre-trained-models)
4. [Registered Task Names](#-registered-task-names)
5. [Training Pipeline](#-training-pipeline)
6. [Deployment to Gazebo](#-deployment-to-gazebo)
7. [4 Locomotion Tasks](#-4-locomotion-tasks)
8. [Troubleshooting](#-troubleshooting)
9. [File Structure](#-complete-file-structure)
10. [Verification](#-verification)

---

## 🚀 Quick Start (30 Minutes)

### Step 1: Clone Repository (2 minutes)

```bash
git clone https://github.com/BhumipatNgamphueak/Vistec_Intern_Exam.git
cd Vistec_Intern_Exam
```

### Step 2: Set Environment Variables (1 minute)

```bash
export VISTEC_REPO=~/Vistec_Intern_Exam
export UNITREE_LAB=~/Vistec_Intern_Exam/unitree_rl_lab
export ACTUATOR_NET=~/Vistec_Intern_Exam/Actuator_net
export VISTEC_WS=~/Vistec_Intern_Exam/Vistec_ex_ws

# Make permanent
echo "export VISTEC_REPO=~/Vistec_Intern_Exam" >> ~/.bashrc
echo "export UNITREE_LAB=~/Vistec_Intern_Exam/unitree_rl_lab" >> ~/.bashrc
echo "export ACTUATOR_NET=~/Vistec_Intern_Exam/Actuator_net" >> ~/.bashrc
echo "export VISTEC_WS=~/Vistec_Intern_Exam/Vistec_ex_ws" >> ~/.bashrc
source ~/.bashrc
```

### Step 3: Install Isaac Lab (15 minutes)

```bash
# Install Isaac Lab 2.3.0
cd ~
git clone https://github.com/isaac-sim/IsaacLab.git
cd ~/IsaacLab
./isaaclab.sh --install  # Takes ~10-15 minutes
```

### Step 4: Install unitree_rl_lab Extension (5 minutes)

```bash
# Activate Isaac Lab environment (if using conda)
conda activate env_isaaclab  # Skip if not using conda

cd $UNITREE_LAB
~/IsaacLab/isaaclab.sh -p -m pip install -e source/unitree_rl_lab
```

### Step 5: Verify Setup (2 minutes)

```bash
cd $VISTEC_REPO
./verify_setup.sh
```

**Expected Output**: ✅ All checks passed (environment vars, directories, software)

### Step 6: Test Pre-trained Policy (2 minutes)

```bash
# Activate Isaac Lab environment (if using conda)
conda activate env_isaaclab  # Skip if not using conda

cd $UNITREE_LAB

# Test with pre-trained MLP policy (RECOMMENDED)
~/IsaacLab/isaaclab.sh -p scripts/rsl_rl/play.py \
  --task Unitree-Go2-Velocity-MLP-Custom \
  --checkpoint $VISTEC_REPO/trained_models/mlp_dr.pt \
  --num_envs 1
```

---

## 📦 What's Included

This repository is **100% self-contained**:

- ✅ Complete Isaac Lab framework (source/, scripts/)
- ✅ **6 pre-trained policies** (27 MB - MLP, LSTM, Implicit with/without DR)
- ✅ Actuator models (LSTM, MLP - R²>0.998)
- ✅ 12 custom training configurations
- ✅ ROS 2 deployment workspace
- ✅ Robot models and descriptions
- ✅ Complete documentation

**No additional cloning required!**

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
5. `Unitree-Go2-Velocity-LSTM-Custom` - LSTM custom configuration
6. `Unitree-Go2-Velocity-LSTM-Custom-Enhanced` - LSTM enhanced version
7. `Unitree-Go2-Velocity-LSTM-MyModel` - LSTM with your trained model

### Implicit Actuator Tasks
8. `Unitree-Go2-Velocity-Implicit-DR` - Implicit with Domain Randomization
9. `Unitree-Go2-Velocity-Implicit` - Implicit without Domain Randomization

### Base Task
10. `Unitree-Go2-Velocity` - Base configuration
11. `Unitree-Go2-Velocity-LSTM` - Base LSTM configuration

### Task Name Pattern

```
Unitree-Go2-Velocity-[ACTUATOR]-[DR_STATUS]
```

Where:
- `[ACTUATOR]` = MLP-Custom, MLP, LSTM, Implicit
- `[DR_STATUS]` = DR, No-DR, Custom, or omitted for base

---

## 🎓 Training Pipeline

### Stage 1: Train Policy

```bash
# Activate Isaac Lab environment (if using conda)
conda activate isaaclab  # Skip if not using conda

cd $UNITREE_LAB

# Train MLP policy with domain randomization (RECOMMENDED)
# Time: 6-8 hours on RTX 3090
~/IsaacLab/isaaclab.sh -p scripts/rsl_rl/train.py \
  --task Unitree-Go2-Velocity-MLP-Custom \
  --num_envs 4096 \
  --headless

# Output: logs/rsl_rl/unitree_go2_velocity_mlp_custom/{timestamp}/model_*.pt
```

**Key Training Parameters**:
- **PD Gains**: Kp=25.0, Kd=0.5 (unified across all actuators)
- **Environments**: 4096 parallel
- **Iterations**: ~25,000 (6-8 hours)
- **Domain Randomization**: 15 strategies

### Stage 2: Test & Export Policy

```bash
# Activate Isaac Lab environment (if using conda)
conda activate isaaclab  # Skip if not using conda

cd $UNITREE_LAB

# Test policy (automatically exports to ONNX/JIT)
~/IsaacLab/isaaclab.sh -p scripts/rsl_rl/play.py \
  --task Unitree-Go2-Velocity-MLP-Custom \
  --num_envs 32 \
  --load_run {timestamp}

# Exported to: logs/rsl_rl/.../exported/policy.onnx
```

---

## 🤖 Deployment to Gazebo

### Prerequisites: Install ROS 2 Humble

```bash
sudo apt update
sudo apt install ros-humble-desktop ros-humble-ros-gz -y
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Build ROS 2 Workspace (First Time)

```bash
cd $VISTEC_WS

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install
source install/setup.bash

# Add to bashrc
echo "source $VISTEC_WS/install/setup.bash" >> ~/.bashrc
```

### Launch Gazebo with Policy

**Step 1: Launch Gazebo Fortress (Terminal 1)**

```bash
# Deactivate conda if active
conda deactivate

cd $VISTEC_WS
source install/setup.bash

# Launch Gazebo with Go2 robot
ros2 launch go2_gazebo_simulation go2_fortress.launch.py
```

**Expected**: Gazebo Fortress opens with Go2 robot spawned in the world

**Step 2: Deploy Policy (Terminal 2)**

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
- `device`: `cpu` or `cuda` (default: cpu, use `cuda` if GPU with CUDA available)

**Step 3: Send Velocity Commands (Terminal 3)**

**Interactive Menu (14 options)**:
```bash
cd $VISTEC_REPO
./send_velocity_commands_gazebo.sh

# Select an option (e.g., Option 3: Walk normal at 1.0 m/s)
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

### Task 1: Standing (1 variant)
| Variant | lin_x | lin_y | ang_z | Gazebo Option | Isaac Preset |
|---------|-------|-------|-------|---------------|--------------|
| Stand still | 0.0 | 0.0 | 0.0 | Option 1 | `--task 1` |

### Task 2: Walking (4 speeds)
| Variant | lin_x (m/s) | lin_y | ang_z | Gazebo Option | Isaac Preset |
|---------|-------------|-------|-------|---------------|--------------|
| Slow | 0.5 | 0.0 | 0.0 | Option 2 | `--task 2.1` |
| Normal | 1.0 | 0.0 | 0.0 | Option 3 | `--task 2.2` ⭐ |
| Fast | 1.5 | 0.0 | 0.0 | Option 4 | `--task 2.3` |
| Moderate | 0.8 | 0.0 | 0.0 | Option 5 | `--task 2.4` |

### Task 3: Turn in Place (4 rates)
| Variant | lin_x | lin_y | ang_z (rad/s) | Gazebo Option | Isaac Preset |
|---------|-------|-------|---------------|---------------|--------------|
| Slow CCW | 0.0 | 0.0 | +0.5 | Option 6 | `--task 3.1` |
| Normal CCW | 0.0 | 0.0 | +1.0 | Option 7 | `--task 3.2` |
| Normal CW | 0.0 | 0.0 | -1.0 | Option 8 | `--task 3.3` |
| Fast CCW | 0.0 | 0.0 | +1.5 | Option 9 | `--task 3.4` |

### Task 4: Walk + Turn (4 combinations)
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
```

**Isaac Lab**:
```bash
# List all tasks
python send_velocity_commands_isaac.py --list

# Test specific task
python send_velocity_commands_isaac.py --task 2.2  # Walk normal
python send_velocity_commands_isaac.py --task 4.1  # Right arc
```

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
chmod +x verify_setup.sh
bash -x verify_setup.sh

# Check specific components
ls $UNITREE_LAB  # Should show: source/, scripts/, logs/
ls $VISTEC_REPO/trained_models  # Should show: 6 .pt files
```

### Issue 3: Module 'unitree_rl_lab' not found

```bash
cd $UNITREE_LAB
~/IsaacLab/isaaclab.sh -p -m pip install -e source/unitree_rl_lab
```

### Issue 4: ROS 2 Workspace Build Fails

```bash
cd $VISTEC_WS

# Clean and rebuild
rm -rf build install log
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### Issue 5: Gazebo Robot Falls

**Cause**: PD gains mismatch or wrong actuator type

**Solution**:
1. Verify using MLP actuator: `actuator_type:=mlp`
2. Check PD gains in URDF (should be Kp=25.0, Kd=0.5)
3. Use MLP-Custom policy (best DR for Gazebo)

### Issue 6: CUDA Not Found

```bash
# Check CUDA
nvidia-smi
nvcc --version

# If missing, install CUDA 11.8 or 12.1
# https://developer.nvidia.com/cuda-downloads
```

---

## 📁 Complete File Structure

```
Vistec_Intern_Exam/                              # ~141 MB total
├── README.md                                    # Main comprehensive guide
├── COMPLETE_USER_GUIDE.md                       # Step-by-step 30-min setup
├── SELF_CONTAINED_REPOSITORY.md                 # Repository structure details
├── CORRECT_TASK_NAMES.md                        # Task naming reference
├── VERIFICATION_SUMMARY.md                      # Setup verification summary
├── ROS2_BUILD_TROUBLESHOOTING.md                # ROS 2 build guide
├── COMPLETE_DOCUMENTATION.md                    # This comprehensive summary
│
├── verify_setup.sh                              # Setup verification script
├── send_velocity_commands_gazebo.sh             # Gazebo velocity control (14 options)
├── send_velocity_commands_isaac.py              # Isaac velocity presets (13 tasks)
│
├── trained_models/                              # 27 MB - Pre-trained policies
│   ├── mlp_dr.pt                                # ⭐ RECOMMENDED (MLP + DR)
│   ├── mlp.pt                                   # MLP without DR
│   ├── lstm_dr.pt                               # LSTM + DR
│   ├── lstm.pt                                  # LSTM without DR
│   ├── Implicit_dr.pt                           # Implicit actuator + DR
│   └── implicit.pt                              # Implicit without DR
│
├── unitree_rl_lab/                              # ⭐ COMPLETE Isaac Lab framework
│   ├── source/unitree_rl_lab/                   # Framework source code
│   │   └── unitree_rl_lab/
│   │       ├── assets/
│   │       │   ├── robots/                      # Go2, H1, G1 definitions
│   │       │   └── actuator_models/             # Pre-trained actuators
│   │       │       ├── actuator.pth             # MLP (R²=0.998)
│   │       │       └── actuator_lstm.pth        # LSTM (R²=0.999)
│   │       ├── tasks/locomotion/robots/go2/     # 12 custom configs
│   │       │   ├── velocity_env_cfg_mlp_custom.py
│   │       │   ├── velocity_env_cfg_lstm_with_dr.py
│   │       │   ├── velocity_env_cfg_implicit_with_dr.py
│   │       │   └── [9 more configs]
│   │       └── utils/                           # Core utilities
│   ├── scripts/
│   │   ├── rsl_rl/
│   │   │   ├── train.py                         # Main training script
│   │   │   └── play.py                          # Policy playback
│   │   ├── actuator_comparison/                 # Comparison tools
│   │   ├── data_collection/                     # Data loggers
│   │   └── motor_testing/                       # Motor tests
│   ├── deploy/                                  # Real robot deployment (C++)
│   ├── docker/                                  # Docker setup
│   ├── Configs/                                 # Original config backups
│   ├── Utils/                                   # Episode generators
│   ├── Testing_Scripts/                         # Chirp tests
│   └── Policy_Playback/                         # Custom playback
│
├── Actuator_net/                                # Actuator training
│   ├── app/resources/
│   │   ├── actuator.pth                         # MLP (598 KB)
│   │   └── actuator_lstm.pth                    # LSTM
│   ├── train.py                                 # MLP training
│   ├── train_lstm.py                            # LSTM training
│   └── test.py                                  # Validation
│
└── Vistec_ex_ws/                                # ROS 2 deployment workspace
    └── src/
        ├── deploy_policy/                       # Policy inference node
        │   ├── launch/
        │   │   └── go2_deploy.launch.py
        │   ├── config/README_CONFIG.md
        │   └── scripts/
        └── go2_gazebo_simulation/               # Gazebo simulation setup
            ├── urdf/                            # Robot models
            ├── worlds/                          # Gazebo worlds
            └── launch/                          # Launch files
```

---

## ✅ Verification

### Automated Verification

```bash
cd $VISTEC_REPO
./verify_setup.sh
```

**Expected Output**:
```
✅ All environment variables set
✅ All directories exist
✅ 6 trained policies found (mlp_dr.pt, mlp.pt, lstm_dr.pt, lstm.pt, Implicit_dr.pt, implicit.pt)
✅ Isaac Lab extension installed
✅ unitree_rl_lab configs accessible
```

### Manual Verification

```bash
# Check environment variables
echo $VISTEC_REPO
echo $UNITREE_LAB
echo $VISTEC_WS

# Verify file counts
ls -lh $VISTEC_REPO/trained_models/*.pt | wc -l  # Should be 6
ls -lh $UNITREE_LAB/scripts/rsl_rl/              # Should show train.py, play.py
ls -lh $UNITREE_LAB/source/                      # Should show unitree_rl_lab/
ls -lh $ACTUATOR_NET/app/resources/*.pth | wc -l # Should be 2+

# Test Isaac Lab extension
cd $UNITREE_LAB
~/IsaacLab/isaaclab.sh -p scripts/list_envs.py | grep "Unitree-Go2-Velocity"
# Should list all registered tasks
```

---

## 🎯 Key Features

### 1. Three Actuator Models

| Actuator | Type | Parameters | R² Score | Use Case |
|----------|------|------------|----------|----------|
| **MLP** | Feedforward NN | 137 KB | 0.998 | **Best for Gazebo transfer** |
| **LSTM** | Recurrent NN | 226 KB | 0.999 | Highest accuracy, temporal dynamics |
| **Implicit** | Physics-based | N/A | N/A | Baseline comparison |

### 2. Comprehensive Domain Randomization (15 Strategies)

1. Physics Material (friction, restitution)
2. Base Mass variation (-1 to +3 kg)
3. COM Position (±2cm per link)
4. Joint Reset Position (±60°)
5. Motor Strength (Kp ±25%, Kd ±50%)
6. Joint Friction (0-0.15 Nm)
7. Joint Armature (0-0.015 kg·m²)
8. Velocity Push (±1.0 m/s)
9. Force Impulses (±10N)
10. Torque Impulses (±3Nm)
11. Action Latency (0-2 steps)
12. Observation Noise (IMU + encoder)
13. Velocity Limits (85% nominal)
14. Spawn Position (0-0.3m height)
15. Terrain Variation

### 3. Unified PD Gains

**All actuators use identical gains**:
- Kp = 25.0 (Stiffness)
- Kd = 0.5 (Damping)

---

## 📊 Environment Versions

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

## 📚 Related Documentation Files

- [README.md](README.md) - Main repository documentation
- [COMPLETE_USER_GUIDE.md](COMPLETE_USER_GUIDE.md) - Step-by-step setup guide
- [SELF_CONTAINED_REPOSITORY.md](SELF_CONTAINED_REPOSITORY.md) - Repository structure details
- [CORRECT_TASK_NAMES.md](CORRECT_TASK_NAMES.md) - All registered task names
- [VERIFICATION_SUMMARY.md](VERIFICATION_SUMMARY.md) - Setup verification details
- [ROS2_BUILD_TROUBLESHOOTING.md](ROS2_BUILD_TROUBLESHOOTING.md) - ROS 2 build guide

---

<div align="center">

**Sim2Sim Quadruped Locomotion Research**

Isaac Lab 2.3.0 | ROS 2 Humble | Tested: Ubuntu 22.04, RTX 3090

Clone & Get Started: `git clone https://github.com/BhumipatNgamphueak/Vistec_Intern_Exam.git`

</div>

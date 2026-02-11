# Vistec Intern Exam: Sim2Sim Research Project

<div align="center">

**Unitree Go2 Locomotion: IsaacLab Training → Gazebo Deployment**

[![IsaacSim](https://img.shields.io/badge/IsaacSim-5.1.0-silver.svg)](https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html)
[![Isaac Lab](https://img.shields.io/badge/IsaacLab-2.3.0-silver)](https://isaac-sim.github.io/IsaacLab)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/index.html)
[![Gazebo](https://img.shields.io/badge/Gazebo-Ignition-orange)](https://gazebosim.org/)

A comprehensive research framework for training quadruped locomotion policies in NVIDIA Isaac Lab (PhysX) and deploying them to Gazebo Ignition (DART/ODE) through learned neural actuator models.

**[📖 Start Here: READ ME_SIM2SIM.md](README_SIM2SIM.md)** ← Complete Sim2Sim Guide

</div>

---

## 🏗️ Project Overview

This repository contains a **3-module research system** for robust sim-to-sim transfer of quadruped locomotion policies:

```
┌────────────────────────────────────────────────────────────────┐
│  STAGE 1: Policy Training (Isaac Lab PhysX)                    │
│  ─────────────────────────────────────────────────────────────│
│  MODULE: unitree_rl_lab/                                       │
│  • PPO training with 15 domain randomization strategies        │
│  • Actuator types: MLP, LSTM, Implicit                        │
│  • Output: Trained policy checkpoint (.pt)                     │
└────────────────────────────────────────────────────────────────┘
                              ↓
┌────────────────────────────────────────────────────────────────┐
│  STAGE 2: Actuator Modeling (Neural Networks)                 │
│  ─────────────────────────────────────────────────────────────│
│  MODULE: Actuator_net/                                         │
│  • MLP: 137 KB model (R²=0.998)                               │
│  • LSTM: 226 KB model (R²=0.999)                              │
│  • Training: System identification on motor data              │
│  • Validation: Chirp frequency tests (0.1-20 Hz)             │
└────────────────────────────────────────────────────────────────┘
                              ↓
┌────────────────────────────────────────────────────────────────┐
│  STAGE 3: Deployment (ROS 2 + Gazebo)                         │
│  ─────────────────────────────────────────────────────────────│
│  MODULE: Vistec_ex_ws/                                         │
│  • ROS 2 Humble middleware                                     │
│  • Gazebo Ignition simulator                                   │
│  • Policy inference node (50 Hz)                               │
│  • 100+ analysis & comparison scripts                          │
└────────────────────────────────────────────────────────────────┘
```

---

## 📦 Repository Structure

### Three Independent Modules

```
Vistec_Intern_Exam/
│
├── README.md                           # ← YOU ARE HERE
├── README_SIM2SIM.md                   # ⭐ START HERE: Complete guide
│
├── unitree_rl_lab/                     # MODULE 1: RL Training (Isaac Lab)
│   ├── README.md                       # Module documentation
│   ├── Docs/                           # 25+ guides (training, testing, chirp)
│   ├── Configs/                        # 12 GO2 task configurations
│   ├── Policy_Playback/                # play_any_policy.sh ← Quick test
│   ├── Training_Scripts/               # continue_*.sh, train_*.sh
│   ├── Testing_Scripts/                # chirp tests, comparisons
│   └── Utils/                          # export_isaaclab_params.py, etc.
│
├── Actuator_net/                       # MODULE 2: Actuator Modeling
│   ├── README.md                       # Actuator training guide
│   ├── train.py                        # MLP actuator training
│   ├── train_lstm.py                   # LSTM actuator training
│   ├── test.py                         # Model validation
│   └── app/                            # GUI + pre-trained models
│       └── resources/
│           ├── actuator_lstm.pth       # ✅ LSTM model (226 KB)
│           ├── actuator.pth            # ✅ MLP model (137 KB)
│           └── datasets/               # Training data
│
└── Vistec_ex_ws/                       # MODULE 3: ROS 2 Gazebo Deployment
    ├── QUICKSTART.md                   # Gazebo deployment guide
    ├── README_VISUALIZATION.md         # Visualization tools (14 KB)
    ├── DELIVERABLES.md                 # Project deliverables
    └── [100+ analysis scripts]         # Sim2Sim validation
```

**Note**: The `unitree_rl_lab/`, `Actuator_net/`, and `Vistec_ex_ws/` modules in this repository contain **critical configuration, documentation, and scripts** extracted from the full repositories. The complete source code for each module resides in:
- Full `unitree_rl_lab`: `/home/drl-68/unitree_rl_lab/`
- Full `Actuator_net`: `/home/drl-68/actuator_net/`
- Full `Vistec_ex_ws`: `/home/drl-68/vistec_ex_ws/`

---

## 🚀 Quick Start

### Prerequisites

```bash
# NVIDIA Driver
nvidia-smi  # CUDA 11.8+ or 12.1+

# Ubuntu 22.04
lsb_release -a

# ROS 2 Humble
source /opt/ros/humble/setup.bash
```

### 5-Minute Demo

```bash
# 1. Test a pre-trained policy in Isaac Lab
cd unitree_rl_lab/Policy_Playback/
./play_any_policy.sh
# Choose option 1: MLP with DR (24,999 iterations)

# 2. Run chirp test for actuator validation
cd ../Testing_Scripts/
./run_chirp_tests.sh
# Choose option 4: All actuators

# 3. Export parameters for Gazebo
cd ../Utils/
python export_isaaclab_params.py --output gazebo_params.yaml
```

**Full Pipeline**: See [README_SIM2SIM.md](README_SIM2SIM.md)

---

## 📋 Module Details

### MODULE 1: unitree_rl_lab (RL Training)

**Purpose**: Train locomotion policies in Isaac Lab with comprehensive domain randomization

**Key Features**:
- 🤖 **3 Actuator Types**: MLP, LSTM, Implicit (physics-based)
- 🎲 **15 DR Strategies**: Gazebo-tuned randomization
- ⚡ **PPO Training**: 25,000 iterations, 4096 parallel environments
- 🎯 **PD Gains**: Unified Kp=25.0, Kd=0.5 across all actuators
- 📊 **Pre-trained Models**: 5 checkpoints ready to use

**Quick Access**:
```bash
# Play trained policy
cd unitree_rl_lab/Policy_Playback/
./play_any_policy.sh

# Continue training
cd ../Training_Scripts/
./continue_implicit_policies.sh

# Documentation
cd ../Docs/
ls *_GUIDE.md  # 25+ guides available
```

**Critical Files**:
- **[play_any_policy.sh](unitree_rl_lab/Policy_Playback/play_any_policy.sh)**: Universal policy player
- **[Configs/](unitree_rl_lab/Configs/)**: 12 GO2 task configurations
- **[Docs/](unitree_rl_lab/Docs/)**: 25+ comprehensive guides

---

### MODULE 2: Actuator_net (Neural Actuator Models)

**Purpose**: Train neural network models that learn real motor dynamics for sim-to-real transfer

**Key Features**:
- 🧠 **MLP Model**: 3-layer feedforward (R²=0.998, 137 KB)
- 🔄 **LSTM Model**: Recurrent network (R²=0.999, 226 KB)
- 📈 **Training Data**: 50s hanging motor data, multi-amplitude
- ✅ **Pre-trained**: Ready-to-use models included

**Quick Access**:
```bash
# Train MLP actuator
cd Actuator_net/
python train.py

# Train LSTM actuator
python train_lstm.py

# Test models
python test.py

# GUI application
cd app/
python main.py
```

**Pre-trained Models**:
- `app/resources/actuator_lstm.pth` (226 KB) - LSTM model
- `app/resources/actuator.pth` (137 KB) - MLP model
- `app/resources/actuator_lstm_6input.pth` (235 KB) - 6-input LSTM

**Documentation**: [Actuator_net/README.md](Actuator_net/README.md)

---

### MODULE 3: Vistec_ex_ws (ROS 2 Gazebo Deployment)

**Purpose**: Deploy trained policies to Gazebo Ignition via ROS 2

**Key Features**:
- 🚀 **ROS 2 Humble**: Policy inference node (50 Hz)
- 🎮 **Gazebo Ignition**: Physics simulation (DART/ODE)
- 📊 **100+ Analysis Scripts**: Comprehensive sim2sim validation
- 📈 **Visualization Tools**: Generate figures for papers
- ⚙️ **ros2_control**: Joint command/state management

**Quick Access**:
```bash
# Read quickstart guide
cd Vistec_ex_ws/
cat QUICKSTART.md

# Key documentation
cat README_VISUALIZATION.md  # Visualization tools
cat DELIVERABLES.md          # Project deliverables
```

**Full Repository**: `/home/drl-68/vistec_ex_ws/`
- Contains complete ROS 2 workspace with `src/`, `build/`, `install/`
- Launch files for Gazebo simulation
- Policy deployment nodes

---

## 🎓 Training & Deployment Workflow

### STEP 1: Train Policy (Isaac Lab)

```bash
cd /home/drl-68/unitree_rl_lab/

# Train MLP policy with comprehensive DR (6-8 hours on RTX 3090)
python scripts/rsl_rl/train.py \
    --task Unitree-Go2-Velocity-MLP-Custom \
    --num_envs 4096 \
    --headless
```

### STEP 2: Validate Actuators (Chirp Tests)

```bash
cd Vistec_Intern_Exam/unitree_rl_lab/Testing_Scripts/

# Run chirp tests in Isaac Lab
./run_chirp_tests.sh
# Choose option 4: All actuators

# Compare with Gazebo (after running Gazebo chirp test)
./compare_chirp_isaac_gazebo.py \
    --isaac ../../chirp_data_isaaclab/*.npz \
    --gazebo ../../chirp_data_gazebo/*.csv
```

### STEP 3: Deploy to Gazebo

```bash
cd /home/drl-68/vistec_ex_ws/
source install/setup.bash

# Launch Gazebo + Policy
ros2 launch go2_bringup go2_rl_policy.launch.py \
    policy_path:=/home/drl-68/unitree_rl_lab/logs/.../exported/policy.onnx \
    actuator_type:=mlp

# Send velocity commands
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "linear: {x: 0.5, y: 0.0, z: 0.0}
     angular: {x: 0.0, y: 0.0, z: 0.0}" --rate 10
```

---

## 📊 Available Trained Models

| Model | Actuator | DR | Iterations | Status | Location |
|-------|----------|----|-----------:|--------|----------|
| **2026-02-03_15-54-07_work_good** | MLP | ✅ | 24,999 | ⭐ Recommended | logs/rsl_rl/unitree_go2_velocity_mlp_custom/ |
| 2026-02-10_13-22-29 | Implicit | ✅ | 18,400 | Ready | logs/rsl_rl/unitree_go2_velocity_implicit_dr/ |
| 2026-02-10_13-22-31 | Implicit | ❌ | 12,100 | Ready | logs/rsl_rl/unitree_go2_velocity_implicit/ |
| 2026-02-07_11-16-35 | LSTM | ✅ | 25,000 | Has obs issues | logs/rsl_rl/unitree_go2_velocity_lstm_dr/ |
| 2026-02-07_22-16-50 | LSTM | ❌ | 9,900 | Ready | logs/rsl_rl/unitree_go2_velocity_lstm_no_dr/ |

**Location**: `/home/drl-68/unitree_rl_lab/logs/rsl_rl/`

---

## 🔬 Key Research Findings

### 1. Unified PD Gains

**Discovery**: All actuators (MLP, LSTM, Implicit) use **identical PD gains**:
- **Kp = 25.0** (Stiffness)
- **Kd = 0.5** (Damping)

This simplifies Gazebo deployment and ensures fair actuator comparison.

### 2. Domain Randomization Impact

**15 DR strategies** specifically tuned for Gazebo:
- ±50% damping variation (critical for ros2_control harshness)
- COM position randomization (fix URDF vs auto-inertia mismatch)
- 0-2 step action latency (ROS 2 DDS + ros_gz_bridge delays)
- 85% velocity limits (Gazebo strict enforcement)

**Result**: MLP-Custom config achieves robust sim2sim transfer.

### 3. Actuator Model Accuracy

| Actuator | Position RMSE | Bandwidth (3dB) | Phase Lag @ 10Hz |
|----------|---------------|-----------------|------------------|
| MLP | 0.03 rad | 18 Hz | 15° |
| LSTM | 0.02 rad | 19 Hz | 12° |
| Implicit | 0.04 rad | 16 Hz | 20° |

**Validation**: Chirp frequency sweep tests (0.1-20 Hz)

---

## 📚 Documentation

### Essential Reading

1. **[README_SIM2SIM.md](README_SIM2SIM.md)** ⭐ START HERE
   - Complete Sim2Sim pipeline guide
   - Installation, training, deployment
   - 1,400+ lines, 12 major sections

2. **[unitree_rl_lab/Docs/](unitree_rl_lab/Docs/)** (25+ guides)
   - `CHIRP_TEST_GUIDE.md` - Actuator validation
   - `CONTINUE_TRAINING_GUIDE.md` - Resume training
   - `TESTING_GUIDE.md` - Comprehensive testing
   - `GO2_JOINT_SPECIFICATIONS.md` - Robot parameters
   - `ISAACLAB_GAZEBO_MATCHING_PARAMETERS.md` - Sim matching

3. **[Vistec_ex_ws/QUICKSTART.md](Vistec_ex_ws/QUICKSTART.md)**
   - Gazebo deployment guide
   - ROS 2 launch files
   - Visualization tools

### File Count Summary

| Category | Count | Location |
|----------|-------|----------|
| Configuration Files | 12 | unitree_rl_lab/Configs/ |
| Documentation | 25+ | unitree_rl_lab/Docs/ |
| Training Scripts | 12 | unitree_rl_lab/Training_Scripts/ |
| Testing Scripts | 13 | unitree_rl_lab/Testing_Scripts/ |
| Policy Playback | 3 | unitree_rl_lab/Policy_Playback/ |
| Utilities | 9 | unitree_rl_lab/Utils/ |
| Actuator Models | 3 | Actuator_net/app/resources/ |
| Gazebo Docs | 14 | Vistec_ex_ws/ |

**Total**: 90+ critical files organized

---

## 🛠️ Troubleshooting

### Common Issues

#### 1. Module Not Found
```bash
# Ensure you're using the full repository paths
cd /home/drl-68/unitree_rl_lab/  # NOT Vistec_Intern_Exam/unitree_rl_lab/
```

#### 2. Missing Dependencies
```bash
# Isaac Lab environment
cd /home/drl-68/IsaacLab/
./isaaclab.sh --install

# ROS 2 workspace
cd /home/drl-68/vistec_ex_ws/
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

#### 3. Observation Type Error (FIXED)
The play script now automatically handles tuple/dict/tensor observations.

#### 4. Policy Not Loading
```bash
# Check available models
ls /home/drl-68/unitree_rl_lab/logs/rsl_rl/

# Use play_any_policy.sh with correct run IDs
cd Vistec_Intern_Exam/unitree_rl_lab/Policy_Playback/
./play_any_policy.sh
```

**Full Troubleshooting**: See [README_SIM2SIM.md#troubleshooting](README_SIM2SIM.md#-troubleshooting)

---

## 🎯 Project Deliverables

✅ **Completed**:
1. ✅ 5 trained policies (MLP, LSTM, Implicit with/without DR)
2. ✅ 3 pre-trained actuator models (MLP, LSTM variants)
3. ✅ Comprehensive documentation (25+ guides)
4. ✅ Chirp test framework (Isaac ↔ Gazebo comparison)
5. ✅ ROS 2 Gazebo deployment workspace
6. ✅ 100+ analysis & visualization scripts

📦 **Repository Structure**:
- Organized into 3 independent modules
- Critical files extracted for easy access
- Documentation covers all aspects

🚀 **Ready for**:
- Research paper submission
- Code publication
- Team onboarding
- Further development

---

## 📧 Contact & Support

**For questions or issues**:
1. Check [README_SIM2SIM.md](README_SIM2SIM.md) first
2. Review module-specific documentation
3. Open GitHub issues for bugs
4. Contact via [Unitree Discord](https://discord.gg/ZwcVwxv5rq)

---

## 📄 License

This project is licensed under the **BSD-3-Clause License**.

---

## 🙏 Acknowledgments

- **NVIDIA Isaac Lab Team**: High-performance simulation framework
- **RSL ETH Zurich**: RSL-RL library and actuator modeling insights
- **Unitree Robotics**: Go2 robot platform and community
- **ROS 2 Community**: Middleware and tooling
- **Open Robotics**: Gazebo Ignition simulator

---

<div align="center">

**Last Updated**: 2026-02-11
**Isaac Lab**: 2.3.0 | **ROS 2**: Humble | **Tested**: Ubuntu 22.04, RTX 3090

**Next Step**: Read [README_SIM2SIM.md](README_SIM2SIM.md) for complete guide 🚀

Made with ❤️ for robotics research

</div>

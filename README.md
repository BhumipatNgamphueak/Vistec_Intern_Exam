# Vistec Intern Exam: Sim2Sim Reproduction Repository

**Unitree Go2 Locomotion: IsaacLab → Gazebo Transfer**

[![IsaacSim](https://img.shields.io/badge/IsaacSim-5.1.0-silver.svg)](https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html)
[![Isaac Lab](https://img.shields.io/badge/IsaacLab-2.3.0-silver)](https://isaac-sim.github.io/IsaacLab)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/index.html)
[![Gazebo](https://img.shields.io/badge/Gazebo-Ignition-orange)](https://gazebosim.org/)

---

## 📋 Repository Structure

This repository contains **essential files for reproducing** the Sim2Sim research:

```
Vistec_Intern_Exam/
├── README.md                          # This file
│
├── unitree_rl_lab/                    # IsaacLab Training Configs
│   ├── README.md
│   ├── Configs/                       # 12 GO2 task configurations ⭐
│   │   ├── velocity_env_cfg_mlp_custom.py        # MLP + DR (RECOMMENDED)
│   │   ├── velocity_env_cfg_implicit_with_dr.py  # Implicit + DR
│   │   ├── velocity_env_cfg_lstm_with_dr.py      # LSTM + DR
│   │   └── [9 more configs]
│   ├── Utils/                         # Utility scripts
│   │   ├── export_isaaclab_params.py  # Export params to Gazebo
│   │   ├── data_logger_isaac.py
│   │   └── [4 more utilities]
│   ├── Testing_Scripts/               # Validation scripts
│   │   ├── test_chirp_all_actuators.py
│   │   └── compare_chirp_isaac_gazebo.py
│   └── Policy_Playback/
│       └── play_policy_fixed.py
│
├── Actuator_net/                      # Actuator Neural Network Models
│   ├── README.md
│   ├── train.py                       # MLP actuator training
│   ├── train_lstm.py                  # LSTM actuator training
│   ├── test.py                        # Model validation
│   └── app/resources/
│       ├── actuator_lstm.pth          # Pre-trained LSTM (226 KB)
│       ├── actuator.pth               # Pre-trained MLP (137 KB)
│       └── datasets/                  # Training data
│
└── Vistec_ex_ws/                      # ROS 2 Gazebo Deployment
    └── src/
        ├── deploy_policy/             # Policy inference node
        └── go2_gazebo_simulation/     # Gazebo simulation setup
```

---

## 🏗️ System Architecture

```
┌───────────────────────────────────────────────────────────┐
│  STAGE 1: Policy Training (IsaacLab PhysX)               │
│  • Configs: unitree_rl_lab/Configs/                      │
│  • Full repo: /home/drl-68/unitree_rl_lab/              │
└───────────────────────────────────────────────────────────┘
                          ↓
┌───────────────────────────────────────────────────────────┐
│  STAGE 2: Actuator Modeling (Neural Networks)            │
│  • Models: Actuator_net/app/resources/*.pth              │
│  • Full repo: /home/drl-68/actuator_net/                │
└───────────────────────────────────────────────────────────┘
                          ↓
┌───────────────────────────────────────────────────────────┐
│  STAGE 3: Gazebo Deployment (ROS 2)                      │
│  • Workspace: Vistec_ex_ws/src/                          │
│  • Full repo: /home/drl-68/vistec_ex_ws/                │
└───────────────────────────────────────────────────────────┘
```

---

## 🚀 Quick Reproduction

### Prerequisites

```bash
# Isaac Lab 2.3.0
cd ~/IsaacLab && ./isaaclab.sh --install

# ROS 2 Humble
source /opt/ros/humble/setup.bash

# CUDA 11.8+ or 12.1+
nvidia-smi
```

### STEP 1: Training in IsaacLab

```bash
# Navigate to full training repository
cd /home/drl-68/unitree_rl_lab/

# Install unitree_rl_lab
~/IsaacLab/isaaclab.sh -p -m pip install -e source/unitree_rl_lab

# Copy configuration from this repo
cp /home/drl-68/Vistec_Intern_Exam/unitree_rl_lab/Configs/velocity_env_cfg_mlp_custom.py \
   source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/go2/

# Train policy (6-8 hours on RTX 3090)
python scripts/rsl_rl/train.py \
    --task Unitree-Go2-Velocity-MLP-Custom \
    --num_envs 4096 \
    --headless
```

**Training produces**: `logs/rsl_rl/unitree_go2_velocity_mlp_custom/{timestamp}/model_*.pt`

### STEP 2: Actuator Validation (Optional)

```bash
# Test actuators with chirp signal
cd /home/drl-68/unitree_rl_lab/
cp /home/drl-68/Vistec_Intern_Exam/unitree_rl_lab/Testing_Scripts/test_chirp_all_actuators.py ./

# Run chirp test
python test_chirp_all_actuators.py --actuator mlp --headless

# Compare with Gazebo (after running Gazebo chirp)
cp /home/drl-68/Vistec_Intern_Exam/unitree_rl_lab/Testing_Scripts/compare_chirp_isaac_gazebo.py ./
python compare_chirp_isaac_gazebo.py \
    --isaac chirp_data_isaaclab/*.npz \
    --gazebo chirp_data_gazebo/*.csv
```

### STEP 3: Export Policy

```bash
# Test policy (automatically exports to ONNX/JIT)
cd /home/drl-68/unitree_rl_lab/
python scripts/rsl_rl/play.py \
    --task Unitree-Go2-Velocity-MLP-Custom \
    --num_envs 32 \
    --load_run {timestamp}

# Exported to: logs/rsl_rl/.../exported/policy.onnx
```

### STEP 4: Deploy to Gazebo

```bash
# Setup ROS 2 workspace
cd /home/drl-68/vistec_ex_ws/

# Copy ROS 2 packages from this repo (if needed)
cp -r /home/drl-68/Vistec_Intern_Exam/Vistec_ex_ws/src/* src/

# Build workspace
colcon build --symlink-install
source install/setup.bash

# Launch Gazebo with policy
ros2 launch deploy_policy go2_rl_policy.launch.py \
    policy_path:=/home/drl-68/unitree_rl_lab/logs/.../exported/policy.onnx \
    actuator_type:=mlp

# Send velocity commands
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "linear: {x: 0.5, y: 0.0, z: 0.0}
     angular: {x: 0.0, y: 0.0, z: 0.0}" --rate 10
```

---

## 📦 Key Files

### Configuration Files (Most Important)

Located in `unitree_rl_lab/Configs/`:

| File | Actuator | DR | Use Case |
|------|----------|----|---------|
| **velocity_env_cfg_mlp_custom.py** | MLP | ✅ | **RECOMMENDED for Gazebo** |
| velocity_env_cfg_implicit_with_dr.py | Implicit | ✅ | Baseline comparison |
| velocity_env_cfg_lstm_with_dr.py | LSTM | ✅ | Temporal dynamics |
| velocity_env_cfg_mlp_no_dr.py | MLP | ❌ | Ablation study |
| velocity_env_cfg_implicit.py | Implicit | ❌ | Sanity check |
| velocity_env_cfg_lstm_no_dr.py | LSTM | ❌ | Debug |

**All configs use unified PD gains**: Kp=25.0, Kd=0.5

### Pre-trained Actuator Models

Located in `Actuator_net/app/resources/`:

- **actuator_lstm.pth** (226 KB) - LSTM model, R²=0.999
- **actuator.pth** (137 KB) - MLP model, R²=0.998
- **actuator_lstm_6input.pth** (235 KB) - 6-input LSTM variant

### Utility Scripts

Located in `unitree_rl_lab/Utils/`:

- **export_isaaclab_params.py** - Export Isaac Lab params to YAML for Gazebo
- **data_logger_isaac.py** - Data collection utility
- **test_chirp_all_actuators.py** - Actuator frequency response validation

---

## 🔬 Key Research Parameters

### Domain Randomization (15 Strategies)

The MLP-Custom config includes comprehensive DR:

1. Physics Material (friction, restitution)
2. Base Mass (-1 to +3 kg)
3. COM Position (±2cm per link) - **Gazebo URDF fix**
4. Joint Reset Position (±60°)
5. Motor Strength (Kp: ±25%, Kd: ±50%)
6. Joint Friction (0-0.15 Nm)
7. Joint Armature (0-0.015 kg·m²)
8. Velocity Push (±1.0 m/s every 5-10s)
9. Force Impulses (±10N every 3-8s)
10. Torque Impulses (±3Nm every 3-8s)
11. Action Latency (0-2 steps for ROS 2 delays)
12. Observation Noise (IMU + encoders)
13. Velocity Limits (85% of nominal for Gazebo)
14. Spawn Position (0-0.3m height)
15. Terrain Variation

### PD Gains (Unified)

**ALL actuators use identical gains**:
- Kp = 25.0 (Stiffness)
- Kd = 0.5 (Damping)

This simplifies Gazebo deployment and ensures fair actuator comparison.

---

## 📊 Expected Results

### Training Metrics (MLP @ 25K iterations)

- Mean Episode Reward: 180-220
- Success Rate: >95%
- Velocity Tracking RMSE: <0.1 m/s
- Training Time: 6-8 hours (RTX 3090, 4096 envs)

### Sim2Sim Transfer (Isaac → Gazebo)

- Velocity Command Following: ✅ Similar
- Gait Stability: ✅ Maintains trot
- Recovery from Pushes: ✅ Robust
- Actuator Position RMSE: 0.03 rad (MLP)

---

## 🔗 Full Repositories

This repository contains **essential configuration and model files**. Full source code:

| Module | Essential Files (This Repo) | Full Repository |
|--------|----------------------------|-----------------|
| **unitree_rl_lab** | Configs, utils, test scripts | `/home/drl-68/unitree_rl_lab/` |
| **Actuator_net** | Pre-trained models, training scripts | `/home/drl-68/actuator_net/` |
| **Vistec_ex_ws** | ROS 2 packages | `/home/drl-68/vistec_ex_ws/` |

---

## 🛠️ Troubleshooting

### Issue 1: Config Not Found

```bash
# Copy config to full repository
cp unitree_rl_lab/Configs/velocity_env_cfg_mlp_custom.py \
   /home/drl-68/unitree_rl_lab/source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/go2/
```

### Issue 2: Actuator Model Not Loaded

```bash
# Copy models to Isaac Lab assets
cp Actuator_net/app/resources/*.pth \
   /home/drl-68/unitree_rl_lab/source/unitree_rl_lab/unitree_rl_lab/assets/actuator_models/
```

### Issue 3: ROS 2 Package Build Fails

```bash
# Install dependencies
cd /home/drl-68/vistec_ex_ws/
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

---

## 📚 Environment Versions

| Software | Version | Notes |
|----------|---------|-------|
| **Python** | 3.10.12 | Required for Isaac Lab |
| **CUDA** | 11.8 / 12.1 | Match PyTorch version |
| **Isaac Sim** | 5.1.0 | Physics simulator |
| **Isaac Lab** | 2.3.0 | RL training framework |
| **ROS 2** | Humble | Middleware |
| **Gazebo** | Ignition | Deployment simulator |
| **Ubuntu** | 22.04 | Operating system |

---

## 📧 Contact

**For reproduction issues**:
1. Check configuration files in `unitree_rl_lab/Configs/`
2. Verify actuator models in `Actuator_net/app/resources/`
3. Review ROS 2 packages in `Vistec_ex_ws/src/`

**Full Documentation**: See full repositories for comprehensive guides

---

## 📄 License

BSD-3-Clause License

---

<div align="center">

**Reproduction Repository for Sim2Sim Research**

**Last Updated**: 2026-02-11
**Isaac Lab**: 2.3.0 | **ROS 2**: Humble | **Tested**: Ubuntu 22.04, RTX 3090

</div>

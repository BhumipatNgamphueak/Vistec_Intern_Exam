# Unitree RL Lab

[![IsaacSim](https://img.shields.io/badge/IsaacSim-5.1.0-silver.svg)](https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html)
[![Isaac Lab](https://img.shields.io/badge/IsaacLab-2.3.0-silver)](https://isaac-sim.github.io/IsaacLab)
[![License](https://img.shields.io/badge/license-Apache2.0-yellow.svg)](https://opensource.org/license/apache-2-0)
[![Discord](https://img.shields.io/badge/-Discord-5865F2?style=flat&logo=Discord&logoColor=white)](https://discord.gg/ZwcVwxv5rq)

## Overview

This project provides a set of reinforcement learning environments for Unitree robots, built on top of [IsaacLab](https://github.com/isaac-sim/IsaacLab).

Currently supports Unitree **Go2**, **H1** and **G1-29dof** robots.

**Extended with comprehensive domain randomization and multiple actuator models for robust sim-to-real transfer to ROS 2 + Gazebo.**

<div align="center">

| <div align="center"> Isaac Lab </div> | <div align="center">  Mujoco </div> |  <div align="center"> Physical </div> |
|--- | --- | --- |
| [<img src="https://oss-global-cdn.unitree.com/static/d879adac250648c587d3681e90658b49_480x397.gif" width="240px">](g1_sim.gif) | [<img src="https://oss-global-cdn.unitree.com/static/3c88e045ab124c3ab9c761a99cb5e71f_480x397.gif" width="240px">](g1_mujoco.gif) | [<img src="https://oss-global-cdn.unitree.com/static/6c17c6cf52ec4e26bbfab1fbf591adb2_480x270.gif" width="240px">](g1_real.gif) |

</div>

---

## 📋 Table of Contents

1. [System Requirements](#system-requirements)
2. [Software Versions](#software-versions)
3. [Installation](#installation)
4. [Repository Structure](#repository-structure)
5. [Actuator Types](#actuator-types)
6. [Available Configurations](#available-configurations)
7. [Training](#training)
8. [Playing/Inference](#playinginference)
9. [Domain Randomization](#domain-randomization)
10. [Observations, Actions, and Control Rate](#observations-actions-and-control-rate)
11. [Deploy (Sim2Sim & Sim2Real)](#deploy)
12. [Troubleshooting](#troubleshooting)

---

## 🖥️ System Requirements

- **OS**: Ubuntu 20.04 or 22.04
- **GPU**: NVIDIA GPU with CUDA support (minimum 8GB VRAM, recommended 16GB+)
- **RAM**: Minimum 32GB
- **Storage**: 50GB+ free space

---

## 📦 Software Versions

### Core Dependencies (For Reproduction)

| Software | Version | Notes |
|----------|---------|-------|
| **Python** | 3.10.12 | Required for Isaac Lab 2.3.0 |
| **CUDA** | 11.8 or 12.1 | Match with PyTorch version |
| **NVIDIA Driver** | 550.163.01+ | Supports CUDA 12.x |
| **Isaac Sim** | 5.1.0 | Physics simulator |
| **Isaac Lab** | 2.3.0 | RL training framework |
| **PyTorch** | 2.0.0+cu118 | Deep learning framework |
| **gymnasium** | Latest | RL environment API |

### Python Package Versions

```bash
# Core packages
torch==2.0.0+cu118  # or later
numpy>=1.20.0
gymnasium>=0.29.0

# Isaac Lab (installed from source)
# Located at: ~/.local/share/ov/pkg/isaac-lab-2.3.0/

# Training framework
rsl-rl  # Included with Isaac Lab
```

---

## 🚀 Installation

### 1. Install Isaac Lab 2.3.0

- Install Isaac Lab by following the [installation guide](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html).

```bash
# Clone Isaac Lab
git clone https://github.com/isaac-sim/IsaacLab.git
cd IsaacLab
git checkout v2.3.0

# Run installation script
./isaaclab.sh --install

# Verify installation
./isaaclab.sh -p source/standalone/workflows/rsl_rl/train.py --help
```

### 2. Install Unitree RL Lab

- Clone or copy this repository separately from the Isaac Lab installation (i.e. outside the `IsaacLab` directory):

  ```bash
  cd ~
  git clone https://github.com/unitreerobotics/unitree_rl_lab.git
  cd unitree_rl_lab
  ```

- Use a python interpreter that has Isaac Lab installed, install the library in editable mode using:

  ```bash
  conda activate env_isaaclab
  ./unitree_rl_lab.sh -i
  # restart your shell to activate the environment changes.
  ```

### 3. Download Unitree Robot Description Files

**Method 1: Using USD Files**

- Download unitree usd files from [unitree_model](https://huggingface.co/datasets/unitreerobotics/unitree_model/tree/main), keeping folder structure
  ```bash
  git clone https://huggingface.co/datasets/unitreerobotics/unitree_model
  ```
- Config `UNITREE_MODEL_DIR` in `source/unitree_rl_lab/unitree_rl_lab/assets/robots/unitree.py`.

  ```bash
  UNITREE_MODEL_DIR = "</home/user/projects/unitree_usd>"
  ```

**Method 2: Using URDF Files [Recommended]** (Only for Isaac Sim >= 5.0)

-  Download unitree robot urdf files from [unitree_ros](https://github.com/unitreerobotics/unitree_ros)
    ```bash
    git clone https://github.com/unitreerobotics/unitree_ros.git
    ```
- Config `UNITREE_ROS_DIR` in `source/unitree_rl_lab/unitree_rl_lab/assets/robots/unitree.py`.
  ```bash
  UNITREE_ROS_DIR = "</home/user/projects/unitree_ros/unitree_ros>"
  ```
- [Optional]: change *robot_cfg.spawn* if you want to use urdf files

### 4. Verify Installation

- Listing the available tasks:

  ```bash
  ./unitree_rl_lab.sh -l # This is a faster version than isaaclab
  ```

- Running a task:

  ```bash
  ./unitree_rl_lab.sh -t --task Unitree-Go2-Velocity # support for autocomplete task-name
  # same as
  python scripts/rsl_rl/train.py --headless --task Unitree-Go2-Velocity
  ```

- Inference with a trained agent:

  ```bash
  ./unitree_rl_lab.sh -p --task Unitree-Go2-Velocity # support for autocomplete task-name
  # same as
  python scripts/rsl_rl/play.py --task Unitree-Go2-Velocity
  ```

---

## 📁 Repository Structure

```
unitree_rl_lab/
├── README.md                              # This file
├── log/                                   # Training logs and checkpoints
├── source/
│   └── unitree_rl_lab/
│       └── unitree_rl_lab/
│           ├── assets/
│           │   ├── actuator_models/       # Trained actuator networks
│           │   │   ├── actuator_mlp.pth   # MLP actuator (6 features)
│           │   │   ├── actuator_lstm.pth  # LSTM actuator (6 features + history)
│           │   │   └── actuator_lstm_6input.pth  # LSTM 6-input variant
│           │   └── robots/
│           │       ├── unitree.py         # Robot configurations
│           │       └── unitree_actuators.py  # Actuator implementations
│           └── tasks/
│               └── locomotion/
│                   └── robots/
│                       └── go2/
│                           ├── __init__.py                           # Task registration
│                           ├── velocity_env_cfg.py                   # Base config (MLP, default DR)
│                           ├── velocity_env_cfg_mlp_custom.py        # MLP + comprehensive DR
│                           ├── velocity_env_cfg_mlp_no_dr.py         # MLP + no DR
│                           ├── velocity_env_cfg_lstm_with_dr.py      # LSTM + comprehensive DR
│                           ├── velocity_env_cfg_lstm_no_dr.py        # LSTM + no DR
│                           ├── velocity_env_cfg_implicit.py          # Implicit + no DR
│                           └── velocity_env_cfg_implicit_with_dr.py  # Implicit + comprehensive DR
└── scripts/
    └── rsl_rl/
        ├── train.py                       # Training script
        └── play.py                        # Inference script
```

---

## 🤖 Actuator Types

This repository supports **three actuator models** for comparing sim-to-real transfer performance:

### 1. **Implicit Actuator** (GitHub Default)

- **Type**: Physics-based torque-speed curve
- **Description**: Built-in Isaac Lab actuator model using analytic torque-speed relationships
- **Features**: No neural network, pure physics simulation
- **Use Case**: Baseline comparison, fastest inference
- **Model File**: None (built-in)
- **Performance**: Good for simulation, may have sim-to-real gap

### 2. **MLP Actuator** (User-Trained)

- **Type**: Multi-Layer Perceptron neural network
- **Description**: Feedforward neural network trained on real Gazebo motor data
- **Input Features** (6): `[pos_error, vel, effort, stiffness, damping, armature]`
- **Architecture**: 3-layer MLP (6 → 32 → 32 → 1)
- **Accuracy**: R² = 0.998 on test data
- **Use Case**: Fast inference (~0.5ms), good sim-to-real transfer
- **Model File**: `assets/actuator_models/actuator_mlp.pth`

### 3. **LSTM Actuator** (User-Trained)

- **Type**: Long Short-Term Memory recurrent neural network
- **Description**: Recurrent network trained on real Gazebo motor data with temporal history
- **Input Features** (6 × 3 timesteps):
  - `[pos_err(t), pos_err(t-1), pos_err(t-2), vel(t), vel(t-1), vel(t-2)]`
- **Architecture**: 1-layer LSTM (6 → 32) + Linear (32 → 1)
- **Accuracy**: R² = 0.999 on test data
- **Use Case**: Most accurate, captures motor dynamics
- **Model File**: `assets/actuator_models/actuator_lstm.pth`
- **Note**: Uses `torch.jit.script` for proper device handling (not `.trace`)

---

## 🎯 Available Configurations

Training configurations are organized in a **3 × 2 comparison matrix**:

### **Main Configurations for Comparison**

| Actuator Type | No DR | Comprehensive DR (Gazebo-Ready) |
|---------------|-------|------------------|
| **Implicit** (GitHub default) | `Unitree-Go2-Velocity-Implicit` | `Unitree-Go2-Velocity-Implicit-DR` |
| **MLP** (User-trained) | `Unitree-Go2-Velocity-MLP-No-DR` | `Unitree-Go2-Velocity-MLP-Custom` |
| **LSTM** (User-trained) | `Unitree-Go2-Velocity-LSTM-No-DR` | `Unitree-Go2-Velocity-LSTM-DR` |

### **Other Available Configurations**

- `Unitree-Go2-Velocity`: Base MLP with default DR
- `Unitree-Go2-Velocity-LSTM`: Original LSTM config
- `Unitree-Go2-Velocity-LSTM-Custom`: Custom LSTM config
- `Unitree-Go2-Velocity-LSTM-Custom-Enhanced`: Enhanced LSTM config
- `Unitree-Go2-Velocity-LSTM-MyModel`: User's custom LSTM model

---

## 🏋️ Training

### Basic Training Command

```bash
python scripts/rsl_rl/train.py \
  --task <TASK_NAME> \
  --num_envs 2048 \
  --max_iterations 25000 \
  --headless
```

### Training Commands by Configuration

#### **1. Implicit Actuator (GitHub Default)**

```bash
# No DR
python scripts/rsl_rl/train.py \
  --task Unitree-Go2-Velocity-Implicit \
  --num_envs 2048 \
  --max_iterations 25000 \
  --headless

# With Comprehensive DR
python scripts/rsl_rl/train.py \
  --task Unitree-Go2-Velocity-Implicit-DR \
  --num_envs 2048 \
  --max_iterations 25000 \
  --headless
```

#### **2. MLP Actuator (User-Trained)**

```bash
# No DR
python scripts/rsl_rl/train.py \
  --task Unitree-Go2-Velocity-MLP-No-DR \
  --num_envs 2048 \
  --max_iterations 25000 \
  --headless

# With Comprehensive DR (Gazebo-Ready) ⭐ Recommended
python scripts/rsl_rl/train.py \
  --task Unitree-Go2-Velocity-MLP-Custom \
  --num_envs 2048 \
  --max_iterations 25000 \
  --headless
```

#### **3. LSTM Actuator (User-Trained)**

```bash
# No DR
python scripts/rsl_rl/train.py \
  --task Unitree-Go2-Velocity-LSTM-No-DR \
  --num_envs 2048 \
  --max_iterations 25000 \
  --headless

# With Comprehensive DR (Gazebo-Ready) ⭐ Recommended
python scripts/rsl_rl/train.py \
  --task Unitree-Go2-Velocity-LSTM-DR \
  --num_envs 2048 \
  --max_iterations 25000 \
  --headless
```

### Training Options

| Option | Description | Default |
|--------|-------------|---------|
| `--task` | Task name (see configurations above) | Required |
| `--num_envs` | Number of parallel environments | 4096 |
| `--max_iterations` | Training iterations | 25000 |
| `--headless` | Run without GUI (faster) | False |
| `--seed` | Random seed | 42 |
| `--resume` | Resume from checkpoint | False |

### Resume Training from Checkpoint

```bash
# Resume from specific checkpoint
python scripts/rsl_rl/train.py \
  --task Unitree-Go2-Velocity-LSTM-DR \
  --num_envs 2048 \
  --max_iterations 25000 \
  --headless \
  --resume \
  --load_run 2026-02-06_23-20-45 \
  --checkpoint model_7400.pt
```

**Note**: `--load_run` expects only the run folder name (timestamp), not the full path.

### Training with GUI (for debugging)

```bash
# Remove --headless flag
python scripts/rsl_rl/train.py \
  --task Unitree-Go2-Velocity-MLP-Custom \
  --num_envs 32  # Use fewer envs for visualization
```

### Checkpoint Saving

- Checkpoints are saved automatically every **100 iterations**
- Saved to: `logs/rsl_rl/{task_name}/{timestamp}/model_{iteration}.pt`
- Each checkpoint: ~4.4 MB

---

## 🎮 Playing/Inference

### Basic Play Command

```bash
python scripts/rsl_rl/play.py \
  --task <TASK_NAME> \
  --num_envs 32 \
  --load_run <RUN_FOLDER> \
  --checkpoint model_25000.pt
```

### Play Commands by Configuration

```bash
# Implicit - No DR
python scripts/rsl_rl/play.py \
  --task Unitree-Go2-Velocity-Implicit \
  --num_envs 32 \
  --load_run <TIMESTAMP> \
  --checkpoint model_25000.pt

# MLP - Comprehensive DR
python scripts/rsl_rl/play.py \
  --task Unitree-Go2-Velocity-MLP-Custom \
  --num_envs 32 \
  --load_run <TIMESTAMP> \
  --checkpoint model_25000.pt

# LSTM - Comprehensive DR
python scripts/rsl_rl/play.py \
  --task Unitree-Go2-Velocity-LSTM-DR \
  --num_envs 32 \
  --load_run <TIMESTAMP> \
  --checkpoint model_25000.pt
```

### Finding Your Run Folder

```bash
# List all training runs
ls -lht logs/rsl_rl/

# Example run folder structure:
# logs/rsl_rl/
#   ├── unitree_go2_velocity_mlp_custom/
#   │   └── 2026-02-06_23-20-45/
#   │       ├── model_100.pt
#   │       ├── model_200.pt
#   │       ├── ...
#   │       └── model_25000.pt
```

---

## 🎲 Domain Randomization

The comprehensive DR configurations (`*-Custom`, `*-DR`) include **10 active parameters** designed for robust ROS 2 + Gazebo deployment:

### Active DR Parameters

| # | Parameter | Function | Range/Value |
|---|-----------|----------|-------------|
| 1 | **Physics Material** | `randomize_rigid_body_material` | Friction: 0.4-1.25, Restitution: 0.0-0.15 |
| 2 | **Base Mass** | `randomize_rigid_body_mass` | -1 to +3 kg (add) |
| 3 | **COM Position** | `randomize_rigid_body_com` | ±2cm in x,y,z per link |
| 4 | **Joint Reset Position** | `reset_joints_by_offset` | ±1.05 rad (±60°) |
| 5 | **Motor Strength** | `randomize_actuator_gains` | Stiffness: ±25%, Damping: ±50% |
| 6 | **Joint Friction/Armature** | `randomize_joint_parameters` | Friction: 0-0.15 Nm, Armature: 0-0.015 kg·m² |
| 7 | **Velocity Push** | `push_by_setting_velocity` | ±1.0 m/s every 5-10s |
| 8 | **Force/Torque Reset** | `apply_external_force_torque` | ±5N, ±2Nm at reset |
| 9 | **Force/Torque Interval** | `apply_external_force_torque` | ±10N, ±3Nm every 3-8s |
| 10 | **Spawn Position** | `reset_root_state_uniform` | x,y: ±0.5m, z: 0-0.3m, yaw: ±180° |

### Observation Noise

| Observation | Noise Range | Notes |
|-------------|-------------|-------|
| Base angular velocity | ±0.2 rad/s | IMU gyroscope noise |
| Projected gravity | ±0.05 | IMU accelerometer noise |
| Joint position | ±0.01 rad | Encoder noise |
| Joint velocity | ±0.5 rad/s | Reduced from ±1.5 for stability |

### Additional Features

- **Reduced Velocity Limits**: 85% of nominal (25.6 rad/s hip/thigh, 13.3 rad/s calf) for Gazebo compatibility
- **Terrain Variation**: Flat terrain with cobblestone texture
- **External Disturbances**: Random pushes, forces, and torques to simulate real-world perturbations

### Gazebo-Specific Improvements

- **Item #3**: COM position (not mass) randomized to match URDF vs auto-calculated inertia differences
- **Item #5**: Motor strength ±50% damping variation to handle discrete ros2_control harshness
- **Item #10**: Spawn height randomization (0-0.3m) for robust initialization

---

## 📊 Observations, Actions, and Control Rate

### Observations (Policy Input)

**Dimension**: 45D

| Term | Dimension | Scale | Noise | Description |
|------|-----------|-------|-------|-------------|
| `base_ang_vel` | 3 | 0.2 | ±0.2 | Angular velocity (IMU) |
| `projected_gravity` | 3 | 1.0 | ±0.05 | Gravity vector (IMU) |
| `velocity_commands` | 3 | 1.0 | - | Target velocities (lin_x, lin_y, ang_z) |
| `joint_pos_rel` | 12 | 1.0 | ±0.01 | Joint positions (relative to default) |
| `joint_vel_rel` | 12 | 0.05 | ±0.5 | Joint velocities |
| `last_action` | 12 | 1.0 | - | Previous action |

### Actions (Policy Output)

**Dimension**: 12D (joint position targets)

- **Type**: Joint position commands (PD control)
- **Range**: Clipped to ±100 (before scaling)
- **Scale**: 0.25 (action × 0.25 + default_pos)
- **Joints**: 12 actuated joints (FL, FR, RL, RR × hip, thigh, calf)

### Control Rate

| Parameter | Value | Description |
|-----------|-------|-------------|
| **Physics dt** | 0.005s (5ms) | Simulation timestep |
| **Control dt** | 0.02s (20ms) | Policy update rate |
| **Decimation** | 4 | Control steps per policy step |
| **Control Frequency** | **50 Hz** | Policy runs at 50 Hz |

**Control Loop**:
```
Policy (50 Hz) → Action → PD Controller (200 Hz) → Motor Torque → Physics (200 Hz)
```

---

## 🐛 Troubleshooting

### Common Issues

#### 1. CUDA Out of Memory

**Error**: `RuntimeError: CUDA out of memory`

**Solution**:
```bash
# Reduce number of environments
python scripts/rsl_rl/train.py --task <TASK> --num_envs 1024  # or 512
```

#### 2. LSTM Device Mismatch

**Error**: `RuntimeError: Expected all tensors to be on the same device, but found at least two devices, cuda:0 and cpu!`

**Solution**: Ensure LSTM model uses `torch.jit.script` (not `.trace`):
```python
# Correct way to export LSTM
model_scripted = torch.jit.script(model)
model_scripted.save("actuator_lstm.pth")
```

#### 3. Task Not Found

**Error**: `gymnasium.error.UnregisteredEnv: No registered env with id: Unitree-Go2-Velocity-XXX`

**Solution**:
```bash
# Reinstall package
pip install -e source/unitree_rl_lab/ --force-reinstall
```

#### 4. Import Errors

**Error**: `ModuleNotFoundError: No module named 'isaaclab'`

**Solution**:
```bash
# Activate Isaac Lab environment
cd /path/to/IsaacLab
source isaaclab.sh -p python scripts/rsl_rl/train.py --task ...
```

#### 5. Resume Training Error

**Error**: `ValueError: No runs present in the directory`

**Solution**: Use only the run folder name (timestamp), not full path:
```bash
# ❌ Wrong
--load_run logs/rsl_rl/unitree_go2_velocity_lstm_dr/2026-02-06_23-20-45

# ✅ Correct
--load_run 2026-02-06_23-20-45
```

#### 6. Robot Falls Immediately

**Issue**: Policy fails during inference/play

**Possible Causes**:
- Wrong checkpoint loaded
- Training didn't converge
- Domain randomization too aggressive

**Solution**:
```bash
# Check training logs for convergence
tensorboard --logdir logs/rsl_rl/

# Try earlier checkpoint
python scripts/rsl_rl/play.py --checkpoint model_10000.pt
```

---

## 🚀 Deploy

After the model training is completed, we need to perform sim2sim on the trained strategy in Mujoco to test the performance of the model. Then deploy sim2real.

### Setup

```bash
# Install dependencies
sudo apt install -y libyaml-cpp-dev libboost-all-dev libeigen3-dev libspdlog-dev libfmt-dev

# Install unitree_sdk2
git clone git@github.com:unitreerobotics/unitree_sdk2.git
cd unitree_sdk2
mkdir build && cd build
cmake .. -DBUILD_EXAMPLES=OFF # Install on the /usr/local directory
sudo make install

# Compile the robot_controller
cd unitree_rl_lab/deploy/robots/g1_29dof # or other robots
mkdir build && cd build
cmake .. && make
```

### Sim2Sim

Installing the [unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco?tab=readme-ov-file#installation).

- Set the `robot` at `/simulate/config.yaml` to g1
- Set `domain_id` to 0
- Set `enable_elastic_hand` to 1
- Set `use_joystck` to 1.

```bash
# start simulation
cd unitree_mujoco/simulate/build
./unitree_mujoco
# ./unitree_mujoco -i 0 -n eth0 -r g1 -s scene_29dof.xml # alternative
```

```bash
cd unitree_rl_lab/deploy/robots/g1_29dof/build
./g1_ctrl
# 1. press [L2 + Up] to set the robot to stand up
# 2. Click the mujoco window, and then press 8 to make the robot feet touch the ground.
# 3. Press [R1 + X] to run the policy.
# 4. Click the mujoco window, and then press 9 to disable the elastic band.
```

### Sim2Real

You can use this program to control the robot directly, but make sure the on-board control program has been closed.

```bash
./g1_ctrl --network eth0 # eth0 is the network interface name.
```

---

## 🙏 Acknowledgements

This repository is built upon the support and contributions of the following open-source projects. Special thanks to:

- [IsaacLab](https://github.com/isaac-sim/IsaacLab): The foundation for training and running codes.
- [mujoco](https://github.com/google-deepmind/mujoco.git): Providing powerful simulation functionalities.
- [robot_lab](https://github.com/fan-ziqi/robot_lab): Referenced for project structure and parts of the implementation.
- [whole_body_tracking](https://github.com/HybridRobotics/whole_body_tracking): Versatile humanoid control framework for motion tracking.

---

## 📝 Citation

If you use this code, please cite:

```bibtex
@misc{unitree_rl_lab,
  title={Unitree RL Lab: Reinforcement Learning Environments for Unitree Robots},
  author={Unitree Robotics},
  year={2024},
  howpublished={\url{https://github.com/unitreerobotics/unitree_rl_lab}}
}
```

---

## 📄 License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENCE) file for details.

---

**Last Updated**: 2026-02-07

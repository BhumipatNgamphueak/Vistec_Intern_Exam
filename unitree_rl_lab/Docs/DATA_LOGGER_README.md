# IsaacLab Locomotion Data Logger - Usage Guide

This directory contains scripts for collecting comprehensive telemetry data from IsaacLab simulation for sim-to-sim transfer experiments.

## 📁 Files

| File | Description |
|------|-------------|
| `episode_config_generator.py` | Generate 200 synchronized test episodes |
| `data_logger_isaac.py` | ROS 2 node for data logging in IsaacLab |
| `config_isaac.yaml` | Configuration for data logger |
| `episode_configs.yaml` | Generated episode configurations |

---

## 🚀 Quick Start

### Step 1: Generate Episode Configurations

```bash
# Generate 200 episodes
python episode_config_generator.py \
  --num_episodes 200 \
  --duration 20.0 \
  --control_freq 50.0 \
  --seed 42 \
  --output episode_configs.yaml
```

**Output**: `episode_configs.yaml` with 200 episodes, each containing:
- Random seed
- Initial robot state (position, orientation, joint positions)
- Command sequence (4 commands × 5 sec each)

**Parameters**:
- `--num_episodes`: Number of test episodes (default: 200)
- `--duration`: Episode duration in seconds (default: 20.0)
- `--control_freq`: Control frequency in Hz (default: 50.0)
- `--seed`: Base random seed (default: 42)
- `--output`: Output YAML file path

### Step 2: Prepare Policy Checkpoint

Ensure you have a trained policy checkpoint:

```bash
# Create policies directory
mkdir -p policies

# Copy or link your trained policy
cp logs/rsl_rl/unitree_go2_velocity_lstm_dr/2026-02-06_23-20-45/model_25000.pt \
   policies/policy_go2_velocity_tracking.pt
```

### Step 3: Run Data Logger

```bash
# Full experiment (200 episodes)
python data_logger_isaac.py \
  --config config_isaac.yaml \
  --episodes episode_configs.yaml \
  --output ./logs/isaac/ \
  --policy ./policies/policy_go2_velocity_tracking.pt

# Test with subset (first 10 episodes)
python data_logger_isaac.py \
  --config config_isaac.yaml \
  --episodes episode_configs.yaml \
  --output ./logs/isaac/ \
  --policy ./policies/policy_go2_velocity_tracking.pt \
  --start_episode 0 \
  --end_episode 10

# Resume from last completed episode
python data_logger_isaac.py \
  --config config_isaac.yaml \
  --episodes episode_configs.yaml \
  --output ./logs/isaac/ \
  --policy ./policies/policy_go2_velocity_tracking.pt \
  --resume
```

---

## 📊 Output Data Structure

### CSV Files

Each episode generates a CSV file with **74 columns** and **1000 rows** (20 sec × 50 Hz):

```
logs/isaac/
├── locomotion_log_isaac_ep000_2026-02-07_14-30-15.csv
├── locomotion_log_isaac_ep001_2026-02-07_14-30-36.csv
├── ...
└── locomotion_log_isaac_ep199_2026-02-07_15-42-08.csv
```

### Column Categories (74 total)

| Category | Columns | Description |
|----------|---------|-------------|
| **Metadata** | 5 | timestamp_sim, timestamp_wall, episode_id, episode_seed, control_cycle |
| **Performance** | 3 | rtf, control_latency_ms, actual_control_dt |
| **Base Position** | 7 | base_pos (x,y,z), base_quat (x,y,z,w) |
| **Base Orientation** | 4 | base_roll, base_pitch, base_yaw, base_height |
| **Base Velocity** | 6 | base_lin_vel (x,y,z), base_ang_vel (x,y,z) |
| **Commands** | 3 | cmd_vx, cmd_vy, cmd_wz |
| **IMU** | 6 | gravity_proj (x,y,z), gyro (x,y,z) |
| **Joint State** | 36 | joint_pos (12), joint_vel (12), joint_acc (12) |
| **Actions** | 48 | action (12), cmd_joint_pos (12), cmd_torque (12), actual_torque (12) |
| **Action History** | 24 | prev_action_1 (12), prev_action_2 (12) |
| **Contact** | 12 | foot_contact_binary (4), foot_contact_force (4), foot_slip_velocity (4) |
| **Derived Metrics** | 4 | instantaneous_power, torque_saturation_count, action_smoothness, gravity_alignment |

---

## 🔧 Configuration

Edit `config_isaac.yaml` to customize:

```yaml
isaac_logger:
  policy_checkpoint: "./policies/policy_go2_velocity_tracking.pt"
  output_directory: "./logs/isaac/"

  robot_config:
    num_joints: 12
    torque_limit: 40.0  # N·m
    default_joint_pos: [0.0, 0.9, -1.8, ...]

  control:
    frequency_hz: 50  # Control rate
    pd_gains:
      kp: 20.0  # Position gain
      kd: 0.5   # Velocity gain

  observation_noise:  # Match training noise
    imu_gyro_std: 0.03
    imu_accel_std: 0.2
    joint_pos_std: 0.005
    joint_vel_std: 0.5
```

---

## ✅ Data Validation

After running experiments, validate the data:

```bash
# Check file count
ls logs/isaac/*.csv | wc -l  # Should be 200

# Verify row count per file
wc -l logs/isaac/locomotion_log_isaac_ep000_*.csv  # Should be ~1001 (1000 + header)

# Check for NaN values
grep -c "NaN" logs/isaac/*.csv  # Should be 0

# Verify RTF (Real-Time Factor)
python -c "
import pandas as pd
import glob

files = glob.glob('logs/isaac/*.csv')
rtf_values = []
for f in files:
    df = pd.read_csv(f)
    rtf_values.append(df['rtf'].mean())

avg_rtf = sum(rtf_values) / len(rtf_values)
print(f'Average RTF: {avg_rtf:.3f}')
print(f'Min RTF: {min(rtf_values):.3f}')
print(f'Max RTF: {max(rtf_values):.3f}')
"
```

**Expected Results**:
- Average RTF: >0.95 (real-time capable)
- No NaN values
- All 200 episodes completed

---

## 🐛 Troubleshooting

### Issue: Policy not loading

```bash
# Check policy file exists
ls -lh policies/policy_go2_velocity_tracking.pt

# Verify it's a valid PyTorch file
python -c "import torch; torch.jit.load('policies/policy_go2_velocity_tracking.pt')"
```

### Issue: ROS 2 topics not available

```bash
# Check available topics
ros2 topic list

# Check if IsaacLab is publishing
ros2 topic echo /isaac/robot_state --once
```

### Issue: Low RTF (< 0.95)

- Reduce number of environments in IsaacLab
- Run with `--headless` flag
- Check GPU utilization: `nvidia-smi`
- Reduce logging frequency

### Issue: Missing sensor data

```bash
# Check sensor callbacks
ros2 topic hz /isaac/imu
ros2 topic hz /isaac/robot_state
```

---

## 📈 Data Analysis

Example analysis script:

```python
import pandas as pd
import matplotlib.pyplot as plt

# Load episode data
df = pd.read_csv('logs/isaac/locomotion_log_isaac_ep000_2026-02-07_14-30-15.csv')

# Plot tracking performance
fig, axes = plt.subplots(3, 1, figsize=(12, 8))

# Velocity tracking
axes[0].plot(df['timestamp_sim'], df['cmd_vx'], label='Command vx')
axes[0].plot(df['timestamp_sim'], df['base_lin_vel_x'], label='Actual vx')
axes[0].set_ylabel('Linear Velocity X (m/s)')
axes[0].legend()
axes[0].grid(True)

# Joint positions
axes[1].plot(df['timestamp_sim'], df['joint_pos_0'], label='Joint 0')
axes[1].set_ylabel('Joint Position (rad)')
axes[1].legend()
axes[1].grid(True)

# Power consumption
axes[2].plot(df['timestamp_sim'], df['instantaneous_power'])
axes[2].set_ylabel('Power (W)')
axes[2].set_xlabel('Time (s)')
axes[2].grid(True)

plt.tight_layout()
plt.savefig('analysis_ep000.png')
plt.show()
```

---

## 🎯 Expected Experiment Duration

- Episode duration: 20 sec
- Setup/reset time: ~1 sec per episode
- Total: ~21 sec per episode

**Estimated total time**:
- 200 episodes × 21 sec = **4200 sec ≈ 70 minutes**

---

## 📝 Requirements

```bash
# Python packages
pip install numpy pandas pyyaml scipy torch

# ROS 2 Humble
sudo apt install ros-humble-desktop

# Python ROS 2 bindings
pip install rclpy
```

---

## 🔗 Related Files

- Training configs: `source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/go2/`
- Trained policies: `logs/rsl_rl/unitree_go2_velocity_lstm_dr/`
- Main README: `README.md`

---

**Last Updated**: 2026-02-07

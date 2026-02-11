# Go2 Training Configurations Comparison

This document compares all available training configurations for Unitree Go2 quadruped robot.

## Configuration Matrix

| Configuration | Actuator | Domain Randomization | Observation Noise | Use Case |
|--------------|----------|---------------------|------------------|----------|
| `unitree_go2_velocity` | MLP | Minimal | Yes | Standard baseline |
| `unitree_go2_velocity_mlp_no_dr` | MLP | **NO** | **NO** | Clean MLP baseline |
| `unitree_go2_velocity_mlp_custom` | MLP | **Comprehensive** | Yes | Gazebo sim2sim transfer |
| `unitree_go2_velocity_lstm` | LSTM | Moderate | Yes | Temporal modeling |
| `unitree_go2_velocity_lstm_no_dr` | **LSTM** | **NO** | **NO** | Clean LSTM baseline |
| `unitree_go2_velocity_lstm_custom` | LSTM | Comprehensive | Yes | LSTM with full DR |

---

## 1. MLP Actuator - No DR (Baseline)

**Config:** `velocity_env_cfg_mlp_no_dr.py`

**Training command:**
```bash
cd ~/unitree_rl_lab
source source/isaaclab.sh
python source/unitree_rl_lab/scripts/rsl_rl/train.py \
    --task=unitree_go2_velocity_mlp_no_dr \
    --num_envs=4096 \
    --headless
```

**Features:**
- ✅ MLP actuator (fast inference)
- ❌ No domain randomization
- ❌ No observation noise
- ❌ No disturbances
- Fixed physics parameters
- Fixed spawn positions

**Purpose:** Pure baseline for comparing MLP vs LSTM performance without DR effects.

---

## 2. MLP Actuator - Comprehensive DR (Gazebo-Ready)

**Config:** `velocity_env_cfg_mlp_custom.py`

**Training command:**
```bash
cd ~/unitree_rl_lab
source source/isaaclab.sh
python source/unitree_rl_lab/scripts/rsl_rl/train.py \
    --task=unitree_go2_velocity_mlp_custom \
    --num_envs=4096 \
    --headless
```

**Features:**
- ✅ MLP actuator (fast inference)
- ✅ Comprehensive domain randomization:
  - Friction: 0.4 - 1.25
  - Mass: -1 to +3 kg
  - Motor strength: ±25% Kp, ±50% Kd
  - External disturbances (forces, torques)
  - COM randomization
- ✅ Observation noise (IMU, encoders)
- ✅ Optimized for Gazebo deployment

**Purpose:** Best configuration for sim2sim transfer (Isaac Lab → Gazebo).

---

## 3. LSTM Actuator - No DR (Baseline) ⭐ NEW

**Config:** `velocity_env_cfg_lstm_no_dr.py`

**Training script:**
```bash
~/unitree_rl_lab/train_go2_lstm_no_dr.sh
```

**Or manually:**
```bash
cd ~/unitree_rl_lab
source source/isaaclab.sh
python source/unitree_rl_lab/scripts/rsl_rl/train.py \
    --task=unitree_go2_velocity_lstm_no_dr \
    --num_envs=4096 \
    --headless
```

**Features:**
- ✅ LSTM actuator (temporal modeling)
- ❌ No domain randomization
- ❌ No observation noise
- ❌ No disturbances
- Fixed physics parameters
- Fixed spawn positions

**Purpose:** Clean LSTM baseline to compare temporal modeling capability against MLP.

---

## 4. LSTM Actuator - Moderate DR

**Config:** `velocity_env_cfg_lstm.py`

**Training command:**
```bash
cd ~/unitree_rl_lab
source source/isaaclab.sh
python source/unitree_rl_lab/scripts/rsl_rl/train.py \
    --task=unitree_go2_velocity_lstm \
    --num_envs=4096 \
    --headless
```

**Features:**
- ✅ LSTM actuator (temporal modeling)
- ✅ Moderate domain randomization
- ✅ Observation noise
- ✅ Velocity pushes

**Purpose:** Standard LSTM training with some robustness.

---

## Recommended Experiments

### Experiment 1: Actuator Comparison (No DR)

**Goal:** Compare MLP vs LSTM pure performance without DR confounding

**Train:**
1. `unitree_go2_velocity_mlp_no_dr` (MLP baseline)
2. `unitree_go2_velocity_lstm_no_dr` (LSTM baseline)

**Compare:**
- Training speed
- Final reward
- Inference speed
- Sim2sim transfer performance (to Gazebo)

### Experiment 2: Domain Randomization Impact

**Goal:** Measure DR effectiveness for sim2sim transfer

**Train:**
1. `unitree_go2_velocity_mlp_no_dr` (no DR)
2. `unitree_go2_velocity_mlp_custom` (comprehensive DR)

**Compare:**
- Sim2sim transfer robustness
- Training time
- Real-world deployment success

### Experiment 3: LSTM + DR Combination

**Goal:** Combine temporal modeling with robustness

**Train:**
1. `unitree_go2_velocity_lstm_no_dr` (LSTM baseline)
2. `unitree_go2_velocity_lstm_custom` (LSTM + full DR)

**Compare:**
- Training stability
- Sim2sim transfer
- Temporal consistency

---

## Training Parameters Comparison

| Parameter | No DR | Moderate DR | Comprehensive DR |
|-----------|-------|-------------|------------------|
| **Friction** | 1.0 (fixed) | 0.3 - 1.2 | 0.4 - 1.25 |
| **Mass offset** | 0 kg | -1 to +3 kg | -1 to +3 kg |
| **Motor Kp** | 25.0 (fixed) | ±20% | ±25% |
| **Motor Kd** | 0.5 (fixed) | ±20% | ±50% |
| **Disturbances** | None | Pushes only | Forces + torques |
| **Spawn position** | Fixed | ±0.5m XY | ±0.5m XY |
| **Joint reset** | Default | ±30% | ±60° offset |
| **Observation noise** | No | Yes | Yes (reduced) |

---

## Observation Dimensions

All configurations use the same observation structure:

```python
Observation (45 dims):
  - base_ang_vel (3) - scaled by 0.2
  - projected_gravity (3) - no scale
  - velocity_commands (3) - no scale
  - joint_pos_rel (12) - no scale
  - joint_vel_rel (12) - scaled by 0.05
  - last_action (12) - no scale
```

**Total:** 45 dimensions (single frame, no history for MLP configs)

---

## Training Tips

### For No-DR Configurations

```bash
# Faster convergence expected
# Watch for overfitting to simulator
# May need more training steps for generalization
```

### For DR Configurations

```bash
# Slower convergence expected
# Better sim2sim transfer
# Monitor domain randomization ranges
```

### General

```bash
# Monitor training:
tensorboard --logdir ~/unitree_rl_lab/logs/rsl_rl/

# Typical training time:
# - No DR: 2-4 hours (4096 envs)
# - With DR: 4-8 hours (4096 envs)

# Early stopping:
# - Converges when reward plateaus for 20+ epochs
```

---

## File Locations

**Training configs:**
```
~/unitree_rl_lab/source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/go2/
├── velocity_env_cfg.py                    # Base config
├── velocity_env_cfg_mlp_no_dr.py         # MLP no DR
├── velocity_env_cfg_mlp_custom.py        # MLP comprehensive DR
├── velocity_env_cfg_lstm.py              # LSTM moderate DR
└── velocity_env_cfg_lstm_no_dr.py        # LSTM no DR (NEW)
```

**Training scripts:**
```
~/unitree_rl_lab/
├── train_go2_lstm_no_dr.sh               # LSTM no-DR script (NEW)
└── source/unitree_rl_lab/scripts/rsl_rl/train.py
```

**Trained models:**
```
~/unitree_rl_lab/logs/rsl_rl/
├── unitree_go2_velocity_mlp_no_dr/
├── unitree_go2_velocity_mlp_custom/
├── unitree_go2_velocity_lstm/
└── unitree_go2_velocity_lstm_no_dr/      # NEW
```

---

## Quick Reference

**Start training LSTM no-DR (easiest):**
```bash
~/unitree_rl_lab/train_go2_lstm_no_dr.sh
```

**Monitor training:**
```bash
tensorboard --logdir ~/unitree_rl_lab/logs/rsl_rl/
```

**Test trained policy:**
```bash
cd ~/unitree_rl_lab
source source/isaaclab.sh
python source/unitree_rl_lab/scripts/rsl_rl/play.py \
    --task=unitree_go2_velocity_lstm_no_dr \
    --num_envs=32 \
    --checkpoint=logs/rsl_rl/unitree_go2_velocity_lstm_no_dr/<timestamp>/model_XXXX.pt
```

---

## Expected Results

### MLP No-DR
- **Training time:** 2-4 hours
- **Final reward:** ~80-100
- **Sim2sim transfer:** Moderate (sensitive to physics mismatch)
- **Inference:** Fast (~1ms)

### MLP Comprehensive DR
- **Training time:** 4-8 hours
- **Final reward:** ~70-90 (lower due to DR difficulty)
- **Sim2sim transfer:** Excellent (robust to variations)
- **Inference:** Fast (~1ms)

### LSTM No-DR
- **Training time:** 3-5 hours (slower than MLP)
- **Final reward:** ~85-105 (may outperform MLP)
- **Sim2sim transfer:** Moderate
- **Inference:** Medium (~2-3ms)

### LSTM With DR
- **Training time:** 6-10 hours
- **Final reward:** ~75-95
- **Sim2sim transfer:** Excellent
- **Inference:** Medium (~2-3ms)

---

## Deployment

All configurations can be deployed to Gazebo using the workspace at `~/vistec_ex_ws/`.

See [~/vistec_ex_ws/README.md](file:///home/drl-68/vistec_ex_ws/README.md) for deployment instructions.

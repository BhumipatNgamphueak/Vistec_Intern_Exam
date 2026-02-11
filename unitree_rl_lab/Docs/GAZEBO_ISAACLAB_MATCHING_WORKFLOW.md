# Gazebo (Vistec_ex_ws) ↔ IsaacLab (unitree_rl_lab) Matching Workflow

## Overview

Collect motor response data with **matching PD gains** in both simulators for fair comparison.

**Key principle:** Use SAME PD gains in both Gazebo and IsaacLab!

---

## Configuration Matrix

Choose which PD gains to test:

| Configuration | PD Gains | Use For |
|--------------|----------|---------|
| **LOW** | Kp=25.0, Kd=0.5 | Compare with MLP/LSTM policies |
| **HIGH** | Kp=160.0, Kd=5.0 | Compare with Implicit policies |

---

## Workflow: LOW Gains (Kp=25, Kd=0.5)

### Step 1: Collect Data in Gazebo (Vistec_ex_ws)

**Location:** `~/Vistec_ex_ws/`

```bash
# Terminal 1 - Start Gazebo with LOW gains
cd ~/Vistec_ex_ws
source install/setup.bash

ros2 launch go2_gazebo go2_control.launch.py \
  spawn_z:=1.5 \
  controller_config:=go2_controllers_mlp_lstm.yaml
```

**Verify gains loaded:**
```bash
# Terminal 2
ros2 param get /controller_manager/go2_position_controller FL_hip_joint.p
# Should show: 25.0

ros2 param get /controller_manager/go2_position_controller FL_hip_joint.d
# Should show: 0.5
```

**Collect chirp data:**
```bash
# Terminal 2
cd ~/unitree_rl_lab

python scripts/actuator_comparison/chirp_test_gazebo.py \
  --joint FL_hip_joint \
  --pd_gains low \
  --f0 0.1 \
  --f1 20.0 \
  --duration 10.0 \
  --amplitude 0.3 \
  --output gazebo_isaaclab_comparison/gazebo
```

**Output:** `gazebo_isaaclab_comparison/gazebo/gazebo_pd_low/chirp_results_*.json`

---

### Step 2: Collect Data in IsaacLab with SAME Gains

**Location:** `~/unitree_rl_lab/`

```bash
cd ~/unitree_rl_lab

python scripts/actuator_comparison/chirp_test_isaaclab.py \
  --actuator mlp \
  --joint FL_hip_joint \
  --f0 0.1 \
  --f1 20.0 \
  --duration 10.0 \
  --amplitude 0.3 \
  --output gazebo_isaaclab_comparison/isaaclab \
  --headless
```

**Note:** Using MLP actuator because it has Kp=25, Kd=0.5 (matches Gazebo LOW)

**Output:** `gazebo_isaaclab_comparison/isaaclab/mlp/chirp_results_*.json`

---

### Step 3: Compare Results

```bash
# Find latest files
GAZEBO_FILE=$(ls -t gazebo_isaaclab_comparison/gazebo/gazebo_pd_low/chirp_results_*.json | head -1)
ISAAC_FILE=$(ls -t gazebo_isaaclab_comparison/isaaclab/mlp/chirp_results_*.json | head -1)

# Compare
python scripts/actuator_comparison/compare_chirp_data.py \
  --gazebo "$GAZEBO_FILE" \
  --isaaclab "$ISAAC_FILE" \
  --output gazebo_isaaclab_comparison/comparison_low_gains \
  --label "LOW Gains (Kp=25, Kd=0.5)"
```

**Output:** `gazebo_isaaclab_comparison/comparison_low_gains/`

---

## Workflow: HIGH Gains (Kp=160, Kd=5)

### Step 1: Collect Data in Gazebo (Vistec_ex_ws)

```bash
# Terminal 1 - Start Gazebo with HIGH gains
cd ~/Vistec_ex_ws
source install/setup.bash

ros2 launch go2_gazebo go2_control.launch.py \
  spawn_z:=1.5 \
  controller_config:=go2_controllers_implicit.yaml
```

**Verify gains:**
```bash
ros2 param get /controller_manager/go2_position_controller FL_hip_joint.p
# Should show: 160.0

ros2 param get /controller_manager/go2_position_controller FL_hip_joint.d
# Should show: 5.0
```

**Collect data:**
```bash
cd ~/unitree_rl_lab

python scripts/actuator_comparison/chirp_test_gazebo.py \
  --joint FL_hip_joint \
  --pd_gains high \
  --f0 0.1 \
  --f1 20.0 \
  --duration 10.0 \
  --amplitude 0.3 \
  --output gazebo_isaaclab_comparison/gazebo
```

---

### Step 2: Collect Data in IsaacLab with SAME Gains

```bash
cd ~/unitree_rl_lab

python scripts/actuator_comparison/chirp_test_isaaclab.py \
  --actuator implicit \
  --joint FL_hip_joint \
  --f0 0.1 \
  --f1 20.0 \
  --duration 10.0 \
  --amplitude 0.3 \
  --output gazebo_isaaclab_comparison/isaaclab \
  --headless
```

**Note:** Using Implicit actuator because it has Kp=160, Kd=5 (matches Gazebo HIGH)

---

### Step 3: Compare Results

```bash
GAZEBO_FILE=$(ls -t gazebo_isaaclab_comparison/gazebo/gazebo_pd_high/chirp_results_*.json | head -1)
ISAAC_FILE=$(ls -t gazebo_isaaclab_comparison/isaaclab/implicit/chirp_results_*.json | head -1)

python scripts/actuator_comparison/compare_chirp_data.py \
  --gazebo "$GAZEBO_FILE" \
  --isaaclab "$ISAAC_FILE" \
  --output gazebo_isaaclab_comparison/comparison_high_gains \
  --label "HIGH Gains (Kp=160, Kd=5)"
```

---

## Directory Structure

```
Vistec_ex_ws/                          # Gazebo workspace
├── src/go2_gazebo/
│   ├── config/
│   │   ├── go2_controllers_mlp_lstm.yaml   # Kp=25, Kd=0.5
│   │   └── go2_controllers_implicit.yaml   # Kp=160, Kd=5
│   └── ...
└── ...

unitree_rl_lab/                        # IsaacLab workspace
├── scripts/actuator_comparison/
│   ├── chirp_test_gazebo.py          # Run from here for Gazebo
│   ├── chirp_test_isaaclab.py        # IsaacLab testing
│   └── compare_chirp_data.py         # NEW: Comparison tool
│
└── gazebo_isaaclab_comparison/        # Output directory
    ├── gazebo/
    │   ├── gazebo_pd_low/             # Gazebo LOW gains data
    │   └── gazebo_pd_high/            # Gazebo HIGH gains data
    ├── isaaclab/
    │   ├── mlp/                        # IsaacLab LOW gains (MLP)
    │   └── implicit/                   # IsaacLab HIGH gains (Implicit)
    └── comparison_low_gains/           # Comparison results LOW
        └── comparison_high_gains/      # Comparison results HIGH
```

---

## PD Gains Mapping

**CRITICAL:** Ensure gains match between platforms!

### LOW Gains Configuration

| Platform | Config | Kp | Kd |
|----------|--------|-----|-----|
| **Gazebo** | `go2_controllers_mlp_lstm.yaml` | 25.0 | 0.5 |
| **IsaacLab** | MLP actuator | 25.0 | 0.5 |
| **IsaacLab** | LSTM actuator | 25.0 | 0.5 |

### HIGH Gains Configuration

| Platform | Config | Kp | Kd |
|----------|--------|-----|-----|
| **Gazebo** | `go2_controllers_implicit.yaml` | 160.0 | 5.0 |
| **IsaacLab** | Implicit actuator | 160.0 | 5.0 |

---

## Expected Results

### Good Match Criteria

**When gains match, expect:**

| Metric | Acceptable Difference |
|--------|---------------------|
| Rise Time | < 20% |
| Overshoot | < 5% |
| Bandwidth | < 20% |
| RMS Error | < 15% |

### Example Good Match

```
Metric              Gazebo    IsaacLab   Diff
Rise Time (ms)      82.0      78.5       4.3%  ✅
Overshoot (%)       5.2       4.8        0.4%  ✅
Bandwidth (Hz)      10.5      11.2       6.7%  ✅
RMS Error (mrad)    45.2      48.1       6.4%  ✅
```

**Interpretation:** Excellent match! Simulators behave very similarly.

---

## Gazebo Setup (Vistec_ex_ws)

### Controller Config Files

Ensure these files exist in `Vistec_ex_ws/src/go2_gazebo/config/`:

**1. go2_controllers_mlp_lstm.yaml** (LOW gains)
```yaml
controller_manager:
  ros__parameters:
    update_rate: 50

go2_position_controller:
  ros__parameters:
    joints:
      - FL_hip_joint
      - FL_thigh_joint
      - FL_calf_joint
      - FR_hip_joint
      - FR_thigh_joint
      - FR_calf_joint
      - RL_hip_joint
      - RL_thigh_joint
      - RL_calf_joint
      - RR_hip_joint
      - RR_thigh_joint
      - RR_calf_joint

    command_interfaces:
      - position

    state_interfaces:
      - position
      - velocity

    gains:
      FL_hip_joint:   {p: 25.0, d: 0.5, i: 0.0}
      FL_thigh_joint: {p: 25.0, d: 0.5, i: 0.0}
      FL_calf_joint:  {p: 25.0, d: 0.5, i: 0.0}
      FR_hip_joint:   {p: 25.0, d: 0.5, i: 0.0}
      FR_thigh_joint: {p: 25.0, d: 0.5, i: 0.0}
      FR_calf_joint:  {p: 25.0, d: 0.5, i: 0.0}
      RL_hip_joint:   {p: 25.0, d: 0.5, i: 0.0}
      RL_thigh_joint: {p: 25.0, d: 0.5, i: 0.0}
      RL_calf_joint:  {p: 25.0, d: 0.5, i: 0.0}
      RR_hip_joint:   {p: 25.0, d: 0.5, i: 0.0}
      RR_thigh_joint: {p: 25.0, d: 0.5, i: 0.0}
      RR_calf_joint:  {p: 25.0, d: 0.5, i: 0.0}
```

**2. go2_controllers_implicit.yaml** (HIGH gains)
```yaml
# Same structure, but gains:
#   FL_hip_joint:   {p: 160.0, d: 5.0, i: 0.0}
#   ... (all joints)
```

These files should already exist from previous setup!

---

## Quick Verification

### Check Gazebo PD Gains

```bash
# After launching Gazebo
ros2 param list /controller_manager/go2_position_controller | grep gains

# Check specific joint
ros2 param get /controller_manager/go2_position_controller FL_hip_joint.p
ros2 param get /controller_manager/go2_position_controller FL_hip_joint.d
```

### Check IsaacLab PD Gains

**MLP/LSTM:**
```bash
# Hardcoded in unitree_actuators.py:
# stiffness = {".*": 25.0}
# damping = {".*": 0.5}
```

**Implicit:**
```bash
# Hardcoded in unitree.py:
# stiffness=160.0
# damping=5.0
```

---

## Summary Commands

### Test LOW Gains (Kp=25, Kd=0.5)

```bash
# 1. Start Gazebo (Vistec_ex_ws)
cd ~/Vistec_ex_ws && source install/setup.bash
ros2 launch go2_gazebo go2_control.launch.py \
  spawn_z:=1.5 controller_config:=go2_controllers_mlp_lstm.yaml

# 2. Collect Gazebo data (unitree_rl_lab)
cd ~/unitree_rl_lab
python scripts/actuator_comparison/chirp_test_gazebo.py \
  --joint FL_hip_joint --pd_gains low \
  --f0 0.1 --f1 20.0 --duration 10.0 \
  --output gazebo_isaaclab_comparison/gazebo

# 3. Collect IsaacLab data
python scripts/actuator_comparison/chirp_test_isaaclab.py \
  --actuator mlp --joint FL_hip_joint \
  --f0 0.1 --f1 20.0 --duration 10.0 \
  --output gazebo_isaaclab_comparison/isaaclab --headless

# 4. Compare
python scripts/actuator_comparison/compare_chirp_data.py \
  --gazebo gazebo_isaaclab_comparison/gazebo/gazebo_pd_low/chirp_results_*.json \
  --isaaclab gazebo_isaaclab_comparison/isaaclab/mlp/chirp_results_*.json \
  --output gazebo_isaaclab_comparison/comparison_low_gains
```

---

### Test HIGH Gains (Kp=160, Kd=5)

```bash
# 1. Start Gazebo with HIGH gains
cd ~/Vistec_ex_ws && source install/setup.bash
ros2 launch go2_gazebo go2_control.launch.py \
  spawn_z:=1.5 controller_config:=go2_controllers_implicit.yaml

# 2. Collect Gazebo data
cd ~/unitree_rl_lab
python scripts/actuator_comparison/chirp_test_gazebo.py \
  --joint FL_hip_joint --pd_gains high \
  --f0 0.1 --f1 20.0 --duration 10.0 \
  --output gazebo_isaaclab_comparison/gazebo

# 3. Collect IsaacLab data
python scripts/actuator_comparison/chirp_test_isaaclab.py \
  --actuator implicit --joint FL_hip_joint \
  --f0 0.1 --f1 20.0 --duration 10.0 \
  --output gazebo_isaaclab_comparison/isaaclab --headless

# 4. Compare
python scripts/actuator_comparison/compare_chirp_data.py \
  --gazebo gazebo_isaaclab_comparison/gazebo/gazebo_pd_high/chirp_results_*.json \
  --isaaclab gazebo_isaaclab_comparison/isaaclab/implicit/chirp_results_*.json \
  --output gazebo_isaaclab_comparison/comparison_high_gains
```

---

## Next Step

I'll create the comparison script: `compare_chirp_data.py`

---

**Last Updated:** 2026-02-10

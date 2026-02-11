# Actuator Response Comparison: IsaacLab vs Gazebo

This directory contains tools to test and compare actuator/motor responses between IsaacLab and Gazebo.

## Purpose

Understanding actuator dynamics is **critical** for sim-to-sim transfer. Different actuator types respond differently to position commands:

- **MLP/LSTM Actuators** (IsaacLab): Neural networks that learn actuator dynamics (Kp=25, Kd=0.5)
- **Implicit Actuators** (IsaacLab): Physics-based IdealPD control (Kp=160, Kd=5)
- **Gazebo Motors**: PD effort controllers (configurable gains)

This tool measures:
- **Step response**: Rise time, overshoot, settling time
- **Tracking performance**: RMSE, phase lag
- **Frequency response**: Bandwidth, resonance
- **Torque output**: Effort characteristics

---

## Quick Start

### 1. Test IsaacLab Actuators

```bash
cd /home/drl-68/unitree_rl_lab

# Test all 3 actuator types
./test_all_actuators_isaaclab.sh
```

**Duration:** ~3 minutes (1 minute per actuator)

**Output:**
```
actuator_analysis/isaaclab/
├── mlp/
│   ├── test_results_*.json
│   └── plots_*/
├── lstm/
│   ├── test_results_*.json
│   └── plots_*/
└── implicit/
    ├── test_results_*.json
    └── plots_*/
```

---

### 2. Test Gazebo Motors

**Prerequisites:**
- Gazebo running with Go2 robot
- Joint controllers active
- ROS 2 sourced

```bash
# Start Gazebo (in separate terminal)
ros2 launch go2_gazebo go2_control.launch.py \
  controller_config:=go2_controllers_mlp_lstm.yaml

# Run tests
./test_gazebo_motors_all.sh
```

**Duration:** ~2 minutes (1 minute per configuration)

**Output:**
```
actuator_analysis/gazebo/
├── pd_low/       # Kp=25, Kd=0.5
│   ├── test_results_*.json
│   └── plots_*/
└── pd_high/      # Kp=160, Kd=5
    ├── test_results_*.json
    └── plots_*/
```

---

### 3. Compare Results

```bash
# Generate comparative plots and metrics
./compare_all_actuators.sh
```

**Output:**
```
actuator_analysis/comparison/
├── mlp_vs_gazebo_low/
│   ├── comparison_step_response.png
│   ├── comparison_sine_1hz.png
│   ├── comparison_sine_5hz.png
│   └── comparison_metrics.csv
├── lstm_vs_gazebo_low/
│   └── ...
└── implicit_vs_gazebo_high/
    └── ...
```

---

## Manual Usage

### Test Specific IsaacLab Actuator

```bash
# MLP actuator
python scripts/actuator_comparison/test_isaaclab_actuators.py \
  --actuator mlp \
  --joint FL_hip_joint \
  --test all \
  --duration 5.0 \
  --headless

# LSTM actuator
python scripts/actuator_comparison/test_isaaclab_actuators.py \
  --actuator lstm \
  --joint FL_hip_joint \
  --test step \
  --duration 3.0 \
  --headless

# Implicit actuator
python scripts/actuator_comparison/test_isaaclab_actuators.py \
  --actuator implicit \
  --joint FL_hip_joint \
  --test sine \
  --duration 5.0 \
  --headless
```

**Options:**
- `--actuator`: `mlp`, `lstm`, or `implicit`
- `--joint`: Joint name (default: `FL_hip_joint`)
- `--test`: `step`, `sine`, `chirp`, or `all`
- `--duration`: Test duration in seconds
- `--output`: Output directory (default: `actuator_analysis/isaaclab`)
- `--headless`: Run without GUI

---

### Test Gazebo Motors

```bash
# LOW gains (for MLP/LSTM comparison)
python scripts/actuator_comparison/test_gazebo_motors.py \
  --joint FL_hip_joint \
  --pd_gains low \
  --test all \
  --duration 5.0

# HIGH gains (for Implicit comparison)
python scripts/actuator_comparison/test_gazebo_motors.py \
  --joint FL_hip_joint \
  --pd_gains high \
  --test all \
  --duration 5.0
```

**Options:**
- `--joint`: Joint name (default: `FL_hip_joint`)
- `--pd_gains`: `low` (25/0.5) or `high` (160/5)
- `--test`: `step`, `sine`, `chirp`, or `all`
- `--duration`: Test duration in seconds
- `--output`: Output directory (default: `actuator_analysis/gazebo`)

---

### Compare Specific Results

```bash
python scripts/actuator_comparison/compare_actuators.py \
  --isaac actuator_analysis/isaaclab/mlp/test_results_*.json \
  --gazebo actuator_analysis/gazebo/pd_low/test_results_*.json \
  --output actuator_analysis/comparison/custom
```

---

## Test Descriptions

### 1. Step Response

**What:** Apply step input (0 → 0.5 rad) and measure response

**Metrics:**
- **Rise time**: Time to go from 10% to 90% of final value
- **Overshoot**: Maximum overshoot beyond target
- **Settling time**: Time to stay within 2% of target

**Why:** Shows actuator speed and damping characteristics

### 2. Sinusoidal Tracking (1 Hz, 5 Hz)

**What:** Track sinusoidal position command

**Metrics:**
- **RMSE**: Root mean square error
- **Max error**: Maximum tracking error
- **Phase lag**: Delay between command and response

**Why:** Shows tracking accuracy and bandwidth

### 3. Frequency Sweep (Chirp)

**What:** Sweep frequency from 0.5 Hz to 10 Hz

**Metrics:**
- Visual inspection of response across frequencies

**Why:** Identifies resonances and bandwidth limits

---

## Understanding Results

### Expected Differences

**MLP/LSTM vs Gazebo LOW gains:**
- Should be similar since both use Kp=25, Kd=0.5
- Small differences due to:
  - Neural network learned dynamics
  - Simulation physics differences
  - Solver differences

**Implicit vs Gazebo HIGH gains:**
- Should be very similar since both use Kp=160, Kd=5
- IdealPD is essentially same as Gazebo PD controller
- Minimal differences expected

### What to Look For

**Good Match:**
- Rise time within 20%
- Overshoot within 5%
- RMSE within 10 mrad
- Similar torque profiles

**Poor Match:**
- Large rise time difference (>50%)
- Very different overshoot (>10%)
- High RMSE difference (>50 mrad)
- Very different torque patterns

**If match is poor:**
1. Verify PD gains loaded correctly in Gazebo
2. Check physics timestep matches (5ms)
3. Verify control frequency matches (50 Hz)
4. Check for additional damping/friction in Gazebo

---

## File Structure

```
unitree_rl_lab/
├── scripts/actuator_comparison/
│   ├── test_isaaclab_actuators.py    # IsaacLab testing
│   ├── test_gazebo_motors.py         # Gazebo testing
│   ├── compare_actuators.py          # Comparison tool
│   └── README.md                      # This file
│
├── test_all_actuators_isaaclab.sh    # Test all IsaacLab
├── test_gazebo_motors_all.sh         # Test all Gazebo
├── compare_all_actuators.sh          # Compare all
│
└── actuator_analysis/                 # Output directory
    ├── isaaclab/
    │   ├── mlp/
    │   ├── lstm/
    │   └── implicit/
    ├── gazebo/
    │   ├── pd_low/
    │   └── pd_high/
    └── comparison/
        ├── mlp_vs_gazebo_low/
        ├── lstm_vs_gazebo_low/
        └── implicit_vs_gazebo_high/
```

---

## Output Files

### Test Results JSON

```json
{
  "actuator_type": "mlp",
  "joint_name": "FL_hip_joint",
  "dt": 0.005,
  "control_freq": 50,
  "tests": {
    "step": {
      "time": [...],
      "position_command": [...],
      "position_actual": [...],
      "velocity_actual": [...],
      "torque": [...],
      "metrics": {
        "rise_time": 0.082,
        "overshoot_percent": 5.2,
        "settling_time": 0.31
      }
    },
    "sine_1hz": { ... },
    "sine_5hz": { ... },
    "chirp": { ... }
  }
}
```

### Comparison Metrics CSV

| Test | Metric | IsaacLab | Gazebo | Difference |
|------|--------|----------|--------|------------|
| Step Response | Rise Time (ms) | 82.0 | 78.5 | 3.5 |
| Step Response | Overshoot (%) | 5.2 | 4.8 | 0.4 |
| Sine 1 Hz | RMSE (mrad) | 8.5 | 9.2 | 0.7 |
| Sine 5 Hz | RMSE (mrad) | 45.2 | 52.1 | 6.9 |

---

## Troubleshooting

### IsaacLab Issues

**Error: "Module 'omni' not found"**
- Make sure IsaacLab is properly installed
- Run from correct conda/venv environment

**Error: "Robot not spawning"**
- Check URDF/USD paths
- Verify assets are available

### Gazebo Issues

**Error: "No /joint_states topic"**
- Make sure Gazebo is running
- Check joint_state_broadcaster is active
- Verify: `ros2 topic list | grep joint`

**Error: "Joint not found"**
- Check joint names: `ros2 topic echo /joint_states --once`
- Use exact joint name from URDF

**Error: "No position controller"**
- Load controller: `ros2 control load_controller go2_position_controller`
- Check: `ros2 control list_controllers`

### Comparison Issues

**Large differences in results:**
1. **Verify PD gains match**:
   ```bash
   ros2 param get /controller_manager/go2_position_controller FL_hip_joint.p
   ```
2. **Check timestep matches** (5ms physics, 50 Hz control)
3. **Verify no additional damping** in Gazebo URDF
4. **Check ground friction** matches (1.0)

---

## Advanced Usage

### Test Different Joints

```bash
# Test hip joint
python scripts/actuator_comparison/test_isaaclab_actuators.py \
  --actuator mlp --joint FL_hip_joint

# Test thigh joint
python scripts/actuator_comparison/test_isaaclab_actuators.py \
  --actuator mlp --joint FL_thigh_joint

# Test calf joint
python scripts/actuator_comparison/test_isaaclab_actuators.py \
  --actuator mlp --joint FL_calf_joint
```

### Custom Test Parameters

```python
# Modify test_isaaclab_actuators.py
amplitude = 0.5  # Larger amplitude
freq = 10.0      # Higher frequency
step_size = 1.0  # Larger step
```

### Export for Analysis

```python
import json
import pandas as pd

# Load results
with open('test_results.json', 'r') as f:
    data = json.load(f)

# Convert to DataFrame
df = pd.DataFrame({
    'time': data['tests']['step']['time'],
    'pos_cmd': data['tests']['step']['position_command'],
    'pos_actual': data['tests']['step']['position_actual']
})

# Analyze
df.to_csv('analysis.csv')
```

---

## Theory Background

### Actuator Types

**1. ActuatorNetMLP (MLP/LSTM)**
- Neural network learns mapping: (q_des, q, dq) → τ
- Trained on real motor data or physics sim
- Models: delays, backlash, torque limits, friction
- Requires LOW PD gains (network provides control authority)

**2. IdealPD (Implicit)**
- Direct physics-based PD control
- τ = Kp(q_des - q) - Kd*dq
- No delays or non-linearities
- Requires HIGH PD gains for stiff control

**3. Gazebo Effort Controller**
- PD control in Gazebo physics engine
- Similar to IdealPD but with sim characteristics
- Configurable gains via ROS 2 parameters

### Transfer Learning Insight

For successful sim-to-real or sim-to-sim transfer:
- Actuator dynamics must match
- PD gains must be appropriate for actuator type
- Neural actuators need learned dynamics from target platform
- Physics actuators need tuned gains

---

## References

- IsaacLab Actuator Docs: [link]
- RSL-RL Actuator Models: [link]
- Gazebo ros2_control: [link]

---

**Last Updated:** 2026-02-10
**Version:** 1.0

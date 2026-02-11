# Chirp Motor Testing - Isaac Lab vs Gazebo

This directory contains scripts for testing motor actuators using chirp signals (frequency sweeps) in a hanging configuration (robot suspended in air).

## Overview

**Purpose:** Compare motor dynamics between Isaac Lab (MLP, LSTM, Implicit actuators) and Gazebo simulation.

**Configuration:**
- Robot hanging at 1.5m height (no ground contact)
- Chirp signal: 0.1 Hz → 20 Hz over 10 seconds
- Single joint testing (e.g., FR_hip_joint)
- Data collection: position, velocity, torque

**Key Finding:** All actuators use **SAME PD gains: Kp=25.0, Kd=0.5**

---

## Quick Start

### 1. Test in Isaac Lab (unitree_rl_lab)

```bash
# Interactive menu
./run_chirp_tests.sh

# Or direct commands:

# Test MLP actuator
python test_chirp_all_actuators.py --actuator mlp

# Test LSTM actuator
python test_chirp_all_actuators.py --actuator lstm

# Test Implicit actuator
python test_chirp_all_actuators.py --actuator implicit

# Test all actuators
python test_chirp_all_actuators.py --actuator all
```

**Output:** Data saved to `chirp_data_isaaclab/`

---

### 2. Test in Gazebo (Vistec_ex_ws)

**IMPORTANT:** Use the SAME PD gains (Kp=25.0, Kd=0.5) in Gazebo!

```bash
# In Vistec_ex_ws workspace
cd ~/Vistec_ex_ws

# Launch Gazebo with hanging robot
ros2 launch go2_description display.launch.py spawn_z:=1.5

# In another terminal, run chirp test
python3 chirp_test_gazebo.py --joint FR_hip_joint
```

**Output:** Data saved to `chirp_data_gazebo/`

---

### 3. Compare Results

```bash
# Compare MLP actuator with Gazebo
python compare_chirp_isaac_gazebo.py \
    --isaac "chirp_data_isaaclab/chirp_mlp_FR_hip_joint_*.npz" \
    --gazebo "chirp_data_gazebo/chirp_FR_hip_joint_*.npz" \
    --output comparison_mlp

# Compare LSTM actuator with Gazebo
python compare_chirp_isaac_gazebo.py \
    --isaac "chirp_data_isaaclab/chirp_lstm_FR_hip_joint_*.npz" \
    --gazebo "chirp_data_gazebo/chirp_FR_hip_joint_*.npz" \
    --output comparison_lstm

# Compare Implicit actuator with Gazebo
python compare_chirp_isaac_gazebo.py \
    --isaac "chirp_data_isaaclab/chirp_implicit_FR_hip_joint_*.npz" \
    --gazebo "chirp_data_gazebo/chirp_FR_hip_joint_*.npz" \
    --output comparison_implicit
```

**Output:** Comparison plots and metrics in `comparison_*/`

---

## Files

| File | Description |
|------|-------------|
| `test_chirp_all_actuators.py` | Isaac Lab chirp test (MLP/LSTM/Implicit) |
| `run_chirp_tests.sh` | Interactive menu for Isaac Lab tests |
| `compare_chirp_isaac_gazebo.py` | Compare Isaac Lab vs Gazebo data |
| `CHIRP_TEST_README.md` | This documentation |

---

## Test Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `--duration` | 10.0 s | Chirp duration |
| `--f0` | 0.1 Hz | Starting frequency |
| `--f1` | 20.0 Hz | Ending frequency |
| `--amplitude` | 0.5 rad | Chirp amplitude (±28.6°) |
| `--test_joint` | FR_hip_joint | Joint to test |
| `--output_dir` | chirp_data_isaaclab | Output directory |

**Customization:**
```bash
python test_chirp_all_actuators.py \
    --actuator mlp \
    --duration 15.0 \
    --f0 0.5 \
    --f1 30.0 \
    --amplitude 0.3 \
    --test_joint FR_thigh_joint
```

---

## Available Joints

Go2 has 12 joints (4 legs × 3 joints each):

**Front Left (FL):**
- `FL_hip_joint` (abduction/adduction)
- `FL_thigh_joint` (forward/backward)
- `FL_calf_joint` (knee)

**Front Right (FR):**
- `FR_hip_joint`
- `FR_thigh_joint`
- `FR_calf_joint`

**Rear Left (RL):**
- `RL_hip_joint`
- `RL_thigh_joint`
- `RL_calf_joint`

**Rear Right (RR):**
- `RR_hip_joint`
- `RR_thigh_joint`
- `RR_calf_joint`

---

## Actuator Types

### 1. MLP Actuator
- **Type:** Neural network (feedforward)
- **Input:** 6 features [pos_err(t), pos_err(t-1), pos_err(t-2), vel(t), vel(t-1), vel(t-2)]
- **Training:** Learned from Gazebo motor dynamics
- **PD Gains:** Kp=25.0, Kd=0.5

### 2. LSTM Actuator
- **Type:** Recurrent neural network
- **Input:** Same 6 features as MLP
- **Architecture:** 2 layers, hidden_size=64
- **Training:** Learned from Gazebo motor dynamics
- **PD Gains:** Kp=25.0, Kd=0.5

### 3. Implicit Actuator
- **Type:** Physics-based (analytical torque-speed curves)
- **Model:** UnitreeActuatorCfg_Go2HV
- **No training required** - uses analytical motor model
- **PD Gains:** Kp=25.0, Kd=0.5

---

## Data Format

### Output Files (`.npz`)

Each test saves data as NumPy compressed format:

```python
data = np.load('chirp_mlp_FR_hip_joint_20260210_120000.npz')

# Available fields:
data['time']           # Time array (seconds)
data['cmd_pos']        # Commanded position (radians)
data['actual_pos']     # Actual position (radians)
data['actual_vel']     # Actual velocity (rad/s)
data['actual_torque']  # Applied torque (Nm)
data['actuator']       # Actuator type ('mlp', 'lstm', 'implicit')
data['joint']          # Joint name (e.g., 'FR_hip_joint')
data['hanging_height'] # Height above ground (meters)
```

### CSV Format

Human-readable CSV files are also saved:
```
time,cmd_pos,actual_pos,actual_vel,actual_torque
0.000000,0.800000,0.799523,0.012345,0.234567
0.005000,0.801234,0.800987,0.023456,0.345678
...
```

---

## Comparison Metrics

The comparison script computes:

| Metric | Description |
|--------|-------------|
| **Tracking RMSE** | Root mean square position tracking error |
| **Tracking MAE** | Mean absolute position tracking error |
| **Tracking Max Error** | Maximum position tracking error |
| **Simulator Difference RMSE** | RMS difference between Isaac and Gazebo |
| **Simulator Difference MAE** | Mean absolute difference |
| **Simulator Difference Max** | Maximum difference |

---

## Workflow Summary

```
┌─────────────────────────────────────────────────────────────┐
│                  CHIRP TEST WORKFLOW                        │
└─────────────────────────────────────────────────────────────┘

1. Isaac Lab Testing (unitree_rl_lab)
   ├─ Test MLP actuator    → chirp_mlp_*.npz
   ├─ Test LSTM actuator   → chirp_lstm_*.npz
   └─ Test Implicit actuator → chirp_implicit_*.npz

2. Gazebo Testing (Vistec_ex_ws)
   └─ Test with PD gains (Kp=25, Kd=0.5) → chirp_gazebo_*.npz

3. Comparison
   ├─ Compare MLP vs Gazebo
   ├─ Compare LSTM vs Gazebo
   └─ Compare Implicit vs Gazebo

4. Analysis
   └─ Review plots and metrics
```

---

## Expected Results

### Position Tracking
- All actuators should track chirp command closely
- MLP/LSTM may show learned dynamics behavior
- Implicit should show physics-based response

### Velocity Response
- Smooth velocity profile following chirp frequency sweep
- Phase lag depends on actuator dynamics

### Torque Output
- MLP/LSTM: Learned torque prediction
- Implicit: Analytical torque-speed curve

### Sim-to-Sim Transfer
- Small difference → good sim-to-sim transfer
- Large difference → model mismatch or parameter differences

---

## Troubleshooting

### Issue: Robot falls to ground
**Solution:** Check that `hanging_height=1.5` is set and base position is maintained

### Issue: Unstable motion
**Solution:** Reduce chirp amplitude (try `--amplitude 0.3`)

### Issue: Different results between simulators
**Solution:** Verify SAME PD gains (Kp=25.0, Kd=0.5) are used in both

### Issue: Actuator model not found
**Solution:** Ensure actuator models are in `assets/actuator_models/`:
- `actuator_mlp.pth`
- `actuator_lstm.pth`

---

## Important Notes

1. **PD Gains:** All actuators use Kp=25.0, Kd=0.5 (confirmed from source code)
2. **Hanging Configuration:** Robot must be suspended to isolate motor dynamics
3. **Data Collection:** Save data immediately for comparison
4. **Timestep:** Isaac Lab uses 5ms (200Hz), ensure Gazebo matches for fair comparison

---

## Next Steps

After collecting data:

1. **Analyze frequency response** - identify bandwidth, resonance
2. **Measure phase lag** - compute delay between command and response
3. **Compare actuator types** - which best matches Gazebo?
4. **Tune models** - adjust MLP/LSTM if needed for better match

---

## References

- **Isaac Lab Docs:** https://isaac-sim.github.io/IsaacLab/
- **Go2 Specs:** See `GO2_JOINT_SPECIFICATIONS.md`
- **PD Gains Analysis:** See `TRAINED_POLICIES_PD_GAINS_SUMMARY.md`

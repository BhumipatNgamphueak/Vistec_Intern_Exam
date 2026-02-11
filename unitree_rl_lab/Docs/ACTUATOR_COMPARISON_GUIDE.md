# Actuator Response Comparison Guide

## Overview

Compare motor/actuator responses between **IsaacLab** and **Gazebo** to understand dynamics and validate sim-to-sim matching.

---

## Why This Matters

Your 6 trained policies use **different actuator types** with **different PD gains**:

| Actuator Type | Policies | PD Gains | Characteristics |
|--------------|----------|----------|-----------------|
| **MLP/LSTM** | 4 policies | Kp=25, Kd=0.5 | Neural network learns dynamics |
| **Implicit** | 2 policies | Kp=160, Kd=5 | Physics-based PD control |

For accurate sim-to-sim comparison, Gazebo motors must **match** IsaacLab actuator responses!

---

## Quick Start (3 Steps)

### Step 1: Test IsaacLab Actuators

```bash
cd /home/drl-68/unitree_rl_lab
./test_all_actuators_isaaclab.sh
```

**Duration:** 3 minutes
**Tests:** MLP, LSTM, Implicit actuators
**Output:** `actuator_analysis/isaaclab/`

---

### Step 2: Test Gazebo Motors

**First, start Gazebo:**
```bash
# Terminal 1
ros2 launch go2_gazebo go2_control.launch.py \
  controller_config:=go2_controllers_mlp_lstm.yaml
```

**Then run tests:**
```bash
# Terminal 2
cd /home/drl-68/unitree_rl_lab
./test_gazebo_motors_all.sh
```

**Duration:** 2 minutes
**Tests:** LOW gains (25/0.5), HIGH gains (160/5)
**Output:** `actuator_analysis/gazebo/`

⚠️ **Script will prompt you to switch controller configs!**

---

### Step 3: Compare Results

```bash
./compare_all_actuators.sh
```

**Output:** `actuator_analysis/comparison/`
**Generates:**
- Comparative plots (step response, sine tracking)
- Metrics table (CSV)

---

## What Gets Tested

### 1. Step Response
- Apply 0 → 0.5 rad step
- Measure: rise time, overshoot, settling time

### 2. Sinusoidal Tracking
- Track 1 Hz and 5 Hz sine waves
- Measure: RMSE, max error, phase lag

### 3. Frequency Sweep
- Sweep 0.5 → 10 Hz
- Identify bandwidth and resonance

---

## Expected Results

### MLP/LSTM vs Gazebo LOW

Both use **Kp=25, Kd=0.5**, should show:
- Similar rise time (~80-100 ms)
- Similar overshoot (~5-10%)
- Small tracking errors (<10 mrad RMSE at 1 Hz)

**Differences due to:**
- Neural network learned dynamics
- Simulation physics differences

---

### Implicit vs Gazebo HIGH

Both use **Kp=160, Kd=5**, should show:
- Fast rise time (~20-30 ms)
- Minimal overshoot (<5%)
- Very accurate tracking (<5 mrad RMSE at 1 Hz)

**Should match very closely** (both are PD controllers!)

---

## Output Structure

```
actuator_analysis/
├── isaaclab/
│   ├── mlp/
│   │   ├── test_results_*.json
│   │   └── plots_*/
│   ├── lstm/
│   └── implicit/
│
├── gazebo/
│   ├── pd_low/       # Kp=25, Kd=0.5
│   └── pd_high/      # Kp=160, Kd=5
│
└── comparison/
    ├── mlp_vs_gazebo_low/
    │   ├── comparison_step_response.png
    │   ├── comparison_sine_1hz.png
    │   ├── comparison_sine_5hz.png
    │   └── comparison_metrics.csv
    ├── lstm_vs_gazebo_low/
    └── implicit_vs_gazebo_high/
```

---

## How to Read Results

### Plots

**comparison_step_response.png:**
- Shows position, velocity, torque over time
- Blue = IsaacLab, Red = Gazebo
- Check if curves overlap

**comparison_sine_1hz.png / 5hz.png:**
- Shows tracking performance
- Check error plot (should be small)
- Check if phase lag is similar

### Metrics CSV

| Test | Metric | IsaacLab | Gazebo | Difference |
|------|--------|----------|--------|------------|
| Step | Rise Time (ms) | 82 | 78 | 4 ✅ |
| Step | Overshoot (%) | 5.2 | 4.8 | 0.4 ✅ |
| Sine 1Hz | RMSE (mrad) | 8.5 | 9.2 | 0.7 ✅ |

**Good Match:**
- Rise time difference < 20%
- Overshoot difference < 5%
- RMSE difference < 10 mrad

---

## Troubleshooting

### IsaacLab Test Fails

**Problem:** Module import errors
**Solution:** Check IsaacLab environment activated

**Problem:** Robot doesn't spawn
**Solution:** Verify asset paths and URDF/USD files

---

### Gazebo Test Fails

**Problem:** "No /joint_states topic"
**Solution:**
```bash
# Check Gazebo running
ros2 topic list | grep joint

# If missing, restart Gazebo
ros2 launch go2_gazebo go2_control.launch.py
```

**Problem:** "Joint not found"
**Solution:**
```bash
# Check available joints
ros2 topic echo /joint_states --once

# Use exact joint name from output
```

---

### Poor Match Between Sims

**Problem:** Large rise time difference
**Check:**
1. PD gains loaded correctly in Gazebo
   ```bash
   ros2 param get /controller_manager/go2_position_controller FL_hip_joint.p
   # Should show 25.0 for LOW or 160.0 for HIGH
   ```

2. Physics timestep matches (5ms)
3. Control frequency matches (50 Hz)

**Problem:** Different torque profiles
**Check:**
1. Joint damping in Gazebo URDF
2. Ground friction settings
3. Mass/inertia properties

---

## Manual Testing

### Test Specific Actuator

**IsaacLab:**
```bash
python scripts/actuator_comparison/test_isaaclab_actuators.py \
  --actuator mlp \
  --joint FL_hip_joint \
  --test step \
  --duration 5.0 \
  --headless
```

**Gazebo:**
```bash
python scripts/actuator_comparison/test_gazebo_motors.py \
  --joint FL_hip_joint \
  --pd_gains low \
  --test step \
  --duration 5.0
```

---

## Understanding Actuator Types

### MLP/LSTM Actuators

**How they work:**
- Neural network: (q_desired, q_actual, dq) → torque
- Learns real motor dynamics (delays, friction, limits)
- Trained on real hardware or physics sim

**Why LOW gains:**
- Network provides control authority
- HIGH gains would conflict with learned dynamics
- Allows network to compensate for non-linearities

**Transfer learning:**
- Need to retrain on target platform
- Learns platform-specific dynamics

---

### Implicit (IdealPD) Actuators

**How they work:**
- Direct PD control: τ = Kp(q_des - q) - Kd*dq
- No learned dynamics, pure physics
- Instant response (no delays)

**Why HIGH gains:**
- Need stiff control for accurate tracking
- No learned compensation available
- Relies on PD gains for performance

**Transfer learning:**
- Doesn't capture real motor dynamics
- Sim-to-real gap larger than neural actuators
- Faster to train, simpler to implement

---

## Next Steps After Testing

### If Results Match Well

✅ Gazebo PD gains configured correctly
✅ Physics parameters aligned
✅ Ready for policy data collection in Gazebo

Proceed with:
```bash
# Continue with Gazebo data collection
# using matched PD gains for each policy
```

### If Results Don't Match

❌ Investigate differences:
1. Verify PD gains
2. Check physics timestep
3. Review URDF damping/friction
4. Compare ground properties
5. Check mass/inertia

**Document differences** for analysis when comparing policy performance!

---

## Integration with Policy Testing

### For Data Collection

When collecting data in Gazebo (per policy):

**MLP/LSTM policies:**
```bash
# Use LOW gains config
ros2 launch go2_gazebo go2_control.launch.py \
  controller_config:=go2_controllers_mlp_lstm.yaml
```

**Implicit policies:**
```bash
# Use HIGH gains config
ros2 launch go2_gazebo go2_control.launch.py \
  controller_config:=go2_controllers_implicit.yaml
```

### Understanding Policy Performance Differences

If policy performs differently in Gazebo vs IsaacLab:
1. Check actuator response matching (this tool!)
2. Check observation matching
3. Check environment parameters (friction, mass, etc.)
4. Check velocity command injection

---

## Files Reference

**Scripts:**
- `test_all_actuators_isaaclab.sh` - Test all IsaacLab actuators
- `test_gazebo_motors_all.sh` - Test Gazebo with both gains
- `compare_all_actuators.sh` - Generate comparisons

**Tools:**
- `scripts/actuator_comparison/test_isaaclab_actuators.py`
- `scripts/actuator_comparison/test_gazebo_motors.py`
- `scripts/actuator_comparison/compare_actuators.py`

**Documentation:**
- `scripts/actuator_comparison/README.md` - Detailed guide
- `ACTUATOR_COMPARISON_GUIDE.md` - This file

---

## Summary

1. **Test IsaacLab**: `./test_all_actuators_isaaclab.sh` (3 min)
2. **Test Gazebo**: `./test_gazebo_motors_all.sh` (2 min)
3. **Compare**: `./compare_all_actuators.sh` (<1 min)

**Goal:** Verify Gazebo motor responses match IsaacLab actuator dynamics for valid sim-to-sim comparison!

---

**Last Updated:** 2026-02-10
**Status:** ✅ Ready to use

**Questions?** See `scripts/actuator_comparison/README.md` for detailed docs.

# Motor Testing Guide - Hanging Configuration

## Overview

Test Go2 motors while robot is **suspended in the air** (no ground contact).

**Why?** Test motors in isolation without ground interference, verify commands work, check joint limits.

---

## Quick Start

### IsaacLab (Interactive)

```bash
cd /home/drl-68/unitree_rl_lab
./test_motors_hanging_isaac.sh
```

**Menu options:**
1. Test single joint (sine wave)
2. Test single joint (step response)
3. Test ALL joints (5s each)
4. Custom test

---

### Gazebo (Interactive)

**Start Gazebo first:**
```bash
# Terminal 1
ros2 launch go2_gazebo go2_control.launch.py \
  spawn_z:=1.5 \
  controller_config:=go2_controllers_mlp_lstm.yaml
```

**Run test:**
```bash
# Terminal 2
./test_motors_hanging_gazebo.sh
```

---

## Manual Commands

### IsaacLab - Single Joint

```bash
# Test FL hip with MLP actuator (sine wave)
python scripts/motor_testing/test_motors_hanging.py \
  --actuator mlp \
  --joint FL_hip_joint \
  --motion sine

# Test with Implicit actuator (step response)
python scripts/motor_testing/test_motors_hanging.py \
  --actuator implicit \
  --joint FR_thigh_joint \
  --motion step \
  --headless
```

**Options:**
- `--actuator`: `mlp`, `lstm`, or `implicit`
- `--joint`: Joint name or `all`
- `--motion`: `sine`, `step`, or `sweep`

---

### IsaacLab - All Joints

```bash
# Test all 12 joints sequentially
python scripts/motor_testing/test_motors_hanging.py \
  --actuator lstm \
  --joint all \
  --motion sine
```

**Behavior:** Each joint moves for 5 seconds, cycles continuously

---

### Gazebo - Single Joint

```bash
# Test FL hip (sine, 20s)
python scripts/motor_testing/test_motors_hanging_gazebo.py \
  --joint FL_hip_joint \
  --motion sine \
  --duration 20

# Test calf (step, 10s)
python scripts/motor_testing/test_motors_hanging_gazebo.py \
  --joint FL_calf_joint \
  --motion step \
  --duration 10
```

---

### Gazebo - All Joints

```bash
# Test all joints (60s total)
python scripts/motor_testing/test_motors_hanging_gazebo.py \
  --motion sine \
  --duration 60
```

---

## Motion Types

| Motion | Description | Use Case |
|--------|-------------|----------|
| **sine** | Smooth oscillation (1 Hz) | Visualize smooth response |
| **step** | Alternate ±0.5 rad | Test speed and damping |
| **sweep** | Slow range sweep | Test full motion range |

---

## Joint Names (12 total)

```
Front Left (FL):     Front Right (FR):     Rear Left (RL):      Rear Right (RR):
  FL_hip_joint         FR_hip_joint          RL_hip_joint         RR_hip_joint
  FL_thigh_joint       FR_thigh_joint        RL_thigh_joint       RR_thigh_joint
  FL_calf_joint        FR_calf_joint         RL_calf_joint        RR_calf_joint
```

---

## Actuator Types (IsaacLab)

| Type | PD Gains | Description |
|------|----------|-------------|
| **MLP** | Kp=25, Kd=0.5 | Neural network, learns dynamics |
| **LSTM** | Kp=25, Kd=0.5 | Recurrent network, temporal modeling |
| **Implicit** | Kp=160, Kd=5 | Physics PD, fast and stiff |

---

## Expected Behavior

### Sine Wave
- Joint oscillates smoothly at 1 Hz
- Other joints stay in standing pose
- Amplitude: ±0.5 rad from default

### Step
- Joint snaps to new position
- Holds for 2.5s, then switches
- May overshoot slightly

### Sweep
- Slow continuous motion over 5s
- Covers ±0.5 rad range
- Returns to start

---

## Hanging Configuration

**IsaacLab:** Robot at 1.5m height, base position maintained automatically

**Gazebo:** Spawn at height via `spawn_z:=1.5` parameter

**Why 1.5m?** High enough to avoid ground, legs have full freedom

---

## Troubleshooting

### IsaacLab

**Robot falls:** Script maintains base position automatically every 10 steps

**Joint doesn't move:** Check actuator type, joint name spelling, command limits

---

### Gazebo

**Robot not at height:**
```bash
ros2 launch go2_gazebo go2_control.launch.py spawn_z:=1.5
```

**No joint states:**
```bash
ros2 topic list | grep joint  # Check if available
```

**Controller not working:**
```bash
ros2 control list_controllers  # Verify active
```

---

## Use Cases

1. **Verify motors work** - Test each joint responds to commands
2. **Check joint limits** - Sweep through range, verify limits
3. **Compare actuators** - Run same test with MLP/LSTM/Implicit
4. **Debug control** - Isolate joint issues without ground contact
5. **Calibrate zeros** - Find neutral position for each joint

---

## Comparison: IsaacLab vs Gazebo

**IsaacLab:**
- ✅ Multiple actuator types (MLP/LSTM/Implicit)
- ✅ GPU-accelerated physics
- ✅ Automatic base fixing
- ✅ Actuator-specific PD gains

**Gazebo:**
- ✅ Standard PD controller
- ✅ Configurable gains
- ✅ ROS 2 integration
- ✅ Real-time visualization

**Both:**
- 50 Hz control frequency
- Same joint limits
- Same standing pose
- Same motion commands

---

## Safety

**Simulation:**
- ✅ Safe to test extreme motions
- ✅ No risk of damage

**Real Hardware:**
- 🚫 DO NOT use these scripts on real Go2
- ✅ Use Unitree official SDK
- ✅ Implement proper safety limits

---

## Files

```
test_motors_hanging_isaac.sh          # IsaacLab helper script
test_motors_hanging_gazebo.sh         # Gazebo helper script

scripts/motor_testing/
├── test_motors_hanging.py            # IsaacLab implementation
├── test_motors_hanging_gazebo.py     # Gazebo implementation
└── README.md                          # Detailed documentation
```

---

## Summary

**Test single joint:**
```bash
# IsaacLab
python scripts/motor_testing/test_motors_hanging.py \
  --actuator mlp --joint FL_hip_joint --motion sine

# Gazebo
python scripts/motor_testing/test_motors_hanging_gazebo.py \
  --joint FL_hip_joint --motion sine --duration 20
```

**Test all joints:**
```bash
# IsaacLab
python scripts/motor_testing/test_motors_hanging.py \
  --actuator mlp --joint all --motion sine

# Gazebo
python scripts/motor_testing/test_motors_hanging_gazebo.py \
  --motion sine --duration 60
```

**Or use interactive menus:**
```bash
./test_motors_hanging_isaac.sh    # IsaacLab
./test_motors_hanging_gazebo.sh   # Gazebo
```

---

**Goal:** Test individual motors in isolation!

**Documentation:** See [scripts/motor_testing/README.md](scripts/motor_testing/README.md)

---

**Last Updated:** 2026-02-10

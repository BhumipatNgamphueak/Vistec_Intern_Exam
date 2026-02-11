# Motor Testing - Hanging Configuration

Test individual Go2 motors while robot is suspended in the air (no ground contact).

## Purpose

**Why hang the robot?**
- Test motors in isolation without ground interference
- Verify joint commands work correctly
- Check joint limits and range of motion
- Debug control issues
- Visualize actuator responses
- Compare different actuator types (MLP/LSTM/Implicit)

**Use cases:**
- Motor calibration
- Control debugging
- Actuator comparison
- Joint limit verification
- Command latency testing

---

## Quick Start

### IsaacLab

```bash
cd /home/drl-68/unitree_rl_lab

# Interactive menu
./test_motors_hanging_isaac.sh
```

**Options:**
1. Test single joint (sine wave)
2. Test single joint (step response)
3. Test ALL joints sequentially
4. Custom test

---

### Gazebo

**First, start Gazebo with robot at height:**
```bash
# Terminal 1
ros2 launch go2_gazebo go2_control.launch.py \
  spawn_z:=1.5 \
  controller_config:=go2_controllers_mlp_lstm.yaml
```

**Then run test:**
```bash
# Terminal 2
cd /home/drl-68/unitree_rl_lab
./test_motors_hanging_gazebo.sh
```

---

## Manual Usage

### IsaacLab - Test Specific Joint

```bash
# Test FL hip with MLP actuator (sine wave)
python scripts/motor_testing/test_motors_hanging.py \
  --actuator mlp \
  --joint FL_hip_joint \
  --motion sine

# Test FR thigh with Implicit actuator (step)
python scripts/motor_testing/test_motors_hanging.py \
  --actuator implicit \
  --joint FR_thigh_joint \
  --motion step \
  --headless
```

**Parameters:**
- `--actuator`: `mlp`, `lstm`, or `implicit`
- `--joint`: Joint name or `all`
- `--motion`: `sine`, `step`, or `sweep`
- `--headless`: Run without GUI

---

### IsaacLab - Test All Joints

```bash
# Test all 12 joints with LSTM actuator
python scripts/motor_testing/test_motors_hanging.py \
  --actuator lstm \
  --joint all \
  --motion sine
```

**Behavior:**
- Each joint moves for 5 seconds
- Cycles through all 12 joints
- Other joints stay in standing pose
- Run indefinitely (Ctrl+C to stop)

---

### Gazebo - Test Specific Joint

```bash
# Test FL hip (sine wave, 20 seconds)
python scripts/motor_testing/test_motors_hanging_gazebo.py \
  --joint FL_hip_joint \
  --motion sine \
  --duration 20

# Test calf joint (step, 10 seconds)
python scripts/motor_testing/test_motors_hanging_gazebo.py \
  --joint FL_calf_joint \
  --motion step \
  --duration 10
```

**Parameters:**
- `--joint`: Joint name (omit for all)
- `--motion`: `sine`, `step`, or `sweep`
- `--duration`: Test duration in seconds

---

### Gazebo - Test All Joints

```bash
# Test all joints (60 seconds total, 5s each)
python scripts/motor_testing/test_motors_hanging_gazebo.py \
  --motion sine \
  --duration 60
```

---

## Motion Types

### 1. Sine Wave
**Command:** `pos = default + amplitude * sin(2πft)`

**Parameters:**
- Amplitude: 0.5 rad
- Frequency: 1 Hz

**Use:** Smooth continuous motion, good for visualizing response

---

### 2. Step Response
**Command:** Alternates between +0.5 rad and -0.5 rad

**Timing:**
- Hold high for 2.5s
- Hold low for 2.5s
- Repeat

**Use:** Test actuator speed and damping

---

### 3. Sweep
**Command:** Slowly sweep through range

**Range:** ±0.5 rad from default

**Duration:** 5 seconds per cycle

**Use:** Test full range of motion

---

## Joint Names

Go2 has **12 joints** (4 legs × 3 joints):

```
Front Left (FL):          Front Right (FR):
  FL_hip_joint              FR_hip_joint
  FL_thigh_joint            FR_thigh_joint
  FL_calf_joint             FR_calf_joint

Rear Left (RL):           Rear Right (RR):
  RL_hip_joint              RR_hip_joint
  RL_thigh_joint            RR_thigh_joint
  RL_calf_joint             RR_calf_joint
```

### Joint Limits

| Joint | Lower | Upper | Velocity Limit |
|-------|-------|-------|----------------|
| Hip | -60° (-1.047 rad) | +60° (1.047 rad) | 23 rad/s |
| Thigh | -38° (-0.663 rad) | +170° (2.966 rad) | 23 rad/s |
| Calf | -156° (-2.721 rad) | -48° (-0.837 rad) | 14 rad/s |

---

## Default Standing Pose

```python
FL: [0.0,  0.8, -1.5]  # hip, thigh, calf (rad)
FR: [0.0,  0.8, -1.5]
RL: [0.0,  1.0, -1.5]
RR: [0.0,  1.0, -1.5]
```

**Note:** Rear legs have slightly different thigh angle (1.0 vs 0.8)

---

## Actuator Types (IsaacLab Only)

### MLP Actuator
- **Type:** Neural network
- **PD Gains:** Kp=25.0, Kd=0.5
- **Characteristics:** Learns actuator dynamics
- **Response:** Smoother, models real motor behavior

### LSTM Actuator
- **Type:** Recurrent neural network
- **PD Gains:** Kp=25.0, Kd=0.5
- **Characteristics:** Temporal dynamics modeling
- **Response:** Similar to MLP but with memory

### Implicit Actuator
- **Type:** Physics-based PD
- **PD Gains:** Kp=160.0, Kd=5.0
- **Characteristics:** Direct PD control
- **Response:** Faster, stiffer, ideal tracking

---

## Expected Behavior

### Visual Observation

**Sine wave:**
- Joint oscillates smoothly
- Other joints remain still in standing pose
- Should complete ~1 cycle per second

**Step:**
- Joint snaps to new position
- Quick rise time
- May overshoot slightly
- Settles at target

**Sweep:**
- Slow continuous motion
- Covers full test range
- Returns to start

---

### Hanging Height

**IsaacLab:** 1.5 meters (automatically maintained)
**Gazebo:** Set via `spawn_z:=1.5` parameter

**Why 1.5m?**
- High enough to avoid ground
- Robot legs have full freedom
- Easy to visualize in camera view

---

## Troubleshooting

### IsaacLab Issues

**Problem:** Robot falls to ground
**Cause:** Base position not maintained
**Solution:** Script automatically fixes base every 10 steps

**Problem:** Joint doesn't move
**Cause:** Command not being applied
**Check:**
- Actuator type correct?
- Joint name spelled correctly?
- Command within joint limits?

---

### Gazebo Issues

**Problem:** Robot not spawned at height
**Solution:**
```bash
ros2 launch go2_gazebo go2_control.launch.py spawn_z:=1.5
```

**Problem:** Robot falls when test starts
**Cause:** Gravity enabled, no base fixed
**Solution:**
```bash
# In Gazebo, you may need to disable gravity on base link
# Or use fixed joint to world
```

**Problem:** "No /joint_states topic"
**Solution:**
```bash
# Check Gazebo running
ros2 topic list | grep joint

# Restart joint_state_broadcaster if needed
ros2 control load_controller joint_state_broadcaster
```

**Problem:** Commands not working
**Check:**
```bash
# Verify controller loaded
ros2 control list_controllers

# Should show:
# go2_position_controller[position_controllers/JointGroupPositionController] active
```

---

## Comparison: IsaacLab vs Gazebo

### Differences

| Aspect | IsaacLab | Gazebo |
|--------|----------|--------|
| Actuators | MLP/LSTM/Implicit | PD controller |
| PD Gains | Actuator-specific | Configurable |
| Base Fixing | Automatic | Manual (spawn height) |
| Physics | GPU-accelerated | CPU |
| Visualization | IsaacSim | Gazebo |

### Similarities

| Aspect | Both Platforms |
|--------|---------------|
| Control Frequency | 50 Hz |
| Joint Limits | Same |
| Standing Pose | Same |
| Motion Commands | Same |

---

## Advanced Usage

### Test Multiple Joints Simultaneously

**IsaacLab** (modify script):
```python
# Move multiple joints at once
cmd[0, joint_idx_1] = target_1
cmd[0, joint_idx_2] = target_2
```

**Gazebo:**
```python
# Set positions for multiple joints
positions = [pos1, pos2, ..., pos12]
tester.send_command(positions)
```

---

### Custom Motion Patterns

**Example: Square wave**
```python
if int(local_t) % 2 == 0:
    cmd[joint] = default + 0.5
else:
    cmd[joint] = default - 0.5
```

**Example: Triangle wave**
```python
progress = (local_t % cycle_time) / cycle_time
if progress < 0.5:
    cmd[joint] = default + (progress * 2) * amplitude
else:
    cmd[joint] = default + ((1 - progress) * 2) * amplitude
```

---

### Record Joint Responses

**Gazebo:**
```bash
# Record all topics
ros2 bag record /joint_states /go2_position_controller/commands

# Then analyze later
ros2 bag play <bag_file>
```

**IsaacLab:**
Modify script to log data:
```python
import csv
with open('joint_data.csv', 'w') as f:
    writer = csv.writer(f)
    writer.writerow(['time', 'position', 'velocity', 'torque'])
    # ... log data in loop
```

---

## Use Cases

### 1. Verify Motor Installation
Test each joint individually to confirm:
- Motor responds to commands
- Direction is correct
- Range of motion is correct

### 2. Calibrate Joint Zeros
Find neutral position for each joint:
```bash
# Command joint to 0.0 and visually check
# Adjust URDF if misaligned
```

### 3. Check Joint Limits
Sweep through range and verify:
- Doesn't exceed physical limits
- Software limits match hardware

### 4. Compare Actuator Types
Run same test with different actuators:
```bash
# MLP
python ... --actuator mlp --motion sine

# Implicit
python ... --actuator implicit --motion sine

# Compare visual response
```

### 5. Debug Control Issues
If walking policy has issues:
- Test individual joints in hanging mode
- Verify commands are reaching motors
- Check for unexpected behavior

---

## Safety Notes

**IsaacLab:**
- ✅ Simulation only, no safety concerns
- Robot can't fall or break
- Safe to test extreme motions

**Gazebo:**
- ✅ Simulation only
- ⚠️ Be careful with real robot connections
- 🛑 DO NOT run on real hardware without safety checks

**Real Hardware:**
- 🚫 DO NOT use these scripts directly on real Go2
- ✅ Use Unitree's official SDK
- ✅ Implement proper safety limits
- ✅ Test in safe environment

---

## Files

```
unitree_rl_lab/
├── scripts/motor_testing/
│   ├── test_motors_hanging.py         # IsaacLab script
│   ├── test_motors_hanging_gazebo.py  # Gazebo script
│   └── README.md                       # This file
│
├── test_motors_hanging_isaac.sh       # IsaacLab helper
└── test_motors_hanging_gazebo.sh      # Gazebo helper
```

---

## Summary

**IsaacLab:**
```bash
./test_motors_hanging_isaac.sh
```

**Gazebo:**
```bash
# Terminal 1: Start Gazebo
ros2 launch go2_gazebo go2_control.launch.py spawn_z:=1.5

# Terminal 2: Run test
./test_motors_hanging_gazebo.sh
```

**Goal:** Test individual motors in isolation without ground interference!

---

**Last Updated:** 2026-02-10
**Version:** 1.0

# 4 Training Tasks - Velocity Command Reference

This document describes the **4 locomotion tasks** used for training and testing the GO2 robot policies.

---

## 📋 Task Overview

All tasks defined in: `generate_4_task_episodes.py` and `episode_configs_4tasks.yaml`

Total episodes: **4 tasks × 50 repeats = 200 episodes**
- Episode duration: **20 seconds**
- Control frequency: **50 Hz**

---

## 🎯 Task Definitions

### TASK 1: Standing
**Goal**: Maintain stable standing position with zero velocity

| Parameter | Value |
|-----------|-------|
| `lin_vel_x` | 0.0 m/s |
| `lin_vel_y` | 0.0 m/s |
| `ang_vel_z` | 0.0 rad/s |
| Duration | 20.0 s |

**Menu Options**:
- **Isaac**: `--task 1`
- **Gazebo**: Option 1

---

### TASK 2: Walking
**Goal**: Walk forward at varying speeds

| Variant | Speed | lin_vel_x | Duration |
|---------|-------|-----------|----------|
| Slow | 0.5 m/s | 0.5 | 5.0 s |
| Normal | 1.0 m/s | 1.0 | 5.0 s |
| Fast | 1.5 m/s | 1.5 | 5.0 s |
| Moderate | 0.8 m/s | 0.8 | 5.0 s |

**All with**: `lin_vel_y=0.0`, `ang_vel_z=0.0`

**Menu Options**:
- **Isaac**: `--task 2.1` (slow), `--task 2.2` (normal), `--task 2.3` (fast), `--task 2.4` (moderate)
- **Gazebo**: Options 2, 3, 4, 5

---

### TASK 3: Turn in Place
**Goal**: Rotate in place without forward motion

| Variant | Rate | ang_vel_z | Direction | Duration |
|---------|------|-----------|-----------|----------|
| Slow CCW | +0.5 rad/s | +0.5 | Counter-clockwise | 5.0 s |
| Normal CCW | +1.0 rad/s | +1.0 | Counter-clockwise | 5.0 s |
| Normal CW | -1.0 rad/s | -1.0 | **Clockwise** | 5.0 s |
| Fast CCW | +1.5 rad/s | +1.5 | Counter-clockwise | 5.0 s |

**All with**: `lin_vel_x=0.0`, `lin_vel_y=0.0`

**Menu Options**:
- **Isaac**: `--task 3.1`, `--task 3.2`, `--task 3.3`, `--task 3.4`
- **Gazebo**: Options 6, 7, 8, 9

**Note**: Task 3.3 includes a **direction change** (CW vs CCW) to test adaptability.

---

### TASK 4: Walk + Turn (Combined Maneuvers)
**Goal**: Execute combined forward and rotational motion

| Variant | Description | lin_vel_x | ang_vel_z | Duration |
|---------|-------------|-----------|-----------|----------|
| Right arc | Arc to the right | 0.8 m/s | +0.6 rad/s | 5.0 s |
| Straight fast | Fast forward | 1.2 m/s | 0.0 rad/s | 2.0 s |
| Left arc | Arc to the left | 0.8 m/s | -0.6 rad/s | 5.0 s |
| Tight turn | Circle walk | 0.5 m/s | +1.0 rad/s | 5.0 s |

**All with**: `lin_vel_y=0.0`

**Menu Options**:
- **Isaac**: `--task 4.1`, `--task 4.2`, `--task 4.3`, `--task 4.4`
- **Gazebo**: Options 10, 11, 12, 13

---

## 🚀 Usage Examples

### Isaac Lab

```bash
# List all tasks
python send_velocity_commands_isaac.py --list

# Test Task 2.2 (Walk normal - 1.0 m/s)
python send_velocity_commands_isaac.py --task 2.2

# Test Task 4.1 (Right arc)
python send_velocity_commands_isaac.py --task 4.1
```

Then modify the config file as shown and run:
```bash
python scripts/rsl_rl/play.py \
    --task Unitree-Go2-Velocity-MLP-Custom \
    --load_run 2026-02-03_15-54-07_work_good \
    --num_envs 1
```

### Gazebo

```bash
# Launch Gazebo with policy
ros2 launch deploy_policy go2_rl_policy.launch.py \
    policy_path:=.../policy.onnx \
    actuator_type:=mlp

# In another terminal, run interactive menu
./send_velocity_commands_gazebo.sh

# Select desired task:
#   Option 3 = Task 2.2 (Walk normal)
#   Option 7 = Task 3.2 (Turn normal CCW)
#   Option 10 = Task 4.1 (Right arc)
```

---

## 📊 Training Distribution

Each task is repeated **50 times** with randomized initial conditions:
- Base height: 0.27 - 0.33 m
- Initial roll/pitch: ±0.05 rad
- Joint positions: ±0.03 rad noise

This ensures the policy learns robust behavior across diverse starting states.

---

## 🎓 Why These 4 Tasks?

1. **TASK 1 (Standing)**: Tests balance and stability
2. **TASK 2 (Walking)**: Tests speed adaptation and gait
3. **TASK 3 (Turn in Place)**: Tests rotation without drift
4. **TASK 4 (Walk + Turn)**: Tests combined maneuvers and coordination

Together, these cover the **fundamental locomotion primitives** needed for general quadruped control.

---

## 🔗 Related Files

- **Episode Generator**: `generate_4_task_episodes.py`
- **Episode Config**: `episode_configs_4tasks.yaml` (200 episodes)
- **Isaac Command Script**: `send_velocity_commands_isaac.py`
- **Gazebo Command Script**: `send_velocity_commands_gazebo.sh`
- **Deployment Guide**: `DEPLOYMENT_GUIDE.md`

---

**Last Updated**: 2026-02-11

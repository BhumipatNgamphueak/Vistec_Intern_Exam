# Guide: Spawn GO2 Robot Floating in the Sky

Multiple methods to spawn the GO2 robot suspended in the air for testing joint movements without ground contact.

---

## Quick Start (Recommended)

```bash
# Easiest method - automated script
cd /home/drl-68/vistec_ex_ws
./spawn_go2_floating.sh 2.0
```

This will:
1. Launch Gazebo with GO2 at 2.0m height
2. Wait 15 seconds for initialization
3. Automatically disable gravity
4. Robot floats in the air ✨

---

## Method 1: Automated Script (Easiest)

### Usage
```bash
./spawn_go2_floating.sh [height_in_meters]

# Examples:
./spawn_go2_floating.sh         # Default: 2.0m
./spawn_go2_floating.sh 5.0     # Spawn at 5.0m
./spawn_go2_floating.sh 10.0    # Spawn at 10.0m
```

### What it does
- Spawns robot at specified height
- Disables gravity after 15 seconds
- Robot stays suspended indefinitely
- Perfect for testing joint movements

---

## Method 2: Manual Launch + Gravity Toggle

### Step 1: Launch robot at high altitude
```bash
ros2 launch go2_gazebo_simulation go2_fortress.launch.py z_pose:=2.0
```

### Step 2: Wait for spawn (15 seconds)
```bash
# Watch the Gazebo window until robot appears
```

### Step 3: Disable gravity
```bash
# Option A: Disable world gravity (affects everything)
ign service -s /world/default/set_physics \
  --reqtype ignition.msgs.Physics \
  --reptype ignition.msgs.Boolean \
  --timeout 1000 \
  --req 'gravity: {x: 0, y: 0, z: 0}'

# Option B: Use Python helper script
python3 toggle_robot_gravity.py off
```

### Step 4: Re-enable gravity when done
```bash
# Re-enable gravity
ign service -s /world/default/set_physics \
  --reqtype ignition.msgs.Physics \
  --reptype ignition.msgs.Boolean \
  --timeout 1000 \
  --req 'gravity: {x: 0, y: 0, z: -9.81}'

# Or use helper script
python3 toggle_robot_gravity.py on
```

---

## Method 3: Different Heights

You can spawn at any height by changing the `z_pose` parameter:

```bash
# Ground level (will fall)
ros2 launch go2_gazebo_simulation go2_fortress.launch.py z_pose:=0.5

# Low altitude (1 meter)
ros2 launch go2_gazebo_simulation go2_fortress.launch.py z_pose:=1.0

# Medium altitude (2 meters) - Good for testing
ros2 launch go2_gazebo_simulation go2_fortress.launch.py z_pose:=2.0

# High altitude (5 meters) - Easier to see
ros2 launch go2_gazebo_simulation go2_fortress.launch.py z_pose:=5.0

# Very high (10 meters) - For visualization
ros2 launch go2_gazebo_simulation go2_fortress.launch.py z_pose:=10.0
```

**Note:** Without disabling gravity, the robot will fall!

---

## Method 4: Custom Position (X, Y, Z)

```bash
# Spawn at specific 3D position
ros2 launch go2_gazebo_simulation go2_fortress.launch.py \
  x_pose:=2.0 \
  y_pose:=1.0 \
  z_pose:=3.0
```

---

## Use Cases

### 1. Joint Movement Testing
```bash
# Spawn floating robot
./spawn_go2_floating.sh 2.0

# In another terminal, test joint commands
ros2 topic pub /model/go2_robot/joint/FL_hip_joint/cmd_pos \
  std_msgs/msg/Float64 "data: 0.5" --once
```

### 2. Visualization and Debugging
```bash
# Spawn at high altitude for better camera view
./spawn_go2_floating.sh 5.0

# Use Gazebo camera controls to orbit around robot
```

### 3. Zero-G Simulation
```bash
# Simulate space environment
./spawn_go2_floating.sh 2.0

# All objects in the world will float
```

### 4. Collision Testing Without Gravity
```bash
# Test collision detection without falling
ros2 launch go2_gazebo_simulation go2_fortress.launch.py z_pose:=2.0
python3 toggle_robot_gravity.py off
```

---

## Troubleshooting

### Robot Falls Immediately
**Problem:** Robot falls after spawning
**Solution:** Disable gravity faster, or increase initial height:
```bash
./spawn_go2_floating.sh 10.0
```

### Gravity Service Fails
**Problem:** `ign service` command not found
**Solution:** Source Gazebo Ignition environment:
```bash
source /usr/share/gazebo/setup.sh
# Or for ROS2 Fortress
source /opt/ros/galactic/setup.bash
```

### Robot Not Responding
**Problem:** Joint commands don't move the robot
**Solution:** Make sure bridges are running (check launch file output)

### Camera Can't Find Robot
**Problem:** Robot too high to see
**Solution:**
1. In Gazebo GUI: Right-click robot → "Move to"
2. Or spawn at lower height: `z_pose:=2.0`

---

## Checking Current State

### Check if gravity is enabled
```bash
# Get current physics settings
ign topic -e -t /world/default/stats

# Or check in Gazebo GUI:
# View → World Stats → Physics → Gravity
```

### Check robot position
```bash
# Subscribe to robot pose
ros2 topic echo /robot_pose
```

---

## Advanced: Freeze Robot in Place

To completely fix the robot at a position (no movement at all):

```bash
# 1. Spawn robot
ros2 launch go2_gazebo_simulation go2_fortress.launch.py z_pose:=2.0

# 2. Disable gravity
python3 toggle_robot_gravity.py off

# 3. Set all joint positions to 0 (default standing pose)
for joint in FL_hip_joint FL_thigh_joint FL_calf_joint \
             FR_hip_joint FR_thigh_joint FR_calf_joint \
             RL_hip_joint RL_thigh_joint RL_calf_joint \
             RR_hip_joint RR_thigh_joint RR_calf_joint; do
  ros2 topic pub /model/go2_robot/joint/${joint}/cmd_pos \
    std_msgs/msg/Float64 "data: 0.0" --once
done
```

---

## Files Created

- `spawn_go2_floating.sh` - Automated spawn + gravity disable
- `toggle_robot_gravity.py` - Python script to toggle gravity
- `FLOATING_GO2_GUIDE.md` - This guide

---

## Quick Reference

| Command | Effect |
|---------|--------|
| `./spawn_go2_floating.sh` | Spawn at 2m, no gravity |
| `./spawn_go2_floating.sh 5.0` | Spawn at 5m, no gravity |
| `python3 toggle_robot_gravity.py off` | Disable gravity |
| `python3 toggle_robot_gravity.py on` | Enable gravity |
| `z_pose:=X` | Set spawn height to X meters |

---

**Tip:** For the cleanest floating effect, use `./spawn_go2_floating.sh` with height 2-5 meters.
This gives you a stable, floating robot perfect for testing!

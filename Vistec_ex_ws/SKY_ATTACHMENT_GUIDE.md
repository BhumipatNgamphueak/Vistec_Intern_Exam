# GO2 Sky Attachment Guide

Complete guide to spawning GO2 robot attached to the sky using URDF fixed joints.

---

## ✅ Method Comparison

| Method | Stability | Realism | Ease | Best For |
|--------|-----------|---------|------|----------|
| **URDF Fixed Joint** ⭐ | Excellent | Low | Easy | Joint testing |
| Disabled Gravity | Good | Low | Medium | Zero-G simulation |
| High Spawn Only | Poor | Medium | Easy | Visualization |

**Recommended:** URDF Fixed Joint (most stable)

---

## Quick Start (URDF Fixed Joint)

```bash
cd /home/drl-68/vistec_ex_ws

# Build the workspace (if not already built)
colcon build --packages-select go2_gazebo_simulation

# Source the workspace
source install/setup.bash

# Launch robot hanging from sky
ros2 launch go2_gazebo_simulation go2_hanging.launch.py
```

**Result:** Robot spawns at 2.0m height, **rigidly attached** to the world. Perfect for testing joint movements!

---

## How It Works

### URDF Structure

The hanging URDF (`go2_base_hanging.urdf`) includes:

```xml
<!-- World link (invisible anchor point) -->
<link name="world"/>

<!-- Fixed joint connecting robot base to world -->
<joint name="fixed_base_joint" type="fixed">
  <parent link="world"/>
  <child link="base"/>
  <origin xyz="0 0 2.0" rpy="0 0 0"/>  <!-- Height: 2.0m -->
</joint>
```

**Effect:**
- Robot base is **rigidly attached** to an invisible "world" link
- Cannot fall or move in space
- All joint movements work normally
- No gravity needed (robot is held in place)

### Launch File

`go2_hanging.launch.py` uses the hanging URDF:
- Loads `go2_base_hanging.urdf` instead of `go2_base.urdf`
- Removes spawn position arguments (position set in URDF)
- All other functionality identical to normal launch

---

## Usage

### Basic Launch

```bash
# Default: 2.0m height
ros2 launch go2_gazebo_simulation go2_hanging.launch.py
```

### Custom Height

To change height, edit the URDF:

```bash
# Edit the hanging URDF
nano src/go2_gazebo_simulation/urdf/go2_base_hanging.urdf

# Find line 11:
<origin xyz="0 0 2.0" rpy="0 0 0"/>

# Change Z value (e.g., 5.0 for 5 meters):
<origin xyz="0 0 5.0" rpy="0 0 0"/>

# Rebuild
colcon build --packages-select go2_gazebo_simulation
source install/setup.bash

# Launch again
ros2 launch go2_gazebo_simulation go2_hanging.launch.py
```

### Test Joint Movements

```bash
# Terminal 1: Launch hanging robot
ros2 launch go2_gazebo_simulation go2_hanging.launch.py

# Terminal 2: Move a joint
ros2 topic pub /model/go2_robot/joint/FL_hip_joint/cmd_pos \
  std_msgs/msg/Float64 "data: 0.5" --once

# Watch the leg move while robot stays perfectly still!
```

---

## Advantages of URDF Fixed Joint

### ✅ Pros:
1. **Most Stable** - Robot cannot drift or fall
2. **No Gravity Needed** - Fixed joint holds everything
3. **Clean Solution** - No external scripts needed
4. **Deterministic** - Always same height and orientation
5. **Gazebo Native** - Uses standard URDF mechanism
6. **Fast** - No service calls or external commands

### ⚠️ Cons:
1. **No Base Movement** - Robot base is completely fixed
2. **Height Changes Require Rebuild** - Need to edit URDF + rebuild
3. **Less Realistic** - Robot attached to invisible point
4. **Cannot Test Balance** - Base cannot move freely

---

## Use Cases

### Perfect For:
- **Joint Movement Testing** - Verify joint commands work correctly
- **Motor Testing** - Test torque control without balance concerns
- **Visualization** - Clear view of robot from all angles
- **Debugging** - Isolate joint issues from balance/physics
- **Gait Pattern Development** - See leg movements clearly

### Not Suitable For:
- Balance testing (base is fixed)
- Walking simulation (cannot move in space)
- COG/stability testing (no free movement)
- Real-world gait validation (too constrained)

---

## Comparison with Other Methods

### Method 1: URDF Fixed Joint (This Method)
```bash
ros2 launch go2_gazebo_simulation go2_hanging.launch.py
```
**Pros:** Most stable, no scripts needed
**Cons:** Requires rebuild to change height

### Method 2: Disabled Gravity
```bash
./spawn_go2_floating.sh 2.0
```
**Pros:** Easy height changes, still some physics
**Cons:** Less stable, requires script

### Method 3: High Spawn Only
```bash
ros2 launch go2_gazebo_simulation go2_fortress.launch.py z_pose:=10.0
```
**Pros:** Simplest command
**Cons:** Robot falls immediately

---

## File Structure

### Created Files:
```
src/go2_gazebo_simulation/
├── urdf/
│   ├── go2_base.urdf                # Original (free movement)
│   └── go2_base_hanging.urdf       # Hanging version (fixed to sky) ⭐
├── launch/
│   ├── go2_fortress.launch.py      # Original launch
│   └── go2_hanging.launch.py       # Hanging launch ⭐
```

### Scripts:
- `create_hanging_urdf.py` - Generates hanging URDF from original
- `spawn_go2_floating.sh` - Alternative: gravity disable method
- `toggle_robot_gravity.py` - Toggle gravity on/off

---

## Troubleshooting

### Robot Still Falls
**Problem:** Robot falls even with hanging URDF
**Solution:** Make sure you're using the hanging launch file:
```bash
# Wrong:
ros2 launch go2_gazebo_simulation go2_fortress.launch.py

# Correct:
ros2 launch go2_gazebo_simulation go2_hanging.launch.py
```

### Cannot Find Hanging Launch File
**Problem:** `ros2 launch` can't find `go2_hanging.launch.py`
**Solution:** Rebuild and source:
```bash
colcon build --packages-select go2_gazebo_simulation
source install/setup.bash
```

### Want to Change Height
**Solution:** Edit URDF line 11, rebuild, relaunch:
```bash
nano src/go2_gazebo_simulation/urdf/go2_base_hanging.urdf
# Change: <origin xyz="0 0 2.0" ...> to <origin xyz="0 0 5.0" ...>
colcon build --packages-select go2_gazebo_simulation
source install/setup.bash
ros2 launch go2_gazebo_simulation go2_hanging.launch.py
```

### Want Horizontal Orientation
**Solution:** Edit URDF line 11, change RPY (roll-pitch-yaw):
```xml
<!-- Hanging upright (default) -->
<origin xyz="0 0 2.0" rpy="0 0 0"/>

<!-- Hanging sideways -->
<origin xyz="0 0 2.0" rpy="0 1.5708 0"/>

<!-- Hanging upside down -->
<origin xyz="0 0 2.0" rpy="3.1416 0 0"/>
```

---

## Advanced: Create Custom Heights

### Quick Script to Generate Custom URDF

```bash
#!/bin/bash
# generate_hanging_urdf.sh HEIGHT

HEIGHT=${1:-2.0}
URDF_OUT="src/go2_gazebo_simulation/urdf/go2_base_hanging.urdf"

python3 create_hanging_urdf.py
sed -i "s/<origin xyz=\"0 0 [0-9.]*\"/<origin xyz=\"0 0 $HEIGHT\"/" $URDF_OUT

echo "✓ Created hanging URDF at height: ${HEIGHT}m"
colcon build --packages-select go2_gazebo_simulation
source install/setup.bash
```

Usage:
```bash
chmod +x generate_hanging_urdf.sh
./generate_hanging_urdf.sh 5.0  # 5 meters high
ros2 launch go2_gazebo_simulation go2_hanging.launch.py
```

---

## Summary

**Best Method:** URDF Fixed Joint (`go2_hanging.launch.py`)

**When to Use:**
- Testing joint movements in isolation
- Visualizing robot from all angles
- Debugging motor control
- Developing gait patterns

**Quick Command:**
```bash
source /home/drl-68/vistec_ex_ws/install/setup.bash
ros2 launch go2_gazebo_simulation go2_hanging.launch.py
```

**Alternative (Gravity Disable):**
```bash
./spawn_go2_floating.sh 2.0
```

---

**✨ Robot is now perfectly suspended in the sky, ready for testing!**

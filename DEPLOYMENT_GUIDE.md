# Deployment Guide: Trained Policies on Isaac Lab & Gazebo

**Quick guide to deploy and test trained locomotion policies**

---

## ⚙️ Environment Setup (First Time Only)

```bash
# Set workspace paths (adjust to your clone location)
export VISTEC_REPO=~/Vistec_Intern_Exam              # This cloned repo
export UNITREE_LAB=~/unitree_rl_lab                   # Full training repo
export ACTUATOR_NET=~/actuator_net                    # Actuator models repo
export VISTEC_WS=~/vistec_ex_ws                       # ROS 2 workspace

# Add to ~/.bashrc for persistence
echo "export VISTEC_REPO=~/Vistec_Intern_Exam" >> ~/.bashrc
echo "export UNITREE_LAB=~/unitree_rl_lab" >> ~/.bashrc
echo "export ACTUATOR_NET=~/actuator_net" >> ~/.bashrc
echo "export VISTEC_WS=~/vistec_ex_ws" >> ~/.bashrc
source ~/.bashrc
```

---

## 📦 Available Trained Models

Located in `$VISTEC_REPO/trained_models/`:

| Model | Actuator | DR | Iterations | Size | Status |
|-------|----------|----|-----------:|------|--------|
| **mlp_with_dr_24999.pt** | MLP | ✅ | 24,999 | 4.4 MB | ⭐ **RECOMMENDED** |
| implicit_dr_latest.pt | Implicit | ✅ | ~18,400 | 4.4 MB | Ready |
| implicit_no_dr_latest.pt | Implicit | ❌ | ~12,100 | 4.4 MB | Ready |
| lstm_dr_25000.pt | LSTM | ✅ | 25,000 | 4.4 MB | Ready (has obs issues) |

**Full logs location** (if you trained yourself): `$UNITREE_LAB/logs/rsl_rl/`

---

## 🚀 Deployment Option 1: Isaac Lab (Visualization & Testing)

### Prerequisites

```bash
# Isaac Lab environment
cd ~/IsaacLab
./isaaclab.sh --install

# Install unitree_rl_lab (clone if needed)
cd $UNITREE_LAB  # Or: git clone <repo> && cd unitree_rl_lab
~/IsaacLab/isaaclab.sh -p -m pip install -e source/unitree_rl_lab
```

### Method A: Play Policy from Full Logs (If You Trained)

```bash
cd $UNITREE_LAB

# Play MLP policy (RECOMMENDED)
python scripts/rsl_rl/play.py \
    --task Unitree-Go2-Velocity-MLP-Custom \
    --num_envs 32 \
    --load_run 2026-02-03_15-54-07_work_good

# Play Implicit with DR
python scripts/rsl_rl/play.py \
    --task Unitree-Go2-Velocity-Implicit-DR \
    --num_envs 32 \
    --load_run 2026-02-10_16-17-40

# Play LSTM with DR (may have observation issues)
python scripts/rsl_rl/play.py \
    --task Unitree-Go2-Velocity-LSTM-DR \
    --num_envs 32 \
    --load_run 2026-02-07_11-16-35
```

**Camera Controls**:
- Mouse drag: Rotate view
- Mouse wheel: Zoom
- Middle click + drag: Pan

### Method B: Play with Pre-trained Models from This Repo

```bash
cd $UNITREE_LAB

# Create log directory structure (if needed)
mkdir -p logs/rsl_rl/unitree_go2_velocity_mlp_custom/pretrained

# Copy model from this repo
cp $VISTEC_REPO/trained_models/mlp_with_dr_24999.pt \
   logs/rsl_rl/unitree_go2_velocity_mlp_custom/pretrained/

# Play with explicit checkpoint
python scripts/rsl_rl/play.py \
    --task Unitree-Go2-Velocity-MLP-Custom \
    --num_envs 32 \
    --load_run pretrained \
    --checkpoint logs/rsl_rl/unitree_go2_velocity_mlp_custom/pretrained/mlp_with_dr_24999.pt
```

### Send Velocity Commands (Isaac Lab)

Isaac Lab policies run in closed loop - velocities are sampled from command ranges.

**To test specific velocities matching the 4 training tasks**, modify the config file:

```python
# Edit: unitree_rl_lab/Configs/velocity_env_cfg_mlp_custom.py

# Set fixed velocity command (line ~388):
self.commands.base_velocity.ranges = mdp.UniformLevelVelocityCommandCfg.Ranges(
    lin_vel_x=(0.5, 0.5),   # Forward 0.5 m/s (fixed)
    lin_vel_y=(0.0, 0.0),   # No lateral (fixed)
    ang_vel_z=(0.0, 0.0),   # No rotation (fixed)
    heading=(-3.14, 3.14),
)
```

Or use the helper script with **4 task presets**:
```bash
# List all tasks
python send_velocity_commands_isaac.py --list

# Task 1: Standing
python send_velocity_commands_isaac.py --task 1

# Task 2: Walking (2.1=slow, 2.2=normal, 2.3=fast, 2.4=moderate)
python send_velocity_commands_isaac.py --task 2.2

# Task 3: Turn in Place (3.1-3.4: different speeds)
python send_velocity_commands_isaac.py --task 3.2

# Task 4: Walk + Turn (4.1-4.4: combined maneuvers)
python send_velocity_commands_isaac.py --task 4.1

# Custom command
python send_velocity_commands_isaac.py --linear_x 0.5 --angular_z 0.3
```

### Export Policy for Gazebo

The play script automatically exports to ONNX and JIT:

```bash
# After running play.py, check exported models:
ls logs/rsl_rl/unitree_go2_velocity_mlp_custom/2026-02-03_15-54-07_work_good/exported/
# Output:
#   policy.pt      # TorchScript (JIT)
#   policy.onnx    # ONNX format
```

---

## 🎮 Deployment Option 2: Gazebo (Realistic Simulation)

### Prerequisites

```bash
# ROS 2 Humble
source /opt/ros/humble/setup.bash

# Build workspace (first time only)
mkdir -p $VISTEC_WS/src
cd $VISTEC_WS

# Copy packages from this repo
cp -r $VISTEC_REPO/Vistec_ex_ws/src/* src/

# Build
colcon build --symlink-install
source install/setup.bash
```

### Launch Gazebo with Policy

```bash
cd $VISTEC_WS
source install/setup.bash

# Option 1: Use pre-trained model from this repo (RECOMMENDED)
ros2 launch deploy_policy go2_rl_policy.launch.py \
    policy_path:=$VISTEC_REPO/trained_models/mlp_with_dr_24999.pt \
    actuator_type:=mlp \
    use_gpu:=true

# Option 2: Use your trained policy (if you trained yourself)
ros2 launch deploy_policy go2_rl_policy.launch.py \
    policy_path:=$UNITREE_LAB/logs/rsl_rl/unitree_go2_velocity_mlp_custom/{timestamp}/exported/policy.onnx \
    actuator_type:=mlp \
    use_gpu:=true

# Option 3: Use Implicit actuator
ros2 launch deploy_policy go2_rl_policy.launch.py \
    policy_path:=$VISTEC_REPO/trained_models/implicit_dr_latest.pt \
    actuator_type:=implicit \
    use_gpu:=true
```

**Launch Arguments**:
- `policy_path`: Path to ONNX or PT model
- `actuator_type`: `mlp`, `lstm`, or `implicit`
- `use_gpu`: `true` (CUDA) or `false` (CPU)

### Send Velocity Commands (Gazebo)

#### Interactive Menu Script - Matching 4 Training Tasks

```bash
# Make executable
chmod +x send_velocity_commands_gazebo.sh

# Run interactive menu
./send_velocity_commands_gazebo.sh

# Select from 4 training tasks:
#
# TASK 1: Standing
#   1) Stand still (0.0, 0.0, 0.0)
#
# TASK 2: Walking
#   2) Walk slow (0.5 m/s)
#   3) Walk normal (1.0 m/s)
#   4) Walk fast (1.5 m/s)
#   5) Walk moderate (0.8 m/s)
#
# TASK 3: Turn in Place
#   6) Turn slow CCW (+0.5 rad/s)
#   7) Turn normal CCW (+1.0 rad/s)
#   8) Turn normal CW (-1.0 rad/s)
#   9) Turn fast CCW (+1.5 rad/s)
#
# TASK 4: Walk + Turn
#   10) Right arc (0.8 m/s, +0.6 rad/s)
#   11) Straight fast (1.2 m/s)
#   12) Left arc (0.8 m/s, -0.6 rad/s)
#   13) Tight turn (0.5 m/s, +1.0 rad/s)
#
# OTHER
#   14) Custom command
```

#### Manual Command

```bash
# Forward walk
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "linear: {x: 0.5, y: 0.0, z: 0.0}
     angular: {x: 0.0, y: 0.0, z: 0.0}" --rate 10

# Lateral walk
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "linear: {x: 0.0, y: 0.3, z: 0.0}
     angular: {x: 0.0, y: 0.0, z: 0.0}" --rate 10

# Circle walk
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "linear: {x: 0.5, y: 0.0, z: 0.0}
     angular: {x: 0.0, y: 0.0, z: 0.3}" --rate 10

# Stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "linear: {x: 0.0, y: 0.0, z: 0.0}
     angular: {x: 0.0, y: 0.0, z: 0.0}" --rate 10
```

**Velocity Ranges**:
- `linear.x`: -1.0 to 1.0 m/s (forward/backward)
- `linear.y`: -0.4 to 0.4 m/s (left/right)
- `angular.z`: -1.0 to 1.0 rad/s (yaw rotation)

### Monitor Robot State

```bash
# Terminal 1: Joint states
ros2 topic echo /joint_states --no-arr

# Terminal 2: Policy inference rate
ros2 topic hz /joint_commands
# Expected: ~50 Hz

# Terminal 3: Base pose
ros2 topic echo /base_pose --no-arr

# Terminal 4: Velocity tracking
ros2 topic echo /cmd_vel --no-arr
```

---

## 📊 Quick Test Scenarios - Matching 4 Training Tasks

### Scenario 1: Task 2 - Walking Test

**Isaac Lab**:
```bash
# Modify config to Task 2.2 (Walk normal - 1.0 m/s)
# Edit $UNITREE_LAB/source/.../velocity_env_cfg_mlp_custom.py:
#   lin_vel_x=(1.0, 1.0), lin_vel_y=(0.0, 0.0), ang_vel_z=(0.0, 0.0)

cd $UNITREE_LAB
python scripts/rsl_rl/play.py \
    --task Unitree-Go2-Velocity-MLP-Custom \
    --num_envs 32
```

**Gazebo**:
```bash
# Terminal 1: Launch Gazebo
cd $VISTEC_WS
source install/setup.bash
ros2 launch deploy_policy go2_rl_policy.launch.py \
    policy_path:=$VISTEC_REPO/trained_models/mlp_with_dr_24999.pt \
    actuator_type:=mlp

# Terminal 2: Send Task 2.2 command
cd $VISTEC_REPO
./send_velocity_commands_gazebo.sh
# Choose option 3 (Walk normal - 1.0 m/s)
```

### Scenario 2: Task 3 - Turn in Place Test

**Isaac Lab**:
```bash
# Modify config to Task 3.2 (Turn normal CCW - 1.0 rad/s)
# Edit velocity_env_cfg_mlp_custom.py:
#   lin_vel_x=(0.0, 0.0), lin_vel_y=(0.0, 0.0), ang_vel_z=(1.0, 1.0)

python scripts/rsl_rl/play.py \
    --task Unitree-Go2-Velocity-MLP-Custom \
    --num_envs 32 \
    --load_run 2026-02-03_15-54-07_work_good
```

**Gazebo**:
```bash
# Terminal 1: Launch (as above)

# Terminal 2: Send Task 3.2 command
./send_velocity_commands_gazebo.sh
# Choose option 7 (Turn normal CCW - 1.0 rad/s)
```

### Scenario 3: Task 4 - Walk + Turn Combined

**Isaac Lab**:
```bash
# Modify config to Task 4.1 (Right arc - 0.8 m/s, 0.6 rad/s)
# Edit velocity_env_cfg_mlp_custom.py:
#   lin_vel_x=(0.8, 0.8), lin_vel_y=(0.0, 0.0), ang_vel_z=(0.6, 0.6)

python scripts/rsl_rl/play.py \
    --task Unitree-Go2-Velocity-MLP-Custom \
    --num_envs 32 \
    --load_run 2026-02-03_15-54-07_work_good
```

**Gazebo**:
```bash
# Terminal 1: Launch (as above)

# Terminal 2: Send Task 4.1 command
./send_velocity_commands_gazebo.sh
# Choose option 10 (Right arc - 0.8 m/s, +0.6 rad/s)
```

### Scenario 4: Recovery from Push

**Isaac Lab**: Built-in domain randomization includes pushes

**Gazebo**: Apply external force
```bash
# While robot is walking (Task 2.2), apply force
ros2 service call /apply_force gazebo_msgs/srv/ApplyBodyWrench \
    "body_name: 'go2::base_link'
     wrench: {force: {x: 50.0, y: 0.0, z: 0.0}}"
```

---

## 🔬 Performance Validation

### Expected Behavior

**Isaac Lab** (PhysX):
- ✅ Smooth trot gait
- ✅ Velocity tracking <0.1 m/s RMSE
- ✅ Stable recovery from pushes

**Gazebo** (DART/ODE):
- ✅ Similar gait (may be slightly stiffer)
- ✅ Velocity tracking <0.15 m/s RMSE
- ✅ Good recovery with DR-trained models

### Metrics to Check

1. **Velocity Tracking**:
   ```bash
   # Compare commanded vs actual velocity
   ros2 topic echo /cmd_vel
   ros2 topic echo /odom
   ```

2. **Gait Stability**:
   - Watch for consistent leg motion
   - Check base height stays ~0.3m
   - Verify no falling

3. **Recovery**:
   - Apply external disturbances
   - Check robot returns to stable gait

---

## 🛠️ Troubleshooting

### Issue 1: Environment Variables Not Set

```bash
# Check if variables are set
echo $VISTEC_REPO $UNITREE_LAB $VISTEC_WS

# If empty, set them
export VISTEC_REPO=~/Vistec_Intern_Exam
export UNITREE_LAB=~/unitree_rl_lab
export VISTEC_WS=~/vistec_ex_ws
```

### Issue 2: Policy Not Loading (Isaac Lab)

```bash
# Check logs directory
ls $UNITREE_LAB/logs/rsl_rl/unitree_go2_velocity_mlp_custom/

# Use pre-trained model from this repo
cd $UNITREE_LAB
mkdir -p logs/rsl_rl/unitree_go2_velocity_mlp_custom/pretrained
cp $VISTEC_REPO/trained_models/mlp_with_dr_24999.pt logs/rsl_rl/unitree_go2_velocity_mlp_custom/pretrained/

python scripts/rsl_rl/play.py \
    --task Unitree-Go2-Velocity-MLP-Custom \
    --load_run pretrained \
    --checkpoint logs/rsl_rl/unitree_go2_velocity_mlp_custom/pretrained/mlp_with_dr_24999.pt
```

### Issue 3: ONNX Export Failed

```bash
# Re-export by running play.py
cd $UNITREE_LAB
python scripts/rsl_rl/play.py \
    --task Unitree-Go2-Velocity-MLP-Custom \
    --num_envs 1 \
    --load_run pretrained \
    --checkpoint logs/rsl_rl/unitree_go2_velocity_mlp_custom/pretrained/mlp_with_dr_24999.pt

# Check exported directory
ls logs/rsl_rl/unitree_go2_velocity_mlp_custom/pretrained/exported/
```

### Issue 3: Gazebo Robot Falls

**Cause**: PD gains mismatch or control frequency issue

**Solution**:
1. Verify PD gains in URDF (Kp=25.0, Kd=0.5)
2. Check control frequency is 50 Hz
3. Use MLP-Custom policy (best DR for Gazebo)

```bash
# Check joint controller parameters
ros2 param list /controller_manager
ros2 param get /controller_manager use_sim_time
```

### Issue 4: ROS 2 Topic Not Publishing

```bash
# Check active topics
ros2 topic list

# Check node
ros2 node list | grep policy

# Restart policy node
ros2 lifecycle set /policy_node activate
```

---

## 📚 Model Details

### MLP with DR (mlp_with_dr_24999.pt) ⭐

**Best for**: Gazebo deployment

**Training**:
- 24,999 iterations
- 15 domain randomization strategies
- Kp=25.0, Kd=0.5
- 4096 parallel environments

**Performance**:
- Velocity tracking: <0.1 m/s RMSE
- Success rate: >95%
- Robust to external pushes

### Implicit with DR (implicit_dr_latest.pt)

**Best for**: Baseline comparison

**Training**:
- ~18,400 iterations
- Physics-based actuator (no neural network)
- Comprehensive DR

**Performance**:
- Good in Isaac Lab
- May have larger sim-to-sim gap

### LSTM with DR (lstm_dr_25000.pt)

**Best for**: Research (has observation issues)

**Training**:
- 25,000 iterations
- Recurrent network for temporal dynamics

**Performance**:
- Highest actuator accuracy
- Observation tuple issue in play script (partially fixed)

---

## 🔗 Related Documentation

- **Main README**: [README.md](README.md) - Repository overview
- **4 Tasks Reference**: [4_TASKS_SUMMARY.md](4_TASKS_SUMMARY.md) - Velocity command guide
- **Configuration Files**: [unitree_rl_lab/Configs/](unitree_rl_lab/Configs/) - All task configs
- **Full Training Repo**: `$UNITREE_LAB` - Complete source code (clone separately)
- **Full Gazebo Workspace**: `$VISTEC_WS` - ROS 2 workspace (build from this repo)

---

## 📧 Support

**For deployment issues**:
1. Check model file exists in `trained_models/`
2. Verify Isaac Lab / ROS 2 environment is sourced
3. Review velocity command ranges
4. Check ROS 2 topics are publishing

---

**Deployment Guide**
**Last Updated**: 2026-02-11
**Isaac Lab**: 2.3.0 | **ROS 2**: Humble | **Tested**: Ubuntu 22.04, RTX 3090

# Complete User Guide: Clone to Running Policies

## ✅ What You Get After Cloning

This repository is **100% self-contained**. After cloning, you will have:

- ✅ Complete Isaac Lab framework (training & testing)
- ✅ 4 pre-trained policies (ready to use)
- ✅ All robot models and actuator networks
- ✅ ROS 2 + Gazebo deployment workspace
- ✅ All documentation

**No additional repositories to clone!**

---

## 🚀 Step-by-Step Setup (30 minutes)

### Step 1: Clone Repository (2 minutes)

```bash
# Clone the repository
git clone https://github.com/BhumipatNgamphueak/Vistec_Intern_Exam.git
cd Vistec_Intern_Exam
```

**Result**: You now have everything (~141 MB)

---

### Step 2: Set Environment Variables (1 minute)

```bash
# Set up environment variables (all paths inside cloned repo)
export VISTEC_REPO=~/Vistec_Intern_Exam
export UNITREE_LAB=~/Vistec_Intern_Exam/unitree_rl_lab
export ACTUATOR_NET=~/Vistec_Intern_Exam/Actuator_net
export VISTEC_WS=~/Vistec_Intern_Exam/Vistec_ex_ws

# Make permanent (add to ~/.bashrc)
echo "export VISTEC_REPO=~/Vistec_Intern_Exam" >> ~/.bashrc
echo "export UNITREE_LAB=~/Vistec_Intern_Exam/unitree_rl_lab" >> ~/.bashrc
echo "export ACTUATOR_NET=~/Vistec_Intern_Exam/Actuator_net" >> ~/.bashrc
echo "export VISTEC_WS=~/Vistec_Intern_Exam/Vistec_ex_ws" >> ~/.bashrc
source ~/.bashrc
```

**Result**: Environment variables point to your cloned repo

---

### Step 3: Install Isaac Lab (15 minutes)

**Prerequisite**: NVIDIA GPU with CUDA support

```bash
# Install Isaac Lab 2.3.0
cd ~
git clone https://github.com/isaac-sim/IsaacLab.git
cd ~/IsaacLab
./isaaclab.sh --install  # Takes ~10-15 minutes
```

**Optional**: Set ISAACLAB_PATH (recommended for USD models)
```bash
export ISAACLAB_PATH=~/IsaacLab
echo "export ISAACLAB_PATH=~/IsaacLab" >> ~/.bashrc
```

**Result**: Isaac Lab framework installed

---

### Step 4: Install unitree_rl_lab Extension (5 minutes)

```bash
# Navigate to unitree_rl_lab in the cloned repo
cd $UNITREE_LAB

# Install as Python package into IsaacLab
~/IsaacLab/isaaclab.sh -p -m pip install -e source/unitree_rl_lab
```

**Result**: unitree_rl_lab extension registered with Isaac Lab

---

### Step 5: Verify Setup (2 minutes)

```bash
# Run automated verification
cd $VISTEC_REPO
./verify_setup.sh
```

**Expected Output**:
```
✅ All environment variables set
✅ All directories exist
✅ 4 trained policies found
✅ Isaac Lab extension installed
✅ unitree_rl_lab configs accessible
```

---

## 🎮 Test 1: Play Pre-trained Policy in Isaac Lab

### Quick Test (1 minute)

**Note**: If you use conda, activate your environment first: `conda activate isaaclab`

```bash
cd $UNITREE_LAB

# Test with pre-trained MLP policy (RECOMMENDED)
~/IsaacLab/isaaclab.sh -p scripts/rsl_rl/play.py \
  --task Unitree-Go2-Velocity-MLP-Custom \
  --checkpoint $VISTEC_REPO/trained_models/mlp_with_dr_24999.pt \
  --num_envs 1
```

**Expected Result**:
- Isaac Sim GUI opens (may take 1-2 minutes first time)
- Go2 robot appears in rough terrain
- Robot walks forward following policy
- No errors about missing files or paths

### Test All 4 Pre-trained Policies

```bash
cd $UNITREE_LAB

# 1. MLP with DR (RECOMMENDED - Best performance)
~/IsaacLab/isaaclab.sh -p scripts/rsl_rl/play.py \
  --task Unitree-Go2-Velocity-MLP-Custom \
  --checkpoint $VISTEC_REPO/trained_models/mlp_with_dr_24999.pt \
  --num_envs 4

# 2. LSTM with DR
~/IsaacLab/isaaclab.sh -p scripts/rsl_rl/play.py \
  --task Unitree-Go2-Velocity-LSTM-DR \
  --checkpoint $VISTEC_REPO/trained_models/lstm_dr_25000.pt \
  --num_envs 4

# 3. Implicit with DR
~/IsaacLab/isaaclab.sh -p scripts/rsl_rl/play.py \
  --task Unitree-Go2-Velocity-Implicit-DR \
  --checkpoint $VISTEC_REPO/trained_models/implicit_dr_latest.pt \
  --num_envs 4

# 4. Implicit without DR
~/IsaacLab/isaaclab.sh -p scripts/rsl_rl/play.py \
  --task Unitree-Go2-Velocity-Implicit \
  --checkpoint $VISTEC_REPO/trained_models/implicit_no_dr_latest.pt \
  --num_envs 4
```

**Expected Result**: All policies run without errors

---

## 🤖 Test 2: Deploy Policy to Gazebo

### Prerequisites

```bash
# Install ROS 2 Humble (if not installed)
sudo apt update
sudo apt install ros-humble-desktop ros-humble-ros-gz -y

# Source ROS 2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Build ROS 2 Workspace (5 minutes)

```bash
cd $VISTEC_WS
colcon build
source install/setup.bash
```

**Expected Output**: Build succeeds with no errors

### Launch Gazebo Simulation (2 minutes)

```bash
# Terminal 1: Launch Gazebo with Go2 robot
cd $VISTEC_WS
source install/setup.bash
ros2 launch go2_gazebo_simulation go2_fortress.launch.py
```

**Expected Result**:
- Gazebo Fortress opens
- Go2 robot spawns in empty world
- No mesh loading errors
- No path errors

### Deploy Pre-trained Policy (2 minutes)

```bash
# Terminal 2: Run policy inference node
cd $VISTEC_WS
source install/setup.bash

# Deploy MLP policy (uses default from repo)
ros2 launch deploy_policy go2_deploy.launch.py \
  policy_path:=$VISTEC_REPO/trained_models/mlp_with_dr_24999.pt \
  device:=cpu
```

**Expected Result**:
- Policy loads successfully
- Robot receives velocity commands
- Robot moves in Gazebo
- No file not found errors

### Send Velocity Commands (1 minute)

```bash
# Terminal 3: Send walking command
cd $VISTEC_REPO
./send_velocity_commands_gazebo.sh

# Select option 3: Walk normal (1.0 m/s)
```

**Expected Result**: Robot walks forward at 1.0 m/s in Gazebo

---

## 📊 Verification Checklist

After completing all steps, verify:

### ✅ Isaac Lab Works
- [ ] `play.py` script runs without errors
- [ ] Pre-trained policies load successfully
- [ ] Robots appear and move in Isaac Sim
- [ ] No "file not found" or "module not found" errors

### ✅ Gazebo Works
- [ ] Gazebo launches with Go2 robot
- [ ] Meshes load correctly (robot looks complete)
- [ ] Policy deployment node starts
- [ ] Velocity commands move the robot
- [ ] No hardcoded path errors

### ✅ Files are Portable
- [ ] All paths use `$VISTEC_REPO` or `$UNITREE_LAB`
- [ ] Works even if cloned to different directory
- [ ] No references to `/home/drl-68/`

---

## 🎓 Training New Policies

After testing pre-trained policies, you can train your own:

```bash
cd $UNITREE_LAB

# Train MLP policy with domain randomization (RECOMMENDED)
~/IsaacLab/isaaclab.sh -p scripts/rsl_rl/train.py \
  --task Unitree-Go2-Velocity-MLP-Custom \
  --num_envs 4096 \
  --headless

# Train for 25000 iterations (~2-3 hours on RTX 3090)
```

**Output**: Trained model saved to `$UNITREE_LAB/logs/rsl_rl/...`

**Test your trained model**:
```bash
~/IsaacLab/isaaclab.sh -p scripts/rsl_rl/play.py \
  --task Unitree-Go2-Velocity-MLP-Custom \
  --checkpoint $UNITREE_LAB/logs/rsl_rl/unitree_go2_velocity_mlp_custom/latest/model.pt
```

---

## 🧪 Test 4 Locomotion Tasks

The repository includes 4 fundamental locomotion tasks:

### Task 1: Standing (1 variant)
```bash
cd $VISTEC_REPO
python send_velocity_commands_isaac.py --task 1
# Or in Gazebo:
./send_velocity_commands_gazebo.sh  # Select option 1
```

### Task 2: Walking (4 speeds)
```bash
# 0.5 m/s
python send_velocity_commands_isaac.py --task 2.1

# 1.0 m/s (normal)
python send_velocity_commands_isaac.py --task 2.2

# 1.5 m/s (fast)
python send_velocity_commands_isaac.py --task 2.3

# 0.8 m/s (moderate)
python send_velocity_commands_isaac.py --task 2.4
```

### Task 3: Turn in Place (4 rates)
```bash
# Turn right +0.5 rad/s
python send_velocity_commands_isaac.py --task 3.1

# Turn left -0.5 rad/s
python send_velocity_commands_isaac.py --task 3.2

# Fast turn right +1.0 rad/s
python send_velocity_commands_isaac.py --task 3.3

# Fast turn left -1.5 rad/s
python send_velocity_commands_isaac.py --task 3.4
```

### Task 4: Walk + Turn (4 combinations)
```bash
# Walk forward + turn right
python send_velocity_commands_isaac.py --task 4.1

# Walk forward + turn left
python send_velocity_commands_isaac.py --task 4.2

# Walk diagonal + turn
python send_velocity_commands_isaac.py --task 4.3

# Walk backward + turn
python send_velocity_commands_isaac.py --task 4.4
```

---

## 🐛 Troubleshooting

### Issue 1: "Module 'unitree_rl_lab' not found"

**Solution**:
```bash
cd $UNITREE_LAB
~/IsaacLab/isaaclab.sh -p -m pip install -e source/unitree_rl_lab
```

### Issue 2: "USD file not found" or robot doesn't appear

**Solution**: Set ISAACLAB_PATH
```bash
export ISAACLAB_PATH=~/IsaacLab
echo "export ISAACLAB_PATH=~/IsaacLab" >> ~/.bashrc
```

### Issue 3: Gazebo meshes not loading

**Solution**: Verify unitree_ros exists
```bash
ls $UNITREE_LAB/unitree_ros/robots/go2_description/meshes/
# Should show: base.dae, calf.dae, foot.dae, hip.dae, thigh.dae
```

If missing, the repository is incomplete. Re-clone.

### Issue 4: Policy file not found

**Solution**: Verify trained models exist
```bash
ls -lh $VISTEC_REPO/trained_models/
# Should show 4 .pt files (~22 MB total)
```

### Issue 5: "command not found: ./isaaclab.sh"

**Solution**: Isaac Lab not installed properly
```bash
cd ~/IsaacLab
./isaaclab.sh --install
```

### Issue 6: ROS 2 workspace build fails

**Solution**: Install dependencies
```bash
sudo apt update
sudo apt install ros-humble-desktop ros-humble-ros-gz python3-colcon-common-extensions -y
```

---

## 📝 Summary: What Makes This Work?

### ✅ Self-Contained Structure
```
Vistec_Intern_Exam/
├── unitree_rl_lab/          # Complete framework (not just configs)
│   ├── source/              # Python package
│   ├── scripts/             # train.py, play.py
│   ├── unitree_ros/         # Robot descriptions ✅ INCLUDED
│   └── ...
├── trained_models/          # 4 policies ✅ INCLUDED
├── Actuator_net/            # Actuator models ✅ INCLUDED
└── Vistec_ex_ws/            # ROS 2 workspace ✅ INCLUDED
```

### ✅ Fixed Hardcoded Paths

All files now use environment variables:
- ✅ `$VISTEC_REPO` → Repository root
- ✅ `$UNITREE_LAB` → Framework directory
- ✅ `$ACTUATOR_NET` → Actuator models
- ✅ `$ISAACLAB_PATH` → Isaac Lab installation

### ✅ Relative Paths in Configs

All actuator model paths use relative paths:
```python
# OLD (hardcoded)
network_file = "/home/drl-68/actuator_net/app/resources/actuator_lstm.pth"

# NEW (relative) ✅
network_file = os.path.join(os.path.dirname(__file__), "..", "..", "..", "..",
                            "assets", "actuator_models", "actuator_lstm.pth")
```

### ✅ Default to Repo Files

All launch files default to pre-trained models in repo:
```python
# OLD (hardcoded)
default_value='/home/drl-68/unitree_rl_lab/logs/.../model.pt'

# NEW (uses repo) ✅
default_value=os.path.join(os.getenv('VISTEC_REPO', '~/Vistec_Intern_Exam'),
                           'trained_models', 'mlp_with_dr_24999.pt')
```

---

## 🎯 Success Criteria

You have successfully set up the repository when:

1. ✅ You can run `play.py` with pre-trained policies in Isaac Lab
2. ✅ Gazebo launches with complete Go2 robot (no missing meshes)
3. ✅ Policy deployment node runs without path errors
4. ✅ Velocity commands control the robot in Gazebo
5. ✅ All 4 tasks (standing, walking, turning, combined) work
6. ✅ Training new policies works and saves correctly

**Time to Complete**: ~30-40 minutes (most time is Isaac Lab installation)

---

## 📚 Additional Resources

- [README.md](README.md) - Main repository documentation
- [SELF_CONTAINED_REPOSITORY.md](SELF_CONTAINED_REPOSITORY.md) - Migration guide
- [unitree_rl_lab/README.md](unitree_rl_lab/README.md) - Framework documentation
- Isaac Lab docs: https://isaac-sim.github.io/IsaacLab
- ROS 2 Humble docs: https://docs.ros.org/en/humble/

---

## ❓ Still Having Issues?

If you followed all steps and still have problems:

1. **Run verification script**:
   ```bash
   cd $VISTEC_REPO
   ./verify_setup.sh
   ```

2. **Check environment variables**:
   ```bash
   echo $VISTEC_REPO
   echo $UNITREE_LAB
   echo $ISAACLAB_PATH
   ```

3. **Verify Isaac Lab installation**:
   ```bash
   ~/IsaacLab/isaaclab.sh -p -c "import isaaclab; print(isaaclab.__version__)"
   ```

4. **Check for missing files**:
   ```bash
   ls -lh $VISTEC_REPO/trained_models/*.pt
   ls -lh $UNITREE_LAB/source/
   ls -lh $UNITREE_LAB/scripts/rsl_rl/
   ```

---

**Repository**: https://github.com/BhumipatNgamphueak/Vistec_Intern_Exam.git
**Last Updated**: February 11, 2026
**Version**: 2.0 (Self-Contained)

# Setup Guide: Getting Started from GitHub

This guide helps you set up the Vistec Intern Exam repository after cloning from GitHub.

---

## 📥 Step 1: Clone This Repository

```bash
# Clone the repository (replace with actual URL)
git clone https://github.com/your-username/Vistec_Intern_Exam.git
cd Vistec_Intern_Exam
```

---

## ⚙️ Step 2: Set Environment Variables

This repository uses environment variables to make paths flexible for different users.

### Quick Setup

```bash
# Set workspace paths (adjust to match your directory structure)
export VISTEC_REPO=~/Vistec_Intern_Exam              # This cloned repo
export UNITREE_LAB=~/unitree_rl_lab                   # Full training repo
export ACTUATOR_NET=~/actuator_net                    # Actuator models repo
export VISTEC_WS=~/vistec_ex_ws                       # ROS 2 workspace

# Make permanent by adding to ~/.bashrc
echo "# Vistec Intern Exam Environment" >> ~/.bashrc
echo "export VISTEC_REPO=~/Vistec_Intern_Exam" >> ~/.bashrc
echo "export UNITREE_LAB=~/unitree_rl_lab" >> ~/.bashrc
echo "export ACTUATOR_NET=~/actuator_net" >> ~/.bashrc
echo "export VISTEC_WS=~/vistec_ex_ws" >> ~/.bashrc

# Reload bash configuration
source ~/.bashrc
```

### Verify Setup

```bash
# Check if variables are set correctly
echo "VISTEC_REPO: $VISTEC_REPO"
echo "UNITREE_LAB: $UNITREE_LAB"
echo "ACTUATOR_NET: $ACTUATOR_NET"
echo "VISTEC_WS: $VISTEC_WS"
```

**Expected Output**:
```
VISTEC_REPO: /home/your-username/Vistec_Intern_Exam
UNITREE_LAB: /home/your-username/unitree_rl_lab
ACTUATOR_NET: /home/your-username/actuator_net
VISTEC_WS: /home/your-username/vistec_ex_ws
```

---

## 🔧 Step 3: Install Prerequisites

### 3.1 Isaac Lab (for Training & Testing)

```bash
# Clone Isaac Lab
cd ~
git clone https://github.com/isaac-sim/IsaacLab.git
cd IsaacLab

# Run installation
./isaaclab.sh --install
```

**Requirements**:
- Ubuntu 22.04
- Python 3.10
- CUDA 11.8+ or 12.1+
- RTX GPU (recommended RTX 3090 or better)

### 3.2 ROS 2 Humble (for Gazebo Deployment)

```bash
# Install ROS 2 Humble
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update
sudo apt install ros-humble-desktop

# Add to bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3.3 Additional Tools

```bash
# ROS 2 tools
sudo apt install python3-colcon-common-extensions
sudo apt install python3-rosdep

# Initialize rosdep (first time only)
sudo rosdep init
rosdep update
```

---

## 📚 Step 4: Clone Full Repositories

This repository contains **essential files only**. You need to clone full repositories for training.

### Option A: Clone Official Repositories (Recommended)

```bash
# 1. Clone unitree_rl_lab (replace with actual URL)
cd ~
git clone <official-unitree-rl-lab-repo-url> unitree_rl_lab

# 2. Clone actuator_net (replace with actual URL)
git clone <official-actuator-net-repo-url> actuator_net

# Verify paths match environment variables
ls -d $UNITREE_LAB $ACTUATOR_NET
```

### Option B: Use Existing Repositories

If the repositories already exist on your system:

```bash
# Update environment variables to point to existing locations
export UNITREE_LAB=/path/to/existing/unitree_rl_lab
export ACTUATOR_NET=/path/to/existing/actuator_net
export VISTEC_WS=/path/to/existing/vistec_ex_ws

# Update ~/.bashrc accordingly
```

---

## 🚀 Step 5: Setup Training Environment

### 5.1 Install unitree_rl_lab

```bash
cd $UNITREE_LAB
~/IsaacLab/isaaclab.sh -p -m pip install -e source/unitree_rl_lab
```

### 5.2 Copy Configuration Files

```bash
# Copy all configs from this repo to training repo
cp $VISTEC_REPO/unitree_rl_lab/Configs/*.py \
   $UNITREE_LAB/source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/go2/

# Copy actuator models
mkdir -p $UNITREE_LAB/source/unitree_rl_lab/unitree_rl_lab/assets/actuator_models/
cp $VISTEC_REPO/Actuator_net/app/resources/*.pth \
   $UNITREE_LAB/source/unitree_rl_lab/unitree_rl_lab/assets/actuator_models/
```

### 5.3 Copy Utility Scripts

```bash
# Copy testing scripts
cp $VISTEC_REPO/unitree_rl_lab/Testing_Scripts/*.py $UNITREE_LAB/
cp $VISTEC_REPO/unitree_rl_lab/Utils/*.py $UNITREE_LAB/
```

---

## 🎮 Step 6: Setup Gazebo Deployment

### 6.1 Create ROS 2 Workspace

```bash
# Create workspace structure
mkdir -p $VISTEC_WS/src
cd $VISTEC_WS

# Copy ROS 2 packages from this repo
cp -r $VISTEC_REPO/Vistec_ex_ws/src/* src/

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
colcon build --symlink-install

# Source workspace
source install/setup.bash

# Add to bashrc for future sessions
echo "source $VISTEC_WS/install/setup.bash" >> ~/.bashrc
```

---

## ✅ Step 7: Verify Installation

### 7.1 Test Isaac Lab

```bash
cd $UNITREE_LAB

# Test with pre-trained model from this repo
mkdir -p logs/rsl_rl/unitree_go2_velocity_mlp_custom/pretrained
cp $VISTEC_REPO/trained_models/mlp_with_dr_24999.pt \
   logs/rsl_rl/unitree_go2_velocity_mlp_custom/pretrained/

# Run play script (should launch Isaac Sim)
python scripts/rsl_rl/play.py \
    --task Unitree-Go2-Velocity-MLP-Custom \
    --num_envs 1 \
    --load_run pretrained \
    --checkpoint logs/rsl_rl/unitree_go2_velocity_mlp_custom/pretrained/mlp_with_dr_24999.pt
```

**Expected**: Isaac Sim opens with GO2 robot executing velocity commands

### 7.2 Test Gazebo Deployment

```bash
# Terminal 1: Launch Gazebo
cd $VISTEC_WS
source install/setup.bash
ros2 launch deploy_policy go2_rl_policy.launch.py \
    policy_path:=$VISTEC_REPO/trained_models/mlp_with_dr_24999.pt \
    actuator_type:=mlp

# Terminal 2: Send velocity command
cd $VISTEC_REPO
./send_velocity_commands_gazebo.sh
# Choose option 3 (Walk normal - 1.0 m/s)
```

**Expected**: Gazebo opens with GO2 robot walking forward at 1.0 m/s

---

## 🐛 Troubleshooting

### Issue 1: Environment Variables Not Working

```bash
# Check current values
env | grep VISTEC
env | grep UNITREE

# If empty, source bashrc again
source ~/.bashrc

# Or set manually for current session
export VISTEC_REPO=~/Vistec_Intern_Exam
export UNITREE_LAB=~/unitree_rl_lab
export VISTEC_WS=~/vistec_ex_ws
```

### Issue 2: Isaac Lab Not Found

```bash
# Check Isaac Lab installation
ls ~/IsaacLab/isaaclab.sh

# If missing, reinstall
cd ~/IsaacLab
./isaaclab.sh --install
```

### Issue 3: ROS 2 Package Build Fails

```bash
# Clean and rebuild
cd $VISTEC_WS
rm -rf build install log
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

### Issue 4: CUDA Not Found

```bash
# Check CUDA installation
nvidia-smi
nvcc --version

# If missing, install CUDA 11.8 or 12.1
# Follow: https://developer.nvidia.com/cuda-downloads
```

### Issue 5: Python Version Mismatch

```bash
# Check Python version (must be 3.10)
python3 --version

# If wrong version, use Isaac Lab's Python
~/IsaacLab/isaaclab.sh -p python3 --version
```

---

## 📋 Directory Structure After Setup

```
~/
├── Vistec_Intern_Exam/          # This cloned repo ($VISTEC_REPO)
│   ├── trained_models/          # Pre-trained policies
│   ├── unitree_rl_lab/          # Config files
│   ├── Actuator_net/            # Pre-trained actuator models
│   └── Vistec_ex_ws/            # ROS 2 package templates
│
├── unitree_rl_lab/              # Full training repo ($UNITREE_LAB)
│   ├── source/                  # Isaac Lab extension
│   ├── scripts/                 # Training/testing scripts
│   └── logs/                    # Training logs
│
├── actuator_net/                # Full actuator repo ($ACTUATOR_NET)
│   ├── train.py                 # Training scripts
│   └── app/resources/           # Pre-trained models
│
├── vistec_ex_ws/                # ROS 2 workspace ($VISTEC_WS)
│   ├── src/                     # ROS 2 packages
│   ├── build/                   # Build artifacts
│   └── install/                 # Installed packages
│
└── IsaacLab/                    # Isaac Lab framework
    ├── source/                  # Core framework
    └── isaaclab.sh              # Setup script
```

---

## 🎯 Quick Start Summary

For users who want to skip details:

```bash
# 1. Clone repo
git clone <repo-url> ~/Vistec_Intern_Exam
cd ~/Vistec_Intern_Exam

# 2. Set environment variables
export VISTEC_REPO=~/Vistec_Intern_Exam
export UNITREE_LAB=~/unitree_rl_lab
export VISTEC_WS=~/vistec_ex_ws
echo "export VISTEC_REPO=~/Vistec_Intern_Exam" >> ~/.bashrc
echo "export UNITREE_LAB=~/unitree_rl_lab" >> ~/.bashrc
echo "export VISTEC_WS=~/vistec_ex_ws" >> ~/.bashrc

# 3. Install Isaac Lab
cd ~ && git clone https://github.com/isaac-sim/IsaacLab.git
cd IsaacLab && ./isaaclab.sh --install

# 4. Clone full repos
cd ~ && git clone <unitree-rl-lab-url> unitree_rl_lab
cd ~ && git clone <actuator-net-url> actuator_net

# 5. Setup training environment
cd $UNITREE_LAB
~/IsaacLab/isaaclab.sh -p -m pip install -e source/unitree_rl_lab
cp $VISTEC_REPO/unitree_rl_lab/Configs/*.py source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/go2/

# 6. Setup Gazebo
mkdir -p $VISTEC_WS/src && cd $VISTEC_WS
cp -r $VISTEC_REPO/Vistec_ex_ws/src/* src/
colcon build --symlink-install

# 7. Test with pre-trained model
cd $VISTEC_REPO
./send_velocity_commands_gazebo.sh  # See deployment options
```

---

## 📚 Next Steps

After completing setup:

1. **Train your own policy**: See [README.md](README.md) - STEP 1
2. **Deploy pre-trained models**: See [DEPLOYMENT_GUIDE.md](DEPLOYMENT_GUIDE.md)
3. **Test 4 locomotion tasks**: See [4_TASKS_SUMMARY.md](4_TASKS_SUMMARY.md)
4. **Understand configurations**: Browse `unitree_rl_lab/Configs/`

---

## 📧 Support

If you encounter issues:

1. Check environment variables are set: `echo $VISTEC_REPO`
2. Verify directory structure matches [Directory Structure](#directory-structure-after-setup)
3. Review [Troubleshooting](#troubleshooting) section
4. Check system requirements (Ubuntu 22.04, CUDA 11.8+, RTX GPU)

---

**Setup Guide**
**Last Updated**: 2026-02-11
**Tested On**: Ubuntu 22.04, Isaac Lab 2.3.0, ROS 2 Humble

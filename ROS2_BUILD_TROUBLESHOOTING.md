# ROS 2 Build Troubleshooting Guide

## ✅ Correct Build Process

### Step 1: Install ROS 2 Humble

```bash
# Update package lists
sudo apt update

# Install ROS 2 Humble Desktop (full installation)
sudo apt install ros-humble-desktop ros-humble-ros-gz -y

# Install build tools
sudo apt install python3-colcon-common-extensions python3-rosdep -y

# Initialize rosdep (first time only)
sudo rosdep init
rosdep update
```

### Step 2: Source ROS 2 Environment

```bash
# Source ROS 2 setup (REQUIRED before building!)
source /opt/ros/humble/setup.bash

# Make it automatic (add to ~/.bashrc)
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 3: Build the Workspace

```bash
# Navigate to workspace
cd $VISTEC_WS

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build all packages
colcon build

# Source the workspace
source install/setup.bash
```

**Expected Output**:
```
Starting >>> deploy_policy
Starting >>> go2_gazebo_simulation
Finished <<< go2_gazebo_simulation [0.5s]
Finished <<< deploy_policy [0.8s]

Summary: 2 packages finished [1.0s]
```

---

## 🐛 Common Build Errors & Solutions

### Error 1: "Command 'colcon' not found"

**Cause**: colcon not installed

**Solution**:
```bash
sudo apt update
sudo apt install python3-colcon-common-extensions -y
```

---

### Error 2: "ROS_DISTRO not set" or "setup.bash: No such file"

**Cause**: ROS 2 not sourced

**Solution**:
```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Verify it worked
echo $ROS_DISTRO
# Should output: humble
```

---

### Error 3: "Package 'geometry_msgs' not found"

**Cause**: Missing ROS 2 dependencies

**Solution**:
```bash
cd $VISTEC_WS

# Install all dependencies
rosdep install --from-paths src --ignore-src -r -y

# Then rebuild
colcon build
```

---

### Error 4: "CMake Error" or "Could not find a package configuration file"

**Cause**: Missing ROS 2 packages

**Solution**:
```bash
# Install missing ROS 2 packages
sudo apt install ros-humble-desktop-full ros-humble-ros-gz -y

# Install additional dependencies
sudo apt install ros-humble-tf2-ros ros-humble-tf2-geometry-msgs -y
sudo apt install ros-humble-xacro ros-humble-joint-state-publisher -y
```

---

### Error 5: "Permission denied" errors

**Cause**: Wrong file permissions

**Solution**:
```bash
cd $VISTEC_WS

# Fix permissions
chmod -R u+rwX src/
chmod +x src/deploy_policy/scripts/*.py

# Rebuild
colcon build
```

---

### Error 6: "Python module not found" (torch, numpy, etc.)

**Cause**: Python packages not installed in system Python

**Solution**:
```bash
# Install Python dependencies for ROS 2 workspace
pip3 install torch numpy scipy

# Or use rosdep
cd $VISTEC_WS
rosdep install --from-paths src --ignore-src -r -y
```

---

### Error 7: Build succeeds but "ros2 launch" can't find packages

**Cause**: Workspace not sourced

**Solution**:
```bash
# Source the workspace after building
cd $VISTEC_WS
source install/setup.bash

# Verify packages are visible
ros2 pkg list | grep deploy_policy
# Should show: deploy_policy

ros2 pkg list | grep go2_gazebo
# Should show: go2_gazebo_simulation
```

---

### Error 8: "fatal error: rclcpp/rclcpp.hpp: No such file or directory"

**Cause**: ROS 2 development files not installed

**Solution**:
```bash
sudo apt install ros-humble-rclcpp ros-humble-rclpy -y
sudo apt install ros-humble-std-msgs ros-humble-sensor-msgs -y
```

---

## 🔍 Diagnostic Commands

### Check ROS 2 Installation

```bash
# Check if ROS 2 is installed
dpkg -l | grep ros-humble-desktop
# Should show installed packages

# Check ROS_DISTRO
echo $ROS_DISTRO
# Should output: humble

# List available ROS 2 packages
apt list --installed | grep ros-humble | wc -l
# Should show 100+ packages
```

### Check Python Dependencies

```bash
# Check Python version
python3 --version
# Should be 3.10 or higher

# Check if torch is installed
python3 -c "import torch; print(torch.__version__)"
# Should show PyTorch version

# Check if numpy is installed
python3 -c "import numpy; print(numpy.__version__)"
# Should show NumPy version
```

### Check Workspace Structure

```bash
cd $VISTEC_WS

# Check workspace structure
tree -L 2 src/
# Should show:
# src/
# ├── deploy_policy
# │   ├── config
# │   ├── deploy_policy
# │   ├── launch
# │   ├── package.xml
# │   └── setup.py
# └── go2_gazebo_simulation
#     ├── launch
#     ├── package.xml
#     ├── setup.py
#     ├── urdf
#     └── worlds

# Check if package.xml files exist
ls src/*/package.xml
# Should show 2 files
```

---

## 🚀 Complete Build Workflow (From Scratch)

### Full Clean Build

```bash
# 1. Install ROS 2 (if not already installed)
sudo apt update
sudo apt install ros-humble-desktop ros-humble-ros-gz -y
sudo apt install python3-colcon-common-extensions python3-rosdep -y

# 2. Source ROS 2
source /opt/ros/humble/setup.bash

# 3. Navigate to workspace
cd $VISTEC_WS

# 4. Clean previous build (if any)
rm -rf build/ install/ log/

# 5. Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# 6. Build
colcon build

# 7. Source workspace
source install/setup.bash

# 8. Verify
ros2 pkg list | grep -E "deploy_policy|go2_gazebo"
```

**Expected**: Both packages should be listed

---

## 🧪 Test After Building

### Test 1: Launch Gazebo

```bash
source /opt/ros/humble/setup.bash
cd $VISTEC_WS
source install/setup.bash

ros2 launch go2_gazebo_simulation go2_fortress.launch.py
```

**Expected**: Gazebo opens with Go2 robot

### Test 2: Check Deploy Policy Node

```bash
source /opt/ros/humble/setup.bash
cd $VISTEC_WS
source install/setup.bash

ros2 run deploy_policy go2_deploy_node --ros-args -p policy_path:=$VISTEC_REPO/trained_models/mlp_with_dr_24999.pt
```

**Expected**: Node starts (may show waiting for robot state)

---

## 📋 Quick Fix Checklist

Before building, ensure:
- [ ] ROS 2 Humble is installed: `dpkg -l | grep ros-humble-desktop`
- [ ] ROS 2 is sourced: `echo $ROS_DISTRO` shows "humble"
- [ ] Colcon is installed: `which colcon` shows a path
- [ ] You're in the workspace: `pwd` shows `.../Vistec_ex_ws`
- [ ] Source code exists: `ls src/` shows 2 directories
- [ ] Environment variables are set: `echo $VISTEC_WS` shows path

---

## 🔧 Build with More Verbose Output

If build fails, get detailed error messages:

```bash
cd $VISTEC_WS

# Build with verbose output
colcon build --event-handlers console_direct+

# Or build one package at a time
colcon build --packages-select deploy_policy
colcon build --packages-select go2_gazebo_simulation
```

---

## 💡 Pro Tips

### 1. Always Source Before Building
```bash
# Create a build script
cat > ~/build_vistec.sh << 'EOF'
#!/bin/bash
source /opt/ros/humble/setup.bash
cd ~/Vistec_Intern_Exam/Vistec_ex_ws
colcon build
source install/setup.bash
echo "✅ Build complete! Workspace sourced."
EOF

chmod +x ~/build_vistec.sh
```

### 2. Add to .bashrc
```bash
# Add these lines to ~/.bashrc for convenience
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/Vistec_Intern_Exam/Vistec_ex_ws/install/setup.bash" >> ~/.bashrc
```

### 3. Check Build Logs
```bash
# If build fails, check logs
cd $VISTEC_WS
cat log/latest_build/deploy_policy/stdout.log
cat log/latest_build/deploy_policy/stderr.log
```

---

## ❓ Still Can't Build?

### Share This Information:

```bash
# System info
cat /etc/os-release | grep VERSION
# ROS 2 version
echo $ROS_DISTRO
# Python version
python3 --version
# Build error
cd $VISTEC_WS
colcon build 2>&1 | tee build_error.log
# Show last 50 lines of error
tail -50 build_error.log
```

Then share the output for specific help!

---

## 📚 Related Documentation

- [ROS 2 Humble Installation](https://docs.ros.org/en/humble/Installation.html)
- [Colcon Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)
- [ROS 2 Workspace Setup](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)

---

**Last Updated**: February 11, 2026

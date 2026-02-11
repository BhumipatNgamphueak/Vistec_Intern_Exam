# Config File Setup Instructions

## For GitHub Users

After cloning this repository, you need to update hardcoded paths in configuration files.

### Quick Fix Using Environment Variables

```bash
# Set your environment variables first (see main SETUP.md)
export VISTEC_REPO=~/Vistec_Intern_Exam
export UNITREE_LAB=~/unitree_rl_lab
export ACTUATOR_NET=~/actuator_net
export VISTEC_WS=~/vistec_ex_ws
```

### Files That May Need Path Updates

1. **go2_deploy.yaml** - Policy and actuator model paths
   - Update `policy_path` to your trained model location
   - Update actuator network paths if using neural net actuators

2. **Launch files** - Robot description paths
   - `go2_fortress.launch.py`
   - `go2_hanging.launch.py`
   - `go2_deploy.launch.py`

### Recommended Usage

Instead of editing these files, use launch arguments:

```bash
# Override policy path at runtime
ros2 launch deploy_policy go2_deploy.launch.py \
    policy_path:=$VISTEC_REPO/trained_models/mlp_dr.pt \
    actuator_type:=mlp
```

This avoids hardcoding paths in config files.

### For Development

If you're doing active development and want to set defaults:

```bash
# Replace hardcoded paths with your paths
cd $VISTEC_WS/src/deploy_policy/config
sed -i "s|/home/drl-68/|$HOME/|g" *.yaml

# For launch files
cd $VISTEC_WS/src/deploy_policy/launch
sed -i "s|/home/drl-68/|$HOME/|g" *.py

cd $VISTEC_WS/src/go2_gazebo_simulation/launch
sed -i "s|/home/drl-68/|$HOME/|g" *.py
```

---

**Note**: It's best to use launch arguments rather than editing config files directly.

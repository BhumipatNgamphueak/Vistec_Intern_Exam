# Configuration Files - Path Setup

## ⚠️ Important for GitHub Users

Some configuration files contain hardcoded paths that need to be updated after cloning.

---

## Files with Hardcoded Actuator Paths

### `velocity_env_cfg_lstm_my_model.py`
- Line 5: `network_file = "/home/drl-68/actuator_net/app/resources/actuator_lstm.pth"`
- Line 52: Same path

### `velocity_env_cfg_lstm_custom_enhanced.py`
- Line 28: `network_file = "/home/drl-68/actuator_net/app/resources/actuator_lstm.pth"`

---

## Quick Fix

### Option 1: Use Environment Variable (Recommended)

```bash
# Set environment variable
export ACTUATOR_NET=~/actuator_net

# Update config files to use environment variable
cd $VISTEC_REPO/unitree_rl_lab/Configs

# For lstm_my_model.py
# Replace hardcoded path with:
# import os
# network_file = os.path.join(os.getenv('ACTUATOR_NET', '~/actuator_net'),
#                             'app/resources/actuator_lstm.pth')
```

### Option 2: Find and Replace

```bash
cd $VISTEC_REPO/unitree_rl_lab/Configs

# Replace /home/drl-68 with your home directory
sed -i "s|/home/drl-68|$HOME|g" velocity_env_cfg_lstm_my_model.py
sed -i "s|/home/drl-68|$HOME|g" velocity_env_cfg_lstm_custom_enhanced.py
```

### Option 3: Manual Edit

Open each file and update the paths:

```python
# OLD:
network_file = "/home/drl-68/actuator_net/app/resources/actuator_lstm.pth"

# NEW (replace with your path):
network_file = "/home/YOUR_USERNAME/actuator_net/app/resources/actuator_lstm.pth"
```

---

## Recommended Approach

Before copying configs to the training repository, update paths:

```bash
# 1. Set environment variables (see SETUP.md)
export VISTEC_REPO=~/Vistec_Intern_Exam
export UNITREE_LAB=~/unitree_rl_lab
export ACTUATOR_NET=~/actuator_net

# 2. Copy actuator models to Isaac Lab assets
mkdir -p $UNITREE_LAB/source/unitree_rl_lab/unitree_rl_lab/assets/actuator_models/
cp $VISTEC_REPO/Actuator_net/app/resources/*.pth \
   $UNITREE_LAB/source/unitree_rl_lab/unitree_rl_lab/assets/actuator_models/

# 3. Copy configs and update paths
cd $VISTEC_REPO/unitree_rl_lab/Configs
for file in *.py; do
    sed "s|/home/drl-68/actuator_net|$ACTUATOR_NET|g" "$file" > /tmp/"$file"
    cp /tmp/"$file" $UNITREE_LAB/source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/go2/
done
```

---

## Which Configs Are Safe?

These configs **DO NOT** have hardcoded paths and can be used as-is:
- ✅ `velocity_env_cfg.py`
- ✅ `velocity_env_cfg_mlp_custom.py`
- ✅ `velocity_env_cfg_mlp_no_dr.py`
- ✅ `velocity_env_cfg_implicit.py`
- ✅ `velocity_env_cfg_implicit_with_dr.py`
- ✅ `velocity_env_cfg_lstm.py`
- ✅ `velocity_env_cfg_lstm_with_dr.py`
- ✅ `velocity_env_cfg_lstm_no_dr.py`
- ✅ `velocity_env_cfg_lstm_custom.py`

These configs **HAVE** hardcoded paths (need update):
- ⚠️ `velocity_env_cfg_lstm_my_model.py`
- ⚠️ `velocity_env_cfg_lstm_custom_enhanced.py`

---

## Verification

After setup, verify the config can load:

```bash
cd $UNITREE_LAB
python -c "from unitree_rl_lab.tasks.locomotion.robots.go2.velocity_env_cfg_lstm_my_model import *; print('Config loaded successfully!')"
```

If you see `Config loaded successfully!`, the paths are correct.

---

**For more setup help, see**: [SETUP.md](../../SETUP.md)

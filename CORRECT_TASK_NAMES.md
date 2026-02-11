# Correct Task Names & Commands

## ✅ Registered Task Names

Based on the `__init__.py` file, here are the **CORRECT** task names:

### MLP Actuator Tasks
1. `Unitree-Go2-Velocity-MLP-Custom` - MLP with Domain Randomization ⭐ **RECOMMENDED**
2. `Unitree-Go2-Velocity-MLP-No-DR` - MLP without Domain Randomization

### LSTM Actuator Tasks
3. `Unitree-Go2-Velocity-LSTM-DR` - LSTM with Domain Randomization ✅ **CORRECT**
4. `Unitree-Go2-Velocity-LSTM-No-DR` - LSTM without Domain Randomization
5. `Unitree-Go2-Velocity-LSTM-Custom` - LSTM custom configuration
6. `Unitree-Go2-Velocity-LSTM-Custom-Enhanced` - LSTM enhanced version
7. `Unitree-Go2-Velocity-LSTM-MyModel` - LSTM with your trained model

### Implicit Actuator Tasks
8. `Unitree-Go2-Velocity-Implicit-DR` - Implicit with Domain Randomization ✅ **CORRECT**
9. `Unitree-Go2-Velocity-Implicit` - Implicit without Domain Randomization

### Base Task
10. `Unitree-Go2-Velocity` - Base configuration
11. `Unitree-Go2-Velocity-LSTM` - Base LSTM configuration

---

## 🔧 Corrected Commands for Pre-trained Policies

### Option A: With Conda Environment (If you use conda)

```bash
# Activate Isaac Lab conda environment (if using conda)
conda activate isaaclab  # or your conda env name

cd $UNITREE_LAB

# 1. MLP with DR (RECOMMENDED)
~/IsaacLab/isaaclab.sh -p scripts/rsl_rl/play.py \
  --task Unitree-Go2-Velocity-MLP-Custom \
  --checkpoint $VISTEC_REPO/trained_models/mlp_with_dr_24999.pt \
  --num_envs 4

# 2. LSTM with DR (CORRECTED TASK NAME!)
~/IsaacLab/isaaclab.sh -p scripts/rsl_rl/play.py \
  --task Unitree-Go2-Velocity-LSTM-DR \
  --checkpoint $VISTEC_REPO/trained_models/lstm_dr_25000.pt \
  --num_envs 4

# 3. Implicit with DR (CORRECTED TASK NAME!)
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

### Option B: Without Conda (Standard Isaac Lab setup)

Isaac Lab's `isaaclab.sh` script manages its own Python environment automatically:

```bash
cd $UNITREE_LAB

# No conda activation needed - isaaclab.sh handles it!

# 1. MLP with DR (RECOMMENDED)
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

---

## ❌ WRONG Task Names (from previous example)

These will cause errors:

```bash
# ❌ WRONG - No such task registered
--task Unitree-Go2-Velocity-Rough-LSTM-With-DR

# ❌ WRONG - No such task registered
--task Unitree-Go2-Velocity-Rough-Implicit-With-DR

# ❌ WRONG - No such task registered
--task Unitree-Go2-Velocity-Rough
```

---

## 🎯 Task Name Mapping

| Pre-trained Model | Correct Task Name | Old (Wrong) Task Name |
|-------------------|-------------------|-----------------------|
| `mlp_with_dr_24999.pt` | `Unitree-Go2-Velocity-MLP-Custom` | ~~Unitree-Go2-Velocity-Rough~~ |
| `lstm_dr_25000.pt` | `Unitree-Go2-Velocity-LSTM-DR` | ~~Unitree-Go2-Velocity-Rough-LSTM-With-DR~~ |
| `implicit_dr_latest.pt` | `Unitree-Go2-Velocity-Implicit-DR` | ~~Unitree-Go2-Velocity-Rough-Implicit-With-DR~~ |
| `implicit_no_dr_latest.pt` | `Unitree-Go2-Velocity-Implicit` | ~~Unitree-Go2-Velocity-Rough-Implicit~~ |

---

## 🐍 Conda vs Non-Conda Setup

### How to Check if You're Using Conda

```bash
# Check if conda is active
echo $CONDA_PREFIX

# If output is empty → You're NOT using conda
# If output shows a path → You ARE using conda
```

### Recommended Setup

**Isaac Lab typically doesn't require conda activation** because:
1. `isaaclab.sh` automatically activates its own virtual environment
2. The script detects conda if available and uses it
3. Most users follow the standard installation which uses Isaac Sim's built-in Python

**Only activate conda if**:
- You explicitly installed Isaac Lab in a conda environment
- Your setup instructions mentioned creating a conda environment
- You see conda environment prompts in your terminal

---

## 📝 Complete Example Workflow

### Standard Setup (No Conda)

```bash
# Set environment variables
export VISTEC_REPO=~/Vistec_Intern_Exam
export UNITREE_LAB=~/Vistec_Intern_Exam/unitree_rl_lab

# Navigate to framework
cd $UNITREE_LAB

# Play pre-trained MLP policy (most recommended)
~/IsaacLab/isaaclab.sh -p scripts/rsl_rl/play.py \
  --task Unitree-Go2-Velocity-MLP-Custom \
  --checkpoint $VISTEC_REPO/trained_models/mlp_with_dr_24999.pt \
  --num_envs 4
```

### With Conda (If You Use Conda)

```bash
# Activate your conda environment first
conda activate isaaclab  # or your env name

# Set environment variables
export VISTEC_REPO=~/Vistec_Intern_Exam
export UNITREE_LAB=~/Vistec_Intern_Exam/unitree_rl_lab

# Navigate to framework
cd $UNITREE_LAB

# Play policy
~/IsaacLab/isaaclab.sh -p scripts/rsl_rl/play.py \
  --task Unitree-Go2-Velocity-MLP-Custom \
  --checkpoint $VISTEC_REPO/trained_models/mlp_with_dr_24999.pt \
  --num_envs 4
```

---

## 🧪 Verify Task Registration

List all available tasks:

```bash
cd $UNITREE_LAB
~/IsaacLab/isaaclab.sh -p scripts/rsl_rl/list_envs.py
```

**Expected output should include**:
```
Unitree-Go2-Velocity
Unitree-Go2-Velocity-MLP-Custom
Unitree-Go2-Velocity-MLP-No-DR
Unitree-Go2-Velocity-LSTM-DR
Unitree-Go2-Velocity-LSTM-No-DR
Unitree-Go2-Velocity-Implicit
Unitree-Go2-Velocity-Implicit-DR
...
```

---

## 🎓 Training Commands (Also Corrected)

### Train MLP with DR

```bash
cd $UNITREE_LAB
~/IsaacLab/isaaclab.sh -p scripts/rsl_rl/train.py \
  --task Unitree-Go2-Velocity-MLP-Custom \
  --num_envs 4096 \
  --headless
```

### Train LSTM with DR

```bash
cd $UNITREE_LAB
~/IsaacLab/isaaclab.sh -p scripts/rsl_rl/train.py \
  --task Unitree-Go2-Velocity-LSTM-DR \
  --num_envs 4096 \
  --headless
```

### Train Implicit with DR

```bash
cd $UNITREE_LAB
~/IsaacLab/isaaclab.sh -p scripts/rsl_rl/train.py \
  --task Unitree-Go2-Velocity-Implicit-DR \
  --num_envs 4096 \
  --headless
```

---

## Summary

✅ **Key Changes**:
1. Task names use **hyphens** not "With" (e.g., `LSTM-DR` not `LSTM-With-DR`)
2. No `Rough` in task names (that's terrain-specific)
3. Isaac Lab's `isaaclab.sh` manages Python environment automatically
4. Conda activation is **optional** - only if you set up Isaac Lab with conda

✅ **Correct Pattern**:
```
Unitree-Go2-Velocity-[ACTUATOR]-[DR_STATUS]
```

Where:
- `[ACTUATOR]` = MLP-Custom, LSTM, Implicit
- `[DR_STATUS]` = DR, No-DR, or omitted for base

---

**Last Updated**: February 11, 2026

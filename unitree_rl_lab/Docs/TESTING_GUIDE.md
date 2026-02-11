# Robot Testing Guide

This guide shows how to test all 6 trained Go2 models with your 4 specific locomotion tasks.

## ⚠️ IMPORTANT: Velocity Command Control

**The data collection script has been FIXED** to actually control the robot's velocity commands!

The script now:
- ✅ **Overrides** the environment's command manager with your specific commands
- ✅ **Injects** the time-varying velocity patterns from episode configs directly
- ✅ **Ensures** the robot follows your 4 task patterns exactly

Without this fix, the environment would use random commands while only logging the intended ones.

## 📋 Model Status Summary

| Configuration | Checkpoint | Iterations | Status |
|--------------|------------|------------|--------|
| **MLP + Custom DR** | model_24999.pt | ~25000 | ✅ Ready |
| **MLP - No DR** | model_24999.pt | ~25000 | ✅ Ready |
| **LSTM + DR** | model_25000.pt | 25000 | ✅ Ready |
| **LSTM - No DR** | model_25000.pt | 25000 | ✅ Ready |
| **Implicit + DR** | model_14600.pt | 14600 | ⚠️ Latest (58% done) |
| **Implicit - No DR** | model_8100.pt | 8100 | ⚠️ Latest (32% done) |

---

## 🎯 Your 4 Locomotion Tasks

### Task 1: Standing
```python
v_cmd = [0.0, 0.0, 0.0]  # Stay still for 20 seconds
```

### Task 2: Walking (Varying Speeds)
```python
t ∈ [0,5):   [0.5, 0, 0]  # Slow
t ∈ [5,10):  [1.0, 0, 0]  # Normal
t ∈ [10,15): [1.5, 0, 0]  # Fast
t ∈ [15,20]: [0.8, 0, 0]  # Moderate
```

### Task 3: Turn in Place
```python
t ∈ [0,5):   [0, 0, +0.5]  # Slow CCW
t ∈ [5,10):  [0, 0, +1.0]  # Normal CCW
t ∈ [10,15): [0, 0, -1.0]  # Normal CW (direction change!)
t ∈ [15,20]: [0, 0, +1.5]  # Fast CCW
```

### Task 4: Walk + Turn
```python
t ∈ [0,5):   [0.8, 0, +0.6]  # Right arc
t ∈ [5,7):   [1.0, 0,  0.0]  # Straight
t ∈ [7,12):  [0.8, 0, -0.6]  # Left arc
t ∈ [12,15): [1.2, 0,  0.0]  # Fast straight
t ∈ [15,20]: [0.5, 0, +1.0]  # Tight turn
```

---

## 🚀 Quick Start: Test All Models on Your 4 Tasks

### Option A: Automated Testing (Recommended)

Run all 6 models on all 4 tasks automatically:

```bash
cd /home/drl-68/unitree_rl_lab
./test_4_tasks_all_models.sh
```

This will:
1. Generate 200 episodes (4 tasks × 50 repeats each)
2. Test all 6 models on these tasks
3. Save results to `logs/data_collection_4tasks/`

**Expected output:**
- 6 folders with 200 CSV files each = 1200 total files
- Each CSV: 1000 rows × 74 columns of telemetry data

---

### Option B: Manual Testing (Step-by-Step)

#### Step 1: Generate Episode Configs

```bash
python generate_4_task_episodes.py \
  --num_repeats 50 \
  --output episode_configs_4tasks.yaml
```

This creates 200 episodes (50 repeats × 4 tasks).

#### Step 2: Test Individual Models

**MLP + Custom DR:**
```bash
python scripts/data_collection/collect_data_isaaclab.py \
  --task Unitree-Go2-Velocity-MLP-Custom \
  --checkpoint logs/rsl_rl/unitree_go2_velocity_mlp_custom/2026-02-03_15-54-07_work_good/model_24999.pt \
  --episodes episode_configs_4tasks.yaml \
  --output logs/data_collection_4tasks/mlp_custom/ \
  --num_episodes 200 \
  --headless
```

**MLP - No DR:**
```bash
python scripts/data_collection/collect_data_isaaclab.py \
  --task Unitree-Go2-Velocity-MLP-No-DR \
  --checkpoint logs/rsl_rl/unitree_go2_velocity_mlp_no_dr/2026-02-05_00-58-13/model_24999.pt \
  --episodes episode_configs_4tasks.yaml \
  --output logs/data_collection_4tasks/mlp_no_dr/ \
  --num_episodes 200 \
  --headless
```

**LSTM + DR:**
```bash
python scripts/data_collection/collect_data_isaaclab.py \
  --task Unitree-Go2-Velocity-LSTM-DR \
  --checkpoint logs/rsl_rl/unitree_go2_velocity_lstm_dr/2026-02-07_11-16-35/model_25000.pt \
  --episodes episode_configs_4tasks.yaml \
  --output logs/data_collection_4tasks/lstm_dr/ \
  --num_episodes 200 \
  --headless
```

**LSTM - No DR:**
```bash
python scripts/data_collection/collect_data_isaaclab.py \
  --task Unitree-Go2-Velocity-LSTM-No-DR \
  --checkpoint logs/rsl_rl/unitree_go2_velocity_lstm_no_dr/2026-02-07_22-16-50/model_25000.pt \
  --episodes episode_configs_4tasks.yaml \
  --output logs/data_collection_4tasks/lstm_no_dr/ \
  --num_episodes 200 \
  --headless
```

**Implicit + DR (Latest: 14600):**
```bash
python scripts/data_collection/collect_data_isaaclab.py \
  --task Unitree-Go2-Velocity-Implicit-DR \
  --checkpoint logs/rsl_rl/unitree_go2_velocity_implicit_dr/2026-02-05_18-14-00/model_14600.pt \
  --episodes episode_configs_4tasks.yaml \
  --output logs/data_collection_4tasks/implicit_dr/ \
  --num_episodes 200 \
  --headless
```

**Implicit - No DR (Latest: 8100):**
```bash
python scripts/data_collection/collect_data_isaaclab.py \
  --task Unitree-Go2-Velocity-Implicit \
  --checkpoint logs/rsl_rl/unitree_go2_velocity_implicit/2026-02-05_16-27-42/model_8100.pt \
  --episodes episode_configs_4tasks.yaml \
  --output logs/data_collection_4tasks/implicit/ \
  --num_episodes 200 \
  --headless
```

---

## 🎥 Option C: Visual Testing (No Data Collection)

Just watch the robots perform (no data logging):

```bash
./test_all_models.sh
```

This runs the standard play script with random commands and video recording.

---

## 📊 Verify Results

After testing, check your data:

```bash
# Count total CSV files (should be 1200)
ls logs/data_collection_4tasks/*/*.csv | wc -l

# Check file structure
head -5 logs/data_collection_4tasks/mlp_custom/locomotion_log_isaac_ep000_*.csv

# Verify no NaN values
grep -c "NaN" logs/data_collection_4tasks/*/*.csv
```

---

## 📝 Episode Config Structure

The generated `episode_configs_4tasks.yaml` has this structure:

```yaml
metadata:
  generator: generate_4_task_episodes.py
  created_at: '2026-02-08T...'
  num_tasks: 4
  num_repeats: 50
  total_episodes: 200
  episode_duration: 20.0
  control_freq: 50.0

episodes:
  - id: 0
    seed: 42
    task_name: Task1_Standing
    repeat_index: 0
    initial_state:
      base_pos: [0.0, 0.0, 0.30]
      base_quat: [0.0, 0.0, 0.0, 1.0]
      joint_pos: [0.0, 0.9, -1.8, ...]
      joint_vel: [0.0, 0.0, ...]
    command_sequence:
      - {vx: 0.0, vy: 0.0, wz: 0.0, duration: 20.0}

  - id: 1
    seed: 43
    task_name: Task1_Standing
    ...
```

Each task is repeated 50 times with slightly randomized initial states.

---

## 🔍 Output Data Format

Each CSV file contains **74 columns** × **~1000 rows** (20s @ 50Hz):

| Column Group | Count | Examples |
|-------------|-------|----------|
| Metadata | 5 | timestamp_sim, episode_id, seed, control_cycle |
| Performance | 3 | rtf, control_latency_ms, actual_control_dt |
| Base State | 17 | base_pos_xyz, base_quat, roll/pitch/yaw, velocities |
| Commands | 3 | **cmd_vx, cmd_vy, cmd_wz** ← Your task commands! |
| Joint State | 36 | joint_pos (12), joint_vel (12), joint_acc (12) |
| Actions | 48 | action (12), cmd_joint_pos (12), torques |
| Contact | 12 | foot_contact, forces, slip |

---

## 🎯 Expected Behavior per Task

### Task 1 (Standing):
- `cmd_vx = cmd_vy = cmd_wz = 0.0` throughout
- Base should remain stationary
- Small swaying is normal due to PD control

### Task 2 (Walking):
- Forward velocity increases: 0.5 → 1.0 → 1.5 → 0.8 m/s
- Base should accelerate/decelerate smoothly
- Check actual base velocity tracks commanded velocity

### Task 3 (Turn in Place):
- Yaw rate changes: +0.5 → +1.0 → -1.0 → +1.5 rad/s
- Direction reversal at t=10s (CW → CCW)
- Base should rotate without translation

### Task 4 (Walk + Turn):
- Combined linear + angular velocity
- Right arc → straight → left arc → fast straight → tight turn
- Most challenging task

---

## 📁 Files Created

| File | Purpose |
|------|---------|
| [`test_all_models.sh`](test_all_models.sh) | Visual testing with video recording |
| [`generate_4_task_episodes.py`](generate_4_task_episodes.py) | Generate custom episode configs |
| [`test_4_tasks_all_models.sh`](test_4_tasks_all_models.sh) | Automated testing pipeline |
| `episode_configs_4tasks.yaml` | Generated episode configs (200 episodes) |

---

## 🔧 Troubleshooting

### Issue: "Model checkpoint not found"
**Solution:** Check that training completed and checkpoint exists:
```bash
ls logs/rsl_rl/unitree_go2_velocity_*/*/model_*.pt
```

### Issue: "TorchScript deserialization error"
**Solution:** Model was trained with different PyTorch version. Retrain or use compatible version.

### Issue: "Episode config not found"
**Solution:** Run the episode generator first:
```bash
python generate_4_task_episodes.py --output episode_configs_4tasks.yaml
```

### Issue: Data collection crashes
**Solution:** Reduce `--num_envs` to 1 for debugging:
```bash
python scripts/data_collection/collect_data_isaaclab.py ... --num_envs 1
```

---

## 📖 Related Documentation

- **Main README**: [`README.md`](README.md) - Installation and overview
- **Data Collection Plan**: [`STEP_BY_STEP_DATA_COLLECTION.md`](STEP_BY_STEP_DATA_COLLECTION.md)
- **Training Configs**: [`TRAINING_CONFIGS_COMPARISON.md`](TRAINING_CONFIGS_COMPARISON.md)
- **Master Plan**: [`MASTER_PLAN_ALL_6_CONFIGS.md`](MASTER_PLAN_ALL_6_CONFIGS.md)

---

## 🎓 Next Steps

After collecting data:
1. Analyze success rates per task
2. Compare MLP vs LSTM vs Implicit actuators
3. Evaluate DR vs No-DR on robustness
4. Train mimic policies or downstream tasks
5. Deploy to real Go2 hardware!

---

**Happy Testing! 🤖**

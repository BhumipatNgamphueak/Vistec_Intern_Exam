# 📊 Step-by-Step Data Collection Guide

Complete guide to collect telemetry data from your trained Go2 policies in IsaacLab.

---

## 📋 Prerequisites

✅ Trained policy checkpoints:
- LSTM-DR: `logs/rsl_rl/unitree_go2_velocity_lstm_dr/2026-02-06_23-20-45/model_7400.pt`
- LSTM-No-DR: `logs/rsl_rl/unitree_go2_velocity_lstm_no_dr/2026-02-06_18-49-02/model_4100.pt`

✅ Python packages:
```bash
pip install numpy pandas pyyaml scipy torch
```

---

## 🚀 Step 1: Generate Episode Configurations

Generate 200 test episodes with randomized initial states and commands.

```bash
cd /home/drl-68/unitree_rl_lab

# Generate 200 episodes
python episode_config_generator.py \
  --num_episodes 200 \
  --duration 20.0 \
  --control_freq 50.0 \
  --seed 42 \
  --output episode_configs.yaml
```

**Expected Output:**
```
Generating episode configurations...
✓ Saved 200 episode configurations to episode_configs.yaml
  - Episodes: 200
  - Duration: 20.0 sec each
  - Control rate: 50 Hz
  - Total timesteps: 1000 per episode
```

**Verify:**
```bash
# Check file was created
ls -lh episode_configs.yaml

# View first few lines
head -50 episode_configs.yaml
```

---

## 🚀 Step 2: Prepare Policy Checkpoints

Organize your trained policies for data collection.

```bash
# Create policies directory
mkdir -p policies

# Option A: Use checkpoint model_7400 (LSTM-DR)
cp logs/rsl_rl/unitree_go2_velocity_lstm_dr/2026-02-06_23-20-45/model_7400.pt \
   policies/policy_lstm_dr.pt

# Option B: Wait for training to complete and use final model
# cp logs/rsl_rl/unitree_go2_velocity_lstm_dr/2026-02-06_23-20-45/model_25000.pt \
#    policies/policy_lstm_dr_final.pt

# Option C: Use LSTM-No-DR checkpoint
cp logs/rsl_rl/unitree_go2_velocity_lstm_no_dr/2026-02-06_18-49-02/model_4100.pt \
   policies/policy_lstm_no_dr.pt
```

**Verify policy loads:**
```bash
python -c "
import torch
policy = torch.jit.load('policies/policy_lstm_dr.pt')
print('✓ Policy loaded successfully')
print(f'  Policy type: {type(policy)}')
"
```

---

## 🚀 Step 3: Test with 1 Episode

Before running all 200 episodes, test with a single episode.

```bash
# Test with 1 episode (LSTM-DR)
python scripts/data_collection/collect_data_isaaclab.py \
  --task Unitree-Go2-Velocity-LSTM-DR \
  --checkpoint policies/policy_lstm_dr.pt \
  --episodes episode_configs.yaml \
  --output logs/data_collection_test/ \
  --num_episodes 1 \
  --headless
```

**Expected Output:**
```
Loading episode configs from episode_configs.yaml
Loaded 200 episodes
Creating environment: Unitree-Go2-Velocity-LSTM-DR
[INFO]: Completed setting up the environment...
Loading policy from policies/policy_lstm_dr.pt
Data collector initialized

============================================================
Episode 0 (seed: 42)
============================================================
  Step 100/1000 (RTF: 0.98, Power: 125.3W)
  Step 200/1000 (RTF: 0.99, Power: 132.1W)
  ...
  Step 1000/1000 (RTF: 0.98, Power: 128.7W)
✓ Saved: logs/data_collection_test/locomotion_log_isaac_ep000_2026-02-07_15-30-22.csv
  Rows: 1000, Columns: 74

============================================================
Data Collection Complete
============================================================
Episodes completed: 1
Output directory: logs/data_collection_test/
============================================================
```

**Verify Output:**
```bash
# Check CSV file
ls -lh logs/data_collection_test/*.csv

# View first few rows
head -20 logs/data_collection_test/*.csv

# Check row count (should be 1001 = 1000 data + 1 header)
wc -l logs/data_collection_test/*.csv

# Verify no NaN values
grep -c "NaN" logs/data_collection_test/*.csv
```

---

## 🚀 Step 4: Collect Data for All Episodes

### Option A: LSTM with DR (Full 200 Episodes)

```bash
cd /home/drl-68/unitree_rl_lab

# Create output directory
mkdir -p logs/data_collection_lstm_dr

# Run data collection
python scripts/data_collection/collect_data_isaaclab.py \
  --task Unitree-Go2-Velocity-LSTM-DR \
  --checkpoint policies/policy_lstm_dr.pt \
  --episodes episode_configs.yaml \
  --output logs/data_collection_lstm_dr/ \
  --num_episodes 200 \
  --headless
```

**Expected Duration:** ~70 minutes (200 episodes × 21 sec/episode)

### Option B: LSTM without DR (Full 200 Episodes)

```bash
cd /home/drl-68/unitree_rl_lab

# Create output directory
mkdir -p logs/data_collection_lstm_no_dr

# Run data collection
python scripts/data_collection/collect_data_isaaclab.py \
  --task Unitree-Go2-Velocity-LSTM-No-DR \
  --checkpoint policies/policy_lstm_no_dr.pt \
  --episodes episode_configs.yaml \
  --output logs/data_collection_lstm_no_dr/ \
  --num_episodes 200 \
  --headless
```

### Option C: Collect in Batches (Safer)

If you want to collect data in smaller batches:

```bash
# Batch 1: Episodes 0-49
python scripts/data_collection/collect_data_isaaclab.py \
  --task Unitree-Go2-Velocity-LSTM-DR \
  --checkpoint policies/policy_lstm_dr.pt \
  --episodes episode_configs.yaml \
  --output logs/data_collection_lstm_dr/ \
  --num_episodes 50 \
  --headless

# Batch 2: Episodes 50-99
# (Modify script to start from episode 50)

# Batch 3: Episodes 100-149
# ...

# Batch 4: Episodes 150-199
# ...
```

---

## 🚀 Step 5: Monitor Progress

While data collection is running, monitor progress:

### Terminal 1: Run data collection
```bash
python scripts/data_collection/collect_data_isaaclab.py ...
```

### Terminal 2: Monitor files
```bash
# Watch new files being created
watch -n 5 'ls -lht logs/data_collection_lstm_dr/*.csv | head -10'

# Count completed episodes
watch -n 5 'ls logs/data_collection_lstm_dr/*.csv | wc -l'

# Check last file size
watch -n 5 'ls -lh logs/data_collection_lstm_dr/*.csv | tail -1'
```

### Terminal 3: Monitor system resources
```bash
# GPU usage
watch -n 2 nvidia-smi

# CPU and memory
htop
```

---

## 🚀 Step 6: Validate Collected Data

After data collection completes:

```bash
cd /home/drl-68/unitree_rl_lab

# Count total episodes
ls logs/data_collection_lstm_dr/*.csv | wc -l
# Expected: 200

# Check all files have correct row count
for f in logs/data_collection_lstm_dr/*.csv; do
    lines=$(wc -l < "$f")
    if [ $lines -ne 1001 ]; then
        echo "WARNING: $f has $lines lines (expected 1001)"
    fi
done

# Check for NaN values (should be 0)
grep -r "NaN" logs/data_collection_lstm_dr/*.csv
# Expected: (no output)

# Verify all episodes present
python -c "
import os
import re

files = os.listdir('logs/data_collection_lstm_dr/')
episode_ids = []
for f in files:
    match = re.search(r'ep(\d{3})', f)
    if match:
        episode_ids.append(int(match.group(1)))

episode_ids.sort()
print(f'Found {len(episode_ids)} episodes')
print(f'Range: {min(episode_ids)} to {max(episode_ids)}')

missing = set(range(200)) - set(episode_ids)
if missing:
    print(f'Missing episodes: {sorted(missing)}')
else:
    print('✓ All 200 episodes collected')
"
```

---

## 🚀 Step 7: Quick Data Analysis

Analyze one episode to verify data quality:

```bash
python << 'EOF'
import pandas as pd
import numpy as np

# Load first episode
df = pd.read_csv('logs/data_collection_lstm_dr/locomotion_log_isaac_ep000_2026-02-07_15-30-22.csv')

print("="*60)
print("Data Quality Check")
print("="*60)
print(f"Shape: {df.shape}")
print(f"Columns: {len(df.columns)}")
print(f"Rows: {len(df)}")
print(f"\nTimestamps:")
print(f"  Duration: {df['timestamp_sim'].max():.2f} sec")
print(f"  Frequency: {len(df) / df['timestamp_sim'].max():.1f} Hz")
print(f"\nPerformance:")
print(f"  Mean RTF: {df['rtf'].mean():.3f}")
print(f"  Mean latency: {df['control_latency_ms'].mean():.2f} ms")
print(f"\nBase state:")
print(f"  Height: {df['base_pos_z'].mean():.3f} ± {df['base_pos_z'].std():.3f} m")
print(f"  Roll: {np.rad2deg(df['base_roll'].std()):.2f}°")
print(f"  Pitch: {np.rad2deg(df['base_pitch'].std()):.2f}°")
print(f"\nPower consumption:")
print(f"  Mean: {df['instantaneous_power'].mean():.1f} W")
print(f"  Max: {df['instantaneous_power'].max():.1f} W")
print(f"\nAction smoothness:")
print(f"  Mean: {df['action_smoothness'].mean():.4f}")
print(f"  Max: {df['action_smoothness'].max():.4f}")
print("="*60)

# Check for issues
issues = []
if df['rtf'].mean() < 0.9:
    issues.append(f"⚠ Low RTF: {df['rtf'].mean():.3f}")
if df.isnull().any().any():
    issues.append(f"⚠ Contains NaN values")
if (df['torque_saturation_count'] > 0).sum() > 100:
    issues.append(f"⚠ Frequent torque saturation")

if issues:
    print("\n⚠ Issues found:")
    for issue in issues:
        print(f"  {issue}")
else:
    print("\n✓ Data quality looks good!")
EOF
```

---

## 🚀 Step 8: Create Summary Report

```bash
python << 'EOF'
import pandas as pd
import glob
import numpy as np

files = sorted(glob.glob('logs/data_collection_lstm_dr/*.csv'))

print("="*60)
print(f"Data Collection Summary")
print("="*60)
print(f"Total episodes: {len(files)}")

# Analyze all episodes
rtf_values = []
power_values = []
latency_values = []

for f in files:
    df = pd.read_csv(f)
    rtf_values.append(df['rtf'].mean())
    power_values.append(df['instantaneous_power'].mean())
    latency_values.append(df['control_latency_ms'].mean())

print(f"\nPerformance Statistics:")
print(f"  RTF: {np.mean(rtf_values):.3f} ± {np.std(rtf_values):.3f}")
print(f"  Latency: {np.mean(latency_values):.2f} ± {np.std(latency_values):.2f} ms")
print(f"\nPower Statistics:")
print(f"  Mean: {np.mean(power_values):.1f} ± {np.std(power_values):.1f} W")
print(f"  Min: {np.min(power_values):.1f} W")
print(f"  Max: {np.max(power_values):.1f} W")

print(f"\nData Size:")
total_size_mb = sum(os.path.getsize(f) for f in files) / (1024 * 1024)
print(f"  Total: {total_size_mb:.1f} MB")
print(f"  Per episode: {total_size_mb / len(files):.2f} MB")
print(f"  Total data points: {len(files) * 1000 * 74:,}")

print("="*60)
EOF
```

---

## 📊 Expected Results

After completing all steps:

```
logs/
├── data_collection_lstm_dr/
│   ├── locomotion_log_isaac_ep000_2026-02-07_15-30-22.csv (74 cols × 1000 rows)
│   ├── locomotion_log_isaac_ep001_2026-02-07_15-30-43.csv
│   ├── ...
│   └── locomotion_log_isaac_ep199_2026-02-07_17-40-15.csv
└── data_collection_lstm_no_dr/
    ├── locomotion_log_isaac_ep000_2026-02-07_18-00-10.csv
    ├── ...
    └── locomotion_log_isaac_ep199_2026-02-07_19-10-25.csv
```

**Total Data:**
- 200 episodes × 1000 timesteps × 74 columns = **14.8 million data points**
- File size: ~200 MB (compressed) to 400 MB (uncompressed)

---

## 🐛 Troubleshooting

### Issue 1: "No module named 'isaaclab'"

```bash
# Set IsaacLab path
export ISAACLAB_PATH=/home/drl-68/IsaacLab

# Or add to script
export PYTHONPATH=/home/drl-68/IsaacLab:$PYTHONPATH
```

### Issue 2: Policy checkpoint not found

```bash
# Check checkpoint exists
ls -lh policies/policy_lstm_dr.pt

# Verify it's a valid PyTorch file
python -c "import torch; torch.jit.load('policies/policy_lstm_dr.pt')"
```

### Issue 3: CUDA out of memory

```bash
# Run with CPU only
python scripts/data_collection/collect_data_isaaclab.py \
  --task Unitree-Go2-Velocity-LSTM-DR \
  --checkpoint policies/policy_lstm_dr.pt \
  --episodes episode_configs.yaml \
  --output logs/data_collection_lstm_dr/ \
  --num_episodes 200 \
  --headless \
  --num_envs 1  # Use single environment
```

### Issue 4: Data collection interrupted

```bash
# Count completed episodes
ls logs/data_collection_lstm_dr/*.csv | wc -l

# Resume from where it stopped
# (Would need to modify script to skip completed episodes)
```

### Issue 5: Low RTF (< 0.9)

- Run headless: `--headless`
- Close other applications
- Check GPU isn't being used by other processes: `nvidia-smi`

---

## ⏱️ Time Estimates

| Task | Duration |
|------|----------|
| **Step 1:** Generate configs | 5 seconds |
| **Step 2:** Prepare checkpoints | 1 minute |
| **Step 3:** Test 1 episode | 1 minute |
| **Step 4:** Collect 200 episodes | 60-90 minutes |
| **Step 5:** Monitor | (during collection) |
| **Step 6:** Validate data | 2 minutes |
| **Step 7:** Quick analysis | 1 minute |
| **Step 8:** Summary report | 1 minute |
| **TOTAL** | **~75-100 minutes** |

---

## 🎯 Next Steps

After data collection:

1. ✅ **Compare LSTM-DR vs LSTM-No-DR**
   - Analyze tracking performance
   - Compare power consumption
   - Evaluate action smoothness

2. ✅ **Prepare for Gazebo deployment**
   - Export policies for ROS 2
   - Test in Gazebo simulation
   - Compare sim-to-sim transfer quality

3. ✅ **Analyze domain randomization impact**
   - Robustness metrics
   - Generalization performance
   - Real-world readiness

---

**Ready to start? Begin with Step 1!** 🚀

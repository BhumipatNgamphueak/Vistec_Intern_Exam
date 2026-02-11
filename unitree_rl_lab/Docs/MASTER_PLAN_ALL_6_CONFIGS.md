# 🎯 Master Plan: Data Collection from All 6 Configurations

Complete plan to train all models to 25000 iterations and collect data from all 6 configurations.

---

## 📊 Current Status

| # | Configuration | Task Name | Latest Model | Status | Action Needed |
|---|---------------|-----------|--------------|--------|---------------|
| 1 | **LSTM + DR** | `Unitree-Go2-Velocity-LSTM-DR` | model_25600.pt | ✅ **READY** | None - Already trained! |
| 2 | **LSTM - No DR** | `Unitree-Go2-Velocity-LSTM-No-DR` | model_27200.pt | ✅ **READY** | None - Already trained! |
| 3 | **MLP + DR** | `Unitree-Go2-Velocity-MLP-Custom` | model_24999.pt | ✅ **READY** | None - Already trained! |
| 4 | **MLP - No DR** | `Unitree-Go2-Velocity-MLP-No-DR` | model_24999.pt | ✅ **READY** | None - Already trained! |
| 5 | **Implicit + DR** | `Unitree-Go2-Velocity-Implicit-DR` | model_14600.pt | ⏳ **58% done** | Train 10,400 more iterations |
| 6 | **Implicit - No DR** | `Unitree-Go2-Velocity-Implicit` | model_8100.pt | ⏳ **32% done** | Train 16,900 more iterations |

**Summary:** 4/6 ready, 2/6 need more training

---

## 🚀 Action Plan

### **Phase 1: Complete Training (2 models)** ⏰ ~12-14 hours

#### Option A: Train Sequentially (Safer)

```bash
cd /home/drl-68/unitree_rl_lab

# Step 1: Train Implicit + DR (10,400 iterations ≈ 7 hours)
./continue_implicit_dr_training.sh

# Step 2: Train Implicit - No DR (16,900 iterations ≈ 11 hours)
./continue_implicit_no_dr_training.sh
```

**Total time:** ~18 hours (sequential)

#### Option B: Train in Parallel (Faster) ⭐ Recommended

```bash
cd /home/drl-68/unitree_rl_lab

# Terminal 1: Implicit + DR
./continue_implicit_dr_training.sh

# Terminal 2: Implicit - No DR
./continue_implicit_no_dr_training.sh
```

**Total time:** ~11 hours (parallel - limited by slower one)

⚠️ **Note:** Requires enough GPU memory to run 2 trainings simultaneously (2048 envs each).
If CUDA OOM occurs, reduce `--num_envs` to 1024 in both scripts.

---

### **Phase 2: Collect Data from All 6 Models** ⏰ ~7-9 hours

After all training completes:

```bash
cd /home/drl-68/unitree_rl_lab

# Collect all 6 configurations (200 episodes each)
./collect_all_data.sh
```

**This will:**
1. Generate 200 episode configurations (if not exists)
2. Copy all checkpoints to `policies/` directory
3. Collect data from each model:
   - LSTM + DR → `logs/data_collection/lstm_dr/`
   - LSTM - No DR → `logs/data_collection/lstm_no_dr/`
   - MLP + DR → `logs/data_collection/mlp_dr/`
   - MLP - No DR → `logs/data_collection/mlp_no_dr/`
   - Implicit + DR → `logs/data_collection/implicit_dr/`
   - Implicit - No DR → `logs/data_collection/implicit_no_dr/`

**Duration per configuration:** ~70 minutes (200 episodes × 21 sec)
**Total duration:** 6 × 70 min = **~7 hours** (sequential)

---

### **Phase 3: Validate & Analyze** ⏰ ~30 minutes

```bash
cd /home/drl-68/unitree_rl_lab

# Validate all collected data
python << 'EOF'
import pandas as pd
import glob
import os

configs = ['lstm_dr', 'lstm_no_dr', 'mlp_dr', 'mlp_no_dr', 'implicit_dr', 'implicit_no_dr']

print("="*80)
print("Data Collection Validation")
print("="*80)

for config in configs:
    files = glob.glob(f'logs/data_collection/{config}/*.csv')

    if len(files) == 0:
        print(f"\n{config}: ✗ No data collected")
        continue

    # Check episode count
    episode_count = len(files)
    status = "✓" if episode_count == 200 else "⚠"
    print(f"\n{config}: {status} {episode_count}/200 episodes")

    # Check first file structure
    if files:
        df = pd.read_csv(files[0])
        print(f"  Columns: {len(df.columns)}")
        print(f"  Rows per episode: {len(df)}")
        print(f"  Data quality: {'✓ OK' if not df.isnull().any().any() else '✗ Contains NaN'}")

        # Check file size
        total_size = sum(os.path.getsize(f) for f in files) / (1024**2)
        print(f"  Total size: {total_size:.1f} MB")

print("\n" + "="*80)
EOF
```

---

## 📋 Detailed Timelines

### **Timeline Option 1: Sequential Training + Sequential Collection**

| Phase | Task | Duration | Cumulative |
|-------|------|----------|------------|
| 1a | Train Implicit + DR | 7 hours | 7h |
| 1b | Train Implicit - No DR | 11 hours | 18h |
| 2 | Collect all 6 configs | 7 hours | 25h |
| 3 | Validate & analyze | 0.5 hours | 25.5h |
| **TOTAL** | | | **~26 hours** |

### **Timeline Option 2: Parallel Training + Sequential Collection** ⭐

| Phase | Task | Duration | Cumulative |
|-------|------|----------|------------|
| 1 | Train both Implicit (parallel) | 11 hours | 11h |
| 2 | Collect all 6 configs | 7 hours | 18h |
| 3 | Validate & analyze | 0.5 hours | 18.5h |
| **TOTAL** | | | **~19 hours** |

### **Timeline Option 3: Parallel Training + Batch Collection** 🚀 Fastest

Train in parallel, collect in batches of 2-3 simultaneously:

| Phase | Task | Duration | Cumulative |
|-------|------|----------|------------|
| 1 | Train both Implicit (parallel) | 11 hours | 11h |
| 2a | Collect 3 configs (batch 1) | 7 hours | 18h |
| 2b | Collect 3 configs (batch 2) | 7 hours | 18h (parallel) |
| 3 | Validate & analyze | 0.5 hours | 18.5h |
| **TOTAL** | | | **~18-19 hours** |

---

## 💾 Expected Data Output

### **Storage Requirements**

| Configuration | Episodes | Size per Episode | Total Size |
|---------------|----------|------------------|------------|
| LSTM + DR | 200 | ~2 MB | ~400 MB |
| LSTM - No DR | 200 | ~2 MB | ~400 MB |
| MLP + DR | 200 | ~2 MB | ~400 MB |
| MLP - No DR | 200 | ~2 MB | ~400 MB |
| Implicit + DR | 200 | ~2 MB | ~400 MB |
| Implicit - No DR | 200 | ~2 MB | ~400 MB |
| **TOTAL** | **1,200** | | **~2.4 GB** |

### **Data Structure**

```
logs/data_collection/
├── lstm_dr/
│   ├── locomotion_log_isaac_ep000_2026-02-08_21-00-00.csv
│   ├── locomotion_log_isaac_ep001_2026-02-08_21-00-21.csv
│   ├── ...
│   └── locomotion_log_isaac_ep199_2026-02-08_23-20-00.csv
├── lstm_no_dr/
│   └── (200 CSV files)
├── mlp_dr/
│   └── (200 CSV files)
├── mlp_no_dr/
│   └── (200 CSV files)
├── implicit_dr/
│   └── (200 CSV files)
└── implicit_no_dr/
    └── (200 CSV files)
```

**Total:** 1,200 CSV files × (1,000 rows × 74 columns) = **88.8 million data points**

---

## 🎯 Quick Commands Cheat Sheet

### **Check Training Status**

```bash
# Check latest checkpoints
for dir in logs/rsl_rl/unitree_go2_velocity_*/; do
    latest=$(ls -t "$dir"/*/model_*.pt 2>/dev/null | head -1)
    echo "$(basename $dir): $(basename $latest 2>/dev/null || echo 'No models')"
done
```

### **Monitor Training Progress**

```bash
# Watch Implicit-DR progress
watch -n 10 'ls -lht logs/rsl_rl/unitree_go2_velocity_implicit_dr/*/model_*.pt | head -5'

# Watch Implicit-No-DR progress
watch -n 10 'ls -lht logs/rsl_rl/unitree_go2_velocity_implicit/*/model_*.pt | head -5'
```

### **Monitor Data Collection**

```bash
# Count collected episodes per config
for config in lstm_dr lstm_no_dr mlp_dr mlp_no_dr implicit_dr implicit_no_dr; do
    count=$(ls logs/data_collection/$config/*.csv 2>/dev/null | wc -l)
    echo "$config: $count/200"
done

# Watch live progress
watch -n 5 'for c in lstm_dr lstm_no_dr mlp_dr mlp_no_dr implicit_dr implicit_no_dr; do echo "$c: $(ls logs/data_collection/$c/*.csv 2>/dev/null | wc -l)/200"; done'
```

### **GPU Monitoring**

```bash
# Monitor GPU usage
watch -n 2 nvidia-smi

# Check if both trainings are running
nvidia-smi | grep python
```

---

## 🐛 Troubleshooting

### Issue 1: CUDA Out of Memory (Training Both in Parallel)

**Solution:** Reduce environments in one or both scripts:

```bash
# Edit continue_implicit_dr_training.sh
# Change: --num_envs 2048
# To:     --num_envs 1024

# Edit continue_implicit_no_dr_training.sh
# Change: --num_envs 2048
# To:     --num_envs 1024
```

### Issue 2: Training Interrupted

**Solution:** Resume from last checkpoint:

```bash
# Find latest checkpoint
ls -lht logs/rsl_rl/unitree_go2_velocity_implicit_dr/*/model_*.pt | head -1

# Update script with new checkpoint number
# Edit continue_implicit_dr_training.sh
# Change: --checkpoint model_14600.pt
# To:     --checkpoint model_XXXXX.pt  # Use latest
```

### Issue 3: Data Collection Fails for One Config

**Solution:** Collect that config individually:

```bash
# Collect only the failed config
./collect_all_data.sh lstm_dr  # Just collect LSTM-DR
./collect_all_data.sh implicit_dr mlp_no_dr  # Collect multiple
```

### Issue 4: Model Checkpoints Don't Reach Exactly 25000

**Solution:** Use closest checkpoint (24999 or 25600 is fine):

```bash
# Edit collect_all_data.sh if needed
# Update the checkpoint name in CONFIGS array
```

---

## ✅ Success Criteria

Before considering data collection complete, verify:

- [ ] All 6 models trained to ≥24999 iterations
- [ ] All 6 models have checkpoints copied to `policies/`
- [ ] All policies load successfully (no TorchScript errors)
- [ ] Each config has exactly 200 CSV files
- [ ] Each CSV has 1,000 rows (+ 1 header = 1,001 total lines)
- [ ] Each CSV has 74 columns
- [ ] No NaN values in any CSV file
- [ ] Total data size ≈ 2.4 GB
- [ ] Average RTF > 0.9 across all episodes

---

## 🚀 Recommended Execution Plan

**Day 1 (Evening):**
```bash
# Start both trainings in parallel
# Terminal 1
./continue_implicit_dr_training.sh

# Terminal 2
./continue_implicit_no_dr_training.sh

# Let run overnight (~11 hours)
```

**Day 2 (Morning):**
```bash
# Verify training completed
ls -lh logs/rsl_rl/unitree_go2_velocity_implicit_dr/*/model_25000.pt
ls -lh logs/rsl_rl/unitree_go2_velocity_implicit/*/model_25000.pt

# Start data collection (~7 hours)
./collect_all_data.sh

# Monitor progress in another terminal
watch -n 30 'for c in lstm_dr lstm_no_dr mlp_dr mlp_no_dr implicit_dr implicit_no_dr; do echo "$c: $(ls logs/data_collection/$c/*.csv 2>/dev/null | wc -l)/200"; done'
```

**Day 2 (Afternoon):**
```bash
# Validate results
python validation_script.py

# Analyze and compare
python analysis_script.py
```

---

## 📊 Expected Results

After completing all steps, you'll have:

✅ **6 trained policies** at 25000 iterations
✅ **1,200 episodes** of telemetry data (200 × 6 configs)
✅ **88.8 million data points** (74 columns × 1,000 timesteps × 1,200 episodes)
✅ **Complete comparison** of:
   - Actuator types: LSTM vs MLP vs Implicit
   - Domain randomization: With DR vs Without DR
   - Performance metrics: tracking, power, smoothness, etc.

---

**Ready to start? Run Phase 1!** 🚀

```bash
./continue_implicit_dr_training.sh      # Terminal 1
./continue_implicit_no_dr_training.sh   # Terminal 2
```

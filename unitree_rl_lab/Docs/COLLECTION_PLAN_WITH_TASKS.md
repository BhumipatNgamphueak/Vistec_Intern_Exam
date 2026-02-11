# 📊 Data Collection Plan: All 6 Configs × 4 Tasks

Complete plan to collect structured locomotion data from all 6 configurations using 4 specific tasks.

---

## 🎯 Test Structure

### **4 Locomotion Tasks**

| Task # | Name | Description | Command Profile |
|--------|------|-------------|-----------------|
| **1** | **Standing** | Maintain balance at zero velocity | `[0.0, 0.0, 0.0]` for 20s |
| **2** | **Walking** | Forward locomotion (0.5→1.0→1.5→0.8 m/s) | 4 speed changes |
| **3** | **Turn in Place** | Yaw rotation (+0.5→+1.0→-1.0→+1.5 rad/s) | Direction reversal! |
| **4** | **Walk + Turn** | Combined forward + yaw motion | 5 maneuvers (arcs, straights, tight turns) |

### **Command Details**

#### Task 1: Standing
```python
t ∈ [0,20]: [0.0, 0.0, 0.0]  # Stay still
```

#### Task 2: Walking
```python
t ∈ [0,5):   [0.5, 0.0, 0.0]  # Slow
t ∈ [5,10):  [1.0, 0.0, 0.0]  # Normal
t ∈ [10,15): [1.5, 0.0, 0.0]  # Fast
t ∈ [15,20]: [0.8, 0.0, 0.0]  # Moderate
```

#### Task 3: Turn in Place
```python
t ∈ [0,5):   [0.0, 0.0, +0.5]  # Slow CCW
t ∈ [5,10):  [0.0, 0.0, +1.0]  # Normal CCW
t ∈ [10,15): [0.0, 0.0, -1.0]  # Normal CW (direction change!)
t ∈ [15,20]: [0.0, 0.0, +1.5]  # Fast CCW
```

#### Task 4: Walk + Turn
```python
t ∈ [0,5):   [0.8, 0.0, +0.6]  # Right arc
t ∈ [5,7):   [1.0, 0.0,  0.0]  # Straight
t ∈ [7,12):  [0.8, 0.0, -0.6]  # Left arc
t ∈ [12,15): [1.2, 0.0,  0.0]  # Fast straight
t ∈ [15,20]: [0.5, 0.0, +1.0]  # Tight turn
```

---

## 📊 Test Matrix

### **6 Configurations × 4 Tasks = 24 Test Conditions**

| Config | Task 1<br>Standing | Task 2<br>Walking | Task 3<br>Turn | Task 4<br>Walk+Turn |
|--------|:------------------:|:-----------------:|:--------------:|:-------------------:|
| **LSTM + DR** | 50 episodes | 50 episodes | 50 episodes | 50 episodes |
| **LSTM - No DR** | 50 episodes | 50 episodes | 50 episodes | 50 episodes |
| **MLP + DR** | 50 episodes | 50 episodes | 50 episodes | 50 episodes |
| **MLP - No DR** | 50 episodes | 50 episodes | 50 episodes | 50 episodes |
| **Implicit + DR** | 50 episodes | 50 episodes | 50 episodes | 50 episodes |
| **Implicit - No DR** | 50 episodes | 50 episodes | 50 episodes | 50 episodes |

**Total:** 6 configs × 4 tasks × 50 episodes = **1,200 episodes**

---

## 🚀 Quick Start

### **Step 1: Generate Task-Based Episodes** ⏰ 5 seconds

```bash
cd /home/drl-68/unitree_rl_lab

# Generate 200 episodes (50 per task × 4 tasks)
python episode_config_generator_tasks.py \
  --episodes_per_task 50 \
  --output episode_configs_tasks.yaml
```

**Output:**
```
✓ 200 episodes generated
  - Standing: 50 episodes
  - Walking: 50 episodes
  - Turn in Place: 50 episodes
  - Walk + Turn: 50 episodes
```

---

### **Step 2: Complete Training (if needed)** ⏰ ~11 hours

Only Implicit models need more training:

```bash
# Terminal 1: Implicit + DR (10,400 iterations remaining)
./continue_implicit_dr_training.sh

# Terminal 2: Implicit - No DR (16,900 iterations remaining)
./continue_implicit_no_dr_training.sh
```

---

### **Step 3: Collect Data from All Configs** ⏰ ~7 hours

```bash
# Update collect_all_data.sh to use task-based episodes
./collect_all_data.sh
```

**Or manually specify:**
```bash
python scripts/data_collection/collect_data_isaaclab.py \
  --task Unitree-Go2-Velocity-LSTM-DR \
  --checkpoint policies/policy_lstm_dr.pt \
  --episodes episode_configs_tasks.yaml \
  --output logs/data_collection_tasks/lstm_dr/ \
  --num_episodes 200 \
  --headless
```

---

## 📁 Expected Output Structure

```
logs/data_collection_tasks/
├── lstm_dr/
│   ├── locomotion_log_isaac_ep000_*.csv  # Standing (episodes 0-49)
│   ├── locomotion_log_isaac_ep050_*.csv  # Walking (episodes 50-99)
│   ├── locomotion_log_isaac_ep100_*.csv  # Turn in Place (episodes 100-149)
│   └── locomotion_log_isaac_ep150_*.csv  # Walk+Turn (episodes 150-199)
├── lstm_no_dr/
│   └── (200 episodes...)
├── mlp_dr/
│   └── (200 episodes...)
├── mlp_no_dr/
│   └── (200 episodes...)
├── implicit_dr/
│   └── (200 episodes...)
└── implicit_no_dr/
    └── (200 episodes...)
```

**Total:** 1,200 CSV files, ~2.4 GB

---

## 📊 Analysis Structure

After collection, you can analyze by:

### **By Configuration (Actuator Comparison)**
```python
# Compare LSTM vs MLP vs Implicit on walking task
configs = ['lstm_dr', 'mlp_dr', 'implicit_dr']
walking_episodes = range(50, 100)  # Episodes 50-99

for config in configs:
    data = load_episodes(config, walking_episodes)
    analyze_tracking_performance(data)
```

### **By Task (Robustness Analysis)**
```python
# Compare all configs on turn-in-place task
task_episodes = range(100, 150)  # Episodes 100-149

for config in all_configs:
    data = load_episodes(config, task_episodes)
    analyze_turning_stability(data)
```

### **By Domain Randomization (DR Impact)**
```python
# Compare DR vs No-DR for each actuator
pairs = [
    ('lstm_dr', 'lstm_no_dr'),
    ('mlp_dr', 'mlp_no_dr'),
    ('implicit_dr', 'implicit_no_dr')
]

for task_range in [standing, walking, turning, combined]:
    for with_dr, without_dr in pairs:
        compare_robustness(with_dr, without_dr, task_range)
```

---

## 🎯 Key Metrics to Analyze

### **Task 1: Standing (Balance)**
- Base position drift (X, Y, Z)
- Orientation stability (roll, pitch, yaw)
- Joint torque variance
- Power consumption (should be minimal)

### **Task 2: Walking (Tracking)**
- Velocity tracking error (commanded vs actual)
- Acceleration smoothness during transitions
- Foot contact stability
- Energy efficiency at different speeds

### **Task 3: Turn in Place (Agility)**
- Yaw tracking accuracy
- Response time to direction reversal
- Base translation (should be minimal)
- Stability during high angular velocity

### **Task 4: Walk + Turn (Combined)**
- Multi-objective tracking (vx + wz simultaneously)
- Path curvature accuracy
- Transition smoothness (arc → straight → arc)
- Recovery from tight turns

---

## 📈 Expected Performance Insights

### **By Actuator Type**

| Actuator | Expected Strength | Expected Weakness |
|----------|-------------------|-------------------|
| **LSTM** | Best tracking accuracy (temporal memory) | Slower inference |
| **MLP** | Fast inference, good balance | Less adaptive to dynamics |
| **Implicit** | Fastest simulation | Largest sim-to-real gap |

### **By Domain Randomization**

| Config | Expected Performance |
|--------|---------------------|
| **With DR** | More robust, handles variations better |
| **Without DR** | Better tracking in nominal conditions, brittle to changes |

---

## 🛠️ Quick Commands

### Generate Episodes
```bash
python episode_config_generator_tasks.py \
  --episodes_per_task 50 \
  --output episode_configs_tasks.yaml
```

### Collect Single Config
```bash
python scripts/data_collection/collect_data_isaaclab.py \
  --task Unitree-Go2-Velocity-LSTM-DR \
  --checkpoint policies/policy_lstm_dr.pt \
  --episodes episode_configs_tasks.yaml \
  --output logs/data_collection_tasks/lstm_dr/ \
  --num_episodes 200 \
  --headless
```

### Analyze Task Performance
```python
import pandas as pd
import glob

# Load all standing episodes for LSTM-DR
files = sorted(glob.glob('logs/data_collection_tasks/lstm_dr/locomotion_log_isaac_ep0[0-4][0-9]_*.csv'))

# Calculate base drift
for f in files:
    df = pd.read_csv(f)
    drift = (df['base_pos_x'].std()**2 + df['base_pos_y'].std()**2)**0.5
    print(f"Episode {df['episode_id'].iloc[0]}: drift = {drift:.4f} m")
```

---

## ⏱️ Time Estimates

| Phase | Duration |
|-------|----------|
| Generate episodes | 5 seconds |
| Complete Implicit training (parallel) | 11 hours |
| Collect 6 configs × 200 episodes | 7 hours |
| Validate & analyze | 1 hour |
| **TOTAL** | **~19 hours** |

---

## 🎯 Advantages of Task-Based Testing

✅ **Reproducible:** Fixed command sequences, not random
✅ **Interpretable:** Clear task objectives (stand, walk, turn, combined)
✅ **Comprehensive:** Tests key locomotion behaviors
✅ **Comparable:** Same tasks across all configs
✅ **Publication-ready:** Standard benchmarking tasks

---

**Ready to start?** 🚀

```bash
# Step 1: Generate task-based episodes
python episode_config_generator_tasks.py --output episode_configs_tasks.yaml

# Step 2: Complete training (if needed)
./continue_implicit_dr_training.sh      # Terminal 1
./continue_implicit_no_dr_training.sh   # Terminal 2

# Step 3: Collect data
./collect_all_data.sh
```

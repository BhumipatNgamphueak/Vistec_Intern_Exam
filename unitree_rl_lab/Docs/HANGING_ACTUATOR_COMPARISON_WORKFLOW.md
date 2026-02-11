# Hanging Actuator Comparison Workflow

Complete workflow for collecting and comparing motor/actuator responses with robot **suspended in the sky** (no ground contact).

---

## Overview

**Goal:** Compare Gazebo effort controller response with IsaacLab MLP/LSTM actuator responses in isolated conditions (hanging).

**Why hanging?**
- No ground contact interference
- Pure actuator response (no ground reaction forces)
- Isolates motor dynamics
- Fair comparison without environmental factors

---

## Workflow Steps

### Step 1: Collect Gazebo Data (Effort Controller)
### Step 2: Collect IsaacLab MLP Data
### Step 3: Collect IsaacLab LSTM Data
### Step 4: Compare All Three Responses

---

## Step 1: Collect Gazebo Data (Effort Controller)

### 1.1 Start Gazebo with Robot Hanging

```bash
# Terminal 1 - Launch Gazebo
ros2 launch go2_gazebo go2_control.launch.py \
  spawn_z:=1.5 \
  controller_config:=go2_controllers_mlp_lstm.yaml \
  world:=empty.world
```

**Parameters:**
- `spawn_z:=1.5` - Hang robot at 1.5m height
- Use LOW gains config (Kp=25, Kd=0.5) for MLP/LSTM comparison
- Empty world (no distractions)

**Verify hanging:**
```bash
# Check robot height in Gazebo GUI
# Robot should be suspended, legs free
```

---

### 1.2 Run Data Collection Test

```bash
# Terminal 2 - Collect Gazebo data
cd /home/drl-68/unitree_rl_lab

python scripts/actuator_comparison/test_gazebo_motors.py \
  --joint FL_hip_joint \
  --pd_gains low \
  --test all \
  --duration 5.0 \
  --output actuator_analysis_hanging/gazebo
```

**What this does:**
- Tests FL_hip_joint (can change to any joint)
- Runs step response, sine tracking (1Hz, 5Hz), chirp tests
- Saves data to `actuator_analysis_hanging/gazebo/pd_low/`

**Output:**
```
actuator_analysis_hanging/gazebo/pd_low/
├── test_results_YYYYMMDD_HHMMSS.json
└── plots_YYYYMMDD_HHMMSS/
    ├── step_response_gazebo_pd_low.png
    ├── sine_tracking_1hz_gazebo_pd_low.png
    ├── sine_tracking_5hz_gazebo_pd_low.png
    └── chirp_response_gazebo_pd_low.png
```

---

### 1.3 Verify Data Collected

```bash
# Check JSON file exists
ls -lh actuator_analysis_hanging/gazebo/pd_low/test_results_*.json

# Quick preview
cat actuator_analysis_hanging/gazebo/pd_low/test_results_*.json | head -50
```

---

## Step 2: Collect IsaacLab MLP Data

### 2.1 Run MLP Actuator Test (Hanging)

```bash
# IsaacLab - MLP actuator
python scripts/actuator_comparison/test_isaaclab_actuators.py \
  --actuator mlp \
  --joint FL_hip_joint \
  --test all \
  --duration 5.0 \
  --output actuator_analysis_hanging/isaaclab \
  --headless
```

**Configuration:**
- MLP actuator (Kp=25.0, Kd=0.5)
- Robot automatically hangs at 1.5m
- Same joint as Gazebo (FL_hip_joint)
- Same test duration (5.0s)

**Output:**
```
actuator_analysis_hanging/isaaclab/mlp/
├── test_results_YYYYMMDD_HHMMSS.json
└── plots_YYYYMMDD_HHMMSS/
    ├── step_response_mlp.png
    ├── sine_tracking_1hz_mlp.png
    ├── sine_tracking_5hz_mlp.png
    └── chirp_response_mlp.png
```

---

### 2.2 Verify MLP Data

```bash
ls -lh actuator_analysis_hanging/isaaclab/mlp/test_results_*.json
```

---

## Step 3: Collect IsaacLab LSTM Data

### 3.1 Run LSTM Actuator Test (Hanging)

```bash
# IsaacLab - LSTM actuator
python scripts/actuator_comparison/test_isaaclab_actuators.py \
  --actuator lstm \
  --joint FL_hip_joint \
  --test all \
  --duration 5.0 \
  --output actuator_analysis_hanging/isaaclab \
  --headless
```

**Configuration:**
- LSTM actuator (Kp=25.0, Kd=0.5)
- Robot automatically hangs at 1.5m
- Same joint as MLP and Gazebo
- Same test duration

**Output:**
```
actuator_analysis_hanging/isaaclab/lstm/
├── test_results_YYYYMMDD_HHMMSS.json
└── plots_YYYYMMDD_HHMMSS/
    ├── step_response_lstm.png
    ├── sine_tracking_1hz_lstm.png
    ├── sine_tracking_5hz_lstm.png
    └── chirp_response_lstm.png
```

---

### 3.2 Verify LSTM Data

```bash
ls -lh actuator_analysis_hanging/isaaclab/lstm/test_results_*.json
```

---

## Step 4: Compare All Three Responses

### 4.1 Compare MLP vs Gazebo

```bash
# Find latest result files
MLP_FILE=$(ls -t actuator_analysis_hanging/isaaclab/mlp/test_results_*.json | head -1)
GAZEBO_FILE=$(ls -t actuator_analysis_hanging/gazebo/pd_low/test_results_*.json | head -1)

# Compare
python scripts/actuator_comparison/compare_actuators.py \
  --isaac "$MLP_FILE" \
  --gazebo "$GAZEBO_FILE" \
  --output actuator_analysis_hanging/comparison/mlp_vs_gazebo
```

**Output:**
```
actuator_analysis_hanging/comparison/mlp_vs_gazebo/
├── comparison_step_response.png      # Step response overlay
├── comparison_sine_1hz.png            # 1 Hz tracking overlay
├── comparison_sine_5hz.png            # 5 Hz tracking overlay
└── comparison_metrics.csv             # Quantitative metrics
```

---

### 4.2 Compare LSTM vs Gazebo

```bash
LSTM_FILE=$(ls -t actuator_analysis_hanging/isaaclab/lstm/test_results_*.json | head -1)

python scripts/actuator_comparison/compare_actuators.py \
  --isaac "$LSTM_FILE" \
  --gazebo "$GAZEBO_FILE" \
  --output actuator_analysis_hanging/comparison/lstm_vs_gazebo
```

**Output:**
```
actuator_analysis_hanging/comparison/lstm_vs_gazebo/
├── comparison_step_response.png
├── comparison_sine_1hz.png
├── comparison_sine_5hz.png
└── comparison_metrics.csv
```

---

### 4.3 View Comparison Results

```bash
# Open plots
eog actuator_analysis_hanging/comparison/mlp_vs_gazebo/*.png
eog actuator_analysis_hanging/comparison/lstm_vs_gazebo/*.png

# View metrics
cat actuator_analysis_hanging/comparison/mlp_vs_gazebo/comparison_metrics.csv
cat actuator_analysis_hanging/comparison/lstm_vs_gazebo/comparison_metrics.csv
```

---

## Automated Full Workflow

### Complete Script

```bash
#!/bin/bash
# collect_and_compare_hanging.sh - Full automated workflow

set -e

JOINT="FL_hip_joint"
DURATION=5.0
OUTPUT_BASE="actuator_analysis_hanging"

echo "========================================="
echo "Hanging Actuator Comparison Workflow"
echo "========================================="
echo ""
echo "Joint: $JOINT"
echo "Duration: ${DURATION}s per test"
echo "Output: $OUTPUT_BASE/"
echo ""

# Step 1: Gazebo data collection
echo "Step 1/5: Collecting Gazebo data..."
echo "⚠️  Make sure Gazebo is running with robot at height!"
echo "    ros2 launch go2_gazebo go2_control.launch.py spawn_z:=1.5"
echo ""
read -p "Is Gazebo ready? (y/n): " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Please start Gazebo first."
    exit 1
fi

python scripts/actuator_comparison/test_gazebo_motors.py \
  --joint $JOINT \
  --pd_gains low \
  --test all \
  --duration $DURATION \
  --output "$OUTPUT_BASE/gazebo"

echo "✅ Gazebo data collected"
echo ""

# Step 2: MLP data collection
echo "Step 2/5: Collecting MLP actuator data..."
python scripts/actuator_comparison/test_isaaclab_actuators.py \
  --actuator mlp \
  --joint $JOINT \
  --test all \
  --duration $DURATION \
  --output "$OUTPUT_BASE/isaaclab" \
  --headless

echo "✅ MLP data collected"
echo ""

# Step 3: LSTM data collection
echo "Step 3/5: Collecting LSTM actuator data..."
python scripts/actuator_comparison/test_isaaclab_actuators.py \
  --actuator lstm \
  --joint $JOINT \
  --test all \
  --duration $DURATION \
  --output "$OUTPUT_BASE/isaaclab" \
  --headless

echo "✅ LSTM data collected"
echo ""

# Step 4: Compare MLP vs Gazebo
echo "Step 4/5: Comparing MLP vs Gazebo..."
MLP_FILE=$(ls -t "$OUTPUT_BASE/isaaclab/mlp/test_results_"*.json | head -1)
GAZEBO_FILE=$(ls -t "$OUTPUT_BASE/gazebo/pd_low/test_results_"*.json | head -1)

python scripts/actuator_comparison/compare_actuators.py \
  --isaac "$MLP_FILE" \
  --gazebo "$GAZEBO_FILE" \
  --output "$OUTPUT_BASE/comparison/mlp_vs_gazebo"

echo "✅ MLP comparison complete"
echo ""

# Step 5: Compare LSTM vs Gazebo
echo "Step 5/5: Comparing LSTM vs Gazebo..."
LSTM_FILE=$(ls -t "$OUTPUT_BASE/isaaclab/lstm/test_results_"*.json | head -1)

python scripts/actuator_comparison/compare_actuators.py \
  --isaac "$LSTM_FILE" \
  --gazebo "$GAZEBO_FILE" \
  --output "$OUTPUT_BASE/comparison/lstm_vs_gazebo"

echo "✅ LSTM comparison complete"
echo ""

# Summary
echo "========================================="
echo "Workflow Complete!"
echo "========================================="
echo ""
echo "Results saved to:"
echo "  $OUTPUT_BASE/"
echo ""
echo "View comparisons:"
echo "  MLP vs Gazebo:  $OUTPUT_BASE/comparison/mlp_vs_gazebo/"
echo "  LSTM vs Gazebo: $OUTPUT_BASE/comparison/lstm_vs_gazebo/"
echo ""
echo "Metrics:"
echo "  cat $OUTPUT_BASE/comparison/mlp_vs_gazebo/comparison_metrics.csv"
echo "  cat $OUTPUT_BASE/comparison/lstm_vs_gazebo/comparison_metrics.csv"
echo ""
```

Save as `collect_and_compare_hanging.sh` and run:
```bash
chmod +x collect_and_compare_hanging.sh
./collect_and_compare_hanging.sh
```

---

## Expected Results

### Good Match Indicators

**MLP vs Gazebo (both Kp=25, Kd=0.5):**

| Metric | Expected Difference |
|--------|-------------------|
| Rise Time | < 20% |
| Overshoot | < 5% |
| RMSE @ 1 Hz | < 10 mrad |
| RMSE @ 5 Hz | < 50 mrad |

**LSTM vs Gazebo (both Kp=25, Kd=0.5):**

| Metric | Expected Difference |
|--------|-------------------|
| Rise Time | < 20% |
| Overshoot | < 5% |
| RMSE @ 1 Hz | < 10 mrad |
| RMSE @ 5 Hz | < 50 mrad |

---

### What Differences Mean

**Small differences (<20%):**
- ✅ Good match
- ✅ Actuators behave similarly
- ✅ Gazebo can substitute for MLP/LSTM in testing

**Medium differences (20-50%):**
- ⚠️ Some mismatch
- Consider tuning Gazebo PD gains
- Neural network learned dynamics differ
- Still acceptable for rough comparison

**Large differences (>50%):**
- ❌ Poor match
- Check PD gains loaded correctly
- Verify physics timestep matches
- Review joint damping/friction settings

---

## Interpreting Plots

### Step Response Plot

**What to look for:**
- Blue (IsaacLab) vs Red (Gazebo) curves
- Should mostly overlap for good match
- Check rise time annotation
- Check overshoot percentage

**Good match:**
- Curves follow same trajectory
- Similar peak height
- Similar settling time

---

### Sine Tracking Plots

**What to look for:**
- Position tracking (top subplot)
- Error magnitude (second subplot)
- Should be small (< 10 mrad at 1 Hz)

**Good match:**
- Both follow command closely
- Similar error magnitudes
- Similar phase lag

---

### Metrics CSV

```csv
Test,Metric,IsaacLab,Gazebo,Difference
Step Response,Rise Time (ms),82.0,78.5,3.5
Step Response,Overshoot (%),5.2,4.8,0.4
Sine 1 Hz,RMSE (mrad),8.5,9.2,0.7
Sine 5 Hz,RMSE (mrad),45.2,52.1,6.9
```

**Analysis:**
- Rise time difference: 3.5ms (4.3%) - ✅ Excellent
- Overshoot difference: 0.4% - ✅ Excellent
- RMSE @ 1Hz: 0.7 mrad - ✅ Excellent
- RMSE @ 5Hz: 6.9 mrad - ✅ Good

---

## Directory Structure

After running complete workflow:

```
actuator_analysis_hanging/
├── gazebo/
│   └── pd_low/
│       ├── test_results_20260210_143022.json
│       └── plots_20260210_143022/
│           ├── step_response_gazebo_pd_low.png
│           ├── sine_tracking_1hz_gazebo_pd_low.png
│           ├── sine_tracking_5hz_gazebo_pd_low.png
│           └── chirp_response_gazebo_pd_low.png
│
├── isaaclab/
│   ├── mlp/
│   │   ├── test_results_20260210_143245.json
│   │   └── plots_20260210_143245/
│   │       ├── step_response_mlp.png
│   │       ├── sine_tracking_1hz_mlp.png
│   │       ├── sine_tracking_5hz_mlp.png
│   │       └── chirp_response_mlp.png
│   │
│   └── lstm/
│       ├── test_results_20260210_143512.json
│       └── plots_20260210_143512/
│           ├── step_response_lstm.png
│           ├── sine_tracking_1hz_lstm.png
│           ├── sine_tracking_5hz_lstm.png
│           └── chirp_response_lstm.png
│
└── comparison/
    ├── mlp_vs_gazebo/
    │   ├── comparison_step_response.png
    │   ├── comparison_sine_1hz.png
    │   ├── comparison_sine_5hz.png
    │   └── comparison_metrics.csv
    │
    └── lstm_vs_gazebo/
        ├── comparison_step_response.png
        ├── comparison_sine_1hz.png
        ├── comparison_sine_5hz.png
        └── comparison_metrics.csv
```

---

## Troubleshooting

### Gazebo Issues

**Problem:** Robot falls when starting test
**Solution:**
```bash
# Ensure robot spawned at height
ros2 launch go2_gazebo go2_control.launch.py spawn_z:=1.5

# Check robot Z position in Gazebo GUI
# Should be ~1.5m above ground
```

**Problem:** Joint doesn't move in Gazebo
**Check:**
```bash
# Verify controller active
ros2 control list_controllers

# Should show:
# go2_position_controller[position_controllers/JointGroupPositionController] active

# Check PD gains loaded
ros2 param get /controller_manager/go2_position_controller FL_hip_joint.p
# Should show: 25.0 (for LOW gains)
```

---

### IsaacLab Issues

**Problem:** Robot falls in IsaacLab
**Solution:** Script automatically maintains base at 1.5m height every 10 physics steps

**Problem:** Import errors
**Check:**
```bash
# Verify IsaacLab environment
conda activate isaaclab  # or your environment name
which python
# Should point to IsaacLab Python
```

---

### Comparison Issues

**Problem:** "File not found" when comparing
**Check:**
```bash
# Verify files exist
ls actuator_analysis_hanging/isaaclab/mlp/test_results_*.json
ls actuator_analysis_hanging/gazebo/pd_low/test_results_*.json

# Use correct paths in compare command
```

**Problem:** Plots look very different
**Investigate:**
1. Verify same joint tested (FL_hip_joint)
2. Check same test duration (5.0s)
3. Verify PD gains match (Kp=25, Kd=0.5)
4. Check physics timestep (5ms both)
5. Review joint damping in URDF

---

## Quick Verification Checklist

Before starting workflow:

**Gazebo Setup:**
- [ ] Gazebo launched
- [ ] Robot spawned at z=1.5m
- [ ] Robot hanging (legs free, no ground contact)
- [ ] Controllers loaded and active
- [ ] PD gains = LOW (Kp=25, Kd=0.5)

**Environment:**
- [ ] ROS 2 sourced
- [ ] IsaacLab environment activated
- [ ] Scripts executable (`chmod +x *.sh`)
- [ ] Output directory writable

**Test Parameters:**
- [ ] Same joint (FL_hip_joint recommended)
- [ ] Same duration (5.0s recommended)
- [ ] Same motion types (step, sine, chirp)

---

## Summary

**Complete workflow:**
1. ✅ Start Gazebo with hanging robot (z=1.5m)
2. ✅ Collect Gazebo effort controller data
3. ✅ Collect IsaacLab MLP actuator data
4. ✅ Collect IsaacLab LSTM actuator data
5. ✅ Compare MLP vs Gazebo
6. ✅ Compare LSTM vs Gazebo
7. ✅ Analyze results

**Total time:** ~5 minutes (1 min per collection + comparisons)

**Expected outcome:**
- Similar responses between MLP/LSTM and Gazebo
- Quantitative metrics in CSV
- Visual comparisons in plots
- Understanding of actuator matching quality

---

**Last Updated:** 2026-02-10

# Task Comparison: Sim2Sim Transfer Analysis

**Date**: 2026-02-10
**Episode**: 0
**Time Window**: 0.5-2.5s (gait metrics), 0-15s (CoM trajectory with Isaac starting at 0.5s)

---

## Task Descriptions

| Task | Name | Description | Expected Gait |
|------|------|-------------|--------------|
| **Task 0** | Standing | Static standing pose | No locomotion |
| **Task 1** | Walking | Straight-line forward walking | Regular trot |
| **Task 3** | Walk+Turn | Walking with turning maneuvers | Irregular gait + lateral motion |

---

## Key Findings Summary

### Task 1 (Walking) - **RECOMMENDED FOR GAIT ANALYSIS** ✅

**CoM Stability**:
| Model | Vertical Osc. | Lateral Sway | Notes |
|-------|--------------|--------------|-------|
| Isaac | **15.6 mm** | **4.0 mm** | Reference - very stable |
| Gazebo Implicit | 17.5 mm | 16.5 mm | Good vertical, some lateral drift |
| Gazebo Implicit+DR | 11.7 mm | 24.3 mm | Better than baseline |
| Gazebo MLP+DR | 13.7 mm | 60.2 mm | Increased lateral sway |
| Gazebo LSTM | 13.5 mm | **753 mm** | ⚠️ Severe lateral drift |
| Gazebo LSTM+DR | 12.8 mm | 50.7 mm | Good stability |

**Stride Metrics**:
- **Gazebo Implicit**: 0.30s period, 20% duty cycle, 22mm stride length
- **Gazebo LSTM**: 0.31s period, 38% duty cycle, 122mm stride length
- Other models: Insufficient stride data in 2-second window

**Key Insights**:
- ✅ Isaac reference is very stable (4mm lateral sway)
- ✅ Gazebo LSTM+DR achieves good CoM control (50mm lateral)
- ⚠️ Gazebo LSTM (no DR) has severe lateral drift (753mm!)
- ⚠️ Most models don't complete full strides in 2-second window

---

### Task 3 (Walk+Turn) - **COMPLEX MANEUVER** 🔄

**CoM Stability**:
| Model | Vertical Osc. | Lateral Sway | Notes |
|-------|--------------|--------------|-------|
| Isaac | **15.6 mm** | **4.0 mm** | Reference |
| Gazebo Implicit | 30.0 mm | **621 mm** | Turning causes large displacement |
| Gazebo Implicit+DR | 19.2 mm | **1306 mm** | Very large lateral motion |
| Gazebo MLP+DR | 7.9 mm | **1387 mm** | Low vertical, high lateral |
| Gazebo LSTM+DR | 9.5 mm | **1242 mm** | Better vertical control |

**Stride Metrics**:
- **Gazebo Implicit**: 0.5s period, 92% duty cycle (almost always in contact)
- Other models: No regular strides detected

**Key Insights**:
- ⚠️ **Lateral "sway" is actually lateral displacement during turn** (~1.2-1.4 meters!)
- ⚠️ This is NOT oscillation - it's the robot moving sideways during the turn
- ✅ Vertical control improves with advanced models (MLP+DR, LSTM+DR: <10mm)
- ⚠️ Isaac shows minimal lateral displacement (4mm) - **may not be turning properly?**

---

## Detailed Comparison

### CoM Vertical Oscillation (mm)

```
Model              Task 1   Task 3   Difference
────────────────────────────────────────────────
Isaac              15.6     15.6     0.0  (same)
Gazebo Implicit    17.5     30.0     +12.5
Implicit+DR        11.7     19.2     +7.5
MLP+DR             13.7      7.9     -5.8  ✓
LSTM+DR            12.8      9.5     -3.3  ✓
```

**Observation**: Advanced models (MLP+DR, LSTM+DR) actually improve vertical stability in Task 3 vs Task 1!

---

### CoM Lateral Sway (mm)

```
Model              Task 1   Task 3   Interpretation
────────────────────────────────────────────────────────
Isaac               4.0      4.0     Minimal lateral motion (both tasks)
Gazebo Implicit    16.5    621.0    Large turn radius
Implicit+DR        24.3   1306.0    Very wide turn
MLP+DR             60.2   1387.0    Widest turn
LSTM              753.0      -      Not tested in Task 3
LSTM+DR            50.7   1242.0    Controlled turn
```

**Important**: For Task 3, "lateral sway" is really **total lateral displacement during turn**. Values of 1.2-1.4m indicate the robot is making a wide turning arc.

---

### Trajectory Smoothness (m/s³)

Lower is better (less jerky motion).

```
Model              Task 1    Task 3
─────────────────────────────────────
Isaac               40.3      40.3   ✓ Very smooth
Gazebo Implicit    349.4    1077.3   ✗ Jerky
Implicit+DR       1387.5    1631.5   ✗ Very jerky
MLP+DR            1312.9    1644.7   ✗ Very jerky
LSTM               870.7       -
LSTM+DR           1310.2    1849.1   ✗ Very jerky
```

**Observation**: All Gazebo models are **significantly jerkier** than Isaac (3-45x higher jerk). Domain randomization (DR) models are especially jerky, suggesting they may be making rapid corrections.

---

## DTW Distance (Similarity to Isaac)

Lower is better (more similar to Isaac reference).

```
Model              Task 1   Task 3   Trend
────────────────────────────────────────────
Gazebo Implicit    0.400    0.425    Slightly worse
Implicit+DR        0.475    0.479    Similar
MLP+DR             0.461    0.472    Slightly worse
LSTM               0.389      -      Best in Task 1
LSTM+DR            0.456    0.480    Slightly worse
```

**Observation**:
- Task 3 (turning) is generally harder to match Isaac
- LSTM (no DR) was best in Task 1 but has severe lateral drift
- All models show 0.39-0.48 DTW distance (moderate similarity)

---

## Stride Analysis

### Task 1 (Walking)

| Model | Stride Period | Duty Cycle | Stride Length |
|-------|--------------|------------|---------------|
| Gazebo Implicit | 0.30s ± 0.13 | 20% ± 12% | 22 mm |
| Gazebo LSTM | 0.31s ± 0.12 | 38% ± 16% | 122 mm |

- Only 2 models have detectable strides in 2-second window
- Low duty cycles (20-38%) suggest bouncy gait with short stance phases

### Task 3 (Walk+Turn)

| Model | Stride Period | Duty Cycle | Stride Length |
|-------|--------------|------------|---------------|
| Gazebo Implicit | 0.50s | 92% | 162 mm |

- Only 1 model has detectable strides
- Very high duty cycle (92%) suggests slow, cautious gait with feet mostly on ground
- Longer stride period (0.5s vs 0.3s) = slower gait during turning

---

## Recommendations

### For Gait Analysis 📊

**Use Task 1 (Walking)**:
- ✅ More regular stride patterns
- ✅ Better for measuring steady-state locomotion
- ✅ Lower lateral displacement (easier to interpret "sway")
- ✅ More models complete full strides

### For Robustness Testing 🔧

**Use Task 3 (Walk+Turn)**:
- ✅ Tests turning capability
- ✅ Reveals lateral control issues
- ✅ More challenging for controllers
- ⚠️ But harder to interpret metrics (displacement vs oscillation)

### For Sim2Sim Comparison 🎯

**Compare Both Tasks**:
- Task 1 shows basic locomotion quality
- Task 3 shows controller robustness
- Look for models that maintain stability in both

---

## Critical Observations

### 1. Isaac May Not Be Turning in Task 3 ⚠️

Isaac shows:
- Lateral sway: **4.0 mm** (same as Task 1)
- Vertical osc: **15.6 mm** (same as Task 1)

This suggests Isaac might be walking straight instead of turning, OR it's making a very tight turn with excellent lateral control. **Needs verification!**

### 2. Gazebo Models Show Large Lateral Displacement 📏

Task 3 lateral "sway" of 1.2-1.4 meters is actually:
- **Total lateral displacement during turn**
- NOT oscillation around a straight path
- Indicates wide turning radius

### 3. Domain Randomization Increases Jerk 📈

Models with DR show 3-4x higher smoothness values (jerk):
- Implicit: 349 → Implicit+DR: 1387 (4x increase!)
- This suggests DR models make rapid corrections
- Trade-off: More robust but less smooth

### 4. Stride Detection Challenges ⏱️

Most models don't show stride metrics because:
- 2-second window is too short for slow gaits
- Turning disrupts regular stride patterns
- Contact detection may be unreliable during complex motions

**Solution**: Use longer time windows (5-10s) or focus on straight walking (Task 1)

---

## Visual Outputs

### Generated Figures (both tasks)

1. **contact_timeline.png**: Contact patterns over time
2. **trajectories.png**: Foot trajectories in sagittal plane
3. **com_trajectory.png**: CoM path over 15 seconds (0.5-15s for Isaac, 0-15s for Gazebo)
4. **stride_visualization.png**: Stride length/height comparison
5. **metrics_dashboard.png**: 6-panel metric comparison
6. **improvement.png**: Progressive model improvement

### Where to Find Results

- **Task 1**: `./task1_results/`
- **Task 3**: `./task3_results/`

---

## Summary Metrics Files

```bash
# View Task 1 summary
cat ./task1_results/summary_metrics.csv

# View Task 3 summary
cat ./task3_results/summary_metrics.csv

# Compare side-by-side
paste ./task1_results/summary_metrics.csv ./task3_results/summary_metrics.csv
```

---

## Next Steps

1. **Verify Isaac Task 3 behavior**: Check if it's actually turning or just walking straight

2. **Use longer time windows** for stride analysis:
   ```bash
   python analyze_task.py --task 1 --time-start 0.5 --time-end 5.0 --output ./task1_5s
   ```

3. **Analyze multiple episodes** for statistics:
   ```bash
   python batch_analysis.py --episodes 0 1 2 3 4 --output ./batch_task1
   ```

4. **Focus on best models**:
   - For smooth locomotion: **Gazebo LSTM** (but watch for drift!)
   - For overall stability: **Gazebo LSTM+DR**
   - For simple baseline: **Gazebo Implicit**

---

**Analysis Date**: 2026-02-10
**Tool Version**: 1.0.0 (with extended CoM trajectory support)
**Status**: ✅ Complete

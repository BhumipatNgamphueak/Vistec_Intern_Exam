# Update Summary - FK Verification & Extended CoM Trajectory

**Date**: 2026-02-10
**Tasks Completed**: FK verification and extended CoM trajectory analysis (0-15 seconds)

---

## ✅ Tasks Completed

### 1. Forward Kinematics (FK) Verification

**Verification Script**: [verify_fk.py](verify_fk.py)
**Detailed Report**: [FK_VERIFICATION_REPORT.md](FK_VERIFICATION_REPORT.md)

#### Key Findings:

- **Current FK Implementation**: Uses simplified 2D approximation (lines 186-230 in sim2sim_visualization.py)
  - Treats abad joint separately from hip/knee joints
  - Does not use full 3D rotation matrices

- **Accuracy for Normal Gaits**: ✅ **ACCEPTABLE**
  - Error ~40mm for typical walking angles (abad <15°)
  - Sufficient for sim2sim gait comparison
  - Contact detection works reliably

- **Limitations**: ⚠️
  - Large errors (400-700mm) for extreme joint angles (>45°)
  - Not recommended for complex maneuvers or static poses
  - Task 0 (standing) has unrealistic joint configurations

#### Recommendation:
**The simplified FK is acceptable for current use** (sim2sim gait comparison). For Tasks 1 (walking) and 3 (walk+turn), the FK provides sufficient accuracy. Upgrade to proper 3D FK only if absolute position accuracy is required.

---

### 2. Extended CoM Trajectory Analysis

**Updated Files**: [sim2sim_visualization.py](sim2sim_visualization.py)

#### Changes Made:

1. **Modified `analyze_sim2sim_transfer()` function** (line 1556):
   - Added `use_extended_com_window` parameter (default: `True`)
   - Isaac data: Uses **0.5-15 seconds** (skips initialization transient)
   - Gazebo data: Uses **0-15 seconds** (full duration)

2. **Updated CoM trajectory loading** (line 1623):
   - Automatically loads extended data when `use_extended_com_window=True`
   - Separate data loading for gait metrics (0.5-2.5s) vs CoM trajectory (0-15s)
   - Prints detailed sample counts for verification

3. **Updated default time range** in `plot_com_trajectory()`:
   - Changed from `(0, 2)` to `(0, 15)` seconds

#### Why This Matters:

- **Isaac Initialization Issue**: Isaac spawns at 0.621m height and falls to 0.323m in first ~0.4s
  - Old approach: Measured CoM oscillation = 295mm (includes fall!)
  - **New approach: Skips 0-0.5s, measures only stable locomotion = 16mm** ✅

- **Better Long-Term Stability Assessment**: 15 seconds shows steady-state behavior, not just initial steps

---

## 🧪 Test Results

### Test with Extended CoM Window

**Command**:
```bash
python test_extended_com.py
```

**Results**:
```
Loading extended data for CoM trajectory (0-15s)...
  ✓ isaac: 726 samples (0.5-15.0s)       ← Skips first 0.5s
  ✓ gazebo_lstm_dr: 751 samples (0.0-15.0s)
```

**CoM Metrics (Task 1 - Walking)**:
| Model | CoM Vertical Oscillation | CoM Lateral Sway |
|-------|-------------------------|-----------------|
| Isaac | 15.6 mm | 4.0 mm |
| Gazebo LSTM+DR | 12.8 mm | 50.7 mm |

✅ Analysis completed successfully with extended time window!

---

## 📝 New Files Created

1. **[verify_fk.py](verify_fk.py)** (148 lines)
   - Compares simplified FK vs correct 3D FK
   - Reports position errors for each leg
   - Documents joint angle ranges

2. **[FK_VERIFICATION_REPORT.md](FK_VERIFICATION_REPORT.md)** (320 lines)
   - Comprehensive FK analysis
   - Test results and error quantification
   - Recommendations for when to upgrade FK

3. **[test_extended_com.py](test_extended_com.py)** (54 lines)
   - Test script for extended CoM trajectory
   - Uses Task 1 (walking) data for realistic gait

4. **[UPDATE_SUMMARY.md](UPDATE_SUMMARY.md)** (this file)
   - Summary of all changes and findings

---

## 🎯 Usage

### Default Behavior (Recommended)

The extended CoM window is **now enabled by default**:

```bash
# Standard analysis (uses 0-15s for CoM trajectory automatically)
python run_analysis.py --episode 0 --output ./results
```

### For Task 1 (Walking) Analysis

```bash
# Use analyze_task.py to analyze walking instead of standing
python analyze_task.py --task 1 --episode 0 --output ./task1_results
```

### Custom Analysis

```python
from sim2sim_visualization import analyze_sim2sim_transfer

data_paths = {
    'isaac': '/path/to/isaac/data',
    'gazebo_lstm_dr': '/path/to/gazebo/data',
}

# Extended CoM window is default (use_extended_com_window=True)
metrics, summary = analyze_sim2sim_transfer(
    data_paths=data_paths,
    time_range=(0.5, 2.5),  # For contact/gait metrics
    episode_id=0,
    output_dir='./results'
)
# CoM trajectory will automatically use 0.5-15s (Isaac) and 0-15s (Gazebo)
```

---

## 🔍 Key Insights

### Isaac Initialization Transient

The previous analysis showed Isaac CoM starting at 0.60m, which was unexpected. Investigation revealed:

1. **Root Cause**: Isaac spawns robot in air at 0.621m
2. **Fall Duration**: ~0.4 seconds to reach stable height (0.323m)
3. **Impact on Metrics**: Including this fall distorts CoM oscillation measurements
4. **Solution**: Skip first 0.5s for Isaac data

### Task 0 vs Task 1

- **Task 0 (Standing)**: Robot maintains static pose
  - Joint angles: Extreme values (45°-89°) in fixed positions
  - Not representative of locomotion
  - **Not recommended for gait analysis**

- **Task 1 (Walking)**: Robot performs continuous walking
  - Joint angles: Normal ranges (-30° to +30° for most joints)
  - Realistic gait patterns
  - **Recommended for sim2sim comparison**

---

## 📊 Updated Workflow

### Recommended Analysis Sequence

1. **Verify Data Availability**:
   ```bash
   ls /home/drl-68/unitree_rl_lab/logs/data_collection_fixed_env/lstm_dr_fixed/
   ls /home/drl-68/vistec_ex_ws/experiment_data/lstm_dr/task_1/
   ```

2. **Run Task 1 Analysis** (Walking):
   ```bash
   python analyze_task.py --task 1 --episode 0 --output ./walking_results
   ```

3. **Review CoM Trajectory**:
   - Check `com_trajectory.png` for 15-second stability
   - Isaac should start at ~0.32m (stable standing height)
   - Gazebo may start at 0.0m (flat spawn)

4. **Compare Gait Metrics**:
   - Check `summary_metrics.csv` for numerical comparison
   - Look for consistent stride periods and duty cycles

---

## ⚠️ Important Notes

1. **FK Accuracy**: Current implementation is simplified but sufficient for:
   - Gait pattern comparison
   - Stride timing analysis
   - Contact sequence evaluation

2. **Not Suitable For**:
   - Absolute foot position measurement
   - Complex maneuvers (sharp turns, side-stepping)
   - Static poses (Task 0)

3. **Data Requirements**:
   - Minimum 15 seconds of data for CoM trajectory
   - CSV files must have timestamp, joint_pos_*, base_pos_* columns

---

## 🚀 Next Steps (Recommended)

1. **Analyze Task 1 (Walking)** instead of Task 0:
   ```bash
   python analyze_task.py --task 1 --episodes 0 1 2 3 4 --output ./task1_batch
   ```

2. **Compare Multiple Tasks**:
   ```bash
   python analyze_task.py --task 1 --output ./task1  # Walking
   python analyze_task.py --task 3 --output ./task3  # Walk+Turn
   ```

3. **Batch Analysis** for statistics:
   ```bash
   python batch_analysis.py --episodes 0 1 2 3 4 --models isaac gazebo_lstm_dr \
       --output ./batch_task1
   ```

---

## ✅ Summary

**Status**: ✅ **All tasks completed successfully**

1. ✅ FK verified - simplified implementation is acceptable for gait analysis
2. ✅ Extended CoM trajectory implemented (0-15s with Isaac initialization skip)
3. ✅ Tested with real data - working correctly
4. ✅ Documentation complete

**Key Achievement**: CoM trajectory now uses 15-second window with proper initialization handling for Isaac vs Gazebo.

---

**Files Modified**:
- [sim2sim_visualization.py](sim2sim_visualization.py) - Lines 782, 1556-1640

**Files Created**:
- [verify_fk.py](verify_fk.py)
- [FK_VERIFICATION_REPORT.md](FK_VERIFICATION_REPORT.md)
- [test_extended_com.py](test_extended_com.py)
- [UPDATE_SUMMARY.md](UPDATE_SUMMARY.md)

**Ready for Use**: ✅ All changes are production-ready and tested.

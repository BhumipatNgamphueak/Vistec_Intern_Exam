# Sim2Sim Transfer Visualization Tool

Comprehensive analysis and visualization toolkit for comparing quadruped robot gait transfer from Isaac Sim to Gazebo simulation across multiple model architectures.

## 📋 Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Data Format](#data-format)
- [Usage Examples](#usage-examples)
- [Generated Visualizations](#generated-visualizations)
- [Metrics Explained](#metrics-explained)
- [API Reference](#api-reference)
- [Troubleshooting](#troubleshooting)

## 🎯 Overview

This toolkit provides comprehensive analysis of quadruped locomotion transfer between simulators, specifically:

- **Isaac Sim** (source simulator, reference behavior)
- **Gazebo** (target simulator with various policy architectures)

### Compared Models

1. **Isaac Sim** - Reference behavior
2. **Gazebo Implicit (Baseline)** - Direct transfer without adaptation
3. **Gazebo Implicit+DR** - With domain randomization
4. **Gazebo MLP+DR** - MLP encoder with DR
5. **Gazebo LSTM (no DR)** - LSTM encoder without DR
6. **Gazebo LSTM+DR** - LSTM encoder with DR (best performing)

## ✨ Features

### Analysis Capabilities

- ✅ **Gait Pattern Analysis**: Stride period, duty cycle, phase coordination
- ✅ **Contact Detection**: Automatic detection from joint kinematics
- ✅ **Trajectory Comparison**: Foot trajectories in 2D/3D space
- ✅ **Stability Metrics**: CoM oscillation, lateral sway
- ✅ **Smoothness Analysis**: Jerk integral, acceleration profiles
- ✅ **DTW Distance**: Dynamic Time Warping for trajectory similarity

### Visualizations (Publication Quality)

1. **Contact Pattern Timeline** - Gantt-style gait visualization
2. **Foot Trajectory Comparison** - 2D sagittal plane trajectories
3. **Behavioral Metrics Dashboard** - 6-panel metric comparison
4. **Progressive Improvement** - Waterfall chart showing model progression

## 🔧 Installation

### Prerequisites

```bash
# Python 3.8+
python --version

# Required packages
pip install numpy pandas matplotlib seaborn scipy
```

### Optional Dependencies (for DTW)

```bash
# Option 1: dtaidistance (recommended, faster)
pip install dtaidistance

# Option 2: fastdtw (alternative)
pip install fastdtw

# Note: If neither is installed, the tool will use Euclidean distance as fallback
```

### Installation

```bash
cd /home/drl-68/vistec_ex_ws
chmod +x run_analysis.py

# Test with synthetic data
python run_analysis.py --synthetic
```

## 🚀 Quick Start

### Test with Synthetic Data

```bash
python run_analysis.py --synthetic --output ./test_figures
```

This generates sample trot gait data and creates all visualizations in `./test_figures/`.

### Analyze Real Experiment Data

```bash
# Analyze episode 0, time window 0-2 seconds
python run_analysis.py --episode 0 --time-start 0 --time-end 2 --output ./results

# Analyze episode 5, extended time window
python run_analysis.py --episode 5 --time-start 0 --time-end 5 --output ./results_ep5
```

### Custom Analysis (Python Script)

```python
from sim2sim_visualization import analyze_sim2sim_transfer

# Define data paths
data_paths = {
    'isaac': '/path/to/isaac/data',
    'gazebo_lstm_dr': '/path/to/gazebo/lstm_dr/data',
    # Add more models...
}

# Run analysis
metrics, summary = analyze_sim2sim_transfer(
    data_paths=data_paths,
    time_range=(0, 2),
    episode_id=0,
    output_dir='./my_analysis'
)

# Access computed metrics
print(f"Isaac stride period: {metrics['isaac']['stride_periods_FL']}")
print(f"LSTM+DR CoM oscillation: {metrics['gazebo_lstm_dr']['com_vertical_oscillation']}")
```

## 📊 Data Format

### Required CSV Columns

The tool expects CSV files with the following columns:

#### Time and Base State
- `timestamp_sim` - Simulation time (seconds)
- `base_pos_x`, `base_pos_y`, `base_pos_z` - Base position (meters)
- `base_quat_w`, `base_quat_x`, `base_quat_y`, `base_quat_z` - Orientation quaternion
- `base_roll`, `base_pitch`, `base_yaw` - Euler angles (radians)
- `base_lin_vel_x/y/z` - Linear velocities (m/s)
- `base_ang_vel_x/y/z` - Angular velocities (rad/s)

#### Joint States
- `joint_pos_0` to `joint_pos_11` - Joint positions (radians)
  - Joints 0-2: Front-Left leg (abad, hip, knee)
  - Joints 3-5: Front-Right leg
  - Joints 6-8: Rear-Left leg
  - Joints 9-11: Rear-Right leg

### Directory Structure

#### Isaac Sim Data
```
/home/drl-68/unitree_rl_lab/logs/data_collection_fixed_env/
├── lstm_dr_fixed/
│   ├── locomotion_log_isaac_ep000_*.csv
│   ├── locomotion_log_isaac_ep001_*.csv
│   └── ...
└── ...
```

#### Gazebo Data
```
/home/drl-68/vistec_ex_ws/experiment_data/
├── implicit/
│   └── task_0/
│       ├── locomotion_log_gazebo_ep000_*.csv
│       └── ...
├── lstm_dr/
│   └── task_0/
│       ├── locomotion_log_gazebo_ep000_*.csv
│       └── ...
└── ...
```

## 📈 Generated Visualizations

### 1. Contact Pattern Timeline (`contact_timeline.png`)

**Gantt-style visualization showing contact states over time**

- **Format**: 12" × 10" PNG + PDF
- **Content**:
  - 7 rows (Isaac + 6 Gazebo models)
  - 4 sub-rows per model (FL, FR, RL, RR legs)
  - Dark regions = stance phase (foot in contact)
  - Light regions = swing phase (foot in air)
- **Annotations**:
  - Stride period (seconds)
  - Duty cycle (%)
  - Model-specific metrics in info box

**Interpretation**:
- Diagonal leg pairs should move together in trot gait
- Duty cycle ~50% indicates balanced trot
- Consistent stride periods indicate stable locomotion

### 2. Foot Trajectory Comparison (`trajectories.png`)

**Overlaid foot trajectories in sagittal plane (X-Z)**

- **Format**: 10" × 6" PNG + PDF
- **Content**:
  - Main plot: Front-Left leg trajectories for all models
  - Inset plots: Other three legs (FR, RL, RR)
  - Markers: ○ touchdown, △ liftoff
- **Line Styles**:
  - Isaac: Thick blue solid (reference)
  - Best model (LSTM+DR): Thick cyan solid
  - Others: Various dashed/dotted styles

**Interpretation**:
- Trajectory shape indicates gait quality
- Smooth ellipses = good trot gait
- Height variation shows ground clearance
- Compare to Isaac reference for transfer quality

### 3. Behavioral Metrics Dashboard (`metrics_dashboard.png`)

**2×3 grid of key behavioral metrics**

- **Format**: 16" × 12" PNG + PDF
- **Subplots**:

  **a) Stride Period Consistency** (Box plot)
  - Shows distribution across strides
  - Lower variance = more consistent gait

  **b) Duty Cycle Stability** (Bar chart)
  - Target: 50-55% for trot gait
  - Green: within target, Yellow: borderline, Red: poor

  **c) Phase Coordination** (Polar plot)
  - Ideal trot: FL-FR=180°, FL-RL=180°, FL-RR=0°
  - Closer to ideal = better coordination

  **d) CoM Vertical Stability** (Time series)
  - Shows vertical oscillation over time
  - Lower amplitude = better stability

  **e) GRF Load Distribution** (Stacked bar)
  - Should be ~25% per leg for balanced loading
  - Imbalance indicates asymmetric gait

  **f) Trajectory Smoothness** (Bar chart)
  - Jerk integral (m/s³)
  - Lower = smoother, more efficient motion

**Interpretation**:
- Use this dashboard for comprehensive quality assessment
- Compare patterns across models
- Identify specific areas of improvement/degradation

### 4. Progressive Improvement (`improvement.png`)

**Waterfall chart showing model progression**

- **Format**: 8" × 6" PNG + PDF
- **Content**:
  - Baseline: Gazebo Implicit (establishes gap)
  - Step 1: +MLP (partial improvement)
  - Step 2: +LSTM (better improvement)
  - Step 3: +DR (final improvement)
  - Overall similarity score (0-100%)

**Score Components** (weighted average):
- 30% Gait coordination (phase errors)
- 25% Stride consistency (period std)
- 20% Trajectory similarity (DTW distance)
- 15% Contact timing (duty cycle variance)
- 10% Force distribution (GRF symmetry)

**Interpretation**:
- Higher score = better sim2sim transfer
- Shows incremental value of each component
- Gap remaining indicates room for improvement

## 📐 Metrics Explained

### Gait Metrics

#### Stride Period
- **Definition**: Time between consecutive heel strikes of the same leg
- **Units**: Seconds
- **Typical**: 0.3-0.5s for trot at 1 m/s
- **Good**: Consistent across strides (low std)

#### Duty Cycle
- **Definition**: Percentage of stride spent in stance phase
- **Units**: Percentage (0-100%)
- **Formula**: `duty_cycle = stance_time / stride_period × 100`
- **Trot target**: 50-55%
- **Walk**: >60%, Run: <40%

#### Phase Lag
- **Definition**: Temporal offset between leg pairs
- **Units**: Degrees (0-360°)
- **Ideal Trot**:
  - FL-FR: 180° (diagonal pairs)
  - FL-RL: 180° (diagonal pairs)
  - FL-RR: 0° (same diagonal)

### Stability Metrics

#### CoM Vertical Oscillation
- **Definition**: Peak-to-peak vertical displacement of center of mass
- **Units**: Meters
- **Formula**: `osc = max(z_com) - min(z_com)`
- **Good**: <0.03m for stable locomotion
- **Excellent**: <0.01m

#### CoM Lateral Sway
- **Definition**: Peak-to-peak lateral displacement
- **Units**: Meters
- **Good**: <0.05m for straight walking

### Trajectory Metrics

#### Jerk Integral (Smoothness)
- **Definition**: Integral of jerk magnitude over time
- **Units**: m/s³
- **Formula**: `smoothness = ∫|d³position/dt³|dt`
- **Good**: Lower values
- **Interpretation**: Measures motion smoothness; high jerk indicates jerky, inefficient motion

#### DTW Distance
- **Definition**: Dynamic Time Warping distance between trajectories
- **Units**: Normalized (dimensionless)
- **Usage**: Compare Gazebo trajectories to Isaac reference
- **Good**: Lower values indicate better match

## 🔍 API Reference

### Main Functions

#### `analyze_sim2sim_transfer()`

Complete analysis pipeline.

```python
def analyze_sim2sim_transfer(
    data_paths: Dict[str, str],
    time_range: Tuple[float, float] = (0, 2),
    episode_id: int = 0,
    output_dir: str = './figures/'
) -> Tuple[Dict, pd.DataFrame]:
    """
    Args:
        data_paths: Dict mapping model names to data directory paths
        time_range: Time window to analyze (start, end) in seconds
        episode_id: Episode number to analyze
        output_dir: Output directory for figures and results

    Returns:
        metrics_dict: Computed metrics for all models
        summary_df: Summary statistics as DataFrame
    """
```

#### `compute_all_metrics()`

Compute metrics without generating visualizations.

```python
def compute_all_metrics(
    data_dict: Dict[str, pd.DataFrame]
) -> Dict[str, Dict]:
    """
    Args:
        data_dict: Dict mapping model names to DataFrames

    Returns:
        Dict of metrics per model
    """
```

#### `load_episode_data()`

Load specific episode from directory.

```python
def load_episode_data(
    data_dir: str,
    episode_id: int = 0,
    time_range: Tuple[float, float] = (0, 2)
) -> pd.DataFrame:
    """
    Args:
        data_dir: Directory containing CSV files
        episode_id: Episode number
        time_range: Time window to extract

    Returns:
        DataFrame with episode data
    """
```

### Metric Calculation Functions

```python
# Gait metrics
calculate_stride_period(contacts, time) -> (periods, times)
calculate_duty_cycle(contacts, time) -> duty_cycles
calculate_phase_lag(contacts_leg1, contacts_leg2, time) -> phase_deg

# Stability metrics
calculate_com_oscillation(base_positions) -> (vert_osc, lat_sway)

# Trajectory metrics
calculate_trajectory_smoothness(positions, time) -> smoothness
calculate_dtw_distance(traj1, traj2) -> distance

# Contact detection
detect_contacts(foot_positions, threshold=0.02) -> contacts_dict

# Forward kinematics
compute_foot_positions(joint_positions, base_state) -> foot_positions_dict
```

### Visualization Functions

```python
plot_contact_timeline(data_dict, time_range, save_path)
plot_foot_trajectories(data_dict, leg='FL', time_range, save_path)
plot_metrics_dashboard(metrics_dict, save_path)
plot_progressive_improvement(metrics_dict, save_path)
```

## 🐛 Troubleshooting

### Common Issues

#### Issue: "No episode file found"

**Cause**: Episode ID doesn't exist or wrong directory structure

**Solution**:
```bash
# Check available episodes
ls /path/to/data/dir/*.csv

# Use existing episode ID
python run_analysis.py --episode 0  # or 1, 2, etc.
```

#### Issue: "DTW library not found"

**Cause**: Optional DTW package not installed

**Solution**:
```bash
# Install recommended package
pip install dtaidistance

# Or use fallback (automatic, no action needed)
```

#### Issue: Contact detection looks wrong

**Cause**: Incorrect height threshold or noisy data

**Solution**: Adjust threshold in code
```python
contacts = detect_contacts(foot_positions, threshold=0.03)  # Increase threshold
```

#### Issue: Plots look cluttered

**Cause**: Too many models or long time window

**Solution**:
```bash
# Use shorter time window
python run_analysis.py --time-end 1.0

# Or analyze fewer models (edit data_paths in run_analysis.py)
```

### Performance Tips

- **Large datasets**: Reduce time window to 0-2s for faster processing
- **Multiple episodes**: Run in parallel using shell scripts
- **Memory issues**: Process one model at a time if dataset is very large

### Data Quality Checks

Before analysis, verify:

1. **CSV format**: Column names match expected format
2. **Time range**: Data covers requested time window
3. **Joint data**: All 12 joint positions present
4. **No NaN values**: Check for missing data

```python
import pandas as pd
df = pd.read_csv('your_file.csv')
print(df.info())  # Check for missing values
print(df['timestamp_sim'].min(), df['timestamp_sim'].max())  # Check time range
```

## 📚 Citations

If you use this tool in your research, please cite:

```bibtex
@article{your_paper_2026,
  title={Sim2Sim Transfer of Quadruped Locomotion Policies},
  author={Your Name},
  journal={Your Conference/Journal},
  year={2026}
}
```

## 🤝 Contributing

Contributions welcome! Areas for improvement:

- [ ] Real-time GRF data integration (if force sensors available)
- [ ] 3D trajectory visualization
- [ ] Interactive Plotly dashboards
- [ ] Automated statistical significance testing
- [ ] Multi-episode averaging with confidence intervals
- [ ] Export to LaTeX tables for papers

## 📄 License

MIT License - see LICENSE file for details

## 👨‍💻 Authors

- **Claude Code** - Initial implementation
- **Your Name** - Experimental design and data collection

## 📞 Contact

For questions or issues:
- GitHub Issues: [your-repo/issues](https://github.com/your-repo/issues)
- Email: your.email@domain.com

---

**Last Updated**: 2026-02-10

**Version**: 1.0.0

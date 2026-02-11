# Quick Start Guide - Sim2Sim Visualization Tool

## 🚀 5-Minute Setup

### Step 1: Install Dependencies

```bash
cd /home/drl-68/vistec_ex_ws
pip install -r requirements_viz.txt
```

### Step 2: Test with Synthetic Data

```bash
python run_analysis.py --synthetic --output ./test_figures
```

This generates sample data and creates all visualizations in `./test_figures/`.

### Step 3: View Results

```bash
# View generated files
ls -lh ./test_figures/

# Open visualizations (if display available)
eog ./test_figures/contact_timeline.png
eog ./test_figures/trajectories.png
eog ./test_figures/metrics_dashboard.png
eog ./test_figures/improvement.png

# View metrics summary
cat ./test_figures/summary_metrics.csv
```

---

## 📊 Analyze Your Experiment Data

### Basic Analysis (Episode 0, 0-2 seconds)

```bash
python run_analysis.py --episode 0 --output ./results_ep0
```

### Custom Time Window

```bash
# Analyze 0-5 seconds
python run_analysis.py --episode 0 --time-start 0 --time-end 5 --output ./results_0_5s

# Analyze 2-4 seconds (mid-gait)
python run_analysis.py --episode 0 --time-start 2 --time-end 4 --output ./results_2_4s
```

### Multiple Episodes

```bash
# Batch process episodes 0-4
for i in {0..4}; do
  python run_analysis.py --episode $i --output ./results_ep$i
done
```

---

## 🐍 Use as Python Module

### Example 1: Basic Analysis

```python
from sim2sim_visualization import analyze_sim2sim_transfer

# Define data paths
data_paths = {
    'isaac': '/home/drl-68/unitree_rl_lab/logs/data_collection_fixed_env/lstm_dr_fixed',
    'gazebo_lstm_dr': '/home/drl-68/vistec_ex_ws/experiment_data/lstm_dr/task_0',
}

# Run analysis
metrics, summary = analyze_sim2sim_transfer(
    data_paths=data_paths,
    time_range=(0, 2),
    episode_id=0,
    output_dir='./my_analysis'
)

# Print summary
print(summary)
```

### Example 2: Custom Metric Extraction

```python
from sim2sim_visualization import (
    load_episode_data,
    compute_all_metrics,
)

# Load data
isaac_df = load_episode_data(
    '/home/drl-68/unitree_rl_lab/logs/data_collection_fixed_env/lstm_dr_fixed',
    episode_id=0,
    time_range=(0, 2)
)

gazebo_df = load_episode_data(
    '/home/drl-68/vistec_ex_ws/experiment_data/lstm_dr/task_0',
    episode_id=0,
    time_range=(0, 2)
)

# Compute metrics
data_dict = {'isaac': isaac_df, 'gazebo': gazebo_df}
metrics = compute_all_metrics(data_dict)

# Access specific metrics
isaac_stride = metrics['isaac']['stride_periods_FL']
gazebo_stride = metrics['gazebo']['stride_periods_FL']

print(f"Isaac stride periods: {isaac_stride}")
print(f"Gazebo stride periods: {gazebo_stride}")
```

### Example 3: Generate Individual Plots

```python
from sim2sim_visualization import (
    compute_all_metrics,
    plot_contact_timeline,
    plot_foot_trajectories,
)

# ... load data and compute metrics ...

# Generate only specific plots
plot_contact_timeline(metrics, time_range=(0, 2),
                     save_path='./my_contact_plot.png')

plot_foot_trajectories(metrics, leg='FL', time_range=(0, 2),
                      save_path='./my_trajectory_plot.png')
```

---

## 📈 Understanding the Outputs

### File Overview

| File | Content | Use Case |
|------|---------|----------|
| `contact_timeline.png` | Gantt-style contact patterns | Visualize gait timing |
| `trajectories.png` | Foot trajectories in 2D | Compare motion quality |
| `metrics_dashboard.png` | 6-panel metric comparison | Comprehensive assessment |
| `improvement.png` | Progressive model improvement | Show incremental gains |
| `summary_metrics.csv` | Numerical metrics table | Statistical analysis |

### Quick Interpretation Guide

#### ✅ Good Indicators
- **Contact Timeline**: Consistent stride periods, clear diagonal pairs
- **Trajectories**: Smooth ellipses, similar to Isaac reference
- **Duty Cycle**: 50-55% (green bars in dashboard)
- **CoM Oscillation**: <0.03m
- **Phase Coordination**: Close to ideal trot pattern (180°, 180°, 0°)

#### ⚠️ Warning Signs
- **Contact Timeline**: Irregular patterns, missing contacts
- **Trajectories**: Jagged lines, ground penetration
- **Duty Cycle**: <40% or >65% (red/yellow bars)
- **CoM Oscillation**: >0.05m
- **High Jerk**: Very high smoothness values

---

## 🔧 Common Tasks

### Task 1: Compare Two Specific Models

```python
data_paths = {
    'isaac': '/path/to/isaac/data',
    'gazebo_lstm_dr': '/path/to/gazebo/lstm_dr',
}

metrics, _ = analyze_sim2sim_transfer(data_paths, output_dir='./comparison')
```

### Task 2: Extract Stride Statistics

```python
from sim2sim_visualization import load_episode_data, compute_all_metrics
import numpy as np

# Load and compute
df = load_episode_data('/path/to/data', episode_id=0)
metrics = compute_all_metrics({'model': df})

# Get stride statistics
strides = metrics['model']['stride_periods_FL']
print(f"Mean stride period: {np.mean(strides):.3f} s")
print(f"Std stride period: {np.std(strides):.3f} s")
print(f"Stride frequency: {1/np.mean(strides):.2f} Hz")
```

### Task 3: Compute Transfer Gap

```python
# After computing metrics for both simulators
isaac_score = compute_similarity_score(metrics['isaac'])
gazebo_score = compute_similarity_score(metrics['gazebo_lstm_dr'])

transfer_gap = abs(isaac_score - gazebo_score)
print(f"Sim2Sim transfer gap: {transfer_gap:.1f}%")
```

---

## 🐛 Quick Troubleshooting

### Issue: Import errors

```bash
# Make sure you're in the right directory
cd /home/drl-68/vistec_ex_ws

# Check Python can find the module
python -c "import sim2sim_visualization; print('OK')"
```

### Issue: No episode found

```bash
# List available episodes
ls /path/to/data/dir/*.csv

# Or check task subdirectories
ls /path/to/data/dir/task_*/

# Use correct episode ID
python run_analysis.py --episode 0  # Start with 0
```

### Issue: Plots look empty

- **Cause**: Time range doesn't contain motion data
- **Solution**: Check your data's timestamp range first

```python
import pandas as pd
df = pd.read_csv('your_file.csv')
print(f"Time range: {df['timestamp_sim'].min():.2f} - {df['timestamp_sim'].max():.2f}s")
```

---

## 📚 Next Steps

1. ✅ Test with synthetic data (you're here!)
2. 📊 Analyze your first real episode
3. 📈 Compare multiple models
4. 📝 Export metrics for your paper
5. 🎨 Customize visualizations (edit `sim2sim_visualization.py`)

For detailed documentation, see [README_VISUALIZATION.md](README_VISUALIZATION.md)

---

**Happy Analyzing! 🎉**

# Fixed Environment Testing for Gazebo Comparison

## Overview

To fairly compare IsaacLab and Gazebo, you need **identical deterministic conditions**:
- ❌ No domain randomization
- ❌ No disturbances (pushes, forces)
- ❌ No observation noise
- ✅ Fixed, deterministic physics

---

## Problem with Current Setup

The current `test_4_tasks_all_models.sh` applies:
- ✅ Clean observations (good)
- ⚠️ Domain randomization events (BAD for Gazebo comparison)
- ⚠️ Disturbances every 3-10 seconds (BAD for Gazebo comparison)

**Result**: Data doesn't match Gazebo's deterministic environment!

---

## Solution: Two Approaches

### Approach 1: Modify Play Configurations (Recommended)

Create custom play configs that disable ALL events.

#### Step 1: Create Fixed Play Config

```python
# File: velocity_env_cfg_mlp_custom.py

class RobotPlayEnvCfgFixed(RobotEnvCfg):
    """Play configuration with FIXED environment (no randomization)."""

    def __post_init__(self):
        super().__post_init__()

        # Disable observation noise
        self.observations.policy.enable_corruption = False

        # Reduce environments
        self.scene.num_envs = 32

        # CRITICAL: Disable ALL events for fixed environment
        # Override event config with empty/disabled events
        from isaaclab.envs import EventTermCfg as EventTerm

        # Create minimal event config with no randomization
        @configclass
        class FixedEventCfg:
            """No randomization events."""
            pass

        # Replace events with fixed config
        self.events = FixedEventCfg()

        # Use full command ranges
        self.commands.base_velocity.ranges = self.commands.base_velocity.limit_ranges
```

---

### Approach 2: Runtime Event Disabling (Easier)

Modify the data collection script to disable events at runtime.

Add this to `collect_data_isaaclab.py` after creating the environment:

```python
# After line 526: env = gym.make(...)

# Disable ALL randomization events
print("\n🔧 Disabling randomization for Gazebo comparison...")

if hasattr(env.unwrapped, '_event_manager'):
    event_manager = env.unwrapped._event_manager

    # Disable all event terms
    for event_name, event_term in event_manager._terms.items():
        if hasattr(event_term, '_mode'):
            original_mode = event_term._mode
            event_term._mode = None  # Disable
            print(f"  - Disabled: {event_name} (was: {original_mode})")

print("✅ All randomization disabled - environment is now FIXED")
```

---

## Quick Fix: Modify Existing Script

Add this function to `collect_data_isaaclab.py`:

```python
def disable_all_randomization(env):
    """Disable all domain randomization for fixed environment testing."""
    print("\n🔧 Disabling all randomization events...")

    unwrapped_env = env.unwrapped

    # Disable event manager if it exists
    if hasattr(unwrapped_env, 'event_manager'):
        event_mgr = unwrapped_env.event_manager

        # Iterate through all event terms
        for term_name in event_mgr._terms:
            term = event_mgr._terms[term_name]

            # Store original mode
            if hasattr(term, '_mode'):
                original_mode = term._mode

                # Disable by setting mode to None
                term._mode = None
                term._is_enabled = False

                print(f"  ✓ Disabled: {term_name} (was: {original_mode})")

    print("✅ Fixed environment ready (matches Gazebo)")
    return env
```

Then call it after creating the environment:

```python
# Line ~527
env = gym.make(args_cli.task, cfg=env_cfg, render_mode="rgb_array" if args_cli.headless else None)

# ADD THIS LINE:
env = disable_all_randomization(env)

# Create data collector
collector = DataCollector(env, args_cli.checkpoint, args_cli.task)
```

---

## Testing: 6 Policies in Fixed Environment

Use the provided script:

```bash
chmod +x test_fixed_env_for_gazebo.sh
./test_fixed_env_for_gazebo.sh
```

This will test:
1. **MLP + Custom DR** (trained with DR, tested without)
2. **MLP - No DR** (trained without, tested without)
3. **LSTM + DR** (trained with DR, tested without)
4. **LSTM - No DR** (trained without, tested without)
5. **Implicit + DR** (trained with DR, tested without)
6. **Implicit** (trained without, tested without)

**Output**: 1200 CSV files with deterministic data matching Gazebo!

---

## What About 8 Policies?

You mentioned 8 policies. Options to add 2 more:

### Option A: Add Intermediate Checkpoints
Test early and late training checkpoints:
- LSTM-DR @ 15k iterations
- LSTM-DR @ 25k iterations

### Option B: Add Different Actuator Configs
If you have trained:
- LSTM-Custom
- LSTM-Custom-Enhanced
- LSTM-MyModel

### Option C: Add Baseline Policies
- Random policy (no training)
- PD controller baseline

**Which 8 policies do you want?** Please specify!

---

## Verification: Fixed vs Randomized

Compare the two datasets:

```python
import pandas as pd
import numpy as np

# Load same episode from both datasets
ep_fixed = pd.read_csv('logs/data_collection_fixed_env/lstm_dr_fixed/locomotion_log_isaac_ep050_*.csv')
ep_random = pd.read_csv('logs/data_collection_4tasks/lstm_dr/locomotion_log_isaac_ep050_*.csv')

# Check base position variance (should be lower in fixed env)
print("Base position X variance:")
print(f"  Fixed env:  {ep_fixed['base_pos_x'].var():.6f}")
print(f"  Random env: {ep_random['base_pos_x'].var():.6f}")

# Check for sudden velocity jumps (pushes)
vel_diff_fixed = np.abs(np.diff(ep_fixed['base_lin_vel_x']))
vel_diff_random = np.abs(np.diff(ep_random['base_lin_vel_x']))

print(f"\nMax velocity jump:")
print(f"  Fixed env:  {vel_diff_fixed.max():.3f} m/s")
print(f"  Random env: {vel_diff_random.max():.3f} m/s")  # Should see ~1.0 m/s pushes
```

**Expected**:
- Fixed env: Smooth, deterministic motion
- Random env: Sudden jumps from pushes/disturbances

---

## Gazebo Data Collection Checklist

To match IsaacLab fixed environment in Gazebo:

- [ ] Use same episode configs (`episode_configs_4tasks.yaml`)
- [ ] Apply same velocity commands at same times
- [ ] Use same PD gains (Kp=160, Kd=5)
- [ ] Use same control frequency (50 Hz)
- [ ] Use same physics timestep (5ms)
- [ ] Disable Gazebo randomization
- [ ] Record same 74 columns
- [ ] Ensure deterministic physics (fixed seed if needed)

---

## Expected Differences: IsaacLab vs Gazebo

Even with fixed environments, expect small differences due to:

1. **Physics engine**: PhysX (IsaacLab) vs ODE (Gazebo)
2. **Contact models**: Different friction/collision algorithms
3. **Numerical precision**: GPU (IsaacLab) vs CPU (Gazebo)
4. **Integration methods**: Slight differences in solvers

**Acceptable variance**: <5% in most metrics

**Large variance (>20%)**: Indicates misconfiguration!

---

## Summary

✅ **Run this to get Gazebo-matching data**:
```bash
./test_fixed_env_for_gazebo.sh
```

✅ **Collect matching data in Gazebo** using the same:
- Episode configs
- Velocity commands
- Control parameters
- Physics settings

✅ **Compare datasets** to validate sim-to-sim transfer!

---

**Ready for Gazebo comparison! 🤖**

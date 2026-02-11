# Continue Training Implicit Policies

## Current Status

You have 2 Implicit policies that can be continued from their checkpoints:

### 1. Implicit (No DR)
- **Checkpoint:** `model_8100.pt` (8100 iterations)
- **Path:** `logs/rsl_rl/unitree_go2_velocity_implicit/2026-02-05_16-27-42/`
- **Task:** `Unitree-Go2-Velocity-Implicit`
- **Actuator:** IdealPD (Kp=160.0, Kd=5.0)
- **Domain Randomization:** DISABLED
- **Remaining:** ~16,900 iterations to reach 25,000

### 2. Implicit + DR
- **Checkpoint:** `model_14600.pt` (14600 iterations)
- **Path:** `logs/rsl_rl/unitree_go2_velocity_implicit_dr/2026-02-05_18-14-00/`
- **Task:** `Unitree-Go2-Velocity-Implicit-DR`
- **Actuator:** IdealPD (Kp=160.0, Kd=5.0)
- **Domain Randomization:** ENABLED
- **Remaining:** ~10,400 iterations to reach 25,000

---

## Quick Start Commands

### Option 1: Interactive Menu (Recommended)

```bash
cd /home/drl-68/unitree_rl_lab
./continue_implicit_training.sh
```

**What it does:**
- Shows menu to select which policy to train
- Options: (1) No DR, (2) + DR, (3) Both sequentially
- Automatically resumes from correct checkpoint

---

### Option 2: Direct Commands

#### Continue Implicit (No DR)
```bash
cd /home/drl-68/unitree_rl_lab
./continue_implicit_no_dr.sh
```

#### Continue Implicit + DR
```bash
cd /home/drl-68/unitree_rl_lab
./continue_implicit_dr.sh
```

---

### Option 3: Manual Python Commands

#### Implicit (No DR)
```bash
python scripts/rsl_rl/train.py \
  --task Unitree-Go2-Velocity-Implicit \
  --num_envs 4096 \
  --resume \
  --load_run 2026-02-05_16-27-42 \
  --headless
```

#### Implicit + DR
```bash
python scripts/rsl_rl/train.py \
  --task Unitree-Go2-Velocity-Implicit-DR \
  --num_envs 4096 \
  --resume \
  --load_run 2026-02-05_18-14-00 \
  --headless
```

---

## Training Configuration

### Environment Settings

Both policies use:
- **Number of environments:** 4096
- **Physics timestep:** 0.005s (5ms)
- **Control frequency:** 50 Hz
- **Actuator type:** IdealPD
- **PD Gains:** Kp=160.0, Kd=5.0 (HIGH - physics-based control)

### Domain Randomization (DR Only)

Implicit + DR includes:
- **Mass randomization:** ±20% base mass
- **Friction randomization:** 0.5 to 1.25
- **Motor strength:** ±20% PD gains
- **Push disturbances:** Random velocity impulses
- **External forces:** Random forces on base

---

## Expected Training Duration

### Implicit (No DR) - 8100 → 25000 iterations

- **Remaining iterations:** ~16,900
- **Estimated time:** ~3-4 hours (depends on GPU)
- **Updates per iteration:** 4096 environments × episode length

### Implicit + DR - 14600 → 25000 iterations

- **Remaining iterations:** ~10,400
- **Estimated time:** ~2-3 hours (depends on GPU)
- **Updates per iteration:** 4096 environments × episode length

**Note:** Training with DR may be slightly slower due to randomization overhead.

---

## Monitoring Training

### TensorBoard

While training, monitor progress with TensorBoard:

```bash
# In a separate terminal
tensorboard --logdir logs/rsl_rl/

# Then open browser to: http://localhost:6006
```

**Key metrics to watch:**
- `Loss/value_function` - Should decrease
- `Loss/surrogate` - Policy loss
- `Policy/mean_reward` - Should increase
- `Policy/mean_episode_length` - Track episode duration

### Console Output

Training will print periodic updates:
```
Iteration 8200/25000
Mean reward: 12.45
Mean episode length: 1000 steps
Time elapsed: 45.2s
```

---

## Checkpoints

### Auto-Save Frequency

Checkpoints are saved every **100 iterations** by default:
- `model_8100.pt` (already exists)
- `model_8200.pt`
- `model_8300.pt`
- ...
- `model_25000.pt` (final)

### Checkpoint Location

**Implicit (No DR):**
```
logs/rsl_rl/unitree_go2_velocity_implicit/2026-02-05_16-27-42/
```

**Implicit + DR:**
```
logs/rsl_rl/unitree_go2_velocity_implicit_dr/2026-02-05_18-14-00/
```

---

## When to Stop Training

### Target Iterations

**Typical for Implicit policies:** 20,000 - 25,000 iterations

### Performance Indicators

Stop when:
1. **Mean reward plateaus** - No improvement for 1000+ iterations
2. **Target performance achieved** - Robot walks reliably
3. **Validation success** - Test policy on different terrains

### Early Stopping

If performance degrades:
- Training may have overfit
- Use earlier checkpoint (e.g., model_20000.pt)
- Check if learning rate needs adjustment

---

## After Training Completes

### 1. Test the Updated Policy

```bash
# Test Implicit (No DR)
python scripts/rsl_rl/play.py \
  --task Unitree-Go2-Velocity-Implicit \
  --num_envs 1 \
  --load_run 2026-02-05_16-27-42 \
  --checkpoint model_25000.pt

# Test Implicit + DR
python scripts/rsl_rl/play.py \
  --task Unitree-Go2-Velocity-Implicit-DR \
  --num_envs 1 \
  --load_run 2026-02-05_18-14-00 \
  --checkpoint model_25000.pt
```

### 2. Update Test Scripts

If new checkpoints perform better, update the test script:

Edit `test_fixed_env_for_gazebo.sh`:
```bash
# Change from:
"Unitree-Go2-Velocity-Implicit:2026-02-05_16-27-42:model_8100.pt:implicit_fixed"

# To:
"Unitree-Go2-Velocity-Implicit:2026-02-05_16-27-42:model_25000.pt:implicit_fixed"
```

### 3. Re-collect Data (Optional)

If you want to compare old vs new checkpoints:

```bash
# Use new checkpoint for data collection
./test_fixed_env_for_gazebo.sh
```

---

## Troubleshooting

### Issue: "Checkpoint not found"

**Check:**
```bash
ls logs/rsl_rl/unitree_go2_velocity_implicit/2026-02-05_16-27-42/
ls logs/rsl_rl/unitree_go2_velocity_implicit_dr/2026-02-05_18-14-00/
```

**Solution:** Verify paths match exactly

### Issue: "CUDA out of memory"

**Reduce number of environments:**
```bash
# Change from 4096 to 2048
python scripts/rsl_rl/train.py \
  --task Unitree-Go2-Velocity-Implicit \
  --num_envs 2048 \  # <-- Reduced
  --resume \
  --load_run 2026-02-05_16-27-42 \
  --headless
```

### Issue: Training very slow

**Possible causes:**
1. GPU not being used (check with `nvidia-smi`)
2. Too many environments for GPU memory
3. Background processes consuming GPU

**Solution:**
- Close other GPU applications
- Reduce `--num_envs`
- Check GPU utilization with `watch -n 1 nvidia-smi`

---

## Comparison: Why Continue Training?

### Current Performance (8100 / 14600 iterations)

Policies may:
- Walk but with some wobble
- Fall occasionally
- Not track velocity commands well
- Struggle with turns

### Expected After 25000 iterations

Policies should:
- Walk smoothly and stably
- Track velocity commands accurately
- Turn in place reliably
- Handle combined walk+turn tasks

---

## Summary

### Quick Commands

**Interactive menu:**
```bash
./continue_implicit_training.sh
```

**Direct commands:**
```bash
./continue_implicit_no_dr.sh     # Implicit (No DR)
./continue_implicit_dr.sh         # Implicit + DR
```

### What to Expect

- Training duration: 2-4 hours per policy
- Automatic checkpoint saving every 100 iterations
- Monitor with TensorBoard for real-time metrics
- Final checkpoints at iteration 25000

### After Training

1. Test new checkpoints with `play.py`
2. Compare performance vs old checkpoints
3. Update test scripts if needed
4. Re-collect data for Gazebo comparison

---

## Additional Notes

### Why Implicit Policies Need More Training?

Implicit (IdealPD) actuators don't learn actuator dynamics, so:
- They rely entirely on PD control (Kp=160, Kd=5)
- Need more iterations to learn optimal joint positions
- May require 20K-25K iterations vs 15K for neural actuators

### Comparison with MLP/LSTM

| Policy Type | Actuator | PD Gains | Typical Iterations |
|------------|----------|----------|-------------------|
| MLP/LSTM | Neural Network | Kp=25, Kd=0.5 | 15,000 - 20,000 |
| Implicit | IdealPD | Kp=160, Kd=5 | 20,000 - 25,000 |

---

**Ready to continue training!**

Run:
```bash
./continue_implicit_training.sh
```

**Last Updated:** 2026-02-10

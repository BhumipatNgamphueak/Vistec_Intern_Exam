# Chirp (Frequency Sweep) Test Guide

## Overview

Test actuator/motor response across frequency range using **chirp signal** (frequency sweep).

**Chirp signal:** Sine wave that sweeps from low frequency to high frequency over time.

**Use for:**
- Frequency response analysis
- Bandwidth identification
- Resonance detection
- System identification
- Bode plot generation
- Comparing actuator dynamics

---

## Quick Start

### Complete Workflow (All Actuators)

```bash
cd /home/drl-68/unitree_rl_lab
./chirp_test_all.sh
```

**Tests:**
1. Gazebo motor (LOW PD gains)
2. MLP actuator
3. LSTM actuator
4. Implicit actuator

**Duration:** ~5 minutes total

**Output:** `chirp_analysis/`

---

## Manual Testing

### IsaacLab - Single Actuator

```bash
# MLP actuator (Kp=25, Kd=0.5)
python scripts/actuator_comparison/chirp_test_isaaclab.py \
  --actuator mlp \
  --joint FL_hip_joint \
  --f0 0.1 \
  --f1 20.0 \
  --duration 10.0 \
  --amplitude 0.3 \
  --headless

# LSTM actuator (Kp=25, Kd=0.5)
python scripts/actuator_comparison/chirp_test_isaaclab.py \
  --actuator lstm \
  --joint FL_hip_joint \
  --f0 0.1 \
  --f1 20.0 \
  --duration 10.0 \
  --amplitude 0.3 \
  --headless

# Implicit actuator (Kp=160, Kd=5)
python scripts/actuator_comparison/chirp_test_isaaclab.py \
  --actuator implicit \
  --joint FL_hip_joint \
  --f0 0.1 \
  --f1 20.0 \
  --duration 10.0 \
  --amplitude 0.3 \
  --headless
```

**Parameters:**
- `--actuator`: `mlp`, `lstm`, or `implicit`
- `--joint`: Joint name (default: FL_hip_joint)
- `--f0`: Start frequency in Hz (default: 0.1)
- `--f1`: End frequency in Hz (default: 20.0)
- `--duration`: Test duration in seconds (default: 10.0)
- `--amplitude`: Chirp amplitude in radians (default: 0.3)
- `--output`: Output directory (default: chirp_analysis)

---

### Gazebo - Motor Test

**Prerequisites:**
```bash
# Terminal 1 - Start Gazebo
ros2 launch go2_gazebo go2_control.launch.py \
  spawn_z:=1.5 \
  controller_config:=go2_controllers_mlp_lstm.yaml
```

**Run test:**
```bash
# Terminal 2 - LOW PD gains (Kp=25, Kd=0.5)
python scripts/actuator_comparison/chirp_test_gazebo.py \
  --joint FL_hip_joint \
  --pd_gains low \
  --f0 0.1 \
  --f1 20.0 \
  --duration 10.0 \
  --amplitude 0.3

# HIGH PD gains (Kp=160, Kd=5)
python scripts/actuator_comparison/chirp_test_gazebo.py \
  --joint FL_hip_joint \
  --pd_gains high \
  --f0 0.1 \
  --f1 20.0 \
  --duration 10.0 \
  --amplitude 0.3
```

---

## What is Chirp Signal?

### Mathematical Definition

```python
# Linear chirp (frequency increases linearly)
f(t) = f0 + k*t                    # Instantaneous frequency
k = (f1 - f0) / T                  # Sweep rate
φ(t) = 2π(f0*t + 0.5*k*t²)        # Phase
x(t) = A * sin(φ(t))               # Chirp signal
```

**Where:**
- `f0` = Start frequency (Hz)
- `f1` = End frequency (Hz)
- `T` = Duration (seconds)
- `A` = Amplitude (radians)

### Example

**Parameters:** f0=0.1 Hz, f1=20 Hz, T=10s

```
t=0s   → frequency = 0.1 Hz  (very slow)
t=5s   → frequency = 10 Hz   (medium)
t=10s  → frequency = 20 Hz   (fast)
```

Robot joint sweeps from slow to fast motion in 10 seconds!

---

## Output Structure

```
chirp_analysis/
├── gazebo_pd_low/
│   ├── chirp_results_YYYYMMDD_HHMMSS.json
│   └── chirp_plots_YYYYMMDD_HHMMSS/
│       ├── chirp_time_domain.png          # Position, velocity, torque vs time
│       ├── chirp_frequency_analysis.png   # Error vs frequency
│       └── chirp_spectrogram.png          # (Gazebo doesn't have this)
│
├── mlp/
│   ├── chirp_results_*.json
│   └── chirp_plots_*/
│       ├── chirp_time_domain.png
│       ├── chirp_frequency_analysis.png
│       └── chirp_spectrogram.png          # IsaacLab only
│
├── lstm/
│   └── ... (same structure)
│
└── implicit/
    └── ... (same structure)
```

---

## Understanding the Plots

### 1. Time Domain Plot

**4 subplots:**

**Position:**
- Black dashed = Command (chirp signal)
- Blue/Red = Actual response
- Should overlap at low frequencies
- May diverge at high frequencies

**Error:**
- Tracking error (actual - command)
- Typically increases with frequency
- Shows RMS error in title

**Velocity:**
- Joint velocity over time
- Amplitude increases as frequency increases

**Torque/Effort:**
- Actuator torque output
- Shows control effort required

---

### 2. Frequency Analysis Plot

**Top subplot:**
- Shows frequency sweep profile (0.1 → 20 Hz)
- Verifies chirp signal correct

**Bottom subplot:**
- X-axis: Frequency (Hz)
- Y-axis: Tracking error magnitude (mrad)
- Color: Time progression
- **Key insight:** Error increases at higher frequencies

**What to look for:**
- **Bandwidth:** Frequency where error starts increasing rapidly
- **Resonance:** Peaks in error at specific frequencies
- **Flat region:** Good tracking range

---

### 3. Spectrogram (IsaacLab Only)

**Command spectrogram:**
- Shows frequency content of command signal
- Should show diagonal line (frequency increasing)

**Actual response spectrogram:**
- Shows frequency content of actual response
- Compare with command to see tracking

---

## Interpreting Results

### Bandwidth Identification

**Definition:** Frequency at which tracking degrades significantly

**How to find:**
1. Look at "Error vs Frequency" plot
2. Find frequency where error > 2× low-frequency error
3. That's your bandwidth!

**Example:**
```
Error @ 1 Hz: 5 mrad    (baseline)
Error @ 10 Hz: 50 mrad  (10× increase)
Error @ 15 Hz: 150 mrad (30× increase)

Bandwidth ≈ 10 Hz
```

**Meaning:** Actuator can track commands up to ~10 Hz accurately

---

### Comparing Actuators

**MLP/LSTM vs Gazebo (LOW gains):**

**Expect similar bandwidth:**
- Both use Kp=25, Kd=0.5
- Should track similarly up to ~5-10 Hz
- Error patterns should match

**Differences indicate:**
- Neural network learned dynamics
- Simulation physics differences
- Different delay characteristics

---

**Implicit vs Gazebo (HIGH gains):**

**Expect wider bandwidth:**
- Both use Kp=160, Kd=5
- Should track well up to ~15-20 Hz
- Very similar responses (both PD controllers)

**Large differences indicate:**
- PD gains not loaded correctly
- Physics timestep mismatch
- Joint damping/friction differences

---

## Resonance Detection

**What is resonance?**
- Frequency where system naturally oscillates
- Shows as peak in error vs frequency plot

**Example:**
```
Error @ 5 Hz:  20 mrad
Error @ 8 Hz:  80 mrad  ← Peak (resonance!)
Error @ 10 Hz: 40 mrad
```

**Meaning:** System has natural frequency around 8 Hz

**Cause:**
- Mechanical compliance
- Inertia + stiffness interaction
- Poorly tuned PD gains

---

## Typical Results

### MLP/LSTM Actuators (Kp=25, Kd=0.5)

**Bandwidth:** ~8-12 Hz

**Characteristics:**
- Lower stiffness allows larger errors
- Smoother response (learned dynamics)
- May show damping effects

---

### Implicit Actuator (Kp=160, Kd=5)

**Bandwidth:** ~15-20 Hz

**Characteristics:**
- Higher stiffness = tighter tracking
- Faster response
- May show oscillations if under-damped

---

### Gazebo Motors

**LOW gains (Kp=25, Kd=0.5):**
- Should match MLP/LSTM
- Bandwidth ~8-12 Hz

**HIGH gains (Kp=160, Kd=5):**
- Should match Implicit
- Bandwidth ~15-20 Hz

---

## Use Cases

### 1. Validate Sim-to-Sim Matching

**Goal:** Ensure Gazebo matches IsaacLab

**Method:**
1. Run chirp test on MLP in IsaacLab
2. Run chirp test on Gazebo with LOW gains
3. Compare bandwidths
4. Compare error patterns

**Good match:** Bandwidth within 20%, similar error curves

---

### 2. Identify Control Bandwidth

**Goal:** Know maximum tracking frequency

**Method:**
1. Run chirp test
2. Find frequency where error > threshold
3. That's your bandwidth limit

**Use:** Design velocity commands within bandwidth!

---

### 3. Tune PD Gains

**Goal:** Find optimal PD gains for Gazebo

**Method:**
1. Run chirp with different gains
2. Compare bandwidths
3. Choose gains matching IsaacLab actuator

**Example:**
```
Kp=10:  Bandwidth = 5 Hz   (too low)
Kp=25:  Bandwidth = 10 Hz  (matches MLP! ✓)
Kp=50:  Bandwidth = 15 Hz  (too high)
```

---

### 4. Detect Mechanical Issues

**Goal:** Find resonances or instabilities

**Method:**
1. Run chirp test
2. Look for error peaks
3. Identify resonant frequencies

**Action:** Avoid commanding at resonant frequencies!

---

## Advanced Analysis

### Export Data for Custom Analysis

```python
import json
import numpy as np
import matplotlib.pyplot as plt

# Load data
with open('chirp_analysis/mlp/chirp_results_*.json', 'r') as f:
    data = json.load(f)

time = np.array(data['time'])
pos_cmd = np.array(data['position_command'])
pos_actual = np.array(data['position_actual'])
freq = np.array(data['frequency'])

# Custom analysis
error = pos_actual - pos_cmd

# Gain vs frequency
gain = np.abs(pos_actual) / np.abs(pos_cmd)
plt.plot(freq, 20*np.log10(gain))  # Bode magnitude plot
plt.xlabel('Frequency (Hz)')
plt.ylabel('Gain (dB)')
plt.show()
```

---

### Generate Bode Plot

```python
from scipy import signal

# Transfer function estimation
f, Pxx = signal.welch(pos_cmd, fs=1/dt)
f, Pyy = signal.welch(pos_actual, fs=1/dt)

# Frequency response
H = np.sqrt(Pyy / Pxx)

plt.semilogx(f, 20*np.log10(H))
plt.xlabel('Frequency (Hz)')
plt.ylabel('Magnitude (dB)')
plt.title('Frequency Response (Bode Plot)')
```

---

## Comparison Workflow

### Complete Chirp Comparison

1. **Run all tests:**
   ```bash
   ./chirp_test_all.sh
   ```

2. **Compare MLP vs Gazebo:**
   - Open both: `chirp_frequency_analysis.png`
   - Check bandwidth similarity
   - Compare error magnitudes

3. **Compare LSTM vs Gazebo:**
   - Same as MLP comparison

4. **Compare Implicit vs Gazebo HIGH:**
   - Run Gazebo with HIGH gains
   - Compare responses

---

## Troubleshooting

### Large Tracking Errors

**Symptom:** Error > 100 mrad even at low frequencies

**Causes:**
- PD gains too low
- Actuator delay
- Joint limits hit

**Solution:**
- Increase PD gains
- Reduce chirp amplitude
- Check joint within limits

---

### Oscillations/Instability

**Symptom:** Error grows exponentially, robot unstable

**Causes:**
- PD gains too high
- Under-damped system
- Control frequency too low

**Solution:**
- Decrease Kp
- Increase Kd
- Verify 50 Hz control frequency

---

### No Response

**Symptom:** Actual position doesn't change

**IsaacLab:**
- Check actuator configured correctly
- Verify joint not locked

**Gazebo:**
- Check controller active: `ros2 control list_controllers`
- Verify joint not in collision

---

## Summary

**Chirp test command:**
```bash
# IsaacLab
python scripts/actuator_comparison/chirp_test_isaaclab.py \
  --actuator mlp --joint FL_hip_joint \
  --f0 0.1 --f1 20.0 --duration 10.0 --headless

# Gazebo
python scripts/actuator_comparison/chirp_test_gazebo.py \
  --joint FL_hip_joint --pd_gains low \
  --f0 0.1 --f1 20.0 --duration 10.0
```

**Or use automated workflow:**
```bash
./chirp_test_all.sh
```

**Key outputs:**
- `chirp_time_domain.png` - Response over time
- `chirp_frequency_analysis.png` - Error vs frequency
- `chirp_spectrogram.png` - Frequency content (IsaacLab)

**Goal:** Characterize actuator frequency response and bandwidth!

---

**Last Updated:** 2026-02-10

#!/bin/bash
# Complete chirp (frequency sweep) test workflow

set -e

echo "========================================="
echo "Chirp Test - Complete Workflow"
echo "========================================="
echo ""
echo "This will perform chirp (frequency sweep) tests:"
echo "  1. Gazebo motor (effort controller)"
echo "  2. IsaacLab MLP actuator"
echo "  3. IsaacLab LSTM actuator"
echo "  4. IsaacLab Implicit actuator"
echo ""
echo "Test Parameters:"
echo "  Frequency range: 0.1 Hz → 20 Hz"
echo "  Duration: 10 seconds"
echo "  Amplitude: 0.3 rad"
echo "  Joint: FL_hip_joint"
echo ""
echo "Output: chirp_analysis/"
echo ""

# Configuration
JOINT="FL_hip_joint"
F0=0.1
F1=20.0
DURATION=10.0
AMPLITUDE=0.3
OUTPUT="chirp_analysis"

# =========================================
# Step 1: Gazebo Test
# =========================================
echo "========================================="
echo "Step 1/4: Gazebo Motor Test"
echo "========================================="
echo ""
echo "⚠️  Gazebo must be running with:"
echo "    ros2 launch go2_gazebo go2_control.launch.py \\"
echo "      spawn_z:=1.5 \\"
echo "      controller_config:=go2_controllers_mlp_lstm.yaml"
echo ""
read -p "Is Gazebo ready? (y/n): " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Please start Gazebo first."
    exit 1
fi

echo ""
echo "Running Gazebo chirp test..."
python scripts/actuator_comparison/chirp_test_gazebo.py \
  --joint $JOINT \
  --pd_gains low \
  --f0 $F0 \
  --f1 $F1 \
  --duration $DURATION \
  --amplitude $AMPLITUDE \
  --output $OUTPUT

echo "✅ Gazebo chirp test complete"

# =========================================
# Step 2: MLP Test
# =========================================
echo ""
echo "========================================="
echo "Step 2/4: MLP Actuator Test"
echo "========================================="
echo ""
echo "Running MLP chirp test (Kp=25, Kd=0.5)..."
python scripts/actuator_comparison/chirp_test_isaaclab.py \
  --actuator mlp \
  --joint $JOINT \
  --f0 $F0 \
  --f1 $F1 \
  --duration $DURATION \
  --amplitude $AMPLITUDE \
  --output $OUTPUT \
  --headless

echo "✅ MLP chirp test complete"

# =========================================
# Step 3: LSTM Test
# =========================================
echo ""
echo "========================================="
echo "Step 3/4: LSTM Actuator Test"
echo "========================================="
echo ""
echo "Running LSTM chirp test (Kp=25, Kd=0.5)..."
python scripts/actuator_comparison/chirp_test_isaaclab.py \
  --actuator lstm \
  --joint $JOINT \
  --f0 $F0 \
  --f1 $F1 \
  --duration $DURATION \
  --amplitude $AMPLITUDE \
  --output $OUTPUT \
  --headless

echo "✅ LSTM chirp test complete"

# =========================================
# Step 4: Implicit Test
# =========================================
echo ""
echo "========================================="
echo "Step 4/4: Implicit Actuator Test"
echo "========================================="
echo ""
echo "Running Implicit chirp test (Kp=160, Kd=5)..."
python scripts/actuator_comparison/chirp_test_isaaclab.py \
  --actuator implicit \
  --joint $JOINT \
  --f0 $F0 \
  --f1 $F1 \
  --duration $DURATION \
  --amplitude $AMPLITUDE \
  --output $OUTPUT \
  --headless

echo "✅ Implicit chirp test complete"

# =========================================
# Summary
# =========================================
echo ""
echo "========================================="
echo "✅ All Chirp Tests Complete!"
echo "========================================="
echo ""
echo "Results saved to:"
echo "  $OUTPUT/gazebo_pd_low/"
echo "  $OUTPUT/mlp/"
echo "  $OUTPUT/lstm/"
echo "  $OUTPUT/implicit/"
echo ""
echo "Each directory contains:"
echo "  - chirp_results_*.json (raw data)"
echo "  - chirp_plots_*/ (visualizations)"
echo "      - chirp_time_domain.png"
echo "      - chirp_frequency_analysis.png"
echo "      - chirp_spectrogram.png (IsaacLab only)"
echo ""
echo "View plots:"
echo "  eog $OUTPUT/*/chirp_plots_*/chirp_time_domain.png"
echo "  eog $OUTPUT/*/chirp_plots_*/chirp_frequency_analysis.png"
echo ""
echo "Analysis:"
echo "  - Compare tracking error vs frequency"
echo "  - Identify bandwidth limits"
echo "  - Detect resonances"
echo ""

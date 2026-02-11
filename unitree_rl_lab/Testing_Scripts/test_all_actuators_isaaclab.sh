#!/bin/bash
# Test all IsaacLab actuator types

set -e

echo "========================================="
echo "Test All IsaacLab Actuators"
echo "========================================="
echo ""
echo "This will test 3 actuator types:"
echo "  1. MLP (Neural Network, Kp=25, Kd=0.5)"
echo "  2. LSTM (Neural Network, Kp=25, Kd=0.5)"
echo "  3. Implicit (IdealPD, Kp=160, Kd=5)"
echo ""
echo "Each test includes:"
echo "  - Step response"
echo "  - Sinusoidal tracking (1 Hz, 5 Hz)"
echo "  - Frequency sweep (0.5-10 Hz)"
echo ""
echo "Total duration: ~3 minutes per actuator"
echo ""

read -p "Continue? (y/n): " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Cancelled."
    exit 0
fi

# Test MLP actuator
echo ""
echo "========================================="
echo "Testing MLP Actuator"
echo "========================================="
python scripts/actuator_comparison/test_isaaclab_actuators.py \
  --actuator mlp \
  --joint FL_hip_joint \
  --test all \
  --duration 5.0 \
  --headless

echo "✅ MLP testing complete!"

# Test LSTM actuator
echo ""
echo "========================================="
echo "Testing LSTM Actuator"
echo "========================================="
python scripts/actuator_comparison/test_isaaclab_actuators.py \
  --actuator lstm \
  --joint FL_hip_joint \
  --test all \
  --duration 5.0 \
  --headless

echo "✅ LSTM testing complete!"

# Test Implicit actuator
echo ""
echo "========================================="
echo "Testing Implicit Actuator"
echo "========================================="
python scripts/actuator_comparison/test_isaaclab_actuators.py \
  --actuator implicit \
  --joint FL_hip_joint \
  --test all \
  --duration 5.0 \
  --headless

echo "✅ Implicit testing complete!"

# Summary
echo ""
echo "========================================="
echo "All IsaacLab Actuator Tests Complete!"
echo "========================================="
echo ""
echo "Results saved to:"
echo "  actuator_analysis/isaaclab/mlp/"
echo "  actuator_analysis/isaaclab/lstm/"
echo "  actuator_analysis/isaaclab/implicit/"
echo ""
echo "Each directory contains:"
echo "  - test_results_*.json (raw data)"
echo "  - plots_*/ (visualization)"
echo ""
echo "Next steps:"
echo "  1. Run Gazebo tests: ./test_gazebo_motors_all.sh"
echo "  2. Compare results: ./compare_all_actuators.sh"
echo ""

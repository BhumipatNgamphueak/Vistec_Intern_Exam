#!/bin/bash
# Test Gazebo motors with different PD gains

set -e

echo "========================================="
echo "Test Gazebo Motors"
echo "========================================="
echo ""
echo "This will test 2 PD gain configurations:"
echo "  1. LOW gains (Kp=25, Kd=0.5) - for MLP/LSTM comparison"
echo "  2. HIGH gains (Kp=160, Kd=5) - for Implicit comparison"
echo ""
echo "Each test includes:"
echo "  - Step response"
echo "  - Sinusoidal tracking (1 Hz, 5 Hz)"
echo "  - Frequency sweep (0.5-10 Hz)"
echo ""
echo "Total duration: ~1 minute per configuration"
echo ""
echo "⚠️  REQUIREMENTS:"
echo "  - Gazebo must be running with Go2 robot"
echo "  - Joint controllers must be active"
echo "  - ROS 2 sourced"
echo ""

read -p "Is Gazebo ready? (y/n): " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Please start Gazebo first:"
    echo "  ros2 launch go2_gazebo go2_control.launch.py"
    exit 0
fi

# Test with LOW gains
echo ""
echo "========================================="
echo "Testing with LOW PD Gains (Kp=25, Kd=0.5)"
echo "========================================="
echo ""
echo "⚠️  Make sure Gazebo is using go2_controllers_mlp_lstm.yaml!"
echo ""
read -p "LOW gains loaded? (y/n): " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Please load correct controller config and restart."
    exit 0
fi

python scripts/actuator_comparison/test_gazebo_motors.py \
  --joint FL_hip_joint \
  --pd_gains low \
  --test all \
  --duration 5.0

echo "✅ LOW gains testing complete!"

# Test with HIGH gains
echo ""
echo "========================================="
echo "Testing with HIGH PD Gains (Kp=160, Kd=5)"
echo "========================================="
echo ""
echo "⚠️  Now switch Gazebo to go2_controllers_implicit.yaml!"
echo "     1. Stop current controller"
echo "     2. Load implicit config"
echo "     3. Restart controller"
echo ""
read -p "HIGH gains loaded? (y/n): " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Please load correct controller config and restart."
    exit 0
fi

python scripts/actuator_comparison/test_gazebo_motors.py \
  --joint FL_hip_joint \
  --pd_gains high \
  --test all \
  --duration 5.0

echo "✅ HIGH gains testing complete!"

# Summary
echo ""
echo "========================================="
echo "All Gazebo Motor Tests Complete!"
echo "========================================="
echo ""
echo "Results saved to:"
echo "  actuator_analysis/gazebo/pd_low/"
echo "  actuator_analysis/gazebo/pd_high/"
echo ""
echo "Each directory contains:"
echo "  - test_results_*.json (raw data)"
echo "  - plots_*/ (visualization)"
echo ""
echo "Next steps:"
echo "  Run comparison: ./compare_all_actuators.sh"
echo ""

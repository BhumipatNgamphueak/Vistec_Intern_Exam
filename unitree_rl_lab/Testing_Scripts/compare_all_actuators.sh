#!/bin/bash
# Compare all actuator responses between IsaacLab and Gazebo

set -e

echo "========================================="
echo "Compare Actuators: IsaacLab vs Gazebo"
echo "========================================="
echo ""

# Find latest test results
echo "Finding latest test results..."
echo ""

# Find latest IsaacLab results
ISAAC_MLP=$(find actuator_analysis/isaaclab/mlp/ -name "test_results_*.json" 2>/dev/null | sort | tail -n 1)
ISAAC_LSTM=$(find actuator_analysis/isaaclab/lstm/ -name "test_results_*.json" 2>/dev/null | sort | tail -n 1)
ISAAC_IMPLICIT=$(find actuator_analysis/isaaclab/implicit/ -name "test_results_*.json" 2>/dev/null | sort | tail -n 1)

# Find latest Gazebo results
GAZEBO_LOW=$(find actuator_analysis/gazebo/pd_low/ -name "test_results_*.json" 2>/dev/null | sort | tail -n 1)
GAZEBO_HIGH=$(find actuator_analysis/gazebo/pd_high/ -name "test_results_*.json" 2>/dev/null | sort | tail -n 1)

# Check if files exist
missing=false

if [ -z "$ISAAC_MLP" ]; then
    echo "❌ IsaacLab MLP results not found"
    missing=true
else
    echo "✅ IsaacLab MLP: $ISAAC_MLP"
fi

if [ -z "$ISAAC_LSTM" ]; then
    echo "❌ IsaacLab LSTM results not found"
    missing=true
else
    echo "✅ IsaacLab LSTM: $ISAAC_LSTM"
fi

if [ -z "$ISAAC_IMPLICIT" ]; then
    echo "❌ IsaacLab Implicit results not found"
    missing=true
else
    echo "✅ IsaacLab Implicit: $ISAAC_IMPLICIT"
fi

if [ -z "$GAZEBO_LOW" ]; then
    echo "❌ Gazebo LOW gains results not found"
    missing=true
else
    echo "✅ Gazebo LOW: $GAZEBO_LOW"
fi

if [ -z "$GAZEBO_HIGH" ]; then
    echo "❌ Gazebo HIGH gains results not found"
    missing=true
else
    echo "✅ Gazebo HIGH: $GAZEBO_HIGH"
fi

echo ""

if [ "$missing" = true ]; then
    echo "Missing test results! Please run:"
    echo "  1. ./test_all_actuators_isaaclab.sh"
    echo "  2. ./test_gazebo_motors_all.sh"
    exit 1
fi

# Run comparisons
echo "Running comparisons..."
echo ""

# Compare MLP vs Gazebo LOW
echo "1/3: Comparing MLP (IsaacLab) vs LOW gains (Gazebo)..."
python scripts/actuator_comparison/compare_actuators.py \
  --isaac "$ISAAC_MLP" \
  --gazebo "$GAZEBO_LOW" \
  --output "actuator_analysis/comparison/mlp_vs_gazebo_low"
echo ""

# Compare LSTM vs Gazebo LOW
echo "2/3: Comparing LSTM (IsaacLab) vs LOW gains (Gazebo)..."
python scripts/actuator_comparison/compare_actuators.py \
  --isaac "$ISAAC_LSTM" \
  --gazebo "$GAZEBO_LOW" \
  --output "actuator_analysis/comparison/lstm_vs_gazebo_low"
echo ""

# Compare Implicit vs Gazebo HIGH
echo "3/3: Comparing Implicit (IsaacLab) vs HIGH gains (Gazebo)..."
python scripts/actuator_comparison/compare_actuators.py \
  --isaac "$ISAAC_IMPLICIT" \
  --gazebo "$GAZEBO_HIGH" \
  --output "actuator_analysis/comparison/implicit_vs_gazebo_high"
echo ""

# Summary
echo "========================================="
echo "All Comparisons Complete!"
echo "========================================="
echo ""
echo "Results saved to:"
echo "  actuator_analysis/comparison/mlp_vs_gazebo_low/"
echo "  actuator_analysis/comparison/lstm_vs_gazebo_low/"
echo "  actuator_analysis/comparison/implicit_vs_gazebo_high/"
echo ""
echo "Each directory contains:"
echo "  - comparison_step_response.png"
echo "  - comparison_sine_1hz.png"
echo "  - comparison_sine_5hz.png"
echo "  - comparison_metrics.csv"
echo ""
echo "View results:"
echo "  - Open PNG files to visualize differences"
echo "  - Check CSV for quantitative metrics"
echo ""

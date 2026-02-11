#!/bin/bash
# Complete workflow: Collect and compare actuator responses (hanging configuration)

set -e

JOINT="FL_hip_joint"
DURATION=5.0
OUTPUT_BASE="actuator_analysis_hanging"

echo "========================================="
echo "Hanging Actuator Comparison Workflow"
echo "========================================="
echo ""
echo "This workflow will:"
echo "  1. Collect Gazebo motor data (effort controller)"
echo "  2. Collect IsaacLab MLP actuator data"
echo "  3. Collect IsaacLab LSTM actuator data"
echo "  4. Compare MLP vs Gazebo"
echo "  5. Compare LSTM vs Gazebo"
echo ""
echo "Configuration:"
echo "  Joint: $JOINT"
echo "  Duration: ${DURATION}s per test"
echo "  Output: $OUTPUT_BASE/"
echo ""
echo "Prerequisites:"
echo "  - Gazebo running with robot at height"
echo "  - IsaacLab environment activated"
echo ""

# =========================================
# Step 1: Gazebo Data Collection
# =========================================
echo "========================================="
echo "Step 1/5: Collect Gazebo Data"
echo "========================================="
echo ""
echo "⚠️  Gazebo must be running with:"
echo "    ros2 launch go2_gazebo go2_control.launch.py \\"
echo "      spawn_z:=1.5 \\"
echo "      controller_config:=go2_controllers_mlp_lstm.yaml"
echo ""
read -p "Is Gazebo ready with hanging robot? (y/n): " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "❌ Please start Gazebo first."
    exit 1
fi

echo ""
echo "Collecting Gazebo motor response data..."
python scripts/actuator_comparison/test_gazebo_motors.py \
  --joint $JOINT \
  --pd_gains low \
  --test all \
  --duration $DURATION \
  --output "$OUTPUT_BASE/gazebo"

if [ $? -eq 0 ]; then
    echo "✅ Gazebo data collected successfully"
else
    echo "❌ Gazebo data collection failed"
    exit 1
fi

# =========================================
# Step 2: MLP Data Collection
# =========================================
echo ""
echo "========================================="
echo "Step 2/5: Collect MLP Actuator Data"
echo "========================================="
echo ""
echo "Testing MLP actuator (Kp=25.0, Kd=0.5)..."
echo "Robot will hang at 1.5m height automatically."
echo ""

python scripts/actuator_comparison/test_isaaclab_actuators.py \
  --actuator mlp \
  --joint $JOINT \
  --test all \
  --duration $DURATION \
  --output "$OUTPUT_BASE/isaaclab" \
  --headless

if [ $? -eq 0 ]; then
    echo "✅ MLP data collected successfully"
else
    echo "❌ MLP data collection failed"
    exit 1
fi

# =========================================
# Step 3: LSTM Data Collection
# =========================================
echo ""
echo "========================================="
echo "Step 3/5: Collect LSTM Actuator Data"
echo "========================================="
echo ""
echo "Testing LSTM actuator (Kp=25.0, Kd=0.5)..."
echo "Robot will hang at 1.5m height automatically."
echo ""

python scripts/actuator_comparison/test_isaaclab_actuators.py \
  --actuator lstm \
  --joint $JOINT \
  --test all \
  --duration $DURATION \
  --output "$OUTPUT_BASE/isaaclab" \
  --headless

if [ $? -eq 0 ]; then
    echo "✅ LSTM data collected successfully"
else
    echo "❌ LSTM data collection failed"
    exit 1
fi

# =========================================
# Step 4: Compare MLP vs Gazebo
# =========================================
echo ""
echo "========================================="
echo "Step 4/5: Compare MLP vs Gazebo"
echo "========================================="
echo ""

# Find latest result files
MLP_FILE=$(ls -t "$OUTPUT_BASE/isaaclab/mlp/test_results_"*.json 2>/dev/null | head -1)
GAZEBO_FILE=$(ls -t "$OUTPUT_BASE/gazebo/pd_low/test_results_"*.json 2>/dev/null | head -1)

if [ -z "$MLP_FILE" ] || [ -z "$GAZEBO_FILE" ]; then
    echo "❌ Result files not found"
    echo "   MLP: $MLP_FILE"
    echo "   Gazebo: $GAZEBO_FILE"
    exit 1
fi

echo "Comparing:"
echo "  MLP:    $MLP_FILE"
echo "  Gazebo: $GAZEBO_FILE"
echo ""

python scripts/actuator_comparison/compare_actuators.py \
  --isaac "$MLP_FILE" \
  --gazebo "$GAZEBO_FILE" \
  --output "$OUTPUT_BASE/comparison/mlp_vs_gazebo"

if [ $? -eq 0 ]; then
    echo "✅ MLP comparison complete"
else
    echo "❌ MLP comparison failed"
    exit 1
fi

# =========================================
# Step 5: Compare LSTM vs Gazebo
# =========================================
echo ""
echo "========================================="
echo "Step 5/5: Compare LSTM vs Gazebo"
echo "========================================="
echo ""

LSTM_FILE=$(ls -t "$OUTPUT_BASE/isaaclab/lstm/test_results_"*.json 2>/dev/null | head -1)

if [ -z "$LSTM_FILE" ]; then
    echo "❌ LSTM result file not found"
    exit 1
fi

echo "Comparing:"
echo "  LSTM:   $LSTM_FILE"
echo "  Gazebo: $GAZEBO_FILE"
echo ""

python scripts/actuator_comparison/compare_actuators.py \
  --isaac "$LSTM_FILE" \
  --gazebo "$GAZEBO_FILE" \
  --output "$OUTPUT_BASE/comparison/lstm_vs_gazebo"

if [ $? -eq 0 ]; then
    echo "✅ LSTM comparison complete"
else
    echo "❌ LSTM comparison failed"
    exit 1
fi

# =========================================
# Summary
# =========================================
echo ""
echo "========================================="
echo "✅ Workflow Complete!"
echo "========================================="
echo ""
echo "All data collected and compared successfully!"
echo ""
echo "Results directory structure:"
echo "  $OUTPUT_BASE/"
echo "  ├── gazebo/pd_low/         (Gazebo motor data)"
echo "  ├── isaaclab/mlp/          (MLP actuator data)"
echo "  ├── isaaclab/lstm/         (LSTM actuator data)"
echo "  ├── comparison/mlp_vs_gazebo/"
echo "  └── comparison/lstm_vs_gazebo/"
echo ""
echo "View comparison plots:"
echo "  eog $OUTPUT_BASE/comparison/mlp_vs_gazebo/*.png"
echo "  eog $OUTPUT_BASE/comparison/lstm_vs_gazebo/*.png"
echo ""
echo "View metrics:"
echo "  cat $OUTPUT_BASE/comparison/mlp_vs_gazebo/comparison_metrics.csv"
echo "  cat $OUTPUT_BASE/comparison/lstm_vs_gazebo/comparison_metrics.csv"
echo ""

# Display metrics summaries
echo "========================================="
echo "MLP vs Gazebo Metrics Summary:"
echo "========================================="
if [ -f "$OUTPUT_BASE/comparison/mlp_vs_gazebo/comparison_metrics.csv" ]; then
    cat "$OUTPUT_BASE/comparison/mlp_vs_gazebo/comparison_metrics.csv"
else
    echo "Metrics file not found"
fi

echo ""
echo "========================================="
echo "LSTM vs Gazebo Metrics Summary:"
echo "========================================="
if [ -f "$OUTPUT_BASE/comparison/lstm_vs_gazebo/comparison_metrics.csv" ]; then
    cat "$OUTPUT_BASE/comparison/lstm_vs_gazebo/comparison_metrics.csv"
else
    echo "Metrics file not found"
fi

echo ""
echo "========================================="
echo "Next Steps:"
echo "========================================="
echo "1. Review comparison plots visually"
echo "2. Analyze metrics CSV for quantitative differences"
echo "3. If match is good, proceed with policy data collection"
echo "4. If match is poor, tune Gazebo PD gains"
echo ""

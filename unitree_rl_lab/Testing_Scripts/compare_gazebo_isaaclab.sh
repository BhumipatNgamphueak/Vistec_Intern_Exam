#!/bin/bash
# Complete workflow: Collect and compare Gazebo (Vistec_ex_ws) vs IsaacLab (unitree_rl_lab)
# Uses SAME PD gains in both platforms

set -e

echo "========================================================================"
echo "Gazebo (Vistec_ex_ws) ↔ IsaacLab (unitree_rl_lab) Comparison Workflow"
echo "========================================================================"
echo ""
echo "This workflow collects motor response data with MATCHING PD gains"
echo "in both Gazebo and IsaacLab for fair comparison."
echo ""
echo "Select PD gains configuration:"
echo "  1) LOW Gains (Kp=25, Kd=0.5)  - For MLP/LSTM comparison"
echo "  2) HIGH Gains (Kp=160, Kd=5)  - For Implicit comparison"
echo "  3) BOTH (run sequentially)"
echo ""

read -p "Select option (1-3): " -n 1 -r
echo

JOINT="FL_hip_joint"
F0=0.1
F1=20.0
DURATION=10.0
AMPLITUDE=0.3
OUTPUT_BASE="gazebo_isaaclab_comparison"

# Function to run LOW gains workflow
run_low_gains() {
    echo ""
    echo "========================================================================"
    echo "LOW Gains Workflow (Kp=25, Kd=0.5)"
    echo "========================================================================"

    # Step 1: Gazebo
    echo ""
    echo "Step 1/3: Collect Gazebo Data (LOW gains)"
    echo "------------------------------------------------------------------------"
    echo ""
    echo "⚠️  Gazebo must be running in Vistec_ex_ws with:"
    echo "    cd ~/Vistec_ex_ws && source install/setup.bash"
    echo "    ros2 launch go2_gazebo go2_control.launch.py \\"
    echo "      spawn_z:=1.5 \\"
    echo "      controller_config:=go2_controllers_mlp_lstm.yaml"
    echo ""
    read -p "Is Gazebo ready with LOW gains? (y/n): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "❌ Please start Gazebo first."
        exit 1
    fi

    echo ""
    echo "Collecting Gazebo chirp data..."
    python scripts/actuator_comparison/chirp_test_gazebo.py \
      --joint $JOINT \
      --pd_gains low \
      --f0 $F0 \
      --f1 $F1 \
      --duration $DURATION \
      --amplitude $AMPLITUDE \
      --output "$OUTPUT_BASE/gazebo"

    echo "✅ Gazebo LOW gains data collected"

    # Step 2: IsaacLab MLP
    echo ""
    echo "Step 2/3: Collect IsaacLab Data (MLP actuator, Kp=25, Kd=0.5)"
    echo "------------------------------------------------------------------------"
    echo ""
    echo "Testing MLP actuator (matches Gazebo LOW gains)..."

    python scripts/actuator_comparison/chirp_test_isaaclab.py \
      --actuator mlp \
      --joint $JOINT \
      --f0 $F0 \
      --f1 $F1 \
      --duration $DURATION \
      --amplitude $AMPLITUDE \
      --output "$OUTPUT_BASE/isaaclab" \
      --headless

    echo "✅ IsaacLab MLP data collected"

    # Step 3: Compare
    echo ""
    echo "Step 3/3: Compare Results"
    echo "------------------------------------------------------------------------"
    echo ""

    GAZEBO_FILE=$(ls -t "$OUTPUT_BASE/gazebo/gazebo_pd_low/chirp_results_"*.json 2>/dev/null | head -1)
    ISAAC_FILE=$(ls -t "$OUTPUT_BASE/isaaclab/mlp/chirp_results_"*.json 2>/dev/null | head -1)

    if [ -z "$GAZEBO_FILE" ] || [ -z "$ISAAC_FILE" ]; then
        echo "❌ Data files not found!"
        exit 1
    fi

    echo "Comparing:"
    echo "  Gazebo:   $GAZEBO_FILE"
    echo "  IsaacLab: $ISAAC_FILE"
    echo ""

    python scripts/actuator_comparison/compare_chirp_data.py \
      --gazebo "$GAZEBO_FILE" \
      --isaaclab "$ISAAC_FILE" \
      --output "$OUTPUT_BASE/comparison_low_gains" \
      --label "LOW Gains (Kp=25.0, Kd=0.5)"

    echo "✅ LOW gains comparison complete!"
    echo ""
    echo "Results: $OUTPUT_BASE/comparison_low_gains/"
}

# Function to run HIGH gains workflow
run_high_gains() {
    echo ""
    echo "========================================================================"
    echo "HIGH Gains Workflow (Kp=160, Kd=5)"
    echo "========================================================================"

    # Step 1: Gazebo
    echo ""
    echo "Step 1/3: Collect Gazebo Data (HIGH gains)"
    echo "------------------------------------------------------------------------"
    echo ""
    echo "⚠️  Gazebo must be running in Vistec_ex_ws with:"
    echo "    cd ~/Vistec_ex_ws && source install/setup.bash"
    echo "    ros2 launch go2_gazebo go2_control.launch.py \\"
    echo "      spawn_z:=1.5 \\"
    echo "      controller_config:=go2_controllers_implicit.yaml"
    echo ""
    read -p "Is Gazebo ready with HIGH gains? (y/n): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "❌ Please start Gazebo first."
        exit 1
    fi

    echo ""
    echo "Collecting Gazebo chirp data..."
    python scripts/actuator_comparison/chirp_test_gazebo.py \
      --joint $JOINT \
      --pd_gains high \
      --f0 $F0 \
      --f1 $F1 \
      --duration $DURATION \
      --amplitude $AMPLITUDE \
      --output "$OUTPUT_BASE/gazebo"

    echo "✅ Gazebo HIGH gains data collected"

    # Step 2: IsaacLab Implicit
    echo ""
    echo "Step 2/3: Collect IsaacLab Data (Implicit actuator, Kp=160, Kd=5)"
    echo "------------------------------------------------------------------------"
    echo ""
    echo "Testing Implicit actuator (matches Gazebo HIGH gains)..."

    python scripts/actuator_comparison/chirp_test_isaaclab.py \
      --actuator implicit \
      --joint $JOINT \
      --f0 $F0 \
      --f1 $F1 \
      --duration $DURATION \
      --amplitude $AMPLITUDE \
      --output "$OUTPUT_BASE/isaaclab" \
      --headless

    echo "✅ IsaacLab Implicit data collected"

    # Step 3: Compare
    echo ""
    echo "Step 3/3: Compare Results"
    echo "------------------------------------------------------------------------"
    echo ""

    GAZEBO_FILE=$(ls -t "$OUTPUT_BASE/gazebo/gazebo_pd_high/chirp_results_"*.json 2>/dev/null | head -1)
    ISAAC_FILE=$(ls -t "$OUTPUT_BASE/isaaclab/implicit/chirp_results_"*.json 2>/dev/null | head -1)

    if [ -z "$GAZEBO_FILE" ] || [ -z "$ISAAC_FILE" ]; then
        echo "❌ Data files not found!"
        exit 1
    fi

    echo "Comparing:"
    echo "  Gazebo:   $GAZEBO_FILE"
    echo "  IsaacLab: $ISAAC_FILE"
    echo ""

    python scripts/actuator_comparison/compare_chirp_data.py \
      --gazebo "$GAZEBO_FILE" \
      --isaaclab "$ISAAC_FILE" \
      --output "$OUTPUT_BASE/comparison_high_gains" \
      --label "HIGH Gains (Kp=160.0, Kd=5.0)"

    echo "✅ HIGH gains comparison complete!"
    echo ""
    echo "Results: $OUTPUT_BASE/comparison_high_gains/"
}

# Execute based on selection
case $REPLY in
  1)
    run_low_gains
    ;;
  2)
    run_high_gains
    ;;
  3)
    run_low_gains
    run_high_gains
    ;;
  *)
    echo "Invalid selection"
    exit 1
    ;;
esac

# Final summary
echo ""
echo "========================================================================"
echo "✅ Workflow Complete!"
echo "========================================================================"
echo ""
echo "Results directory: $OUTPUT_BASE/"
echo ""
echo "Output structure:"
echo "  $OUTPUT_BASE/"
echo "  ├── gazebo/                     (Gazebo motor data)"
echo "  ├── isaaclab/                   (IsaacLab actuator data)"
echo "  ├── comparison_low_gains/       (Comparison results LOW)"
echo "  │   ├── comparison_time_domain.png"
echo "  │   ├── comparison_frequency_domain.png"
echo "  │   ├── comparison_bandwidth.png"
echo "  │   ├── comparison_metrics.csv"
echo "  │   └── summary.txt"
echo "  └── comparison_high_gains/      (Comparison results HIGH)"
echo "      └── ... (same structure)"
echo ""
echo "View results:"
echo "  cat $OUTPUT_BASE/comparison_*/comparison_metrics.csv"
echo "  cat $OUTPUT_BASE/comparison_*/summary.txt"
echo ""
echo "View plots:"
echo "  eog $OUTPUT_BASE/comparison_*/comparison_*.png"
echo ""

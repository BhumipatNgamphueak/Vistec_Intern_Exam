#!/bin/bash
# Run chirp tests for all actuators in Isaac Lab (hanging configuration)

set -e

echo "========================================"
echo "Go2 Chirp Motor Tests - Isaac Lab"
echo "Hanging Configuration"
echo "========================================"
echo ""

# Test parameters
DURATION=10.0
F0=0.1
F1=20.0
AMPLITUDE=0.5
JOINT="FR_hip_joint"
OUTPUT_DIR="chirp_data_isaaclab"

echo "Test Configuration:"
echo "  Joint: $JOINT"
echo "  Duration: ${DURATION}s"
echo "  Frequency: ${F0} Hz → ${F1} Hz"
echo "  Amplitude: ${AMPLITUDE} rad (≈28.6°)"
echo "  Output: $OUTPUT_DIR/"
echo ""

# Interactive menu
echo "Select test mode:"
echo "1) MLP Actuator only"
echo "2) LSTM Actuator only"
echo "3) Implicit Actuator only"
echo "4) All actuators (sequential)"
echo "5) Custom joint test (all actuators)"
echo "6) With visualization (slower)"
echo ""
read -p "Enter choice [1-6]: " choice

case $choice in
    1)
        echo ""
        echo "Testing MLP Actuator..."
        python test_chirp_all_actuators.py \
            --actuator mlp \
            --duration $DURATION \
            --f0 $F0 \
            --f1 $F1 \
            --amplitude $AMPLITUDE \
            --test_joint $JOINT \
            --output_dir $OUTPUT_DIR
        ;;
    2)
        echo ""
        echo "Testing LSTM Actuator..."
        python test_chirp_all_actuators.py \
            --actuator lstm \
            --duration $DURATION \
            --f0 $F0 \
            --f1 $F1 \
            --amplitude $AMPLITUDE \
            --test_joint $JOINT \
            --output_dir $OUTPUT_DIR
        ;;
    3)
        echo ""
        echo "Testing Implicit Actuator..."
        python test_chirp_all_actuators.py \
            --actuator implicit \
            --duration $DURATION \
            --f0 $F0 \
            --f1 $F1 \
            --amplitude $AMPLITUDE \
            --test_joint $JOINT \
            --output_dir $OUTPUT_DIR
        ;;
    4)
        echo ""
        echo "Testing all actuators sequentially..."
        python test_chirp_all_actuators.py \
            --actuator all \
            --duration $DURATION \
            --f0 $F0 \
            --f1 $F1 \
            --amplitude $AMPLITUDE \
            --test_joint $JOINT \
            --output_dir $OUTPUT_DIR
        ;;
    5)
        echo ""
        read -p "Enter joint name (e.g., FR_hip_joint, FR_thigh_joint): " custom_joint
        echo ""
        echo "Testing all actuators on $custom_joint..."
        python test_chirp_all_actuators.py \
            --actuator all \
            --duration $DURATION \
            --f0 $F0 \
            --f1 $F1 \
            --amplitude $AMPLITUDE \
            --test_joint $custom_joint \
            --output_dir $OUTPUT_DIR
        ;;
    6)
        echo ""
        read -p "Enter actuator (mlp/lstm/implicit/all): " actuator_choice
        echo ""
        echo "Testing with visualization (slower)..."
        python test_chirp_all_actuators.py \
            --actuator $actuator_choice \
            --duration $DURATION \
            --f0 $F0 \
            --f1 $F1 \
            --amplitude $AMPLITUDE \
            --test_joint $JOINT \
            --output_dir $OUTPUT_DIR \
            --enable_cameras
        ;;
    *)
        echo "Invalid choice. Exiting..."
        exit 1
        ;;
esac

echo ""
echo "========================================"
echo "Tests Complete!"
echo "========================================"
echo ""
echo "Data saved to: $OUTPUT_DIR/"
echo ""
echo "Next steps:"
echo "1. Run chirp test in Gazebo (in Vistec_ex_ws)"
echo "2. Compare data using: python compare_chirp_isaac_gazebo.py"
echo ""

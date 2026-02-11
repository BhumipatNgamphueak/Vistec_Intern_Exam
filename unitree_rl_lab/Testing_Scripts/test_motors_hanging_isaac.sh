#!/bin/bash
# Test Go2 motors in IsaacLab (hanging configuration)

set -e

echo "========================================="
echo "Go2 Motor Testing - IsaacLab Hanging"
echo "========================================="
echo ""
echo "This will hang the Go2 robot at 1.5m height"
echo "and test motor commands without ground contact."
echo ""
echo "Options:"
echo "  1. Test single joint (sine wave)"
echo "  2. Test single joint (step response)"
echo "  3. Test ALL joints (sine wave, 5s each)"
echo "  4. Custom test"
echo ""

read -p "Select option (1-4): " -n 1 -r
echo

case $REPLY in
  1)
    echo ""
    echo "Available joints:"
    echo "  FL_hip_joint, FL_thigh_joint, FL_calf_joint"
    echo "  FR_hip_joint, FR_thigh_joint, FR_calf_joint"
    echo "  RL_hip_joint, RL_thigh_joint, RL_calf_joint"
    echo "  RR_hip_joint, RR_thigh_joint, RR_calf_joint"
    echo ""
    read -p "Enter joint name: " joint_name

    echo ""
    echo "Select actuator:"
    echo "  1) MLP (Kp=25, Kd=0.5)"
    echo "  2) LSTM (Kp=25, Kd=0.5)"
    echo "  3) Implicit (Kp=160, Kd=5)"
    echo ""
    read -p "Select (1-3): " -n 1 -r
    echo

    case $REPLY in
      1) actuator="mlp" ;;
      2) actuator="lstm" ;;
      3) actuator="implicit" ;;
      *) echo "Invalid choice"; exit 1 ;;
    esac

    echo ""
    echo "Testing $joint_name with $actuator actuator (sine wave)..."
    python scripts/motor_testing/test_motors_hanging.py \
      --actuator $actuator \
      --joint $joint_name \
      --motion sine
    ;;

  2)
    echo ""
    read -p "Enter joint name: " joint_name

    echo ""
    echo "Select actuator:"
    echo "  1) MLP"
    echo "  2) LSTM"
    echo "  3) Implicit"
    echo ""
    read -p "Select (1-3): " -n 1 -r
    echo

    case $REPLY in
      1) actuator="mlp" ;;
      2) actuator="lstm" ;;
      3) actuator="implicit" ;;
      *) echo "Invalid choice"; exit 1 ;;
    esac

    echo ""
    echo "Testing $joint_name with $actuator actuator (step)..."
    python scripts/motor_testing/test_motors_hanging.py \
      --actuator $actuator \
      --joint $joint_name \
      --motion step
    ;;

  3)
    echo ""
    echo "Select actuator:"
    echo "  1) MLP (Kp=25, Kd=0.5)"
    echo "  2) LSTM (Kp=25, Kd=0.5)"
    echo "  3) Implicit (Kp=160, Kd=5)"
    echo ""
    read -p "Select (1-3): " -n 1 -r
    echo

    case $REPLY in
      1) actuator="mlp" ;;
      2) actuator="lstm" ;;
      3) actuator="implicit" ;;
      *) echo "Invalid choice"; exit 1 ;;
    esac

    echo ""
    echo "Testing ALL joints with $actuator actuator..."
    echo "Each joint will move for 5 seconds."
    echo ""
    python scripts/motor_testing/test_motors_hanging.py \
      --actuator $actuator \
      --joint all \
      --motion sine
    ;;

  4)
    echo ""
    echo "Custom test"
    echo ""
    read -p "Actuator (mlp/lstm/implicit): " actuator
    read -p "Joint (or 'all'): " joint
    read -p "Motion (sine/step/sweep): " motion

    echo ""
    echo "Running custom test..."
    python scripts/motor_testing/test_motors_hanging.py \
      --actuator $actuator \
      --joint $joint \
      --motion $motion
    ;;

  *)
    echo "Invalid choice"
    exit 1
    ;;
esac

echo ""
echo "✅ Motor test complete!"

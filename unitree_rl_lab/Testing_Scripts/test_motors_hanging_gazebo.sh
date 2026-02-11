#!/bin/bash
# Test Go2 motors in Gazebo (hanging configuration)

set -e

echo "========================================="
echo "Go2 Motor Testing - Gazebo Hanging"
echo "========================================="
echo ""
echo "⚠️  PREREQUISITES:"
echo "  1. Gazebo must be running"
echo "  2. Go2 robot spawned at height (spawn_z:=1.5)"
echo "  3. Joint controllers loaded"
echo "  4. ROS 2 sourced"
echo ""
echo "To start Gazebo with hanging robot:"
echo "  ros2 launch go2_gazebo go2_control.launch.py \\"
echo "    spawn_z:=1.5 \\"
echo "    controller_config:=go2_controllers_mlp_lstm.yaml"
echo ""

read -p "Is Gazebo ready with robot hanging? (y/n): " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Please start Gazebo first."
    exit 0
fi

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
    echo "Testing $joint_name (sine wave)..."
    python scripts/motor_testing/test_motors_hanging_gazebo.py \
      --joint $joint_name \
      --motion sine \
      --duration 20
    ;;

  2)
    echo ""
    read -p "Enter joint name: " joint_name

    echo ""
    echo "Testing $joint_name (step)..."
    python scripts/motor_testing/test_motors_hanging_gazebo.py \
      --joint $joint_name \
      --motion step \
      --duration 20
    ;;

  3)
    echo ""
    echo "Testing ALL joints..."
    echo "Each joint will move for 5 seconds."
    echo "Total duration: ~60 seconds"
    echo ""
    python scripts/motor_testing/test_motors_hanging_gazebo.py \
      --motion sine \
      --duration 60
    ;;

  4)
    echo ""
    echo "Custom test"
    echo ""
    read -p "Joint (or leave empty for all): " joint
    read -p "Motion (sine/step/sweep): " motion
    read -p "Duration (seconds): " duration

    echo ""
    echo "Running custom test..."
    if [ -z "$joint" ]; then
        python scripts/motor_testing/test_motors_hanging_gazebo.py \
          --motion $motion \
          --duration $duration
    else
        python scripts/motor_testing/test_motors_hanging_gazebo.py \
          --joint $joint \
          --motion $motion \
          --duration $duration
    fi
    ;;

  *)
    echo "Invalid choice"
    exit 1
    ;;
esac

echo ""
echo "✅ Motor test complete!"

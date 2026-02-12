#!/bin/bash
# Send velocity commands to Gazebo simulation via ROS 2
# Simplified: 4 main tasks only

set -e

echo "=========================================="
echo "Gazebo Velocity Command Sender"
echo "=========================================="
echo ""

# Check if ROS 2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "ERROR: ROS 2 not sourced!"
    echo "Run: source /opt/ros/humble/setup.bash"
    exit 1
fi

# Interactive menu - 4 Main Tasks
echo "🎯 4 LOCOMOTION TASKS:"
echo ""
echo "1) Task 1: Standing        (0.0, 0.0, 0.0)"
echo "2) Task 2: Walking         (1.0 m/s forward)"
echo "3) Task 3: Turn in Place   (1.0 rad/s CCW)"
echo "4) Task 4: Walk + Turn     (0.8 m/s, 0.6 rad/s)"
echo ""
echo "5) Custom command"
echo ""
read -p "Enter choice [1-5]: " choice

case $choice in
    # TASK 1: Standing
    1)
        echo ""
        echo "TASK 1: Standing"
        LIN_X=0.0
        LIN_Y=0.0
        ANG_Z=0.0
        ;;
    # TASK 2: Walking
    2)
        echo ""
        echo "TASK 2: Walking (1.0 m/s)"
        LIN_X=1.0
        LIN_Y=0.0
        ANG_Z=0.0
        ;;
    # TASK 3: Turn in Place
    3)
        echo ""
        echo "TASK 3: Turn in Place (1.0 rad/s CCW)"
        LIN_X=0.0
        LIN_Y=0.0
        ANG_Z=1.0
        ;;
    # TASK 4: Walk + Turn
    4)
        echo ""
        echo "TASK 4: Walk + Turn (arc)"
        LIN_X=0.8
        LIN_Y=0.0
        ANG_Z=0.6
        ;;
    # Custom
    5)
        echo ""
        read -p "Enter linear_x (m/s, -1.0 to 1.0): " LIN_X
        read -p "Enter linear_y (m/s, -0.4 to 0.4): " LIN_Y
        read -p "Enter angular_z (rad/s, -1.0 to 1.0): " ANG_Z
        ;;
    *)
        echo "Invalid choice. Exiting..."
        exit 1
        ;;
esac

echo ""
echo "Sending velocity command:"
echo "  linear.x:  ${LIN_X} m/s"
echo "  linear.y:  ${LIN_Y} m/s"
echo "  angular.z: ${ANG_Z} rad/s"
echo ""
echo "Publishing to /cmd_vel at 10 Hz..."
echo "Press Ctrl+C to stop"
echo ""

# Send command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "linear:
  x: ${LIN_X}
  y: ${LIN_Y}
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: ${ANG_Z}" \
    --rate 10

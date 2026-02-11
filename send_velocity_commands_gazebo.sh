#!/bin/bash
# Send velocity commands to Gazebo simulation via ROS 2

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

# Interactive menu - Matches 4 Training Tasks
echo "🎯 QUICK SELECT (4 main tasks):"
echo "  1) Task 1: Standing     | 3) Task 2: Walk normal  "
echo "  7) Task 3: Turn normal  | 10) Task 4: Walk+Turn   "
echo ""
echo "📋 ALL OPTIONS (14 variants):"
echo ""
echo "=== TASK 1: Standing ==="
echo "1) Stand still (0.0, 0.0, 0.0) ⭐"
echo ""
echo "=== TASK 2: Walking ==="
echo "2) Walk slow (0.5 m/s)"
echo "3) Walk normal (1.0 m/s) ⭐"
echo "4) Walk fast (1.5 m/s)"
echo "5) Walk moderate (0.8 m/s)"
echo ""
echo "=== TASK 3: Turn in Place ==="
echo "6) Turn slow CCW (+0.5 rad/s)"
echo "7) Turn normal CCW (+1.0 rad/s) ⭐"
echo "8) Turn normal CW (-1.0 rad/s)"
echo "9) Turn fast CCW (+1.5 rad/s)"
echo ""
echo "=== TASK 4: Walk + Turn ==="
echo "10) Right arc (0.8 m/s, +0.6 rad/s) ⭐"
echo "11) Straight fast (1.2 m/s)"
echo "12) Left arc (0.8 m/s, -0.6 rad/s)"
echo "13) Tight turn (0.5 m/s, +1.0 rad/s)"
echo ""
echo "=== OTHER ==="
echo "14) Custom command"
echo ""
echo "⭐ = Recommended representative for each task"
echo ""
read -p "Enter choice [1-14]: " choice

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
        echo "TASK 2: Walk slow (0.5 m/s)"
        LIN_X=0.5
        LIN_Y=0.0
        ANG_Z=0.0
        ;;
    3)
        echo ""
        echo "TASK 2: Walk normal (1.0 m/s)"
        LIN_X=1.0
        LIN_Y=0.0
        ANG_Z=0.0
        ;;
    4)
        echo ""
        echo "TASK 2: Walk fast (1.5 m/s)"
        LIN_X=1.5
        LIN_Y=0.0
        ANG_Z=0.0
        ;;
    5)
        echo ""
        echo "TASK 2: Walk moderate (0.8 m/s)"
        LIN_X=0.8
        LIN_Y=0.0
        ANG_Z=0.0
        ;;
    # TASK 3: Turn in Place
    6)
        echo ""
        echo "TASK 3: Turn slow CCW (+0.5 rad/s)"
        LIN_X=0.0
        LIN_Y=0.0
        ANG_Z=0.5
        ;;
    7)
        echo ""
        echo "TASK 3: Turn normal CCW (+1.0 rad/s)"
        LIN_X=0.0
        LIN_Y=0.0
        ANG_Z=1.0
        ;;
    8)
        echo ""
        echo "TASK 3: Turn normal CW (-1.0 rad/s)"
        LIN_X=0.0
        LIN_Y=0.0
        ANG_Z=-1.0
        ;;
    9)
        echo ""
        echo "TASK 3: Turn fast CCW (+1.5 rad/s)"
        LIN_X=0.0
        LIN_Y=0.0
        ANG_Z=1.5
        ;;
    # TASK 4: Walk + Turn
    10)
        echo ""
        echo "TASK 4: Right arc (0.8 m/s, +0.6 rad/s)"
        LIN_X=0.8
        LIN_Y=0.0
        ANG_Z=0.6
        ;;
    11)
        echo ""
        echo "TASK 4: Straight fast (1.2 m/s)"
        LIN_X=1.2
        LIN_Y=0.0
        ANG_Z=0.0
        ;;
    12)
        echo ""
        echo "TASK 4: Left arc (0.8 m/s, -0.6 rad/s)"
        LIN_X=0.8
        LIN_Y=0.0
        ANG_Z=-0.6
        ;;
    13)
        echo ""
        echo "TASK 4: Tight turn (0.5 m/s, +1.0 rad/s)"
        LIN_X=0.5
        LIN_Y=0.0
        ANG_Z=1.0
        ;;
    # Custom
    14)
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

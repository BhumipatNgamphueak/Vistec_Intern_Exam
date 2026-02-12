#!/bin/bash
# Test Gazebo deployment with exact training experiment sequences
# Automatically sends time-varying velocity commands matching the 4 training tasks

set -e

echo "=========================================="
echo "Gazebo Task Tester - Training Sequences"
echo "=========================================="
echo ""

# Check if ROS 2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ ERROR: ROS 2 not sourced!"
    echo "Run: source /opt/ros/humble/setup.bash"
    exit 1
fi

# Interactive menu - 4 Main Tasks
echo "🎯 4 TRAINING TASKS:"
echo ""
echo "0) Task 0: Standing (20s)"
echo "   └─ vx=0.0 for entire 20s"
echo ""
echo "1) Task 1: Walking (20s, 4 speeds)"
echo "   ├─ vx=0.5 (5s) → Slow"
echo "   ├─ vx=1.0 (5s) → Normal"
echo "   ├─ vx=1.5 (5s) → Fast"
echo "   └─ vx=0.8 (5s) → Moderate"
echo ""
echo "2) Task 2: Turn in Place (20s, 4 rates)"
echo "   ├─ wz=+0.5 (5s) → Slow CCW"
echo "   ├─ wz=+1.0 (5s) → Normal CCW"
echo "   ├─ wz=-1.0 (5s) → Normal CW"
echo "   └─ wz=+1.5 (5s) → Fast CCW"
echo ""
echo "3) Task 3: Walk + Turn (20s, 5 maneuvers)"
echo "   ├─ (0.8, +0.6) 5s → Right arc"
echo "   ├─ (1.0,  0.0) 2s → Straight"
echo "   ├─ (0.8, -0.6) 5s → Left arc"
echo "   ├─ (1.2,  0.0) 3s → Fast straight"
echo "   └─ (0.5, +1.0) 5s → Tight turn"
echo ""
read -p "Enter task [0-3]: " TASK

# Function to publish velocity command
publish_cmd() {
    local vx=$1
    local vy=$2
    local wz=$3
    local duration=$4

    echo "  Publishing: vx=$vx, vy=$vy, wz=$wz for ${duration}s"

    timeout ${duration}s ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
        "linear:
  x: ${vx}
  y: ${vy}
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: ${wz}" \
        --rate 10 > /dev/null 2>&1 || true
}

# Execute task sequence
case $TASK in
    0)
        echo ""
        echo "Running Task 0: Standing (20s)"
        echo "─────────────────────────────────────"
        publish_cmd 0.0 0.0 0.0 20.0
        echo "✅ Task 0 complete"
        ;;

    1)
        echo ""
        echo "Running Task 1: Walking (20s)"
        echo "─────────────────────────────────────"
        echo "1/4: Slow (0.5 m/s)..."
        publish_cmd 0.5 0.0 0.0 5.0

        echo "2/4: Normal (1.0 m/s)..."
        publish_cmd 1.0 0.0 0.0 5.0

        echo "3/4: Fast (1.5 m/s)..."
        publish_cmd 1.5 0.0 0.0 5.0

        echo "4/4: Moderate (0.8 m/s)..."
        publish_cmd 0.8 0.0 0.0 5.0

        echo "✅ Task 1 complete"
        ;;

    2)
        echo ""
        echo "Running Task 2: Turn in Place (20s)"
        echo "─────────────────────────────────────"
        echo "1/4: Slow CCW (+0.5 rad/s)..."
        publish_cmd 0.0 0.0 0.5 5.0

        echo "2/4: Normal CCW (+1.0 rad/s)..."
        publish_cmd 0.0 0.0 1.0 5.0

        echo "3/4: Normal CW (-1.0 rad/s)..."
        publish_cmd 0.0 0.0 -1.0 5.0

        echo "4/4: Fast CCW (+1.5 rad/s)..."
        publish_cmd 0.0 0.0 1.5 5.0

        echo "✅ Task 2 complete"
        ;;

    3)
        echo ""
        echo "Running Task 3: Walk + Turn (20s)"
        echo "─────────────────────────────────────"
        echo "1/5: Right arc (0.8 m/s, +0.6 rad/s)..."
        publish_cmd 0.8 0.0 0.6 5.0

        echo "2/5: Straight (1.0 m/s)..."
        publish_cmd 1.0 0.0 0.0 2.0

        echo "3/5: Left arc (0.8 m/s, -0.6 rad/s)..."
        publish_cmd 0.8 0.0 -0.6 5.0

        echo "4/5: Fast straight (1.2 m/s)..."
        publish_cmd 1.2 0.0 0.0 3.0

        echo "5/5: Tight turn (0.5 m/s, +1.0 rad/s)..."
        publish_cmd 0.5 0.0 1.0 5.0

        echo "✅ Task 3 complete"
        ;;

    *)
        echo "❌ Invalid choice. Exiting..."
        exit 1
        ;;
esac

echo ""
echo "=========================================="
echo "Task execution finished!"
echo "=========================================="

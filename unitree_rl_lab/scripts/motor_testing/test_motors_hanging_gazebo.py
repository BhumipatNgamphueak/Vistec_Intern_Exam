#!/usr/bin/env python3
"""
Test Go2 motors in Gazebo while hanging (no ground contact).

Prerequisites:
- Gazebo running with Go2 robot spawned at height
- Joint controllers loaded
- ROS 2 sourced

To spawn robot at height, modify spawn command:
  ros2 launch go2_gazebo go2_control.launch.py spawn_z:=1.5
"""

import argparse
import numpy as np
import time
from enum import Enum

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray
except ImportError:
    print("Error: ROS 2 Python packages not found!")
    print("Source ROS 2: source /opt/ros/humble/setup.bash")
    exit(1)


class MotorTester(Node):
    """ROS 2 node for testing motors while hanging."""

    def __init__(self):
        super().__init__('motor_tester_hanging')

        # Joint state subscriber
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Command publisher
        self.cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/go2_position_controller/commands',
            10
        )

        # State storage
        self.joint_names = []
        self.joint_positions = []
        self.joint_velocities = []
        self.joint_efforts = []
        self.data_received = False

        # Wait for first joint state
        print("Waiting for joint states...")
        while not self.data_received:
            rclpy.spin_once(self, timeout_sec=0.1)

        print(f"✅ Connected! Found {len(self.joint_names)} joints")

    def joint_state_callback(self, msg):
        """Process joint state messages."""
        if not self.data_received:
            self.joint_names = list(msg.name)
            print(f"\nJoint order:")
            for i, name in enumerate(self.joint_names):
                print(f"  [{i}] {name}")
            self.data_received = True

        self.joint_positions = list(msg.position)
        self.joint_velocities = list(msg.velocity)
        self.joint_efforts = list(msg.effort)

    def send_command(self, positions):
        """Send position command to all joints."""
        msg = Float64MultiArray()
        msg.data = positions
        self.cmd_pub.publish(msg)

    def get_standing_pose(self):
        """Return default standing pose."""
        # FL, FR, RL, RR (hip, thigh, calf each)
        return [
            0.0, 0.8, -1.5,  # FL
            0.0, 0.8, -1.5,  # FR
            0.0, 1.0, -1.5,  # RL
            0.0, 1.0, -1.5,  # RR
        ]


def run_motor_test(tester, args):
    """Run motor test with specified motion."""

    print("\n" + "="*60)
    print("Go2 Motor Testing - Hanging Configuration")
    print("="*60)
    print(f"\nJoint: {args.joint if args.joint else 'ALL'}")
    print(f"Motion: {args.motion}")
    print(f"Duration: {args.duration}s")
    print("")
    print("Press Ctrl+C to stop")
    print("="*60)
    print("")

    # Get standing pose
    default_pose = tester.get_standing_pose()

    # Determine which joints to test
    if args.joint:
        if args.joint in tester.joint_names:
            test_joints = [tester.joint_names.index(args.joint)]
            print(f"Testing joint: {args.joint} (index {test_joints[0]})")
        else:
            print(f"❌ Joint '{args.joint}' not found!")
            available = ", ".join(tester.joint_names)
            print(f"Available joints: {available}")
            return
    else:
        # Test all joints sequentially
        test_joints = list(range(len(tester.joint_names)))
        print(f"Testing ALL {len(test_joints)} joints sequentially")

    print("")

    # Motion parameters
    rate = tester.create_rate(50)  # 50 Hz control
    start_time = time.time()
    joint_cycle_duration = 5.0  # seconds per joint

    try:
        while (time.time() - start_time) < args.duration:
            t = time.time() - start_time

            # Cycle through joints if testing all
            if not args.joint:
                current_joint_idx = int(t / joint_cycle_duration) % len(test_joints)
                joint_to_move = test_joints[current_joint_idx]

                # Announce which joint every 5 seconds
                if int(t) % 5 == 0 and int(t) != int(t - 1/50):
                    print(f"[{t:.1f}s] Testing: {tester.joint_names[joint_to_move]}")
            else:
                joint_to_move = test_joints[0]

            # Generate command
            cmd = default_pose.copy()
            local_t = t % joint_cycle_duration

            if args.motion == "sine":
                amplitude = 0.5
                frequency = 1.0
                cmd[joint_to_move] = default_pose[joint_to_move] + \
                                    amplitude * np.sin(2 * np.pi * frequency * local_t)

            elif args.motion == "step":
                step_size = 0.5
                if local_t < joint_cycle_duration / 2:
                    cmd[joint_to_move] = default_pose[joint_to_move] + step_size
                else:
                    cmd[joint_to_move] = default_pose[joint_to_move] - step_size

            elif args.motion == "sweep":
                progress = local_t / joint_cycle_duration
                sweep_range = 1.0
                cmd[joint_to_move] = default_pose[joint_to_move] + \
                                    sweep_range * (progress - 0.5)

            # Send command
            tester.send_command(cmd)

            # Spin to process callbacks
            rclpy.spin_once(tester, timeout_sec=0.0)
            rate.sleep()

    except KeyboardInterrupt:
        print("\n\n⏹️  Stopped by user")

    # Return to standing pose
    print("\nReturning to standing pose...")
    for _ in range(50):
        tester.send_command(default_pose)
        rclpy.spin_once(tester, timeout_sec=0.0)
        rate.sleep()


def main():
    parser = argparse.ArgumentParser(description="Test Go2 motors in Gazebo (hanging)")
    parser.add_argument("--joint", type=str, default=None,
                        help="Specific joint to test (or omit for all)")
    parser.add_argument("--motion", type=str, default="sine",
                        choices=["sine", "step", "sweep"],
                        help="Motion type")
    parser.add_argument("--duration", type=float, default=30.0,
                        help="Test duration in seconds")

    args = parser.parse_args()

    # Initialize ROS
    rclpy.init()

    # Create tester
    tester = MotorTester()

    try:
        # Run test
        run_motor_test(tester, args)
    finally:
        tester.destroy_node()
        rclpy.shutdown()

    print("\n" + "="*60)
    print("Motor Testing Complete!")
    print("="*60)


if __name__ == "__main__":
    main()

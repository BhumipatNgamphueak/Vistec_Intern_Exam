#!/usr/bin/env python3
"""
IsaacLab Test Experiments - Exact Replication in Gazebo

This script replicates the 4 test experiments from IsaacLab with time-varying velocity commands.
Each task is 20 seconds long (1000 timesteps @ 50 Hz).

Tasks:
1. Standing (static stability)
2. Forward walking with speed transitions
3. Turn in place with direction changes
4. Combined walk + turn trajectories
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import sys


class IsaacTestExperiments(Node):
    """Publishes time-varying velocity commands matching IsaacLab test experiments."""

    def __init__(self, task_id: int):
        super().__init__('isaac_test_experiments')

        self.task_id = task_id
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.02, self.timer_callback)  # 50 Hz

        self.start_time = time.time()
        self.current_time = 0.0

        # Task definitions
        self.task_names = {
            0: "Task 1: Standing (Static Stability)",
            1: "Task 2: Forward Walking with Speed Transitions",
            2: "Task 3: Turn in Place with Direction Changes",
            3: "Task 4: Combined Walk + Turn Trajectories"
        }

        self.get_logger().info(f"Starting {self.task_names[task_id]}")
        self.get_logger().info("Duration: 20 seconds (1000 timesteps @ 50 Hz)")
        self.get_logger().info("Press Ctrl+C to stop early\n")

    def get_velocity_command(self, t: float) -> tuple:
        """
        Get velocity command [vx, vy, wz] based on current time and task.

        Args:
            t: Current time in seconds (0-20)

        Returns:
            (vx, vy, wz) velocity command in m/s and rad/s
        """
        if self.task_id == 0:
            # Task 1: Standing (Static Stability)
            # Stay completely still for entire 20 seconds
            return (0.0, 0.0, 0.0)

        elif self.task_id == 1:
            # Task 2: Forward Walking with Speed Transitions
            if 0 <= t < 5:
                return (0.5, 0.0, 0.0)   # Slow walk
            elif 5 <= t < 10:
                return (1.0, 0.0, 0.0)   # Normal walk
            elif 10 <= t < 15:
                return (1.5, 0.0, 0.0)   # Fast walk
            else:  # 15 <= t <= 20
                return (0.8, 0.0, 0.0)   # Moderate walk

        elif self.task_id == 2:
            # Task 3: Turn in Place with Direction Changes
            if 0 <= t < 5:
                return (0.0, 0.0, 0.5)   # Slow CCW turn
            elif 5 <= t < 10:
                return (0.0, 0.0, 1.0)   # Normal CCW turn
            elif 10 <= t < 15:
                return (0.0, 0.0, -1.0)  # Normal CW turn (direction change!)
            else:  # 15 <= t <= 20
                return (0.0, 0.0, 1.5)   # Fast CCW turn

        elif self.task_id == 3:
            # Task 4: Combined Walk + Turn Trajectories
            if 0 <= t < 5:
                return (0.8, 0.0, 0.6)   # Right arc
            elif 5 <= t < 7:
                return (1.0, 0.0, 0.0)   # Straight walk
            elif 7 <= t < 12:
                return (0.8, 0.0, -0.6)  # Left arc
            elif 12 <= t < 15:
                return (1.2, 0.0, 0.0)   # Fast straight
            else:  # 15 <= t <= 20
                return (0.5, 0.0, 1.0)   # Tight turn

        else:
            return (0.0, 0.0, 0.0)

    def timer_callback(self):
        """Publish velocity command at 50 Hz."""
        # Update current time
        self.current_time = time.time() - self.start_time

        # Check if experiment is complete
        if self.current_time >= 20.0:
            self.get_logger().info(f"\n{'='*70}")
            self.get_logger().info("Experiment completed! (20 seconds)")
            self.get_logger().info(f"{'='*70}\n")
            self.get_logger().info("Sending zero velocity and shutting down...")

            # Send stop command
            msg = Twist()
            self.publisher.publish(msg)

            # Shutdown
            rclpy.shutdown()
            return

        # Get velocity command for current time
        vx, vy, wz = self.get_velocity_command(self.current_time)

        # Create and publish message
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = wz

        self.publisher.publish(msg)

        # Log velocity transitions
        if self.current_time % 1.0 < 0.02:  # Log every second
            self.get_logger().info(
                f"t={self.current_time:5.2f}s | vx={vx:+5.2f} vy={vy:+5.2f} wz={wz:+5.2f}"
            )


def main(args=None):
    """Main function."""

    # Check command line arguments
    if len(sys.argv) != 2:
        print("Usage: ros2 run deploy_policy isaac_test_experiments <task_id>")
        print("\nTasks:")
        print("  0 - Standing (static stability)")
        print("  1 - Forward walking with speed transitions")
        print("  2 - Turn in place with direction changes")
        print("  3 - Combined walk + turn trajectories")
        sys.exit(1)

    try:
        task_id = int(sys.argv[1])
        if task_id not in [0, 1, 2, 3]:
            raise ValueError("Task ID must be 0, 1, 2, or 3")
    except ValueError as e:
        print(f"Error: {e}")
        print("Task ID must be 0, 1, 2, or 3")
        sys.exit(1)

    # Initialize ROS 2
    rclpy.init(args=args)

    # Create and run node
    node = IsaacTestExperiments(task_id)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nInterrupted by user!")
    finally:
        # Send stop command
        msg = Twist()
        node.publisher.publish(msg)

        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

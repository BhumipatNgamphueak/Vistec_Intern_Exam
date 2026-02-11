#!/usr/bin/env python3
"""
Send velocity commands to Isaac Lab simulation for testing trained policies.

Matches the 4 training tasks:
1. Standing
2. Walking (varying speeds)
3. Turn in Place
4. Walk + Turn (combined)

Usage:
    python send_velocity_commands_isaac.py --task 1        # Standing
    python send_velocity_commands_isaac.py --task 2.1      # Walk slow
    python send_velocity_commands_isaac.py --linear_x 0.5  # Custom
"""

import argparse
import numpy as np

# Define 4 training tasks
TASKS = {
    # TASK 1: Standing
    "1": {"name": "Standing", "vx": 0.0, "vy": 0.0, "wz": 0.0},

    # TASK 2: Walking
    "2.1": {"name": "Walk slow", "vx": 0.5, "vy": 0.0, "wz": 0.0},
    "2.2": {"name": "Walk normal", "vx": 1.0, "vy": 0.0, "wz": 0.0},
    "2.3": {"name": "Walk fast", "vx": 1.5, "vy": 0.0, "wz": 0.0},
    "2.4": {"name": "Walk moderate", "vx": 0.8, "vy": 0.0, "wz": 0.0},

    # TASK 3: Turn in Place
    "3.1": {"name": "Turn slow CCW", "vx": 0.0, "vy": 0.0, "wz": 0.5},
    "3.2": {"name": "Turn normal CCW", "vx": 0.0, "vy": 0.0, "wz": 1.0},
    "3.3": {"name": "Turn normal CW", "vx": 0.0, "vy": 0.0, "wz": -1.0},
    "3.4": {"name": "Turn fast CCW", "vx": 0.0, "vy": 0.0, "wz": 1.5},

    # TASK 4: Walk + Turn
    "4.1": {"name": "Right arc", "vx": 0.8, "vy": 0.0, "wz": 0.6},
    "4.2": {"name": "Straight fast", "vx": 1.2, "vy": 0.0, "wz": 0.0},
    "4.3": {"name": "Left arc", "vx": 0.8, "vy": 0.0, "wz": -0.6},
    "4.4": {"name": "Tight turn", "vx": 0.5, "vy": 0.0, "wz": 1.0},
}

def list_tasks():
    """Print all available tasks."""
    print("="*70)
    print("Available Tasks (matching training episodes)")
    print("="*70)
    print("\nTASK 1: Standing")
    print("  1      - Stand still (0.0, 0.0, 0.0)")
    print("\nTASK 2: Walking")
    print("  2.1    - Walk slow (0.5 m/s)")
    print("  2.2    - Walk normal (1.0 m/s)")
    print("  2.3    - Walk fast (1.5 m/s)")
    print("  2.4    - Walk moderate (0.8 m/s)")
    print("\nTASK 3: Turn in Place")
    print("  3.1    - Turn slow CCW (+0.5 rad/s)")
    print("  3.2    - Turn normal CCW (+1.0 rad/s)")
    print("  3.3    - Turn normal CW (-1.0 rad/s)")
    print("  3.4    - Turn fast CCW (+1.5 rad/s)")
    print("\nTASK 4: Walk + Turn")
    print("  4.1    - Right arc (0.8 m/s, +0.6 rad/s)")
    print("  4.2    - Straight fast (1.2 m/s)")
    print("  4.3    - Left arc (0.8 m/s, -0.6 rad/s)")
    print("  4.4    - Tight turn (0.5 m/s, +1.0 rad/s)")
    print("="*70)

def send_velocity_command(linear_x=0.0, linear_y=0.0, angular_z=0.0, task_name="Custom"):
    """
    Send velocity command to Isaac Lab.

    Isaac Lab policies run in closed loop - this shows how to configure
    the environment for specific velocity commands.
    """
    print("="*70)
    print("Isaac Lab Velocity Command Sender - 4 Tasks")
    print("="*70)
    print(f"\nTask: {task_name}")
    print(f"  Linear X:  {linear_x:6.2f} m/s  (forward/backward)")
    print(f"  Linear Y:  {linear_y:6.2f} m/s  (left/right)")
    print(f"  Angular Z: {angular_z:6.2f} rad/s (yaw rotation)")
    print()

    print("NOTE: Isaac Lab policies run in closed loop via play.py script.")
    print("To test with this specific velocity command:")
    print()
    print("Method: Modify environment command ranges")
    print("-" * 70)
    print("Edit the config file (e.g., velocity_env_cfg_mlp_custom.py):")
    print()
    print("  commands.base_velocity.ranges = mdp.UniformVelocityCommandCfg.Ranges(")
    print(f"      lin_vel_x=({linear_x}, {linear_x}),  # Fixed X velocity")
    print(f"      lin_vel_y=({linear_y}, {linear_y}),  # Fixed Y velocity")
    print(f"      ang_vel_z=({angular_z}, {angular_z}),  # Fixed yaw rate")
    print("      heading=(-3.14, 3.14),")
    print("  )")
    print()
    print("Then run the policy:")
    print("  python scripts/rsl_rl/play.py \\")
    print("      --task Unitree-Go2-Velocity-MLP-Custom \\")
    print("      --load_run 2026-02-03_15-54-07_work_good \\")
    print("      --num_envs 1")
    print()
    print("="*70)
    print("For Gazebo deployment with ROS 2 topics, use:")
    print("  ./send_velocity_commands_gazebo.sh")
    print("="*70)

    return True

def main():
    parser = argparse.ArgumentParser(
        description="Isaac Lab Velocity Command Sender - Matching 4 Training Tasks",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python send_velocity_commands_isaac.py --task 1       # Standing
  python send_velocity_commands_isaac.py --task 2.2     # Walk normal
  python send_velocity_commands_isaac.py --task 3.1     # Turn slow CCW
  python send_velocity_commands_isaac.py --task 4.1     # Right arc
  python send_velocity_commands_isaac.py --list         # List all tasks
  python send_velocity_commands_isaac.py --linear_x 0.6 # Custom command
        """
    )
    parser.add_argument("--task", type=str, help="Task ID (e.g., 1, 2.1, 3.2, 4.4)")
    parser.add_argument("--linear_x", type=float, help="Forward velocity (m/s), range: -1.0 to 1.0")
    parser.add_argument("--linear_y", type=float, default=0.0, help="Lateral velocity (m/s), range: -0.4 to 0.4")
    parser.add_argument("--angular_z", type=float, help="Yaw rate (rad/s), range: -1.0 to 1.0")
    parser.add_argument("--list", action="store_true", help="List all available tasks")

    args = parser.parse_args()

    # List tasks and exit
    if args.list:
        list_tasks()
        return

    # Use task preset
    if args.task:
        if args.task not in TASKS:
            print(f"ERROR: Unknown task '{args.task}'")
            print("Run with --list to see available tasks")
            return

        task = TASKS[args.task]
        linear_x = task["vx"]
        linear_y = task["vy"]
        angular_z = task["wz"]
        task_name = task["name"]

    # Use custom command
    elif args.linear_x is not None or args.angular_z is not None:
        linear_x = args.linear_x if args.linear_x is not None else 0.0
        linear_y = args.linear_y
        angular_z = args.angular_z if args.angular_z is not None else 0.0
        task_name = "Custom"

        # Validate ranges
        linear_x = np.clip(linear_x, -1.0, 1.0)
        linear_y = np.clip(linear_y, -0.4, 0.4)
        angular_z = np.clip(angular_z, -1.0, 1.0)

    # No arguments - show help
    else:
        list_tasks()
        print("\nUsage examples:")
        print("  python send_velocity_commands_isaac.py --task 2.2")
        print("  python send_velocity_commands_isaac.py --linear_x 0.8 --angular_z 0.5")
        return

    send_velocity_command(linear_x, linear_y, angular_z, task_name)

if __name__ == "__main__":
    main()

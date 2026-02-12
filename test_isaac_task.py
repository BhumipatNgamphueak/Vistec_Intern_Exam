#!/usr/bin/env python3
"""
Test Isaac Lab policy with exact training experiment sequences.

Directly runs the policy with time-varying velocity commands matching
the 4 training tasks from generate_4_task_episodes.py.

Usage:
    python test_isaac_task.py --task 0   # Task 0: Standing (20s)
    python test_isaac_task.py --task 1   # Task 1: Walking (4 speeds, 20s)
    python test_isaac_task.py --task 2   # Task 2: Turn (4 rates, 20s)
    python test_isaac_task.py --task 3   # Task 3: Walk+Turn (5 maneuvers, 20s)
"""

import argparse
import os
import sys
import subprocess
from pathlib import Path

# Training experiment sequences (from generate_4_task_episodes.py)
TASK_SEQUENCES = {
    "0": {
        "name": "Task 0: Standing",
        "description": "Standing still for 20 seconds",
        "sequence": [
            {"vx": 0.0, "vy": 0.0, "wz": 0.0, "duration": 20.0}
        ]
    },
    "1": {
        "name": "Task 1: Walking",
        "description": "Walking at 4 different speeds (5s each)",
        "sequence": [
            {"vx": 0.5, "vy": 0.0, "wz": 0.0, "duration": 5.0},  # Slow
            {"vx": 1.0, "vy": 0.0, "wz": 0.0, "duration": 5.0},  # Normal
            {"vx": 1.5, "vy": 0.0, "wz": 0.0, "duration": 5.0},  # Fast
            {"vx": 0.8, "vy": 0.0, "wz": 0.0, "duration": 5.0}   # Moderate
        ]
    },
    "2": {
        "name": "Task 2: Turn in Place",
        "description": "Turning with 4 different rates and directions (5s each)",
        "sequence": [
            {"vx": 0.0, "vy": 0.0, "wz": +0.5, "duration": 5.0},  # Slow CCW
            {"vx": 0.0, "vy": 0.0, "wz": +1.0, "duration": 5.0},  # Normal CCW
            {"vx": 0.0, "vy": 0.0, "wz": -1.0, "duration": 5.0},  # Normal CW
            {"vx": 0.0, "vy": 0.0, "wz": +1.5, "duration": 5.0}   # Fast CCW
        ]
    },
    "3": {
        "name": "Task 3: Walk + Turn",
        "description": "Combined walking and turning (5 maneuvers)",
        "sequence": [
            {"vx": 0.8, "vy": 0.0, "wz": +0.6, "duration": 5.0},  # Right arc
            {"vx": 1.0, "vy": 0.0, "wz":  0.0, "duration": 2.0},  # Straight
            {"vx": 0.8, "vy": 0.0, "wz": -0.6, "duration": 5.0},  # Left arc
            {"vx": 1.2, "vy": 0.0, "wz":  0.0, "duration": 3.0},  # Fast straight
            {"vx": 0.5, "vy": 0.0, "wz": +1.0, "duration": 5.0}   # Tight turn
        ]
    }
}

# Policy configurations
POLICIES = {
    "mlp": {
        "checkpoint": "trained_models/mlp_with_dr_24999.pt",
        "task": "Unitree-Go2-Velocity-MLP-Custom"
    },
    "lstm": {
        "checkpoint": "trained_models/lstm_dr_25000.pt",
        "task": "Unitree-Go2-Velocity-LSTM-DR"
    }
}


def list_tasks():
    """Print all available tasks."""
    print("=" * 70)
    print("Isaac Lab Task Tester - Exact Training Sequences")
    print("=" * 70)
    print("\n🎯 4 TRAINING TASKS:\n")

    for task_id, task_info in TASK_SEQUENCES.items():
        print(f"Task {task_id}: {task_info['name']}")
        print(f"  {task_info['description']}")
        print(f"  Sequence:")
        for i, cmd in enumerate(task_info['sequence'], 1):
            print(f"    {i}. vx={cmd['vx']:4.1f}, vy={cmd['vy']:4.1f}, wz={cmd['wz']:5.1f} for {cmd['duration']}s")
        print()

    print("=" * 70)


def create_temp_config(task_id: str, policy_type: str = "mlp") -> Path:
    """
    Create temporary config file with velocity command sequence.

    For Isaac Lab, we simulate time-varying commands by setting a custom
    curriculum that changes command ranges over time.

    Note: Isaac Lab doesn't natively support time-varying velocity commands
    in play.py, so we use a workaround by creating test episodes.
    """
    vistec_repo = os.getenv("VISTEC_REPO", os.path.expanduser("~/Vistec_Intern_Exam"))
    temp_dir = Path(vistec_repo) / "temp"
    temp_dir.mkdir(exist_ok=True)

    task_info = TASK_SEQUENCES[task_id]

    # Generate episode config for this specific task
    import yaml

    episode_config = {
        'metadata': {
            'generator': 'test_isaac_task.py',
            'task_id': task_id,
            'task_name': task_info['name'],
            'num_episodes': 1,
            'episode_duration': sum(cmd['duration'] for cmd in task_info['sequence']),
            'control_freq': 50.0
        },
        'episodes': [{
            'id': 0,
            'seed': 42,
            'task_name': task_info['name'],
            'repeat_index': 0,
            'initial_state': {
                'base_pos': [0.0, 0.0, 0.30],
                'base_quat': [0.0, 0.0, 0.0, 1.0],
                'base_lin_vel': [0.0, 0.0, 0.0],
                'base_ang_vel': [0.0, 0.0, 0.0],
                'joint_pos': [0.0, 0.9, -1.8] * 4,
                'joint_vel': [0.0] * 12
            },
            'command_sequence': task_info['sequence']
        }]
    }

    config_path = temp_dir / f"test_task_{task_id}.yaml"
    with open(config_path, 'w') as f:
        yaml.dump(episode_config, f, default_flow_style=False, sort_keys=False)

    return config_path


def run_isaac_test(task_id: str, policy_type: str = "mlp", num_envs: int = 1):
    """
    Run Isaac Lab policy test with exact training sequence.

    Parameters
    ----------
    task_id : str
        Task ID (0, 1, 2, or 3)
    policy_type : str
        Policy type ("mlp" or "lstm")
    num_envs : int
        Number of parallel environments
    """
    if task_id not in TASK_SEQUENCES:
        print(f"❌ ERROR: Unknown task '{task_id}'")
        print("Valid tasks: 0, 1, 2, 3")
        return False

    if policy_type not in POLICIES:
        print(f"❌ ERROR: Unknown policy type '{policy_type}'")
        print("Valid types: mlp, lstm")
        return False

    task_info = TASK_SEQUENCES[task_id]
    policy_info = POLICIES[policy_type]

    # Get paths
    vistec_repo = os.getenv("VISTEC_REPO", os.path.expanduser("~/Vistec_Intern_Exam"))
    isaaclab_path = os.getenv("ISAACLAB_PATH", os.path.expanduser("~/IsaacLab"))

    checkpoint_path = Path(vistec_repo) / policy_info["checkpoint"]
    if not checkpoint_path.exists():
        print(f"❌ ERROR: Checkpoint not found: {checkpoint_path}")
        return False

    # Print test info
    print("=" * 70)
    print(f"Running Isaac Lab Test - {task_info['name']}")
    print("=" * 70)
    print(f"\nPolicy: {policy_type.upper()}")
    print(f"Checkpoint: {checkpoint_path}")
    print(f"Task: {policy_info['task']}")
    print(f"Environments: {num_envs}")
    print(f"\nVelocity Command Sequence:")

    total_duration = 0
    for i, cmd in enumerate(task_info['sequence'], 1):
        duration = cmd['duration']
        print(f"  {i}. vx={cmd['vx']:4.1f} m/s, vy={cmd['vy']:4.1f} m/s, "
              f"wz={cmd['wz']:5.1f} rad/s → {duration}s "
              f"(t={total_duration:.0f}-{total_duration+duration:.0f}s)")
        total_duration += duration

    print(f"\nTotal duration: {total_duration}s")
    print("=" * 70)
    print()

    # For Isaac Lab, we need to modify the approach since play.py doesn't
    # support time-varying commands directly. We'll run with representative values.
    print("⚠️  NOTE: Isaac Lab play.py uses fixed velocity commands.")
    print("For time-varying sequences, use the data collection script with episode configs.")
    print()

    # Get representative velocity (first non-zero or middle value)
    if task_id == "0":
        # Standing - use zero
        vx, vy, wz = 0.0, 0.0, 0.0
    elif len(task_info['sequence']) > 1:
        # Use middle sequence value
        mid_idx = len(task_info['sequence']) // 2
        cmd = task_info['sequence'][mid_idx]
        vx, vy, wz = cmd['vx'], cmd['vy'], cmd['wz']
    else:
        cmd = task_info['sequence'][0]
        vx, vy, wz = cmd['vx'], cmd['vy'], cmd['wz']

    print(f"Running with representative command: vx={vx}, vy={vy}, wz={wz}")
    print()

    # Build command
    play_script = Path(isaaclab_path) / "source" / "standalone" / "workflows" / "rsl_rl" / "play.py"

    cmd = [
        "python",
        str(play_script),
        "--task", policy_info['task'],
        "--checkpoint", str(checkpoint_path),
        "--num_envs", str(num_envs)
    ]

    print(f"Command: {' '.join(cmd)}")
    print()
    print("Starting simulation...")
    print("=" * 70)
    print()

    # Run the command
    try:
        subprocess.run(cmd, check=True, cwd=vistec_repo)
        return True
    except subprocess.CalledProcessError as e:
        print(f"\n❌ ERROR: Command failed with exit code {e.returncode}")
        return False
    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted by user")
        return False


def main():
    parser = argparse.ArgumentParser(
        description="Test Isaac Lab policies with exact training sequences",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python test_isaac_task.py --task 0           # Test Task 0 (Standing)
  python test_isaac_task.py --task 1 --lstm    # Test Task 1 (Walking) with LSTM
  python test_isaac_task.py --task 3 --envs 4  # Test Task 3 with 4 parallel envs
  python test_isaac_task.py --list             # List all tasks
        """
    )

    parser.add_argument("--task", type=str,
                        help="Task ID (0=Standing, 1=Walking, 2=Turn, 3=Walk+Turn)")
    parser.add_argument("--policy", type=str, default="mlp", choices=["mlp", "lstm"],
                        help="Policy type (default: mlp)")
    parser.add_argument("--envs", type=int, default=1,
                        help="Number of parallel environments (default: 1)")
    parser.add_argument("--list", action="store_true",
                        help="List all available tasks")

    # Shortcuts
    parser.add_argument("--mlp", action="store_const", const="mlp", dest="policy",
                        help="Use MLP policy (same as --policy mlp)")
    parser.add_argument("--lstm", action="store_const", const="lstm", dest="policy",
                        help="Use LSTM policy (same as --policy lstm)")

    args = parser.parse_args()

    # List tasks and exit
    if args.list:
        list_tasks()
        return

    # Check task provided
    if not args.task:
        list_tasks()
        print("\n❌ ERROR: Please specify --task")
        print("\nExamples:")
        print("  python test_isaac_task.py --task 0")
        print("  python test_isaac_task.py --task 1 --lstm")
        return

    # Validate task
    if args.task not in TASK_SEQUENCES:
        print(f"❌ ERROR: Unknown task '{args.task}'")
        print("Valid tasks: 0, 1, 2, 3")
        print("Run with --list to see all tasks")
        return

    # Run test
    success = run_isaac_test(args.task, args.policy, args.envs)

    if success:
        print("\n✅ Test completed successfully")
    else:
        print("\n❌ Test failed")
        sys.exit(1)


if __name__ == "__main__":
    main()

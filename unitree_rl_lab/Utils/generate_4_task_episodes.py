#!/usr/bin/env python3
"""
Generate episode configs for 4 specific locomotion tasks:
1. Standing (20s)
2. Walking at varying speeds (20s)
3. Turn in place with direction changes (20s)
4. Walk + Turn combined maneuvers (20s)

Each episode has custom time-varying velocity commands.
"""

import numpy as np
import yaml
from datetime import datetime
from scipy.spatial.transform import Rotation


def generate_4_task_episodes(num_repeats: int = 50, base_seed: int = 42):
    """
    Generate 4 tasks × num_repeats episodes with different initial states.

    Parameters
    ----------
    num_repeats : int
        Number of times to repeat each task with different initial conditions
    base_seed : int
        Base random seed for reproducibility

    Returns
    -------
    dict
        Episode configuration dictionary
    """
    config = {
        'metadata': {
            'generator': 'generate_4_task_episodes.py',
            'created_at': datetime.now().isoformat(),
            'num_tasks': 4,
            'num_repeats': num_repeats,
            'total_episodes': 4 * num_repeats,
            'episode_duration': 20.0,
            'control_freq': 50.0
        },
        'episodes': []
    }

    # Default joint positions for Go2
    default_joint_pos = [
        0.0, 0.9, -1.8,  # FL: hip, thigh, calf
        0.0, 0.9, -1.8,  # FR
        0.0, 0.9, -1.8,  # RL
        0.0, 0.9, -1.8   # RR
    ]

    # Task definitions
    tasks = {
        'Task1_Standing': [
            {'vx': 0.0, 'vy': 0.0, 'wz': 0.0, 'duration': 20.0}
        ],
        'Task2_Walking': [
            {'vx': 0.5, 'vy': 0.0, 'wz': 0.0, 'duration': 5.0},   # Slow
            {'vx': 1.0, 'vy': 0.0, 'wz': 0.0, 'duration': 5.0},   # Normal
            {'vx': 1.5, 'vy': 0.0, 'wz': 0.0, 'duration': 5.0},   # Fast
            {'vx': 0.8, 'vy': 0.0, 'wz': 0.0, 'duration': 5.0}    # Moderate
        ],
        'Task3_TurnInPlace': [
            {'vx': 0.0, 'vy': 0.0, 'wz': +0.5, 'duration': 5.0},  # Slow CCW
            {'vx': 0.0, 'vy': 0.0, 'wz': +1.0, 'duration': 5.0},  # Normal CCW
            {'vx': 0.0, 'vy': 0.0, 'wz': -1.0, 'duration': 5.0},  # Normal CW (direction change!)
            {'vx': 0.0, 'vy': 0.0, 'wz': +1.5, 'duration': 5.0}   # Fast CCW
        ],
        'Task4_WalkTurn': [
            {'vx': 0.8, 'vy': 0.0, 'wz': +0.6, 'duration': 5.0},  # Right arc
            {'vx': 1.0, 'vy': 0.0, 'wz':  0.0, 'duration': 2.0},  # Straight
            {'vx': 0.8, 'vy': 0.0, 'wz': -0.6, 'duration': 5.0},  # Left arc
            {'vx': 1.2, 'vy': 0.0, 'wz':  0.0, 'duration': 3.0},  # Fast straight
            {'vx': 0.5, 'vy': 0.0, 'wz': +1.0, 'duration': 5.0}   # Tight turn
        ]
    }

    ep_id = 0

    for task_name, command_sequence in tasks.items():
        for repeat in range(num_repeats):
            seed = base_seed + ep_id
            rng = np.random.default_rng(seed)

            # Randomize initial state slightly for diversity
            base_height = rng.uniform(0.27, 0.33)
            roll = rng.uniform(-0.05, 0.05)
            pitch = rng.uniform(-0.05, 0.05)
            yaw = 0.0

            # Convert to quaternion [x, y, z, w]
            rotation = Rotation.from_euler('xyz', [roll, pitch, yaw])
            quat = rotation.as_quat()

            # Add small joint noise
            joint_pos = [jp + rng.uniform(-0.03, 0.03) for jp in default_joint_pos]

            initial_state = {
                'base_pos': [0.0, 0.0, float(base_height)],
                'base_quat': [float(q) for q in quat],
                'base_lin_vel': [0.0, 0.0, 0.0],
                'base_ang_vel': [0.0, 0.0, 0.0],
                'joint_pos': joint_pos,
                'joint_vel': [0.0] * 12
            }

            episode = {
                'id': ep_id,
                'seed': seed,
                'task_name': task_name,
                'repeat_index': repeat,
                'initial_state': initial_state,
                'command_sequence': command_sequence
            }

            config['episodes'].append(episode)
            ep_id += 1

    return config


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='Generate 4-task episode configurations')
    parser.add_argument('--num_repeats', type=int, default=50,
                        help='Number of times to repeat each task (default: 50)')
    parser.add_argument('--output', type=str, default='episode_configs_4tasks.yaml',
                        help='Output YAML file (default: episode_configs_4tasks.yaml)')
    parser.add_argument('--base_seed', type=int, default=42,
                        help='Base random seed (default: 42)')

    args = parser.parse_args()

    print(f"Generating 4 tasks × {args.num_repeats} repeats = {4 * args.num_repeats} episodes...")
    config = generate_4_task_episodes(args.num_repeats, args.base_seed)

    # Save to YAML
    with open(args.output, 'w') as f:
        yaml.dump(config, f, default_flow_style=False, sort_keys=False)

    print(f"✅ Saved {len(config['episodes'])} episodes to {args.output}")
    print(f"\nTask breakdown:")
    print(f"  Task 1 (Standing):      {args.num_repeats} episodes")
    print(f"  Task 2 (Walking):       {args.num_repeats} episodes")
    print(f"  Task 3 (Turn in Place): {args.num_repeats} episodes")
    print(f"  Task 4 (Walk + Turn):   {args.num_repeats} episodes")
    print(f"\nUse with data collection:")
    print(f"  python scripts/data_collection/collect_data_isaaclab.py \\")
    print(f"    --task Unitree-Go2-Velocity-MLP-Custom \\")
    print(f"    --checkpoint logs/rsl_rl/.../model_25000.pt \\")
    print(f"    --episodes {args.output} \\")
    print(f"    --num_episodes {len(config['episodes'])} \\")
    print(f"    --headless")

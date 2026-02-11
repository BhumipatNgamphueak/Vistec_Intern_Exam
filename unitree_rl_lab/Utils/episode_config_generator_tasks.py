#!/usr/bin/env python3
"""
Episode Configuration Generator with Structured Tasks

Generates test episodes with 4 specific locomotion tasks:
1. Standing (0 velocity)
2. Walking (forward at various speeds)
3. Turn in Place (yaw rotation)
4. Walk + Turn (combined locomotion)

Author: Updated 2026-02-08
"""

import numpy as np
import yaml
from datetime import datetime
from typing import List, Dict, Any
from scipy.spatial.transform import Rotation


# Define the 4 locomotion tasks
LOCOMOTION_TASKS = {
    'standing': {
        'name': 'Standing',
        'description': 'Stay still for 20 seconds',
        'commands': [
            {'vx': 0.0, 'vy': 0.0, 'wz': 0.0, 'duration': 20.0}
        ]
    },
    'walking': {
        'name': 'Walking',
        'description': 'Forward locomotion at various speeds',
        'commands': [
            {'vx': 0.5, 'vy': 0.0, 'wz': 0.0, 'duration': 5.0},  # Slow
            {'vx': 1.0, 'vy': 0.0, 'wz': 0.0, 'duration': 5.0},  # Normal
            {'vx': 1.5, 'vy': 0.0, 'wz': 0.0, 'duration': 5.0},  # Fast
            {'vx': 0.8, 'vy': 0.0, 'wz': 0.0, 'duration': 5.0},  # Moderate
        ]
    },
    'turn_in_place': {
        'name': 'Turn in Place',
        'description': 'Yaw rotation without translation',
        'commands': [
            {'vx': 0.0, 'vy': 0.0, 'wz': +0.5, 'duration': 5.0},  # Slow CCW
            {'vx': 0.0, 'vy': 0.0, 'wz': +1.0, 'duration': 5.0},  # Normal CCW
            {'vx': 0.0, 'vy': 0.0, 'wz': -1.0, 'duration': 5.0},  # Normal CW (direction change!)
            {'vx': 0.0, 'vy': 0.0, 'wz': +1.5, 'duration': 5.0},  # Fast CCW
        ]
    },
    'walk_turn': {
        'name': 'Walk + Turn',
        'description': 'Combined forward and yaw motion',
        'commands': [
            {'vx': 0.8, 'vy': 0.0, 'wz': +0.6, 'duration': 5.0},  # Right arc
            {'vx': 1.0, 'vy': 0.0, 'wz':  0.0, 'duration': 2.0},  # Straight
            {'vx': 0.8, 'vy': 0.0, 'wz': -0.6, 'duration': 5.0},  # Left arc
            {'vx': 1.2, 'vy': 0.0, 'wz':  0.0, 'duration': 3.0},  # Fast straight
            {'vx': 0.5, 'vy': 0.0, 'wz': +1.0, 'duration': 5.0},  # Tight turn
        ]
    }
}


class TaskBasedEpisodeGenerator:
    """Generate task-based episode configurations."""

    def __init__(
        self,
        episodes_per_task: int = 50,
        control_freq: float = 50.0,
        base_seed: int = 42
    ):
        """
        Initialize generator.

        Parameters
        ----------
        episodes_per_task : int
            Number of episodes per task (total = episodes_per_task × 4 tasks)
        control_freq : float
            Control frequency in Hz
        base_seed : int
            Base random seed
        """
        self.episodes_per_task = episodes_per_task
        self.control_freq = control_freq
        self.base_seed = base_seed

        # Robot specifications
        self.num_joints = 12
        self.default_joint_pos = np.array([
            0.0, 0.9, -1.8,  # FL: hip, thigh, calf
            0.0, 0.9, -1.8,  # FR
            0.0, 0.9, -1.8,  # RL
            0.0, 0.9, -1.8   # RR
        ])

        # Safe ranges for randomization
        self.base_height_range = [0.27, 0.33]
        self.base_rp_range = [-0.05, 0.05]
        self.joint_pos_noise = 0.03
        self.joint_limits = [-np.pi, np.pi]

    def _sample_initial_state(self, rng: np.random.Generator) -> Dict[str, Any]:
        """Sample a safe initial state."""
        # Base position
        base_pos = [0.0, 0.0, rng.uniform(*self.base_height_range)]

        # Base orientation
        roll = rng.uniform(*self.base_rp_range)
        pitch = rng.uniform(*self.base_rp_range)
        yaw = 0.0
        rotation = Rotation.from_euler('xyz', [roll, pitch, yaw])
        quat = rotation.as_quat()
        base_quat = quat.tolist()

        # Joint positions with noise
        joint_noise = rng.uniform(-self.joint_pos_noise, self.joint_pos_noise, size=self.num_joints)
        joint_pos = self.default_joint_pos + joint_noise
        joint_pos = np.clip(joint_pos, *self.joint_limits).tolist()

        # Zero velocities
        base_lin_vel = [0.0, 0.0, 0.0]
        base_ang_vel = [0.0, 0.0, 0.0]
        joint_vel = [0.0] * self.num_joints

        return {
            'base_pos': base_pos,
            'base_quat': base_quat,
            'base_lin_vel': base_lin_vel,
            'base_ang_vel': base_ang_vel,
            'joint_pos': joint_pos,
            'joint_vel': joint_vel
        }

    def generate_configs(self) -> Dict[str, Any]:
        """
        Generate all episode configurations.

        Returns
        -------
        Dict[str, Any]
            Complete configuration with all tasks and episodes
        """
        config = {
            'metadata': {
                'robot': 'unitree_go2',
                'control_freq_hz': int(self.control_freq),
                'episode_duration_sec': 20.0,
                'episodes_per_task': self.episodes_per_task,
                'total_episodes': self.episodes_per_task * len(LOCOMOTION_TASKS),
                'timesteps_per_episode': int(20.0 * self.control_freq),
                'created_date': datetime.now().strftime('%Y-%m-%d'),
                'created_timestamp': datetime.now().isoformat(),
                'base_seed': self.base_seed,
                'tasks': list(LOCOMOTION_TASKS.keys())
            },
            'tasks': LOCOMOTION_TASKS,
            'episodes': []
        }

        episode_id = 0

        # Generate episodes for each task
        for task_name, task_info in LOCOMOTION_TASKS.items():
            print(f"\nGenerating {self.episodes_per_task} episodes for task: {task_info['name']}")

            for i in range(self.episodes_per_task):
                seed = self.base_seed + episode_id
                rng = np.random.default_rng(seed)

                # Generate initial state (randomized for robustness)
                initial_state = self._sample_initial_state(rng)

                # Use fixed command sequence for this task
                command_sequence = task_info['commands']

                # Verify total duration
                total_duration = sum(cmd['duration'] for cmd in command_sequence)
                if not np.isclose(total_duration, 20.0):
                    raise ValueError(
                        f"Task {task_name} duration is {total_duration}s, expected 20.0s"
                    )

                # Create episode
                episode = {
                    'id': episode_id,
                    'seed': seed,
                    'task': task_name,
                    'task_name': task_info['name'],
                    'initial_state': initial_state,
                    'command_sequence': command_sequence
                }

                config['episodes'].append(episode)
                episode_id += 1

        return config

    def save_to_yaml(self, config: Dict[str, Any], filepath: str) -> None:
        """Save configuration to YAML file."""
        with open(filepath, 'w') as f:
            yaml.dump(config, f, default_flow_style=False, sort_keys=False)

        print(f"\n{'='*70}")
        print(f"✓ Saved episode configurations to {filepath}")
        print(f"{'='*70}")
        print(f"Total episodes: {len(config['episodes'])}")
        print(f"Episodes per task: {self.episodes_per_task}")
        print(f"\nTask breakdown:")
        for task_name, task_info in LOCOMOTION_TASKS.items():
            count = sum(1 for ep in config['episodes'] if ep['task'] == task_name)
            print(f"  - {task_info['name']}: {count} episodes")
        print(f"{'='*70}")


def main():
    """Generate task-based episode configurations."""
    import argparse

    parser = argparse.ArgumentParser(
        description='Generate task-based episode configurations for locomotion testing'
    )
    parser.add_argument(
        '--episodes_per_task',
        type=int,
        default=50,
        help='Number of episodes per task (default: 50, total = 50×4 = 200)'
    )
    parser.add_argument(
        '--control_freq',
        type=float,
        default=50.0,
        help='Control frequency in Hz (default: 50.0)'
    )
    parser.add_argument(
        '--seed',
        type=int,
        default=42,
        help='Base random seed (default: 42)'
    )
    parser.add_argument(
        '--output',
        type=str,
        default='episode_configs_tasks.yaml',
        help='Output YAML file (default: episode_configs_tasks.yaml)'
    )

    args = parser.parse_args()

    # Create generator
    generator = TaskBasedEpisodeGenerator(
        episodes_per_task=args.episodes_per_task,
        control_freq=args.control_freq,
        base_seed=args.seed
    )

    # Generate configurations
    print("="*70)
    print("Task-Based Episode Configuration Generator")
    print("="*70)
    print("\nLocomotion Tasks:")
    for task_name, task_info in LOCOMOTION_TASKS.items():
        print(f"\n{task_info['name']}:")
        print(f"  Description: {task_info['description']}")
        print(f"  Commands:")
        for i, cmd in enumerate(task_info['commands'], 1):
            print(f"    {i}. vx={cmd['vx']:+.1f}, vy={cmd['vy']:+.1f}, "
                  f"wz={cmd['wz']:+.1f} for {cmd['duration']:.1f}s")

    configs = generator.generate_configs()

    # Save to file
    generator.save_to_yaml(configs, args.output)

    # Print sample episodes
    print("\n" + "="*70)
    print("Sample Episodes (one per task):")
    print("="*70)
    for task_name in LOCOMOTION_TASKS.keys():
        sample = next(ep for ep in configs['episodes'] if ep['task'] == task_name)
        print(f"\nTask: {sample['task_name']} (Episode {sample['id']})")
        print(f"  Seed: {sample['seed']}")
        print(f"  Initial height: {sample['initial_state']['base_pos'][2]:.3f}m")
        print(f"  Commands: {len(sample['command_sequence'])} segments")
        first_cmd = sample['command_sequence'][0]
        print(f"  First command: vx={first_cmd['vx']}, vy={first_cmd['vy']}, "
              f"wz={first_cmd['wz']} for {first_cmd['duration']}s")

    print("="*70)


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
Episode Configuration Generator for Sim-to-Sim Transfer Experiment

Generates 200 synchronized test episodes with randomized initial states
and command sequences for both IsaacLab and Gazebo simulators.

Author: Robotics Internship Exam 2
Date: 2026-02-07
"""

import numpy as np
import yaml
from datetime import datetime
from typing import List, Dict, Any
from scipy.spatial.transform import Rotation


class EpisodeConfigGenerator:
    """Generate reproducible episode configurations for locomotion testing."""

    def __init__(
        self,
        num_episodes: int = 200,
        episode_duration: float = 20.0,
        control_freq: float = 50.0,
        base_seed: int = 42
    ):
        """
        Initialize the episode configuration generator.

        Parameters
        ----------
        num_episodes : int
            Total number of episodes to generate
        episode_duration : float
            Duration of each episode in seconds
        control_freq : float
            Control frequency in Hz
        base_seed : int
            Base random seed (episodes will use [base_seed, base_seed+num_episodes))
        """
        self.num_episodes = num_episodes
        self.episode_duration = episode_duration
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
        self.base_height_range = [0.27, 0.33]  # meters
        self.base_rp_range = [-0.05, 0.05]     # roll/pitch in radians
        self.joint_pos_noise = 0.03            # ±3cm joint variation

        # Command ranges (m/s or rad/s)
        self.vx_range = [-0.5, 1.5]
        self.vy_range = [-0.5, 0.5]
        self.wz_range = [-1.0, 1.0]

        # Joint limits
        self.joint_limits = [-np.pi, np.pi]

    def _sample_initial_state(self, rng: np.random.Generator) -> Dict[str, Any]:
        """
        Sample a safe initial state for the robot.

        Parameters
        ----------
        rng : np.random.Generator
            Random number generator

        Returns
        -------
        Dict[str, Any]
            Initial state dictionary with position, orientation, and velocities
        """
        # Base position: randomize height only
        base_pos = [
            0.0,
            0.0,
            rng.uniform(*self.base_height_range)
        ]

        # Base orientation: small random roll/pitch, zero yaw
        roll = rng.uniform(*self.base_rp_range)
        pitch = rng.uniform(*self.base_rp_range)
        yaw = 0.0

        # Convert to quaternion [x, y, z, w]
        rotation = Rotation.from_euler('xyz', [roll, pitch, yaw])
        quat = rotation.as_quat()  # Returns [x, y, z, w]
        base_quat = quat.tolist()

        # Joint positions: default stance with small noise
        joint_noise = rng.uniform(
            -self.joint_pos_noise,
            self.joint_pos_noise,
            size=self.num_joints
        )
        joint_pos = self.default_joint_pos + joint_noise

        # Clip to joint limits
        joint_pos = np.clip(joint_pos, *self.joint_limits).tolist()

        # All velocities start at zero
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

    def _generate_command_sequence(
        self,
        rng: np.random.Generator,
        num_commands: int = 4,
        command_duration: float = 5.0
    ) -> List[Dict[str, float]]:
        """
        Generate a sequence of velocity commands.

        Parameters
        ----------
        rng : np.random.Generator
            Random number generator
        num_commands : int
            Number of commands in sequence
        command_duration : float
            Duration of each command in seconds

        Returns
        -------
        List[Dict[str, float]]
            List of command dictionaries with vx, vy, wz, duration
        """
        command_sequence = []

        for _ in range(num_commands):
            cmd = {
                'vx': float(rng.uniform(*self.vx_range)),
                'vy': float(rng.uniform(*self.vy_range)),
                'wz': float(rng.uniform(*self.wz_range)),
                'duration': command_duration
            }
            command_sequence.append(cmd)

        return command_sequence

    def _validate_initial_state(self, state: Dict[str, Any]) -> bool:
        """
        Validate that the initial state is safe and within bounds.

        Parameters
        ----------
        state : Dict[str, Any]
            Initial state dictionary

        Returns
        -------
        bool
            True if state is valid, False otherwise
        """
        # Check joint positions
        joint_pos = np.array(state['joint_pos'])
        if np.any(np.abs(joint_pos) > np.pi):
            return False

        # Check base height (avoid ground collision)
        base_z = state['base_pos'][2]
        if base_z < 0.25 or base_z > 0.35:
            return False

        # Check quaternion normalization
        quat = np.array(state['base_quat'])
        quat_norm = np.linalg.norm(quat)
        if not np.isclose(quat_norm, 1.0, atol=1e-3):
            return False

        return True

    def generate_configs(self) -> Dict[str, Any]:
        """
        Generate all episode configurations.

        Returns
        -------
        Dict[str, Any]
            Complete configuration dictionary with metadata and episodes
        """
        config = {
            'metadata': {
                'robot': 'unitree_go2',
                'control_freq_hz': int(self.control_freq),
                'episode_duration_sec': self.episode_duration,
                'total_episodes': self.num_episodes,
                'timesteps_per_episode': int(self.episode_duration * self.control_freq),
                'created_date': datetime.now().strftime('%Y-%m-%d'),
                'created_timestamp': datetime.now().isoformat(),
                'base_seed': self.base_seed
            },
            'episodes': []
        }

        # Generate each episode
        for ep_id in range(self.num_episodes):
            seed = self.base_seed + ep_id
            rng = np.random.default_rng(seed)

            # Generate initial state
            initial_state = self._sample_initial_state(rng)

            # Validate state
            if not self._validate_initial_state(initial_state):
                raise ValueError(f"Invalid initial state generated for episode {ep_id}")

            # Generate command sequence
            command_sequence = self._generate_command_sequence(rng)

            # Create episode config
            episode = {
                'id': ep_id,
                'seed': seed,
                'initial_state': initial_state,
                'command_sequence': command_sequence
            }

            config['episodes'].append(episode)

        return config

    def save_to_yaml(self, config: Dict[str, Any], filepath: str) -> None:
        """
        Save configuration to YAML file.

        Parameters
        ----------
        config : Dict[str, Any]
            Configuration dictionary
        filepath : str
            Output file path
        """
        with open(filepath, 'w') as f:
            yaml.dump(
                config,
                f,
                default_flow_style=False,
                sort_keys=False,
                allow_unicode=True
            )

        print(f"✓ Saved {len(config['episodes'])} episode configurations to {filepath}")
        print(f"  - Episodes: {config['metadata']['total_episodes']}")
        print(f"  - Duration: {config['metadata']['episode_duration_sec']} sec each")
        print(f"  - Control rate: {config['metadata']['control_freq_hz']} Hz")
        print(f"  - Total timesteps: {config['metadata']['timesteps_per_episode']} per episode")


def main():
    """Generate episode configurations and save to YAML."""
    import argparse

    parser = argparse.ArgumentParser(
        description='Generate synchronized episode configurations for sim-to-sim transfer'
    )
    parser.add_argument(
        '--num_episodes',
        type=int,
        default=200,
        help='Number of episodes to generate (default: 200)'
    )
    parser.add_argument(
        '--duration',
        type=float,
        default=20.0,
        help='Episode duration in seconds (default: 20.0)'
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
        default='episode_configs.yaml',
        help='Output YAML file path (default: episode_configs.yaml)'
    )

    args = parser.parse_args()

    # Create generator
    generator = EpisodeConfigGenerator(
        num_episodes=args.num_episodes,
        episode_duration=args.duration,
        control_freq=args.control_freq,
        base_seed=args.seed
    )

    # Generate configurations
    print("Generating episode configurations...")
    configs = generator.generate_configs()

    # Save to file
    generator.save_to_yaml(configs, args.output)

    # Print sample episode
    print("\n" + "="*60)
    print("Sample Episode (ID: 0):")
    print("="*60)
    sample = configs['episodes'][0]
    print(f"Seed: {sample['seed']}")
    print(f"Initial base position: {sample['initial_state']['base_pos']}")
    print(f"Initial base quaternion: {sample['initial_state']['base_quat']}")
    print(f"Number of commands: {len(sample['command_sequence'])}")
    print(f"First command: vx={sample['command_sequence'][0]['vx']:.2f}, "
          f"vy={sample['command_sequence'][0]['vy']:.2f}, "
          f"wz={sample['command_sequence'][0]['wz']:.2f}")
    print("="*60)


if __name__ == '__main__':
    main()

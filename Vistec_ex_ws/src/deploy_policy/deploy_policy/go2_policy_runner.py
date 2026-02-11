#!/usr/bin/env python3
"""
Policy Runner for IsaacLab trained RL policies for Go2 quadruped robot.
Loads a trained PyTorch model and runs inference for robot control.
"""

import torch
import numpy as np
from collections import deque
from typing import Dict, List, Optional


class Go2PolicyRunner:
    """Runs inference on a trained RL policy for Go2 quadruped robot."""

    # Joint names in SDK order (used for ROS communication)
    JOINT_NAMES = [
        "FR_hip_joint",    # SDK 0
        "FR_thigh_joint",  # SDK 1
        "FR_calf_joint",   # SDK 2
        "FL_hip_joint",    # SDK 3
        "FL_thigh_joint",  # SDK 4
        "FL_calf_joint",   # SDK 5
        "RR_hip_joint",    # SDK 6
        "RR_thigh_joint",  # SDK 7
        "RR_calf_joint",   # SDK 8
        "RL_hip_joint",    # SDK 9
        "RL_thigh_joint",  # SDK 10
        "RL_calf_joint",   # SDK 11
    ]

    # Joint ID map from deploy.yaml: maps policy index -> SDK index
    # Policy order: FL_hip, FR_hip, RL_hip, RR_hip, FL_thigh, FR_thigh, RL_thigh, RR_thigh, FL_calf, FR_calf, RL_calf, RR_calf
    JOINT_IDS_MAP = np.array([3, 0, 9, 6, 4, 1, 10, 7, 5, 2, 11, 8])

    # Default joint positions in POLICY order (from deploy.yaml)
    # [FL_hip, FR_hip, RL_hip, RR_hip, FL_thigh, FR_thigh, RL_thigh, RR_thigh, FL_calf, FR_calf, RL_calf, RR_calf]
    DEFAULT_JOINT_POS_POLICY = np.array([0.1, -0.1, 0.1, -0.1, 0.8, 0.8, 1.0, 1.0, -1.5, -1.5, -1.5, -1.5])

    # Default joint positions in SDK order (for ROS communication)
    DEFAULT_JOINT_POS_SDK = {
        "FR_hip_joint": -0.1,     # Right hip
        "FR_thigh_joint": 0.8,    # Front thigh
        "FR_calf_joint": -1.5,    # Calf
        "FL_hip_joint": 0.1,      # Left hip
        "FL_thigh_joint": 0.8,    # Front thigh
        "FL_calf_joint": -1.5,    # Calf
        "RR_hip_joint": -0.1,     # Right hip
        "RR_thigh_joint": 1.0,    # Rear thigh
        "RR_calf_joint": -1.5,    # Calf
        "RL_hip_joint": 0.1,      # Left hip
        "RL_thigh_joint": 1.0,    # Rear thigh
        "RL_calf_joint": -1.5,    # Calf
    }

    # Observation scales from IsaacLab training (velocity_env_cfg.py)
    OBS_SCALES = {
        "ang_vel": 0.2,      # base_ang_vel scale=0.2
        "cmd_vel": 1.0,      # velocity_commands: NO SCALE in Isaac Lab
        "dof_vel": 0.05,     # joint_vel_rel scale=0.05
    }

    def __init__(
        self,
        policy_path: str,
        device: str = "cpu",
        history_length: int = 1,  # Go2 uses no history (single frame)
        action_scale: float = 0.25,
    ):
        """
        Initialize the policy runner.

        Args:
            policy_path: Path to the trained model checkpoint (.pt file)
            device: Device to run inference on ("cpu" or "cuda")
            history_length: Number of timesteps to keep in observation history
            action_scale: Scale factor for action output
        """
        self.device = torch.device(device)
        self.history_length = history_length
        self.action_scale = action_scale
        self.num_joints = len(self.JOINT_NAMES)  # 12

        # Observation dim per step: ang_vel(3) + gravity(3) + cmd(3) + pos(12) + vel(12) + action(12) = 45
        self.obs_dim_per_step = 3 + 3 + 3 + 12 + 12 + 12  # 45
        self.total_obs_dim = self.obs_dim_per_step * history_length

        # Default joint positions in SDK order (for ROS communication)
        self.default_joint_pos_sdk = np.array([
            self.DEFAULT_JOINT_POS_SDK[name] for name in self.JOINT_NAMES
        ])

        # Default joint positions in policy order (for policy computation)
        self.default_joint_pos_policy = self.DEFAULT_JOINT_POS_POLICY.copy()

        # Load the trained policy
        self.policy = self._load_policy(policy_path)

        # History buffer for observations
        self.obs_history = deque(maxlen=history_length)

        # Last action for observation
        self.last_action = np.zeros(self.num_joints)

        # Initialize with zeros
        for _ in range(history_length):
            self.obs_history.append(np.zeros(self.obs_dim_per_step))

        # Velocity command (vx, vy, wz)
        self.velocity_cmd = np.zeros(3)

        print(f"[Go2PolicyRunner] Loaded policy from: {policy_path}")
        print(f"[Go2PolicyRunner] Device: {self.device}")
        print(f"[Go2PolicyRunner] Observation dim: {self.total_obs_dim}")
        print(f"[Go2PolicyRunner] Action dim: {self.num_joints}")

    def _load_policy(self, policy_path: str) -> torch.nn.Module:
        """Load the trained policy from checkpoint."""
        checkpoint = torch.load(policy_path, map_location=self.device)

        # RSL-RL saves the model state dict
        if "model_state_dict" in checkpoint:
            model_state_dict = checkpoint["model_state_dict"]
        else:
            model_state_dict = checkpoint

        # Build the actor network (matching training architecture)
        # Typical RSL-RL architecture: Input -> 512 -> 256 -> 128 -> Output
        actor = torch.nn.Sequential(
            torch.nn.Linear(self.total_obs_dim, 512),
            torch.nn.ELU(),
            torch.nn.Linear(512, 256),
            torch.nn.ELU(),
            torch.nn.Linear(256, 128),
            torch.nn.ELU(),
            torch.nn.Linear(128, self.num_joints),
        )

        # Load actor weights from checkpoint
        actor_state_dict = {}
        for key, value in model_state_dict.items():
            if key.startswith("actor."):
                # Remove "actor." prefix
                new_key = key[6:]
                actor_state_dict[new_key] = value

        if actor_state_dict:
            actor.load_state_dict(actor_state_dict)
            print(f"[Go2PolicyRunner] Loaded actor weights successfully")
        else:
            print(f"[Go2PolicyRunner] Warning: No actor weights found in checkpoint")
            print(f"[Go2PolicyRunner] Available keys: {list(model_state_dict.keys())[:10]}")

        actor.to(self.device)
        actor.eval()

        return actor

    def set_velocity_command(self, vx: float, vy: float, wz: float):
        """Set the velocity command for the robot."""
        self.velocity_cmd = np.array([vx, vy, wz])

    def sdk_to_policy_order(self, data_sdk: np.ndarray) -> np.ndarray:
        """Convert joint data from SDK order to policy order.

        SDK order: FR_hip, FR_thigh, FR_calf, FL_hip, FL_thigh, FL_calf, RR_hip, RR_thigh, RR_calf, RL_hip, RL_thigh, RL_calf
        Policy order: FL_hip, FR_hip, RL_hip, RR_hip, FL_thigh, FR_thigh, RL_thigh, RR_thigh, FL_calf, FR_calf, RL_calf, RR_calf
        """
        # JOINT_IDS_MAP[policy_idx] = sdk_idx
        # So data_policy[policy_idx] = data_sdk[JOINT_IDS_MAP[policy_idx]]
        return data_sdk[self.JOINT_IDS_MAP]

    def policy_to_sdk_order(self, data_policy: np.ndarray) -> np.ndarray:
        """Convert joint data from policy order to SDK order.

        Policy order: FL_hip, FR_hip, RL_hip, RR_hip, FL_thigh, FR_thigh, RL_thigh, RR_thigh, FL_calf, FR_calf, RL_calf, RR_calf
        SDK order: FR_hip, FR_thigh, FR_calf, FL_hip, FL_thigh, FL_calf, RR_hip, RR_thigh, RR_calf, RL_hip, RL_thigh, RL_calf
        """
        # JOINT_IDS_MAP[policy_idx] = sdk_idx
        # So data_sdk[JOINT_IDS_MAP[policy_idx]] = data_policy[policy_idx]
        data_sdk = np.zeros(self.num_joints)
        data_sdk[self.JOINT_IDS_MAP] = data_policy
        return data_sdk

    def compute_observation(
        self,
        joint_positions: np.ndarray,
        joint_velocities: np.ndarray,
        angular_velocity: np.ndarray,
        projected_gravity: np.ndarray,
    ) -> np.ndarray:
        """
        Compute the observation vector for the policy.

        Observation order (matching IsaacLab velocity_env_cfg.py):
        1. base_ang_vel (scaled by 0.2)
        2. projected_gravity
        3. velocity_commands
        4. joint_pos_rel (in POLICY order)
        5. joint_vel_rel (scaled by 0.05, in POLICY order)
        6. last_action (in POLICY order)

        Args:
            joint_positions: Current joint positions in SDK order (12,)
            joint_velocities: Current joint velocities in SDK order (12,)
            angular_velocity: Base angular velocity from IMU (3,)
            projected_gravity: Gravity vector in robot frame (3,)

        Returns:
            Full observation vector
        """
        # Convert joint data from SDK order to POLICY order
        joint_pos_policy = self.sdk_to_policy_order(joint_positions)
        joint_vel_policy = self.sdk_to_policy_order(joint_velocities)

        # Compute relative joint positions in policy order
        joint_pos_rel = joint_pos_policy - self.default_joint_pos_policy

        # Scale observations (matching Isaac Lab velocity_env_cfg.py)
        ang_vel_scaled = angular_velocity * self.OBS_SCALES["ang_vel"]
        joint_vel_scaled = joint_vel_policy * self.OBS_SCALES["dof_vel"]
        cmd_vel = self.velocity_cmd * self.OBS_SCALES["cmd_vel"]

        # Build current observation (45 dims)
        # Order matches Isaac Lab: ang_vel, gravity, cmd, joint_pos, joint_vel, action
        # Joint data is in POLICY order!
        current_obs = np.concatenate([
            ang_vel_scaled,        # 3: base_ang_vel (scaled by 0.2)
            projected_gravity,     # 3: projected_gravity (no scale)
            cmd_vel,               # 3: velocity_commands (no scale)
            joint_pos_rel,         # 12: joint_pos_rel in POLICY order
            joint_vel_scaled,      # 12: joint_vel_rel in POLICY order (scaled by 0.05)
            self.last_action,      # 12: last_action in POLICY order
        ])

        # Add to history
        self.obs_history.append(current_obs)

        # Concatenate history (oldest to newest)
        full_obs = np.concatenate(list(self.obs_history))

        return full_obs

    def get_action(
        self,
        joint_positions: np.ndarray,
        joint_velocities: np.ndarray,
        angular_velocity: np.ndarray,
        projected_gravity: np.ndarray,
    ) -> np.ndarray:
        """
        Get the action from the policy.

        Args:
            joint_positions: Current joint positions in SDK order (12,)
            joint_velocities: Current joint velocities in SDK order (12,)
            angular_velocity: Base angular velocity from IMU (3,)
            projected_gravity: Gravity vector in robot frame (3,)

        Returns:
            Target joint positions in SDK order (12,)
        """
        # Compute observation (internally converts to policy order)
        obs = self.compute_observation(
            joint_positions, joint_velocities,
            angular_velocity, projected_gravity
        )

        # Convert to tensor
        obs_tensor = torch.from_numpy(obs).float().unsqueeze(0).to(self.device)

        # Run inference
        with torch.no_grad():
            action = self.policy(obs_tensor)

        # Convert to numpy (action is in POLICY order)
        action_policy = action.cpu().numpy().squeeze()

        # Store for next observation (keep in policy order for obs computation)
        self.last_action = action_policy.copy()

        # Compute target positions in POLICY order
        target_pos_policy = self.default_joint_pos_policy + action_policy * self.action_scale

        # Convert target positions from POLICY order to SDK order
        target_positions = self.policy_to_sdk_order(target_pos_policy)

        return target_positions

    def reset(self):
        """Reset the policy state."""
        self.obs_history.clear()
        for _ in range(self.history_length):
            self.obs_history.append(np.zeros(self.obs_dim_per_step))
        self.last_action = np.zeros(self.num_joints)
        self.velocity_cmd = np.zeros(3)

    def get_joint_names(self) -> List[str]:
        """Get the list of joint names in order."""
        return self.JOINT_NAMES.copy()

    def get_default_positions(self) -> np.ndarray:
        """Get the default joint positions in SDK order (for ROS communication)."""
        return self.default_joint_pos_sdk.copy()

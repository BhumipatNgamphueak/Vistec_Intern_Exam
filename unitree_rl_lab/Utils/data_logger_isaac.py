#!/usr/bin/env python3
"""
IsaacLab Locomotion Data Logger - Sim-to-Sim Transfer Experiment

ROS 2 node that executes a trained policy in IsaacLab and logs comprehensive
telemetry at 50 Hz across 200 test episodes.

Author: Robotics Internship Exam 2
Date: 2026-02-07
"""

import sys
import os
import time
import logging
from pathlib import Path
from datetime import datetime
from typing import Dict, List, Any, Optional, Deque
from collections import deque
from dataclasses import dataclass, asdict

import numpy as np
import pandas as pd
import torch
import yaml

# ROS 2 imports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# ROS 2 message types
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, Imu
from rosgraph_msgs.msg import Clock
from std_srvs.srv import Trigger
from scipy.spatial.transform import Rotation


# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s] [%(levelname)s] %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger(__name__)


@dataclass
class DataRow:
    """Single timestep data row (74 columns)."""

    # Metadata (5 columns)
    timestamp_sim: float
    timestamp_wall: float
    episode_id: int
    episode_seed: int
    control_cycle: int

    # Real-Time Performance (3 columns)
    rtf: float
    control_latency_ms: float
    actual_control_dt: float

    # Base State - Position (7 columns)
    base_pos_x: float
    base_pos_y: float
    base_pos_z: float
    base_quat_x: float
    base_quat_y: float
    base_quat_z: float
    base_quat_w: float

    # Base State - Euler Angles (4 columns)
    base_roll: float
    base_pitch: float
    base_yaw: float
    base_height: float

    # Base Velocity - Local Frame (6 columns)
    base_lin_vel_x: float
    base_lin_vel_y: float
    base_lin_vel_z: float
    base_ang_vel_x: float
    base_ang_vel_y: float
    base_ang_vel_z: float

    # Commands (3 columns)
    cmd_vx: float
    cmd_vy: float
    cmd_wz: float

    # IMU Observations (6 columns)
    gravity_proj_x: float
    gravity_proj_y: float
    gravity_proj_z: float
    gyro_x: float
    gyro_y: float
    gyro_z: float

    # Joint States (36 columns = 12 joints × 3)
    joint_pos: List[float]  # 12 values
    joint_vel: List[float]  # 12 values
    joint_acc: List[float]  # 12 values

    # Actions & Control (48 columns = 12 joints × 4)
    action: List[float]  # 12 values (raw policy output)
    cmd_joint_pos: List[float]  # 12 values
    cmd_torque: List[float]  # 12 values
    actual_torque: List[float]  # 12 values

    # Action History (24 columns)
    prev_action_1: List[float]  # 12 values (t-1)
    prev_action_2: List[float]  # 12 values (t-2)

    # Contact State (12 columns = 4 feet × 3)
    foot_contact_binary: List[bool]  # 4 values
    foot_contact_force: List[float]  # 4 values
    foot_slip_velocity: List[float]  # 4 values

    # Derived Metrics (4 columns)
    instantaneous_power: float
    torque_saturation_count: int
    action_smoothness: float
    gravity_alignment: float


class PolicyWrapper:
    """Wrapper for PyTorch policy network."""

    def __init__(self, checkpoint_path: str, device: str = 'cuda'):
        """
        Initialize policy wrapper.

        Parameters
        ----------
        checkpoint_path : str
            Path to policy checkpoint (.pt file)
        device : str
            Device to run inference on ('cuda' or 'cpu')
        """
        self.device = torch.device(device if torch.cuda.is_available() else 'cpu')

        logger.info(f"Loading policy from {checkpoint_path}")
        self.model = torch.jit.load(checkpoint_path, map_location=self.device)
        self.model.eval()

        # Action history for recurrent policies
        self.action_history: Deque[np.ndarray] = deque(maxlen=3)
        self.action_history.append(np.zeros(12))  # t-1
        self.action_history.append(np.zeros(12))  # t-2

        self.last_valid_action = np.zeros(12)

        logger.info(f"Policy loaded successfully on {self.device}")

    def forward(self, obs_dict: Dict[str, np.ndarray]) -> np.ndarray:
        """
        Run policy inference.

        Parameters
        ----------
        obs_dict : Dict[str, np.ndarray]
            Observation dictionary with keys:
            - 'gravity_proj': [3] projected gravity
            - 'ang_vel': [3] angular velocity
            - 'joint_pos': [12] joint positions
            - 'joint_vel': [12] joint velocities
            - 'cmd': [3] velocity commands

        Returns
        -------
        np.ndarray
            Action vector [12] (residual joint positions)
        """
        try:
            # Construct observation tensor (45D)
            obs_tensor = torch.cat([
                torch.tensor(obs_dict['ang_vel'], dtype=torch.float32),  # 3
                torch.tensor(obs_dict['gravity_proj'], dtype=torch.float32),  # 3
                torch.tensor(obs_dict['cmd'], dtype=torch.float32),  # 3
                torch.tensor(obs_dict['joint_pos'], dtype=torch.float32),  # 12
                torch.tensor(obs_dict['joint_vel'], dtype=torch.float32),  # 12
                torch.tensor(self.action_history[0], dtype=torch.float32),  # 12 (last action)
            ]).to(self.device)

            # Policy inference
            with torch.no_grad():
                action = self.model(obs_tensor.unsqueeze(0)).squeeze(0)

            action_np = action.cpu().numpy()

            # Clip to safe range
            action_np = np.clip(action_np, -1.0, 1.0)

            # Update history
            self.action_history.appendleft(action_np.copy())
            self.last_valid_action = action_np.copy()

            return action_np

        except Exception as e:
            logger.error(f"Policy inference failed: {e}")
            return self.last_valid_action

    def reset(self):
        """Reset action history."""
        self.action_history.clear()
        self.action_history.append(np.zeros(12))
        self.action_history.append(np.zeros(12))
        self.last_valid_action = np.zeros(12)


class IsaacLabDataLogger(Node):
    """ROS 2 node for data logging in IsaacLab."""

    def __init__(self, config_path: str):
        """
        Initialize the data logger node.

        Parameters
        ----------
        config_path : str
            Path to configuration YAML file
        """
        super().__init__('isaac_data_logger')

        # Load configuration
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)

        # Initialize policy
        policy_path = self.config['isaac_logger']['policy_checkpoint']
        self.policy = PolicyWrapper(policy_path)

        # Robot configuration
        self.num_joints = self.config['isaac_logger']['robot_config']['num_joints']
        self.torque_limit = self.config['isaac_logger']['robot_config']['torque_limit']
        self.default_joint_pos = np.array(
            self.config['isaac_logger']['robot_config']['default_joint_pos']
        )

        # Control parameters
        self.control_freq = self.config['isaac_logger']['control']['frequency_hz']
        self.control_dt = 1.0 / self.control_freq
        self.kp = self.config['isaac_logger']['control']['pd_gains']['kp']
        self.kd = self.config['isaac_logger']['control']['pd_gains']['kd']

        # Observation noise parameters
        noise_cfg = self.config['isaac_logger']['observation_noise']
        self.imu_gyro_std = noise_cfg['imu_gyro_std']
        self.imu_accel_std = noise_cfg['imu_accel_std']
        self.joint_pos_std = noise_cfg['joint_pos_std']
        self.joint_vel_std = noise_cfg['joint_vel_std']

        # State variables
        self.current_sim_time = 0.0
        self.current_wall_time = time.time()
        self.prev_sim_time = None
        self.prev_wall_time = None

        self.base_pose = None
        self.base_velocity = None
        self.imu_data = None
        self.joint_state = None
        self.contact_forces = np.zeros(4)

        self.prev_joint_vel = np.zeros(self.num_joints)

        # Create QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ROS 2 subscribers
        self.create_subscription(Clock, '/clock', self.clock_callback, qos_profile)
        self.create_subscription(Imu, '/isaac/imu', self.imu_callback, qos_profile)
        self.create_subscription(
            JointState, '/isaac/robot_state', self.robot_state_callback, qos_profile
        )

        # ROS 2 publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/isaac/cmd_vel', qos_profile)
        self.joint_cmd_pub = self.create_publisher(
            JointState, '/isaac/joint_targets', qos_profile
        )

        # Create service clients
        self.reset_client = self.create_client(Trigger, '/isaac/reset_simulation')

        logger.info("IsaacLab Data Logger initialized")

    def clock_callback(self, msg: Clock):
        """Update simulation time from /clock topic."""
        self.current_sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def imu_callback(self, msg: Imu):
        """Process IMU data."""
        self.imu_data = {
            'orientation': [
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w
            ],
            'angular_velocity': [
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z
            ],
            'linear_acceleration': [
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z
            ]
        }

    def robot_state_callback(self, msg: JointState):
        """Process robot state (joints + base)."""
        self.joint_state = {
            'position': np.array(msg.position[:self.num_joints]),
            'velocity': np.array(msg.velocity[:self.num_joints]),
            'effort': np.array(msg.effort[:self.num_joints])
        }

    def calculate_rtf(self) -> float:
        """
        Calculate Real-Time Factor.

        Returns
        -------
        float
            RTF value (sim_dt / wall_dt)
        """
        if self.prev_sim_time is None:
            rtf = 1.0
        else:
            sim_dt = self.current_sim_time - self.prev_sim_time
            wall_dt = self.current_wall_time - self.prev_wall_time
            rtf = sim_dt / wall_dt if wall_dt > 0 else 0.0

        self.prev_sim_time = self.current_sim_time
        self.prev_wall_time = self.current_wall_time

        return rtf

    def add_observation_noise(self, obs_dict: Dict[str, np.ndarray]) -> Dict[str, np.ndarray]:
        """
        Add observation noise matching training conditions.

        Parameters
        ----------
        obs_dict : Dict[str, np.ndarray]
            Clean observation dictionary

        Returns
        -------
        Dict[str, np.ndarray]
            Noisy observation dictionary
        """
        noisy_obs = obs_dict.copy()

        # IMU noise
        noisy_obs['ang_vel'] += np.random.normal(0, self.imu_gyro_std, size=3)
        noisy_obs['gravity_proj'] += np.random.normal(0, self.imu_accel_std, size=3)

        # Joint noise
        noisy_obs['joint_pos'] += np.random.normal(0, self.joint_pos_std, size=self.num_joints)
        noisy_obs['joint_vel'] += np.random.normal(0, self.joint_vel_std, size=self.num_joints)

        return noisy_obs

    def compute_pd_torques(
        self,
        target_pos: np.ndarray,
        current_pos: np.ndarray,
        current_vel: np.ndarray
    ) -> np.ndarray:
        """
        Compute PD control torques.

        Parameters
        ----------
        target_pos : np.ndarray
            Target joint positions [12]
        current_pos : np.ndarray
            Current joint positions [12]
        current_vel : np.ndarray
            Current joint velocities [12]

        Returns
        -------
        np.ndarray
            Commanded torques [12]
        """
        pos_error = target_pos - current_pos
        torque = self.kp * pos_error - self.kd * current_vel

        # Clip to torque limits
        torque = np.clip(torque, -self.torque_limit, self.torque_limit)

        return torque

    def collect_observations(self) -> Dict[str, np.ndarray]:
        """
        Collect current observations from sensors.

        Returns
        -------
        Dict[str, np.ndarray]
            Observation dictionary
        """
        # Extract projected gravity from IMU
        if self.imu_data is None or self.joint_state is None:
            raise ValueError("Missing sensor data")

        # Get gravity projection from orientation
        quat = self.imu_data['orientation']
        rotation = Rotation.from_quat(quat)
        gravity_world = np.array([0, 0, -1])
        gravity_proj = rotation.inv().apply(gravity_world)

        obs = {
            'gravity_proj': gravity_proj,
            'ang_vel': np.array(self.imu_data['angular_velocity']),
            'joint_pos': self.joint_state['position'] - self.default_joint_pos,  # Relative
            'joint_vel': self.joint_state['velocity'],
            'cmd': np.zeros(3)  # Will be filled by episode manager
        }

        return obs

    def construct_data_row(
        self,
        obs: Dict[str, np.ndarray],
        action: np.ndarray,
        cmd: np.ndarray,
        episode_id: int,
        episode_seed: int,
        cycle: int,
        rtf: float,
        latency_ms: float
    ) -> DataRow:
        """
        Construct a complete data row for logging.

        Parameters
        ----------
        obs : Dict[str, np.ndarray]
            Observations
        action : np.ndarray
            Policy action
        cmd : np.ndarray
            Velocity command [vx, vy, wz]
        episode_id : int
            Episode number
        episode_seed : int
            Random seed
        cycle : int
            Control cycle number
        rtf : float
            Real-time factor
        latency_ms : float
            Control latency in milliseconds

        Returns
        -------
        DataRow
            Complete data row
        """
        # Get current state
        quat = self.imu_data['orientation']
        rotation = Rotation.from_quat(quat)
        euler = rotation.as_euler('xyz')

        joint_pos_abs = self.joint_state['position']
        joint_vel = self.joint_state['velocity']

        # Compute joint acceleration (numerical)
        joint_acc = (joint_vel - self.prev_joint_vel) / self.control_dt
        self.prev_joint_vel = joint_vel.copy()

        # Compute target joint positions
        action_scale = 0.25
        target_joint_pos = self.default_joint_pos + action * action_scale

        # Compute commanded torques
        cmd_torque = self.compute_pd_torques(target_joint_pos, joint_pos_abs, joint_vel)
        actual_torque = self.joint_state['effort']

        # Derived metrics
        power = np.sum(np.abs(actual_torque * joint_vel))
        torque_sat_count = np.sum(np.abs(cmd_torque) > 0.9 * self.torque_limit)

        prev_action = self.policy.action_history[1]
        action_smoothness = np.sum((action - prev_action) ** 2)

        gravity_alignment = np.linalg.norm(obs['gravity_proj'][:2])

        # Contact state (placeholder - would need actual contact sensor data)
        foot_contact_binary = [False] * 4
        foot_contact_force = [0.0] * 4
        foot_slip_velocity = [0.0] * 4

        # Construct data row
        row = DataRow(
            # Metadata
            timestamp_sim=self.current_sim_time,
            timestamp_wall=self.current_wall_time,
            episode_id=episode_id,
            episode_seed=episode_seed,
            control_cycle=cycle,

            # Performance
            rtf=rtf,
            control_latency_ms=latency_ms,
            actual_control_dt=self.control_dt,

            # Base position
            base_pos_x=0.0,  # Would need base state topic
            base_pos_y=0.0,
            base_pos_z=0.0,
            base_quat_x=quat[0],
            base_quat_y=quat[1],
            base_quat_z=quat[2],
            base_quat_w=quat[3],

            # Base orientation
            base_roll=euler[0],
            base_pitch=euler[1],
            base_yaw=euler[2],
            base_height=0.0,  # Would need ground distance

            # Base velocity
            base_lin_vel_x=0.0,  # Would need velocity topic
            base_lin_vel_y=0.0,
            base_lin_vel_z=0.0,
            base_ang_vel_x=obs['ang_vel'][0],
            base_ang_vel_y=obs['ang_vel'][1],
            base_ang_vel_z=obs['ang_vel'][2],

            # Commands
            cmd_vx=cmd[0],
            cmd_vy=cmd[1],
            cmd_wz=cmd[2],

            # IMU
            gravity_proj_x=obs['gravity_proj'][0],
            gravity_proj_y=obs['gravity_proj'][1],
            gravity_proj_z=obs['gravity_proj'][2],
            gyro_x=obs['ang_vel'][0],
            gyro_y=obs['ang_vel'][1],
            gyro_z=obs['ang_vel'][2],

            # Joints
            joint_pos=joint_pos_abs.tolist(),
            joint_vel=joint_vel.tolist(),
            joint_acc=joint_acc.tolist(),

            # Actions
            action=action.tolist(),
            cmd_joint_pos=target_joint_pos.tolist(),
            cmd_torque=cmd_torque.tolist(),
            actual_torque=actual_torque.tolist(),

            # Action history
            prev_action_1=self.policy.action_history[0].tolist(),
            prev_action_2=self.policy.action_history[1].tolist(),

            # Contact
            foot_contact_binary=foot_contact_binary,
            foot_contact_force=foot_contact_force,
            foot_slip_velocity=foot_slip_velocity,

            # Metrics
            instantaneous_power=power,
            torque_saturation_count=int(torque_sat_count),
            action_smoothness=action_smoothness,
            gravity_alignment=gravity_alignment
        )

        return row

    def validate_data_row(self, row: DataRow) -> bool:
        """Validate data row for anomalies."""
        # Check for NaN/Inf
        row_dict = asdict(row)
        for key, value in row_dict.items():
            if isinstance(value, (list, np.ndarray)):
                if np.any(np.isnan(value)) or np.any(np.isinf(value)):
                    logger.warning(f"NaN/Inf in {key} at cycle {row.control_cycle}")
                    return False
            elif isinstance(value, float):
                if np.isnan(value) or np.isinf(value):
                    logger.warning(f"NaN/Inf in {key} at cycle {row.control_cycle}")
                    return False

        # Check RTF
        if row.rtf < 0.5 or row.rtf > 2.0:
            logger.warning(f"Unusual RTF: {row.rtf} at cycle {row.control_cycle}")

        return True

    def datarow_to_flat_dict(self, row: DataRow) -> Dict[str, Any]:
        """
        Convert DataRow to flat dictionary for CSV.

        Parameters
        ----------
        row : DataRow
            Data row object

        Returns
        -------
        Dict[str, Any]
            Flattened dictionary with expanded arrays
        """
        flat = {}
        row_dict = asdict(row)

        for key, value in row_dict.items():
            if isinstance(value, list):
                # Expand lists into individual columns
                for i, v in enumerate(value):
                    flat[f"{key}_{i}"] = v
            else:
                flat[key] = value

        return flat

    def save_episode_data(
        self,
        data_buffer: List[DataRow],
        episode_id: int,
        output_dir: str
    ):
        """
        Save episode data to CSV.

        Parameters
        ----------
        data_buffer : List[DataRow]
            List of data rows
        episode_id : int
            Episode number
        output_dir : str
            Output directory
        """
        # Convert to flat dictionaries
        flat_rows = [self.datarow_to_flat_dict(row) for row in data_buffer]

        # Create DataFrame
        df = pd.DataFrame(flat_rows)

        # Generate filename
        timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        filename = f"locomotion_log_isaac_ep{episode_id:03d}_{timestamp}.csv"
        filepath = Path(output_dir) / filename

        # Save to CSV
        df.to_csv(filepath, index=False)

        logger.info(f"✓ Saved episode {episode_id} data to {filepath}")
        logger.info(f"  - Rows: {len(df)}")
        logger.info(f"  - Columns: {len(df.columns)}")


def main():
    """Main entry point."""
    import argparse

    parser = argparse.ArgumentParser(description='IsaacLab Data Logger')
    parser.add_argument('--config', required=True, help='Config YAML file')
    parser.add_argument('--episodes', required=True, help='Episode config YAML file')
    parser.add_argument('--output', required=True, help='Output directory')
    parser.add_argument('--policy', required=True, help='Policy checkpoint file')
    parser.add_argument('--start_episode', type=int, default=0, help='Start episode')
    parser.add_argument('--end_episode', type=int, default=200, help='End episode')
    parser.add_argument('--resume', action='store_true', help='Resume from last episode')

    args = parser.parse_args()

    # Create output directory
    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    # Initialize ROS 2
    rclpy.init()

    # Create logger node
    logger_node = IsaacLabDataLogger(args.config)

    # Load episode configurations
    with open(args.episodes, 'r') as f:
        episode_configs = yaml.safe_load(f)

    logger.info(f"Loaded {len(episode_configs['episodes'])} episode configurations")
    logger.info(f"Running episodes {args.start_episode} to {args.end_episode}")

    # TODO: Implement full episode execution loop
    # This would require IsaacLab environment integration

    logger.info("Data logger ready. Press Ctrl+C to stop.")

    try:
        rclpy.spin(logger_node)
    except KeyboardInterrupt:
        logger.info("Shutting down...")
    finally:
        logger_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

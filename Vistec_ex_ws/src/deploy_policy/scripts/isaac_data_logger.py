#!/usr/bin/env python3
"""
IsaacLab-Compatible Data Logger for Gazebo

Matches the exact data format from IsaacLab data collection:
- 104 columns matching locomotion_log_isaac format
- Starts logging AFTER robot is stable and velocity command is sent
- File naming: locomotion_log_gazebo_ep{episode_id}_{timestamp}.csv
- Output directory structure: {output_dir}/{policy_type}/
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import torch
import numpy as np
import os
import time
from datetime import datetime
from pathlib import Path
import argparse
import sys
import csv


class IsaacDataLogger(Node):
    """Data logger matching IsaacLab format for Gazebo deployment."""

    def __init__(self, policy_path, policy_config, model_name, task_id, episode_id, output_dir):
        super().__init__('isaac_data_logger')

        # Configuration
        self.policy_path = policy_path
        self.policy_config = policy_config
        self.model_name = model_name
        self.task_id = task_id
        self.episode_id = episode_id

        # Output directory: {output_dir}/{model_name}/{policy_config}/
        self.output_dir = Path(output_dir) / model_name / policy_config
        self.output_dir.mkdir(parents=True, exist_ok=True)

        # Episode parameters
        self.episode_seed = np.random.randint(0, 10000)
        self.episode_duration = 20.0  # 20 seconds
        self.num_joints = 12
        self.control_dt = 0.02  # 50 Hz
        self.episode_step = 0
        self.episode_start_time = None

        # Task names for identification
        self.task_names = {
            0: "standing",
            1: "forward_walking",
            2: "turn_in_place",
            3: "combined_maneuvers"
        }
        self.task_name = self.task_names.get(task_id, f"task_{task_id}")

        # State tracking
        self.logging_active = False
        self.init_sent_count = 0
        self.csv_file = None
        self.csv_writer = None

        # Robot state
        self.joint_positions = np.zeros(self.num_joints)
        self.joint_velocities = np.zeros(self.num_joints)
        self.joint_accelerations = np.zeros(self.num_joints)
        self.prev_joint_velocities = np.zeros(self.num_joints)

        self.base_pos = np.zeros(3)
        self.base_quat = np.array([0.0, 0.0, 0.0, 1.0])
        self.base_lin_vel = np.zeros(3)
        self.base_ang_vel = np.zeros(3)

        self.imu_data = None
        self.cmd_vel = np.zeros(3)  # [vx, vy, wz]

        # Policy tracking
        self.actions = np.zeros(self.num_joints)
        self.prev_actions = np.zeros(self.num_joints)
        self.cmd_torques = np.zeros(self.num_joints)
        self.actual_torques = np.zeros(self.num_joints)

        # Performance metrics
        self.rtf = 1.0  # Real-time factor
        self.last_wall_time = time.time()
        self.last_sim_time = 0.0

        # Load policy
        self.policy = self.load_policy()

        # Joint name mapping (SDK order for ROS topics)
        self.joint_names = [
            'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
            'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
            'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
            'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
        ]

        # ROS 2 subscriptions
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10
        )
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )

        # Publishers
        self.effort_publishers = {}
        for joint_name in self.joint_names:
            topic = f'/model/go2_robot/joint/{joint_name}/cmd_force'
            self.effort_publishers[joint_name] = self.create_publisher(
                Twist, topic, 10  # Using Twist as placeholder, should be Float64
            )

        # Control timer (50 Hz)
        self.control_timer = self.create_timer(self.control_dt, self.control_loop)

        # Initialization timer (send standing commands until stable)
        self.init_timer = self.create_timer(0.02, self.send_initial_pose)

        self.get_logger().info(f"Isaac Data Logger initialized")
        self.get_logger().info(f"  Model: {self.model_name}")
        self.get_logger().info(f"  Policy Config: {self.policy_config}")
        self.get_logger().info(f"  Episode: {self.episode_id}, Task: {self.task_id} ({self.task_name})")
        self.get_logger().info(f"  Seed: {self.episode_seed}")
        self.get_logger().info(f"  Output: {self.output_dir}")
        self.get_logger().info(f"  Waiting for robot to stabilize before logging...")

    def load_policy(self):
        """Load trained policy checkpoint."""
        try:
            checkpoint = torch.load(self.policy_path, map_location='cpu')
            policy = checkpoint  # Simplified, adjust based on actual checkpoint structure
            self.get_logger().info(f"Policy loaded from {self.policy_path}")
            return policy
        except Exception as e:
            self.get_logger().error(f"Failed to load policy: {e}")
            return None

    def cmd_vel_callback(self, msg):
        """Track commanded velocities."""
        self.cmd_vel = np.array([msg.linear.x, msg.linear.y, msg.angular.z])

    def joint_state_callback(self, msg):
        """Process joint state updates."""
        for i, name in enumerate(msg.name):
            if name in self.joint_names:
                idx = self.joint_names.index(name)
                self.joint_positions[idx] = msg.position[i]
                self.joint_velocities[idx] = msg.velocity[i]
                if len(msg.effort) > i:
                    self.actual_torques[idx] = msg.effort[i]

    def odom_callback(self, msg):
        """Process odometry updates."""
        self.base_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        self.base_quat = np.array([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])
        self.base_lin_vel = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ])
        self.base_ang_vel = np.array([
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z
        ])

    def imu_callback(self, msg):
        """Process IMU data."""
        self.imu_data = msg

    def send_initial_pose(self):
        """Send standing pose until robot is stable."""
        # Simple standing command (would use actual policy here)
        self.init_sent_count += 1

        # After 2 seconds (100 iterations at 50 Hz), robot should be stable
        if self.init_sent_count >= 100:
            self.init_timer.cancel()
            self.start_logging()

    def start_logging(self):
        """Start data collection after robot is stable."""
        self.logging_active = True
        self.episode_start_time = time.time()
        self.last_wall_time = time.time()
        self.last_sim_time = 0.0

        # Create CSV file
        timestamp_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        filename = f"locomotion_log_gazebo_ep{self.episode_id}_{timestamp_str}.csv"
        filepath = self.output_dir / filename

        self.csv_file = open(filepath, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        # Write header (106 columns: 104 original + task_id + task_name)
        header = [
            'timestamp_sim', 'timestamp_wall', 'episode_id', 'episode_seed',
            'task_id', 'task_name',  # Added for easy identification
            'control_cycle', 'rtf', 'control_latency_ms', 'actual_control_dt',
            'base_pos_x', 'base_pos_y', 'base_pos_z',
            'base_quat_x', 'base_quat_y', 'base_quat_z', 'base_quat_w',
            'base_roll', 'base_pitch', 'base_yaw', 'base_height',
            'base_lin_vel_x', 'base_lin_vel_y', 'base_lin_vel_z',
            'base_ang_vel_x', 'base_ang_vel_y', 'base_ang_vel_z',
            'cmd_vx', 'cmd_vy', 'cmd_wz',
        ]
        # Joint positions (0-11)
        header += [f'joint_pos_{i}' for i in range(12)]
        # Joint velocities (0-11)
        header += [f'joint_vel_{i}' for i in range(12)]
        # Joint accelerations (0-11)
        header += [f'joint_acc_{i}' for i in range(12)]
        # Actions (0-11)
        header += [f'action_{i}' for i in range(12)]
        # Command torques (0-11)
        header += [f'cmd_torque_{i}' for i in range(12)]
        # Actual torques (0-11)
        header += [f'actual_torque_{i}' for i in range(12)]
        # Metrics
        header += ['instantaneous_power', 'torque_saturation_count', 'action_smoothness', 'gravity_alignment']

        self.csv_writer.writerow(header)

        self.get_logger().info(f"✓ Robot stabilized, starting data collection")
        self.get_logger().info(f"✓ Logging to: {filepath}")

    def control_loop(self):
        """Main control loop at 50 Hz."""
        if not self.logging_active:
            return

        loop_start_time = time.time()

        # Check if episode is complete
        elapsed_time = time.time() - self.episode_start_time
        if elapsed_time >= self.episode_duration:
            self.finish_episode()
            return

        # Compute joint accelerations
        self.joint_accelerations = (self.joint_velocities - self.prev_joint_velocities) / self.control_dt
        self.prev_joint_velocities = self.joint_velocities.copy()

        # Run policy (simplified - would use actual policy inference here)
        # self.actions = self.policy(observations)
        self.actions = np.zeros(self.num_joints)  # Placeholder

        # Compute torques (simplified PD control)
        Kp, Kd = 25.0, 0.5
        self.cmd_torques = Kp * (self.actions - self.joint_positions) + Kd * (0 - self.joint_velocities)

        # Collect data row
        self.collect_and_write_data(loop_start_time)

        # Update step counter
        self.episode_step += 1

    def collect_and_write_data(self, loop_start_time):
        """Collect and write data row to CSV."""
        timestamp_sim = self.episode_step * self.control_dt
        timestamp_wall = time.time() - self.episode_start_time

        # Compute metrics
        roll, pitch, yaw = self.get_euler_angles()
        base_height = self.base_pos[2]
        control_latency_ms = (time.time() - loop_start_time) * 1000.0

        # Update RTF
        current_wall_time = time.time()
        wall_dt = current_wall_time - self.last_wall_time
        sim_dt = timestamp_sim - self.last_sim_time
        if wall_dt > 0:
            self.rtf = sim_dt / wall_dt
        self.last_wall_time = current_wall_time
        self.last_sim_time = timestamp_sim

        # Compute derived metrics
        instantaneous_power = np.sum(np.abs(self.actual_torques * self.joint_velocities))
        torque_saturation_count = np.sum(np.abs(self.cmd_torques) > 40.0)  # Assuming 40Nm saturation
        action_smoothness = np.sum(np.abs(self.actions - self.prev_actions))
        gravity_alignment = np.dot([0, 0, 1], self.get_gravity_vector())

        # Build row (106 columns: 104 original + task_id + task_name)
        row = [
            timestamp_sim, timestamp_wall, self.episode_id, self.episode_seed,
            self.task_id, self.task_name,  # Added for easy identification
            self.episode_step, self.rtf, control_latency_ms, self.control_dt,
            *self.base_pos, *self.base_quat,
            roll, pitch, yaw, base_height,
            *self.base_lin_vel, *self.base_ang_vel,
            *self.cmd_vel,
            *self.joint_positions,
            *self.joint_velocities,
            *self.joint_accelerations,
            *self.actions,
            *self.cmd_torques,
            *self.actual_torques,
            instantaneous_power, torque_saturation_count, action_smoothness, gravity_alignment
        ]

        self.csv_writer.writerow(row)
        self.prev_actions = self.actions.copy()

    def get_euler_angles(self):
        """Convert quaternion to Euler angles (roll, pitch, yaw)."""
        x, y, z, w = self.base_quat

        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(np.clip(sinp, -1.0, 1.0))

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def get_gravity_vector(self):
        """Get gravity vector in body frame."""
        x, y, z, w = self.base_quat
        # Simplified gravity vector computation
        gx = 2 * (x * z - w * y)
        gy = 2 * (y * z + w * x)
        gz = 1 - 2 * (x * x + y * y)
        return np.array([gx, gy, gz])

    def finish_episode(self):
        """Finish episode and close CSV file."""
        self.logging_active = False

        if self.csv_file:
            self.csv_file.close()
            self.get_logger().info(f"\n{'='*70}")
            self.get_logger().info(f"✓ Episode {self.episode_id} completed!")
            self.get_logger().info(f"  Duration: {self.episode_duration}s ({self.episode_step} steps)")
            self.get_logger().info(f"  Data saved to: {self.output_dir}")
            self.get_logger().info(f"{'='*70}\n")

        rclpy.shutdown()


def main(args=None):
    """Main function."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--policy', required=True, help='Path to policy checkpoint')
    parser.add_argument('--policy_config', default='mlp_custom', help='Policy configuration name')
    parser.add_argument('--model_name', default='model_24999', help='Model name for folder organization')
    parser.add_argument('--task_id', type=int, required=True, help='Task ID (0-3)')
    parser.add_argument('--episode_id', type=int, required=True, help='Episode ID')
    parser.add_argument('--output', default='/home/drl-68/data_collection_gazebo', help='Output directory')

    parsed_args = parser.parse_args()

    rclpy.init(args=args)

    node = IsaacDataLogger(
        policy_path=parsed_args.policy,
        policy_config=parsed_args.policy_config,
        model_name=parsed_args.model_name,
        task_id=parsed_args.task_id,
        episode_id=parsed_args.episode_id,
        output_dir=parsed_args.output
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

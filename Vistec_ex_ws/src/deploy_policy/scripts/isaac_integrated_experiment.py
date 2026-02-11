#!/usr/bin/env python3
"""
Integrated IsaacLab Test Experiment with Data Logging

Combines velocity command publishing and data logging in a single node
to ensure perfect synchronization. Both start at exactly the same time
after the robot stabilization phase.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
import torch
import numpy as np
import os
import time
from datetime import datetime
from pathlib import Path
import argparse
import sys
import csv


class IsaacIntegratedExperiment(Node):
    """
    Integrated experiment node that:
    1. Waits for robot to stabilize (2 seconds)
    2. Starts velocity commands AND data logging simultaneously
    3. Runs for 20 seconds
    4. Stops and saves data
    """

    def __init__(self, policy_path, policy_config, model_name, task_id, episode_id, output_dir):
        super().__init__('isaac_integrated_experiment')

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
        self.experiment_start_time = None

        # Task names for identification
        self.task_names = {
            0: "standing",
            1: "forward_walking",
            2: "turn_in_place",
            3: "combined_maneuvers"
        }
        self.task_name = self.task_names.get(task_id, f"task_{task_id}")

        # State tracking
        self.stabilization_phase = True
        self.experiment_active = False
        self.stabilization_count = 0
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
            Odometry, '/odom', self.odom_callback, 10
        )
        # Subscribe to ground truth pose from Gazebo
        self.pose_sub = self.create_subscription(
            PoseStamped, '/model/go2_robot/pose', self.pose_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10
        )

        # State for pose-based odometry
        self.last_pose_time = None
        self.last_pose_position = None

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.effort_publishers = {}
        for joint_name in self.joint_names:
            topic = f'/model/go2_robot/joint/{joint_name}/cmd_force'
            self.effort_publishers[joint_name] = self.create_publisher(
                Twist, topic, 10  # Using Twist as placeholder, should be Float64
            )

        # Main control timer (50 Hz)
        self.control_timer = self.create_timer(self.control_dt, self.control_loop)

        self.get_logger().info("="*70)
        self.get_logger().info("Integrated IsaacLab Experiment with Data Logging")
        self.get_logger().info("="*70)
        self.get_logger().info(f"  Model: {self.model_name}")
        self.get_logger().info(f"  Policy Config: {self.policy_config}")
        self.get_logger().info(f"  Episode: {self.episode_id}, Task: {self.task_id} ({self.task_name})")
        self.get_logger().info(f"  Seed: {self.episode_seed}")
        self.get_logger().info(f"  Output: {self.output_dir}")
        self.get_logger().info("")
        self.get_logger().info("→ Phase 1: Robot stabilization (2 seconds)...")

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

    def pose_callback(self, msg):
        """Process ground truth pose from Gazebo and compute velocity."""
        current_time = self.get_clock().now().nanoseconds * 1e-9

        # Update position
        current_pos = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])

        # Update quaternion
        self.base_quat = np.array([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ])

        # Compute velocity by differentiating position
        if self.last_pose_time is not None and self.last_pose_position is not None:
            dt = current_time - self.last_pose_time
            if dt > 0.001:  # Avoid division by very small dt
                velocity = (current_pos - self.last_pose_position) / dt
                self.base_lin_vel = velocity

        # Update position AFTER computing velocity
        self.base_pos = current_pos
        self.last_pose_position = current_pos.copy()
        self.last_pose_time = current_time

    def imu_callback(self, msg):
        """Process IMU data."""
        self.imu_data = msg

    def get_velocity_command(self, t: float) -> tuple:
        """
        Get velocity command [vx, vy, wz] based on current time and task.

        Args:
            t: Current time in seconds (0-20)

        Returns:
            (vx, vy, wz) velocity command in m/s and rad/s
        """
        if self.task_id == 0:
            # Task 0: Standing (Static Stability)
            return (0.0, 0.0, 0.0)

        elif self.task_id == 1:
            # Task 1: Forward Walking with Speed Transitions
            if 0 <= t < 5:
                return (0.5, 0.0, 0.0)   # Slow walk
            elif 5 <= t < 10:
                return (1.0, 0.0, 0.0)   # Normal walk
            elif 10 <= t < 15:
                return (1.5, 0.0, 0.0)   # Fast walk
            else:  # 15 <= t <= 20
                return (0.8, 0.0, 0.0)   # Moderate walk

        elif self.task_id == 2:
            # Task 2: Turn in Place with Direction Changes
            if 0 <= t < 5:
                return (0.0, 0.0, 0.5)   # Slow CCW turn
            elif 5 <= t < 10:
                return (0.0, 0.0, 1.0)   # Normal CCW turn
            elif 10 <= t < 15:
                return (0.0, 0.0, -1.0)  # Normal CW turn (direction change!)
            else:  # 15 <= t <= 20
                return (0.0, 0.0, 1.5)   # Fast CCW turn

        elif self.task_id == 3:
            # Task 3: Combined Walk + Turn Trajectories
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

    def start_experiment(self):
        """
        Start experiment phase: velocity commands AND data logging begin simultaneously.
        """
        self.experiment_active = True
        self.experiment_start_time = time.time()
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

        self.get_logger().info("✓ Robot stabilized!")
        self.get_logger().info("→ Phase 2: Experiment running (velocity commands + data logging)...")
        self.get_logger().info(f"  Duration: 20 seconds")
        self.get_logger().info(f"  Logging to: {filepath}")
        self.get_logger().info("")

    def control_loop(self):
        """
        Main control loop at 50 Hz.

        Phase 1: Stabilization (2 seconds) - send zero commands, no logging
        Phase 2: Experiment (20 seconds) - velocity commands + data logging (synchronized)
        """
        loop_start_time = time.time()

        # Phase 1: Stabilization
        if self.stabilization_phase:
            self.stabilization_count += 1

            # Send zero velocity command during stabilization
            msg = Twist()
            self.cmd_vel_publisher.publish(msg)

            # After 2 seconds (100 iterations at 50 Hz), start experiment
            if self.stabilization_count >= 100:
                self.stabilization_phase = False
                self.start_experiment()

            return

        # Phase 2: Experiment (velocity commands + data logging)
        if not self.experiment_active:
            return

        # Check if episode is complete
        elapsed_time = time.time() - self.experiment_start_time
        if elapsed_time >= self.episode_duration:
            self.finish_episode()
            return

        # === Velocity Command Publishing ===
        vx, vy, wz = self.get_velocity_command(elapsed_time)
        self.cmd_vel = np.array([vx, vy, wz])

        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = wz

        self.cmd_vel_publisher.publish(msg)

        # Log velocity transitions
        if elapsed_time % 5.0 < self.control_dt:  # Log every 5 seconds
            self.get_logger().info(
                f"  t={elapsed_time:5.2f}s | vx={vx:+5.2f} vy={vy:+5.2f} wz={wz:+5.2f}"
            )

        # === Data Collection ===
        # Compute joint accelerations
        self.joint_accelerations = (self.joint_velocities - self.prev_joint_velocities) / self.control_dt
        self.prev_joint_velocities = self.joint_velocities.copy()

        # Run policy (simplified - would use actual policy inference here)
        # self.actions = self.policy(observations)
        self.actions = np.zeros(self.num_joints)  # Placeholder

        # Compute torques (simplified PD control)
        Kp, Kd = 25.0, 0.5
        self.cmd_torques = Kp * (self.actions - self.joint_positions) + Kd * (0 - self.joint_velocities)

        # Collect and write data row
        self.collect_and_write_data(loop_start_time)

        # Update step counter
        self.episode_step += 1

    def collect_and_write_data(self, loop_start_time):
        """Collect and write data row to CSV."""
        timestamp_sim = self.episode_step * self.control_dt
        timestamp_wall = time.time() - self.experiment_start_time

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
        self.experiment_active = False

        # Send stop command
        msg = Twist()
        self.cmd_vel_publisher.publish(msg)

        if self.csv_file:
            self.csv_file.close()
            self.get_logger().info("")
            self.get_logger().info("="*70)
            self.get_logger().info(f"✓ Episode {self.episode_id} completed!")
            self.get_logger().info(f"  Duration: {self.episode_duration}s ({self.episode_step} steps)")
            self.get_logger().info(f"  Data saved to: {self.output_dir}")
            self.get_logger().info("="*70)

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

    node = IsaacIntegratedExperiment(
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

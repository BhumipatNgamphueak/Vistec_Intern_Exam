#!/usr/bin/env python3
"""
Single Episode Gazebo Data Logger
Collects 74-column telemetry for one episode (1000 timesteps at 50 Hz)
Designed to be called repeatedly with Gazebo restarted between episodes
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np
import torch
import csv
import time
import yaml
import argparse
from pathlib import Path
from datetime import datetime
from scipy.spatial.transform import Rotation
from collections import deque

# ROS 2 messages
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

# Import policy runner and actuator network
from deploy_policy.go2_policy_runner import Go2PolicyRunner
from deploy_policy.actuator_network import ActuatorNetworkRunner


class TimeVaryingCommand:
    """Time-varying velocity commands for the 4 locomotion tasks."""

    @staticmethod
    def get_command(task_id: int, time_s: float) -> np.ndarray:
        """
        Get velocity command [vx, vy, wz] for given task and time.

        Args:
            task_id: 0=Standing, 1=Walking, 2=Turning, 3=Combined
            time_s: Time in seconds since episode start

        Returns:
            np.ndarray: [vx, vy, wz] command
        """
        if task_id == 0:
            # Task 1: Standing (20 seconds)
            return np.array([0.0, 0.0, 0.0])

        elif task_id == 1:
            # Task 2: Walking with speed transitions
            if time_s < 5.0:
                return np.array([0.5, 0.0, 0.0])   # Slow
            elif time_s < 10.0:
                return np.array([1.0, 0.0, 0.0])   # Normal
            elif time_s < 15.0:
                return np.array([1.5, 0.0, 0.0])   # Fast
            else:
                return np.array([0.8, 0.0, 0.0])   # Moderate

        elif task_id == 2:
            # Task 3: Turn in place with direction changes
            if time_s < 5.0:
                return np.array([0.0, 0.0, 0.5])   # Slow CCW
            elif time_s < 10.0:
                return np.array([0.0, 0.0, 1.0])   # Normal CCW
            elif time_s < 15.0:
                return np.array([0.0, 0.0, -1.0])  # Normal CW
            else:
                return np.array([0.0, 0.0, 1.5])   # Fast CCW

        elif task_id == 3:
            # Task 4: Combined walk + turn (MUST MATCH Isaac Lab Task4_WalkTurn)
            if time_s < 5.0:
                return np.array([0.8, 0.0, 0.6])   # Right arc (5.0s)
            elif time_s < 7.0:
                return np.array([1.0, 0.0, 0.0])   # Straight (2.0s)
            elif time_s < 12.0:
                return np.array([0.8, 0.0, -0.6])  # Left arc (5.0s)
            elif time_s < 15.0:
                return np.array([1.2, 0.0, 0.0])   # Fast straight (3.0s)
            else:
                return np.array([0.5, 0.0, 1.0])   # Tight turn (5.0s)

        else:
            raise ValueError(f"Invalid task_id: {task_id}")


class SingleEpisodeDataLogger(Node):
    """Collects data for a single episode and exits."""

    def __init__(self, config_path: str, policy_path: str, policy_config: str,
                 task_id: int, episode_id: int, output_dir: str):
        super().__init__("single_episode_data_logger")

        self.task_id = task_id
        self.episode_id = episode_id
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

        # Store policy information for metadata
        self.policy_path = policy_path
        self.policy_config = policy_config

        # Episode parameters
        self.max_timesteps = 1000  # 20 seconds at 50 Hz
        self.control_dt = 0.02     # 50 Hz
        self.episode_step = 0
        self.episode_start_time = time.time()

        # Data buffer
        self.data_rows = []

        # Policy runner
        self.get_logger().info(f"Loading policy from {policy_path}")
        self.policy_runner = Go2PolicyRunner(
            policy_path=policy_path,
            device="cpu"
        )

        # Actuator network (disabled for Gazebo - uses simple PD control instead)
        self.use_actuator_network = False
        self.actuator_network = None
        try:
            actuator_model_path = "/home/drl-68/actuator_net/app/resources/actuator.pth"
            actuator_scaler_path = "/home/drl-68/actuator_net/app/resources/scaler.pkl"
            self.actuator_network = ActuatorNetworkRunner(
                model_path=actuator_model_path,
                scaler_path=actuator_scaler_path,
                num_joints=12,
                device="cpu"
            )
            self.get_logger().info("✓ MLP Actuator Network loaded successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to load actuator network: {e}")
            self.get_logger().warn("Falling back to simple PD control")
            self.use_actuator_network = False

        # Action history
        self.action_history = deque(maxlen=2)
        self.prev_action = np.zeros(12)

        # Robot state (in SDK order to match policy runner)
        self.joint_positions = np.zeros(12)
        self.joint_velocities = np.zeros(12)
        self.joint_efforts = np.zeros(12)
        self.joint_accelerations = np.zeros(12)
        self.prev_joint_velocities = np.zeros(12)

        # Joint name to SDK index mapping (CRITICAL for correct joint ordering)
        self.joint_names_sdk = self.policy_runner.get_joint_names()
        self.joint_name_to_idx = {name: i for i, name in enumerate(self.joint_names_sdk)}

        # PD gains - MUST match IsaacLab training (GO2HV actuator)
        self.kp_gains = np.full(12, 25.0)  # Stiffness
        self.kd_gains = np.full(12, 0.5)   # Damping

        self.base_pos = np.zeros(3)
        self.base_quat = np.array([0.0, 0.0, 0.0, 1.0])
        self.base_lin_vel = np.zeros(3)
        self.base_ang_vel = np.zeros(3)

        # Flags
        self.joint_state_received = False
        self.odom_received = False

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Joint command publishers - use SDK order and map by name (CRITICAL FIX)
        self.joint_cmd_pubs = {}
        for joint_name in self.joint_names_sdk:
            topic = f'/model/go2_robot/joint/{joint_name}/cmd_force'
            self.joint_cmd_pubs[joint_name] = self.create_publisher(Float64, topic, 10)

        # Subscribers
        qos_reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            qos_reliable
        )

        # Subscribe to FIXED odometry (with IMU orientation merged in)
        # If /odom_fixed is not available, falls back to /odom
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom_fixed',  # Changed from /odom to /odom_fixed
            self.odom_callback,
            qos_reliable
        )

        # Subscribe to IMU for backup orientation data
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            qos_reliable
        )

        # Get default standing positions from policy runner
        self.default_positions = self.policy_runner.get_default_positions()

        # Send initial standing pose until joint states arrive
        self.init_timer = self.create_timer(0.02, self.send_initial_pose)  # 50Hz
        self.init_sent_count = 0
        self.logging_active = False  # Don't start logging until robot is stable

        # Control timer at 50 Hz
        self.control_timer = self.create_timer(self.control_dt, self.control_loop)

        self.get_logger().info(
            f"Initialized for Episode {episode_id}, Task {task_id}, "
            f"Target: {self.max_timesteps} timesteps"
        )
        self.get_logger().info("Sending initial standing pose...")

    def joint_state_callback(self, msg):
        """Process joint state messages - maps joints by name to SDK order."""
        self.prev_joint_velocities = self.joint_velocities.copy()

        # Map joints by name to SDK index (CRITICAL FIX for joint ordering)
        for i, name in enumerate(msg.name):
            if name in self.joint_name_to_idx:
                sdk_idx = self.joint_name_to_idx[name]
                if i < len(msg.position):
                    self.joint_positions[sdk_idx] = msg.position[i]
                if i < len(msg.velocity):
                    self.joint_velocities[sdk_idx] = msg.velocity[i]
                if i < len(msg.effort):
                    self.joint_efforts[sdk_idx] = msg.effort[i]

        # Compute acceleration
        if self.joint_state_received:
            self.joint_accelerations = (self.joint_velocities - self.prev_joint_velocities) / self.control_dt

        self.joint_state_received = True

    def odom_callback(self, msg):
        """Process odometry messages."""
        self.base_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])

        # CRITICAL: Convert from ROS format (x,y,z,w) to Isaac Lab format (w,x,y,z)
        # for correct projected_gravity computation
        self.base_quat = np.array([
            msg.pose.pose.orientation.w,  # w first!
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z
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

        self.odom_received = True

    def imu_callback(self, msg):
        """Process IMU messages - use as backup if odom orientation is invalid."""
        # Check if odometry has invalid orientation (all zeros)
        if np.allclose(self.base_quat, [0, 0, 0, 0], atol=1e-6):
            # CRITICAL: Convert from ROS format (x,y,z,w) to Isaac Lab format (w,x,y,z)
            self.base_quat = np.array([
                msg.orientation.w,  # w first!
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z
            ])

            # Also use IMU angular velocity (more accurate)
            self.base_ang_vel = np.array([
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z
            ])

    def compute_projected_gravity(self):
        """Compute projected gravity in body frame.

        NOTE: self.base_quat is in Isaac Lab format (w,x,y,z)
        but scipy.Rotation.from_quat expects (x,y,z,w) format.
        We must convert before using it.
        """
        # Convert from Isaac Lab format (w,x,y,z) to scipy format (x,y,z,w)
        quat_scipy = np.array([
            self.base_quat[1],  # x
            self.base_quat[2],  # y
            self.base_quat[3],  # z
            self.base_quat[0]   # w
        ])
        rot = Rotation.from_quat(quat_scipy)
        gravity_world = np.array([0.0, 0.0, -1.0])
        projected_gravity = rot.inv().apply(gravity_world)
        return projected_gravity

    def get_euler_angles(self):
        """Convert quaternion to Euler angles (roll, pitch, yaw).

        NOTE: self.base_quat is in Isaac Lab format (w,x,y,z)
        but scipy.Rotation.from_quat expects (x,y,z,w) format.
        """
        # Convert from Isaac Lab format (w,x,y,z) to scipy format (x,y,z,w)
        quat_scipy = np.array([
            self.base_quat[1],  # x
            self.base_quat[2],  # y
            self.base_quat[3],  # z
            self.base_quat[0]   # w
        ])
        rot = Rotation.from_quat(quat_scipy)
        euler = rot.as_euler('xyz')
        return euler[0], euler[1], euler[2]

    def send_initial_pose(self):
        """Send initial standing pose until robot is stable."""
        if self.joint_state_received and self.odom_received:
            # Wait a bit more for robot to stabilize
            if self.init_sent_count >= 100:  # 2 seconds of stable commands
                self.init_timer.cancel()
                self.logging_active = True
                self.episode_start_time = time.time()  # Reset start time
                self.get_logger().info("Robot stabilized, starting data collection")
                return

        # Compute PD torques to hold default position
        position_error = self.default_positions - self.joint_positions
        torques = self.kp_gains * position_error - self.kd_gains * self.joint_velocities

        # Publish holding torques by joint name (CRITICAL FIX)
        for i, joint_name in enumerate(self.joint_names_sdk):
            if joint_name in self.joint_cmd_pubs:
                msg = Float64()
                msg.data = float(torques[i])
                self.joint_cmd_pubs[joint_name].publish(msg)

        self.init_sent_count += 1
        if self.init_sent_count % 25 == 0:  # Log every 0.5 seconds
            self.get_logger().info(f"Stabilizing robot... ({self.init_sent_count/50:.1f}s)")

    def control_loop(self):
        """Main control loop at 50 Hz."""
        # Wait for topics
        if not self.joint_state_received or not self.odom_received:
            if not hasattr(self, '_last_warning') or time.time() - self._last_warning > 5.0:
                self._last_warning = time.time()
                self.get_logger().warn(
                    f"Waiting for topics: joints={self.joint_state_received}, "
                    f"odom={self.odom_received}"
                )
            return

        # Don't start logging until robot is stable
        if not self.logging_active:
            return

        # Check if episode complete
        if self.episode_step >= self.max_timesteps:
            self.get_logger().info("Episode complete, saving data...")
            self.save_and_exit()
            return

        loop_start_time = time.time()

        # Get time-varying command
        time_in_episode = self.episode_step * self.control_dt
        cmd_vel = TimeVaryingCommand.get_command(self.task_id, time_in_episode)

        # Publish velocity command
        twist = Twist()
        twist.linear.x = float(cmd_vel[0])
        twist.linear.y = float(cmd_vel[1])
        twist.angular.z = float(cmd_vel[2])
        self.cmd_vel_pub.publish(twist)

        # Set command in policy
        self.policy_runner.set_velocity_command(cmd_vel[0], cmd_vel[1], cmd_vel[2])

        # Get projected gravity
        projected_gravity = self.compute_projected_gravity()

        # Get action from policy
        try:
            action = self.policy_runner.get_action(
                self.joint_positions.copy(),
                self.joint_velocities.copy(),
                self.base_ang_vel.copy(),
                projected_gravity
            )
        except Exception as e:
            self.get_logger().error(f"Policy error: {e}")
            return

        # Compute torques using actuator network or PD control
        if self.use_actuator_network and self.actuator_network is not None:
            # Use MLP actuator network (matches training - CRITICAL FIX)
            torques = self.actuator_network.compute_torques(
                action, self.joint_positions, self.joint_velocities
            )
        else:
            # Fallback to simple PD control
            position_error = action - self.joint_positions
            torques = self.kp_gains * position_error - self.kd_gains * self.joint_velocities

        # Publish joint commands by name (CRITICAL FIX)
        for i, joint_name in enumerate(self.joint_names_sdk):
            if joint_name in self.joint_cmd_pubs:
                msg = Float64()
                msg.data = float(torques[i])
                self.joint_cmd_pubs[joint_name].publish(msg)

        # Collect data row (76 columns)
        data_row = self.collect_data_row(cmd_vel, action, loop_start_time)
        self.data_rows.append(data_row)

        # Update action history
        self.action_history.append(action.copy())
        self.prev_action = action.copy()

        self.episode_step += 1

        # Progress logging
        if self.episode_step % 100 == 0:
            self.get_logger().info(
                f"Progress: {self.episode_step}/{self.max_timesteps} timesteps "
                f"({100 * self.episode_step / self.max_timesteps:.1f}%)"
            )

    def collect_data_row(self, cmd_vel, action, loop_start_time):
        """Collect all 76 columns for current timestep."""
        timestamp_sim = self.episode_step * self.control_dt
        timestamp_wall = time.time() - self.episode_start_time

        # Compute metrics
        roll, pitch, yaw = self.get_euler_angles()
        base_height = self.base_pos[2]

        # Control latency
        control_latency_ms = (time.time() - loop_start_time) * 1000.0

        # Action smoothness
        if len(self.action_history) >= 2:
            action_smoothness = np.linalg.norm(self.action_history[-1] - self.action_history[-2])
        else:
            action_smoothness = 0.0

        # Instantaneous power
        instantaneous_power = np.sum(np.abs(self.joint_efforts * self.joint_velocities))

        # Gravity alignment
        projected_gravity = self.compute_projected_gravity()
        gravity_alignment = np.dot(projected_gravity, np.array([0.0, 0.0, -1.0]))

        # Build row dictionary (76 columns - added policy_path and policy_config)
        row = {
            # A. Metadata (7 columns - added policy tracking)
            'timestamp_sim': timestamp_sim,
            'timestamp_wall': timestamp_wall,
            'episode_id': self.episode_id,
            'episode_seed': 42,  # TODO: Add randomization
            'control_cycle': self.episode_step,
            'policy_path': self.policy_path,  # Track which model was used
            'policy_config': self.policy_config,  # Track policy configuration

            # B. Performance Metrics (3 columns)
            'rtf': timestamp_sim / timestamp_wall if timestamp_wall > 0 else 1.0,
            'control_latency_ms': control_latency_ms,
            'actual_control_dt': self.control_dt,

            # C. Base Position (7 columns)
            # NOTE: base_quat is stored in Isaac Lab format (w,x,y,z)
            'base_pos_x': self.base_pos[0],
            'base_pos_y': self.base_pos[1],
            'base_pos_z': self.base_pos[2],
            'base_quat_w': self.base_quat[0],  # w first (Isaac Lab format)
            'base_quat_x': self.base_quat[1],
            'base_quat_y': self.base_quat[2],
            'base_quat_z': self.base_quat[3],

            # D. Base Orientation (4 columns)
            'base_roll': roll,
            'base_pitch': pitch,
            'base_yaw': yaw,
            'base_height': base_height,

            # E. Base Velocity (6 columns)
            'base_lin_vel_x': self.base_lin_vel[0],
            'base_lin_vel_y': self.base_lin_vel[1],
            'base_lin_vel_z': self.base_lin_vel[2],
            'base_ang_vel_x': self.base_ang_vel[0],
            'base_ang_vel_y': self.base_ang_vel[1],
            'base_ang_vel_z': self.base_ang_vel[2],

            # F. Commanded Velocities (3 columns)
            'cmd_vx': cmd_vel[0],
            'cmd_vy': cmd_vel[1],
            'cmd_wz': cmd_vel[2],
        }

        # G. Joint States (36 columns)
        for i in range(12):
            row[f'joint_pos_{i}'] = self.joint_positions[i]
        for i in range(12):
            row[f'joint_vel_{i}'] = self.joint_velocities[i]
        for i in range(12):
            row[f'joint_acc_{i}'] = self.joint_accelerations[i]

        # H. Actions & Commands (12 columns for now - simplified)
        for i in range(12):
            row[f'action_{i}'] = action[i]

        # I. Action History (0 columns for now - will add if needed)

        # J. Contact & Foot State (0 columns - not available in basic Gazebo)

        # K. Derived Metrics (4 columns)
        row['instantaneous_power'] = instantaneous_power
        row['torque_saturation_count'] = 0  # TODO: Implement
        row['action_smoothness'] = action_smoothness
        row['gravity_alignment'] = gravity_alignment

        return row

    def save_and_exit(self):
        """Save collected data to CSV and exit."""
        if not self.data_rows:
            self.get_logger().error("No data collected!")
            rclpy.shutdown()
            return

        # Generate filename
        timestamp_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        filename = f"locomotion_log_gazebo_ep{self.episode_id:03d}_{timestamp_str}.csv"
        filepath = self.output_dir / filename

        # Write to CSV
        self.get_logger().info(f"Saving {len(self.data_rows)} rows to {filepath}")

        with open(filepath, 'w', newline='') as f:
            if self.data_rows:
                writer = csv.DictWriter(f, fieldnames=self.data_rows[0].keys())
                writer.writeheader()
                writer.writerows(self.data_rows)

        self.get_logger().info(f"✅ Episode {self.episode_id} data saved successfully!")
        self.get_logger().info(f"   File: {filepath}")
        self.get_logger().info(f"   Rows: {len(self.data_rows)}")

        # Shutdown
        rclpy.shutdown()


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True, help='Path to deployment config YAML')
    parser.add_argument('--policy', required=True, help='Path to policy checkpoint (.pt)')
    parser.add_argument('--policy_config', required=True, help='Policy configuration name')
    parser.add_argument('--task_id', type=int, required=True, help='Task ID (0-3)')
    parser.add_argument('--episode_id', type=int, required=True, help='Episode ID')
    parser.add_argument('--output', required=True, help='Output directory')

    parsed_args = parser.parse_args()

    rclpy.init(args=args)

    logger = SingleEpisodeDataLogger(
        config_path=parsed_args.config,
        policy_path=parsed_args.policy,
        policy_config=parsed_args.policy_config,
        task_id=parsed_args.task_id,
        episode_id=parsed_args.episode_id,
        output_dir=parsed_args.output
    )

    try:
        rclpy.spin(logger)
    except KeyboardInterrupt:
        logger.get_logger().info("Interrupted by user")
    finally:
        if logger.episode_step > 0 and logger.episode_step < logger.max_timesteps:
            logger.get_logger().warn("Episode incomplete, saving partial data...")
            logger.save_and_exit()
        logger.destroy_node()


if __name__ == '__main__':
    main()

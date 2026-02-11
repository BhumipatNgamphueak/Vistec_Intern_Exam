#!/usr/bin/env python3
"""
Enhanced Gazebo Data Logger for Sim2Sim Transfer
Collects 152-column telemetry with time-varying velocity commands

Matches specifications:
- 6 policy configurations (MLP/LSTM/Implicit × DR/No-DR)
- 4 locomotion tasks with time-varying commands
- 200 episodes per config (50 repeats × 4 tasks)
- 152 columns at 50 Hz
- 1000 timesteps per episode (20 seconds)
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

# Import policy runner
from deploy_policy.go2_policy_runner import Go2PolicyRunner
from deploy_policy.actuator_network import ActuatorNetworkRunner


class TimeVaryingCommand:
    """Handles time-varying velocity commands for different tasks."""
    
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
                return np.array([0.0, 0.0, -1.0])  # Normal CW (direction change)
            else:
                return np.array([0.0, 0.0, 1.5])   # Fast CCW
        
        elif task_id == 3:
            # Task 4: Combined walk + turn
            if time_s < 5.0:
                return np.array([0.8, 0.0, 0.6])   # Right arc
            elif time_s < 7.0:
                return np.array([1.0, 0.0, 0.0])   # Straight
            elif time_s < 12.0:
                return np.array([0.8, 0.0, -0.6])  # Left arc
            elif time_s < 15.0:
                return np.array([1.2, 0.0, 0.0])   # Fast straight
            else:
                return np.array([0.5, 0.0, 1.0])   # Tight turn
        
        else:
            raise ValueError(f"Invalid task_id: {task_id}")


class EnhancedGazeboDataLogger(Node):
    """Enhanced data logger with 152 columns and time-varying commands."""
    
    def __init__(self, config_path: str, policy_path: str, output_dir: str, policy_config: str):
        super().__init__("enhanced_gazebo_data_logger")
        
        # Load configuration
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)['gazebo_logger']
        
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.policy_config_name = policy_config
        
        # Initialize policy runner
        self.get_logger().info(f"Loading policy: {policy_config}")
        self.get_logger().info(f"Checkpoint: {policy_path}")
        
        self.policy_runner = Go2PolicyRunner(
            policy_path=policy_path,
            device=self.config.get('device', 'cpu'),
            history_length=1,
            action_scale=0.25,
        )
        
        # Initialize actuator network
        self.use_actuator_network = self.config.get('use_actuator_network', True)
        if self.use_actuator_network:
            actuator_cfg = self.config['actuator_network']
            try:
                self.actuator_network = ActuatorNetworkRunner(
                    model_path=actuator_cfg['model_path'],
                    scaler_path=actuator_cfg['scaler_path'],
                    num_joints=12,
                    device=self.config.get('device', 'cpu')
                )
                self.get_logger().info("✓ Actuator Network loaded")
            except Exception as e:
                self.get_logger().warn(f"Actuator network failed, using PD: {e}")
                self.use_actuator_network = False
        
        # PD gains
        self.kp_gains = np.full(12, 25.0)
        self.kd_gains = np.full(12, 0.5)
        
        # State variables
        self.joint_positions = np.zeros(12)
        self.joint_velocities = np.zeros(12)
        self.joint_accelerations = np.zeros(12)  # NEW
        self.prev_joint_velocities = np.zeros(12)
        
        self.base_pos = np.zeros(3)
        self.base_quat = np.array([0, 0, 0, 1])
        self.base_lin_vel = np.zeros(3)
        self.base_ang_vel = np.zeros(3)
        
        # Action history
        self.last_action = np.zeros(12)
        self.prev_action_1 = np.zeros(12)  # NEW: t-1
        self.prev_action_2 = np.zeros(12)  # NEW: t-2
        
        # Current targets
        self.target_positions = np.zeros(12)
        self.commanded_torques = np.zeros(12)
        self.actual_torques = np.zeros(12)
        
        # Episode state
        self.current_task_id = 0
        self.current_episode = 0
        self.episode_step = 0
        self.episode_seed = 0
        self.logging_active = False
        self.csv_writer = None
        self.csv_file = None
        
        # Timing
        self.episode_start_time_sim = 0.0
        self.episode_start_time_wall = 0.0
        self.control_loop_start_time = 0.0
        
        # QoS
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, qos
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/model/go2_robot/odometry', self.odom_callback, qos
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, qos
        )
        
        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Joint effort publishers
        self.joint_effort_pubs = {}
        joint_names = self.policy_runner.get_joint_names()
        for joint_name in joint_names:
            topic = f"/model/go2_robot/joint/{joint_name}/cmd_force"
            self.joint_effort_pubs[joint_name] = self.create_publisher(
                Twist, topic, 10  # Using Twist as placeholder
            )
        
        # Control timer at 50 Hz
        self.control_timer = self.create_timer(0.02, self.control_loop)
        
        self.joint_state_received = False
        self.odom_received = False
        
        self.get_logger().info("="*70)
        self.get_logger().info("Enhanced Gazebo Data Logger Initialized")
        self.get_logger().info(f"Policy config: {policy_config}")
        self.get_logger().info(f"Output: {self.output_dir}")
        self.get_logger().info("="*70)

        # Wait for topics (max 10 seconds)
        self.get_logger().info("Waiting for Gazebo topics...")
        wait_start = time.time()
        while (time.time() - wait_start < 10.0 and
               (not self.joint_state_received or not self.odom_received)):
            rclpy.spin_once(self, timeout_sec=0.1)

        if not self.joint_state_received or not self.odom_received:
            self.get_logger().error("="*70)
            self.get_logger().error("ERROR: Gazebo topics not available!")
            self.get_logger().error(f"joint_states: {self.joint_state_received}")
            self.get_logger().error(f"odometry: {self.odom_received}")
            self.get_logger().error("")
            self.get_logger().error("Please start Gazebo first:")
            self.get_logger().error("  ros2 launch go2_gazebo_simulation go2_fortress.launch.py")
            self.get_logger().error("="*70)
            raise RuntimeError("Gazebo not running - required topics not available")

        self.get_logger().info("✓ All topics available - ready to log data")
    
    def joint_state_callback(self, msg: JointState):
        """Process joint states and compute accelerations."""
        joint_names = self.policy_runner.get_joint_names()
        
        for i, name in enumerate(msg.name):
            if name in joint_names:
                idx = joint_names.index(name)
                self.joint_positions[idx] = msg.position[i]
                new_vel = msg.velocity[i]
                
                # Compute acceleration
                dt = 0.005  # 200 Hz physics
                self.joint_accelerations[idx] = (new_vel - self.prev_joint_velocities[idx]) / dt
                self.joint_velocities[idx] = new_vel
                self.prev_joint_velocities[idx] = new_vel
                
                # Effort (torque)
                if i < len(msg.effort):
                    self.actual_torques[idx] = msg.effort[i]
        
        self.joint_state_received = True
    
    def odom_callback(self, msg: Odometry):
        """Process odometry."""
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
        
        # Transform velocity to body frame
        v_world = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ])
        rot = Rotation.from_quat(self.base_quat)
        self.base_lin_vel = rot.inv().apply(v_world)
        
        self.base_ang_vel = np.array([
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z
        ])
        
        self.odom_received = True
    
    def imu_callback(self, msg: Imu):
        """Process IMU (backup for angular velocity)."""
        pass
    
    def compute_projected_gravity(self) -> np.ndarray:
        """Compute gravity in body frame."""
        gravity_world = np.array([0.0, 0.0, -1.0])
        rot = Rotation.from_quat(self.base_quat)
        return rot.inv().apply(gravity_world)
    
    def compute_euler_angles(self) -> tuple:
        """Compute roll, pitch, yaw from quaternion."""
        rot = Rotation.from_quat(self.base_quat)
        euler = rot.as_euler('xyz')  # Roll, pitch, yaw
        return euler[0], euler[1], euler[2]
    
    def control_loop(self):
        """Main control loop at 50 Hz."""
        if not self.joint_state_received or not self.odom_received:
            # Log warning every 5 seconds
            if not hasattr(self, '_last_topic_warning') or time.time() - self._last_topic_warning > 5.0:
                self._last_topic_warning = time.time()
                self.get_logger().warn(
                    f"Waiting for topics: joint_states={self.joint_state_received}, "
                    f"odom={self.odom_received}"
                )
            return

        if not self.logging_active:
            return
        
        self.control_loop_start_time = time.time()
        
        # Get time-varying command
        time_in_episode = self.episode_step * 0.02
        cmd_vel = TimeVaryingCommand.get_command(self.current_task_id, time_in_episode)
        
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
            self.target_positions = self.policy_runner.get_action(
                self.joint_positions.copy(),
                self.joint_velocities.copy(),
                self.base_ang_vel.copy(),
                projected_gravity
            )
        except Exception as e:
            self.get_logger().error(f"Policy error: {e}")
            return
        
        # Compute torques
        if self.use_actuator_network and self.actuator_network is not None:
            self.commanded_torques = self.actuator_network.compute_torques(
                self.target_positions,
                self.joint_positions.copy(),
                self.joint_velocities.copy()
            )
        else:
            pos_error = self.target_positions - self.joint_positions
            self.commanded_torques = self.kp_gains * pos_error - self.kd_gains * self.joint_velocities
        
        self.commanded_torques = np.clip(self.commanded_torques, -40.0, 40.0)
        
        # Update action history
        self.prev_action_2 = self.prev_action_1.copy()
        self.prev_action_1 = self.last_action.copy()
        self.last_action = self.policy_runner.last_action.copy()  # Raw policy output
        
        # Log data
        if self.logging_active:
            self.log_timestep(cmd_vel, projected_gravity)
            self.episode_step += 1
        
        # Check episode completion
        if self.episode_step >= 1000:  # 20 seconds at 50 Hz
            self.stop_episode()
    
    def log_timestep(self, cmd_vel, projected_gravity):
        """Log one timestep with all 152 columns."""
        if self.csv_writer is None:
            return
        
        # Timing
        wall_time = time.time()
        sim_time = self.episode_step * 0.02
        control_latency = (wall_time - self.control_loop_start_time) * 1000  # ms
        rtf = 0.02 / (wall_time - self.episode_start_time_wall + 1e-9) if self.episode_step > 0 else 1.0
        
        # Euler angles
        roll, pitch, yaw = self.compute_euler_angles()
        
        # Derived metrics
        instantaneous_power = np.sum(np.abs(self.commanded_torques * self.joint_velocities))
        torque_saturation_count = np.sum(np.abs(self.commanded_torques) > 39.0)
        action_smoothness = np.linalg.norm(self.last_action - self.prev_action_1)
        gravity_alignment = np.dot(projected_gravity, np.array([0, 0, -1]))

        # Build 152-column row
        row = [
            # Metadata (5)
            sim_time,
            wall_time - self.episode_start_time_wall,
            self.current_episode,
            self.episode_seed,
            self.episode_step,
            
            # Performance (3)
            rtf,
            control_latency,
            0.02,  # actual_control_dt (nominal)
            
            # Base position (7)
            *self.base_pos,
            *self.base_quat,
            
            # Base orientation (4)
            roll, pitch, yaw,
            self.base_pos[2],  # height
            
            # Base velocity (6)
            *self.base_lin_vel,
            *self.base_ang_vel,
            
            # Commanded velocities (3)
            *cmd_vel,
            
            # Joint positions (12)
            *self.joint_positions,
            
            # Joint velocities (12)
            *self.joint_velocities,
            
            # Joint accelerations (12)
            *self.joint_accelerations,
            
            # Policy output (12)
            *self.last_action,
            
            # Commanded joint positions (12)
            *self.target_positions,
            
            # Commanded torques (12)
            *self.commanded_torques,
            
            # Actual torques (12)
            *self.actual_torques,
            
            # Action history t-1 (12)
            *self.prev_action_1,
            
            # Action history t-2 (12)
            *self.prev_action_2,
            
            # Contact (placeholder - 12 values)
            0.0, 0.0, 0.0, 0.0,  # binary contact
            0.0, 0.0, 0.0, 0.0,  # forces
            0.0, 0.0, 0.0, 0.0,  # slip velocities
            
            # Derived metrics (4)
            instantaneous_power,
            torque_saturation_count,
            action_smoothness,
            gravity_alignment,
        ]
        
        # Verify 152 columns
        assert len(row) == 152, f"Expected 152 columns, got {len(row)}"
        
        self.csv_writer.writerow(row)
    
    def start_episode(self, episode_id: int, task_id: int, seed: int):
        """Start new episode."""
        self.current_episode = episode_id
        self.current_task_id = task_id
        self.episode_seed = seed
        self.episode_step = 0
        self.episode_start_time_sim = 0.0
        self.episode_start_time_wall = time.time()
        
        # Reset action history
        self.last_action = np.zeros(12)
        self.prev_action_1 = np.zeros(12)
        self.prev_action_2 = np.zeros(12)
        
        # Open CSV
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        task_names = ['standing', 'walking', 'turning', 'combined']
        filename = f"locomotion_log_gazebo_ep{episode_id:03d}_{task_names[task_id]}_{timestamp}.csv"
        filepath = self.output_dir / filename
        
        self.csv_file = open(filepath, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        # Write 152-column header
        header = self._get_csv_header()
        self.csv_writer.writerow(header)
        
        self.logging_active = True
        self.get_logger().info(f"Started EP{episode_id} Task{task_id} Seed{seed}: {filename}")
    
    def stop_episode(self):
        """Stop current episode."""
        self.logging_active = False
        
        if self.csv_file is not None:
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None
        
        self.get_logger().info(f"Completed EP{self.current_episode} (steps: {self.episode_step})")
    
    def _get_csv_header(self):
        """Generate 152-column header."""
        header = [
            # Metadata (5)
            'timestamp_sim', 'timestamp_wall', 'episode_id', 'episode_seed', 'control_cycle',
            
            # Performance (3)
            'rtf', 'control_latency_ms', 'actual_control_dt',
            
            # Base position (7)
            'base_pos_x', 'base_pos_y', 'base_pos_z',
            'base_quat_x', 'base_quat_y', 'base_quat_z', 'base_quat_w',
            
            # Base orientation (4)
            'base_roll', 'base_pitch', 'base_yaw', 'base_height',
            
            # Base velocity (6)
            'base_lin_vel_x', 'base_lin_vel_y', 'base_lin_vel_z',
            'base_ang_vel_x', 'base_ang_vel_y', 'base_ang_vel_z',
            
            # Commanded velocities (3)
            'cmd_vx', 'cmd_vy', 'cmd_wz',
            
            # Joint positions (12)
            *[f'joint_pos_{i}' for i in range(12)],
            
            # Joint velocities (12)
            *[f'joint_vel_{i}' for i in range(12)],
            
            # Joint accelerations (12)
            *[f'joint_acc_{i}' for i in range(12)],
            
            # Policy output (12)
            *[f'action_{i}' for i in range(12)],
            
            # Commanded positions (12)
            *[f'cmd_joint_pos_{i}' for i in range(12)],
            
            # Commanded torques (12)
            *[f'cmd_torque_{i}' for i in range(12)],
            
            # Actual torques (12)
            *[f'actual_torque_{i}' for i in range(12)],
            
            # Action history t-1 (12)
            *[f'prev_action_1_{i}' for i in range(12)],
            
            # Action history t-2 (12)
            *[f'prev_action_2_{i}' for i in range(12)],
            
            # Contact (12)
            'foot_contact_binary_FL', 'foot_contact_binary_FR', 
            'foot_contact_binary_RL', 'foot_contact_binary_RR',
            'foot_contact_force_FL', 'foot_contact_force_FR',
            'foot_contact_force_RL', 'foot_contact_force_RR',
            'foot_slip_velocity_FL', 'foot_slip_velocity_FR',
            'foot_slip_velocity_RL', 'foot_slip_velocity_RR',
            
            # Derived metrics (4)
            'instantaneous_power', 'torque_saturation_count',
            'action_smoothness', 'gravity_alignment',
        ]
        
        assert len(header) == 152, f"Expected 152 columns, got {len(header)}"
        return header


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True)
    parser.add_argument('--policy', required=True)
    parser.add_argument('--policy_config', required=True, 
                        choices=['mlp_custom', 'mlp_no_dr', 'lstm_dr', 'lstm_no_dr', 'implicit_dr', 'implicit'])
    parser.add_argument('--output', required=True)
    parser.add_argument('--num_repeats', type=int, default=50)
    
    cli_args = parser.parse_args()
    
    rclpy.init(args=args)
    
    try:
        logger = EnhancedGazeboDataLogger(
            config_path=cli_args.config,
            policy_path=cli_args.policy,
            output_dir=cli_args.output,
            policy_config=cli_args.policy_config
        )
        
        # Run 200 episodes (50 repeats × 4 tasks)
        episode_id = 0
        for repeat in range(cli_args.num_repeats):
            for task_id in range(4):
                seed = repeat * 4 + task_id
                
                logger.get_logger().info(f"\n{'='*70}")
                logger.get_logger().info(f"Episode {episode_id}: Repeat {repeat}, Task {task_id}, Seed {seed}")
                logger.get_logger().info('='*70)
                
                logger.start_episode(episode_id, task_id, seed)
                
                # Run for 20 seconds (1000 steps at 50 Hz)
                start_time = time.time()
                while time.time() - start_time < 20.0 and logger.logging_active:
                    rclpy.spin_once(logger, timeout_sec=0.01)
                
                # Ensure episode is stopped
                if logger.logging_active:
                    logger.stop_episode()
                
                episode_id += 1
        
        logger.get_logger().info("\n" + "="*70)
        logger.get_logger().info(f"All 200 episodes completed for {cli_args.policy_config}!")
        logger.get_logger().info("="*70)
        
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

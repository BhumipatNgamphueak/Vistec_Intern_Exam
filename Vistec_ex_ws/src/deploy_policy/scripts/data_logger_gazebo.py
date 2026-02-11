#!/usr/bin/env python3
"""
Gazebo Ignition Locomotion Data Logger - Sim-to-Sim Transfer Experiment

Logs identical telemetry as IsaacLab for sim2sim comparison.
Deploys IsaacLab-trained policy in Gazebo WITHOUT retraining.

Matches exact parameters from Isaac Lab training:
- Physics dt: 0.005s (200 Hz)
- Control freq: 50 Hz (decimation=4)
- PD gains: Kp=25.0, Kd=0.5
- Action scale: 0.25
- Observation scales: ang_vel=0.2, joint_vel=0.05
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import numpy as np
import torch
import csv
import time
import yaml
import argparse
from pathlib import Path
from datetime import datetime
from scipy.spatial.transform import Rotation

# ROS 2 messages
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from rosgraph_msgs.msg import Clock

# Gazebo services
from gz.msgs10.contacts_pb2 import Contacts
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

# Import policy runner from deployment package
from deploy_policy.go2_policy_runner import Go2PolicyRunner
from deploy_policy.actuator_network import ActuatorNetworkRunner


class GazeboDataLogger(Node):
    """
    ROS 2 node for logging Go2 locomotion data in Gazebo.
    Matches Isaac Lab telemetry format for sim2sim comparison.
    """
    
    # Gazebo joint names in SDK order (matches ROS /joint_states)
    GAZEBO_JOINT_NAMES = [
        "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
        "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
        "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
        "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    ]
    
    # Foot link names for contact detection
    FOOT_NAMES = ["FR_foot", "FL_foot", "RR_foot", "RL_foot"]
    
    def __init__(self, config_path: str, policy_path: str, output_dir: str):
        super().__init__("gazebo_data_logger")
        
        # Load configuration
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)['gazebo_logger']
        
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # Initialize policy runner (from Isaac Lab training)
        self.get_logger().info(f"Loading policy from: {policy_path}")
        self.policy_runner = Go2PolicyRunner(
            policy_path=policy_path,
            device=self.config.get('device', 'cpu'),
            history_length=1,  # No history for MLP
            action_scale=0.25,  # From deploy.yaml
        )
        
        # Initialize actuator network if configured
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
                self.get_logger().info("✓ MLP Actuator Network loaded")
            except Exception as e:
                self.get_logger().error(f"Failed to load actuator network: {e}")
                self.get_logger().warn("Falling back to PD control")
                self.use_actuator_network = False
        
        # PD gains from Isaac Lab training (deploy.yaml)
        self.kp_gains = np.full(12, 25.0)  # Stiffness
        self.kd_gains = np.full(12, 0.5)   # Damping
        
        # State variables
        self.joint_positions = np.zeros(12)
        self.joint_velocities = np.zeros(12)
        self.base_pos = np.zeros(3)
        self.base_quat = np.array([0, 0, 0, 1])  # (x, y, z, w)
        self.base_lin_vel = np.zeros(3)  # In world frame
        self.base_ang_vel = np.zeros(3)  # In body frame
        self.imu_ang_vel = np.zeros(3)
        self.foot_forces = np.zeros(4)  # [FR, FL, RR, RL]
        
        # Velocity command (from episode config)
        self.cmd_vel = np.zeros(3)  # [vx, vy, wz]
        
        # Data received flags
        self.joint_state_received = False
        self.odom_received = False
        self.imu_received = False
        
        # Logging state
        self.logging_active = False
        self.current_episode = 0
        self.episode_step = 0
        self.csv_writer = None
        self.csv_file = None
        self.episode_start_time = 0.0
        
        # QoS profiles
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, qos_sensor
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/model/go2_robot/odometry', self.odom_callback, qos_sensor
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, qos_sensor
        )
        
        # Publishers for robot control
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel', qos_reliable
        )
        
        # Joint effort publishers (individual topics for Gazebo)
        self.joint_effort_pubs = {}
        for joint_name in self.GAZEBO_JOINT_NAMES:
            topic = f"/model/go2_robot/joint/{joint_name}/cmd_force"
            self.joint_effort_pubs[joint_name] = self.create_publisher(
                Float64MultiArray, topic, qos_reliable
            )
        
        # Control timer (50 Hz to match Isaac Lab)
        self.control_period = 0.02  # 50 Hz
        self.control_timer = self.create_timer(
            self.control_period, self.control_loop
        )
        
        self.get_logger().info("="*70)
        self.get_logger().info("Gazebo Data Logger Initialized")
        self.get_logger().info(f"Policy: {policy_path}")
        self.get_logger().info(f"Control frequency: 50 Hz")
        self.get_logger().info(f"Output directory: {self.output_dir}")
        self.get_logger().info("="*70)
    
    def joint_state_callback(self, msg: JointState):
        """Process joint states from Gazebo."""
        # Reorder joints to match canonical order
        for i, name in enumerate(msg.name):
            if name in self.GAZEBO_JOINT_NAMES:
                idx = self.GAZEBO_JOINT_NAMES.index(name)
                self.joint_positions[idx] = msg.position[i]
                self.joint_velocities[idx] = msg.velocity[i]
        
        self.joint_state_received = True
    
    def odom_callback(self, msg: Odometry):
        """Process odometry from Gazebo."""
        # Base position
        self.base_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        
        # Base orientation (quaternion)
        self.base_quat = np.array([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])
        
        # Linear velocity (in world frame)
        v_world = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ])
        
        # Transform to body frame
        rot = Rotation.from_quat(self.base_quat)  # (x,y,z,w)
        self.base_lin_vel = rot.inv().apply(v_world)
        
        # Angular velocity (already in body frame)
        self.base_ang_vel = np.array([
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z
        ])
        
        self.odom_received = True
    
    def imu_callback(self, msg: Imu):
        """Process IMU data from Gazebo."""
        # Angular velocity from IMU (body frame)
        self.imu_ang_vel = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        
        self.imu_received = True
    
    def compute_projected_gravity(self) -> np.ndarray:
        """Compute gravity vector in robot body frame."""
        gravity_world = np.array([0.0, 0.0, -1.0])
        rot = Rotation.from_quat(self.base_quat)
        gravity_body = rot.inv().apply(gravity_world)
        return gravity_body
    
    def control_loop(self):
        """Main control loop at 50 Hz."""
        if not self.joint_state_received or not self.odom_received:
            return
        
        # Get observations
        projected_gravity = self.compute_projected_gravity()
        
        # Get action from policy
        try:
            target_positions = self.policy_runner.get_action(
                self.joint_positions.copy(),
                self.joint_velocities.copy(),
                self.imu_ang_vel.copy(),
                projected_gravity
            )
        except Exception as e:
            self.get_logger().error(f"Policy inference error: {e}")
            return
        
        # Compute torques
        if self.use_actuator_network and self.actuator_network is not None:
            efforts = self.actuator_network.compute_torques(
                target_positions,
                self.joint_positions.copy(),
                self.joint_velocities.copy()
            )
        else:
            # PD control
            position_error = target_positions - self.joint_positions
            efforts = self.kp_gains * position_error - self.kd_gains * self.joint_velocities
        
        # Clip efforts to limits
        efforts = np.clip(efforts, -40.0, 40.0)
        
        # Publish efforts
        for i, joint_name in enumerate(self.GAZEBO_JOINT_NAMES):
            msg = Float64MultiArray()
            msg.data = [float(efforts[i])]
            self.joint_effort_pubs[joint_name].publish(msg)
        
        # Log data if active
        if self.logging_active:
            self.log_timestep(target_positions, efforts, projected_gravity)
            self.episode_step += 1
    
    def log_timestep(self, target_pos, efforts, projected_gravity):
        """Log one timestep of data (74 columns matching Isaac Lab)."""
        if self.csv_writer is None:
            return
        
        # Compute relative joint positions
        default_pos_policy = self.policy_runner.default_joint_pos_policy
        joint_pos_policy = self.policy_runner.sdk_to_policy_order(self.joint_positions)
        joint_pos_rel = joint_pos_policy - default_pos_policy
        
        # Build data row (74 columns)
        row = [
            # Timestamp (1)
            time.time() - self.episode_start_time,
            
            # Base state (13): pos(3) + quat(4) + lin_vel(3) + ang_vel(3)
            *self.base_pos,
            *self.base_quat,
            *self.base_lin_vel,
            *self.base_ang_vel,
            
            # Joint positions (12)
            *self.joint_positions,
            
            # Joint velocities (12)
            *self.joint_velocities,
            
            # Joint target positions (12)
            *target_pos,
            
            # Joint efforts/torques (12)
            *efforts,
            
            # Foot contact forces (4): [FR, FL, RR, RL]
            *self.foot_forces,
            
            # Velocity command (3): [vx, vy, wz]
            *self.cmd_vel,
            
            # Projected gravity (3)
            *projected_gravity,
            
            # Episode info (2): episode_id, step
            self.current_episode,
            self.episode_step,
        ]
        
        self.csv_writer.writerow(row)
    
    def start_episode(self, episode_id: int, cmd_vel: np.ndarray):
        """Start logging a new episode."""
        self.current_episode = episode_id
        self.episode_step = 0
        self.cmd_vel = cmd_vel
        self.episode_start_time = time.time()
        
        # Set velocity command in policy
        self.policy_runner.set_velocity_command(cmd_vel[0], cmd_vel[1], cmd_vel[2])
        
        # Publish velocity command
        twist = Twist()
        twist.linear.x = float(cmd_vel[0])
        twist.linear.y = float(cmd_vel[1])
        twist.angular.z = float(cmd_vel[2])
        self.cmd_vel_pub.publish(twist)
        
        # Open CSV file
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"locomotion_log_gazebo_ep{episode_id:03d}_{timestamp}.csv"
        filepath = self.output_dir / filename
        
        self.csv_file = open(filepath, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        # Write header (74 columns matching Isaac Lab)
        header = [
            'timestamp',
            # Base state (13)
            'base_pos_x', 'base_pos_y', 'base_pos_z',
            'base_quat_x', 'base_quat_y', 'base_quat_z', 'base_quat_w',
            'base_lin_vel_x', 'base_lin_vel_y', 'base_lin_vel_z',
            'base_ang_vel_x', 'base_ang_vel_y', 'base_ang_vel_z',
            # Joint positions (12)
            *[f'joint_pos_{i}' for i in range(12)],
            # Joint velocities (12)
            *[f'joint_vel_{i}' for i in range(12)],
            # Joint targets (12)
            *[f'joint_target_{i}' for i in range(12)],
            # Joint efforts (12)
            *[f'joint_effort_{i}' for i in range(12)],
            # Foot forces (4)
            'foot_force_FR', 'foot_force_FL', 'foot_force_RR', 'foot_force_RL',
            # Command (3)
            'cmd_vel_x', 'cmd_vel_y', 'cmd_vel_z',
            # Projected gravity (3)
            'proj_gravity_x', 'proj_gravity_y', 'proj_gravity_z',
            # Episode info (2)
            'episode_id', 'step'
        ]
        
        self.csv_writer.writerow(header)
        
        self.logging_active = True
        self.get_logger().info(f"Started episode {episode_id}: {filename}")
    
    def stop_episode(self):
        """Stop logging current episode."""
        self.logging_active = False
        
        if self.csv_file is not None:
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None
        
        self.get_logger().info(f"Stopped episode {self.current_episode} (steps: {self.episode_step})")


def main(args=None):
    parser = argparse.ArgumentParser(description='Gazebo Data Logger for Sim2Sim Transfer')
    parser.add_argument('--config', required=True, help='Config YAML file')
    parser.add_argument('--policy', required=True, help='Policy checkpoint (.pt file)')
    parser.add_argument('--output', required=True, help='Output directory for logs')
    parser.add_argument('--episodes', required=True, help='Episode configurations YAML')
    parser.add_argument('--start_episode', type=int, default=0, help='Start from episode N')
    
    cli_args = parser.parse_args()
    
    rclpy.init(args=args)
    
    try:
        # Create logger node
        logger = GazeboDataLogger(
            config_path=cli_args.config,
            policy_path=cli_args.policy,
            output_dir=cli_args.output
        )
        
        # Load episode configurations
        with open(cli_args.episodes, 'r') as f:
            episodes = yaml.safe_load(f)['episodes']
        
        logger.get_logger().info(f"Loaded {len(episodes)} episodes")
        logger.get_logger().info(f"Starting from episode {cli_args.start_episode}")
        
        # Run episodes
        for ep_id, episode in enumerate(episodes[cli_args.start_episode:], start=cli_args.start_episode):
            cmd_vel = np.array(episode['velocity_command'])
            duration = episode['duration']
            
            logger.get_logger().info(f"\n{'='*70}")
            logger.get_logger().info(f"Episode {ep_id}: cmd=[{cmd_vel[0]:.2f}, {cmd_vel[1]:.2f}, {cmd_vel[2]:.2f}], duration={duration}s")
            logger.get_logger().info('='*70)
            
            # Start episode
            logger.start_episode(ep_id, cmd_vel)
            
            # Run for specified duration
            start_time = time.time()
            while time.time() - start_time < duration:
                rclpy.spin_once(logger, timeout_sec=0.01)
            
            # Stop episode
            logger.stop_episode()
        
        logger.get_logger().info("\n" + "="*70)
        logger.get_logger().info("All episodes completed!")
        logger.get_logger().info("="*70)
        
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

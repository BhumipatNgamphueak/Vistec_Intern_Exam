#!/usr/bin/env python3
"""
Passive Data Collector - Collects data without running policy
Matches the full 76-column format of data_logger_gazebo_single.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import numpy as np
import time
import csv
import sys
from scipy.spatial.transform import Rotation

class PassiveDataCollector(Node):
    def __init__(self, output_file, model_name="lstm_dr", task_id=3, episode_id=0):
        super().__init__('passive_data_collector')

        self.output_file = output_file
        self.model_name = model_name
        self.task_id = task_id
        self.episode_id = episode_id
        self.policy_path = f"/home/drl-68/unitree_rl_lab/logs/rsl_rl/unitree_go2_velocity_{model_name}/model.pt"

        # QoS profile for Gazebo topics (best effort)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers with proper QoS
        self.odom_sub = self.create_subscription(Odometry, '/odom_fixed', self.odom_callback, qos_profile)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, qos_profile)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

        # Try to subscribe to action topic if available
        self.action_sub = self.create_subscription(
            Float64MultiArray, '/policy_actions', self.action_callback, 10
        )

        # Data storage
        self.data_rows = []
        self.start_time = time.time()
        self.wall_start_time = time.time()
        self.max_duration = 20.5  # Slightly longer to ensure we get all data
        self.control_dt = 0.02  # 50 Hz

        # State
        self.base_pos = np.zeros(3)
        self.base_quat = np.array([1.0, 0.0, 0.0, 0.0])  # w,x,y,z
        self.base_lin_vel = np.zeros(3)
        self.base_ang_vel = np.zeros(3)
        self.joint_pos = np.zeros(12)
        self.joint_vel = np.zeros(12)
        self.joint_acc = np.zeros(12)
        self.prev_joint_vel = np.zeros(12)
        self.cmd_vel = np.zeros(3)
        self.actions = np.zeros(12)
        self.prev_actions = np.zeros(12)

        # Counters
        self.control_cycle = 0

        # Timer for collection at 50 Hz
        self.timer = self.create_timer(self.control_dt, self.collect_data)

        self.get_logger().info(f'Passive data collector started')
        self.get_logger().info(f'Output: {output_file}')
        self.get_logger().info(f'Duration: {self.max_duration}s')

    def odom_callback(self, msg):
        self.base_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        self.base_quat = np.array([
            msg.pose.pose.orientation.w,
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

    def joint_callback(self, msg):
        if len(msg.position) == 12:
            self.joint_pos = np.array(msg.position)
            new_vel = np.array(msg.velocity) if len(msg.velocity) == 12 else np.zeros(12)

            # Compute acceleration
            self.joint_acc = (new_vel - self.prev_joint_vel) / self.control_dt
            self.prev_joint_vel = new_vel.copy()
            self.joint_vel = new_vel

    def cmd_callback(self, msg):
        self.cmd_vel = np.array([msg.linear.x, msg.linear.y, msg.angular.z])

    def action_callback(self, msg):
        if len(msg.data) == 12:
            self.actions = np.array(msg.data)

    def get_euler_angles(self):
        """Convert quaternion (w,x,y,z) to euler angles (roll, pitch, yaw)"""
        quat_scipy = np.array([
            self.base_quat[1],  # x
            self.base_quat[2],  # y
            self.base_quat[3],  # z
            self.base_quat[0]   # w
        ])
        rot = Rotation.from_quat(quat_scipy)
        euler = rot.as_euler('xyz')
        return euler[0], euler[1], euler[2]

    def compute_gravity_alignment(self):
        """Compute gravity alignment (projected gravity z-component)"""
        quat_scipy = np.array([
            self.base_quat[1], self.base_quat[2],
            self.base_quat[3], self.base_quat[0]
        ])
        rot = Rotation.from_quat(quat_scipy)
        gravity_world = np.array([0.0, 0.0, -1.0])
        projected_gravity = rot.inv().apply(gravity_world)
        return projected_gravity[2]

    def collect_data(self):
        elapsed = time.time() - self.start_time

        if elapsed > self.max_duration:
            self.get_logger().info('Collection time limit reached, saving...')
            self.save_and_exit()
            return

        # Compute derived values
        roll, pitch, yaw = self.get_euler_angles()
        base_height = self.base_pos[2]
        gravity_alignment = self.compute_gravity_alignment()

        # Compute action smoothness
        action_smoothness = np.linalg.norm(self.actions - self.prev_actions) if self.control_cycle > 0 else 0.0
        self.prev_actions = self.actions.copy()

        # Compute instantaneous power (simplified)
        instantaneous_power = np.sum(np.abs(self.joint_vel * self.actions))

        # Torque saturation count (simplified - assume no saturation)
        torque_saturation_count = 0

        # Build row (76 columns to match original format)
        row = {
            'timestamp_sim': elapsed,
            'timestamp_wall': time.time() - self.wall_start_time,
            'episode_id': self.episode_id,
            'episode_seed': 0,
            'control_cycle': self.control_cycle,
            'policy_path': self.policy_path,
            'policy_config': f'velocity_{self.model_name}',
            'rtf': 1.0,  # Real-time factor
            'control_latency_ms': 0.0,
            'actual_control_dt': self.control_dt,
            'base_pos_x': self.base_pos[0],
            'base_pos_y': self.base_pos[1],
            'base_pos_z': self.base_pos[2],
            'base_quat_w': self.base_quat[0],
            'base_quat_x': self.base_quat[1],
            'base_quat_y': self.base_quat[2],
            'base_quat_z': self.base_quat[3],
            'base_roll': roll,
            'base_pitch': pitch,
            'base_yaw': yaw,
            'base_height': base_height,
            'base_lin_vel_x': self.base_lin_vel[0],
            'base_lin_vel_y': self.base_lin_vel[1],
            'base_lin_vel_z': self.base_lin_vel[2],
            'base_ang_vel_x': self.base_ang_vel[0],
            'base_ang_vel_y': self.base_ang_vel[1],
            'base_ang_vel_z': self.base_ang_vel[2],
            'cmd_vx': self.cmd_vel[0],
            'cmd_vy': self.cmd_vel[1],
            'cmd_wz': self.cmd_vel[2],
        }

        # Joint positions (12)
        for i in range(12):
            row[f'joint_pos_{i}'] = self.joint_pos[i]

        # Joint velocities (12)
        for i in range(12):
            row[f'joint_vel_{i}'] = self.joint_vel[i]

        # Joint accelerations (12)
        for i in range(12):
            row[f'joint_acc_{i}'] = self.joint_acc[i]

        # Actions (12)
        for i in range(12):
            row[f'action_{i}'] = self.actions[i]

        # Metrics (4)
        row['instantaneous_power'] = instantaneous_power
        row['torque_saturation_count'] = torque_saturation_count
        row['action_smoothness'] = action_smoothness
        row['gravity_alignment'] = gravity_alignment

        self.data_rows.append(row)
        self.control_cycle += 1

        if self.control_cycle % 100 == 0:
            self.get_logger().info(
                f'Collected {self.control_cycle} samples ({elapsed:.1f}s / {self.max_duration}s)'
            )

    def save_and_exit(self):
        self.get_logger().info(f'Saving {len(self.data_rows)} rows to {self.output_file}')

        if not self.data_rows:
            self.get_logger().error('No data collected!')
            rclpy.shutdown()
            return

        try:
            with open(self.output_file, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=self.data_rows[0].keys())
                writer.writeheader()
                writer.writerows(self.data_rows)

            self.get_logger().info(f'✅ Data saved successfully! ({len(self.data_rows)} rows)')
        except Exception as e:
            self.get_logger().error(f'Failed to save data: {e}')

        rclpy.shutdown()

def main():
    if len(sys.argv) < 2:
        print("Usage: passive_data_collector.py <output_file> [model_name] [task_id] [episode_id]")
        sys.exit(1)

    output_file = sys.argv[1]
    model_name = sys.argv[2] if len(sys.argv) > 2 else "lstm_dr"
    task_id = int(sys.argv[3]) if len(sys.argv) > 3 else 3
    episode_id = int(sys.argv[4]) if len(sys.argv) > 4 else 0

    rclpy.init()
    collector = PassiveDataCollector(output_file, model_name, task_id, episode_id)

    try:
        rclpy.spin(collector)
    except KeyboardInterrupt:
        collector.save_and_exit()
    except Exception as e:
        print(f"Error: {e}")
        collector.save_and_exit()

if __name__ == '__main__':
    main()
